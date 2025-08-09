#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from sensor_msgs.msg import JointState
import time
import math
import numpy as np
import cv2  # AGREGADO: Faltaba este import
import json
import os

# Constantes para el control de empuje (basadas en el script de referencia)
THETA_EXT = 0.27  # Ángulo mínimo del shoulder (radianes) - ~15.5°
THETA_RET = np.pi/4  # Ángulo máximo del shoulder (radianes) - 45°

def cart2pol(x, y):
    """Convierte coordenadas cartesianas a polares"""
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return rho, phi

def get_other_angles(theta_shoulder):
    """Calcula wrist y elbow basados en shoulder para mantener orientación vertical"""
    theta_wrist = theta_shoulder + np.pi/2
    theta_elbow = np.pi/2 - 2*theta_shoulder
    return theta_wrist, theta_elbow

class VisionPickAndPlace(Node):
    def __init__(self):
        super().__init__('vision_pick_and_place')
        
        # Publishers
        self.arm_publisher = self.create_publisher(
            JointTrajectory, 
            '/position_trajectory_controller/joint_trajectory', 
            10
        )
        self.gripper_publisher = self.create_publisher(
            JointTrajectory, 
            '/gripper_controller/joint_trajectory', 
            10
        )
        
        # Subscriber
        self.object_subscriber = self.create_subscription(
            PointStamped,
            '/detected_object_coords',
            self.object_detected_callback,
            10
        )
        
        # MoveIt client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.moveit_available = self.ik_client.wait_for_service(timeout_sec=5.0)
        
        if self.moveit_available:
            self.get_logger().info('MoveIt disponible')
        else:
            self.get_logger().warn('MoveIt NO disponible - usando fallback')
        
        # Configuración simple
        self.positions = {
            'home': [0.0, 1.57, 0.0, 0.0, 0.0],
            'drop': [1.57, 1.2, 1.0, 0.8, 0.0]
        }
        self.gripper = {'open': 0.0, 'closed': 0.8, 'middle': 0.8}
        self.timing = {'movement': 4.0, 'grip': 2.0}
        
        # Parámetros del brazo para IK analítica (calibrados del script de referencia)
        self.L = 0.125  # Longitud del brazo principal (metros)
        self.l = 0.095  # Offset desde la base (metros)
        
        # Cargar homografía
        self.load_homography()
        
        # Estado
        self.object_detected = False
        self.target_x = 0.0
        self.target_y = 0.0
        self.pick_active = False
        
        # Estado de la secuencia pick and place
        self.sequence_state = 'idle'  # idle, home, open_gripper, push, wait_settle, approach, descend, grab, lift, drop, release, return_home
        self.state_start_time = None
        self.needs_push = False  # Flag para indicar si se necesita empuje
        self.push_step = 0  # Para seguir el paso actual de la secuencia de empuje
        self.waiting_for_object_settle = False  # Flag para esperar que el objeto se asiente
        
        self.timer = self.create_timer(0.1, self.state_machine_update)  # 10Hz para mejor responsividad
        self.get_logger().info('Pick and Place iniciado')
    
    def get_targets_analytical(self, x, y):
        """Calcula IK analítica como el script de referencia"""
        try:
            rho, phi = cart2pol(x, y)
            theta_shoulder = np.arccos((rho - self.l) / self.L)
            theta_wrist, theta_elbow = get_other_angles(theta_shoulder)
            return [phi, theta_shoulder, theta_elbow, theta_wrist, 1.5708]  # gripper fijo
        except:
            return None
    
    def should_push_object(self, x, y):
        """Determina si el objeto necesita ser empujado basado en IK analítica"""
        targets = self.get_targets_analytical(x, y)
        if targets is None:
            return True  # Si no se puede calcular IK, empujar
        
        theta_shoulder = targets[1]
        
        # Fuera del dominio válido
        if theta_shoulder < THETA_EXT:
            self.get_logger().warn(f'Objeto fuera del dominio: theta={theta_shoulder:.3f} < {THETA_EXT:.3f}')
            return False  # No se puede alcanzar, ni empujando
        
        # Muy cerca, necesita empuje
        if theta_shoulder > THETA_RET:
            self.get_logger().warn(f'Objeto muy cerca, empujando: theta={theta_shoulder:.3f} > {THETA_RET:.3f}')
            return True
        
        # En rango válido
        return False
    
    def go_to_raise(self):
        """Elevar el brazo a posición segura"""
        raise_position = [0.0, 1.15, 0.13, 2.29, 1.57]
        self.move_to_position(raise_position)
    
    def go_to_raise(self):
        """Elevar el brazo a posición segura"""
        # Elevar el brazo (similar al movimiento home pero manteniendo orientación actual)
        goal_msg = JointTrajectory()
        goal_msg.joint_names = self.joint_names

        # Posición elevada segura
        target_joints = [
            0.0,      # base (mantener posición actual)
            0.4,      # shoulder (elevar)
            0.8,      # elbow (elevar)
            0.0,      # wrist_pitch
            self.gripper['middle']  # gripper en posición media
        ]

        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start = Duration(seconds=self.timing['movement']).to_msg()
        goal_msg.points = [point]

        self.publisher_.publish(goal_msg)
        self.get_logger().info('⬆️ Elevando brazo a posición segura')

    def execute_push_sequence_step(self, base_angle, current_step):
        """Ejecuta un paso de la secuencia de empuje"""
        # Secuencia de empuje del script original
        push_positions = [
            [base_angle, 1.15, 0.13, 2.29, 1.57],  # 0. Raise + rotate to angle
            [base_angle, 2.7, 0.01, 0.01, 1.57],   # 1. Bajar completamente
            [base_angle, 2.1, 0.01, 0.01, 1.57],   # 2. Subir un poco
            [base_angle, 0.5, 1.8, 0.1, 1.57],     # 3. Empujar hacia atrás
            [base_angle, 2.1, 0.01, 0.01, 1.57],   # 4. Volver a bajar
            [base_angle, 2.7, 0.01, 0.01, 1.57],   # 5. Posición final baja
        ]
        
        if current_step < len(push_positions):
            self.get_logger().info(f'🔄 Paso empuje {current_step+1}/{len(push_positions)}')
            self.move_to_position(push_positions[current_step])
            return current_step + 1
        else:
            self.get_logger().info('✅ Secuencia de empuje completada')
            return -1  # Secuencia completada
        
    def load_homography(self):
        """Cargar matriz de homografía"""
        try:
            with open("/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_vision/config/camera_calibration.json", 'r') as f:
                data = json.load(f)
            self.homography_matrix = np.array(data['homography_matrix'])
            self.get_logger().info('Homografía cargada exitosamente')
        except Exception as e:
            self.homography_matrix = None
            self.get_logger().warn(f'Sin homografía ({e}) - usando mapeo simple')
    
    def object_detected_callback(self, msg):
        """Callback para recibir posiciones de objetos detectados"""
        # Aplicar transformación homográfica
        if self.homography_matrix is not None:
            pixel_point = np.array([[msg.x, msg.y]], dtype=np.float32)
            world_point = cv2.perspectiveTransform(pixel_point.reshape(-1, 1, 2), self.homography_matrix)
            new_x, new_y = world_point[0, 0, 0], world_point[0, 0, 1]
        else:
            new_x, new_y = msg.x, msg.y

        if self.sequence_state == 'idle':
            self.target_x, self.target_y = new_x, new_y
            self.object_detected = True
            self.get_logger().info(f'Objeto transformado a: ({self.target_x:.3f}, {self.target_y:.3f})')
        elif self.sequence_state == 'wait_settle' and self.waiting_for_object_settle:
            # Durante espera post-empuje, actualizar posición del objeto
            self.target_x, self.target_y = new_x, new_y
            self.get_logger().info(f'Nueva posición tras empuje: ({self.target_x:.3f}, {self.target_y:.3f})')
            self.waiting_for_object_settle = False  # Marcar que ya detectamos el objeto
    
    def transform_pixels_to_robot(self, pixel_x, pixel_y):
        """Transformar píxeles a coordenadas del robot"""
        if self.homography_matrix is not None:
            try:
                # Usar homografía
                pixel_point = np.array([[pixel_x, pixel_y]], dtype=np.float32).reshape(-1, 1, 2)
                robot_point = cv2.perspectiveTransform(pixel_point, self.homography_matrix)
                x, y = float(robot_point[0][0][0]), float(robot_point[0][0][1])
                self.get_logger().info(f'Homografía: píxel ({pixel_x},{pixel_y}) -> robot ({x:.3f},{y:.3f})')
                return x, y
            except Exception as e:
                self.get_logger().error(f'Error en homografía: {e}')
        
        # Mapeo simple mejorado (calibración básica para Braccio)
        # Asumiendo imagen 640x480 y workspace del Braccio
        x = (pixel_x - 320) * 0.0005  # Escalado mejorado
        y = (240 - pixel_y) * 0.0005 + 0.15  # Invertir Y y ajustar offset
        
        self.get_logger().info(f'Mapeo simple: píxel ({pixel_x},{pixel_y}) -> robot ({x:.3f},{y:.3f})')
        return x, y
    
    def get_targets_analytical(self, x, y):
        """Calcula IK analítica como el script de referencia"""
        try:
            rho, phi = cart2pol(x, y)
            theta_shoulder = np.arccos((rho - self.l) / self.L)
            theta_wrist, theta_elbow = get_other_angles(theta_shoulder)
            return [phi, theta_shoulder, theta_elbow, theta_wrist, 1.5708]  # gripper fijo
        except:
            return None
    
    def should_push_object(self, x, y):
        """Determina si el objeto necesita ser empujado basado en IK analítica"""
        targets = self.get_targets_analytical(x, y)
        if targets is None:
            return True  # Si no se puede calcular IK, empujar
        
        theta_shoulder = targets[1]
        
        # Fuera del dominio válido
        if theta_shoulder < THETA_EXT:
            self.get_logger().warn(f'Objeto fuera del dominio: theta={theta_shoulder:.3f} < {THETA_EXT:.3f}')
            return False  # No se puede alcanzar, ni empujando
        
        # Muy cerca, necesita empuje
        if theta_shoulder > THETA_RET:
            self.get_logger().warn(f'Objeto muy cerca, empujando: theta={theta_shoulder:.3f} > {THETA_RET:.3f}')
            return True
        
        # En rango válido
        return False
    
    def go_to_raise(self):
        """Elevar el brazo a posición segura"""
        raise_position = [0.0, 1.15, 0.13, 2.29, 1.57]
        self.move_to_position(raise_position)
    
    def execute_push_sequence(self, base_angle):
        """Ejecuta la secuencia de empuje como el script de referencia"""
        self.get_logger().info(f'🔄 Ejecutando secuencia de empuje con ángulo base: {math.degrees(base_angle):.1f}°')
        
        # 1. Elevar y gripper medio
        self.go_to_raise()
        self.control_gripper(self.gripper['middle'])
        
        # 2. Rotar base hacia el objeto
        if base_angle != 0:
            base_only = [base_angle, 1.15, 0.13, 2.29, 1.57]
            self.move_to_position(base_only)
        
        # 3. Secuencia de empuje (valores del script original)
        push_positions = [
            [base_angle, 2.7, 0.01, 0.01, 1.57],  # Bajar completamente
            [base_angle, 2.1, 0.01, 0.01, 1.57],  # Subir un poco
            [base_angle, 0.5, 1.8, 0.1, 1.57],    # Empujar hacia atrás
            [base_angle, 2.1, 0.01, 0.01, 1.57],  # Volver a bajar
            [base_angle, 2.7, 0.01, 0.01, 1.57],  # Posición final baja
        ]
        
        for i, position in enumerate(push_positions):
            self.get_logger().info(f'Paso empuje {i+1}/5')
            self.move_to_position(position)
            time.sleep(1)  # Pausa entre movimientos
        
        self.get_logger().info('✅ Secuencia de empuje completada')
    
    def state_machine_update(self):
        """Máquina de estados para pick and place sin bloqueos"""
        current_time = time.time()
        
        # Iniciar secuencia si hay objeto detectado
        if self.object_detected and self.sequence_state == 'idle':
            self.get_logger().info('Objeto detectado - iniciando pick and place')
            self.sequence_state = 'home'
            self.state_start_time = current_time
            self.object_detected = False
            return
        
        # Ejecutar estado actual
        if self.sequence_state == 'idle':
            return
            
        if self.sequence_state == 'home':
            if self.state_start_time == current_time:  # Primera ejecución
                self.get_logger().info('1. Yendo a home')
                self.move_to_position(self.positions['home'])
            elif current_time - self.state_start_time >= self.timing['movement']:
                self.get_logger().info('Estado HOME completado, pasando a abrir gripper')
                self.sequence_state = 'open_gripper'
                self.state_start_time = current_time
                
        elif self.sequence_state == 'open_gripper':
            if self.state_start_time == current_time:  # Primera ejecución
                self.get_logger().info('2. Abriendo gripper')
                self.control_gripper(self.gripper['open'])
            elif current_time - self.state_start_time >= self.timing['grip']:
                # Verificar si necesita empuje ANTES de approach
                self.needs_push = self.should_push_object(self.target_x, self.target_y)
                
                if self.needs_push:
                    self.get_logger().info('🔄 Objeto necesita empuje - iniciando secuencia')
                    self.sequence_state = 'push'
                    self.push_step = 0  # Reiniciar contador de pasos
                else:
                    self.get_logger().info('✅ Objeto en rango válido, pasando a approach')
                    self.sequence_state = 'approach'
                self.state_start_time = current_time
                
        elif self.sequence_state == 'push':
            if self.state_start_time == current_time:  # Primera ejecución del paso actual
                # Calcular ángulo base para el empuje
                base_angle = math.atan2(self.target_y, self.target_x)
                self.get_logger().info(f'🔄 Iniciando empuje con ángulo base: {math.degrees(base_angle):.1f}°')
                
                # Ejecutar primer paso: gripper a posición media y elevar
                self.control_gripper(self.gripper['middle'])
                self.push_step = self.execute_push_sequence_step(base_angle, self.push_step)
                
            elif current_time - self.state_start_time >= self.timing['movement']:
                if self.push_step >= 0:  # Continuar con siguiente paso
                    base_angle = math.atan2(self.target_y, self.target_x)
                    self.push_step = self.execute_push_sequence_step(base_angle, self.push_step)
                    self.state_start_time = current_time  # Reiniciar timer para siguiente paso
                else:  # Secuencia de empuje completada
                    self.get_logger().info('🔄 Empuje completado, esperando que objeto se asiente...')
                    self.sequence_state = 'wait_settle'
                    self.state_start_time = current_time
                    self.waiting_for_object_settle = True
                    
        elif self.sequence_state == 'wait_settle':
            # Esperar a que el objeto se asiente y sea detectado en nueva posición
            if current_time - self.state_start_time >= 3.0:  # Esperar 3 segundos
                if not self.waiting_for_object_settle:
                    # Objeto ya fue detectado en nueva posición, proceder con approach
                    self.get_logger().info('✅ Objeto detectado en nueva posición, procediendo con pick')
                    self.sequence_state = 'approach'
                    self.state_start_time = current_time
                else:
                    # Timeout - usar última posición conocida
                    self.get_logger().warn('⚠️ Timeout esperando nueva detección, usando última posición')
                    self.waiting_for_object_settle = False
                    self.sequence_state = 'approach'
                    self.state_start_time = current_time
                
        elif self.sequence_state == 'approach':
            if self.state_start_time == current_time:  # Primera ejecución
                self.get_logger().info(f'3. Calculando IK para target: ({self.target_x:.3f}, {self.target_y:.3f})')
                self.pick_high = self.calculate_ik_position(self.target_x, self.target_y, 0.08)
                self.pick_low = self.calculate_ik_position(self.target_x, self.target_y, 0.01)
                
                self.get_logger().info(f'Pick high: {[f"{math.degrees(j):.1f}°" for j in self.pick_high]}')
                self.get_logger().info(f'Pick low: {[f"{math.degrees(j):.1f}°" for j in self.pick_low]}')
                self.get_logger().info('4. Aproximándose al objeto')
                self.move_to_position(self.pick_high)
            elif current_time - self.state_start_time >= self.timing['movement']:
                self.sequence_state = 'descend'
                self.state_start_time = current_time
                
        elif self.sequence_state == 'descend':
            if self.state_start_time == current_time:  # Primera ejecución
                self.get_logger().info('5. Descendiendo')
                self.move_to_position(self.pick_low)
            elif current_time - self.state_start_time >= self.timing['movement']:
                self.sequence_state = 'grab'
                self.state_start_time = current_time
                
        elif self.sequence_state == 'grab':
            if self.state_start_time == current_time:  # Primera ejecución
                self.get_logger().info('6. Agarrando objeto')
                self.control_gripper(self.gripper['closed'])
            elif current_time - self.state_start_time >= self.timing['grip']:
                self.sequence_state = 'lift'
                self.state_start_time = current_time
                
        elif self.sequence_state == 'lift':
            if self.state_start_time == current_time:  # Primera ejecución
                self.get_logger().info('7. Levantando')
                self.move_to_position(self.pick_high)
            elif current_time - self.state_start_time >= self.timing['movement']:
                self.sequence_state = 'drop'
                self.state_start_time = current_time
                
        elif self.sequence_state == 'drop':
            if self.state_start_time == current_time:  # Primera ejecución
                self.get_logger().info('8. Yendo a drop')
                self.move_to_position(self.positions['drop'])
            elif current_time - self.state_start_time >= self.timing['movement']:
                self.sequence_state = 'release'
                self.state_start_time = current_time
                
        elif self.sequence_state == 'release':
            if self.state_start_time == current_time:  # Primera ejecución
                self.get_logger().info('9. Soltando objeto')
                self.control_gripper(self.gripper['open'])
            elif current_time - self.state_start_time >= self.timing['grip']:
                self.sequence_state = 'return_home'
                self.state_start_time = current_time
                
        elif self.sequence_state == 'return_home':
            if self.state_start_time == current_time:  # Primera ejecución
                self.get_logger().info('10. Volviendo a home')
                self.move_to_position(self.positions['home'])
            elif current_time - self.state_start_time >= self.timing['movement']:
                self.get_logger().info('¡Pick and place completado!')
                self.sequence_state = 'idle'
                self.state_start_time = None
    
    def calculate_ik_position(self, x, y, z):
        """Calcular posición usando MoveIt IK"""
        if not self.moveit_available:
            self.get_logger().info('Usando fallback (sin MoveIt)')
            base_angle = math.atan2(y, x)
            # Clamp al rango válido
            base_angle = max(-1.57, min(1.57, base_angle))
            return [base_angle, 1.2, 0.8, 0.7, 1.57]
        
        try:
            self.get_logger().info(f'Calculando IK para pose: ({x:.3f}, {y:.3f}, {z:.3f})')
            
            # Crear request para MoveIt
            ik_request = PositionIKRequest()
            ik_request.group_name = "arm"  # Grupo correcto según diagnóstico
            
            # Configurar pose objetivo
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            # Orientación: gripper apuntando hacia abajo
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.707  # Rotación 90° en Y para apuntar hacia abajo
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.707
            
            ik_request.pose_stamped = pose
            ik_request.timeout.sec = 5
            
            # Llamar al servicio
            request = GetPositionIK.Request(ik_request=ik_request)
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() and future.result().error_code.val == 1:
                self.get_logger().info('IK exitoso')
                # Éxito - extraer ángulos de joints
                joint_state = future.result().solution.joint_state
                positions = []
                joint_names = ['joint_base', 'joint_1', 'joint_2', 'joint_3', 'joint_4']
                
                for name in joint_names:
                    if name in joint_state.name:
                        idx = joint_state.name.index(name)
                        positions.append(joint_state.position[idx])
                    else:
                        self.get_logger().warn(f'Joint {name} no encontrado')
                        positions.append(0.0)
                
                return positions
            else:
                error_code = future.result().error_code.val if future.result() else "Sin respuesta"
                self.get_logger().warn(f'IK falló (código: {error_code}) - usando fallback')
                base_angle = math.atan2(y, x)
                base_angle = max(-1.57, min(1.57, base_angle))
                return [base_angle, 1.2, 0.8, 0.7, 1.57]
                
        except Exception as e:
            self.get_logger().error(f'Error en IK: {e}')
            base_angle = math.atan2(y, x)
            base_angle = max(-1.57, min(1.57, base_angle))
            return [base_angle, 1.2, 0.8, 0.7, 1.57]
    
    def move_to_position(self, joint_positions):
        """Mover el brazo a una posición"""
        self.get_logger().info(f'Enviando comando: {[f"{math.degrees(j):.1f}°" for j in joint_positions]}')
        
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = [
            'joint_base', 'joint_1', 'joint_2', 
            'joint_3', 'joint_4'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=int(self.timing['movement']))
        trajectory.points = [point]
        
        self.arm_publisher.publish(trajectory)
        self.get_logger().info('Comando publicado')
    
    def control_gripper(self, position):
        """Controlar gripper"""
        self.get_logger().info(f'Gripper a posición: {position}')
        
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = ['right_gripper_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(self.timing['grip']))
        trajectory.points = [point]
        
        self.gripper_publisher.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = VisionPickAndPlace()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()