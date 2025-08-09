#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import os
import math
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class ConfigurablePickAndPlace(Node):
    def __init__(self):
        super().__init__('configurable_pick_and_place')
        
        # Cargar configuración
        self.load_config()
        
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
        
        self.get_logger().info('Nodo Configurable Pick and Place iniciado')
        self.get_logger().info(f'Configuración cargada: {len(self.config.get("sequences", {}))} secuencias disponibles')

    def load_config(self):
        """Cargar configuración desde archivo YAML"""
        try:
            config_path = '/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_moveit_config/config/pick_and_place_config.yaml'
            
            with open(config_path, 'r') as file:
                self.config = yaml.safe_load(file)
            
            self.get_logger().info(f'Configuración cargada desde: {config_path}')
            
            # Debug: mostrar las posiciones de pick que se van a usar
            if 'joint_positions' in self.config:
                pick_approach = self.config['joint_positions'].get('pick_approach', 'No definido')
                pick_position = self.config['joint_positions'].get('pick_position', 'No definido')
                self.get_logger().info(f'🎯 Pick approach: {pick_approach}')
                self.get_logger().info(f'🎯 Pick position: {pick_position}')
            
        except Exception as e:
            self.get_logger().error(f'Error cargando configuración: {str(e)}')
            # Configuración por defecto
            self.config = self.get_default_config()

    def get_default_config(self):
        """Configuración por defecto si no se puede cargar el archivo"""
        return {
            'joint_positions': {
                'home': [0.0, 1.57, 0.0, 0.0, 0.0],
                'pick_approach': [0.0, 1.2, 1.0, 0.8, 0.0],
                'pick_position': [0.0, 1.0, 1.4, 1.2, 0.0],
                'place_approach': [1.57, 1.2, 1.0, 0.8, 0.0],
                'place_position': [1.57, 1.0, 1.4, 1.2, 0.0]
            },
            'gripper': {
                'open_position': 0.0,
                'closed_position': 0.8,
                'open_time': 1.0,
                'close_time': 2.0
            },
            'movement': {
                'default_duration': 3.0
            }
        }

    def create_arm_trajectory(self, positions, duration_sec=None):
        """Crea una trayectoria para el brazo"""
        if duration_sec is None:
            duration_sec = self.config['movement']['default_duration']
        
        traj = JointTrajectory()
        traj.joint_names = [
            "joint_base",
            "joint_1",
            "joint_2", 
            "joint_3",
            "joint_4"
        ]
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        traj.points.append(point)
        
        return traj

    def create_gripper_trajectory(self, position, duration_sec):
        """Crea una trayectoria para el gripper"""
        traj = JointTrajectory()
        traj.joint_names = ["right_gripper_joint"]
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        traj.points.append(point)
        
        return traj

    def move_joints(self, position_name, duration=None):
        """Mueve el brazo a una posición nombrada"""
        if position_name not in self.config['joint_positions']:
            self.get_logger().error(f'Posición {position_name} no encontrada en configuración')
            return False
        
        positions = self.config['joint_positions'][position_name]
        if duration is None:
            duration = self.config['movement']['default_duration']
        
        self.get_logger().info(f'Moviendo a posición: {position_name}')
        
        trajectory = self.create_arm_trajectory(positions, duration)
        self.arm_publisher.publish(trajectory)
        
        time.sleep(duration + 0.5)
        return True

    def control_gripper(self, open_gripper=True):
        """Controla el gripper"""
        gripper_config = self.config['gripper']
        
        if open_gripper:
            position = gripper_config['open_position']  # 0.0
            duration = gripper_config['open_time']      # 2.0
            action = "Abriendo"
        else:
            position = gripper_config['closed_position'] # 0.8
            duration = gripper_config['close_time']      # 3.0
            action = "Cerrando"
        
        self.get_logger().info(f'{action} gripper a posición: {position}')
        
        trajectory = self.create_gripper_trajectory(position, duration)
        self.gripper_publisher.publish(trajectory)
        
        time.sleep(duration + 0.5)
        return True

    def execute_action(self, action):
        """Ejecuta una acción individual"""
        action_type = action['action']
        description = action.get('description', '')
        
        self.get_logger().info(f'Ejecutando: {description}')
        
        if action_type == 'move_joints':
            target = action['target']
            duration = action.get('duration')
            return self.move_joints(target, duration)
        
        elif action_type == 'open_gripper':
            return self.control_gripper(open_gripper=True)
        
        elif action_type == 'close_gripper':
            return self.control_gripper(open_gripper=False)
        
        elif action_type == 'wait':
            wait_time = action.get('time', 1.0)
            time.sleep(wait_time)
            return True
        
        else:
            self.get_logger().error(f'Acción desconocida: {action_type}')
            return False

    def execute_sequence(self, sequence_name):
        """Ejecuta una secuencia completa"""
        if sequence_name not in self.config.get('sequences', {}):
            self.get_logger().error(f'Secuencia {sequence_name} no encontrada')
            return False
        
        sequence = self.config['sequences'][sequence_name]
        
        self.get_logger().info(f'=== EJECUTANDO SECUENCIA: {sequence_name.upper()} ===')
        
        for i, action in enumerate(sequence):
            self.get_logger().info(f'Paso {i+1}/{len(sequence)}')
            
            if not self.execute_action(action):
                self.get_logger().error(f'Error en paso {i+1}, abortando secuencia')
                return False
            
            time.sleep(0.5)  # Pequeña pausa entre acciones
        
        self.get_logger().info(f'=== SECUENCIA {sequence_name.upper()} COMPLETADA ===')
        return True

    def list_available_sequences(self):
        """Lista las secuencias disponibles"""
        sequences = self.config.get('sequences', {})
        self.get_logger().info('Secuencias disponibles:')
        for name in sequences.keys():
            self.get_logger().info(f'  - {name}')
        return list(sequences.keys())

    def list_available_positions(self):
        """Lista las posiciones disponibles"""
        positions = self.config.get('joint_positions', {})
        self.get_logger().info('Posiciones disponibles:')
        for name, pos in positions.items():
            self.get_logger().info(f'  - {name}: {pos}')
        return list(positions.keys())

    def calculate_push_positions(self, object_x, object_y, target_x, target_y, base_angle):
        """Calcular posiciones de empuje usando cinemática inversa simple"""
        try:
            # Calcular distancias para cinemática inversa
            object_distance = math.sqrt(object_x**2 + object_y**2)
            target_distance = math.sqrt(target_x**2 + target_y**2)
            
            # Ángulos base (rotación hacia objeto y objetivo)
            object_base_angle = math.atan2(object_y, object_x)
            target_base_angle = math.atan2(target_y, target_x)
            
            # Cinemática inversa simple para Braccio (asumiendo eslabones de ~20cm cada uno)
            # Altura del objeto (mesa a ~0cm del suelo del robot)
            object_height = 0.02  # 2cm altura del cubo
            
            # Posiciones de empuje calculadas
            push_positions = [
                # 1. Posición inicial segura
                [0.0, 1.57, 0.0, 0.0, 1.57],  # Home position
                
                # 2. Rotar hacia el objeto y elevar
                [object_base_angle, 1.0, 0.5, 1.5, 1.57],  # Aproximación alta
                
                # 3. Bajar hacia el objeto (cinemática inversa simple)
                [object_base_angle, 1.8, 1.2, 0.8, 1.57],  # Cerca del objeto
                
                # 4. Posición de contacto con el objeto
                [object_base_angle, 2.0, 1.4, 0.6, 1.57],  # Contacto
                
                # 5. Empujar hacia la posición objetivo
                [target_base_angle, 2.0, 1.4, 0.6, 1.57],  # Empuje
                
                # 6. Levantar ligeramente después del empuje
                [target_base_angle, 1.8, 1.2, 0.8, 1.57],  # Alejar
                
                # 7. Volver a posición segura
                [0.0, 1.57, 0.0, 0.0, 1.57],  # Home
            ]
            
            self.get_logger().info(f'📐 Ángulo objeto: {math.degrees(object_base_angle):.1f}°')
            self.get_logger().info(f'📐 Ángulo objetivo: {math.degrees(target_base_angle):.1f}°')
            
            return push_positions
            
        except Exception as e:
            self.get_logger().error(f'❌ Error calculando posiciones de empuje: {e}')
            return None

    def execute_push_sequence(self, base_angle=0.0, object_x=0.0, object_y=0.0):
        """Ejecutar secuencia de empuje inteligente basada en posición del objeto"""
        self.get_logger().info('🔄 === INICIANDO SECUENCIA DE EMPUJE INTELIGENTE ===')
        self.get_logger().info(f'🎯 Objeto en: ({object_x:.3f}, {object_y:.3f})')
        
        # Calcular la distancia del objeto al centro
        object_distance = math.sqrt(object_x**2 + object_y**2)
        self.get_logger().info(f'📏 Distancia del objeto: {object_distance:.3f}m')
        
        # Determinar dirección del empuje (empujar hacia el centro, zona cómoda)
        if object_distance > 0.01:  # Evitar división por cero
            # Vector normalizado desde objeto hacia centro (zona cómoda)
            push_direction_x = -object_x / object_distance
            push_direction_y = -object_y / object_distance
            
            # Magnitud del empuje (mover ~5cm hacia el centro)
            push_magnitude = 0.05
            target_x = object_x + push_direction_x * push_magnitude
            target_y = object_y + push_direction_y * push_magnitude
            
            self.get_logger().info(f'👉 Empujando hacia: ({target_x:.3f}, {target_y:.3f})')
        else:
            # Si está muy cerca del centro, empujar ligeramente hacia adelante
            target_x = object_x + 0.03
            target_y = object_y
            self.get_logger().info('👉 Empujando hacia adelante (objeto centrado)')
        
        # Calcular posiciones de empuje usando cinemática inversa
        push_positions = self.calculate_push_positions(object_x, object_y, target_x, target_y, base_angle)
        
        if push_positions is None:
            self.get_logger().error('❌ No se pudieron calcular las posiciones de empuje')
            return False
        
        step_descriptions = [
            "Posición inicial segura",
            "Rotando y aproximándose al objeto",
            "Acercándose al objeto", 
            "Contacto con objeto",
            "Empujando hacia zona cómoda",
            "Alejándose tras empuje",
            "Retirando a posición segura"
        ]
        
        for i, (position, description) in enumerate(zip(push_positions, step_descriptions)):
            self.get_logger().info(f'🔄 Paso {i+1}/{len(push_positions)}: {description}')
            
            # Enviar comando de movimiento
            if not self.move_to_joint_position_direct(position):
                self.get_logger().error(f'❌ Error enviando comando paso {i+1}')
                return False
            
            # Esperar que se complete el movimiento
            time.sleep(3.5)
        
        self.get_logger().info('✅ Secuencia de empuje inteligente completada')
        return True

    def move_to_joint_position_direct(self, joint_positions):
        """Mover a posición de articulaciones directamente (para empuje)"""
        try:
            # Usar el mismo método que en modo normal para garantizar consistencia
            duration = 3.0  # Usar la misma duración que en modo normal
            
            trajectory = self.create_arm_trajectory(joint_positions, duration)
            self.arm_publisher.publish(trajectory)
            
            self.get_logger().info(f'📤 Comando empuje enviado: {[f"{math.degrees(j):.1f}°" for j in joint_positions]}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Error moviendo a posición: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ConfigurablePickAndPlace()
        
        # Esperar conexiones
        node.get_logger().info('Esperando conexiones...')
        time.sleep(3.0)
        
        # Mostrar opciones disponibles
        node.list_available_sequences()
        node.list_available_positions()
        
        # Verificar si necesita empuje (usando archivo flag)
        push_flag_file = "/tmp/braccio_needs_push.flag"
        needs_push = os.path.exists(push_flag_file)
        
        if needs_push:
            node.get_logger().info('🔄 MODO EMPUJE + PICK detectado')
            
            # Leer ángulo base del archivo si existe
            base_angle = 0.0
            angle_file = "/tmp/braccio_push_angle.txt"
            if os.path.exists(angle_file):
                try:
                    with open(angle_file, 'r') as f:
                        base_angle = float(f.read().strip())
                    node.get_logger().info(f'📐 Ángulo base leído: {math.degrees(base_angle):.1f}°')
                except:
                    node.get_logger().warn('⚠️  Error leyendo ángulo, usando 0°')
            
            # Leer coordenadas del objeto si existen
            object_x, object_y = 0.0, 0.0
            coords_file = "/tmp/braccio_object_coords.txt"
            if os.path.exists(coords_file):
                try:
                    with open(coords_file, 'r') as f:
                        coords = f.read().strip().split(',')
                        object_x = float(coords[0])
                        object_y = float(coords[1])
                    node.get_logger().info(f'📍 Coordenadas objeto leídas: ({object_x:.3f}, {object_y:.3f})')
                except:
                    node.get_logger().warn('⚠️  Error leyendo coordenadas, usando (0,0)')
            
            # Ejecutar secuencia de empuje inteligente
            if node.execute_push_sequence(base_angle, object_x, object_y):
                node.get_logger().info('✅ Empuje completado, esperando asentamiento...')
                time.sleep(3.0)
                
                # Limpiar archivos de empuje
                try:
                    os.remove(push_flag_file)
                    if os.path.exists(angle_file):
                        os.remove(angle_file)
                    if os.path.exists(coords_file):
                        os.remove(coords_file)
                except:
                    pass
            else:
                node.get_logger().error('❌ Error en secuencia de empuje')
                return
        
        # Ejecutar secuencia normal de pick and place
        sequence_to_run = 'basic_demo'  # Cambiar aquí para ejecutar otra secuencia
        
        if sequence_to_run in node.config.get('sequences', {}):
            success = node.execute_sequence(sequence_to_run)
            
            if success:
                node.get_logger().info('Secuencia ejecutada exitosamente')
            else:
                node.get_logger().error('Error durante la ejecución')
        else:
            node.get_logger().error(f'Secuencia {sequence_to_run} no encontrada')
            
    except KeyboardInterrupt:
        node.get_logger().info('Interrumpido por el usuario')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
