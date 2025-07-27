#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import yaml
import os
import time
from ament_index_python.packages import get_package_share_directory

class VisionPickAndPlace(Node):
    def __init__(self):
        super().__init__('vision_pick_and_place')
        
        # Publishers para control del robot
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
        
        # Subscriber para coordenadas detectadas
        self.object_subscriber = self.create_subscription(
            PointStamped,
            '/detected_object_coords',
            self.object_detected_callback,
            10
        )
        
        # Cargar configuraciones
        self.load_configurations()
        
        # Estado del sistema
        self.object_detected = False
        self.target_x = 0.0
        self.target_y = 0.0
        self.pick_and_place_active = False
        
        # Timer para operaciones automatizadas
        self.timer = self.create_timer(1.0, self.check_for_objects)
        
        self.get_logger().info('Sistema de Pick and Place con Visión iniciado')
        self.get_logger().info('Esperando detección de objetos...')
        
    def load_configurations(self):
        """Cargar configuraciones desde archivos YAML"""
        try:
            # Cargar configuración de visión
            vision_config_path = os.path.join(
                get_package_share_directory('braccio_vision'),
                'config', 'vision_config.yaml'
            )
            with open(vision_config_path, 'r') as file:
                self.vision_config = yaml.safe_load(file)
            
            # Cargar configuración de pick and place
            try:
                pick_config_path = os.path.join(
                    get_package_share_directory('braccio_moveit_config'),
                    'config', 'pick_and_place_config.yaml'
                )
                with open(pick_config_path, 'r') as file:
                    self.pick_config = yaml.safe_load(file)
            except:
                # Si no existe, usar configuración por defecto
                self.pick_config = self.get_default_pick_config()
            
            # Convertir estructura del archivo YAML a formato esperado por el código
            self.normalize_pick_config()
                
            self.get_logger().info('Configuraciones cargadas exitosamente')
            
        except Exception as e:
            self.get_logger().error(f'Error cargando configuraciones: {e}')
            self.pick_config = self.get_default_pick_config()
    
    def normalize_pick_config(self):
        """Normalizar configuración para compatibilidad con código existente"""
        if 'joint_positions' in self.pick_config:
            # Convertir nueva estructura a formato esperado
            if 'positions' not in self.pick_config:
                self.pick_config['positions'] = {}
            
            # Mapear posiciones de joints (mantener solo 5 joints del brazo)
            self.pick_config['positions']['home'] = self.pick_config['joint_positions']['home']
            
            # Crear posición de drop (usar place_position si existe, sino usar home)
            if 'place_position' in self.pick_config['joint_positions']:
                self.pick_config['positions']['drop_position'] = self.pick_config['joint_positions']['place_position']
            else:
                self.pick_config['positions']['drop_position'] = self.pick_config['joint_positions']['home']
            
            # Convertir gripper config
            if 'gripper' in self.pick_config:
                if 'open_position' in self.pick_config['gripper']:
                    self.pick_config['gripper']['open'] = self.pick_config['gripper']['open_position']
                if 'closed_position' in self.pick_config['gripper']:
                    self.pick_config['gripper']['closed'] = self.pick_config['gripper']['closed_position']
            
            # Convertir timing config
            if 'movement' in self.pick_config:
                self.pick_config['timing'] = {
                    'movement_duration': self.pick_config['movement']['default_duration'],
                    'grip_duration': self.pick_config['gripper'].get('close_time', 3.0),
                    'wait_between_actions': 0.5
                }
            
    def get_default_pick_config(self):
        """Configuración por defecto si no existe archivo"""
        return {
            'positions': {
                'home': [0.0, 1.57, 0.0, 0.0, 0.0],
                'pre_pick': [0.0, 1.2, 1.0, 0.8, 0.0],
                'drop_position': [1.57, 1.2, 1.0, 0.8, 0.0]
            },
            'gripper': {
                'open': 0.0,
                'closed': 0.8
            },
            'timing': {
                'movement_duration': 3.0,
                'grip_duration': 2.0,
                'wait_between_actions': 0.5
            }
        }
    
    def object_detected_callback(self, msg):
        """Callback cuando se detecta un objeto"""
        if not self.pick_and_place_active:
            self.target_x = msg.point.x
            self.target_y = msg.point.y
            self.object_detected = True
            
            self.get_logger().info(f'Objeto detectado en: x={self.target_x:.3f}, y={self.target_y:.3f}')
    
    def check_for_objects(self):
        """Verificar si hay objetos detectados y ejecutar pick and place"""
        if self.object_detected and not self.pick_and_place_active:
            self.get_logger().info('Iniciando secuencia de pick and place...')
            self.execute_vision_pick_and_place()
    
    def execute_vision_pick_and_place(self):
        """Ejecutar secuencia completa de pick and place guiada por visión"""
        self.pick_and_place_active = True
        self.object_detected = False
        
        try:
            # 1. Ir a posición home
            self.get_logger().info('1. Moviendo a posición home...')
            self.move_to_position(self.pick_config['positions']['home'])
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 2. Abrir gripper
            self.get_logger().info('2. Abriendo gripper...')
            self.control_gripper(self.pick_config['gripper']['open'])
            time.sleep(self.pick_config['timing']['grip_duration'])
            
            # 3. Calcular posición de pick basada en coordenadas detectadas
            pick_position = self.calculate_pick_position(self.target_x, self.target_y)
            
            # 4. Ir a pre-pick (posición segura sobre el objeto)
            pre_pick_position = pick_position.copy()
            pre_pick_position[1] += 0.3  # Elevar 30cm sobre el objeto
            
            self.get_logger().info('3. Moviendo a posición pre-pick...')
            self.move_to_position(pre_pick_position)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 5. Descender a posición de pick
            self.get_logger().info('4. Descendiendo a objeto...')
            self.move_to_position(pick_position)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 6. Cerrar gripper (agarrar objeto)
            self.get_logger().info('5. Cerrando gripper - agarrando objeto...')
            self.control_gripper(self.pick_config['gripper']['closed'])
            time.sleep(self.pick_config['timing']['grip_duration'])
            
            # 7. Levantar objeto
            self.get_logger().info('6. Levantando objeto...')
            self.move_to_position(pre_pick_position)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 8. Ir a posición de drop
            self.get_logger().info('7. Moviendo a posición de drop...')
            self.move_to_position(self.pick_config['positions']['drop_position'])
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 9. Soltar objeto
            self.get_logger().info('8. Abriendo gripper - soltando objeto...')
            self.control_gripper(self.pick_config['gripper']['open'])
            time.sleep(self.pick_config['timing']['grip_duration'])
            
            # 10. Volver a home
            self.get_logger().info('9. Regresando a home...')
            self.move_to_position(self.pick_config['positions']['home'])
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            self.get_logger().info('¡Secuencia de pick and place completada!')
            
        except Exception as e:
            self.get_logger().error(f'Error en pick and place: {e}')
        
        finally:
            self.pick_and_place_active = False
            self.get_logger().info('Listo para detectar nuevo objeto...')
    
    def calculate_pick_position(self, x, y):
        """Calcular posición de articulaciones para coordenadas x,y del mundo"""
        # Configuración básica de transformación
        # Estas constantes deberían calibrarse según la configuración real
        
        # Posición base cerca del objeto
        base_rotation = 0.0  # Rotación base
        if x != 0:
            base_rotation = 1.57 * (x / abs(x)) if abs(x) > 0.1 else 0.0
        
        # Ajustar alcance según distancia
        distance = (x**2 + y**2)**0.5
        
        # Configuración de articulaciones para alcanzar posición
        shoulder = 0.5 + min(distance * 0.3, 0.5)  # Ajustar según alcance
        elbow = -0.8 - min(distance * 0.2, 0.4)    # Compensar con codo
        wrist = 0.3                                # Mantener orientación
        wrist_rot = 0.0                           # Sin rotación de muñeca
        
        position = [
            base_rotation,  # Base
            shoulder,       # Shoulder
            elbow,         # Elbow  
            wrist,         # Wrist vertical
            wrist_rot      # Wrist rotation
        ]
        
        self.get_logger().info(f'Posición calculada: {position}')
        return position
    
    def move_to_position(self, target_position):
        """Mover brazo a posición específica"""
        msg = JointTrajectory()
        msg.joint_names = [
            "joint_base",
            "joint_1",
            "joint_2", 
            "joint_3",
            "joint_4"
        ]
        
        # Solo usar los primeros 5 valores (sin gripper)
        arm_positions = target_position[:5] if len(target_position) > 5 else target_position
        
        point = JointTrajectoryPoint()
        point.positions = arm_positions
        point.time_from_start = Duration(
            sec=int(self.pick_config['timing']['movement_duration']),
            nanosec=int((self.pick_config['timing']['movement_duration'] % 1) * 1e9)
        )
        
        msg.points = [point]
        self.arm_publisher.publish(msg)
    
    def control_gripper(self, position):
        """Controlar gripper usando JointTrajectory"""
        msg = JointTrajectory()
        msg.joint_names = ["right_gripper_joint"]
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(
            sec=int(self.pick_config['timing']['grip_duration']),
            nanosec=int((self.pick_config['timing']['grip_duration'] % 1) * 1e9)
        )
        
        msg.points = [point]
        self.gripper_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    vision_pick_place = VisionPickAndPlace()
    
    try:
        rclpy.spin(vision_pick_place)
    except KeyboardInterrupt:
        vision_pick_place.get_logger().info('Sistema detenido por usuario')
    finally:
        vision_pick_place.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
