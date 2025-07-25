#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import os
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
        """Carga la configuración desde el archivo YAML"""
        try:
            pkg_share = get_package_share_directory('braccio_moveit_config')
            config_path = os.path.join(pkg_share, 'config', 'pick_and_place_config.yaml')
            
            if not os.path.exists(config_path):
                # Buscar en el directorio del workspace
                config_path = '/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_moveit_config/config/pick_and_place_config.yaml'
            
            with open(config_path, 'r') as file:
                self.config = yaml.safe_load(file)
            
            self.get_logger().info(f'Configuración cargada desde: {config_path}')
            
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
        
        # Ejecutar secuencia por defecto
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
