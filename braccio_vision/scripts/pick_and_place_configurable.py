#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from linkattacher_msgs.srv import AttachLink, DetachLink
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
        
        # Cliente para attach/detach
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')

        self.get_logger().info('Nodo Configurable Pick and Place iniciado')
        self.get_logger().info(f'Configuración cargada: {len(self.config.get("sequences", {}))} secuencias disponibles')

    def load_config(self):
        """Cargar configuración desde archivo YAML"""
        try:
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
                'closed_position': 0.7,
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
        
        # Hardware interface now handles the offset - send logical positions directly
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
        
        # Crear trayectoria normal
        traj = self.create_arm_trajectory(positions, duration)
        
        # Log del movimiento
        self.get_logger().info(f'🤖 Moviendo a: {position_name}')
        self.get_logger().info(f'📐 Posiciones: {positions}')
        
        self.arm_publisher.publish(traj)
        
        time.sleep(duration + 0.5)
        return True

    def call_attach(self, model2_name, link2_name="link"):
        req = AttachLink.Request()
        req.model1_name = "braccio"
        req.link1_name = "right_gripper_link"
        req.model2_name = model2_name
        req.link2_name = link2_name
        if not self.attach_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Servicio /ATTACHLINK no disponible')
            return False
        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() and hasattr(future.result(), 'success'):
            if future.result().success:
                self.get_logger().info(f'✅ ATTACHLINK exitoso para modelo: {model2_name}')
            else:
                self.get_logger().warn(f'⚠️ ATTACHLINK falló para modelo: {model2_name} - {future.result().message}')
        else:
            self.get_logger().info(f'📎 ATTACHLINK ejecutado para modelo: {model2_name}')
        return True

    def call_detach(self, model2_name, link2_name="link"):
        req = DetachLink.Request()
        req.model1_name = "braccio"
        req.link1_name = "right_gripper_link"
        req.model2_name = model2_name
        req.link2_name = link2_name
        if not self.detach_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Servicio /DETACHLINK no disponible')
            return False
        future = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() and hasattr(future.result(), 'success'):
            if future.result().success:
                self.get_logger().info(f'✅ DETACHLINK exitoso para modelo: {model2_name}')
            else:
                self.get_logger().warn(f'⚠️ DETACHLINK falló para modelo: {model2_name} - {future.result().message}')
        else:
            self.get_logger().info(f'🔗 DETACHLINK ejecutado para modelo: {model2_name}')
        return True

    def control_gripper(self, open_gripper=True, model2_name=None, link2_name="link"):
        """Controla el gripper y llama a attach/detach si corresponde"""
        # Si no se especifica model2_name, usar un valor por defecto
        if model2_name is None:
            model2_name = "green_cube"
            self.get_logger().warn(f'⚠️ No se especificó model2_name, usando por defecto: {model2_name}')
        
        gripper_config = self.config['gripper']
        if open_gripper:
            position = gripper_config['open_position']  # 0.0
            duration = gripper_config['open_time']      # 2.0
            action = "Abriendo"
            self.get_logger().info(f'{action} gripper a posición: {position}')
            trajectory = self.create_gripper_trajectory(position, duration)
            self.gripper_publisher.publish(trajectory)
            time.sleep(0.2)  # Espera 0.2 segundos tras iniciar apertura
            self.call_detach(model2_name, link2_name)
            return True
        else:
            position = gripper_config['closed_position'] # 0.8
            duration = gripper_config['close_time']      # 3.0
            action = "Cerrando"
            self.get_logger().info(f'{action} gripper a posición: {position}')
            trajectory = self.create_gripper_trajectory(position, duration)
            self.gripper_publisher.publish(trajectory)
            time.sleep(duration + 0.5)
            self.call_attach(model2_name, link2_name)
            return True

    def execute_action(self, action, target_model=None):
        """Ejecuta una acción individual"""
        action_type = action['action']
        description = action.get('description', '')
        
        self.get_logger().info(f'Ejecutando: {description} (objetivo: {target_model})')
        
        if action_type == 'move_joints':
            target = action['target']
            duration = action.get('duration')
            return self.move_joints(target, duration)
        
        elif action_type == 'open_gripper':
            return self.control_gripper(open_gripper=True, model2_name=target_model)
        
        elif action_type == 'close_gripper':
            return self.control_gripper(open_gripper=False, model2_name=target_model)
        
        elif action_type == 'wait':
            wait_time = action.get('time', 1.0)
            time.sleep(wait_time)
            return True
        
        else:
            self.get_logger().error(f'Acción desconocida: {action_type}')
            return False

    def execute_sequence(self, sequence_name, target_model=None):
        """Ejecuta una secuencia completa"""
        if sequence_name not in self.config.get('sequences', {}):
            self.get_logger().error(f'Secuencia {sequence_name} no encontrada')
            return False
        
        sequence = self.config['sequences'][sequence_name]
        
        self.get_logger().info(f'=== EJECUTANDO SECUENCIA: {sequence_name.upper()} PARA {target_model} ===')
        
        for i, action in enumerate(sequence):
            self.get_logger().info(f'Paso {i+1}/{len(sequence)}')
            
            if not self.execute_action(action, target_model):
                self.get_logger().error(f'Error en paso {i+1}, abortando secuencia')
                return False
            
            time.sleep(0.5)  # Pequeña pausa entre acciones
        
        self.get_logger().info(f'=== SECUENCIA {sequence_name.upper()} COMPLETADA PARA {target_model} ===')
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

    def execute_pick_and_place_for_target(self, target_model_name, sequence_name='basic_demo'):
        """Método público para ejecutar pick and place para un modelo específico"""
        self.get_logger().info(f'🎯 Iniciando pick and place para: {target_model_name}')
        
        # IMPORTANTE: Recargar configuración antes de ejecutar
        self.get_logger().info('🔄 Recargando configuración actualizada...')
        self.load_config()
        
        success = self.execute_sequence(sequence_name, target_model_name)
        
        if success:
            self.get_logger().info(f'✅ Pick and place completado exitosamente para {target_model_name}')
        else:
            self.get_logger().error(f'❌ Error durante pick and place para {target_model_name}')
        
        return success

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
