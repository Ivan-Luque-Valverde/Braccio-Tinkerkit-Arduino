#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time

class PositionTester(Node):
    def __init__(self):
        super().__init__('position_tester')
        
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
        
        # Subscriber para joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_states = None
        
        self.get_logger().info('🔧 Position Tester iniciado')
        self.get_logger().info('📋 Usa este nodo para probar y calibrar posiciones')

    def load_config(self):
        """Carga la configuración desde el archivo YAML"""
        try:
            pkg_share = get_package_share_directory('braccio_moveit_config')
            config_path = os.path.join(pkg_share, 'config', 'pick_and_place_config.yaml')
            
            if not os.path.exists(config_path):
                config_path = '/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_moveit_config/config/pick_and_place_config.yaml'
            
            with open(config_path, 'r') as file:
                self.config = yaml.safe_load(file)
            
            self.get_logger().info(f'✅ Configuración cargada desde: {config_path}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error cargando configuración: {str(e)}')
            self.config = {'joint_positions': {}, 'gripper': {}}

    def joint_state_callback(self, msg):
        """Callback para recibir estados actuales de joints"""
        self.current_joint_states = msg

    def create_arm_trajectory(self, positions, duration_sec=3.0):
        """Crea una trayectoria para el brazo"""
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

    def create_gripper_trajectory(self, position, duration_sec=1.0):
        """Crea una trayectoria para el gripper"""
        traj = JointTrajectory()
        traj.joint_names = ["right_gripper_joint"]
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        traj.points.append(point)
        
        return traj

    def get_current_positions(self):
        """Obtiene las posiciones actuales de los joints"""
        if self.current_joint_states is None:
            return None
        
        positions = {}
        for i, name in enumerate(self.current_joint_states.name):
            positions[name] = self.current_joint_states.position[i]
        
        return positions

    def print_current_positions(self):
        """Imprime las posiciones actuales en formato YAML"""
        positions = self.get_current_positions()
        if positions is None:
            self.get_logger().warning('⚠️  No hay datos de joint states disponibles')
            return
        
        # Extraer posiciones de los joints del brazo
        arm_joints = ["joint_base", "joint_1", "joint_2", "joint_3", "joint_4"]
        arm_positions = []
        
        for joint in arm_joints:
            if joint in positions:
                arm_positions.append(round(positions[joint], 3))
            else:
                arm_positions.append(0.0)
        
        # Gripper
        gripper_pos = round(positions.get("right_gripper_joint", 0.0), 3)
        
        self.get_logger().info('📍 POSICIONES ACTUALES:')
        self.get_logger().info(f'   Brazo: {arm_positions}')
        self.get_logger().info(f'   Gripper: {gripper_pos}')
        self.get_logger().info('📝 Para usar en YAML:')
        self.get_logger().info(f'   nueva_posicion: {arm_positions}')

    def test_position(self, position_name):
        """Prueba una posición específica del archivo de configuración"""
        if position_name not in self.config.get('joint_positions', {}):
            self.get_logger().error(f'❌ Posición "{position_name}" no encontrada en configuración')
            self.list_available_positions()
            return False
        
        positions = self.config['joint_positions'][position_name]
        self.get_logger().info(f'🎯 Probando posición: {position_name}')
        self.get_logger().info(f'📊 Valores: {positions}')
        
        trajectory = self.create_arm_trajectory(positions, 3.0)
        self.arm_publisher.publish(trajectory)
        
        time.sleep(4.0)  # Esperar a que complete el movimiento
        self.get_logger().info(f'✅ Movimiento a "{position_name}" completado')
        return True

    def test_gripper(self, open_gripper=True):
        """Prueba el gripper"""
        gripper_config = self.config.get('gripper', {})
        
        if open_gripper:
            position = gripper_config.get('open_position', 0.0)  # 0.0 por defecto
            action = "Abriendo"
            duration = gripper_config.get('open_time', 2.0)     # 2.0 por defecto
        else:
            position = gripper_config.get('closed_position', 0.8) # 0.8 por defecto
            action = "Cerrando"
            duration = gripper_config.get('close_time', 3.0)     # 3.0 por defecto
        
        self.get_logger().info(f'🔧 {action} gripper a posición: {position}')
        
        trajectory = self.create_gripper_trajectory(position, duration)
        self.gripper_publisher.publish(trajectory)
        
        time.sleep(duration + 0.5)
        self.get_logger().info(f'✅ Gripper {action.lower()} completado')

    def list_available_positions(self):
        """Lista las posiciones disponibles"""
        positions = self.config.get('joint_positions', {})
        self.get_logger().info('📋 POSICIONES DISPONIBLES:')
        for name, pos in positions.items():
            self.get_logger().info(f'   • {name}: {pos}')

    def interactive_test_mode(self):
        """Modo interactivo para probar posiciones"""
        self.get_logger().info('🎮 MODO INTERACTIVO ACTIVADO')
        self.get_logger().info('📝 Comandos disponibles:')
        self.get_logger().info('   • list - Listar posiciones')
        self.get_logger().info('   • current - Mostrar posición actual')
        self.get_logger().info('   • test <nombre> - Probar posición')
        self.get_logger().info('   • grip_open - Abrir gripper')
        self.get_logger().info('   • grip_close - Cerrar gripper')
        self.get_logger().info('   • sequence - Ejecutar secuencia completa')
        self.get_logger().info('   • quit - Salir')
        
        while rclpy.ok():
            try:
                command = input('\n🎯 Comando: ').strip().lower()
                
                if command == 'quit':
                    break
                elif command == 'list':
                    self.list_available_positions()
                elif command == 'current':
                    self.print_current_positions()
                elif command.startswith('test '):
                    position_name = command[5:].strip()
                    self.test_position(position_name)
                elif command == 'grip_open':
                    self.test_gripper(open_gripper=True)
                elif command == 'grip_close':
                    self.test_gripper(open_gripper=False)
                elif command == 'sequence':
                    self.test_full_sequence()
                else:
                    self.get_logger().warning(f'⚠️  Comando desconocido: {command}')
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f'❌ Error: {str(e)}')

    def test_full_sequence(self):
        """Prueba la secuencia completa de pick and place"""
        self.get_logger().info('🚀 INICIANDO SECUENCIA COMPLETA DE PRUEBA')
        
        sequence = [
            ('home', 'Posición inicial'),
            ('pick_approach', 'Acercarse al objeto'),
            ('pick_position', 'Posición de agarre'),
            ('pick_approach', 'Levantar objeto'),
            ('place_approach', 'Acercarse al destino'),
            ('place_position', 'Posición de colocación'),
            ('place_approach', 'Alejarse del destino'),
            ('home', 'Volver a home')
        ]
        
        # Abrir gripper al inicio
        self.test_gripper(open_gripper=True)
        
        for i, (position_name, description) in enumerate(sequence):
            self.get_logger().info(f'📍 Paso {i+1}/{len(sequence)}: {description}')
            
            if not self.test_position(position_name):
                self.get_logger().error(f'❌ Error en paso {i+1}, abortando secuencia')
                return False
            
            # Acciones especiales en ciertos pasos
            if position_name == 'pick_position':
                time.sleep(1.0)
                self.test_gripper(open_gripper=False)  # Cerrar gripper
            elif position_name == 'place_position':
                time.sleep(1.0)
                self.test_gripper(open_gripper=True)   # Abrir gripper
        
        self.get_logger().info('🎉 SECUENCIA COMPLETA TERMINADA')
        return True

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PositionTester()
        
        # Esperar conexiones
        node.get_logger().info('⏳ Esperando conexiones...')
        time.sleep(3.0)
        
        # Iniciar modo interactivo
        node.interactive_test_mode()
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
