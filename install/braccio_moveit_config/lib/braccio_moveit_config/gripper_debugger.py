#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time

class GripperDebugger(Node):
    def __init__(self):
        super().__init__('gripper_debugger')
        
        # Publisher para el gripper
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
        self.get_logger().info('🔍 Gripper Debugger iniciado')

    def joint_state_callback(self, msg):
        """Callback para recibir estados de joints"""
        self.current_joint_states = msg

    def get_gripper_position(self):
        """Obtiene la posición actual del gripper"""
        if self.current_joint_states is None:
            return None
        
        try:
            idx = self.current_joint_states.name.index('right_gripper_joint')
            return self.current_joint_states.position[idx]
        except ValueError:
            return None

    def create_gripper_trajectory(self, position, duration_sec=2.0):
        """Crea una trayectoria para el gripper"""
        traj = JointTrajectory()
        traj.joint_names = ["right_gripper_joint"]
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        traj.points.append(point)
        
        return traj

    def test_gripper_movement(self, target_position, description=""):
        """Prueba el movimiento del gripper a una posición específica"""
        current_pos = self.get_gripper_position()
        
        self.get_logger().info(f'🎯 Probando: {description}')
        self.get_logger().info(f'📍 Posición actual: {current_pos}')
        self.get_logger().info(f'🏁 Posición objetivo: {target_position}')
        
        # Crear y enviar trayectoria
        trajectory = self.create_gripper_trajectory(target_position, 3.0)
        self.gripper_publisher.publish(trajectory)
        
        # Monitorear el movimiento
        start_time = time.time()
        timeout = 5.0
        
        while time.time() - start_time < timeout:
            current_pos = self.get_gripper_position()
            if current_pos is not None:
                self.get_logger().info(f'📊 Posición: {current_pos:.3f} -> Objetivo: {target_position:.3f}')
            time.sleep(0.5)
        
        final_pos = self.get_gripper_position()
        self.get_logger().info(f'✅ Posición final: {final_pos}')
        
        return final_pos

    def interactive_gripper_test(self):
        """Modo interactivo para probar el gripper"""
        self.get_logger().info('🎮 MODO INTERACTIVO DEL GRIPPER')
        self.get_logger().info('📝 Comandos:')
        self.get_logger().info('   • open - Abrir gripper (0.0)')
        self.get_logger().info('   • close - Cerrar gripper (0.8)')
        self.get_logger().info('   • pos <valor> - Ir a posición específica')
        self.get_logger().info('   • current - Mostrar posición actual')
        self.get_logger().info('   • range - Probar rango completo')
        self.get_logger().info('   • quit - Salir')
        
        while rclpy.ok():
            try:
                command = input('\n🔧 Comando gripper: ').strip().lower()
                
                if command == 'quit':
                    break
                elif command == 'open':
                    self.test_gripper_movement(0.0, "Abrir gripper")
                elif command == 'close':
                    self.test_gripper_movement(0.8, "Cerrar gripper")
                elif command.startswith('pos '):
                    try:
                        pos = float(command[4:].strip())
                        self.test_gripper_movement(pos, f"Posición {pos}")
                    except ValueError:
                        self.get_logger().error('❌ Valor inválido')
                elif command == 'current':
                    pos = self.get_gripper_position()
                    self.get_logger().info(f'📍 Posición actual: {pos}')
                elif command == 'range':
                    self.test_gripper_range()
                else:
                    self.get_logger().warning(f'⚠️  Comando desconocido: {command}')
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f'❌ Error: {str(e)}')

    def test_gripper_range(self):
        """Prueba diferentes posiciones del gripper"""
        positions = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
        
        self.get_logger().info('🔄 Probando rango completo del gripper')
        
        for i, pos in enumerate(positions):
            self.get_logger().info(f'📍 Paso {i+1}/{len(positions)}: Posición {pos}')
            self.test_gripper_movement(pos, f"Posición {pos}")
            time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = GripperDebugger()
        
        # Esperar conexiones
        node.get_logger().info('⏳ Esperando conexiones...')
        time.sleep(3.0)
        
        # Iniciar modo interactivo
        node.interactive_gripper_test()
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
