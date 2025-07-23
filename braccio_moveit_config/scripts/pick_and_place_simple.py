#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class SimplePickAndPlace(Node):
    def __init__(self):
        super().__init__('simple_pick_and_place')
        
        # Publisher para el controlador de trayectoria
        self.arm_publisher = self.create_publisher(
            JointTrajectory,
            '/position_trajectory_controller/joint_trajectory',
            10
        )
        
        # Publisher separado para el gripper
        self.gripper_publisher = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info('Nodo Simple Pick and Place iniciado')

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

    def move_arm(self, positions, description="", duration=3.0):
        """Mueve el brazo a las posiciones especificadas"""
        self.get_logger().info(f'Moviendo brazo: {description}')
        
        trajectory = self.create_arm_trajectory(positions, duration)
        self.arm_publisher.publish(trajectory)
        
        # Esperar a que complete el movimiento
        time.sleep(duration + 0.5)

    def control_gripper(self, open_gripper=True, duration=None):
        """Controla el gripper (True = abrir, False = cerrar)"""
        if open_gripper:
            position = 0.0  # Completamente abierto
            action = "Abriendo"
            duration = 2.0 if duration is None else duration
        else:
            position = 0.8  # Cerrado (valor que funciona)
            action = "Cerrando"
            duration = 3.0 if duration is None else duration
        
        self.get_logger().info(f'{action} gripper a posición: {position}')
        
        trajectory = self.create_gripper_trajectory(position, duration)
        self.gripper_publisher.publish(trajectory)
        
        # Esperar a que complete el movimiento
        time.sleep(duration + 0.5)

    def execute_pick_and_place_sequence(self):
        """Ejecuta la secuencia completa de pick and place"""
        try:
            self.get_logger().info('=== INICIANDO SECUENCIA SIMPLE PICK AND PLACE ===')
            
            # Definir posiciones (en radianes)
            home_position = [0.0, 1.57, 0.0, 0.0, 0.0]           # Posición inicial
            approach_position = [0.0, 1.2, 1.0, 0.8, 0.0]        # Acercarse al objeto
            pick_position = [0.0, 1.0, 1.4, 1.2, 0.0]            # Posición de agarre
            lift_position = [0.0, 1.2, 1.0, 0.8, 0.0]            # Levantar objeto
            transport_position = [1.57, 1.2, 1.0, 0.8, 0.0]      # Transportar
            place_position = [1.57, 1.0, 1.4, 1.2, 0.0]          # Colocar objeto
            retreat_position = [1.57, 1.2, 1.0, 0.8, 0.0]        # Retroceder
            
            # Paso 1: Ir a posición home y abrir gripper
            self.move_arm(home_position, "posición home", 3.0)
            self.control_gripper(open_gripper=True)
            
            # Paso 2: Acercarse al objeto
            self.move_arm(approach_position, "acercarse al objeto", 3.0)
            
            # Paso 3: Posición final de agarre
            self.move_arm(pick_position, "posición de agarre", 2.0)
            
            # Paso 4: Cerrar gripper para agarrar
            self.control_gripper(open_gripper=False)
            
            # Paso 5: Levantar objeto
            self.move_arm(lift_position, "levantar objeto", 2.0)
            
            # Paso 6: Transportar a posición de destino
            self.move_arm(transport_position, "transportar objeto", 4.0)
            
            # Paso 7: Bajar para colocar
            self.move_arm(place_position, "bajar para colocar", 2.0)
            
            # Paso 8: Abrir gripper para soltar
            self.control_gripper(open_gripper=True)
            
            # Paso 9: Retroceder
            self.move_arm(retreat_position, "retroceder", 2.0)
            
            # Paso 10: Volver a home
            self.move_arm(home_position, "volver a home", 4.0)
            
            self.get_logger().info('=== SECUENCIA COMPLETADA EXITOSAMENTE ===')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error durante pick and place: {str(e)}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimplePickAndPlace()
        
        # Esperar a que los publishers se conecten
        node.get_logger().info('Esperando conexiones...')
        time.sleep(3.0)
        
        # Ejecutar la secuencia
        success = node.execute_pick_and_place_sequence()
        
        if success:
            node.get_logger().info('Pick and place ejecutado exitosamente')
        else:
            node.get_logger().error('Error durante la ejecución')
            
    except KeyboardInterrupt:
        node.get_logger().info('Interrumpido por el usuario')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
