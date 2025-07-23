#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionIKRequest,
    WorkspaceParameters,
)
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import math

class BraccioPickAndPlace(Node):
    def __init__(self):
        super().__init__('braccio_pick_and_place')
        
        # Cliente de acción para MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Publisher para el gripper (directamente a joint_trajectory)
        self.gripper_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Configuración del brazo Braccio
        self.joint_names = [
            "joint_base",
            "joint_1", 
            "joint_2",
            "joint_3",
            "joint_4"
        ]
        
        self.gripper_joint = "right_gripper_joint"
        
        # Posiciones predefinidas
        self.home_position = [0.0, 1.57, 0.0, 0.0, 0.0]  # Posición home
        self.pick_position = [0.0, 1.2, 1.0, 0.5, 0.0]   # Posición de agarre
        self.place_position = [1.57, 1.2, 1.0, 0.5, 0.0] # Posición de colocación
        
        self.get_logger().info('Nodo Pick and Place iniciado')

    def wait_for_move_group(self):
        """Espera a que el servicio de MoveGroup esté disponible"""
        self.get_logger().info('Esperando al servidor MoveGroup...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('Servidor MoveGroup disponible')

    def create_joint_goal(self, joint_positions):
        """Crea un goal de MoveGroup para posiciones de joint"""
        goal_msg = MoveGroup.Goal()
        
        # Configurar el request
        goal_msg.request.group_name = "braccio_arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # Crear constraints para los joints
        joint_constraints = []
        for i, joint_name in enumerate(self.joint_names):
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = joint_positions[i]
            constraint.tolerance_above = 0.1
            constraint.tolerance_below = 0.1
            constraint.weight = 1.0
            joint_constraints.append(constraint)
        
        goal_constraint = Constraints()
        goal_constraint.joint_constraints = joint_constraints
        goal_msg.request.goal_constraints = [goal_constraint]
        
        return goal_msg

    def move_to_joint_position(self, joint_positions, description=""):
        """Mueve el brazo a la posición de joints especificada"""
        self.get_logger().info(f'Moviendo a: {description}')
        
        goal_msg = self.create_joint_goal(joint_positions)
        
        # Enviar goal
        future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazado')
            return False
        
        # Esperar resultado
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info(f'Movimiento completado: {description}')
            return True
        else:
            self.get_logger().error(f'Error en movimiento: {result.result.error_code.val}')
            return False

    def control_gripper(self, open_gripper=True):
        """Controla el gripper (True = abrir, False = cerrar)"""
        gripper_value = 0.0 if open_gripper else 1.0
        action = "Abriendo" if open_gripper else "Cerrando"
        
        self.get_logger().info(f'{action} gripper')
        
        # Crear mensaje JointState para el gripper
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [self.gripper_joint]
        joint_state.position = [gripper_value]
        
        # Publicar varias veces para asegurar que se reciba
        for _ in range(10):
            self.gripper_publisher.publish(joint_state)
            time.sleep(0.1)

    def execute_pick_and_place(self):
        """Ejecuta la secuencia completa de pick and place"""
        try:
            self.get_logger().info('=== INICIANDO SECUENCIA PICK AND PLACE ===')
            
            # Paso 1: Ir a posición home
            if not self.move_to_joint_position(self.home_position, "posición home"):
                return False
            time.sleep(1.0)
            
            # Paso 2: Abrir gripper
            self.control_gripper(open_gripper=True)
            time.sleep(1.0)
            
            # Paso 3: Ir a posición de agarre
            if not self.move_to_joint_position(self.pick_position, "posición de agarre"):
                return False
            time.sleep(1.0)
            
            # Paso 4: Cerrar gripper (agarrar objeto)
            self.control_gripper(open_gripper=False)
            time.sleep(2.0)  # Dar tiempo para que se cierre completamente
            
            # Paso 5: Levantar un poco
            lift_position = self.pick_position.copy()
            lift_position[2] -= 0.3  # Levantar joint_2
            if not self.move_to_joint_position(lift_position, "levantar objeto"):
                return False
            time.sleep(1.0)
            
            # Paso 6: Ir a posición de colocación
            if not self.move_to_joint_position(self.place_position, "posición de colocación"):
                return False
            time.sleep(1.0)
            
            # Paso 7: Bajar para colocar
            lower_position = self.place_position.copy()
            lower_position[2] += 0.3  # Bajar joint_2
            if not self.move_to_joint_position(lower_position, "bajar para colocar"):
                return False
            time.sleep(1.0)
            
            # Paso 8: Abrir gripper (soltar objeto)
            self.control_gripper(open_gripper=True)
            time.sleep(2.0)
            
            # Paso 9: Retroceder un poco
            if not self.move_to_joint_position(self.place_position, "retroceder"):
                return False
            time.sleep(1.0)
            
            # Paso 10: Volver a home
            if not self.move_to_joint_position(self.home_position, "volver a home"):
                return False
            
            self.get_logger().info('=== SECUENCIA PICK AND PLACE COMPLETADA ===')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error durante pick and place: {str(e)}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BraccioPickAndPlace()
        
        # Esperar a que MoveGroup esté disponible
        node.wait_for_move_group()
        
        # Esperar un poco más para que todo se inicialice
        time.sleep(3.0)
        
        # Ejecutar la secuencia
        success = node.execute_pick_and_place()
        
        if success:
            node.get_logger().info('Pick and place ejecutado exitosamente')
        else:
            node.get_logger().error('Error durante la ejecución de pick and place')
        
        # Mantener el nodo activo por un momento
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()