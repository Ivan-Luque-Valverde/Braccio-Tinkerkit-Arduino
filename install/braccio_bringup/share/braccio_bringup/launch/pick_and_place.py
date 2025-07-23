#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        
        # Inicializar MoveItPy
        self.moveit = MoveItPy(node_name="pick_and_place")
        self.arm_group = self.moveit.get_planning_component("braccio_arm")
        self.gripper_group = self.moveit.get_planning_component("braccio_gripper")
        
        # Configurar velocidades
        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.5)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def move_to_pose(self, pose):
        try:
            self.arm_group.set_start_state_to_current_state()
            self.arm_group.set_goal_state(pose)
            plan = self.arm_group.plan()
            if plan[0]:
                self.arm_group.execute()
                return True
            return False
        except Exception as e:
            self.get_logger().error(f'Error en move_to_pose: {str(e)}')
            return False

    def control_gripper(self, open_gripper=True):
        try:
            joint_positions = [0.7] if open_gripper else [0.0]
            self.gripper_group.set_joint_value_target(joint_positions)
            plan = self.gripper_group.plan()
            if plan[0]:
                self.gripper_group.execute()
                return True
            return False
        except Exception as e:
            self.get_logger().error(f'Error en control_gripper: {str(e)}')
            return False

    def pick_and_place(self):
        # Definir poses
        pick_pose = Pose()
        pick_pose.position.x = 0.15
        pick_pose.position.y = 0.0
        pick_pose.position.z = 0.05
        pick_pose.orientation.w = 1.0

        place_pose = Pose()
        place_pose.position.x = 0.15
        place_pose.position.y = 0.15
        place_pose.position.z = 0.05
        place_pose.orientation.w = 1.0

        try:
            # Secuencia de pick and place
            self.get_logger().info('Iniciando pick and place')
            
            # 1. Abrir gripper
            if not self.control_gripper(open_gripper=True):
                return False
            
            # 2. Mover a posición de pick
            if not self.move_to_pose(pick_pose):
                return False
            
            # 3. Cerrar gripper
            if not self.control_gripper(open_gripper=False):
                return False
            
            # 4. Mover a posición de place
            if not self.move_to_pose(place_pose):
                return False
            
            # 5. Abrir gripper
            if not self.control_gripper(open_gripper=True):
                return False

            return True

        except Exception as e:
            self.get_logger().error(f'Error en pick_and_place: {str(e)}')
            return False

def main(args=None):
    rclpy.init(args=args)
    pick_place_node = PickAndPlace()
    
    try:
        success = pick_place_node.pick_and_place()
        if success:
            pick_place_node.get_logger().info('Pick and place completado con éxito')
        else:
            pick_place_node.get_logger().error('Pick and place falló')
    finally:
        pick_place_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()