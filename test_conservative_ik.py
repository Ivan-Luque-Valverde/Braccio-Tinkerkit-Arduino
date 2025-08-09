#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped
import time

class TestConservativeIK(Node):
    def __init__(self):
        super().__init__('test_conservative_ik')
        
        # Cliente para el servicio de IK
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # Esperar a que el servicio esté disponible
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Servicio compute_ik no disponible')
            return
            
        self.get_logger().info('Servicio compute_ik conectado')

    def test_conservative_position(self):
        """Prueba posición más conservadora"""
        
        # Posición sugerida por el análisis
        x, y, z = 0.15, 0.05, 0.12
        
        # Orientaciones a probar
        orientations = [
            {"name": "Gripper hacia abajo", "x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707},
            {"name": "Gripper diagonal", "x": 0.0, "y": 0.5, "z": 0.0, "w": 0.866},
            {"name": "Gripper levemente inclinado", "x": 0.0, "y": 0.3, "z": 0.0, "w": 0.954},
            {"name": "Gripper casi horizontal", "x": 0.0, "y": 0.1, "z": 0.0, "w": 0.995}
        ]
        
        for orientation in orientations:
            print(f"\n=== Probando {orientation['name']} ===")
            print(f"Posición: ({x:.3f}, {y:.3f}, {z:.3f})")
            print(f"Orientación: ({orientation['x']:.3f}, {orientation['y']:.3f}, {orientation['z']:.3f}, {orientation['w']:.3f})")
            
            result = self.test_ik_position(x, y, z, orientation)
            if result['success']:
                print(f"✅ ÉXITO! Ángulos encontrados:")
                for i, angle in enumerate(result['joint_angles']):
                    print(f"  joint_{i}: {angle:.4f} rad = {angle*180/3.14159:.1f}°")
                break
            else:
                print(f"❌ Error: {result['error_code']}")

    def test_ik_position(self, x, y, z, orientation):
        """Prueba una posición específica con MoveIt IK"""
        
        # Crear request
        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        
        # Configurar pose objetivo
        request.ik_request.group_name = "arm"
        request.ik_request.robot_state.joint_state.name = [
            "joint_base", "joint_1", "joint_2", "joint_3", "joint_4"
        ]
        request.ik_request.robot_state.joint_state.position = [0.0, 1.5, 1.5, 1.5, 1.5]
        
        # Pose objetivo
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        
        pose_stamped.pose.orientation.x = orientation['x']
        pose_stamped.pose.orientation.y = orientation['y'] 
        pose_stamped.pose.orientation.z = orientation['z']
        pose_stamped.pose.orientation.w = orientation['w']
        
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.sec = 5
        
        # Llamar al servicio
        try:
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=6.0)
            
            if future.result() is not None:
                response = future.result()
                
                if response.error_code.val == 1:  # SUCCESS
                    joint_angles = response.solution.joint_state.position[:5]
                    return {'success': True, 'joint_angles': joint_angles}
                else:
                    return {'success': False, 'error_code': response.error_code.val}
            else:
                return {'success': False, 'error_code': 'TIMEOUT'}
                
        except Exception as e:
            return {'success': False, 'error_code': f'EXCEPTION: {e}'}

def main():
    rclpy.init()
    
    node = TestConservativeIK()
    
    try:
        node.test_conservative_position()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
