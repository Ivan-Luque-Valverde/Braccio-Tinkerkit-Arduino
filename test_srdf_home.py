#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped

class TestFromSRDFHome(Node):
    def __init__(self):
        super().__init__('test_from_srdf_home')
        
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Servicio compute_ik no disponible')
            return
            
        self.get_logger().info('Servicio compute_ik conectado')

    def test_with_srdf_home(self):
        """Prueba IK usando la configuración home del SRDF como punto inicial"""
        
        print("\n=== Probando IK con posición inicial del SRDF ===")
        
        # Posición home del SRDF:
        # joint_1: 1.55, joint_2: 0, joint_3: 0, joint_4: 0, joint_base: 0
        srdf_home = [0.0, 1.55, 0.0, 0.0, 0.0]  # [base, 1, 2, 3, 4]
        
        print(f"Usando estado inicial SRDF home: {srdf_home}")
        
        # Probar varias posiciones alcanzables
        test_positions = [
            {"name": "Muy cerca del home", "pos": (0.05, 0.0, 0.25), "orient": (0.0, 0.0, 0.0, 1.0)},
            {"name": "Cerca del home", "pos": (0.1, 0.0, 0.2), "orient": (0.0, 0.0, 0.0, 1.0)},
            {"name": "Posición original", "pos": (0.2, 0.1, 0.08), "orient": (0.0, 0.707, 0.0, 0.707)},
        ]
        
        for test in test_positions:
            print(f"\n--- {test['name']} ---")
            print(f"Posición: {test['pos']}")
            print(f"Orientación: {test['orient']}")
            
            result = self.test_ik_position(
                test['pos'][0], test['pos'][1], test['pos'][2],
                test['orient'][0], test['orient'][1], test['orient'][2], test['orient'][3],
                srdf_home
            )
            
            if result['success']:
                print(f"✅ ÉXITO! Ángulos encontrados:")
                for i, angle in enumerate(result['joint_angles']):
                    print(f"  joint_{i}: {angle:.4f} rad = {angle*180/3.14159:.1f}°")
                break
            else:
                print(f"❌ Error: {result['error_code']}")

    def test_ik_position(self, x, y, z, qx, qy, qz, qw, initial_state):
        """Prueba IK con estado inicial específico"""
        
        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        
        request.ik_request.group_name = "arm"
        
        # Usar el estado inicial del SRDF
        request.ik_request.robot_state.joint_state.name = [
            "joint_base", "joint_1", "joint_2", "joint_3", "joint_4"
        ]
        request.ik_request.robot_state.joint_state.position = initial_state
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw
        
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.sec = 10  # Tiempo largo
        
        try:
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=12.0)
            
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
    
    node = TestFromSRDFHome()
    
    try:
        node.test_with_srdf_home()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
