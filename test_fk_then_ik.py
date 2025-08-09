#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class TestNearHome(Node):
    def __init__(self):
        super().__init__('test_near_home')
        
        # Clientes para servicios
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
        # Esperar servicios
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Servicio compute_ik no disponible')
            return
            
        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Servicio compute_fk no disponible')
            return
            
        self.get_logger().info('Servicios conectados')

    def test_fk_then_ik(self):
        """Primero obtener posición con FK, luego probar IK en esa misma posición"""
        
        print("\n=== PASO 1: Forward Kinematics ===")
        
        # Configuración de home conocida
        home_joints = [1.57, 1.57, 1.57, 1.57, 1.57]  # posición home típica
        
        print(f"Ángulos home: {home_joints}")
        
        # Obtener posición con FK
        fk_result = self.get_fk_position(home_joints)
        
        if not fk_result['success']:
            print(f"❌ FK falló: {fk_result['error']}")
            return
            
        pose = fk_result['pose']
        print(f"✅ FK exitoso:")
        print(f"  Posición: ({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f})")
        print(f"  Orientación: ({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, {pose.orientation.z:.4f}, {pose.orientation.w:.4f})")
        
        print(f"\n=== PASO 2: Inverse Kinematics en la misma posición ===")
        
        # Ahora probar IK en esa misma posición
        ik_result = self.test_ik_position(
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        
        if ik_result['success']:
            print(f"✅ IK exitoso! Ángulos encontrados:")
            for i, angle in enumerate(ik_result['joint_angles']):
                original = home_joints[i]
                diff = abs(angle - original)
                print(f"  joint_{i}: {angle:.4f} rad (original: {original:.4f}, diff: {diff:.4f})")
        else:
            print(f"❌ IK falló: código {ik_result['error_code']}")
            print("¡Esto indica un problema fundamental del solver!")

    def get_fk_position(self, joint_angles):
        """Obtener posición usando Forward Kinematics"""
        
        request = GetPositionFK.Request()
        
        request.fk_link_names = ["link_4"]  # end-effector
        
        # Estado de las articulaciones
        joint_state = JointState()
        joint_state.name = ["joint_base", "joint_1", "joint_2", "joint_3", "joint_4"]
        joint_state.position = joint_angles
        request.robot_state.joint_state = joint_state
        
        try:
            future = self.fk_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            
            if future.result() is not None:
                response = future.result()
                if response.error_code.val == 1:  # SUCCESS
                    return {'success': True, 'pose': response.pose_stamped[0].pose}
                else:
                    return {'success': False, 'error': response.error_code.val}
            else:
                return {'success': False, 'error': 'TIMEOUT'}
                
        except Exception as e:
            return {'success': False, 'error': f'EXCEPTION: {e}'}

    def test_ik_position(self, x, y, z, qx, qy, qz, qw):
        """Prueba IK para una posición específica"""
        
        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        
        request.ik_request.group_name = "arm"
        request.ik_request.robot_state.joint_state.name = [
            "joint_base", "joint_1", "joint_2", "joint_3", "joint_4"
        ]
        request.ik_request.robot_state.joint_state.position = [1.57, 1.57, 1.57, 1.57, 1.57]
        
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
        request.ik_request.timeout.sec = 5
        
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
    
    node = TestNearHome()
    
    try:
        node.test_fk_then_ik()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
