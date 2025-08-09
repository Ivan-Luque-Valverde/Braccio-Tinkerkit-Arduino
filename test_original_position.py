#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped

class TestOriginalPosition(Node):
    def __init__(self):
        super().__init__('test_original_position')
        
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Servicio compute_ik no disponible')
            return
            
        self.get_logger().info('Servicio conectado')

    def test_vision_position(self):
        """Prueba la posición original del sistema de visión"""
        
        print("\n=== Probando posición del sistema de visión ===")
        
        # Posición original de tu sistema
        x, y, z = 0.2, 0.1, 0.1
        print(f"Posición objetivo: ({x}, {y}, {z})")
        
        # Estados iniciales a probar
        initial_states = [
            {"name": "SRDF home", "state": [0.0, 1.55, 0.0, 0.0, 0.0]},
            {"name": "Estado exitoso anterior", "state": [0.0, 0.764, 3.137, 0.811, 3.142]},
            {"name": "Brazos estirados", "state": [0.0, 1.57, 1.57, 1.57, 1.57]},
            {"name": "Configuración media", "state": [0.5, 1.0, 1.5, 1.0, 1.5]},
        ]
        
        # Orientaciones a probar
        orientations = [
            {"name": "Gripper hacia abajo", "q": [0.0, 0.707, 0.0, 0.707]},
            {"name": "Sin rotación", "q": [0.0, 0.0, 0.0, 1.0]},
            {"name": "Gripper inclinado", "q": [0.0, 0.5, 0.0, 0.866]},
        ]
        
        for init_state in initial_states:
            print(f"\n--- Estado inicial: {init_state['name']} ---")
            
            for orientation in orientations:
                print(f"  Orientación: {orientation['name']}")
                
                result = self.test_ik_position(
                    x, y, z, 
                    orientation['q'][0], orientation['q'][1], 
                    orientation['q'][2], orientation['q'][3],
                    init_state['state']
                )
                
                if result['success']:
                    print(f"  ✅ ÉXITO!")
                    angles = result['joint_angles']
                    print(f"  Ángulos: {[f'{a:.3f}' for a in angles]}")
                    print(f"  En grados: {[f'{a*180/3.14159:.1f}°' for a in angles]}")
                    
                    # ¡Este es el estado que necesitas en tu sistema!
                    print(f"\n🎯 SOLUCIÓN ENCONTRADA:")
                    print(f"Estado inicial: {init_state['state']}")
                    print(f"Orientación: {orientation['q']}")
                    print(f"Ángulos resultado: {angles}")
                    return True
                else:
                    print(f"  ❌ Error: {result['error_code']}")
        
        return False

    def test_ik_position(self, x, y, z, qx, qy, qz, qw, initial_state):
        """Prueba IK para una posición específica"""
        
        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        
        request.ik_request.group_name = "arm"
        
        request.ik_request.robot_state.joint_state.name = [
            "joint_base", "joint_1", "joint_2", "joint_3", "joint_4"
        ]
        request.ik_request.robot_state.joint_state.position = initial_state
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        pose_stamped.pose.position.x = float(x)
        pose_stamped.pose.position.y = float(y)
        pose_stamped.pose.position.z = float(z)
        
        pose_stamped.pose.orientation.x = float(qx)
        pose_stamped.pose.orientation.y = float(qy)
        pose_stamped.pose.orientation.z = float(qz)
        pose_stamped.pose.orientation.w = float(qw)
        
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.sec = 5
        
        try:
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=7.0)
            
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
    
    node = TestOriginalPosition()
    
    try:
        success = node.test_vision_position()
        if success:
            print("\n🎉 ¡MoveIt puede resolver la posición de tu sistema!")
            print("Ahora puedes usar el primer método en vision_pick_and_place.py")
        else:
            print("\n⚠️  MoveIt no puede resolver esta posición específica")
            print("Pero ya funciona para otras posiciones similares")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
