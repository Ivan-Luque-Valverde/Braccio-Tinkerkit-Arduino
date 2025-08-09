#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import math

class QuickTestIK(Node):
    def __init__(self):
        super().__init__('quick_test_ik')
        
        # Cliente para cinemática inversa
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # Esperar a que el servicio esté disponible
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('❌ Servicio /compute_ik no disponible')
            return
        
        self.get_logger().info('✅ Servicio /compute_ik encontrado')
        
        # Probar con la posición problemática de los logs
        self.test_problematic_position()
        
    def test_problematic_position(self):
        """Probar con la posición que estaba causando problemas: (0.311, 0.101)"""
        target_x = 0.2
        target_y = 0.1
        target_z = 0.08  # Altura más realista (8cm del suelo)
        
        self.get_logger().info(f'🧪 Probando posición problemática: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}')
        
        # Distancia desde la base
        distance = math.sqrt(target_x**2 + target_y**2)
        self.get_logger().info(f'📏 Distancia desde base: {distance:.3f}m')
        
        if distance > 0.3:
            self.get_logger().warn(f'⚠️  Posición fuera de alcance: {distance:.3f}m > 0.3m')
            return
        
        # Crear request
        request = GetPositionIK.Request()
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        pose_stamped.pose.position.x = float(target_x)
        pose_stamped.pose.position.y = float(target_y)
        pose_stamped.pose.position.z = float(target_z)
        
        # Probar diferentes orientaciones
        orientations = [
            (1.0, 0.0, 0.0, 0.0),    # Orientación similar a home position del Braccio
            (0.0, 1.0, 0.0, 0.0),    # 180° en X 
            (0.0, 0.0, 1.0, 0.0),    # 180° en Y
            (0.0, 0.0, 0.0, 1.0),    # Identidad (último recurso)
        ]
        
        for i, (ox, oy, oz, ow) in enumerate(orientations):
            pose_stamped.pose.orientation.x = ox
            pose_stamped.pose.orientation.y = oy
            pose_stamped.pose.orientation.z = oz
            pose_stamped.pose.orientation.w = ow
            
            request.ik_request.group_name = "arm"
            request.ik_request.pose_stamped = pose_stamped
            request.ik_request.avoid_collisions = False
            request.ik_request.timeout = Duration(sec=3, nanosec=0)
            
            # Estado inicial
            request.ik_request.robot_state = RobotState()
            request.ik_request.robot_state.joint_state = JointState()
            request.ik_request.robot_state.joint_state.name = [
                "joint_base", "joint_1", "joint_2", "joint_3", "joint_4"
            ]
            request.ik_request.robot_state.joint_state.position = [0.0, 1.5, 1.0, 1.0, 1.5]
            request.ik_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
            
            self.get_logger().info(f'🔄 Intento {i+1}/4 con orientación ({ox:.1f}, {oy:.1f}, {oz:.1f}, {ow:.1f})')
            
            try:
                future = self.ik_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() is not None:
                    response = future.result()
                    
                    if response.error_code.val == 1:  # SUCCESS
                        joint_positions = response.solution.joint_state.position[:5]
                        self.get_logger().info(f'✅ ¡ÉXITO en intento {i+1}! Joints: {[f"{math.degrees(j):.1f}°" for j in joint_positions]}')
                        return
                    else:
                        self.get_logger().info(f'⚠️  Intento {i+1} falló: error {response.error_code.val}')
                else:
                    self.get_logger().info(f'⚠️  Timeout en intento {i+1}')
                    
            except Exception as e:
                self.get_logger().error(f'❌ Error en intento {i+1}: {e}')
        
        self.get_logger().warn('❌ Todos los intentos fallaron')

def main(args=None):
    rclpy.init(args=args)
    
    test_node = QuickTestIK()
    
    # Mantener el nodo activo brevemente
    rclpy.spin_once(test_node, timeout_sec=1.0)
    
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
