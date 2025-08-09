#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import math

class TestMoveItFK(Node):
    def __init__(self):
        super().__init__('test_moveit_fk')
        
        # Cliente para cinemática directa
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
        # Esperar a que el servicio esté disponible
        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('❌ Servicio /compute_fk no disponible')
            return
        
        self.get_logger().info('✅ Servicio /compute_fk encontrado')
        
        # Hacer una prueba de cinemática directa
        self.test_forward_kinematics()
        
    def test_forward_kinematics(self):
        """Calcular posición de end effector para joints conocidos"""
        self.get_logger().info('🧪 Probando cinemática directa...')
        
        # Crear request
        request = GetPositionFK.Request()
        
        # Configurar estado del robot con joints válidos
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = [
            "joint_base", "joint_1", "joint_2", "joint_3", "joint_4"
        ]
        
        # Usar configuración de home del SRDF
        joint_positions = [0.0, 1.55, 0.0, 0.0, 0.0]  # Valores del SRDF
        request.robot_state.joint_state.position = joint_positions
        request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Especificar el link del que queremos la pose
        request.fk_link_names = ["link_5"]  # End effector
        request.header.frame_id = "base_link"
        request.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info(f'📤 Enviando request FK:')
        self.get_logger().info(f'   🤖 Joints: {[f"{math.degrees(j):.1f}°" for j in joint_positions]}')
        
        try:
            # Llamar al servicio
            future = self.fk_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                self.get_logger().info(f'📥 Respuesta FK recibida: código = {response.error_code.val}')
                
                if response.error_code.val == 1:  # SUCCESS
                    if response.pose_stamped:
                        pose = response.pose_stamped[0].pose
                        self.get_logger().info(f'✅ ¡Éxito! Posición calculada:')
                        self.get_logger().info(f'   📍 x={pose.position.x:.4f}, y={pose.position.y:.4f}, z={pose.position.z:.4f}')
                        self.get_logger().info(f'   🔄 Orientación: x={pose.orientation.x:.3f}, y={pose.orientation.y:.3f}, z={pose.orientation.z:.3f}, w={pose.orientation.w:.3f}')
                        
                        # Ahora probar cinemática inversa con esta posición
                        self.test_inverse_with_known_position(
                            pose.position.x, pose.position.y, pose.position.z,
                            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
                        )
                    else:
                        self.get_logger().warn('⚠️  No se recibió pose_stamped')
                else:
                    self.get_logger().warn(f'⚠️  FK falló: error {response.error_code.val}')
            else:
                self.get_logger().error('❌ Timeout - no se recibió respuesta FK')
                
        except Exception as e:
            self.get_logger().error(f'❌ Error en FK: {e}')
    
    def test_inverse_with_known_position(self, x, y, z, qx, qy, qz, qw):
        """Probar cinemática inversa con una posición que sabemos que es alcanzable"""
        self.get_logger().info(f'🔄 Probando IK con posición conocida: x={x:.4f}, y={y:.4f}, z={z:.4f}')
        self.get_logger().info(f'   🔄 Orientación conocida: x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}')
        
        # Importar y usar el servicio IK
        from moveit_msgs.srv import GetPositionIK
        from moveit_msgs.msg import PositionIKRequest
        from geometry_msgs.msg import PoseStamped
        from builtin_interfaces.msg import Duration
        
        ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        if not ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('❌ Servicio IK no disponible')
            return
        
        # Crear request IK
        request = GetPositionIK.Request()
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        pose_stamped.pose.position.x = float(x)
        pose_stamped.pose.position.y = float(y)
        pose_stamped.pose.position.z = float(z)
        
        # Usar la orientación exacta que calculó FK
        pose_stamped.pose.orientation.x = float(qx)
        pose_stamped.pose.orientation.y = float(qy)
        pose_stamped.pose.orientation.z = float(qz)
        pose_stamped.pose.orientation.w = float(qw)
        
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
        request.ik_request.robot_state.joint_state.position = [0.0, 1.55, 0.0, 0.0, 0.0]
        request.ik_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        
        try:
            future = ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                self.get_logger().info(f'📥 Respuesta IK: código = {response.error_code.val}')
                
                if response.error_code.val == 1:  # SUCCESS
                    joint_positions = response.solution.joint_state.position[:5]
                    self.get_logger().info(f'✅ ¡IK Éxito! Joints: {[f"{math.degrees(j):.1f}°" for j in joint_positions]}')
                else:
                    self.get_logger().warn(f'⚠️  IK falló con error {response.error_code.val}')
            else:
                self.get_logger().error('❌ Timeout IK')
        except Exception as e:
            self.get_logger().error(f'❌ Error IK: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    test_node = TestMoveItFK()
    
    # Mantener el nodo activo brevemente
    rclpy.spin_once(test_node, timeout_sec=1.0)
    
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
