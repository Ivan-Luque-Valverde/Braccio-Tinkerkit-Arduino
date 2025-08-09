#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

class TestMoveItIK(Node):
    def __init__(self):
        super().__init__('test_moveit_ik')
        
        # Cliente para cinemática inversa
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # Esperar a que el servicio esté disponible
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('❌ Servicio /compute_ik no disponible')
            return
        
        self.get_logger().info('✅ Servicio /compute_ik encontrado')
        
        # Hacer una prueba simple
        self.test_simple_ik()
        
    def test_simple_ik(self):
        """Hacer una prueba simple de cinemática inversa"""
        self.get_logger().info('🧪 Probando cinemática inversa simple...')
        
        # Crear request
        request = GetPositionIK.Request()
        
        # Configurar pose objetivo simple
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        # Posición simple dentro del alcance del robot
        pose_stamped.pose.position.x = 0.1   # Posición mucho más conservadora
        pose_stamped.pose.position.y = 0.05  # Posición mucho más conservadora  
        pose_stamped.pose.position.z = 0.08  # Altura más alta
        
        # Orientación identidad
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        
        # Configurar request
        request.ik_request.group_name = "arm"
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout = Duration(sec=3, nanosec=0)
        
        # Estado inicial del robot
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state = JointState()
        request.ik_request.robot_state.joint_state.name = [
            "joint_base", "joint_1", "joint_2", "joint_3", "joint_4"
        ]
        request.ik_request.robot_state.joint_state.position = [0.0, 1.5, 1.0, 1.0, 1.5]
        request.ik_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info(f'📤 Enviando request:')
        self.get_logger().info(f'   🎯 Grupo: {request.ik_request.group_name}')
        self.get_logger().info(f'   📍 Posición: x={pose_stamped.pose.position.x}, y={pose_stamped.pose.position.y}, z={pose_stamped.pose.position.z}')
        self.get_logger().info(f'   ⏱️  Timeout: {request.ik_request.timeout.sec} segundos')
        
        try:
            # Llamar al servicio
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                self.get_logger().info(f'📥 Respuesta recibida: código de error = {response.error_code.val}')
                
                if response.error_code.val == 1:  # SUCCESS
                    joint_positions = response.solution.joint_state.position[:5]
                    self.get_logger().info(f'✅ ¡Éxito! Ángulos calculados:')
                    for i, pos in enumerate(joint_positions):
                        self.get_logger().info(f'   joint_{i if i == 0 else i}: {pos:.3f} rad = {pos*57.3:.1f}°')
                else:
                    self.get_logger().warn(f'⚠️  No se encontró solución: error {response.error_code.val}')
            else:
                self.get_logger().error('❌ Timeout - no se recibió respuesta del servicio')
                
        except Exception as e:
            self.get_logger().error(f'❌ Error en la llamada: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    test_node = TestMoveItIK()
    
    # Mantener el nodo activo brevemente
    rclpy.spin_once(test_node, timeout_sec=1.0)
    
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
