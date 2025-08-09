#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, MoveItErrorCodes
from geometry_msgs.msg import PoseStamped

class DetailedIKTest(Node):
    def __init__(self):
        super().__init__('detailed_ik_test')
        
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Servicio compute_ik no disponible')
            return
            
        self.get_logger().info('Servicio compute_ik conectado')

    def test_simple_position(self):
        """Prueba con una posición extremadamente simple y alcanzable"""
        
        print("\n=== Diagnóstico detallado de IK ===")
        
        # Posición muy simple: solo mover un poco hacia adelante desde el centro
        x, y, z = 0.01, 0.0, 0.3  # Muy cerca de la posición home (0, 0, 0.3095)
        
        print(f"Probando posición simple: ({x}, {y}, {z})")
        print("Orientación: sin rotación (0, 0, 0, 1)")
        
        # Estado inicial muy conservador (cerca de home SRDF)
        initial_state = [0.0, 1.55, 0.0, 0.0, 0.0]
        
        print(f"Estado inicial: {initial_state}")
        
        # Probar con diferentes configuraciones
        configs = [
            {"name": "Sin colisiones, timeout largo", "avoid_collisions": False, "timeout": 10},
            {"name": "Sin colisiones, timeout corto", "avoid_collisions": False, "timeout": 1},
            {"name": "Con colisiones, timeout largo", "avoid_collisions": True, "timeout": 10},
        ]
        
        for config in configs:
            print(f"\n--- {config['name']} ---")
            result = self.test_ik_detailed(x, y, z, 0, 0, 0, 1, initial_state, 
                                         config['avoid_collisions'], config['timeout'])
            
            if result['success']:
                print(f"✅ ÉXITO!")
                angles = result['joint_angles']
                print(f"Ángulos: {[f'{a:.3f}' for a in angles]}")
                print(f"En grados: {[f'{a*180/3.14159:.1f}°' for a in angles]}")
                return True
            else:
                error_name = self.get_error_name(result['error_code'])
                print(f"❌ Error {result['error_code']}: {error_name}")
        
        return False

    def get_error_name(self, error_code):
        """Convertir código de error a nombre descriptivo"""
        error_names = {
            -31: "NO_IK_SOLUTION",
            -1: "FAILURE", 
            -2: "PLANNING_FAILED",
            -3: "INVALID_MOTION_PLAN",
            -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
            -5: "CONTROL_FAILED",
            -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
            -7: "TIMED_OUT",
            -10: "PREEMPTED",
            -11: "START_STATE_IN_COLLISION",
            -12: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
            -13: "GOAL_IN_COLLISION",
            -14: "GOAL_VIOLATES_PATH_CONSTRAINTS",
            -15: "GOAL_CONSTRAINTS_VIOLATED",
            -16: "INVALID_GROUP_NAME",
            -17: "INVALID_GOAL_CONSTRAINTS",
            -18: "INVALID_ROBOT_STATE",
            -19: "INVALID_LINK_NAME",
            -20: "INVALID_OBJECT_NAME",
            -21: "FRAME_TRANSFORM_FAILURE",
            -31: "NO_IK_SOLUTION",
            1: "SUCCESS"
        }
        return error_names.get(error_code, f"UNKNOWN_ERROR_{error_code}")

    def test_ik_detailed(self, x, y, z, qx, qy, qz, qw, initial_state, avoid_collisions, timeout):
        """Prueba IK con configuración específica"""
        
        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        
        request.ik_request.group_name = "arm"
        
        # Estado inicial
        request.ik_request.robot_state.joint_state.name = [
            "joint_base", "joint_1", "joint_2", "joint_3", "joint_4"
        ]
        request.ik_request.robot_state.joint_state.position = initial_state
        
        # Pose objetivo
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
        request.ik_request.avoid_collisions = avoid_collisions
        request.ik_request.timeout.sec = timeout
        
        try:
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout + 2)
            
            if future.result() is not None:
                response = future.result()
                
                if response.error_code.val == 1:  # SUCCESS
                    joint_angles = response.solution.joint_state.position[:5]
                    return {'success': True, 'joint_angles': joint_angles}
                else:
                    return {'success': False, 'error_code': response.error_code.val}
            else:
                return {'success': False, 'error_code': 'SERVICE_TIMEOUT'}
                
        except Exception as e:
            return {'success': False, 'error_code': f'EXCEPTION: {e}'}

def main():
    rclpy.init()
    
    node = DetailedIKTest()
    
    try:
        success = node.test_simple_position()
        if not success:
            print("\n🔍 PROBLEMA CONFIRMADO: MoveIt IK no funciona para este robot")
            print("Posibles causas:")
            print("1. Configuración incorrecta de kinematics.yaml")
            print("2. Problema en la cadena cinemática del URDF")
            print("3. Solver KDL no compatible con este robot específico")
            print("4. Límites de joints demasiado restrictivos")
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
