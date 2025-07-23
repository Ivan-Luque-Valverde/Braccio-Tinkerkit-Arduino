#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
    WorkspaceParameters,
    PlanningScene,
    CollisionObject
)
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Vector3
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import math

class AdvancedPickAndPlace(Node):
    def __init__(self):
        super().__init__('advanced_pick_and_place')
        
        # Cliente de acción para MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Publisher para planning scene (obstáculos)
        self.planning_scene_publisher = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        # Subscriber para estados de joints
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Variables de estado
        self.current_joint_states = None
        self.gripper_closed = False
        
        # Configuración del workspace
        self.workspace_limits = {
            'x_min': -0.5, 'x_max': 0.5,
            'y_min': -0.5, 'y_max': 0.5,
            'z_min': 0.0, 'z_max': 0.8
        }
        
        # Posiciones de objetos y destinos
        self.object_positions = [
            {'name': 'objeto1', 'pose': [0.3, 0.0, 0.1]},
            {'name': 'objeto2', 'pose': [0.2, 0.2, 0.1]},
        ]
        
        self.target_positions = [
            {'name': 'destino1', 'pose': [-0.3, 0.0, 0.1]},
            {'name': 'destino2', 'pose': [-0.2, 0.2, 0.1]},
        ]
        
        self.get_logger().info('Nodo Advanced Pick and Place iniciado')

    def joint_state_callback(self, msg):
        """Callback para estados de joints"""
        self.current_joint_states = msg
        # Verificar estado del gripper
        if 'right_gripper_joint' in msg.name:
            idx = msg.name.index('right_gripper_joint')
            self.gripper_closed = msg.position[idx] > 0.5

    def wait_for_move_group(self):
        """Espera a que el servicio de MoveGroup esté disponible"""
        self.get_logger().info('Esperando al servidor MoveGroup...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('Servidor MoveGroup disponible')

    def add_table_to_scene(self):
        """Añade una mesa como obstáculo en la escena"""
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        
        # Crear objeto mesa
        table = CollisionObject()
        table.header.frame_id = "base_link"
        table.header.stamp = self.get_clock().now().to_msg()
        table.id = "table"
        
        # Definir forma de la mesa (caja)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.8, 0.8, 0.02]  # 80cm x 80cm x 2cm
        
        # Posición de la mesa
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = -0.01  # Justo debajo del workspace
        pose.orientation.w = 1.0
        
        table.primitives.append(box)
        table.primitive_poses.append(pose)
        table.operation = CollisionObject.ADD
        
        scene_msg.world.collision_objects.append(table)
        self.planning_scene_publisher.publish(scene_msg)
        
        self.get_logger().info('Mesa añadida a la escena de planificación')
        time.sleep(1.0)  # Dar tiempo para que se procese

    def create_cartesian_goal(self, target_pose, orientation=None):
        """Crea un goal de MoveGroup para una pose cartesiana"""
        goal_msg = MoveGroup.Goal()
        
        # Configurar el request
        goal_msg.request.group_name = "braccio_arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # Configurar workspace
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.min_corner.x = self.workspace_limits['x_min']
        goal_msg.request.workspace_parameters.min_corner.y = self.workspace_limits['y_min']
        goal_msg.request.workspace_parameters.min_corner.z = self.workspace_limits['z_min']
        goal_msg.request.workspace_parameters.max_corner.x = self.workspace_limits['x_max']
        goal_msg.request.workspace_parameters.max_corner.y = self.workspace_limits['y_max']
        goal_msg.request.workspace_parameters.max_corner.z = self.workspace_limits['z_max']
        
        # Crear constraint de posición
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base_link"
        position_constraint.link_name = "link_5"  # End effector
        
        # Definir región objetivo (pequeña caja alrededor del objetivo)
        box_constraint = SolidPrimitive()
        box_constraint.type = SolidPrimitive.BOX
        box_constraint.dimensions = [0.02, 0.02, 0.02]  # 2cm de tolerancia
        
        pose_constraint = Pose()
        pose_constraint.position.x = target_pose[0]
        pose_constraint.position.y = target_pose[1]
        pose_constraint.position.z = target_pose[2]
        pose_constraint.orientation.w = 1.0
        
        position_constraint.constraint_region.primitives.append(box_constraint)
        position_constraint.constraint_region.primitive_poses.append(pose_constraint)
        position_constraint.weight = 1.0
        
        # Crear constraint de orientación
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = "link_5"
        
        if orientation is None:
            # Orientación por defecto (gripper hacia abajo)
            orientation_constraint.orientation.x = 0.0
            orientation_constraint.orientation.y = 0.707
            orientation_constraint.orientation.z = 0.0
            orientation_constraint.orientation.w = 0.707
        else:
            orientation_constraint.orientation.x = orientation[0]
            orientation_constraint.orientation.y = orientation[1]
            orientation_constraint.orientation.z = orientation[2]
            orientation_constraint.orientation.w = orientation[3]
        
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        
        # Combinar constraints
        goal_constraint = Constraints()
        goal_constraint.position_constraints.append(position_constraint)
        goal_constraint.orientation_constraints.append(orientation_constraint)
        
        goal_msg.request.goal_constraints = [goal_constraint]
        
        return goal_msg

    def move_to_pose(self, target_pose, description="", orientation=None):
        """Mueve el end effector a una pose específica"""
        self.get_logger().info(f'Moviendo a pose: {description}')
        
        goal_msg = self.create_cartesian_goal(target_pose, orientation)
        
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

    def control_gripper_advanced(self, open_gripper=True, force=0.5):
        """Control avanzado del gripper con feedback"""
        action = "Abriendo" if open_gripper else "Cerrando"
        self.get_logger().info(f'{action} gripper con fuerza {force}')
        
        # Aquí se podría implementar control de fuerza
        # Por ahora, simulamos con tiempo de espera variable
        wait_time = 1.0 if open_gripper else 2.0
        time.sleep(wait_time)
        
        return True

    def verify_grasp(self):
        """Verifica si el objeto fue agarrado correctamente"""
        if self.current_joint_states is None:
            return False
        
        # Verificar posición del gripper y posibles sensores de fuerza
        return self.gripper_closed

    def execute_advanced_pick_and_place(self, object_idx=0, target_idx=0):
        """Ejecuta una secuencia avanzada de pick and place"""
        try:
            self.get_logger().info('=== INICIANDO SECUENCIA AVANZADA ===')
            
            # Añadir obstáculos a la escena
            self.add_table_to_scene()
            
            # Obtener posiciones
            obj_pos = self.object_positions[object_idx]['pose']
            target_pos = self.target_positions[target_idx]['pose']
            
            # Posiciones calculadas
            approach_height = 0.15  # 15cm sobre el objeto
            approach_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + approach_height]
            pick_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + 0.02]  # 2cm sobre el objeto
            
            target_approach_pos = [target_pos[0], target_pos[1], target_pos[2] + approach_height]
            place_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.02]
            
            # Secuencia de movimientos
            movements = [
                (approach_pos, "acercarse al objeto"),
                (pick_pos, "posición de agarre"),
                (approach_pos, "levantar objeto"),
                (target_approach_pos, "acercarse al destino"),
                (place_pos, "posición de colocación"),
                (target_approach_pos, "alejarse del destino")
            ]
            
            # Abrir gripper al inicio
            self.control_gripper_advanced(open_gripper=True)
            
            # Ejecutar movimientos de pick
            for i, (pos, desc) in enumerate(movements[:3]):
                if not self.move_to_pose(pos, desc):
                    self.get_logger().error(f'Error en movimiento: {desc}')
                    return False
                
                # Cerrar gripper después de llegar a posición de agarre
                if i == 1:  # pick_pos
                    time.sleep(0.5)
                    self.control_gripper_advanced(open_gripper=False)
                    time.sleep(1.0)
                    
                    # Verificar agarre
                    if not self.verify_grasp():
                        self.get_logger().warning('Agarre no verificado, continuando...')
            
            # Ejecutar movimientos de place
            for i, (pos, desc) in enumerate(movements[3:]):
                if not self.move_to_pose(pos, desc):
                    self.get_logger().error(f'Error en movimiento: {desc}')
                    return False
                
                # Abrir gripper después de llegar a posición de colocación
                if i == 1:  # place_pos
                    time.sleep(0.5)
                    self.control_gripper_advanced(open_gripper=True)
                    time.sleep(1.0)
            
            # Volver a posición home
            home_pos = [0.0, 0.0, 0.3]
            self.move_to_pose(home_pos, "volver a home")
            
            self.get_logger().info('=== SECUENCIA AVANZADA COMPLETADA ===')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error durante pick and place avanzado: {str(e)}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AdvancedPickAndPlace()
        
        # Esperar a que MoveGroup esté disponible
        node.wait_for_move_group()
        
        # Esperar a que todo se inicialice
        time.sleep(5.0)
        
        # Ejecutar la secuencia avanzada
        success = node.execute_advanced_pick_and_place(object_idx=0, target_idx=0)
        
        if success:
            node.get_logger().info('Pick and place avanzado ejecutado exitosamente')
        else:
            node.get_logger().error('Error durante la ejecución avanzada')
        
        # Mantener el nodo activo
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
