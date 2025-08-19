#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from linkattacher_msgs.srv import AttachLink, DetachLink
from gazebo_msgs.msg import ModelStates
import tf_transformations
from tf2_ros import Buffer, TransformListener
import math

class GamepadTeleopSimple(Node):
    def __init__(self):
        super().__init__('gamepad_teleop_simple')
        self.current_joint_positions = [1.573, 0.763, 3.142, 3.127, 1.572, 0.0]
        self.get_logger().info('🎮 Gamepad Teleop Simple iniciado')
        self.get_logger().info(f'📐 Posiciones iniciales (estado real): {self.current_joint_positions}')
        self.joy_subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/position_trajectory_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        self.get_logger().info('✅ Publisher configurado usando método pick_and_place')
        self.get_logger().info('🎮 Usa el stick izquierdo para controlar base y joint_1')
        self.get_logger().info('🎮 Usa el stick derecho para controlar joint_2 y joint_3')
        self.get_logger().info('🎮 Usa L1/R1 para controlar joint_4')
        self.get_logger().info('⭕ Usa Círculo para pick and place (attach)')
        self.get_logger().info('❌ Usa X para detach (soltar cubo)')
        self.get_logger().info('📍 Modo: Incremental desde estado real del robot')

        # Para almacenar posiciones de modelos
        self.model_states = None
        self.model_states_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_states_callback, 10)

        # Nombre del link del gripper (frame TF)
        self.gripper_link_name = 'right_gripper_link'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def model_states_callback(self, msg):
        self.model_states = msg

    def joy_callback(self, msg):
        deadzone = 0.1
        left_x = msg.axes[0] if abs(msg.axes[0]) > deadzone else 0.0
        left_y = msg.axes[1] if abs(msg.axes[1]) > deadzone else 0.0
        right_x = msg.axes[3] if abs(msg.axes[3]) > deadzone else 0.0
        right_y = msg.axes[4] if abs(msg.axes[4]) > deadzone else 0.0
        joint_4_input = 0.0
        if len(msg.buttons) > 5:
            if msg.buttons[4]: joint_4_input = 1.0
            if msg.buttons[5]: joint_4_input = -1.0
        gripper_input = 0.0
        if len(msg.buttons) > 7:
            if msg.buttons[6]: gripper_input = 1.0
            if msg.buttons[7]: gripper_input = -1.0
        # Pick basado en proximidad (sin requerir contacto físico)
        if len(msg.buttons) > 2 and msg.buttons[2]:
            model, link = self.get_closest_object_to_gripper()
            if model:
                # Verificar que el objeto esté lo suficientemente cerca (umbral de proximidad)
                try:
                    trans = self.tf_buffer.lookup_transform('world', self.gripper_link_name, rclpy.time.Time())
                    gripper_pos = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
                    
                    # Buscar la posición del objeto más cercano
                    for i, name in enumerate(self.model_states.name):
                        if name == model:
                            obj_pose = self.model_states.pose[i]
                            obj_pos = (obj_pose.position.x, obj_pose.position.y, obj_pose.position.z)
                            dist = math.sqrt(sum((a-b)**2 for a, b in zip(gripper_pos, obj_pos)))
                            
                            # Umbral de proximidad para pick (15cm)
                            if dist <= 0.15:
                                self.get_logger().info(f'🎯 Pick por proximidad: {model} (distancia: {dist:.3f}m)')
                                self.call_attach_service(model, link)
                            else:
                                self.get_logger().warn(f'❌ Objeto {model} demasiado lejos para pick: {dist:.3f}m (máx: 0.15m)')
                            break
                except Exception as e:
                    self.get_logger().warn(f'Error calculando proximidad para pick: {e}')
            else:
                self.get_logger().warn('No se encontró objeto cercano para pick')
        if len(msg.buttons) > 0 and msg.buttons[0]:
            model, link = self.get_closest_object_to_gripper()
            if model:
                self.get_logger().info(f'Modelo más cercano para detach: {model}, link: {link}')
                self.call_detach_service(model, link)
            else:
                self.get_logger().warn('No se encontró objeto cercano para detach')

        # Movimiento del brazo y gripper siempre funciona
        if any(abs(x) > 0 for x in [left_x, left_y, right_x, right_y, joint_4_input, gripper_input]):
            velocity_factor = 0.01
            self.current_joint_positions[0] += left_x * velocity_factor
            self.current_joint_positions[1] += left_y * velocity_factor
            self.current_joint_positions[2] += right_y * velocity_factor
            self.current_joint_positions[3] += right_x * velocity_factor
            self.current_joint_positions[4] += joint_4_input * velocity_factor
            self.current_joint_positions[5] += gripper_input * velocity_factor
            self.apply_joint_limits()
            self.send_arm_trajectory_command()
            self.send_gripper_trajectory_command()
            self.get_logger().info(
                f'🎮 Input: LX={left_x:.2f} LY={left_y:.2f} RX={right_x:.2f} RY={right_y:.2f} J4={joint_4_input:.2f} G={gripper_input:.2f} → Pos: {self.current_joint_positions}')

    def get_closest_object_to_gripper(self):
        # Espera a tener datos de modelos SOLO para pick/detach
        if self.model_states is None:
            self.get_logger().warn('model_states aún no disponible')
            return None, None
        self.get_logger().info(f'Modelos en model_states: {self.model_states.name}')
        # Obtén la posición del gripper usando TF
        try:
            trans = self.tf_buffer.lookup_transform('world', self.gripper_link_name, rclpy.time.Time())
            gripper_pos = (
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            )
            self.get_logger().info(f'Posición gripper TF: {gripper_pos}')
        except Exception as e:
            self.get_logger().warn(f'No se pudo obtener la posición del gripper por TF: {e}')
            return None, None
        min_dist = float('inf')
        closest_model = None
        for i, name in enumerate(self.model_states.name):
            if name in ['braccio', 'ground_plane']:
                continue
            obj_pose = self.model_states.pose[i]
            obj_pos = (obj_pose.position.x, obj_pose.position.y, obj_pose.position.z)
            self.get_logger().info(f'Comparando con modelo: {name}, posición: {obj_pos}')
            dist = math.sqrt(sum((a-b)**2 for a, b in zip(gripper_pos, obj_pos)))
            self.get_logger().info(f'Distancia a {name}: {dist}')
            if dist < min_dist:
                min_dist = dist
                closest_model = name
        self.get_logger().info(f'Modelo más cercano: {closest_model}, distancia: {min_dist}')
        return closest_model, 'link'

    def call_attach_service(self, model2_name, link2_name):
        if not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Servicio /ATTACHLINK no disponible')
            return
        req = AttachLink.Request()
        req.model1_name = 'braccio'
        req.link1_name = 'right_gripper_link'
        req.model2_name = model2_name
        req.link2_name = link2_name
        future = self.attach_client.call_async(req)
        self.get_logger().info(f'📎 Intentando attach: {model2_name}/{link2_name}')

    def call_detach_service(self, model2_name, link2_name):
        if not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Servicio /DETACHLINK no disponible')
            return
        req = DetachLink.Request()
        req.model1_name = 'braccio'
        req.link1_name = 'right_gripper_link'
        req.model2_name = model2_name
        req.link2_name = link2_name
        future = self.detach_client.call_async(req)
        self.get_logger().info(f'🔗 Intentando detach: {model2_name}/{link2_name}')

    def apply_joint_limits(self):
        min_limits = [0, 0.4, 0.0, 0.0, 0.0, 0.1]
        max_limits = [3.14, 2.7, 3.14, 3.14, 3.14, 1.0]
        self.current_joint_positions = [max(min(val, max_limits[i]), min_limits[i]) for i, val in enumerate(self.current_joint_positions)]

    def send_arm_trajectory_command(self):
        traj = JointTrajectory()
        traj.joint_names = ['joint_base', 'joint_1', 'joint_2', 'joint_3', 'joint_4']
        point = JointTrajectoryPoint()
        point.positions = self.current_joint_positions[:5]
        point.time_from_start = Duration(sec=0, nanosec=50000000)
        traj.points.append(point)
        self.arm_pub.publish(traj)

    def send_gripper_trajectory_command(self):
        traj = JointTrajectory()
        traj.joint_names = ['right_gripper_joint']
        point = JointTrajectoryPoint()
        point.positions = [self.current_joint_positions[5]]
        point.time_from_start = Duration(sec=0, nanosec=20000000)
        traj.points.append(point)
        self.gripper_pub.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = GamepadTeleopSimple()
    node.get_logger().info('🚀 Iniciando teleoperación simple con gamepad...')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
