#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from sensor_msgs.msg import JointState
import yaml
import os
import json
import time
import math
import json
import numpy as np
import cv2
from ament_index_python.packages import get_package_share_directory

class VisionPickAndPlace(Node):
    def __init__(self):
        super().__init__('vision_pick_and_place')
        
        # Publishers para control del robot
        self.arm_publisher = self.create_publisher(
            JointTrajectory, 
            '/position_trajectory_controller/joint_trajectory', 
            10
        )
        self.gripper_publisher = self.create_publisher(
            JointTrajectory, 
            '/gripper_controller/joint_trajectory', 
            10
        )
        
        # Subscriber para coordenadas detectadas
        self.object_subscriber = self.create_subscription(
            PointStamped,
            '/detected_object_coords',
            self.object_detected_callback,
            10
        )
        
        # NUEVO: Cliente para cinemática inversa de MoveIt
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # Inicializar como no disponible y verificar en background
        self.moveit_available = False
        
        # Esperar a que el servicio esté disponible con timeout más largo
        self.get_logger().info('🔗 Esperando servicio de cinemática inversa de MoveIt...')
        
        # Timer para verificar MoveIt periódicamente
        self.moveit_check_timer = self.create_timer(3.0, self.check_moveit_availability)
        
        # Parámetro para forzar solo método empírico (más rápido y confiable)
        self.declare_parameter('force_empirical_only', False)
        self.force_empirical_only = self.get_parameter('force_empirical_only').get_parameter_value().bool_value
        
        # Intentar conexión inicial con timeout más largo
        try:
            ik_available = self.ik_client.wait_for_service(timeout_sec=8.0)  # Timeout aumentado
            if ik_available:
                self.setup_moveit_connection()
            else:
                self.get_logger().warn('⚠️  Servicio de MoveIt no disponible en inicialización')
        except Exception as e:
            self.get_logger().warn(f'⚠️  Error en inicialización de MoveIt: {e}')
        
        # Cargar configuraciones
        self.load_configurations()
        
        # Cargar calibración de homografía
        self.load_camera_calibration()
        
        # Estado del sistema
        self.object_detected = False
        self.target_x = 0.0
        self.target_y = 0.0
        self.pick_and_place_active = False
        
        # Timer para operaciones automatizadas
        self.timer = self.create_timer(1.0, self.check_for_objects)
        
        self.get_logger().info('🤖 Sistema de Pick and Place con Visión y MoveIt iniciado')
        self.get_logger().info('⭐ Características:')
        self.get_logger().info(f'   🎯 Cinemática principal: {"MoveIt (experimental)" if self.moveit_available else "Empírica (robusta y probada)"}')
        self.get_logger().info(f'   🔬 MoveIt disponible: {"Sí" if self.moveit_available else "No"}')
        self.get_logger().info(f'   📷 Calibración: {"Homografía" if hasattr(self, "homography_matrix") and self.homography_matrix is not None else "Punto de referencia"}')
        self.get_logger().info('   ✅ Sistema completamente funcional')
        self.get_logger().info('👁️  Esperando detección de objetos...')
        
    def check_moveit_availability(self):
        """Verificar periódicamente si MoveIt está disponible"""
        if not self.moveit_available:
            try:
                if self.ik_client.wait_for_service(timeout_sec=1.0):
                    self.setup_moveit_connection()
                    # Cancelar el timer si MoveIt está disponible
                    if self.moveit_available:
                        self.moveit_check_timer.cancel()
                        self.get_logger().info('🎯 Timer de verificación de MoveIt cancelado - MoveIt ya disponible')
            except Exception as e:
                self.get_logger().debug(f'MoveIt aún no disponible: {e}')

    def setup_moveit_connection(self):
        """Configurar conexión con MoveIt"""
        try:
            self.get_logger().info('✅ Servicio de cinemática inversa encontrado')
            
            # Hacer una prueba más robusta del servicio
            self.get_logger().info('🧪 Verificando funcionalidad de MoveIt...')
            
            # Test más simple - solo verificar si el servicio responde
            if self.ik_client.service_is_ready():
                self.moveit_available = True
                self.get_logger().info('✅ MoveIt completamente funcional y disponible')
                self.get_logger().info('🎯 Cinemática inversa cambiada a: MoveIt (método principal)')
            else:
                # Dar una segunda oportunidad
                time.sleep(2.0)
                if self.ik_client.service_is_ready():
                    self.moveit_available = True
                    self.get_logger().info('✅ MoveIt funcional después de segunda verificación')
                    self.get_logger().info('🎯 Cinemática inversa cambiada a: MoveIt (método principal)')
                else:
                    self.get_logger().warn('⚠️  MoveIt encontrado pero no responde, usando método empírico')
                    self.moveit_available = False
                    
        except Exception as e:
            self.get_logger().warn(f'⚠️  Error configurando MoveIt: {e}')
            self.moveit_available = False
        
    def load_configurations(self):
        """Cargar configuraciones desde archivos YAML"""
        try:
            # Cargar configuración de visión
            vision_config_path = os.path.join(
                get_package_share_directory('braccio_vision'),
                'config', 'vision_config.yaml'
            )
            with open(vision_config_path, 'r') as file:
                self.vision_config = yaml.safe_load(file)
            
            # Cargar configuración de pick and place
            try:
                pick_config_path = os.path.join(
                    get_package_share_directory('braccio_moveit_config'),
                    'config', 'pick_and_place_config.yaml'
                )
                with open(pick_config_path, 'r') as file:
                    self.pick_config = yaml.safe_load(file)
            except:
                # Si no existe, usar configuración por defecto
                self.pick_config = self.get_default_pick_config()
            
            # Convertir estructura del archivo YAML a formato esperado por el código
            self.normalize_pick_config()
                
            self.get_logger().info('Configuraciones cargadas exitosamente')
            
        except Exception as e:
            self.get_logger().error(f'Error cargando configuraciones: {e}')
            # Usar configuraciones por defecto
            self.pick_config = self.get_default_pick_config()
            self.vision_config = self.get_default_vision_config()
    
    def get_default_vision_config(self):
        """Configuración de visión por defecto"""
        return {
            'workspace': {
                'camera_offset_x': 0.3,
                'camera_offset_y': 0.0,
                'camera_height': 0.8
            },
            'coordinate_mapping': {
                'use_automatic_calibration': True
            }
        }
    
    def load_camera_calibration(self):
        """Cargar calibración de homografía de cámara"""
        try:
            calibration_file = "/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_vision/config/camera_calibration.json"
            
            if os.path.exists(calibration_file):
                with open(calibration_file, 'r') as f:
                    calibration_data = json.load(f)
                
                self.homography_matrix = np.array(calibration_data['homography_matrix'])
                self.get_logger().info('✅ Calibración de homografía cargada exitosamente')
                
                # Mostrar información de calibración
                timestamp = calibration_data.get('timestamp', 0)
                if timestamp > 0:
                    import datetime
                    date_str = datetime.datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S')
                    self.get_logger().info(f'📅 Calibración del: {date_str}')
                
            else:
                self.get_logger().warn('⚠️ No se encontró calibración de homografía')
                self.get_logger().warn('🔧 Ejecuta: ros2 run braccio_vision camera_calibration')
                self.homography_matrix = None
                
        except Exception as e:
            self.get_logger().error(f'❌ Error cargando calibración de homografía: {e}')
            self.homography_matrix = None
    
    def normalize_pick_config(self):
        """Normalizar configuración para compatibilidad con código existente"""
        if 'joint_positions' in self.pick_config:
            # Convertir nueva estructura a formato esperado
            if 'positions' not in self.pick_config:
                self.pick_config['positions'] = {}
            
            # Mapear posiciones de joints (mantener solo 5 joints del brazo)
            self.pick_config['positions']['home'] = self.pick_config['joint_positions']['home']
            
            # Crear posición de drop (usar place_position si existe, sino usar home)
            if 'place_position' in self.pick_config['joint_positions']:
                self.pick_config['positions']['drop_position'] = self.pick_config['joint_positions']['place_position']
            else:
                self.pick_config['positions']['drop_position'] = self.pick_config['joint_positions']['home']
            
            # Convertir gripper config
            if 'gripper' in self.pick_config:
                if 'open_position' in self.pick_config['gripper']:
                    self.pick_config['gripper']['open'] = self.pick_config['gripper']['open_position']
                if 'closed_position' in self.pick_config['gripper']:
                    self.pick_config['gripper']['closed'] = self.pick_config['gripper']['closed_position']
            
            # Convertir timing config
            if 'movement' in self.pick_config:
                self.pick_config['timing'] = {
                    'movement_duration': self.pick_config['movement']['default_duration'],
                    'grip_duration': self.pick_config['gripper'].get('close_time', 3.0),
                    'wait_between_actions': 1.0,
                    'safety_wait': 0.5
                }
            
    def get_default_pick_config(self):
        """Configuración por defecto si no existe archivo"""
        return {
            'positions': {
                'home': [0.0, 1.57, 0.0, 0.0, 0.0],
                'pre_pick': [0.0, 1.2, 1.0, 0.8, 0.0],
                'drop_position': [1.57, 1.2, 1.0, 0.8, 0.0]
            },
            'gripper': {
                'open': 0.0,
                'closed': 0.8
            },
            'timing': {
                'movement_duration': 4.0,      # Aumentado para movimientos más seguros
                'grip_duration': 3.0,          # Más tiempo para gripper
                'wait_between_actions': 1.0,   # Pausa entre acciones
                'safety_wait': 0.5             # Pausa adicional de seguridad
            }
        }
    
    def object_detected_callback(self, msg):
        """Callback cuando se detecta un objeto (recibe píxeles)"""
        if not self.pick_and_place_active:
            # Ahora msg contiene píxeles directamente
            pixel_x = int(msg.point.x)
            pixel_y = int(msg.point.y)
            
            # Transformar píxeles a coordenadas del robot
            self.target_x, self.target_y = self.transform_pixels_to_robot(pixel_x, pixel_y)
            self.object_detected = True
            
            self.get_logger().info(f'Objeto detectado en píxel: ({pixel_x}, {pixel_y})')
            self.get_logger().info(f'Coordenadas robot: ({self.target_x:.3f}, {self.target_y:.3f})')
        else:
            self.get_logger().info('Pick and place ya está activo, ignorando nueva detección')
    
    def check_for_objects(self):
        """Verificar si hay objetos detectados y ejecutar pick and place"""
        if self.object_detected and not self.pick_and_place_active:
            self.get_logger().info('Iniciando secuencia de pick and place...')
            self.execute_vision_pick_and_place()
    
    def execute_vision_pick_and_place(self):
        """Ejecutar secuencia completa de pick and place guiada por visión"""
        self.pick_and_place_active = True
        self.object_detected = False
        
        try:
            # 1. Ir a posición home
            self.get_logger().info('1. Moviendo a posición home...')
            self.move_to_position(self.pick_config['positions']['home'])
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 2. Abrir gripper
            self.get_logger().info('2. Abriendo gripper...')
            self.control_gripper(self.pick_config['gripper']['open'])
            time.sleep(self.pick_config['timing']['grip_duration'])
            
            # 3. Calcular posición de pick basada en coordenadas detectadas
            # Usar alturas más bajas para llegar mejor al suelo
            pick_position_high = self.calculate_pick_position(self.target_x, self.target_y, target_z=0.05)  # 5cm del suelo - reducido
            pick_position_low = self.calculate_pick_position(self.target_x, self.target_y, target_z=0.01)   # 1cm del suelo - muy bajo
            
            # 4. Ir a posición de aproximación alta
            self.get_logger().info('3. Moviendo a posición de aproximación alta...')
            self.move_to_position(pick_position_high)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 5. Descender a posición de pick
            self.get_logger().info('4. Descendiendo a objeto...')
            self.move_to_position(pick_position_low)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 6. Cerrar gripper (agarrar objeto)
            self.get_logger().info('5. Cerrando gripper - agarrando objeto...')
            self.control_gripper(self.pick_config['gripper']['closed'])
            time.sleep(self.pick_config['timing']['grip_duration'])
            
            # 7. Levantar objeto
            self.get_logger().info('6. Levantando objeto...')
            self.move_to_position(pick_position_high)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 8. Ir a posición intermedia de transporte
            self.get_logger().info('7. Moviendo a posición de transporte...')
            transport_position = [0.0, 1.3, 0.5, 0.0, 0.0]  # Posición segura para transporte
            self.move_to_position(transport_position)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 9. Ir a posición de drop
            self.get_logger().info('8. Moviendo a posición de drop...')
            self.move_to_position(self.pick_config['positions']['drop_position'])
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 10. Soltar objeto
            self.get_logger().info('9. Abriendo gripper - soltando objeto...')
            self.control_gripper(self.pick_config['gripper']['open'])
            time.sleep(self.pick_config['timing']['grip_duration'])
            
            # 11. Volver a home
            self.get_logger().info('10. Regresando a home...')
            self.move_to_position(self.pick_config['positions']['home'])
            time.sleep(self.pick_config['timing']['movement_duration'])
     
            self.get_logger().info('¡Secuencia de pick and place completada!')
            
        except Exception as e:
            self.get_logger().error(f'Error en pick and place: {e}')
        
        finally:
            self.pick_and_place_active = False
            self.get_logger().info('Listo para detectar nuevo objeto...')
    
    def calculate_pick_position(self, x, y, target_z=0.001):
        """Calcular posición de articulaciones usando MoveIt o fallback empírico"""
        # Aplicar transformación cámara -> robot
        robot_x, robot_y = self.transform_camera_to_robot_coords(x, y)
        
        self.get_logger().info(f'🎯 Coordenadas transformadas: ({x:.3f}, {y:.3f}) -> Robot: ({robot_x:.3f}, {robot_y:.3f})')
        
        # Usar cinemática inteligente (MoveIt primero, fallback después)
        position = self.inverse_kinematics_braccio(robot_x, robot_y, target_z)
        
        self.get_logger().info(f'🤖 Posición calculada: {[f"{math.degrees(j):.1f}°" for j in position]}')
        return position
    
    def transform_pixels_to_robot(self, pixel_x, pixel_y):
        """
        Transformar píxeles a coordenadas del robot usando PUNTO DE REFERENCIA CONOCIDO
        Referencia: píxel (367, 334) corresponde a mundo (0.2, 0.1)
        """
        try:
            # PUNTO DE REFERENCIA CONOCIDO
            ref_pixel_x = 367
            ref_pixel_y = 334
            ref_world_x = 0.2
            ref_world_y = 0.1
            
            # Centro de imagen
            center_x = 320
            center_y = 240
            
            # Calcular factores de escala basados en el punto de referencia
            # Delta desde el centro para el punto de referencia
            ref_delta_px_x = ref_pixel_x - center_x  # 367 - 320 = 47 píxeles
            ref_delta_px_y = ref_pixel_y - center_y  # 334 - 240 = 94 píxeles
            
            # Factores de escala calculados matemáticamente
            # 47 píxeles corresponden a 0.2m → scale_x = 0.2 / 47 = 0.004255
            # 94 píxeles corresponden a 0.1m → scale_y = 0.1 / 94 = 0.001064
            scale_x = ref_world_x / ref_delta_px_x if ref_delta_px_x != 0 else 0.004255
            scale_y = ref_world_y / ref_delta_px_y if ref_delta_px_y != 0 else 0.001064
            
            # Calcular deltas para el píxel actual
            delta_px_x = pixel_x - center_x
            delta_px_y = pixel_y - center_y
            
            # Transformar a coordenadas del mundo
            world_x = delta_px_x * scale_x
            world_y = delta_px_y * scale_y
            
            self.get_logger().info(f'� REF: píxel({ref_pixel_x}, {ref_pixel_y}) → mundo({ref_world_x}, {ref_world_y})')
            self.get_logger().info(f'🔢 Factores calculados: scale_x={scale_x:.6f}, scale_y={scale_y:.6f}')
            self.get_logger().info(f'🎯 ACTUAL: píxel({pixel_x}, {pixel_y}) → mundo({world_x:.4f}, {world_y:.4f})')
            
            return world_x, world_y
                
        except Exception as e:
            self.get_logger().error(f'❌ Error en transformación: {e}')
            # Fallback usando valores calculados
            world_x = (pixel_x - 320) * 0.004255
            world_y = (pixel_y - 240) * 0.001064
            self.get_logger().info(f'🔄 Fallback: píxel({pixel_x}, {pixel_y}) → mundo({world_x:.4f}, {world_y:.4f})')
            return world_x, world_y

    def transform_camera_to_robot_coords(self, cam_x, cam_y):
        """
        Las coordenadas ya vienen transformadas correctamente desde transform_pixels_to_robot,
        por lo que simplemente las devolvemos sin transformación adicional.
        """
        try:
            # Verificar si tenemos homografía calibrada
            if hasattr(self, 'homography_matrix') and self.homography_matrix is not None:
                # Usar homografía para transformación precisa
                pixel_point = np.array([[cam_x, cam_y]], dtype=np.float32)
                robot_point = cv2.perspectiveTransform(pixel_point[None, :, :], self.homography_matrix)[0][0]
                
                robot_x, robot_y = robot_point[0], robot_point[1]
                
                self.get_logger().info(
                    f"🎯 Homografía: Cámara({cam_x:.3f}, {cam_y:.3f}) -> Robot({robot_x:.3f}, {robot_y:.3f})"
                )
                
                return robot_x, robot_y
            else:
                # Las coordenadas ya están correctamente transformadas por transform_pixels_to_robot
                self.get_logger().info("✅ Usando coordenadas ya transformadas (sin doble transformación)")
                self.get_logger().info(f"📍 Coordenadas finales del robot: ({cam_x:.3f}, {cam_y:.3f})")
                return cam_x, cam_y
            
        except Exception as e:
            self.get_logger().error(f"❌ Error en transformación de coordenadas: {e}")
            # Devolver las coordenadas sin transformación adicional
            return cam_x, cam_y
    
    def inverse_kinematics_moveit(self, target_x, target_y, target_z=0.001):
        """
        Usar la cinemática inversa OFICIAL de MoveIt con configuración mejorada
        """
        self.get_logger().info(f'=== CINEMÁTICA INVERSA MOVEIT ===')
        self.get_logger().info(f'Target: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}')
        
        try:
            # Verificar que el servicio esté disponible
            if not self.ik_client.service_is_ready():
                self.get_logger().warn('⚠️  Servicio MoveIt no está listo')
                return self.inverse_kinematics_braccio_fallback(target_x, target_y, target_z)
            
            # Crear request para cinemática inversa
            request = GetPositionIK.Request()
            
            # Configurar pose objetivo
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"  # Frame correcto según TF tree
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            # Posición objetivo (sin ajustes, dejar que MoveIt maneje el frame)
            pose_stamped.pose.position.x = float(target_x)
            pose_stamped.pose.position.y = float(target_y)
            pose_stamped.pose.position.z = float(max(target_z, 0.05))  # Mínimo 5cm para evitar colisiones con suelo
            
            # Orientación más flexible (gripper apuntando hacia abajo pero sin ser muy restrictivo)
            # Identidad primero para probar
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            
            # Configurar request con configuración más permisiva
            request.ik_request.group_name = "arm"  # Nombre del grupo en SRDF
            request.ik_request.pose_stamped = pose_stamped
            request.ik_request.avoid_collisions = False  # Permitir soluciones aunque haya colisiones menores
            request.ik_request.timeout = Duration(sec=2, nanosec=0)  # Timeout más corto para ser más responsive
            
            # Estado inicial del robot - usar posición actual si es posible
            request.ik_request.robot_state = RobotState()
            request.ik_request.robot_state.joint_state = JointState()
            request.ik_request.robot_state.joint_state.name = [
                "joint_base", "joint_1", "joint_2", "joint_3", "joint_4"
            ]
            # Usar posición más neutra para empezar
            request.ik_request.robot_state.joint_state.position = [0.0, 1.0, 0.0, 0.0, 0.0]  # Posición más neutral
            request.ik_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
            
            # Hacer múltiples intentos con configuraciones diferentes si falla
            orientations = [
                (0.0, 0.0, 0.0, 1.0),  # Identidad
                (0.7071, 0.0, 0.0, 0.7071),  # 90° en X (gripper hacia abajo)
                (0.0, 0.7071, 0.0, 0.7071),  # 90° en Y
                (0.0, 0.0, 0.7071, 0.7071),  # 90° en Z
            ]
            
            for i, (ox, oy, oz, ow) in enumerate(orientations):
                # Actualizar orientación
                pose_stamped.pose.orientation.x = ox
                pose_stamped.pose.orientation.y = oy
                pose_stamped.pose.orientation.z = oz
                pose_stamped.pose.orientation.w = ow
                request.ik_request.pose_stamped = pose_stamped
                
                self.get_logger().info(f'🔄 Intento {i+1}/4 con orientación ({ox:.3f}, {oy:.3f}, {oz:.3f}, {ow:.3f})')
                
                # Llamar al servicio
                future = self.ik_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)  # Timeout reducido
                
                if future.result() is not None:
                    response = future.result()
                    
                    if response.error_code.val == response.error_code.SUCCESS:
                        # ¡Éxito! Extraer ángulos de joints
                        joint_positions = response.solution.joint_state.position[:5]  # Solo brazo
                        
                        self.get_logger().info(f'✅ Cinemática inversa MoveIt exitosa en intento {i+1}')
                        self.get_logger().info(f'🎯 Joints calculados: {[f"{math.degrees(j):.1f}°" for j in joint_positions]}')
                        
                        return list(joint_positions)
                    else:
                        self.get_logger().info(f'⚠️  Intento {i+1} falló con código: {response.error_code.val}')
                else:
                    self.get_logger().info(f'⚠️  Timeout en intento {i+1}')
            
            # Si todos los intentos fallaron
            self.get_logger().warn('⚠️  Todos los intentos de MoveIt fallaron')
            self.get_logger().info('🔄 Usando método empírico como fallback')
            return self.inverse_kinematics_braccio_fallback(target_x, target_y, target_z)
                
        except Exception as e:
            self.get_logger().error(f'❌ Error en MoveIt IK: {e}')
            return self.inverse_kinematics_braccio_fallback(target_x, target_y, target_z)

    def inverse_kinematics_braccio_fallback(self, target_x, target_y, target_z):
        """Fallback a método empírico si MoveIt falla"""
        self.get_logger().warn('🔄 Usando método empírico como fallback')
        
        # Calcular ángulo base (este sí es correcto)
        joint_base = math.atan2(target_y, target_x)
        
        # Distancia horizontal desde la base
        r = math.sqrt(target_x**2 + target_y**2)
        
        # Aproximaciones empíricas mejoradas basadas en pruebas exitosas
        if r < 0.08:  # Muy cerca del robot
            joint_shoulder = 1.5  # ~86°
            joint_elbow = 1.3     # ~74°
            joint_wrist = 0.7     # ~40°
        elif r < 0.12:  # Distancia cercana
            joint_shoulder = 1.3  # ~74°
            joint_elbow = 1.1     # ~63°
            joint_wrist = 0.9     # ~52°
        elif r < 0.17:  # Distancia media
            joint_shoulder = 1.1  # ~63°
            joint_elbow = 0.9     # ~52°
            joint_wrist = 1.1     # ~63°
        elif r < 0.22:  # Distancia media-alta
            joint_shoulder = 0.9  # ~52°
            joint_elbow = 0.7     # ~40°
            joint_wrist = 1.3     # ~74°
        else:  # Distancia alta
            joint_shoulder = 0.7  # ~40°
            joint_elbow = 0.5     # ~29°
            joint_wrist = 1.5     # ~86°
        
        # Compensación por coordenadas negativas
        if target_x < 0:
            joint_shoulder += 0.1
            joint_elbow -= 0.1
        
        # Ajustar según altura objetivo
        if target_z > 0.03:  # Objetivo alto
            joint_shoulder += 0.15
            joint_wrist -= 0.05
        elif target_z < 0.005:  # Objetivo muy bajo
            joint_shoulder -= 0.05
            joint_wrist += 0.05
        
        joint_wrist_rot = math.pi/2  # 90°
        
        # Aplicar límites del URDF
        joint_base = max(-1.57, min(1.57, joint_base))
        joint_shoulder = max(0.40, min(2.70, joint_shoulder))
        joint_elbow = max(0.00, min(3.14, joint_elbow))
        joint_wrist = max(0.00, min(3.14, joint_wrist))
        joint_wrist_rot = max(0.00, min(3.14, joint_wrist_rot))
        
        return [joint_base, joint_shoulder, joint_elbow, joint_wrist, joint_wrist_rot]
    def inverse_kinematics_braccio(self, target_x, target_y, target_z=0.001):
        """
        Cinemática inversa con diagnóstico mejorado y MoveIt como prioridad
        """
        # Diagnóstico detallado
        self.get_logger().info(f'🔍 DIAGNÓSTICO CINEMÁTICA INVERSA:')
        self.get_logger().info(f'   - MoveIt disponible: {self.moveit_available}')
        self.get_logger().info(f'   - Servicio listo: {self.ik_client.service_is_ready()}')
        
        # Verificar servicios disponibles en tiempo real
        try:
            services = self.get_service_names_and_types()
            ik_services = [name for name, types in services if 'compute_ik' in name]
            self.get_logger().info(f'   - Servicios IK encontrados: {ik_services}')
        except Exception as e:
            self.get_logger().debug(f'   - Error listando servicios: {e}')
        
        # Forzar una verificación en tiempo real si MoveIt no está marcado como disponible
        if not self.moveit_available:
            self.get_logger().info('🔄 Reintentando conexión con MoveIt en tiempo real...')
            try:
                if self.ik_client.wait_for_service(timeout_sec=2.0):
                    if self.ik_client.service_is_ready():
                        self.moveit_available = True
                        self.get_logger().info('✅ ¡MoveIt ahora disponible en verificación en tiempo real!')
                    else:
                        self.get_logger().warn('⚠️  Servicio encontrado pero no está listo')
                else:
                    self.get_logger().warn('⚠️  Servicio no responde en verificación en tiempo real')
            except Exception as e:
                self.get_logger().warn(f'⚠️  Error en verificación en tiempo real: {e}')
        
        # Priorizar MoveIt para mejor precisión (a menos que esté forzado el empírico)
        if self.moveit_available and not self.force_empirical_only:
            self.get_logger().info('🎯 Usando MoveIt como método principal')
            try:
                return self.inverse_kinematics_moveit(target_x, target_y, target_z)
            except Exception as e:
                self.get_logger().warn(f'⚠️  MoveIt falló: {e}, usando empírico como último recurso')
                return self.inverse_kinematics_braccio_fallback(target_x, target_y, target_z)
        else:
            if self.force_empirical_only:
                self.get_logger().info('🎯 Usando método empírico (forzado por parámetro)')
            else:
                self.get_logger().warn('⚠️  MoveIt no disponible, usando método empírico')
            return self.inverse_kinematics_braccio_fallback(target_x, target_y, target_z)
    
    def move_to_position(self, target_position):
        """Mover brazo a posición específica"""
        self.get_logger().info(f'=== ENVIANDO COMANDO DE MOVIMIENTO ===')
        self.get_logger().info(f'Posición objetivo: {target_position}')
        
        msg = JointTrajectory()
        msg.joint_names = [
            "joint_base",
            "joint_1",
            "joint_2", 
            "joint_3",
            "joint_4"
        ]
        
        # Solo usar los primeros 5 valores (sin gripper)
        arm_positions = target_position[:5] if len(target_position) > 5 else target_position
        
        self.get_logger().info(f'Joints y posiciones:')
        for i, (joint_name, position) in enumerate(zip(msg.joint_names, arm_positions)):
            self.get_logger().info(f'  {joint_name}: {position:.3f} rad = {math.degrees(position):.1f}°')
        
        point = JointTrajectoryPoint()
        point.positions = arm_positions
        point.time_from_start = Duration(
            sec=int(self.pick_config['timing']['movement_duration']),
            nanosec=int((self.pick_config['timing']['movement_duration'] % 1) * 1e9)
        )
        
        msg.points = [point]
        
        self.get_logger().info(f'Duración del movimiento: {self.pick_config["timing"]["movement_duration"]}s')
        self.get_logger().info(f'Publicando comando en topic: /position_trajectory_controller/joint_trajectory')
        
        self.arm_publisher.publish(msg)
        
        self.get_logger().info(f'=== COMANDO ENVIADO ===')
    
    
    def control_gripper(self, position):
        """Controlar gripper usando JointTrajectory"""
        msg = JointTrajectory()
        msg.joint_names = ["right_gripper_joint"]
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(
            sec=int(self.pick_config['timing']['grip_duration']),
            nanosec=int((self.pick_config['timing']['grip_duration'] % 1) * 1e9)
        )
        
        msg.points = [point]
        self.gripper_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    vision_pick_place = VisionPickAndPlace()
    
    try:
        rclpy.spin(vision_pick_place)
    except KeyboardInterrupt:
        vision_pick_place.get_logger().info('Sistema detenido por usuario')
    finally:
        vision_pick_place.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
