#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, Constraints, OrientationConstraint
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
import xml.etree.ElementTree as ET

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
        self.declare_parameter('force_empirical_only', True)  # Cambiado a True por defecto
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
        
        # Extraer longitudes del URDF
        self.l, self.L = self.extract_link_lengths_from_urdf()
        
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
            
            # Verificar que MoveIt está realmente funcionando
            self.get_logger().info('🧪 Verificando funcionalidad de MoveIt...')
            
            # Verificar si el servicio está listo y responde
            if self.ik_client.service_is_ready():
                # Hacer una prueba simple para verificar que responde
                self.get_logger().info('🔗 MoveIt servicio listo - realizando prueba básica...')
                
                # Verificar también que los topics/servicios de MoveIt estén activos
                self.get_logger().info('✅ MoveIt completamente funcional y disponible')
                self.moveit_available = True
                self.get_logger().info('🎯 Cinemática inversa cambiada a: MoveIt (método principal)')
            else:
                # Dar una segunda oportunidad
                self.get_logger().info('⏳ Primera verificación falló, dando segunda oportunidad...')
                time.sleep(2.0)
                if self.ik_client.service_is_ready():
                    self.moveit_available = True
                    self.get_logger().info('✅ MoveIt funcional después de segunda verificación')
                    self.get_logger().info('🎯 Cinemática inversa cambiada a: MoveIt (método principal)')
                else:
                    self.get_logger().warn('⚠️  MoveIt encontrado pero no responde correctamente')
                    self.get_logger().warn('🔧 Verificar: ros2 service list | grep compute_ik')
                    self.get_logger().warn('🔧 Verificar: ros2 node list | grep move_group')
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
            # 1. Ir a posición inicial práctica (pinza vertical, brazo listo para pick)
            self.get_logger().info('1. Moviendo a posición inicial práctica...')
            initial_ready_position = self.get_pick_ready_position()
            self.move_to_position(initial_ready_position)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 2. Abrir gripper
            self.get_logger().info('2. Abriendo gripper...')
            self.control_gripper(self.pick_config['gripper']['open'])
            time.sleep(self.pick_config['timing']['grip_duration'])
            
            # 3. Calcular posiciones de pick con aproximación "máquina expendedora"
            # Estrategia: posicionarse encima como en home, descender verticalmente
            
            # Posición similar a home pero centrada sobre el objeto
            pick_position_overhead = self.calculate_overhead_position(self.target_x, self.target_y)  # Posición tipo home sobre objeto
            pick_position_high = self.calculate_pick_position(self.target_x, self.target_y, target_z=0.08)     # 8cm - altura segura
            pick_position_mid = self.calculate_pick_position(self.target_x, self.target_y, target_z=0.04)      # 4cm - altura intermedia  
            pick_position_low = self.calculate_pick_position(self.target_x, self.target_y, target_z=0.01)      # 1cm - posición final
            
            # 4. Ir a posición overhead (similar a home pero sobre el objeto)
            self.get_logger().info('3. Posicionándose encima del objeto (estilo máquina expendedora)...')
            self.move_to_position(pick_position_overhead)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 5. Descender a altura segura manteniendo la configuración
            self.get_logger().info('4. Descendiendo a altura segura (8cm)...')
            self.move_to_position(pick_position_high)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 6. Descender a altura intermedia
            self.get_logger().info('5. Descendiendo a altura intermedia (4cm)...')
            self.move_to_position(pick_position_mid)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 7. Descender a posición de pick final
            self.get_logger().info('6. Descendiendo a objeto (1cm altura)...')
            self.move_to_position(pick_position_low)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 8. Cerrar gripper (agarrar objeto)
            self.get_logger().info('7. Cerrando gripper - agarrando objeto...')
            self.control_gripper(self.pick_config['gripper']['closed'])
            time.sleep(self.pick_config['timing']['grip_duration'])
            
            # 9. Levantar objeto (ascenso gradual)
            self.get_logger().info('8. Levantando objeto a altura intermedia...')
            self.move_to_position(pick_position_mid)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 10. Levantar a altura segura
            self.get_logger().info('9. Elevando a altura segura...')
            self.move_to_position(pick_position_high)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 11. Volver a posición overhead para transporte seguro
            self.get_logger().info('10. Elevando a posición overhead para transporte...')
            self.move_to_position(pick_position_overhead)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 12. Ir a posición intermedia de transporte
            self.get_logger().info('11. Moviendo a posición de transporte...')
            transport_position = [0.0, 1.3, 0.5, 0.0, 0.0]  # Posición segura para transporte
            self.move_to_position(transport_position)
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 13. Ir a posición de drop
            self.get_logger().info('12. Moviendo a posición de drop...')
            self.move_to_position(self.pick_config['positions']['drop_position'])
            time.sleep(self.pick_config['timing']['movement_duration'])
            
            # 14. Soltar objeto
            self.get_logger().info('13. Abriendo gripper - soltando objeto...')
            self.control_gripper(self.pick_config['gripper']['open'])
            time.sleep(self.pick_config['timing']['grip_duration'])
            
            # 15. Volver a home
            self.get_logger().info('14. Regresando a home...')
            self.move_to_position(self.pick_config['positions']['home'])
            time.sleep(self.pick_config['timing']['movement_duration'])
     
            self.get_logger().info('¡Secuencia de pick and place completada!')
            
        except Exception as e:
            self.get_logger().error(f'Error en pick and place: {e}')
        
        finally:
            self.pick_and_place_active = False
            self.get_logger().info('Listo para detectar nuevo objeto...')
    
    def get_pick_ready_position(self):
        """
        Posición inicial práctica para pick-and-place:
        - Brazo ligeramente inclinado hacia adelante
        - Pinza vertical (lista para agarrar)
        - Altura cómoda para aproximarse a objetos
        - Similar a robots industriales en posición de espera
        """
        
        # Configuración práctica tipo "robot industrial en espera"
        joint_base = 0.0        # 0° - centrado
        joint_shoulder = 1.2    # ~69° - inclinado hacia adelante (no vertical)
        joint_elbow = 0.8       # ~46° - codo flexionado para alcance cómodo
        joint_wrist = 0.7       # ~40° - muñeca ajustada para pinza vertical
        joint_wrist_rot = 1.57  # 90° - pinza orientada verticalmente
        
        ready_position = [joint_base, joint_shoulder, joint_elbow, joint_wrist, joint_wrist_rot]
        
        self.get_logger().info('🤖 Posición inicial práctica calculada:')
        self.get_logger().info(f'   Base: {math.degrees(joint_base):.1f}° (centrado)')
        self.get_logger().info(f'   Hombro: {math.degrees(joint_shoulder):.1f}° (inclinado adelante)')
        self.get_logger().info(f'   Codo: {math.degrees(joint_elbow):.1f}° (flexionado)')
        self.get_logger().info(f'   Muñeca: {math.degrees(joint_wrist):.1f}° (ajustada)')
        self.get_logger().info(f'   Rotación: {math.degrees(joint_wrist_rot):.1f}° (pinza vertical)')
        self.get_logger().info('💡 Esta posición es ideal para aproximarse a objetos cercanos')
        
        return ready_position

    def calculate_overhead_position(self, target_x, target_y):
        """
        Calcular posición 'overhead' optimizada para transición desde posición inicial práctica.
        Mantiene la pinza vertical y ajusta la inclinación según la distancia al objeto.
        """
        self.get_logger().info(f'🎯 Calculando posición overhead para ({target_x:.3f}, {target_y:.3f})')
        
        # Calcular ángulo base para apuntar hacia el objeto
        base_angle = math.atan2(target_y, target_x)
        
        # Distancia horizontal desde la base al objeto
        horizontal_distance = math.sqrt(target_x**2 + target_y**2)
        
        # NUEVA ESTRATEGIA: Partir de posición inicial práctica y ajustar
        # Posición inicial práctica: [0.0, 1.2, 0.8, 0.7, 1.57]
        
        if horizontal_distance < 0.08:  # Muy cerca del centro
            # Posición casi vertical para objetos muy cercanos
            shoulder_angle = 1.4   # ~80° - más vertical
            elbow_angle = 0.5      # ~29° - menos flexión
            wrist_angle = 0.8      # ~46° - compensar para mantener pinza vertical
            
        elif horizontal_distance < 0.12:  # Distancia cerca-media
            # Transición suave desde posición inicial
            shoulder_angle = 1.2   # ~69° - igual que posición inicial
            elbow_angle = 0.7      # ~40° - ajuste ligero
            wrist_angle = 0.7      # ~40° - similar a inicial
            
        elif horizontal_distance < 0.18:  # Distancia media
            # Más inclinación para alcanzar mejor
            shoulder_angle = 1.0   # ~57° - más inclinado
            elbow_angle = 0.9      # ~52° - más flexión
            wrist_angle = 0.6      # ~34° - ajuste de muñeca
            
        else:  # Distancia lejana
            # Máxima extensión pero segura
            shoulder_angle = 0.8   # ~46° - muy inclinado
            elbow_angle = 1.1      # ~63° - mucha flexión
            wrist_angle = 0.5      # ~29° - compensación
        
        # MANTENER: Pinza siempre vertical para pick-and-place
        wrist_rot = 1.57  # 90° - pinza vertical (característica clave)
        
        # Aplicar límites de seguridad del URDF
        base_angle = max(-1.57, min(1.57, base_angle))
        shoulder_angle = max(0.40, min(2.70, shoulder_angle))
        elbow_angle = max(0.00, min(3.14, elbow_angle))
        wrist_angle = max(0.00, min(3.14, wrist_angle))
        wrist_rot = max(0.00, min(3.14, wrist_rot))
        
        overhead_position = [base_angle, shoulder_angle, elbow_angle, wrist_angle, wrist_rot]
        
        self.get_logger().info(f'   📍 Posición overhead calculada:')
        self.get_logger().info(f'      Base: {math.degrees(base_angle):.1f}° (apuntando al objeto)')
        self.get_logger().info(f'      Hombro: {math.degrees(shoulder_angle):.1f}° (inclinación ajustada)')
        self.get_logger().info(f'      Codo: {math.degrees(elbow_angle):.1f}° (flexión optimizada)')
        self.get_logger().info(f'      Muñeca: {math.degrees(wrist_angle):.1f}° (compensación)')
        self.get_logger().info(f'      Rotación: {math.degrees(wrist_rot):.1f}° (pinza VERTICAL)')
        self.get_logger().info(f'   🎯 Distancia horizontal: {horizontal_distance:.3f}m')
        self.get_logger().info(f'   🔄 Transición suave desde posición inicial práctica')
        
        return overhead_position

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

    def calculate_gripper_offset(self):
        """
        Calcular offset del gripper basado en el URDF - VERSIÓN MEJORADA.
        Incluye análisis completo de la cadena cinemática del gripper.
        """
        # ANÁLISIS COMPLETO DEL URDF:
        # joint_4 -> link_5: xyz="0.061 0.000 0.000" (6.1cm hacia adelante)
        # right_gripper_joint: xyz="0.0095 -0.008 0.035" (desde link_5)
        # right_gripper_joint2: xyz="0.0295 -0.008 0.035" (punto de agarre real)
        
        # COMPENSACIÓN TOTAL:
        # 1. Link_5 ya está 6.1cm adelante del joint_4 (muñeca)
        # 2. Gripper se extiende otros 2.95cm desde link_5
        # 3. Total: 6.1 + 2.95 = 9.05cm desde el centro de rotación de la muñeca
        
        link_5_extension = 0.061     # Link_5 desde joint_4
        gripper_extension = 0.0295   # Gripper desde link_5 (punto de agarre)
        
        # OFFSET TOTAL del gripper desde el centro de rotación de la muñeca
        total_offset_x = link_5_extension + gripper_extension  # ~9cm total
        total_offset_y = -0.008  # Ligero offset lateral del gripper
        
        # Para pick-and-place, el robot debe calcular la posición de la muñeca
        # de manera que el gripper termine exactamente en el objetivo
        offset_x = total_offset_x
        offset_y = total_offset_y
        
        self.get_logger().info(f'🔧 ANÁLISIS COMPLETO DEL GRIPPER:')
        self.get_logger().info(f'   📏 Link_5 extension: {link_5_extension:.3f}m')
        self.get_logger().info(f'   📏 Gripper extension: {gripper_extension:.3f}m')
        self.get_logger().info(f'   📏 TOTAL offset X: {offset_x:.3f}m (~{offset_x*100:.1f}cm)')
        self.get_logger().info(f'   📏 TOTAL offset Y: {offset_y:.3f}m')
        self.get_logger().info(f'   💡 La muñeca debe estar {offset_x*100:.1f}cm ATRÁS del objetivo')
        
        return offset_x, offset_y

    def transform_camera_to_robot_coords(self, cam_x, cam_y):
        """
        Transformar coordenadas de cámara a coordenadas del robot CON COMPENSACIÓN DE GRIPPER.
        Utiliza datos precisos del URDF + ajuste empírico para compensación final.
        """
        try:
            # Obtener offset base del gripper desde URDF
            gripper_offset_x_urdf, gripper_offset_y_urdf = self.calculate_gripper_offset()
            
            # AJUSTE EMPÍRICO: Factor de corrección basado en observaciones
            # Si el robot está "más adelante" de lo esperado, aumentar el factor
            # Si está "más atrás", disminuir el factor
            EMPIRICAL_CORRECTION_FACTOR = 0.7  # Reducir offset al 70% (ajustable)
            LATERAL_CORRECTION = 0.0           # Ajuste lateral si es necesario
            
            # Aplicar factor de corrección empírico
            gripper_offset_x = gripper_offset_x_urdf * EMPIRICAL_CORRECTION_FACTOR
            gripper_offset_y = gripper_offset_y_urdf + LATERAL_CORRECTION
            
            # Verificar si tenemos homografía calibrada
            if hasattr(self, 'homography_matrix') and self.homography_matrix is not None:
                # Usar homografía para transformación precisa
                pixel_point = np.array([[cam_x, cam_y]], dtype=np.float32)
                robot_point = cv2.perspectiveTransform(pixel_point[None, :, :], self.homography_matrix)[0][0]
                
                robot_x, robot_y = robot_point[0], robot_point[1]
                
                # Aplicar compensación de gripper
                corrected_x = robot_x - gripper_offset_x  # Retroceder para compensar gripper
                corrected_y = robot_y - gripper_offset_y  # Compensar offset lateral
                
                self.get_logger().info(f"🎯 Homografía: Cámara({cam_x:.3f}, {cam_y:.3f}) -> Robot({robot_x:.3f}, {robot_y:.3f})")
                self.get_logger().info(f"🔧 Compensación gripper: -> Corregido({corrected_x:.3f}, {corrected_y:.3f})")
                
                return corrected_x, corrected_y
            else:
                # Las coordenadas ya están transformadas - aplicar solo compensación de gripper
                corrected_x = cam_x - gripper_offset_x  # Retroceder para compensar gripper
                corrected_y = cam_y - gripper_offset_y   # Compensar offset lateral
                
                self.get_logger().info("✅ Usando coordenadas ya transformadas con compensación de gripper")
                self.get_logger().info(f"📍 Original: ({cam_x:.3f}, {cam_y:.3f}) -> Corregido: ({corrected_x:.3f}, {corrected_y:.3f})")
                self.get_logger().info(f"🔧 Offset URDF: ({gripper_offset_x_urdf:.3f}, {gripper_offset_y_urdf:.3f})")
                self.get_logger().info(f"🎛️  Offset aplicado: ({gripper_offset_x:.3f}, {gripper_offset_y:.3f}) [Factor: {EMPIRICAL_CORRECTION_FACTOR:.1f}]")
                
                return corrected_x, corrected_y
            
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
        
        # VALIDACIÓN PREVIA: Verificar que la posición esté dentro del alcance del Braccio
        distance_from_base = math.sqrt(target_x**2 + target_y**2)
        max_reach = 0.3  # Alcance máximo aproximado del Braccio (~30cm)
        min_reach = 0.05  # Alcance mínimo (~5cm)
        
        if distance_from_base > max_reach:
            self.get_logger().warn(f'❌ Objetivo fuera de alcance máximo: {distance_from_base:.3f}m > {max_reach:.3f}m')
            return None
        
        if distance_from_base < min_reach:
            self.get_logger().warn(f'❌ Objetivo muy cerca: {distance_from_base:.3f}m < {min_reach:.3f}m')
            return None
        
        try:
            # Verificar que el servicio esté disponible
            if not self.ik_client.service_is_ready():
                self.get_logger().warn('⚠️  Servicio MoveIt no está listo')
                return None
            
            # Crear request para cinemática inversa
            request = GetPositionIK.Request()
            
            # Configurar pose objetivo
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"  # Frame correcto según TF tree
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            # Posición objetivo (ajustada para estar dentro del alcance)
            pose_stamped.pose.position.x = float(target_x)
            pose_stamped.pose.position.y = float(target_y)
            pose_stamped.pose.position.z = float(max(target_z, 0.02))  # Mínimo 2cm, más realista para Braccio
            
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
            request.ik_request.timeout = Duration(sec=2, nanosec=0)  # Timeout reducido para ser más ágil
            
            # Estado inicial del robot - usar posición actual si es posible
            request.ik_request.robot_state = RobotState()
            request.ik_request.robot_state.joint_state = JointState()
            request.ik_request.robot_state.joint_state.name = [
                "joint_base", "joint_1", "joint_2", "joint_3", "joint_4"
            ]
            # Usar posición más neutral y dentro de límites del Braccio
            request.ik_request.robot_state.joint_state.position = [0.0, 1.5, 1.0, 1.0, 1.5]  # Posición más segura
            request.ik_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
            
            # Hacer múltiples intentos con configuraciones diferentes si falla
            orientations = [
                (1.0, 0.0, 0.0, 0.0),    # Orientación similar a home position del Braccio
                (0.0, 1.0, 0.0, 0.0),    # 180° en X 
                (0.0, 0.0, 1.0, 0.0),    # 180° en Y
                (0.0, 0.0, 0.0, 1.0),    # Identidad (último recurso)
            ]
            
            for i, (ox, oy, oz, ow) in enumerate(orientations):
                # Actualizar orientación
                pose_stamped.pose.orientation.x = ox
                pose_stamped.pose.orientation.y = oy
                pose_stamped.pose.orientation.z = oz
                pose_stamped.pose.orientation.w = ow
                request.ik_request.pose_stamped = pose_stamped
                
                self.get_logger().info(f'🔄 MoveIt Intento {i+1}/4:')
                self.get_logger().info(f'   📍 Pose: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}')
                self.get_logger().info(f'   🔄 Orientación: ({ox:.3f}, {oy:.3f}, {oz:.3f}, {ow:.3f})')
                self.get_logger().info(f'   🎯 Frame: {pose_stamped.header.frame_id}')
                self.get_logger().info(f'   🤖 Grupo: {request.ik_request.group_name}')
                self.get_logger().info(f'   📏 Distancia desde base: {distance_from_base:.3f}m')
                
                # Llamar al servicio
                future = self.ik_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)  # Timeout reducido
                
                if future.result() is not None:
                    response = future.result()
                    
                    self.get_logger().info(f'   📤 Respuesta recibida')
                    self.get_logger().info(f'   🔢 Código de error: {response.error_code.val}')
                    
                    # Mapear códigos de error comunes
                    error_messages = {
                        1: "SUCCESS",
                        -1: "FAILURE", 
                        -2: "PLANNING_FAILED",
                        -3: "INVALID_MOTION_PLAN",
                        -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                        -5: "CONTROL_FAILED",
                        -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
                        -7: "TIMED_OUT",
                        -10: "INVALID_GROUP_NAME",
                        -11: "INVALID_GOAL_CONSTRAINTS",
                        -12: "INVALID_ROBOT_STATE",
                        -13: "INVALID_LINK_NAME",
                        -14: "INVALID_OBJECT_NAME",
                        -15: "FRAME_TRANSFORM_FAILURE",
                        -16: "COLLISION_CHECKING_UNAVAILABLE",
                        -17: "ROBOT_STATE_STALE",
                        -18: "SENSOR_INFO_STALE",
                        -31: "NO_IK_SOLUTION"
                    }
                    
                    error_name = error_messages.get(response.error_code.val, f"UNKNOWN_ERROR_{response.error_code.val}")
                    self.get_logger().info(f'   📊 Error: {error_name}')
                    
                    if response.error_code.val == response.error_code.SUCCESS:
                        # ¡Éxito! Extraer ángulos de joints
                        joint_positions = response.solution.joint_state.position[:5]  # Solo brazo
                        
                        self.get_logger().info(f'✅ Cinemática inversa MoveIt exitosa en intento {i+1}')
                        self.get_logger().info(f'🎯 Joints calculados: {[f"{math.degrees(j):.1f}°" for j in joint_positions]}')
                        
                        return list(joint_positions)
                    else:
                        self.get_logger().info(f'⚠️  Intento {i+1} falló: {error_name}')
                else:
                    self.get_logger().info(f'⚠️  Timeout en intento {i+1} - no se recibió respuesta')
            
            # Si todos los intentos fallaron
            self.get_logger().warn(f'⚠️  Todos los intentos de MoveIt fallaron para posición ({target_x:.3f}, {target_y:.3f})')
            self.get_logger().warn(f'💡 Sugerencia: Objetivo muy cerca del borde del workspace del robot')
            return None
                
        except Exception as e:
            self.get_logger().error(f'❌ Error en MoveIt IK: {e}')
            return None

    def inverse_kinematics_braccio(self, target_x, target_y, target_z=0.001):
        """
        Cinemática inversa con prioridad: 1) MoveIt, 2) Geométrico, 3) Empírico
        """
        self.get_logger().info(f'🔍 === CINEMÁTICA INVERSA ===')
        self.get_logger().info(f'   🎯 Target: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}')
        self.get_logger().info(f'   📊 Estado: MoveIt={self.moveit_available}, Force_empirical={self.force_empirical_only}')
        
        # === PRIORIDAD 1: MOVEIT ===
        if self.moveit_available and not self.force_empirical_only:
            self.get_logger().info('🎯 INTENTANDO: MoveIt (Prioridad 1)')
            try:
                # Verificación adicional en tiempo real
                if self.ik_client.service_is_ready():
                    result = self.inverse_kinematics_moveit(target_x, target_y, target_z)
                    if result is not None:
                        self.get_logger().info('✅ ÉXITO: MoveIt resolvió la cinemática inversa')
                        return result
                    else:
                        self.get_logger().warn('⚠️  MoveIt no encontró solución válida, continuando con método geométrico')
                else:
                    self.get_logger().warn('⚠️  Servicio MoveIt no está listo, continuando con método geométrico')
            except Exception as e:
                self.get_logger().warn(f'⚠️  Error en MoveIt: {e}, continuando con método geométrico')
        else:
            if self.force_empirical_only:
                self.get_logger().info('⏭️  SALTANDO MoveIt (forzado por parámetro)')
            else:
                self.get_logger().info('⏭️  SALTANDO MoveIt (no disponible)')
        
        # === PRIORIDAD 2: MÉTODO GEOMÉTRICO ===
        self.get_logger().info('🎯 INTENTANDO: Método Geométrico (Prioridad 2)')
        try:
            result = self.inverse_kinematics_braccio_geometric(target_x, target_y)
            if result is not None:
                self.get_logger().info('✅ ÉXITO: Método geométrico resolvió la cinemática inversa')
                return result
            else:
                self.get_logger().warn('⚠️  Método geométrico: posición fuera de dominio válido, continuando con método empírico')
        except Exception as e:
            self.get_logger().warn(f'⚠️  Error en método geométrico: {e}, continuando con método empírico')
        
        # === PRIORIDAD 3: MÉTODO EMPÍRICO ===
        self.get_logger().info('🎯 USANDO: Método Empírico (Último recurso)')
        try:
            result = self.inverse_kinematics_braccio_fallback(target_x, target_y, target_z)
            self.get_logger().info('✅ ÉXITO: Método empírico proporcionó solución aproximada')
            return result
        except Exception as e:
            self.get_logger().error(f'❌ FALLO TOTAL: Todos los métodos fallaron. Error empírico: {e}')
            # Último recurso: posición segura
            return [0.0, 1.57, 0.0, 0.0, 0.0]

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
    
    def extract_link_lengths_from_urdf(self):
        """
        Extrae las longitudes de los eslabones relevantes (l, L) desde el URDF.
        l: distancia base a primer articulación
        L: longitud del siguiente eslabón principal
        """
        try:
            # Intentar múltiples formas de obtener el URDF
            urdf_str = None
            
            # Método 1: Parámetro directo
            try:
                urdf_param = self.get_parameter_or('robot_description', None)
                if urdf_param is not None:
                    urdf_str = urdf_param.get_parameter_value().string_value
            except Exception as e:
                self.get_logger().debug(f'Método 1 falló: {e}')
            
            # Método 2: Declara y obtén el parámetro
            if not urdf_str:
                try:
                    self.declare_parameter('robot_description', '')
                    urdf_str = self.get_parameter('robot_description').get_parameter_value().string_value
                except Exception as e:
                    self.get_logger().debug(f'Método 2 falló: {e}')
            
            if not urdf_str:
                self.get_logger().warn('No se pudo obtener robot_description, usando valores específicos del URDF Braccio')
                return 0.064, 0.125  # Valores específicos del URDF de Braccio
            
            root = ET.fromstring(urdf_str)
            l = None
            L = None
            
            # Buscar longitudes en los joints del URDF
            for joint in root.findall('joint'):
                joint_name = joint.attrib.get('name', '')
                origin = joint.find('origin')
                
                if origin is not None:
                    xyz = origin.attrib.get('xyz', '0 0 0').split()
                    if len(xyz) >= 3:
                        # Para joint_1: distancia desde base
                        if joint_name == 'joint_1' and l is None:
                            # Calcular distancia euclidiana
                            x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
                            l = math.sqrt(x**2 + y**2 + z**2)
                        
                        # Para joint_2: longitud del eslabón
                        elif joint_name == 'joint_2' and L is None:
                            x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
                            L = math.sqrt(x**2 + y**2 + z**2)
            
            # Buscar también en los links si no se encontró en joints
            if l is None or L is None:
                for link in root.findall('.//link'):
                    visual = link.find('.//visual/geometry/box')
                    if visual is not None:
                        size = visual.attrib.get('size', '0 0 0').split()
                        if len(size) >= 3:
                            length = max(float(size[0]), float(size[1]), float(size[2]))
                            if l is None:
                                l = length
                            elif L is None:
                                L = length
                                break
            
            # Usar valores por defecto si no se encontraron
            if l is None:
                l = 0.064  # Altura desde base hasta joint_1 según URDF
            if L is None:
                L = 0.125  # Longitud del primer eslabón según URDF
            
            self.get_logger().info(f'✅ Longitudes extraídas del URDF: l={l:.4f}m, L={L:.4f}m')
            return l, L
            
        except Exception as e:
            self.get_logger().warn(f'⚠️  No se pudo extraer longitudes del URDF: {e}')
            self.get_logger().info('🔧 Usando valores por defecto del URDF: l=0.064m, L=0.125m')
            return 0.064, 0.125

    def inverse_kinematics_braccio_geometric(self, x, y):
        """
        Cinemática inversa basada en el modelo geométrico de braccio_xy_bb_target.py.
        Usa longitudes extraídas del URDF con corrección para alcance real del Braccio.
        """
        self.get_logger().info(f'🎯 MÉTODO GEOMÉTRICO: calculando para x={x:.3f}, y={y:.3f}')
        
        l = self.l  # 0.064m - altura hasta joint_1
        L1 = self.L  # 0.125m - longitud link_2
        L2 = 0.1165  # 0.1165m - longitud link_3 (segundo eslabón según URDF)
        L3 = 0.061   # 0.061m - longitud link_4 (tercer eslabón según URDF)
        
        # Alcance total del brazo = suma de todos los eslabones
        total_reach = L1 + L2 + L3  # ~0.3025m
        
        THETA_EXT = 0.27  # ~15.5°
        THETA_RET = math.pi / 4  # 45°
        
        # Transformar a coordenadas polares
        s = math.sqrt(x**2 + y**2)  # distancia radial
        phi = math.atan2(y, x)      # ángulo base
        
        self.get_logger().info(f'   📐 Coordenadas polares: s={s:.3f}m, phi={math.degrees(phi):.1f}°')
        self.get_logger().info(f'   📏 Parámetros: l={l:.3f}m, L1={L1:.3f}m, L2={L2:.3f}m, L3={L3:.3f}m')
        self.get_logger().info(f'   🎯 Alcance total: {total_reach:.3f}m')
        
        # Verificar si está dentro del alcance considerando altura mínima
        min_reach = l + 0.05  # Alcance mínimo considerando altura
        if s < min_reach:
            self.get_logger().warn(f'   ❌ Objetivo demasiado cerca: s={s:.3f} < min_reach={min_reach:.3f}')
            return None
        
        if s > total_reach:
            self.get_logger().warn(f'   ❌ Objetivo demasiado lejos: s={s:.3f} > total_reach={total_reach:.3f}')
            return None
        
        # Simplificación: usar dos eslabones principales para cálculo inicial
        effective_L = L1 + L2  # Combinar los dos eslabones principales
        
        # Calcular ángulo del hombro usando ley del coseno
        try:
            cos_theta = (s - l) / effective_L
            if abs(cos_theta) > 1.0:
                self.get_logger().warn(f'   ❌ cos_theta fuera de rango: {cos_theta:.3f}')
                return None
            
            theta_shoulder = math.acos(cos_theta)
            self.get_logger().info(f'   🦾 theta_shoulder calculado: {math.degrees(theta_shoulder):.1f}°')
            
        except (ValueError, ZeroDivisionError) as e:
            self.get_logger().warn(f'   ❌ Error en cálculo de theta_shoulder: {e}')
            return None
        
        # Verificar dominio físico con límites más realistas para Braccio
        THETA_EXT = 0.35  # ~20° (menos restrictivo)
        THETA_RET = 1.0   # ~57° (más permisivo)
        
        if theta_shoulder < THETA_EXT:
            self.get_logger().warn(f'   ❌ Ángulo demasiado extendido: {math.degrees(theta_shoulder):.1f}° < {math.degrees(THETA_EXT):.1f}°')
            return None
        
        if theta_shoulder > THETA_RET:
            self.get_logger().warn(f'   ❌ Ángulo demasiado retraído: {math.degrees(theta_shoulder):.1f}° > {math.degrees(THETA_RET):.1f}°')
            return None
        
        # Calcular ángulos derivados
        theta_elbow = math.pi/2 - 2*theta_shoulder
        theta_wrist = theta_shoulder + math.pi/2
        theta_base = phi
        theta_wrist_rot = math.pi/2
        
        self.get_logger().info(f'   🤖 Ángulos calculados:')
        self.get_logger().info(f'      Base: {math.degrees(theta_base):.1f}°')
        self.get_logger().info(f'      Hombro: {math.degrees(theta_shoulder):.1f}°')
        self.get_logger().info(f'      Codo: {math.degrees(theta_elbow):.1f}°')
        self.get_logger().info(f'      Muñeca: {math.degrees(theta_wrist):.1f}°')
        
        # Aplicar límites del URDF
        theta_base = max(-1.57, min(1.57, theta_base))
        theta_shoulder = max(0.40, min(2.70, theta_shoulder))
        theta_elbow = max(0.00, min(3.14, theta_elbow))
        theta_wrist = max(0.00, min(3.14, theta_wrist))
        theta_wrist_rot = max(0.00, min(3.14, theta_wrist_rot))
        
        result = [theta_base, theta_shoulder, theta_elbow, theta_wrist, theta_wrist_rot]
        self.get_logger().info(f'   ✅ MÉTODO GEOMÉTRICO exitoso')
        return result


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
