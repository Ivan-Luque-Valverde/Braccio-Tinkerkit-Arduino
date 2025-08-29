#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # Cargar configuración
        self.load_config()
        
        # Bridge para convertir imágenes
        self.bridge = CvBridge()
        
        # Variables de estado
        self.camera_info = None
        self.latest_image = None
        
        # Lista temporal de coordenadas bloqueadas (para evitar re-detección inmediata)
        self.blocked_coordinates = {}  # {(x,y): timestamp}
        
        # Subscribers
        self.image_subscriber = self.create_subscription(
            Image,
            self.config['camera']['topic_image'],
            self.image_callback,
            10
        )
        
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            self.config['camera']['topic_camera_info'],
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.debug_image_publisher = self.create_publisher(
            Image,
            self.config['publishing']['debug_image_topic'],
            10
        )
        
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            self.config['publishing']['object_markers_topic'],
            10
        )
        
        # Publishers para coordenadas del objeto detectado (CRÍTICO para pick and place)
        self.coords_publisher = self.create_publisher(
            PointStamped,
            '/detected_object_coords',
            10
        )
        
        self.vision_coords_publisher = self.create_publisher(
            PointStamped,
            '/vision/detected_objects',
            10
        )
        
        # Timer para procesamiento
        detection_rate = self.config['publishing']['detection_rate']
        self.detection_timer = self.create_timer(
            1.0 / detection_rate,
            self.process_detection
        )
        
        self.get_logger().info('🔍 Object Detector iniciado')
        self.get_logger().info(f'📷 Escuchando imagen en: {self.config["camera"]["topic_image"]}')

    def load_config(self):
        """Carga la configuración desde el archivo YAML"""
        try:
            pkg_share = get_package_share_directory('braccio_vision')
            config_path = os.path.join(pkg_share, 'config', 'vision_config.yaml')
            
            if not os.path.exists(config_path):
                config_path = '/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_vision/config/vision_config.yaml'
            
            with open(config_path, 'r') as file:
                self.config = yaml.safe_load(file)
            
            self.get_logger().info(f'✅ Configuración cargada desde: {config_path}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error cargando configuración: {str(e)}')
            # Configuración mínima por defecto
            self.config = self.get_default_config()

    def get_default_config(self):
        """Configuración por defecto"""
        return {
            'camera': {
            'topic_image': '/overhead_camera/image_raw',
            'topic_camera_info': '/overhead_camera/camera_info'
            },
            'publishing': {
            'debug_image_topic': '/vision/debug_image',
            'object_markers_topic': '/vision/object_markers',
            'detection_rate': 10.0
            },
            'object_detection': {
            'red_cube': {
                'enabled': True,
                'hsv_lower': [0, 100, 70], # Rojo típico
                'hsv_upper': [10, 255, 255],
                'min_area': 100,
                'max_area': 5000
            },
            'green_cube': {
                'enabled': True,
                'hsv_lower': [40, 40, 40],  # Verde típico
                'hsv_upper': [85, 255, 255],
                'min_area': 100,
                'max_area': 5000
            },
            },
            'workspace': {
            'camera_height': 1.0
            },
            'debug': {
            'show_mask': True
            }
        }

    def image_callback(self, msg):
        """Callback para recibir imágenes"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error convertir imagen: {str(e)}')

    def camera_info_callback(self, msg):
        """Callback para información de la cámara"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('📷 Información de cámara recibida')

    def detect_objects_by_color(self, image):
        """Detecta objetos por color HSV"""
        detected_objects = []
        # Convertir a HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Crear imagen de debug
        debug_image = image.copy()
        show_mask = self.config.get('debug', {}).get('show_mask', False)
        # Detectar cada tipo de objeto configurado
        for obj_name, obj_config in self.config['object_detection'].items():
            if not obj_config.get('enabled', False):
                continue
            # Crear máscara de color
            lower = np.array(obj_config['hsv_lower'])
            upper = np.array(obj_config['hsv_upper'])
            # Para el rojo, necesitamos dos rangos debido a la naturaleza circular del canal H
            if 'red' in obj_name:
                mask1 = cv2.inRange(hsv, np.array([0, 100, 50]), np.array([15, 255, 255]))
                mask2 = cv2.inRange(hsv, np.array([165, 100, 50]), np.array([180, 255, 255]))
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv, lower, upper)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            if show_mask:
                mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                debug_image = cv2.addWeighted(debug_image, 0.7, mask_bgr, 0.3, 0)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                # Filtrar por área
                if (area >= obj_config['min_area'] and area <= obj_config['max_area']):
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        detected_objects.append({
                            'name': obj_name,
                            'pixel_x': cx,
                            'pixel_y': cy,
                            'area': area,
                            'contour': contour
                        })
        # --- DIBUJAR SOLO LOS OBJETOS FILTRADOS POR ÁREA EN LA IMAGEN DE DEBUG ---
        # El filtrado por área adicional se hace en process_detection, así que aquí solo dibujamos los que nos pasen
        # Esta función se usará en process_detection después del filtrado
        return detected_objects, debug_image

    def pixel_to_world(self, pixel_x, pixel_y):
        """Convierte coordenadas de píxel a coordenadas del mundo"""
        if self.camera_info is None:
            # Usar valores por defecto
            cx = self.config['camera']['width'] / 2
            cy = self.config['camera']['height'] / 2
            fx = 381.346          # Focal length x (calculado: width/2 / tan(FOV/2))
            fy = 381.346          # Focal length y (igual a fx para píxeles cuadrados)
        else:
            # Usar parámetros de la cámara
            cx = self.camera_info.k[2]  # Principal point x
            cy = self.camera_info.k[5]  # Principal point y
            fx = self.camera_info.k[0]  # Focal length x
            fy = self.camera_info.k[4]  # Focal length y
        
        # Calcular coordenadas del mundo usando geometría de cámara cenital
        camera_height = self.config['workspace']['camera_height']
        
        # Coordenadas normalizadas
        x_norm = (pixel_x - cx) / fx
        y_norm = (pixel_y - cy) / fy
        
        # Proyección al plano Z=0 (mesa)
        world_x = x_norm * camera_height
        world_y = y_norm * camera_height
        
        return world_x, world_y

    def create_object_markers(self, detected_objects):
        """Crea markers de visualización para los objetos detectados"""
        marker_array = MarkerArray()
        for i, obj in enumerate(detected_objects):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            # Convertir píxel a mundo para la posición
            world_x, world_y = self.pixel_to_world(obj['pixel_x'], obj['pixel_y'])
            marker.pose.position.x = world_x
            marker.pose.position.y = world_y
            marker.pose.position.z = 0.025  # Altura del marker
            marker.pose.orientation.w = 1.0
            # Tamaño
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            # Color según el tipo de objeto
            if 'red' in obj['name']:
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            elif 'green' in obj['name']:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            elif 'blue' in obj['name']:
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
            else:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
            marker.lifetime.sec = 1  # Duración del marker
            marker_array.markers.append(marker)
        return marker_array

    def process_detection(self):
        """Procesa la detección de objetos"""
        if self.latest_image is None:
            return
        try:
            # Detectar objetos
            detected_objects, _ = self.detect_objects_by_color(self.latest_image)
            filtered_objects = []
            ideal_area = 250  # área ideal en píxeles para tu cubo (ajusta según tu imagen)
            area_tolerance = 0.5  # 50% de tolerancia
            for obj in detected_objects:
                if ideal_area * (1 - area_tolerance) < obj['area'] < ideal_area * (1 + area_tolerance):
                    filtered_objects.append(obj)
            detected_objects = filtered_objects
            # --- DIBUJAR SOLO LOS OBJETOS FILTRADOS EN LA IMAGEN DE DEBUG ---
            debug_image = self.latest_image.copy()
            for obj in detected_objects:
                contour = obj['contour']
                cx = obj['pixel_x']
                cy = obj['pixel_y']
                name = obj['name']
                cv2.drawContours(debug_image, [contour], -1, (0, 0, 0), 2)
                if 'red' in name:
                    cv2.circle(debug_image, (cx, cy), 2, (0, 0, 0), -1)
                    cv2.putText(debug_image, f'({cx}, {cy})', (cx - 30, cy + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)
                elif 'green' in name:
                    cv2.circle(debug_image, (cx, cy), 2, (0, 0 , 0), -1)
                    cv2.putText(debug_image, f'({cx}, {cy})', (cx - 30, cy + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
                elif 'blue' in name:
                    cv2.circle(debug_image, (cx, cy), 2, (0, 0, 0), -1)
                    cv2.putText(debug_image, f'({cx}, {cy})', (cx - 30, cy + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)
                else:
                    cv2.circle(debug_image, (cx, cy), 2, (0, 0, 0), -1)
                    cv2.putText(debug_image, f'{name}', (cx - 50, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    cv2.putText(debug_image, f'({cx}, {cy})', (cx - 30, cy + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 2)
            
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = self.config['camera']['frame_id']
            self.debug_image_publisher.publish(debug_msg)
            # Debug: contar objetos detectados
            self.get_logger().info(f'🔍 Objetos detectados: {len(detected_objects) if detected_objects else 0}')
            # Publicar markers para todos los objetos
            if detected_objects:
                self.get_logger().info(f'📤 Publicando markers para {len(detected_objects)} objetos')
                markers = self.create_object_markers(detected_objects)
                self.marker_publisher.publish(markers)
                
                # Log específico para cubos rojos, verdes y azules
                red_objs = [obj for obj in detected_objects if 'red' in obj['name']]
                green_objs = [obj for obj in detected_objects if 'green' in obj['name']]
                blue_objs = [obj for obj in detected_objects if 'blue' in obj['name']]

                # Mostrar coordenadas de cubos rojos
                if red_objs:
                    self.get_logger().info(f'🔴 CUBOS ROJOS DETECTADOS: {len(red_objs)}')
                    for i, red_obj in enumerate(red_objs):
                        self.get_logger().info(f'   🎯 Cubo rojo {i+1}: píxel({red_obj["pixel_x"]}, {red_obj["pixel_y"]})')

                # Mostrar coordenadas de cubos verdes
                if green_objs:
                    self.get_logger().info(f'🟢 CUBOS VERDES DETECTADOS: {len(green_objs)}')
                    for i, green_obj in enumerate(green_objs):
                        self.get_logger().info(f'   🎯 Cubo verde {i+1}: píxel({green_obj["pixel_x"]}, {green_obj["pixel_y"]})')

                # Mostrar coordenadas de cubos azules
                if blue_objs:
                    self.get_logger().info(f'🔵 CUBOS AZULES DETECTADOS: {len(blue_objs)}')
                    for i, blue_obj in enumerate(blue_objs):
                        self.get_logger().info(f'   🎯 Cubo azul {i+1}: píxel({blue_obj["pixel_x"]}, {blue_obj["pixel_y"]})')
                
                # Publicar coordenadas de UN SOLO cubo a la vez para evitar conflictos
                # Prioridad: verde primero, luego azul
                published_coords = None
                
                # Limpiar coordenadas bloqueadas antiguas (más de 30 segundos)
                import time
                current_time = time.time()
                self.blocked_coordinates = {
                    coord: timestamp for coord, timestamp in self.blocked_coordinates.items()
                    if current_time - timestamp < 30.0
                }
                
                # Procesar TODOS los cubos verdes primero
                green_published = False
                if green_objs:
                    for green_obj in green_objs:
                        coord_key = (green_obj['pixel_x'], green_obj['pixel_y'])
                        
                        # Verificar si estas coordenadas están bloqueadas
                        if coord_key not in self.blocked_coordinates:
                            coords_msg = PointStamped()
                            coords_msg.header.stamp = self.get_clock().now().to_msg()
                            coords_msg.header.frame_id = "overhead_camera"
                            coords_msg.point.x = float(green_obj['pixel_x'])
                            coords_msg.point.y = float(green_obj['pixel_y'])
                            coords_msg.point.z = 0.0
                            self.coords_publisher.publish(coords_msg)
                            self.vision_coords_publisher.publish(coords_msg)
                            published_coords = f"verde píxel({green_obj['pixel_x']}, {green_obj['pixel_y']})"
                            self.get_logger().info(f'📤 Publicado para pick and place: {published_coords}')
                            
                            # Bloquear estas coordenadas por 10 segundos
                            self.blocked_coordinates[coord_key] = current_time
                            green_published = True
                            break  # Solo publicar uno a la vez
                        else:
                            self.get_logger().info(f'⏳ Cubo verde en píxel({green_obj["pixel_x"]}, {green_obj["pixel_y"]}) temporalmente bloqueado')
                
                # Solo procesar azules si no se publicó ningún verde
                if not green_published and blue_objs:
                    for blue_obj in blue_objs:
                        coord_key = (blue_obj['pixel_x'], blue_obj['pixel_y'])
                        
                        # Verificar si estas coordenadas están bloqueadas
                        if coord_key not in self.blocked_coordinates:
                            coords_msg = PointStamped()
                            coords_msg.header.stamp = self.get_clock().now().to_msg()
                            coords_msg.header.frame_id = "overhead_camera"
                            coords_msg.point.x = float(blue_obj['pixel_x'])
                            coords_msg.point.y = float(blue_obj['pixel_y'])
                            coords_msg.point.z = 0.0
                            self.coords_publisher.publish(coords_msg)
                            self.vision_coords_publisher.publish(coords_msg)
                            published_coords = f"azul píxel({blue_obj['pixel_x']}, {blue_obj['pixel_y']})"
                            self.get_logger().info(f'📤 Publicado para pick and place: {published_coords}')
                            
                            # Bloquear estas coordenadas por 10 segundos
                            self.blocked_coordinates[coord_key] = current_time
                            break  # Solo publicar uno a la vez
                        else:
                            self.get_logger().info(f'⏳ Cubo azul en píxel({blue_obj["pixel_x"]}, {blue_obj["pixel_y"]}) temporalmente bloqueado')
            else:
                self.get_logger().info('⚠️ No se detectaron objetos válidos')
        except Exception as e:
            self.get_logger().error(f'Error en detección: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ObjectDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
