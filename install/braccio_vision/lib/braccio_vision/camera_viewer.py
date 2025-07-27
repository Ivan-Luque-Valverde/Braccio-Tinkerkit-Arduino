#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # Cargar configuración
        self.load_config()
        
        # Bridge para convertir imágenes
        self.bridge = CvBridge()
        
        # Subscriber para imagen raw
        self.image_subscriber = self.create_subscription(
            Image,
            self.config['camera']['topic_image'],
            self.image_callback,
            10
        )
        
        # Subscriber para imagen de debug
        self.debug_subscriber = self.create_subscription(
            Image,
            self.config['publishing']['debug_image_topic'],
            self.debug_callback,
            10
        )
        
        self.get_logger().info('📺 Camera Viewer iniciado')
        self.get_logger().info('🎮 Presiona "q" en las ventanas para cerrar')

    def load_config(self):
        """Carga la configuración básica"""
        try:
            pkg_share = get_package_share_directory('braccio_vision')
            config_path = os.path.join(pkg_share, 'config', 'vision_config.yaml')
            
            if not os.path.exists(config_path):
                config_path = '/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_vision/config/vision_config.yaml'
            
            with open(config_path, 'r') as file:
                self.config = yaml.safe_load(file)
                
        except Exception as e:
            self.get_logger().error(f'Error cargando configuración: {str(e)}')
            self.config = {
                'camera': {
                    'topic_image': '/overhead_camera/image_raw'
                },
                'publishing': {
                    'debug_image_topic': '/vision/debug_image'
                }
            }

    def image_callback(self, msg):
        """Callback para imagen raw de la cámara"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Agregar información a la imagen
            text = "Camara Cenital - Raw"
            cv2.putText(cv_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       1, (255, 255, 255), 2)
            
            # Mostrar imagen
            cv2.imshow("Overhead Camera - Raw", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error mostrando imagen raw: {str(e)}')

    def debug_callback(self, msg):
        """Callback para imagen de debug con detecciones"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Agregar información a la imagen
            text = "Deteccion de Objetos"
            cv2.putText(cv_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       1, (0, 255, 0), 2)
            
            # Mostrar imagen
            cv2.imshow("Object Detection - Debug", cv_image)
            
            # Verificar si se presionó 'q' para salir
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Cerrando visor de cámara...')
                rclpy.shutdown()
            
        except Exception as e:
            self.get_logger().error(f'Error mostrando imagen debug: {str(e)}')

    def destroy_node(self):
        """Limpiar al destruir el nodo"""
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraViewer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
