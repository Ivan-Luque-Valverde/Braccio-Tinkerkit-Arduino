#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # Bridge para convertir mensajes ROS a OpenCV
        self.bridge = CvBridge()
        
        # Subscriber para la imagen raw de la cámara
        self.image_subscription = self.create_subscription(
            Image,
            '/overhead_camera/image_raw',
            self.image_callback,
            10
        )
        
        # Subscriber para la imagen de debug con detecciones
        self.debug_subscription = self.create_subscription(
            Image,
            '/vision/debug_image',
            self.debug_callback,
            10
        )
        
        # Variables para las imágenes
        self.current_image = None
        self.debug_image = None
        self.image_received = False
        self.debug_received = False
        
        # Timer para mostrar imágenes
        self.timer = self.create_timer(0.1, self.display_images)
        
        self.get_logger().info('🖥️ Visor de Cámara iniciado')
        self.get_logger().info('📷 Esperando imágenes de cámara...')
        
        # Configurar ventanas OpenCV
        cv2.namedWindow('Camara Principal', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Vision Debug', cv2.WINDOW_AUTOSIZE)
        
    def image_callback(self, msg):
        """Callback para imagen raw de la cámara"""
        try:
            # Convertir mensaje ROS a imagen OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.current_image = cv_image
            self.image_received = True
            
        except Exception as e:
            self.get_logger().error(f'Error procesando imagen: {e}')
    
    def debug_callback(self, msg):
        """Callback para imagen de debug con detecciones"""
        try:
            # Convertir mensaje ROS a imagen OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.debug_image = cv_image
            self.debug_received = True
            
        except Exception as e:
            self.get_logger().error(f'Error procesando imagen debug: {e}')
    
    def display_images(self):
        """Mostrar las imágenes en ventanas OpenCV"""
        try:
            # Mostrar imagen principal si está disponible
            if self.image_received and self.current_image is not None:
                # Agregar texto informativo
                img_display = self.current_image.copy()
                
                # Información del sistema
                cv2.putText(img_display, 'Braccio Vision System', (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(img_display, 'Camera Feed', (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Estado de debug
                debug_status = "Debug: ON" if self.debug_received else "Debug: Waiting..."
                color = (0, 255, 0) if self.debug_received else (0, 0, 255)
                cv2.putText(img_display, debug_status, (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                cv2.imshow('Camara Principal', img_display)
            
            # Mostrar imagen de debug si está disponible
            if self.debug_received and self.debug_image is not None:
                cv2.imshow('Vision Debug', self.debug_image)
            
            # Procesar eventos de ventana
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Cerrando visor de cámara...')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error mostrando imágenes: {e}')
    
    def destroy_node(self):
        """Limpiar recursos al cerrar"""
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        viewer = CameraViewer()
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'viewer' in locals():
            viewer.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
