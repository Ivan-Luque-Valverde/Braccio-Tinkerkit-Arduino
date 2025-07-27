#!/usr/bin/env python3

"""
Sistema de Demostración Completo
Este script demuestra todas las capacidades del sistema de visión Braccio
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import time

class VisionSystemDemo(Node):
    def __init__(self):
        super().__init__('vision_system_demo')
        
        # Suscriptor para coordenadas detectadas
        self.coords_subscription = self.create_subscription(
            Point,
            '/detected_object_coords',
            self.coords_callback,
            10
        )
        
        # Publicador para mensajes de demostración
        self.demo_publisher = self.create_publisher(String, '/demo_status', 10)
        
        self.detected_objects = []
        self.demo_running = True
        
        # Timer para la demostración
        self.demo_timer = self.create_timer(5.0, self.demo_status)
        
        self.get_logger().info('🎬 Demostración del Sistema de Visión Braccio iniciada')
        self.get_logger().info('📷 Monitoreando detecciones de objetos...')
        
    def coords_callback(self, msg):
        """Callback para coordenadas detectadas"""
        detection_time = time.time()
        
        detection_info = {
            'x': msg.x,
            'y': msg.y,
            'time': detection_time
        }
        
        self.detected_objects.append(detection_info)
        
        self.get_logger().info(f'🎯 DETECCIÓN: x={msg.x:.3f}m, y={msg.y:.3f}m')
        
        # Publicar estado de demostración
        status_msg = String()
        status_msg.data = f"Objeto detectado en ({msg.x:.3f}, {msg.y:.3f})"
        self.demo_publisher.publish(status_msg)
    
    def demo_status(self):
        """Mostrar estado de la demostración"""
        if self.demo_running:
            total_detections = len(self.detected_objects)
            
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'📊 ESTADO DE LA DEMOSTRACIÓN')
            self.get_logger().info(f'🔢 Total de detecciones: {total_detections}')
            
            if total_detections > 0:
                last_detection = self.detected_objects[-1]
                self.get_logger().info(f'🎯 Última detección: ({last_detection["x"]:.3f}, {last_detection["y"]:.3f})')
                
                # Calcular promedio de posiciones
                avg_x = sum(d['x'] for d in self.detected_objects) / total_detections
                avg_y = sum(d['y'] for d in self.detected_objects) / total_detections
                
                self.get_logger().info(f'📍 Posición promedio: ({avg_x:.3f}, {avg_y:.3f})')
            else:
                self.get_logger().info('⏳ Esperando detecciones...')
                self.get_logger().info('💡 TIP: Ejecuta "ros2 run braccio_vision object_spawner.py"')
            
            self.get_logger().info('=' * 50)
    
    def show_system_info(self):
        """Mostrar información del sistema"""
        self.get_logger().info('🤖 SISTEMA DE VISIÓN BRACCIO')
        self.get_logger().info('=' * 40)
        self.get_logger().info('📦 Componentes activos:')
        self.get_logger().info('  - 📷 Cámara cenital')
        self.get_logger().info('  - 🎯 Detector de objetos')  
        self.get_logger().info('  - 🤖 Control de manipulación')
        self.get_logger().info('  - 🖥️ Visualización en tiempo real')
        self.get_logger().info('=' * 40)

def main(args=None):
    rclpy.init(args=args)
    
    demo = VisionSystemDemo()
    demo.show_system_info()
    
    try:
        rclpy.spin(demo)
    except KeyboardInterrupt:
        demo.get_logger().info('🛑 Demostración finalizada por usuario')
        
        # Mostrar resumen final
        total = len(demo.detected_objects)
        demo.get_logger().info(f'📊 RESUMEN FINAL: {total} detecciones realizadas')
        
        if total > 0:
            demo.get_logger().info('✅ Sistema de visión funcionando correctamente!')
        else:
            demo.get_logger().info('⚠️  No se detectaron objetos durante la demostración')
    
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
