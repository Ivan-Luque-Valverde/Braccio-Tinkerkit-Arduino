#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import subprocess
import time
import math
import numpy as np
import cv2
import json
import os
import sys

class VisionBasedPickAndPlace(Node):
    def __init__(self):
        super().__init__('vision_based_pick_and_place')
        
        # NO crear publishers para evitar conflictos
        # Los comandos se enviarán via subprocess
        
        # Subscriber para detección de objetos
        self.object_subscriber = self.create_subscription(
            PointStamped,
            '/detected_object_coords',
            self.object_detected_callback,
            10
        )
        
        # Configuración
        self.object_detected = False
        self.pixel_x = 0
        self.pixel_y = 0
        self.needs_push = False  # Flag para detectar si necesita empuje
        
        # Cargar homografía para transformación píxel -> mundo
        self.load_homography()
        
        # Estado del sistema
        self.processing = False
        
        self.get_logger().info('🤖 Vision-Based Pick and Place iniciado')
        self.get_logger().info('👁️  Esperando detección de cubo verde...')
        
        # Timer para verificar detecciones periódicamente
        self.timer = self.create_timer(1.0, self.check_for_objects)

    def load_homography(self):
        """Cargar matriz de homografía para transformación píxel -> mundo"""
        try:
            config_path = "/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_vision/config/camera_calibration.json"
            with open(config_path, 'r') as f:
                data = json.load(f)
            self.homography_matrix = np.array(data['homography_matrix'])
            self.get_logger().info('✅ Homografía cargada exitosamente')
        except Exception as e:
            self.homography_matrix = None
            self.get_logger().warn(f'⚠️  Sin homografía ({e}) - usando mapeo simple')

    def transform_pixels_to_world(self, pixel_x, pixel_y):
        """Transformar coordenadas de píxeles a coordenadas del mundo real"""
        if self.homography_matrix is not None:
            try:
                # Usar homografía calibrada
                pixel_point = np.array([[pixel_x, pixel_y]], dtype=np.float32).reshape(-1, 1, 2)
                world_point = cv2.perspectiveTransform(pixel_point, self.homography_matrix)
                x, y = float(world_point[0][0][0]), float(world_point[0][0][1])
                self.get_logger().info(f'🎯 Homografía: píxel ({pixel_x},{pixel_y}) -> mundo ({x:.3f},{y:.3f})')
                return x, y
            except Exception as e:
                self.get_logger().error(f'❌ Error en homografía: {e}')
        
        # Mapeo simple como fallback (calibración básica para Braccio)
        # Asumiendo imagen 640x480 y workspace del Braccio
        x = (pixel_x - 320) * 0.0008  # Escalado ajustado para posiciones realistas
        y = (240 - pixel_y) * 0.0008 + 0.20  # Invertir Y y ajustar offset
        
        # Asegurar que esté dentro del workspace válido
        x = max(0.10, min(0.25, x))  # Limitar X entre 10cm y 25cm
        y = max(-0.10, min(0.10, y))  # Limitar Y entre -10cm y 10cm
        
        self.get_logger().info(f'🎯 Mapeo simple: píxel ({pixel_x},{pixel_y}) -> mundo ({x:.3f},{y:.3f})')
        return x, y

    def object_detected_callback(self, msg):
        """Callback cuando se detecta un objeto"""
        if not self.processing:
            self.pixel_x = int(msg.point.x)
            self.pixel_y = int(msg.point.y)
            self.object_detected = True
            
            self.get_logger().info(f'👁️  Cubo verde detectado en píxeles: ({self.pixel_x}, {self.pixel_y})')

    def check_for_objects(self):
        """Verificar si hay objetos detectados y procesar"""
        if self.object_detected and not self.processing:
            self.process_detected_object()

    def process_detected_object(self):
        """Procesar el objeto detectado y ejecutar pick and place"""
        self.processing = True
        self.object_detected = False
        
        try:
            self.get_logger().info('🔄 Procesando objeto detectado...')
            
            # 1. Transformar píxeles a coordenadas del mundo
            object_x, object_y = self.transform_pixels_to_world(self.pixel_x, self.pixel_y)
            object_z = 0.025  # Altura ajustada para cubos pequeños
            
            self.get_logger().info(f'📍 Objeto localizado en: ({object_x:.3f}, {object_y:.3f}, {object_z:.3f})')
            
            # 2. Calcular cinemática inversa
            if self.run_ik_calculation(object_x, object_y, object_z):
                
                # 3. Ejecutar pick and place
                self.get_logger().info('🤖 Iniciando secuencia pick and place...')
                if self.run_pick_and_place():
                    self.get_logger().info('🎉 ¡PICK AND PLACE COMPLETADO EXITOSAMENTE!')
                else:
                    self.get_logger().error('❌ Error en la ejecución del pick and place')
            else:
                self.get_logger().error('❌ Error en el cálculo de cinemática inversa')
                
        except Exception as e:
            self.get_logger().error(f'❌ Error procesando objeto: {e}')
        finally:
            self.processing = False
            self.get_logger().info('✅ Listo para detectar nuevos objetos')

    def run_ik_calculation(self, object_x, object_y, object_z):
        """Ejecutar el calculador de cinemática inversa y detectar estrategia"""
        self.get_logger().info('🧮 Calculando cinemática inversa...')
        
        script_path = "/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_moveit_config/scripts/inverse_kinematics_calculator.py"
        
        try:
            # Cambiar al directorio del workspace
            original_dir = os.getcwd()
            os.chdir("/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino")
            
            # Ejecutar el calculador de IK con auto-guardado
            cmd = [
                "bash", "-c", 
                f"source install/setup.bash && python3 {script_path} {object_x} {object_y} {object_z}"
            ]
            
            # Ejecutar y capturar la salida
            result = subprocess.run(cmd, capture_output=True, text=True, input="y\n", timeout=30)
            
            # Restaurar directorio original
            os.chdir(original_dir)
            
            if result.returncode == 0:
                self.get_logger().info('✅ Cinemática inversa calculada exitosamente')
                
                # Analizar la salida para detectar la estrategia
                output = result.stdout
                if "ESTRATEGIA: EMPUJE + PICK" in output or "EMPUJE + PICK" in output:
                    self.get_logger().info('🔄 Estrategia detectada: EMPUJE + PICK')
                    self.needs_push = True
                else:
                    self.get_logger().info('✅ Estrategia detectada: PICK DIRECTO')
                    self.needs_push = False
                
                return True
            else:
                self.get_logger().error('❌ Error en cálculo de cinemática inversa:')
                self.get_logger().error(result.stderr)
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('⏰ Timeout en cálculo de cinemática inversa')
            return False
        except Exception as e:
            self.get_logger().error(f'❌ Error ejecutando calculador de IK: {e}')
            return False

    def run_pick_and_place(self):
        """Ejecutar la secuencia de pick and place (con o sin empuje)"""
        
        if self.needs_push:
            # Implementar estrategia de empuje directamente
            self.get_logger().info('🔄 Ejecutando pick and place CON empuje')
            return self.execute_push_strategy()
        else:
            # Usar el sistema configurable normal
            self.get_logger().info('✅ Ejecutando pick and place DIRECTO')
            return self.execute_normal_pick_and_place()

    def execute_push_strategy(self):
        """Ejecutar estrategia de empuje integrada"""
        try:
            self.get_logger().info('🔄 === INICIANDO ESTRATEGIA DE EMPUJE ===')
            
            # 1. Calcular ángulo base hacia el objeto
            base_angle = math.atan2(self.pixel_y - 240, self.pixel_x - 320)  # Ángulo desde centro de imagen
            self.get_logger().info(f'📐 Ángulo base calculado: {math.degrees(base_angle):.1f}°')
            
            # 2. Configurar modo empuje
            if self.execute_push_sequence(base_angle):
                self.get_logger().info('✅ Modo empuje configurado')
                
                # 3. Ejecutar pick_and_place_configurable con empuje integrado
                self.get_logger().info('🎯 Ejecutando secuencia completa (empuje + pick)...')
                return self.execute_normal_pick_and_place()
            else:
                self.get_logger().error('❌ Error configurando modo empuje')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Error en estrategia de empuje: {e}')
            return False

    def execute_push_sequence(self, base_angle):
        """Crear archivos flag para activar empuje en pick_and_place_configurable"""
        try:
            self.get_logger().info('🔄 Configurando modo empuje inteligente...')
            
            # Crear archivo flag para indicar que necesita empuje
            push_flag_file = "/tmp/braccio_needs_push.flag"
            with open(push_flag_file, 'w') as f:
                f.write("push_required")
            
            # Guardar ángulo base para el empuje
            angle_file = "/tmp/braccio_push_angle.txt"
            with open(angle_file, 'w') as f:
                f.write(str(base_angle))
            
            # Guardar coordenadas del objeto para empuje inteligente
            coords_file = "/tmp/braccio_object_coords.txt"
            object_x, object_y = self.transform_pixels_to_world(self.pixel_x, self.pixel_y)
            with open(coords_file, 'w') as f:
                f.write(f"{object_x},{object_y}")
            
            self.get_logger().info(f'✅ Modo empuje configurado:')
            self.get_logger().info(f'   📐 Ángulo: {math.degrees(base_angle):.1f}°')
            self.get_logger().info(f'   📍 Objeto: ({object_x:.3f}, {object_y:.3f})')
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Error configurando modo empuje: {e}')
            return False

    def execute_normal_pick_and_place(self):
        """Ejecutar pick and place normal usando el script configurable"""
        script_path = "/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_moveit_config/scripts/pick_and_place_configurable.py"
        
        try:
            # Cambiar al directorio del workspace
            original_dir = os.getcwd()
            os.chdir("/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino")
            
            # Ejecutar el script configurable
            cmd = ["bash", "-c", f"source install/setup.bash && python3 {script_path}"]
            
            result = subprocess.run(cmd, timeout=120)  # Timeout de 2 minutos
            
            # Restaurar directorio original
            os.chdir(original_dir)
            
            if result.returncode == 0:
                self.get_logger().info('✅ Secuencia ejecutada exitosamente')
                return True
            else:
                self.get_logger().error('❌ Error en ejecución de pick and place')
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('⏰ Timeout en ejecución de pick and place')
            return False
        except Exception as e:
            self.get_logger().error(f'❌ Error ejecutando pick and place: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VisionBasedPickAndPlace()
        
        print("\n🤖 VISION-BASED PICK AND PLACE")
        print("="*50)
        print("👁️  Escuchando detecciones de cubo verde...")
        print("🎯 Destino: Configurado en pick_and_place_config.yaml")
        print("🛑 Ctrl+C para detener")
        print("="*50)
        
        # Ejecutar el nodo
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n👋 Detenido por el usuario")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
