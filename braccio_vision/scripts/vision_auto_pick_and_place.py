#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import time
import math
import numpy as np
import cv2
import json
import os
import sys

# Importar directamente la clase ConfigurablePickAndPlace
sys.path.append('/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_vision/scripts')
from pick_and_place_configurable import ConfigurablePickAndPlace

class VisionBasedPickAndPlace(Node):
    def __init__(self):
        super().__init__('vision_based_pick_and_place')
        
        # Crear instancia del nodo de pick and place
        self.pick_and_place_node = ConfigurablePickAndPlace()
        
        # Subscriber para detección de objetos
        self.object_subscriber = self.create_subscription(
            PointStamped,
            '/detected_object_coords',
            self.object_detected_callback,
            10
        )
        
        # Configuración
        self.detected_objects = []  # Lista de objetos detectados con nombres (pixel_x, pixel_y, model_name)
        self.processed_objects = set()  # Set de objetos ya procesados (model_name)
        self.processing = False
        
        # Cargar homografía para transformación píxel -> mundo
        self.load_homography()
        
        self.get_logger().info('🤖 Vision-Based Pick and Place iniciado')
        self.get_logger().info('👁️  Esperando detección de cubos verdes...')
        
        # Timer para verificar detecciones periódicamente
        self.timer = self.create_timer(1.0, self.check_for_objects)
        
        # Timer para mostrar estadísticas cada 10 segundos
        self.stats_timer = self.create_timer(10.0, self.show_statistics)

    def load_homography(self):
        """Cargar matriz de homografía para transformación píxel -> mundo"""
        try:
            config_path = "/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_vision/config/camera_calibration.json"
            with open(config_path, 'r') as f:
                data = json.load(f)
            self.homography_matrix = np.array(data['homography_matrix'])
            self.get_logger().info('✅ Homografía cargada exitosamente')
            # Debug: mostrar la matriz
            self.get_logger().info(f'🔍 Matriz de homografía: {self.homography_matrix.tolist()}')
        except Exception as e:
            self.homography_matrix = None
            self.get_logger().warn(f'⚠️  Sin homografía ({e}) - usando mapeo simple')

    def transform_pixels_to_world(self, pixel_x, pixel_y):
        """Transformar coordenadas de píxeles a coordenadas del mundo real"""
        self.get_logger().info(f'🔄 TRANSFORMANDO: píxel({pixel_x},{pixel_y}) usando {"HOMOGRAFÍA" if self.homography_matrix is not None else "MAPEO SIMPLE"}')
        
        if self.homography_matrix is not None:
            try:
                # Usar homografía calibrada
                pixel_point = np.array([[pixel_x, pixel_y]], dtype=np.float32).reshape(-1, 1, 2)
                world_point = cv2.perspectiveTransform(pixel_point, self.homography_matrix)
                x, y = float(world_point[0][0][0]), float(world_point[0][0][1])
                rho = math.sqrt(x**2 + y**2)
                self.get_logger().info(f'🎯 Homografía: píxel({pixel_x},{pixel_y}) -> mundo({x:.3f},{y:.3f}) rho={rho:.3f}m')
                return x, y
            except Exception as e:
                self.get_logger().error(f'❌ Error en homografía: {e}')
        
        # Mapeo corregido para el workspace real del Braccio
        # Basado en análisis de posiciones reales vs detectadas
        
        # Cálculo base sin limitaciones restrictivas
        x = (pixel_x - 320) * 0.0008  # Mantener factor de escala
        y = (240 - pixel_y) * 0.0008  # Mantener factor de escala, invertir Y
        
        # Ajuste de offset para centrar en el workspace del Braccio
        x_offset = 0.30  # Mover el origen más lejos de la base (era 0.20)
        y_offset = 0.0   # Sin offset en Y (era 0.20)
        
        x_world = x + x_offset
        y_world = y + y_offset
        
        # Límites ampliados para el workspace real del Braccio
        # Basado en posiciones reales: X=[0.24-0.35], Y=[-0.23, 0.18]
        x_world = max(0.20, min(0.40, x_world))  # Rango ampliado y realista
        y_world = max(-0.25, min(0.20, y_world))  # Rango ampliado y realista
        
        rho = math.sqrt(x_world**2 + y_world**2)
        self.get_logger().info(f'🎯 Mapeo corregido: píxel({pixel_x},{pixel_y}) -> mundo({x_world:.3f},{y_world:.3f}) rho={rho:.3f}m')
        return x_world, y_world

    def object_detected_callback(self, msg):
        """Añadir nueva detección a la lista si no está ya presente"""
        pixel_x = int(msg.point.x)
        pixel_y = int(msg.point.y)
        
        # DEBUG: Mostrar exactamente qué recibimos
        self.get_logger().info(f'📨 RECIBIDO del object_detector: píxel({pixel_x}, {pixel_y})')
        
        # Verificar si el objeto ya está en la lista (evitar duplicados)
        # Usando un umbral de distancia para evitar ruido de detección
        threshold = 10  # píxeles de tolerancia
        is_duplicate = False
        
        for existing_x, existing_y, _ in self.detected_objects:
            distance = math.sqrt((pixel_x - existing_x)**2 + (pixel_y - existing_y)**2)
            if distance < threshold:
                is_duplicate = True
                break
        
        if not is_duplicate:
            # Generar nombre de modelo basado en posición
            model_name = self.determine_model_name(pixel_x, pixel_y)
            
            # Si no se pudo determinar un modelo válido (ya procesado), ignorar
            if model_name is None:
                self.get_logger().info(f'🚫 No se pudo mapear cubo en ({pixel_x}, {pixel_y}) - posiblemente ya procesado')
                return
            
            # Verificar si el objeto ya fue procesado (doble verificación)
            if model_name in self.processed_objects:
                self.get_logger().info(f'🚫 Cubo {model_name} ya procesado anteriormente, ignorando...')
                return
            
            self.detected_objects.append((pixel_x, pixel_y, model_name))
            color_type = "verde" if "green" in model_name else "azul" if "blue" in model_name else "desconocido"
            self.get_logger().info(f'👁️  Nuevo cubo {color_type} detectado: {model_name} en píxeles ({pixel_x}, {pixel_y})')
            self.get_logger().info(f'📋 Total de cubos en lista: {len(self.detected_objects)}')
        else:
            self.get_logger().info(f'🔄 Objeto duplicado ignorado: píxel({pixel_x}, {pixel_y})')

    def determine_model_name(self, pixel_x, pixel_y):
        """
        Determina el nombre del modelo real más cercano a las coordenadas detectadas.
        Mapea coordenadas detectadas a los nombres reales de los objetos en Gazebo.
        """
        # Convertir coordenadas de píxeles a coordenadas del mundo
        world_x, world_y = self.transform_pixels_to_world(pixel_x, pixel_y)
        
        # Lista de objetos reales con sus posiciones (basada en object_spawner.py)
        real_objects = [
            {"name": "green_cube1", "x": 0.35, "y": 0.05, "z": 0.025},
            {"name": "green_cube2", "x": 0.28, "y": 0.18, "z": 0.025}, 
            {"name": "blue_cube1", "x": 0.28, "y": -0.15, "z": 0.025},
            {"name": "blue_cube2", "x": 0.24, "y": -0.23, "z": 0.025},  
        ]
        
        # Encontrar el objeto más cercano
        min_distance = float('inf')
        closest_object = None
        
        for obj in real_objects:
            # Calcular distancia euclidiana en 2D (X, Y)
            distance = math.sqrt((world_x - obj["x"])**2 + (world_y - obj["y"])**2)
            if distance < min_distance:
                min_distance = distance
                closest_object = obj["name"]
        
        # Verificar que el objeto no esté ya procesado
        if closest_object in self.processed_objects:
            self.get_logger().warn(f'⚠️ Objeto más cercano {closest_object} ya fue procesado')
            return None
        
        self.get_logger().info(f'🎯 Coordenadas detectadas ({world_x:.3f}, {world_y:.3f}) → Objeto real: {closest_object} (distancia: {min_distance:.3f}m)')
        return closest_object

    def check_for_objects(self):
        """Procesar el primer objeto de la lista si no estamos ocupados"""
        if self.detected_objects and not self.processing:
            # Separar verdes y azules
            green_objects = [(x, y, name) for x, y, name in self.detected_objects if 'green' in name]
            blue_objects = [(x, y, name) for x, y, name in self.detected_objects if 'blue' in name]
            # Prioridad: verdes primero
            if green_objects:
                pixel_x, pixel_y, model_name = green_objects[0]
                self.get_logger().info(f'🎯 Procesando cubo verde {model_name} en posición: ({pixel_x}, {pixel_y})')
                self.get_logger().info(f'📋 Quedan {len(self.detected_objects)-1} cubos en cola')
                self.process_detected_object(pixel_x, pixel_y, model_name)
            elif blue_objects:
                pixel_x, pixel_y, model_name = blue_objects[0]
                self.get_logger().info(f'🎯 Procesando cubo azul {model_name} en posición: ({pixel_x}, {pixel_y})')
                self.get_logger().info(f'📋 Quedan {len(self.detected_objects)-1} cubos en cola')
                self.process_detected_object(pixel_x, pixel_y, model_name)

    def process_detected_object(self, pixel_x, pixel_y, model_name):
        """Procesar el objeto detectado y ejecutar pick and place"""
        self.processing = True
        try:
            self.get_logger().info(f'🔄 Procesando objeto {model_name}...')
            # 1. Transformar píxeles a coordenadas del mundo
            object_x, object_y = self.transform_pixels_to_world(pixel_x, pixel_y)
            object_z = 0.025  # Altura real de los cubos spawneados
            
            # Por ahora, usamos las coordenadas originales para el diagnóstico
            object_x_corrected = object_x
            object_y_corrected = object_y
            
            self.get_logger().info(f'📍 Objeto {model_name} localizado en: ({object_x:.3f}, {object_y:.3f}, {object_z:.3f})')
            self.get_logger().info(f'🔧 Coordenadas para IK: ({object_x_corrected:.3f}, {object_y_corrected:.3f}, {object_z:.3f})')
            
            # 2. Calcular cinemática inversa
            if not self.run_ik_calculation(object_x_corrected, object_y_corrected, object_z):
                self.get_logger().error(f'❌ Error en el cálculo de cinemática inversa para {model_name}. Pick and place cancelado.')
                print(f"❌ Error: {model_name} está fuera de alcance o la cinemática ha fallado. Pick and place cancelado.")
                # Eliminar el objeto de la lista aunque falle (objeto inaccesible)
                if self.detected_objects:
                    self.detected_objects.pop(0)
                    self.get_logger().info(f'🗑️  {model_name} eliminado de la lista (fuera de alcance)')
                return
            
            # 3. Ejecutar pick and place usando el nodo configurado
            self.get_logger().info(f'🤖 Iniciando secuencia pick and place para {model_name}...')
            # Seleccionar secuencia según color del cubo
            if "green" in model_name:
                sequence_name = 'green_cube_sequence'
                self.get_logger().info(f'🟢 Usando secuencia para cubos verdes → destino: (0.25, 0.10)')
            elif "blue" in model_name:
                sequence_name = 'blue_cube_sequence'
                self.get_logger().info(f'🔵 Usando secuencia para cubos azules → destino: (-0.25, 0.10)')
            else:
                sequence_name = 'basic_demo'  # fallback
                self.get_logger().info(f'⚪ Usando secuencia por defecto')

            # Llamar al nodo de pick and place con la secuencia específica
            if self.pick_and_place_node.execute_pick_and_place_for_target(model_name, sequence_name):
                self.get_logger().info(f'🎉 ¡PICK AND PLACE COMPLETADO EXITOSAMENTE PARA {model_name}!')
                # Marcar el objeto como procesado
                self.processed_objects.add(model_name)
                # Eliminar el objeto de la lista tras recogerlo exitosamente
                if self.detected_objects:
                    self.detected_objects.pop(0)
                    self.get_logger().info(f'✅ {model_name} recogido y marcado como procesado')
                    self.get_logger().info(f'📝 Objetos procesados: {list(self.processed_objects)}')
            else:
                self.get_logger().error(f'❌ Error en la ejecución del pick and place para {model_name}')
                # También eliminar si falla la ejecución (evitar bucle infinito)
                if self.detected_objects:
                    self.detected_objects.pop(0)
                    self.get_logger().info('🗑️  Objeto eliminado de la lista (error en ejecución)')
                    
        except Exception as e:
            self.get_logger().error(f'❌ Error procesando objeto: {e}')
            # Eliminar objeto en caso de excepción
            if self.detected_objects:
                self.detected_objects.pop(0)
                self.get_logger().info('🗑️  Objeto eliminado de la lista (excepción)')
        finally:
            self.processing = False
            if self.detected_objects:
                self.get_logger().info(f'📋 Objetos restantes en cola: {len(self.detected_objects)}')
            else:
                self.get_logger().info('✅ Lista de objetos vacía. Esperando nuevas detecciones...')

    def run_ik_calculation(self, object_x, object_y, object_z):
        """Ejecutar el calculador de cinemática inversa y detectar estrategia"""
        self.get_logger().info('🧮 Calculando cinemática inversa...')
        
        try:
            # Importar directamente la clase de IK
            sys.path.append('/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_vision/scripts')
            from inverse_kinematics_calculator import InverseKinematicsCalculator
            
            # Crear instancia y calcular IK
            ik_calculator = InverseKinematicsCalculator()
            
            # Verificar si las coordenadas están en el workspace
            if not ik_calculator.validate_workspace(object_x, object_y, object_z):
                self.get_logger().error('❌ Error: Objeto fuera del workspace válido')
                return False
            
            # Calcular las posiciones de pick
            pick_positions = ik_calculator.calculate_pick_positions(object_x, object_y, object_z, approach_height=0.08)
            
            if pick_positions is None:
                self.get_logger().error('❌ Error: No se pudieron calcular las posiciones de pick')
                return False
            
            # DIAGNÓSTICO: Mostrar posiciones calculadas
            self.get_logger().info(f'🧮 DIAGNÓSTICO - Posiciones IK calculadas para ({object_x:.3f}, {object_y:.3f}, {object_z:.3f}):')
            if 'pick_approach' in pick_positions:
                self.get_logger().info(f'   📍 Pick approach: {[round(x, 4) for x in pick_positions["pick_approach"]]}')
            if 'pick_position' in pick_positions:
                self.get_logger().info(f'   📍 Pick position: {[round(x, 4) for x in pick_positions["pick_position"]]}')
            
            # Verificar que las posiciones son diferentes para objetos diferentes
            if hasattr(self, 'last_pick_positions'):
                if (pick_positions.get('pick_approach') == self.last_pick_positions.get('pick_approach') and
                    pick_positions.get('pick_position') == self.last_pick_positions.get('pick_position')):
                    self.get_logger().warn('⚠️  ALERTA: Las posiciones IK son idénticas al objeto anterior!')
                else:
                    self.get_logger().info('✅ Las posiciones IK son diferentes al objeto anterior')
            self.last_pick_positions = pick_positions.copy()
            
            # Guardar las posiciones en el archivo de configuración
            config_path = '/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_moveit_config/config/pick_and_place_config.yaml'
            
            if ik_calculator.save_positions_to_config(pick_positions, config_path):
                self.get_logger().info('✅ Cinemática inversa calculada y configuración guardada exitosamente')
                # CRÍTICO: Tiempo adicional para asegurar escritura completa y estabilización
                self.get_logger().info('⏳ Esperando estabilización del archivo de configuración...')
                time.sleep(1.0)  # Tiempo aumentado para evitar problemas de sincronización
                return True
            else:
                self.get_logger().error('❌ Error: No se pudo guardar la configuración')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Error ejecutando calculador de IK: {e}')
            return False

    def show_statistics(self):
        """Mostrar estadísticas del sistema cada 10 segundos"""
        if self.processed_objects:
            self.get_logger().info(f'📊 ESTADÍSTICAS: {len(self.processed_objects)} objetos procesados: {list(self.processed_objects)}')
        else:
            self.get_logger().info('📊 ESTADÍSTICAS: Ningún objeto procesado aún')
        
        if self.detected_objects:
            self.get_logger().info(f'📊 ESTADÍSTICAS: {len(self.detected_objects)} objetos en cola de procesamiento')

    def reset_processed_objects(self):
        """Resetear la lista de objetos procesados (útil para pruebas)"""
        self.processed_objects.clear()
        self.get_logger().info('🔄 Lista de objetos procesados reseteada')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VisionBasedPickAndPlace()
        
        print("\n🤖 VISION-BASED PICK AND PLACE")
        print("="*50)
        print("👁️  Escuchando detecciones de cubos verdes...")
        print("📋 Procesará múltiples objetos en orden de detección")
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
