#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import json
import sys
import numpy as np
import yaml

# Constantes del Braccio (basadas en el script original)
THETA_EXT = 0.27  # Ángulo mínimo del shoulder (radianes)
THETA_RET = np.pi/4  # Ángulo máximo del shoulder (radianes)

def cart2pol(x, y):
    """Convierte coordenadas cartesianas a polares"""
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return rho, phi

def pol2cart(rho, phi):
    """Convierte coordenadas polares a cartesianas"""
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return x, y

def get_other_angles(theta_shoulder):
    """Calcula wrist y elbow basados en shoulder para mantener orientación vertical"""
    theta_wrist = theta_shoulder + np.pi/2
    theta_elbow = np.pi/2 - 2*theta_shoulder
    return theta_wrist, theta_elbow

class InverseKinematicsCalculator(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_calculator')
        
        # Parámetros calibrados del Braccio (debes ajustar según tu robot)
        self.L = 0.3025  # Longitud del brazo principal (metros)
        self.l = 0.064  # Offset desde la base (metros)
        
        self.get_logger().info('🤖 Calculadora de IK analítica para Braccio iniciada')
        self.get_logger().info(f'📏 Parámetros: L={self.L}m, l={self.l}m')

    def validate_workspace(self, x, y, z):
        """Valida que las coordenadas estén dentro del workspace del Braccio"""
        # Convertir a coordenadas polares para validar
        rho, phi = cart2pol(x, y)
        
        # Verificar altura
        z_min, z_max = 0.01, 0.30
        if not (z_min <= z <= z_max):
            self.get_logger().error(f'❌ Altura fuera de rango: z={z:.3f} (rango: {z_min}-{z_max})')
            return False
        
        # Verificar radio alcanzable
        rho_min = self.l + self.L * np.cos(THETA_RET)  # Radio mínimo
        rho_max = self.l + self.L * np.cos(THETA_EXT)  # Radio máximo
        
        self.get_logger().info(f'📏 Workspace: rho={rho:.3f}m, rango=[{rho_min:.3f}, {rho_max:.3f}]m')
        
        if not (rho_min <= rho <= rho_max):
            self.get_logger().error(f'❌ Radio fuera de rango: rho={rho:.3f} (rango: {rho_min:.3f}-{rho_max:.3f})')
            return False
            
        return True

    def calculate_ik_xy(self, x, y):
        """
        Calcula la cinemática inversa analítica para posición (x, y)
        Retorna [phi, theta_shoulder, theta_elbow, theta_wrist, theta_gripper]
        """
        try:
            # Convertir a coordenadas polares
            rho, phi = cart2pol(x, y)
            
            # Calcular ángulo del shoulder
            cos_theta = (rho - self.l) / self.L
            
            # Verificar que esté en el dominio válido
            if cos_theta < np.cos(THETA_RET) or cos_theta > np.cos(THETA_EXT):
                self.get_logger().error(f'❌ Posición fuera del dominio: cos_theta={cos_theta:.3f}')
                return None
            
            theta_shoulder = np.arccos(cos_theta)
            
            # Calcular otros ángulos basados en shoulder
            theta_wrist, theta_elbow = get_other_angles(theta_shoulder)
            theta_gripper = np.pi/2  # Gripper vertical
            
            # Retornar ángulos en el orden correcto para el Braccio
            angles = [phi, theta_shoulder, theta_elbow, theta_wrist, theta_gripper]
            
            self.get_logger().info(f'✅ IK calculado para ({x:.3f}, {y:.3f})')
            self.get_logger().info(f'📐 Ángulos: {[f"{math.degrees(a):.1f}°" for a in angles]}')
            
            return angles
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en cálculo IK: {e}')
            return None


    def calculate_strategy(self, object_x, object_y, object_z):
        """
        Solo calcula pick si el objeto está en rango. Si no, muestra mensaje y no devuelve posiciones.
        """
        self.get_logger().info('=== ANALIZANDO ESTRATEGIA ===')
        pick_positions = self.calculate_pick_positions(object_x, object_y, object_z)
        if pick_positions:
            self.get_logger().info(f'✅ Estrategia: PICK DIRECTO')
            return {
                'needs_pick': True,
                'reason': 'En rango válido',
                'positions': pick_positions
            }
        else:
            self.get_logger().warn('❌ Objeto fuera de alcance. No se puede realizar pick ni empuje.')
            print("❌ Objeto fuera de alcance. No se puede realizar pick ni empuje.")
            return {
                'needs_pick': False,
                'reason': 'Objeto fuera de alcance',
                'positions': {}
            }

    def calculate_pick_positions(self, object_x, object_y, object_z, approach_height=0.05):
        """
        Calcula las posiciones para pick (approach y position)
        Ahora REALMENTE usa la altura Z para calcular diferentes posiciones
        """
        self.get_logger().info('=== CALCULANDO POSICIONES DE PICK ===')
        
        positions = {}
        
        # Validar workspace
        if not self.validate_workspace(object_x, object_y, object_z):
            self.get_logger().error(f'❌ Objeto fuera del workspace: ({object_x:.3f}, {object_y:.3f}, {object_z:.3f})')
            return None
        
        # 1. Posición de aproximación (más alta que el objeto)
        approach_z = object_z + approach_height  # Z más alta para aproximación
        pick_approach = self.calculate_ik_xyz(object_x, object_y, approach_z)
        if pick_approach:
            positions['pick_approach'] = pick_approach
            self.get_logger().info(f'✅ Pick approach calculado para Z={approach_z:.3f}m')
        else:
            self.get_logger().error(f'❌ No se pudo calcular pick_approach para Z={approach_z:.3f}m')
            return None
        
        # 2. Posición exacta del objeto (altura real del objeto)
        pick_position = self.calculate_ik_xyz(object_x, object_y, object_z)
        if pick_position:
            positions['pick_position'] = pick_position
            self.get_logger().info(f'✅ Pick position calculado para Z={object_z:.3f}m')
        else:
            self.get_logger().error(f'❌ No se pudo calcular pick_position para Z={object_z:.3f}m')
            return None
        
        self.get_logger().info('=== CÁLCULOS DE PICK COMPLETADOS ===')
        self.get_logger().info(f'📏 Diferencia de altura: {approach_height:.3f}m entre approach y position')
        
        return positions

    def calculate_ik_xyz(self, x, y, z):
        """
        Calcula la cinemática inversa para posición (x, y, z)
        Ajusta el ELBOW según altura Z para controlar altura efectiva
        """
        try:
            # Primero calcular IK básica en 2D
            base_angles = self.calculate_ik_xy(x, y)
            if base_angles is None:
                return None
            
            # Extraer ángulos base
            phi, theta_shoulder, theta_elbow, theta_wrist, theta_gripper = base_angles
            
            # Ajustar ELBOW según altura Z (más efectivo que shoulder)
            # LÓGICA CORRECTA:
            # Z más baja → elbow más BAJO (ángulo menor) para BAJAR el efector
            # Z más alta → elbow más ALTO (ángulo mayor) para SUBIR el efector
            z_nominal = 0.08  # Altura de referencia
            z_factor = 4.0    # Factor de sensibilidad para el elbow
            
            # Calcular ajuste: Z menor debe dar elbow menor para bajar efector
            z_adjustment = (z - z_nominal) * z_factor  # SIN negativo!
            theta_elbow_adjusted = theta_elbow + z_adjustment
            
            # Limitar elbow a rango válido (0.1 a π-0.1)
            theta_elbow_adjusted = max(0.1, min(np.pi - 0.1, theta_elbow_adjusted))
            
            # Recalcular wrist para mantener orientación (compensar cambio en elbow)
            theta_wrist_adjusted = theta_shoulder + np.pi/2 + (theta_elbow_adjusted - theta_elbow) * 0.5
            
            # Retornar ángulos ajustados
            adjusted_angles = [phi, theta_shoulder, theta_elbow_adjusted, theta_wrist_adjusted, theta_gripper]
            
            self.get_logger().info(f'✅ IK 3D calculado para ({x:.3f}, {y:.3f}, {z:.3f})')
            self.get_logger().info(f'📐 Ajuste Z: {z_adjustment:.4f} rad ({math.degrees(z_adjustment):.1f}°) en ELBOW')
            self.get_logger().info(f'📐 Elbow: {math.degrees(theta_elbow):.1f}° → {math.degrees(theta_elbow_adjusted):.1f}°')
            self.get_logger().info(f'📐 Ángulos: {[f"{math.degrees(a):.1f}°" for a in adjusted_angles]}')
            
            return adjusted_angles
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en cálculo IK 3D: {e}')
            return None

    def save_positions_to_config(self, positions, config_file_path):
        """
        Guarda las posiciones calculadas en el archivo de configuración YAML
        """
        try:
            # Leer configuración existente
            with open(config_file_path, 'r') as file:
                config = yaml.safe_load(file)
            
            # Actualizar solo las posiciones de pick (mantener place y home existentes)
            for name, pos in positions.items():
                if name in ['pick_approach', 'pick_position'] and isinstance(pos, list):
                    config['joint_positions'][name] = [float(x) for x in pos]
            
            # Guardar archivo actualizado
            with open(config_file_path, 'w') as file:
                yaml.dump(config, file, default_flow_style=False, indent=2)
            
            self.get_logger().info(f'✅ Configuración actualizada en: {config_file_path}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Error guardando configuración: {e}')
            return False

    def print_positions_for_yaml(self, positions):
        """
        Imprime las posiciones en formato YAML para copiar y pegar
        """
        print("\n" + "="*50)
        print("POSICIONES CALCULADAS (formato YAML)")
        print("="*50)
        
        for name, pos in positions.items():
            pos_rounded = [round(float(p), 4) for p in pos]
            print(f"  {name}: {pos_rounded}")
        
        print("="*50)
        print("Estas posiciones han sido guardadas automáticamente")
        print("="*50 + "\n")

    def print_strategy_summary(self, strategy):
        """Imprime un resumen de la estrategia calculada"""
        print("\n" + "="*60)
        print("ESTRATEGIA CALCULADA")
        print("="*60)
        
        if strategy.get('needs_pick', True):
            print("✅ ESTRATEGIA: PICK DIRECTO")
            print(f"📋 Razón: {strategy.get('reason', 'Cálculo exitoso')}")
            print("\n✅ SECUENCIA ESTÁNDAR:")
            print("1. pick_approach → pick_position")
            print("2. place_approach → place_position")
        else:
            print("❌ ESTRATEGIA: OBJETO FUERA DE ALCANCE")
            print(f"📋 Razón: {strategy.get('reason', 'No se pudo calcular IK')}")
            print("\n❌ NO SE PUEDE EJECUTAR PICK AND PLACE")
        
        print("\n📐 POSICIONES CALCULADAS:")
        for name, pos in strategy.get('positions', {}).items():
            if isinstance(pos, list) and len(pos) == 5:
                pos_degrees = [f"{math.degrees(p):.1f}°" for p in pos]
                print(f"  {name}: {pos_degrees}")
        
        print("="*60 + "\n")

def main():
    rclpy.init()
    
    # Crear nodo
    ik_calc = InverseKinematicsCalculator()
    
    try:
        print("\n🤖 CALCULADORA DE CINEMÁTICA INVERSA PARA BRACCIO")
        print("="*60)
        
        if len(sys.argv) >= 4:
            # Usar argumentos de línea de comandos (solo objeto x, y, z)
            object_x = float(sys.argv[1])
            object_y = float(sys.argv[2])
            object_z = float(sys.argv[3])
            
            print(f"📍 Objeto en: ({object_x}, {object_y}, {object_z})")
            
            # Calcular estrategia solo pick
            strategy = ik_calc.calculate_strategy(object_x, object_y, object_z)
            if strategy['positions']:
                config_path = '/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_moveit_config/config/pick_and_place_config.yaml'
                if ik_calc.save_positions_to_config(strategy['positions'], config_path):
                    print("✅ ¡Configuración actualizada automáticamente!")
                    ik_calc.print_strategy_summary(strategy)
                else:
                    print("❌ Error guardando la configuración")
        
        else:
            # Modo interactivo
            print("💡 Uso: python3 script.py object_x object_y object_z")
            print("📍 Ejemplo: python3 script.py 0.17 0.0 0.025")
            
            # Usar valores por defecto para demostración
            object_x, object_y, object_z = 0.17, 0.0, 0.025
            
            print(f"\n� Probando con posición por defecto: ({object_x}, {object_y}, {object_z})")
            
            positions = ik_calc.calculate_pick_positions(object_x, object_y, object_z)
            
            if positions:
                ik_calc.print_positions_for_yaml(positions)
                
                # Preguntar si guardar
                save_choice = input("¿Guardar en archivo de configuración? (y/n): ").lower()
                if save_choice == 'y':
                    config_path = '/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_moveit_config/config/pick_and_place_config.yaml'
                    ik_calc.save_positions_to_config(positions, config_path)
                    print("✅ ¡Listo! Ahora puedes ejecutar el script configurable.")
            else:
                print("❌ No se pudieron calcular las posiciones.")
        
    except KeyboardInterrupt:
        print("\n👋 Interrumpido por el usuario")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        ik_calc.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
