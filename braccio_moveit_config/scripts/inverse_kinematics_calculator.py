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
        self.L = 0.125  # Longitud del brazo principal (metros)
        self.l = 0.095  # Offset desde la base (metros)
        
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

    def should_push_object(self, x, y):
        """Determina si el objeto necesita ser empujado basado en IK analítica"""
        targets = self.calculate_ik_xy(x, y)
        if targets is None:
            return True, "No se puede calcular IK"
        
        theta_shoulder = targets[1]
        
        # Fuera del dominio válido (muy lejos)
        if theta_shoulder < THETA_EXT:
            return False, f"Fuera del dominio: theta={math.degrees(theta_shoulder):.1f}° < {math.degrees(THETA_EXT):.1f}°"
        
        # Muy cerca, necesita empuje
        if theta_shoulder > THETA_RET:
            return True, f"Muy cerca, empujando: theta={math.degrees(theta_shoulder):.1f}° > {math.degrees(THETA_RET):.1f}°"
        
        # En rango válido
        return False, f"En rango válido: theta={math.degrees(theta_shoulder):.1f}°"

    def calculate_push_strategy(self, object_x, object_y, object_z):
        """
        Calcula la estrategia completa: empuje + pick o pick directo
        """
        self.get_logger().info('=== ANALIZANDO ESTRATEGIA ===')
        
        needs_push, reason = self.should_push_object(object_x, object_y)
        self.get_logger().info(f'📊 Análisis: {reason}')
        
        strategy = {
            'needs_push': needs_push,
            'reason': reason,
            'positions': {}
        }
        
        if needs_push:
            # Calcular posiciones de empuje
            base_angle = math.atan2(object_y, object_x)
            strategy['positions']['push_base_angle'] = base_angle
            strategy['positions']['push_raise'] = [base_angle, 1.15, 0.13, 2.29, np.pi/2]
            strategy['positions']['push_sequence'] = [
                [base_angle, 2.7, 0.01, 0.01, np.pi/2],
                [base_angle, 2.1, 0.01, 0.01, np.pi/2],
                [base_angle, 0.5, 1.8, 0.1, np.pi/2],
                [base_angle, 2.1, 0.01, 0.01, np.pi/2],
                [base_angle, 2.7, 0.01, 0.01, np.pi/2]
            ]
            
            # Estimar nueva posición después del empuje (más cerca de la zona cómoda)
            rho, phi = cart2pol(object_x, object_y)
            new_rho = self.l + self.L * np.cos((THETA_EXT + THETA_RET) / 2)  # Posición central
            new_x, new_y = pol2cart(new_rho, phi)
            
            # Calcular pick para la nueva posición
            pick_positions = self.calculate_pick_positions(new_x, new_y, object_z)
            if pick_positions:
                strategy['positions'].update(pick_positions)
            
            self.get_logger().info(f'🔄 Estrategia: EMPUJE + PICK')
            self.get_logger().info(f'📍 Posición original: ({object_x:.3f}, {object_y:.3f})')
            self.get_logger().info(f'🎯 Posición estimada post-empuje: ({new_x:.3f}, {new_y:.3f})')
            
        else:
            # Pick directo
            pick_positions = self.calculate_pick_positions(object_x, object_y, object_z)
            if pick_positions:
                strategy['positions'].update(pick_positions)
            
            self.get_logger().info(f'✅ Estrategia: PICK DIRECTO')
        
        return strategy

    def calculate_pick_positions(self, object_x, object_y, object_z, approach_height=0.03):
        """
        Calcula las posiciones para pick (approach y position)
        Solo calcula para la posición del objeto detectado
        """
        self.get_logger().info('=== CALCULANDO POSICIONES DE PICK ===')
        
        positions = {}
        
        # Validar workspace
        if not self.validate_workspace(object_x, object_y, object_z):
            self.get_logger().error(f'❌ Objeto fuera del workspace: ({object_x:.3f}, {object_y:.3f}, {object_z:.3f})')
            return None
        
        # 1. Posición de aproximación al objeto
        pick_approach = self.calculate_ik_xy(object_x, object_y)
        if pick_approach:
            positions['pick_approach'] = pick_approach
            self.get_logger().info('✅ Pick approach calculado')
        else:
            self.get_logger().error('❌ No se pudo calcular pick_approach')
            return None
        
        # 2. Posición exacta del objeto (mismos ángulos que approach)
        positions['pick_position'] = pick_approach.copy()
        self.get_logger().info('✅ Pick position calculado (igual que approach)')
        
        self.get_logger().info('=== CÁLCULOS DE PICK COMPLETADOS ===')
        return positions

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
        
        if strategy['needs_push']:
            print("🔄 ESTRATEGIA: EMPUJE + PICK")
            print(f"📋 Razón: {strategy['reason']}")
            print("\n🔄 SECUENCIA RECOMENDADA:")
            print("1. Ejecutar secuencia de empuje")
            print("2. Detectar nueva posición del objeto")
            print("3. Ejecutar pick desde nueva posición")
        else:
            print("✅ ESTRATEGIA: PICK DIRECTO")
            print(f"📋 Razón: {strategy['reason']}")
            print("\n✅ SECUENCIA ESTÁNDAR:")
            print("1. pick_approach → pick_position")
            print("2. place_approach → place_position")
        
        print("\n📐 POSICIONES CALCULADAS:")
        for name, pos in strategy['positions'].items():
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
            
            # Calcular estrategia completa
            strategy = ik_calc.calculate_push_strategy(object_x, object_y, object_z)
            
            if strategy['positions']:
                # Guardar automáticamente cuando se llama desde otro script
                config_path = '/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_moveit_config/config/pick_and_place_config.yaml'
                if ik_calc.save_positions_to_config(strategy['positions'], config_path):
                    print("✅ ¡Configuración actualizada automáticamente!")
                    ik_calc.print_strategy_summary(strategy)
                else:
                    print("❌ Error guardando la configuración")
            else:
                print("❌ No se pudieron calcular las posiciones")
        
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
