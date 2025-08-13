#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import json
import sys
import numpy as np
import yaml
import os
import time

# Constantes del Braccio (basadas en el script original)
THETA_EXT = 0.27  # Ángulo mínimo del shoulder (radianes)
THETA_RET = np.pi/4  # Ángulo máximo del shoulder (radianes)

# Límites del joint base (de braccio_description.urdf.xacro)
JOINT_BASE_LOWER_LIMIT = 0.0    # 0°
JOINT_BASE_UPPER_LIMIT = np.pi  # 180°

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
        Incluye lógica de configuración simétrica para ampliar workspace
        Retorna [phi, theta_shoulder, theta_elbow, theta_wrist, theta_gripper]
        """
        try:
            # Convertir a coordenadas polares
            rho, phi = cart2pol(x, y)
            
            # NUEVA LÓGICA: Verificar si phi está dentro de los límites del joint_base
            phi_original = phi
            configuration_type = "ORIGINAL"
            
            if phi < JOINT_BASE_LOWER_LIMIT or phi > JOINT_BASE_UPPER_LIMIT:
                # Calcular configuración simétrica corregida
                # Para ángulos negativos: usar π + |phi| = π - phi (ya que phi es negativo)
                # Para ángulos > π: usar phi - π (reflejar al otro lado)
                if phi < 0:
                    phi_symmetric = np.pi + phi  # Equivale a π - |phi| cuando phi es negativo
                else:
                    phi_symmetric = phi - np.pi  # Para ángulos > π
                
                # Normalizar a rango [0, 2π] si es necesario
                while phi_symmetric < 0:
                    phi_symmetric += 2 * np.pi
                while phi_symmetric > 2 * np.pi:
                    phi_symmetric -= 2 * np.pi
                
                # Verificar si la configuración simétrica está dentro de límites
                if JOINT_BASE_LOWER_LIMIT <= phi_symmetric <= JOINT_BASE_UPPER_LIMIT:
                    phi = phi_symmetric
                    configuration_type = "SIMÉTRICA"
                    self.get_logger().info(f'🔄 Configuración simétrica: φ {math.degrees(phi_original):.1f}° → {math.degrees(phi):.1f}°')
                else:
                    self.get_logger().error(f'❌ Ninguna configuración válida: φ_orig={math.degrees(phi_original):.1f}°, φ_sim={math.degrees(phi_symmetric):.1f}°')
                    return None
            
            # Calcular ángulo del shoulder
            cos_theta = (rho - self.l) / self.L
            
            # Verificar que esté en el dominio válido
            if cos_theta < np.cos(THETA_RET) or cos_theta > np.cos(THETA_EXT):
                self.get_logger().error(f'❌ Posición fuera del dominio: cos_theta={cos_theta:.3f}')
                return None
            
            theta_shoulder = np.arccos(cos_theta)
            
            # Calcular otros ángulos basados en shoulder
            theta_wrist, theta_elbow = get_other_angles(theta_shoulder)
            
            # Gripper vertical (valor por defecto)
            theta_gripper = np.pi/2
            
            # IMPORTANTE: Para configuración simétrica, solo invertir base y shoulder
            if configuration_type == "SIMÉTRICA":
                self.get_logger().info(f'🔄 Configuración simétrica: invirtiendo SOLO base y shoulder...')
                theta_shoulder_original = theta_shoulder
                theta_shoulder = np.pi - theta_shoulder
                # Elbow, wrist y gripper se mantienen IGUALES
                
                self.get_logger().info(f'🔄 Configuración simétrica aplicada:')
                self.get_logger().info(f'   Base: {math.degrees(phi_original):.1f}° → {math.degrees(phi):.1f}°')
                self.get_logger().info(f'   Shoulder: {math.degrees(theta_shoulder_original):.1f}° → {math.degrees(theta_shoulder):.1f}°')
                self.get_logger().info(f'   Elbow: SIN CAMBIOS ({math.degrees(theta_elbow):.1f}°)')
                self.get_logger().info(f'   Wrist: SIN CAMBIOS ({math.degrees(theta_wrist):.1f}°)')
                self.get_logger().info(f'   Gripper: SIN CAMBIOS ({math.degrees(theta_gripper):.1f}°)')
            
            # Retornar ángulos en el orden correcto para el Braccio
            angles = [phi, theta_shoulder, theta_elbow, theta_wrist, theta_gripper]
            
            self.get_logger().info(f'✅ IK calculado para ({x:.3f}, {y:.3f}) - Config: {configuration_type}')
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

    def calculate_pick_positions(self, object_x, object_y, object_z, approach_height=0.03):
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
        
        # 2. Posición de agarre (ligeramente por encima del objeto para evitar empujarlo)
        pick_z_safe = object_z   
        pick_position = self.calculate_ik_xyz(object_x, object_y, pick_z_safe)
        if pick_position:
            positions['pick_position'] = pick_position
            self.get_logger().info(f'✅ Pick position calculado para Z={pick_z_safe:.3f}m (objeto en {object_z:.3f}m + 5mm de seguridad)')
        else:
            self.get_logger().error(f'❌ No se pudo calcular pick_position para Z={pick_z_safe:.3f}m')
            return None
        
        self.get_logger().info('=== CÁLCULOS DE PICK COMPLETADOS ===')
        actual_height_diff = approach_z - pick_z_safe
        self.get_logger().info(f'📏 Diferencia de altura: {actual_height_diff:.3f}m entre approach y position')
        self.get_logger().info(f'📏 Approach Z: {approach_z:.3f}m | Pick Z: {pick_z_safe:.3f}m | Objeto Z: {object_z:.3f}m')
        
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
            
            # Verificar si estamos en configuración simétrica para ajustar la lógica de altura
            rho, phi_check = cart2pol(x, y)
            is_symmetric = phi_check < JOINT_BASE_LOWER_LIMIT or phi_check > JOINT_BASE_UPPER_LIMIT
            
            if is_symmetric:
                # Para configuración simétrica, usar altura Z para calcular ajuste dinámico
                self.get_logger().info(f'🔧 Configuración simétrica detectada - ajuste dinámico basado en Z={z:.3f}m')
                
                # Altura de referencia para posición base en configuración simétrica
                z_reference_symmetric = 0.08  # Altura de referencia aún más alta para simétrica
                
                # Factor de ajuste dinámico basado en la altura solicitada
                # Z más baja requiere MUCHA más reducción del elbow para bajar el efector
                z_diff = z - z_reference_symmetric
                base_reduction = 1.0  # Reducción base MUCHO mayor (unos 57°)
                height_factor = -25.0  # Factor multiplicador MÁS AGRESIVO para diferencias de altura
                
                elbow_reduction = base_reduction + (z_diff * height_factor)
                
                # Limitar la reducción a rangos seguros pero permitir MÁS reducción
                elbow_reduction = max(0.3, min(2.0, elbow_reduction))
                
                # NUEVO: También ajustar el SHOULDER para bajar más
                # Para alturas muy bajas, aumentar el shoulder para que el brazo baje más
                shoulder_adjustment = 0.0
                if z < 0.06:  # Umbral más alto para activar ajuste (< 6cm)
                    shoulder_adjustment = (0.06 - z) * 20.0  # Factor MUCHO MÁS AGRESIVO para bajar
                    shoulder_adjustment = min(shoulder_adjustment, 1.2)  # Limitar a ~69° (mucho más que antes)
                    self.get_logger().info(f'🔧 Altura baja ({z:.3f}m) - aumentando shoulder AGRESIVAMENTE en {math.degrees(shoulder_adjustment):.1f}°')
                
                theta_shoulder_adjusted = theta_shoulder + shoulder_adjustment
                # Limitar shoulder a rango válido
                theta_shoulder_adjusted = max(0.1, min(np.pi - 0.1, theta_shoulder_adjusted))
                
                # NUEVO ENFOQUE: En lugar de reducir elbow drasticamente, 
                # hacer que elbow tenga orientación similar al shoulder para bajar correctamente
                # Para configuración simétrica y alturas bajas, elbow debe seguir al shoulder
                if z < 0.06:  # Para alturas bajas
                    # Hacer que elbow tenga orientación similar al shoulder ajustado
                    # Factor de alineación: 0.0 = elbow independiente, 1.0 = elbow = shoulder
                    alignment_factor = min(0.8, (0.06 - z) * 12.0)  # Factor más suave para menos rigidez
                    
                    # Calcular elbow alineado con shoulder
                    theta_elbow_aligned = theta_shoulder_adjusted * alignment_factor + theta_elbow * (1.0 - alignment_factor)
                    theta_elbow_adjusted = theta_elbow_aligned
                    
                    self.get_logger().info(f'🔧 Alineando elbow con shoulder: factor={alignment_factor:.2f}')
                    self.get_logger().info(f'🔧 Elbow alineado: {math.degrees(theta_elbow):.1f}° → {math.degrees(theta_elbow_adjusted):.1f}° (similar a shoulder {math.degrees(theta_shoulder_adjusted):.1f}°)')
                else:
                    # Para alturas normales, usar la reducción original pero más moderada
                    elbow_reduction_moderate = elbow_reduction * 0.3  # Reducir menos agresivamente
                    theta_elbow_adjusted = theta_elbow - elbow_reduction_moderate
                    self.get_logger().info(f'🔧 Reducción moderada de elbow: {math.degrees(elbow_reduction_moderate):.1f}°')
                
                # Limitar elbow a rango válido
                theta_elbow_adjusted = max(0.1, min(np.pi - 0.1, theta_elbow_adjusted))
                
                # Ajustar wrist para compensar AMBOS cambios (shoulder y elbow alineado)
                if z < 0.06:
                    # Para alturas bajas con elbow alineado, wrist debe compensar la nueva configuración
                    # Y además bajar extra para que el gripper pueda agarrar el objeto
                    wrist_compensation = (theta_shoulder_adjusted - theta_shoulder) * 0.2 + (theta_elbow_adjusted - theta_elbow) * 0.3
                    extra_wrist_down = 0.5  # Radianes extra (~29°) para bajar MÁS el gripper
                    theta_wrist_adjusted = theta_wrist + wrist_compensation + extra_wrist_down
                    
                    self.get_logger().info(f'🔧 Wrist bajado extra {math.degrees(extra_wrist_down):.1f}° para mejor agarre')
                else:
                    # Para alturas normales, compensación estándar
                    theta_wrist_adjusted = theta_wrist + elbow_reduction * 0.5 - shoulder_adjustment * 0.3
                
                z_adjustment = -(theta_elbow_adjusted - theta_elbow)  # Ajuste basado en cambio real de elbow
                
                self.get_logger().info(f'🔧 Z={z:.3f}m, ref={z_reference_symmetric:.3f}m, diff={z_diff:.3f}m')
                self.get_logger().info(f'🔧 Shoulder: {math.degrees(theta_shoulder):.1f}° → {math.degrees(theta_shoulder_adjusted):.1f}° (ajuste: {math.degrees(shoulder_adjustment):.1f}°)')
                self.get_logger().info(f'🔧 Elbow reducido en {math.degrees(elbow_reduction):.1f}° (base={math.degrees(base_reduction):.1f}° + altura)')
                self.get_logger().info(f'🔧 Elbow: {math.degrees(theta_elbow):.1f}° → {math.degrees(theta_elbow_adjusted):.1f}°')
                
                # Usar los ángulos ajustados en lugar de los originales
                phi_final = phi
                theta_shoulder_final = theta_shoulder_adjusted
                theta_elbow_final = theta_elbow_adjusted
                theta_wrist_final = theta_wrist_adjusted
                theta_gripper_final = theta_gripper
            else:
                # Para configuración normal, usar ajustes estándar
                z_nominal = 0.05  # Altura de referencia estándar
                # Factor de sensibilidad DINÁMICO basado en el ángulo del shoulder
                shoulder_deg = math.degrees(theta_shoulder)
                if shoulder_deg < 20.0:  # Shoulder muy bajo (brazo muy extendido)
                    z_factor = 12.0      # Factor más agresivo para posiciones alejadas
                    self.get_logger().info(f'🎯 Shoulder bajo ({shoulder_deg:.1f}°) - usando z_factor agresivo: {z_factor}')
                elif shoulder_deg < 25.0:  # Shoulder medio-bajo
                    z_factor = 10.0
                    self.get_logger().info(f'🎯 Shoulder medio-bajo ({shoulder_deg:.1f}°) - usando z_factor medio: {z_factor}')
                else:  # Shoulder más alto (posiciones más cercanas)
                    z_factor = 8.0       # Factor estándar
                    self.get_logger().info(f'🎯 Shoulder normal ({shoulder_deg:.1f}°) - usando z_factor estándar: {z_factor}')
                
                # Calcular ajuste: Z menor debe dar elbow menor para bajar efector
                z_adjustment = (z - z_nominal) * z_factor
                theta_elbow_adjusted = theta_elbow + z_adjustment
                
                # Limitar elbow a rango válido (0.1 a π-0.1)
                theta_elbow_adjusted = max(0.1, min(np.pi - 0.1, theta_elbow_adjusted))
                
                # Recalcular wrist para mantener orientación (compensar cambio en elbow)
                theta_wrist_adjusted = theta_shoulder + np.pi/2 + (theta_elbow_adjusted - theta_elbow) * 0.5
                
                # Para configuración normal, no ajustar shoulder
                phi_final = phi
                theta_shoulder_final = theta_shoulder
                theta_elbow_final = theta_elbow_adjusted
                theta_wrist_final = theta_wrist_adjusted
                theta_gripper_final = theta_gripper
            
            # Retornar ángulos ajustados usando las variables finales
            adjusted_angles = [phi_final, theta_shoulder_final, theta_elbow_final, theta_wrist_final, theta_gripper_final]
            
            config_type = "SIMÉTRICA" if is_symmetric else "ORIGINAL"
            self.get_logger().info(f'✅ IK 3D calculado para ({x:.3f}, {y:.3f}, {z:.3f}) - Config: {config_type}')
            if is_symmetric:
                self.get_logger().info(f'📐 Ajuste Z: {z_adjustment:.4f} rad ({math.degrees(z_adjustment):.1f}°) en ELBOW')
            else:
                self.get_logger().info(f'📐 Ajuste Z: {z_adjustment:.4f} rad ({math.degrees(z_adjustment):.1f}°) en ELBOW')
            self.get_logger().info(f'📐 Ángulos finales: {[f"{math.degrees(a):.1f}°" for a in adjusted_angles]}')
            
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
            
            # Guardar archivo actualizado con flush forzado
            with open(config_file_path, 'w') as file:
                yaml.dump(config, file, default_flow_style=False, indent=2)
                file.flush()  # Forzar escritura inmediata
                os.fsync(file.fileno())  # Sincronizar con disco
            
            # Esperar un momento adicional para asegurar escritura completa
            time.sleep(0.5)
            
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
        print("  # Formato: [radianes]   |   [grados]")
        for name, pos in strategy.get('positions', {}).items():
            if isinstance(pos, list) and len(pos) == 5:
                pos_rounded = [round(float(p), 4) for p in pos]
                pos_degrees = [round(math.degrees(float(p)), 1) for p in pos]
                print(f"  {name}: {pos_rounded}   |   {pos_degrees}")
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
