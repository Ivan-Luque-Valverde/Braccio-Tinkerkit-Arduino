#!/usr/bin/env python3

import math

def test_position_angles():
    """Calcula los ángulos aproximados para posición (0.2, 0.1, 0.08)"""
    
    x, y, z = 0.2, 0.1, 0.08
    
    # Base rotation - aquí podría estar el problema
    joint_base = math.atan2(y, x)
    print(f"joint_base (base): {joint_base:.3f} rad = {math.degrees(joint_base):.1f}°")
    print(f"Límites base: 0.0 a 3.1416 rad (0° a 180°)")
    print(f"Base OK: {0.0 <= joint_base <= 3.1416}")
    print()
    
    # Distancia horizontal desde base
    r = math.sqrt(x*x + y*y)
    print(f"Distancia horizontal r: {r:.3f} m")
    
    # Para joint_1 (shoulder), necesitamos alcanzar altura z
    # Aproximación simple: si el brazo está extendido horizontalmente
    # joint_1 controla elevación
    
    # Longitudes aproximadas del brazo (revisar URDF)
    L1 = 0.125  # link_1 length
    L2 = 0.125  # link_2 length  
    L3 = 0.195  # link_3 length
    
    total_reach = L1 + L2 + L3
    print(f"Alcance total estimado: {total_reach:.3f} m")
    
    # Ángulo aproximado para joint_1 (elevación del brazo)
    # Si todo el brazo apunta hacia el objetivo
    angle_to_target = math.atan2(z, r)
    print(f"Ángulo hacia objetivo: {angle_to_target:.3f} rad = {math.degrees(angle_to_target):.1f}°")
    
    # joint_1 en configuración típica necesita estar alrededor de π/2 para horizontal
    # Valores típicos para alcanzar hacia adelante
    joint_1_needed = math.pi/2 + angle_to_target
    print(f"joint_1 necesario (aprox): {joint_1_needed:.3f} rad = {math.degrees(joint_1_needed):.1f}°")
    print(f"Límites joint_1: 0.4 a 2.7 rad (23° a 155°)")
    print(f"joint_1 OK: {0.4 <= joint_1_needed <= 2.7}")
    print()
    
    # Probemos una posición más conservadora
    print("=== POSICIÓN SUGERIDA (más conservadora) ===")
    x_new, y_new, z_new = 0.15, 0.05, 0.12
    
    joint_base_new = math.atan2(y_new, x_new)
    r_new = math.sqrt(x_new*x_new + y_new*y_new)
    angle_new = math.atan2(z_new, r_new)
    joint_1_new = math.pi/2 + angle_new
    
    print(f"Posición: ({x_new}, {y_new}, {z_new})")
    print(f"joint_base: {joint_base_new:.3f} rad = {math.degrees(joint_base_new):.1f}° (OK: {0.0 <= joint_base_new <= 3.1416})")
    print(f"joint_1: {joint_1_new:.3f} rad = {math.degrees(joint_1_new):.1f}° (OK: {0.4 <= joint_1_new <= 2.7})")

if __name__ == "__main__":
    test_position_angles()
