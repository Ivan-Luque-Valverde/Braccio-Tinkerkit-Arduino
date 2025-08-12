#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

class IKWorkspaceTester(Node):
    def __init__(self):
        super().__init__('ik_workspace_tester')
        self.L = 0.3025  # Longitud del brazo principal (m) 125+116.5+61
        self.l = 0.064  # Offset desde la base (m)
        self.THETA_EXT = 0.27  # Ángulo mínimo del shoulder (radianes)
        self.THETA_RET = math.pi/4  # Ángulo máximo del shoulder (radianes)

    def calculate_ik(self, x, y):
        s = math.sqrt(x**2 + y**2)
        phi = math.atan2(y, x)
        cos_theta = (s - self.l) / self.L
        if abs(cos_theta) > 1.0:
            return None, 'Fuera de alcance'
        theta_shoulder = math.acos(cos_theta)
        if theta_shoulder < self.THETA_EXT:
            return None, 'Demasiado lejos (theta_shoulder < THETA_EXT)'
        if theta_shoulder > self.THETA_RET:
            return None, 'Demasiado cerca (theta_shoulder > THETA_RET)'
        theta_wrist = theta_shoulder + math.pi/2
        theta_elbow = math.pi/2 - 2*theta_shoulder
        theta_gripper = math.pi/2
        angles = [phi, theta_shoulder, theta_elbow, theta_wrist, theta_gripper]
        return angles, 'OK'

def main():
    rclpy.init()
    node = IKWorkspaceTester()
    print('\n=== TESTER DE WORKSPACE DE PICK DIRECTO ===')
    print('x (m)\ty (m)\tResultado\t\tÁngulos (grados)')

    # Rango ampliado: X desde 0.20 hasta 0.35m, Y desde 0 hasta 0.35m
    x_vals = [round(x, 3) for x in list(frange(0.20, 0.36, 0.01))]
    y_vals = [round(y, 3) for y in list(frange(0, 0.36, 0.01))]
    # Rango seguro: X desde 0.20 hasta 0.30m, Y desde 0.17 hasta 0.25m 
    pick_directo_count = 0
    total_count = 0
    
    for x in x_vals:
        for y in y_vals:
            total_count += 1
            angles, status = node.calculate_ik(x, y)
            if status == 'OK':
                pick_directo_count += 1
                angles_deg = [math.degrees(angle) for angle in angles]
                print(f'{x:.3f}\t{y:.3f}\tPICK DIRECTO\t\t[{angles_deg[0]:.1f}, {angles_deg[1]:.1f}, {angles_deg[2]:.1f}, {angles_deg[3]:.1f}, {angles_deg[4]:.1f}]')
            else:
                print(f'{x:.3f}\t{y:.3f}\t{status}')
    
    print(f'\n📊 RESUMEN:')
    print(f'   Total de posiciones probadas: {total_count}')
    print(f'   Posiciones PICK DIRECTO: {pick_directo_count}')
    print(f'   Porcentaje de éxito: {(pick_directo_count/total_count)*100:.1f}%')
    print('\n🎯 POSICIONES RECOMENDADAS PARA CUBO VERDE:')
    
    
    node.destroy_node()
    rclpy.shutdown()

def frange(start, stop, step):
    while start <= stop:
        yield start
        start += step

if __name__ == '__main__':
    main()
