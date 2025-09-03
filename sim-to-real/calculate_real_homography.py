#!/usr/bin/env python3
"""
Script para calcular la matriz de homografía a partir de 4 puntos en píxeles y sus correspondientes coordenadas en el mundo.
Guarda el resultado en camera_calibration.json.
"""
import numpy as np
import cv2
import json

# === EDITA AQUÍ TUS PUNTOS ===
# Puntos en píxeles (y, x) - orden: esquina1, esquina2, esquina3, esquina4, punto_objeto
pixel_points = [
    [115, 98],    
    [297,107],
    [497,113],
    [180,165],
    [412,179],
    [106,222],
    [288,233],
    [489,246],
    [169,282],
    [406,301],
    [97,344],
    [279,361],
    [461,381],
]

# Puntos en el mundo real (x, y) en metros - deben corresponder al orden de arriba
world_points = [
    [-0.325, -0.215],
    [0,-0.215],
    [0.325, -0.215],
    [-0.2, -0.1],
    [0.2, -0.1],
    [-0.325, 0.0],
    [0.0, 0.0],
    [0.325, 0.0],
    [-0.2, 0.1],
    [0.2, 0.1],
    [-0.325, 0.215],
    [0.0, 0.215],
    [0.325, 0.215]

]

# Convertir a np.float32
pixel_points = np.array(pixel_points, dtype=np.float32)
world_points = np.array(world_points, dtype=np.float32)

# Calcular la homografía usando RANSAC para manejar múltiples puntos
H, status = cv2.findHomography(pixel_points, world_points, cv2.RANSAC, 0.01)

print("Matriz de homografía calculada (con 5 puntos):")
print(H)

# Verificar el punto problemático
test_pixel = np.array([[460, 156]], dtype=np.float32).reshape(-1, 1, 2)
projected_world = cv2.perspectiveTransform(test_pixel, H)
print(f"\nVerificación:")
print(f"Píxel (460,156) -> Mundo {projected_world[0][0]}")
print(f"Esperado: (0.25, -0.15)")
print(f"Error: {np.linalg.norm(projected_world[0][0] - np.array([0.25, -0.15])):.4f}m")

# Guardar en archivo JSON
calib = {"homography_matrix": H.tolist()}
with open("/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_vision/config/camera_calibration.json", "w") as f:
    json.dump(calib, f, indent=2)
print("Homografía guardada en braccio_vision/config/camera_calibration.json")
