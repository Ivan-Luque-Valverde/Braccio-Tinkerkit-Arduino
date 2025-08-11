#!/usr/bin/env python3
"""
Script para verificar la homografía calculada.
Dado un conjunto de puntos en píxeles y su correspondencia en el mundo,
proyecta ambos sentidos y muestra el error de reproyección.
"""
import numpy as np
import cv2
import json

# === EDITA AQUÍ TUS PUNTOS ===
# Puntos en píxeles (y, x) - mismo orden que en calculate_homography.py
pixel_points = [
    [202, 75],    # esquina1
    [436, 75],   # esquina2
    [436, 403],  # esquina3
    [202, 403],   # esquina4
    [320, 240],  # centro de la imagen (opcional)
]

# Puntos en el mundo real (x, y) en metros - deben corresponder al orden de arriba
world_points = [
    [-0.35, -0.25],  # esquina1
    [-0.35, 0.25],   # esquina2
    [0.35, 0.25],    # esquina3
    [0.35, -0.25],   # esquina4
    [0.0, 0.0],     # centro de la imagen (opcional)
]

# Cargar la matriz de homografía
with open("/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_vision/config/camera_calibration.json", "r") as f:
    calib = json.load(f)
H = np.array(calib["homography_matrix"])

print("Matriz de homografía cargada:")
print(H)

# Proyección de píxel a mundo
print("\nProyección de píxel a mundo:")
for i, px in enumerate(pixel_points):
    px_h = np.array([px[0], px[1], 1.0])
    world_proj = H @ px_h
    world_proj = world_proj / world_proj[2]
    print(f"Píxel {px} -> Mundo estimado: ({world_proj[0]:.4f}, {world_proj[1]:.4f}) | Mundo real: {world_points[i]}")

# Proyección inversa de mundo a píxel
print("\nProyección inversa de mundo a píxel:")
H_inv = np.linalg.inv(H)
for i, wp in enumerate(world_points):
    wp_h = np.array([wp[0], wp[1], 1.0])
    pixel_proj = H_inv @ wp_h
    pixel_proj = pixel_proj / pixel_proj[2]
    print(f"Mundo {wp} -> Píxel estimado: ({pixel_proj[0]:.1f}, {pixel_proj[1]:.1f}) | Píxel real: {pixel_points[i]}")

# Cálculo de error medio
print("\nErrores de reproyección (píxel -> mundo -> píxel):")
total_error = 0.0
for i, px in enumerate(pixel_points):
    px_h = np.array([px[0], px[1], 1.0])
    world_proj = H @ px_h
    world_proj = world_proj / world_proj[2]
    wp_h = np.array([world_proj[0], world_proj[1], 1.0])
    pixel_back = H_inv @ wp_h
    pixel_back = pixel_back / pixel_back[2]
    error = np.linalg.norm(np.array(px) - pixel_back[:2])
    total_error += error
    print(f"Píxel {px} -> Mundo ({world_proj[0]:.4f}, {world_proj[1]:.4f}) -> Píxel ({pixel_back[0]:.1f}, {pixel_back[1]:.1f}) | Error: {error:.2f}")
print(f"\nError medio de reproyección: {total_error/len(pixel_points):.2f} píxeles")

# === TEST MANUAL DE UN PUNTO ===
# Cambia estos valores por el punto que quieras probar
test_pixel = [319, 310]  # Ejemplo: centro de la imagen (y,x)

px_h = np.array([test_pixel[0], test_pixel[1], 1.0])
world_proj = H @ px_h
world_proj = world_proj / world_proj[2]
print(f"\nTest manual: Píxel {test_pixel} -> Mundo estimado: ({world_proj[0]:.4f}, {world_proj[1]:.4f})")

