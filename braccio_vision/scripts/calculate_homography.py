#!/usr/bin/env python3
"""
Script para calcular la matriz de homografía a partir de 4 puntos en píxeles y sus correspondientes coordenadas en el mundo.
Guarda el resultado en camera_calibration.json.
"""
import numpy as np
import cv2
import json

# === EDITA AQUÍ TUS PUNTOS ===
# Puntos en píxeles (y, x) - orden: esquina1, esquina2, esquina3, esquina4
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

# Convertir a np.float32
pixel_points = np.array(pixel_points, dtype=np.float32)
world_points = np.array(world_points, dtype=np.float32)

# Calcular la homografía
H, status = cv2.findHomography(pixel_points, world_points)

print("Matriz de homografía calculada:")
print(H)

# Guardar en archivo JSON
calib = {"homography_matrix": H.tolist()}
with open("/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_vision/config/camera_calibration.json", "w") as f:
    json.dump(calib, f, indent=2)
print("Homografía guardada en braccio_vision/config/camera_calibration.json")
