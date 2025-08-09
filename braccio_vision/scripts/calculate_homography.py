#!/usr/bin/env python3
"""
Script para calcular la matriz de homografía a partir de 4 puntos en píxeles y sus correspondientes coordenadas en el mundo.
Guarda el resultado en camera_calibration.json.
"""
import numpy as np
import cv2
import json

# === EDITA AQUÍ TUS PUNTOS ===
# Puntos en píxeles (x, y) - orden: esquina1, esquina2, esquina3, esquina4
pixel_points = [
    [375, 365],    # esquina1
    [263, 365],   # esquina2
    [263, 278],  # esquina3
    [375, 278],   # esquina4
]

# Puntos en el mundo real (x, y) en metros - deben corresponder al orden de arriba
world_points = [
    [0.08, -0.12],  # esquina1
    [0.27, -0.12],   # esquina2
    [0.27, 0.12],    # esquina3
    [0.08, 0.12],   # esquina4
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
