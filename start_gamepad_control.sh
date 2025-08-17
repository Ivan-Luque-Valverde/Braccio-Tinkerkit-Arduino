#!/bin/bash

# Cambiar al directorio del workspace
cd /home/ivan/Escritorio/Braccio-Tinkerkit-Arduino

# Cargar el entorno ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash

# Verificar que el paquete esté disponible
echo "Verificando paquetes disponibles..."
ros2 pkg list | grep braccio_gamepad_teleop

# Ejecutar el launch
echo "Iniciando sistema de teleoperación con gamepad..."
ros2 launch braccio_gamepad_teleop gamepad_control.launch.py
