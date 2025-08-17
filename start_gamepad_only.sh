#!/bin/bash

# Script para agregar control de gamepad al brazo físico funcionando
cd /home/ivan/Escritorio/Braccio-Tinkerkit-Arduino
source install/setup.bash

echo "🎮 Iniciando control por gamepad para Braccio..."
echo "💡 Asegúrate de que el brazo físico esté ejecutándose con:"
echo "   ros2 launch braccio_bringup bringup.launch.py sim:=false"
echo ""
echo "🎯 Controles del gamepad:"
echo "   Stick Izquierdo: Movimiento X,Y"
echo "   Stick Derecho: Movimiento Z"
echo "   Botón A/X: Rotación +"
echo "   Botón B/O: Rotación -"
echo ""

ros2 launch braccio_gamepad_teleop gamepad_only.launch.py
