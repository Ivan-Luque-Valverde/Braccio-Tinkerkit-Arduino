#!/bin/bash

echo "🎮 Iniciando control DIRECTO del gamepad para Braccio..."
echo "🔧 NUEVO: Sin MoveIt - Control directo del hardware"
echo ""
echo "⚠️  IMPORTANTE: Usa este comando JUNTO CON el robot físico:"
echo "   Terminal 1: ros2 launch braccio_bringup bringup.launch.py sim:=false"
echo "   Terminal 2: $0"
echo ""
echo "🎯 Controles del gamepad:"
echo "   Stick Izquierdo X: Base rotation"
echo "   Stick Izquierdo Y: Shoulder"  
echo "   Stick Derecho Y:   Elbow"
echo "   Botón A/X:        Wrist arriba"
echo "   Botón B/O:        Wrist abajo"
echo ""

ros2 launch braccio_gamepad_teleop gamepad_direct.launch.py
