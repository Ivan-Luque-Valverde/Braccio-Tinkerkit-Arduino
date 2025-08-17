# 🎮 Guía de Uso: Control de Braccio con Gamepad

## ✅ Método Recomendado (Simple)

Dado que ya tienes funcionando el brazo físico con MoveIt, este es el enfoque más directo:

### Paso 1: Iniciar el brazo físico (Terminal 1)
```bash
cd /home/ivan/Escritorio/Braccio-Tinkerkit-Arduino
source install/setup.bash
ros2 launch braccio_bringup bringup.launch.py sim:=false
```

**Esperar hasta ver**: `You can start planning now!`

### Paso 2: Agregar control por gamepad (Terminal 2)
```bash
cd /home/ivan/Escritorio/Braccio-Tinkerkit-Arduino
source install/setup.bash
ros2 launch braccio_gamepad_teleop gamepad_only.launch.py
```

**Deberías ver**: 
- `Opened joystick: Xbox 360 Controller`
- `MoveGroup inicializado. Gamepad teleop listo`

## 🎯 Controles del Gamepad

| Control | Función |
|---------|---------|
| **Stick Izquierdo** | |
| - Arriba/Abajo | Movimiento Y (adelante/atrás) |
| - Izquierda/Derecha | Movimiento X (izquierda/derecha) |
| **Stick Derecho** | |
| - Arriba/Abajo | Movimiento Z (arriba/abajo) |
| - Izquierda/Derecha | *Reservado para rotación* |
| **Botones** | |
| - A (Xbox) / X (PS) | Rotación positiva |
| - B (Xbox) / O (PS) | Rotación negativa |

## 🔧 Opciones Alternativas

### Opción A: Launch todo-en-uno
```bash
ros2 launch braccio_gamepad_teleop gamepad_control.launch.py
```
*Incluye bringup del brazo físico + gamepad*

### Opción B: Solo el nodo de teleoperación
```bash
# (Después de tener bringup funcionando)
ros2 run braccio_gamepad_teleop gamepad_teleop
```

## 🔍 Verificación

### Verificar gamepad conectado:
```bash
ls /dev/input/js*  # Debe mostrar /dev/input/js0
```

### Verificar datos del gamepad:
```bash
ros2 topic echo /joy --once
```

### Verificar estado de MoveGroup:
```bash
ros2 topic list | grep move_group
```

## ⚠️ Solución de Problemas

### "MoveGroup aún no inicializado"
- **Causa**: El brazo no está ejecutándose
- **Solución**: Verificar que `bringup.launch.py` esté corriendo y muestre "You can start planning now!"

### "No joystick found"
- **Causa**: Gamepad no detectado
- **Solución**: 
  1. Verificar conexión USB/Bluetooth
  2. Verificar permisos: `sudo chmod 666 /dev/input/js0`

### El brazo no se mueve
- **Causa**: Possible movimiento muy pequeño o problema de escalas
- **Solución**: Los movimientos son incrementales de 1cm por tick del gamepad

## 📊 Configuración Avanzada

### Ajustar sensibilidad:
```bash
ros2 param set /gamepad_teleop scale_linear 0.05  # Más lento
ros2 param set /gamepad_teleop scale_linear 0.2   # Más rápido
```

### Cambiar dispositivo del gamepad:
```bash
# Si tu gamepad está en /dev/input/js1
ros2 param set /joy_node dev /dev/input/js1
```

## 🚀 Scripts de Inicio Rápido

### Script para brazo físico + gamepad:
```bash
#!/bin/bash
# start_physical_robot_with_gamepad.sh

cd /home/ivan/Escritorio/Braccio-Tinkerkit-Arduino
source install/setup.bash

echo "Iniciando brazo físico y control por gamepad..."
ros2 launch braccio_gamepad_teleop gamepad_control.launch.py
```

### Script solo gamepad (para usar con bringup existente):
```bash
#!/bin/bash
# start_gamepad_only.sh

cd /home/ivan/Escritorio/Braccio-Tinkerkit-Arduino
source install/setup.bash

echo "Iniciando control por gamepad..."
ros2 launch braccio_gamepad_teleop gamepad_only.launch.py
```

## 📋 Lista de Verificación Pre-Uso

- [ ] Gamepad conectado y detectado en `/dev/input/js0`
- [ ] Brazo Braccio conectado por USB
- [ ] Arduino programado con el firmware correcto
- [ ] Entorno ROS2 cargado (`source install/setup.bash`)
- [ ] Workspace compilado (`colcon build`)

## 🎯 Flujo de Trabajo Recomendado

1. **Conectar hardware**: Gamepad y brazo Braccio
2. **Terminal 1**: `ros2 launch braccio_bringup bringup.launch.py sim:=false`
3. **Esperar inicialización**: "You can start planning now!"
4. **Terminal 2**: `ros2 launch braccio_gamepad_teleop gamepad_only.launch.py`
5. **Verificar**: "Gamepad teleop listo"
6. **¡Usar el gamepad para controlar el brazo!**

---

**💡 Tip**: El enfoque de dos terminales separadas es más robusto porque puedes reiniciar solo el gamepad si hay problemas, sin afectar el brazo físico.
