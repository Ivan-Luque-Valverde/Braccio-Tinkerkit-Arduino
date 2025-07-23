# ROS2 para el Tinkerkit Braccio robot

### Ejecución archivos

     cd Escritorio/Braccio-Tinkerkit-Arduino
     colcon build
     source install/setup.bash

### Matar Gazebos abiertos

     pkill -f gazebo
     

### Model view

Launches RViz to show robot model.

     ros2 launch braccio_description view_robot.launch.py

### Gazebo classic simulation

Simulates the Braccio robot using Gazebo Classic and launches RViz to control it.

     ros2 launch braccio_bringup bringup.launch.py sim:=true

### Real robot control

Communicates with the Braccio robot via serial communication and launches RViz to control it.

     ros2 launch braccio_bringup bringup.launch.py sim:=false

### Testing communications

In order to test communication with the Arduino, a verbose option is implemented to check serial communication messages.

     ros2 launch braccio_bringup bringup.launch.py sim:=false hw_test:=true

### Video demostración de funcionamiento

[Video demostración del robot Braccio](https://drive.google.com/file/d/1czyuYS2wScXaEFBbtQSDKvis1xbDTHjN/view?usp=sharing "Haz clic para ver el video de demostración")

## Programas de Pick and Place

Este repositorio incluye programas de pick and place funcionales para el robot Braccio, enfocados en la practicidad y facilidad de configuración.

### Simulación Básica

Lanza la simulación en Gazebo Classic sin MoveIt para usar scripts directos:

```bash
ros2 launch braccio_bringup bringup.launch.py sim:=true
```

### Scripts de Pick and Place Disponibles

#### 1. Pick and Place Simple (`pick_and_place_simple.py`)

Programa básico y funcional que usa controladores de trayectoria directamente. **Recomendado para comenzar**.

**Características:**
- ✅ Control directo de trayectorias (funciona siempre)
- ✅ Secuencia predefinida de movimientos
- ✅ Control separado del brazo y gripper
- ✅ Fácil de entender y modificar
- ✅ No depende de MoveIt

**Ejecución:**
```bash
# Primero lanzar simulación
ros2 launch braccio_bringup bringup.launch.py sim:=true

# En otra terminal
ros2 run braccio_moveit_config pick_and_place_simple.py
```

#### 2. Pick and Place Configurable (`pick_and_place_configurable.py`) ⭐

**¡El programa principal!** Completamente configurable mediante archivo YAML. Ideal para personalizar comportamientos.

**Características:**
- ⭐ Configuración completa mediante archivo YAML
- ⭐ Secuencias personalizables sin tocar código
- ⭐ Posiciones nombradas fáciles de modificar
- ⭐ Múltiples secuencias predefinidas
- ⭐ Sistema de configuración robusto

**Ejecución:**
```bash
# Primero lanzar simulación
ros2 launch braccio_bringup bringup.launch.py sim:=true

# En otra terminal
ros2 run braccio_moveit_config pick_and_place_configurable.py
```

#### 3. Position Tester (`position_tester.py`) 🔧

Herramienta interactiva para probar y calibrar posiciones antes de usarlas en pick and place.

**Características:**
- 🔧 Modo interactivo para probar posiciones
- 🔧 Muestra posiciones actuales del robot
- 🔧 Prueba posiciones individuales del archivo YAML
- 🔧 Prueba secuencias completas
- 🔧 Control directo del gripper

**Ejecución:**
```bash
# Primero lanzar simulación
ros2 launch braccio_bringup bringup.launch.py sim:=true

# En otra terminal (modo interactivo)
ros2 run braccio_moveit_config position_tester.py
```

**Comandos del Position Tester:**
- `list` - Ver todas las posiciones disponibles
- `current` - Mostrar posición actual del robot
- `test home` - Probar la posición "home"
- `grip_open` - Abrir gripper
- `grip_close` - Cerrar gripper
- `sequence` - Ejecutar secuencia completa
- `quit` - Salir

### Configuración y Calibración de Posiciones 🎯

**¡Paso más importante!** El archivo de configuración permite personalizar completamente el comportamiento:

```
braccio_moveit_config/config/pick_and_place_config.yaml
```
