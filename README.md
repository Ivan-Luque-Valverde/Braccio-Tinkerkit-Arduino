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

## Sistema de Visión 📷

Este repositorio incluye un sistema de visión modular completo que permite al robot Braccio detectar objetos y calcular sus coordenadas para operaciones de pick and place automatizadas.

### Configuración del Sistema de Visión

El sistema de visión está implementado como un paquete completamente modular en:
```
braccio_vision/
```

### Lanzamiento del Sistema de Visión

#### 1. Simulación con Cámara Cenital

Lanza la simulación con una cámara fija posicionada sobre el área de trabajo:

```bash
ros2 launch braccio_vision vision_simulation.launch.py
```

Este comando lanza:
- ✅ Simulación de Gazebo con cámara cenital
- ✅ Robot Braccio en el mundo de visión
- ✅ Nodo de detección de objetos
- ✅ Visor de cámara en tiempo real

#### 2. Componentes del Sistema de Visión

**a) Detector de Objetos (`object_detector.py`)** 🎯
- Detección de objetos por color (HSV)
- Cálculo de coordenadas mundo desde píxeles
- Publicación de coordenadas detectadas
- Filtrado de ruido y contornos mínimos

**b) Visor de Cámara (`camera_viewer.py`)** 👁️
- Visualización en tiempo real de la cámara
- Overlay de detecciones de objetos
- Debug visual del sistema de detección

**c) Pick and Place con Visión (`vision_pick_and_place.py`)** 🤖
- Integración completa detección + manipulación
- Automatización del proceso completo
- Calibración de coordenadas cámara-robot

### Configuración de Visión

El sistema es completamente configurable mediante:
```
braccio_vision/config/vision_config.yaml
```

**Parámetros principales:**
- 🎨 Rangos de color HSV para detección
- 📏 Parámetros de calibración de cámara
- 🎯 Transformación píxel-a-mundo
- ⚙️ Filtros de detección (área mínima, etc.)

### Calibración del Sistema de Visión 🔧

Para ajustar la detección de objetos:

1. **Calibrar colores HSV:** Modificar rangos en `vision_config.yaml`
2. **Calibrar coordenadas:** Usar objetos de referencia conocidos
3. **Ajustar filtros:** Configurar área mínima y máxima de detección

### Arquitectura del Sistema de Visión

```
braccio_vision/
├── braccio_vision/          # Nodos Python
│   ├── object_detector.py   # Detección de objetos
│   ├── camera_viewer.py     # Visor de cámara
│   └── vision_pick_and_place.py  # Pick&place con visión
├── config/
│   └── vision_config.yaml  # Configuración del sistema
├── launch/
│   ├── vision_simulation.launch.py  # Lanzador principal
│   └── vision_bringup.launch.py     # Solo nodos de visión
├── urdf/
│   └── vision_world.xacro   # Mundo con cámara cenital
└── scripts/                 # Scripts ejecutables
```

### Ejecución Paso a Paso del Sistema Completo ⭐


```bash
# Terminal 1: Lanzar simulación con cámara
ros2 launch braccio_vision vision_simulation.launch.py

# Terminal 2: Ejecutar detector de objetos  
ros2 run braccio_vision object_detector.py

# Terminal 3: Ver feed de cámara con detecciones
ros2 run braccio_vision camera_viewer.py

# Terminal 4: Ejecutar pick and place guiado por visión
ros2 run braccio_vision vision_pick_and_place.py
```

### Arquitectura Técnica

```
Sistema de Visión Braccio
├── Simulación (Gazebo + Camera Plugin)
├── Detección (OpenCV + HSV)
├── Transformación (Pixel-to-World)
├── Manipulación (Trajectory Controllers)
└── Control (Estado + Secuencias)
```
