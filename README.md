# Braccio Tinkerkit + ROS 2 (Pick & Place, Visión, Teleoperación y Control Real)

Proyecto integral para el brazo Tinkerkit Braccio empleando ROS 2 Humble: simulación Gazebo Classic + RViz, visión artificial (detección por color + homografía), cinemática inversa analítica, pick & place automático, teleoperación con mando PS4 y soporte para robot físico vía ros2_control + Arduino.

---

## 1. Objetivos Principales
- Simular y controlar el Braccio en Gazebo y en el hardware real.
- Detectar cubos de colores desde cámara cenital (simulada o real).
- Convertir coordenadas de píxel a coordenadas métricas mediante homografía o intrínsecos/FOV.
- Calcular cinemática inversa (IK) analítica y validar workspace.
- Ejecutar secuencias de pick & place (visión → IK → attach/detach).
- Teleoperar las articulaciones y gripper con mando PS4.
- Unificar flujo sim → real minimizando cambios.

---

## 2. Estructura General (Paquetes / Scripts Clave)

| Componente | Función |
|------------|---------|
| braccio_description | URDF/XACRO, configuración ros2_control, mundo Gazebo y cámara. |
| braccio_bringup | Launch unificado (simulación o real) + carga de controladores. |
| braccio_hardware | Interfaz ros2_control para Arduino (serie) + protocolo `<l>/<r>/<w,...>`. |
| braccio_moveit_config | Configuración MoveIt2, planning y ejecución. |
| braccio_vision | Visión: cámara, detector, homografía, pick & place automático. |
| braccio_gamepad_teleop | Teleoperación con mando PS4 (joy → joint trajectories + attach/detach). |
| sim-to-real | Adaptaciones físicas
| Scripts de utilidades | Calibración, pruebas IK, homografía, viewers. |

---

## 3. Instalación y Compilación

```bash
# Clonar el repositorio
git clone https://github.com/Ivan-Luque-Valverde/Braccio-Tinkerkit-Arduino.git
cd Braccio-Tinkerkit-Arduino

# Compilar el workspace ROS2
colcon build
source install/setup.bash
```

**Requisitos previos:**
- ROS 2 Humble instalado
- Gazebo Classic
- MoveIt2
- OpenCV
- Arduino IDE (para hardware real)


---

## 4. Simulación y Visualización

| Acción | Comando |
|-------|---------|
| Ver modelo en RViz | `ros2 launch braccio_description view_robot.launch.py` |
| Simulación completa (Gazebo + RViz + MoveIt + control) | `ros2 launch braccio_bringup bringup.launch.py sim:=true` |
| Teleop mando | `ros2 launch braccio_gamepad_teleop gamepad_teleop.launch.py` |

---

## 5. Control en Robot Real

```bash
ros2 launch braccio_bringup bringup.launch.py sim:=false
```

Requisitos:
- Firmware Arduino compatible (protocolo `<l>`, `<r>`, `<w,...>`).
- Puerto serie correcto (editar en XACRO si no es `/dev/ttyACM0`).
- Permisos grupo `dialout` (Linux) o configuración equivalente.

---

## 6. Visión: Pipeline Completo

1. Cámara cenital (URDF + plugin Gazebo) publica:
   - `/overhead_camera/image_raw`
   - `/overhead_camera/camera_info`
2. `object_detector.py`:
   - Lee imagen + CameraInfo.
   - Segmenta por color HSV (config en `vision_config.yaml`).
   - Obtiene contornos y centroides en píxeles.
   - Publica:
     - `/vision/debug_image` (overlay)
     - Markers (`/vision/object_markers`)
     - Coordenadas (actualmente en píxeles) `/detected_object_coords` (PointStamped).
3. Conversión píxel→mundo:
   - Homografía (`camera_calibration.json`) calculada con `calculate_homography.py` (usa 4+ pares píxel↔mundo).
   - Validación con `test_homography.py`.
   - Alternativa fallback: proyección simple usando intrínsecos (FOV=80° → fx≈381.97) y altura cámara.
4. `camera_viewer.py`: visualización feed + debug.
5. Integración: `vision_auto_pick_and_place.py` transforma píxeles a metros, resuelve IK y lanza secuencia.

Calibración recomendada:
```bash
python3 braccio_vision/scripts/calculate_homography.py
python3 braccio_vision/scripts/test_homography.py
```

---

## 7. Cálculo de Cinemática Inversa

`inverse_kinematics_calculator.py`:
- Parámetros geométricos: longitudes (`L`, offset `l`).
- Workspace radial mediante ángulos del shoulder.
- Funciones:
  - `calculate_ik_xy(x,y)` → solución 2D (incluye lógica "configuración simétrica" reflejando φ si cae fuera de [0,π]).
  - `calculate_ik_xyz(x,y,z)` → ajustes por altura (heurísticos).
  - `calculate_pick_positions()` → `pick_approach` y `pick_position`.
- Escritura opcional en `braccio_moveit_config/config/pick_and_place_config.yaml`.

Pruebas de cobertura:
```bash
python3 braccio_vision/scripts/ik_workspace_tester.py
```

---

## 8. Pick & Place Basado en Visión

`vision_auto_pick_and_place.py`:
1. Subscribirse a `/detected_object_coords`.
2. Transformar píxeles → (x,y) metros (homografía).
3. Determinar modelo (nombre en Gazebo) por proximidad.
4. Calcular IK (approach + grasp).
5. Guardar/usar posiciones.
6. Invocar `ConfigurablePickAndPlace`:
   - Publica trayectorias (joint trajectory controllers).
   - Opera gripper.
   - Servicios `/ATTACHLINK` y `/DETACHLINK` (plugin Gazebo `libgazebo_link_attacher.so` cargado en `braccio.world`).
7. Marcar objeto como procesado (evita duplicados).

`pick_and_place_configurable.py`:
- Carga YAML con poses/acciones.
- Ejecuta secuencias (approach → descend → cerrar → attach → levantar → place → detach).

---

## 9. Teleoperación con Mando PS4

`gamepad_teleop.py`:
- Suscriptor `/joy` (paquete `joy`).
- Mapeo típico:
  - Stick izq X → base
  - Stick izq Y → hombro
  - Stick der Y → codo
  - Stick der X → muñeca
  - L1/R1 → rotación final (joint_4)
  - Botones específicos → abrir/cerrar gripper
  - Botones acción → attach/detach (servicios)
- Publica:
  - `/position_trajectory_controller/joint_trajectory`
  - `/gripper_controller/joint_trajectory`
- Usa `/gazebo/model_states` + TF para proximidad en pick manual.

---

## 10. Attach / Detach (Gazebo)

- Plugin cargado en `braccio_description/gazebo/braccio.world`:
  ```xml
  <plugin name="gazebo_link_attacher" filename="libgazebo_link_attacher.so"/>
  ```
- Servicios:
  - `/ATTACHLINK` (AttachLink.srv)
  - `/DETACHLINK` (DetachLink.srv)
- Clientes en: `pick_and_place_configurable.py` y `gamepad_teleop.py`.

Verificación:
```bash
ros2 service list | grep ATTACH
```

---

## 11. Scripts y Utilidades

- **calculate_homography.py** — Cálculo homografía desde correspondencias píxel↔mundo.
- **test_homography.py** — Validación error reproyección homografía.
- **camera_spawner.py** — Spawnea cámara cenital virtual en Gazebo.
- **object_spawner.py** — Genera cubos de colores en posiciones conocidas.
- **camera_viewer.py** — Visualización feed cámara + detecciones debug.
- **object_detector.py** — Detección HSV + publicación coordenadas objetos.
- **inverse_kinematics_calculator.py** — IK analítica (configuración simétrica incluida).
- **ik_workspace_tester.py** — Test cobertura workspace y límites IK.
- **vision_auto_pick_and_place.py** — Orquestador visión→IK→pick completo.
- **pick_and_place_configurable.py** — Ejecutor secuencias YAML (MoveIt + attach/detach).
- **gamepad_teleop.py** — Teleoperación PS4 + pick proximidad manual.

---

## 12. Flujo de Ejecución Recomendado (Simulación)

```bash
# 1. Simulación + cámara + control
ros2 launch braccio_vision vision_simulation.launch.py

# 2. Detector de objetos
ros2 run braccio_vision object_detector.py

# 3. Visualizador cámara
ros2 run braccio_vision camera_viewer.py

# 4. Pick & place visión
ros2 launch braccio_vision vision_auto_pick_and_place.launch.py

# (Opc) Teleoperación
ros2 launch braccio_gamepad_teleop gamepad_teleop.launch.py

# (Opc) Pruebas IK
python3 braccio_vision/scripts/ik_workspace_tester.py
```

---

## 13. Flujo de Ejecución (Robot Real)

```bash
# 1. Hardware + control
ros2 launch braccio_bringup bringup.launch.py sim:=false

# 2. Cámara real 
python3 sim-to-real/webcam_publisher.py

# 3. Detector de objetos
ros2 run braccio_vision object_detector.py

# 4. Visualizador cámara
ros2 run braccio_vision camera_viewer.py

# 5. Pick & place visión
ros2 launch braccio_vision vision_auto_pick_and_place.launch.py

# 6. Calibración real (opcional)
python3 sim-to-real/calculate_real_homography.py
```

---

## 14. Videos de Demostración

###  Pick & Place Automático (Simulación)
[![Video Demo 3](https://img.shields.io/badge/▶️_Ver_Video-Pick_&_Place_Auto-green?style=for-the-badge)](https://drive.google.com/file/d/1j93a8JxrDpSgNt0ZDkJTsVWRrKjXWaPa/view?usp=drive_link)

**Contenido:** Sistema completo de visión artificial detectando y recogiendo múltiples cubos de colores de forma autónoma en simulación.

### Teleoperación con Gamepad (Simulación)
[![Video Demo 1](https://img.shields.io/badge/▶️_Ver_Video-Teleop_Gamepad-red?style=for-the-badge)](https://drive.google.com/file/d/1-mpbLrjvFX8We0Mxjq25MYwNQcKWb8i_/view?usp=drive_link)

**Contenido:** Control manual del robot Braccio en simulación usando mando PS4/gamepad controller.

### Robot Físico en Acción
[![Video Demo 2](https://img.shields.io/badge/▶️_Ver_Video-Robot_Real-blue?style=for-the-badge)](https://drive.google.com/file/d/1NQYd4aHFoAMx7IZzLsldv3O5Icrmtra2/view?usp=drive_link)

**Contenido:** Integración física - Robot Braccio real moviéndose y ejecutando comandos en el hardware físico.

---

## Licencia

MIT License