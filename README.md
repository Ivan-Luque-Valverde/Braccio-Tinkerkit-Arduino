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

Si falla, añadir el código de braccio_ros_arduino desde la propia IDE de Arduino.

### Testing communications

In order to test communication with the Arduino, a verbose option is implemented to check serial communication messages.

     ros2 launch braccio_bringup bringup.launch.py sim:=false hw_test:=true

### Video demostración de funcionamiento

[Video demostración del robot Braccio](https://drive.google.com/file/d/1czyuYS2wScXaEFBbtQSDKvis1xbDTHjN/view?usp=sharing "Haz clic para ver el video de demostración")

## Programas de Pick and Place

- **calculate_homography.py**  
  Script para calcular la matriz de homografía a partir de puntos conocidos en la imagen y el mundo real. Permite calibrar la cámara para transformar coordenadas de píxeles a coordenadas reales del workspace. El resultado se guarda en `camera_calibration.json`.

- **camera_spawner.py**  
  Lanza una cámara cenital virtual en Gazebo para simular la visión superior del workspace. Es útil para pruebas de visión artificial y calibración.

- **camera_viewer.py**  
  Visualiza en tiempo real el feed de la cámara cenital y las imágenes de debug con las detecciones de objetos. Permite comprobar visualmente la detección y calibración.

- **object_detector.py**  
  Nodo ROS2 que detecta cubos verdes en la imagen de la cámara, publica sus coordenadas y muestra las detecciones en la imagen de debug. Utiliza OpenCV y la configuración de la cámara para segmentar y localizar los objetos.

- **object_spawner.py**  
  Genera automáticamente cubos de colores (rojos y verdes) en posiciones conocidas del workspace en Gazebo. Útil para pruebas de calibración y pick and place.

- **ik_workspace_tester.py**  
  Herramienta para probar la cinemática inversa del Braccio en todo el workspace, visualizando los límites y posibles errores de alcance.

- **inverse_kinematics_calculator.py**  
  Calculadora analítica de cinemática inversa para el Braccio, utilizada por los sistemas de pick and place para convertir posiciones objetivo en ángulos de las articulaciones.

- **pick_and_place_configurable.py**  
  Nodo ROS2 que ejecuta secuencias de pick and place configurables mediante YAML. Permite definir trayectorias, posiciones y acciones del gripper de forma flexible.

- **vision_auto_pick_and_place.py**  
  Sistema completo de pick and place basado en visión: detecta cubos verdes, calcula su posición real, ejecuta la secuencia de pick and place y controla el gripper automáticamente.


  [Video demostración pick and place 3 cubos](https://drive.google.com/file/d/1wLfVnPr-vZvINre7aAciuyq-dVhB1gt1/view?usp=drive_link "Haz clic para ver el video de demostración")

### Ejecución Paso a Paso del Sistema Completo ⭐

     cd Escritorio/Braccio-Tinkerkit-Arduino
     colcon build
     source install/setup.bash

```bash
# Terminal 1: Lanzar simulación con cámara
ros2 launch braccio_vision vision_simulation.launch.py

# Terminal 2: Ejecutar detector de objetos  
ros2 run braccio_vision object_detector.py

# Terminal 3: Ver feed de cámara con detecciones
ros2 run braccio_vision camera_viewer.py

# Terminal 4: Ejecutar pick and place guiado por visión
ros2 launch braccio_vision vision_auto_pick_and_place.launch.py

# Terminal 5: Pruebas IK
python3 braccio_vision/scripts/ik_workspace_tester.py

# Control por mando usb
ros2 launch braccio_gamepad_teleop gamepad_teleop.launch.py

python3 sim-to-real/webcam_publisher.py

python3 sim-to-real/calculate_real_homography.py


```

