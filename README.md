# ROS2 para el Tinkerkit Braccio robot

### Ejecución archivos

     cd Escritorio/Braccio-Tinkerkit-Arduino
     colcon build
     source install/setup.bash

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