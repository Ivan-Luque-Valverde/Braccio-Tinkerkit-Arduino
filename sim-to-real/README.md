Sim-to-Real utilities

Scripts:
- webcam_publisher.py : publica la webcam local en /overhead_camera/image_raw y sus datos de camera_info
- color_square_detector.py : detecta cuadrados de colores, publica /vision/debug_image y /vision/detections
- camera_viewer.py : muestra imagen raw y debug usando OpenCV

Requisitos:
- ROS2 (Humble+)
- rclpy, cv_bridge y OpenCV instalados
- Permisos para /dev/video0 (añadir al grupo video si hace falta)

Ejemplo de uso:
1) En una terminal: source install/setup.bash
2) Lanzar el publicador de webcam:
   python3 sim-to-real/webcam_publisher.py
3) En otra terminal, lanzar el detector:
   python3 sim-to-real/color_square_detector.py
4) En otra, lanzar el viewer:
   python3 sim-to-real/camera_viewer.py

Notas:
- El detector asume unos parámetros intrínsecos por defecto (fx/fy/cx/cy). Ajustarlos con ROS params si tienes calibración.
- Publicamos detecciones como PoseStamped en frame 'camera_link' con z = object_z (parametro). Si quieres transformar a 'world' haz un lookup_transform en otro nodo.
