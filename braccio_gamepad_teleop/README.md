# Braccio Gamepad Teleoperación

Este paquete permite controlar el brazo robótico Braccio usando un gamepad (mando de videojuegos) a través de MoveIt.

## Requisitos

- Gamepad compatible (Xbox 360/One, PlayStation, o similar)
- ROS2 Humble
- MoveIt configurado para Braccio
- Paquete `joy` instalado

## Instalación

El paquete `joy` debe estar instalado:
```bash
sudo apt install ros-humble-joy
```

## Configuración del Gamepad

1. Conectar el gamepad por USB o Bluetooth
2. Verificar que se detecte correctamente:
```bash
ls /dev/input/js*
```
Debería mostrar algo como `/dev/input/js0`

3. Probar el gamepad:
```bash
ros2 run joy joy_node
```

## Controles del Gamepad

### Stick Izquierdo (Movimiento Lineal)
- **Arriba/Abajo**: Movimiento en eje Y (adelante/atrás)
- **Izquierda/Derecha**: Movimiento en eje X (izquierda/derecha)

### Stick Derecho (Movimiento Vertical y Rotación)
- **Arriba/Abajo**: Movimiento en eje Z (arriba/abajo)
- **Izquierda/Derecha**: Rotación en eje Z (giro)

### Botones
- **Botón A (o X en PlayStation)**: Activar/Desactivar control
- **Gatillos L2/R2**: Control de rotación en ejes X e Y respectivamente

## Uso

### Método 1: Launch completo (Recomendado)
```bash
ros2 launch braccio_gamepad_teleop gamepad_control.launch.py
```

Este comando inicia:
- MoveIt con la configuración del Braccio
- El driver del gamepad
- El nodo de teleoperación

### Método 2: Nodos individuales

En terminales separadas:

1. Iniciar MoveIt:
```bash
ros2 launch braccio_moveit_config demo.launch.py
```

2. Iniciar el driver del gamepad:
```bash
ros2 run joy joy_node
```

3. Iniciar la teleoperación:
```bash
ros2 run braccio_gamepad_teleop gamepad_teleop
```

## Parámetros de Configuración

El nodo acepta los siguientes parámetros:

- `planning_group`: Grupo de planificación de MoveIt (por defecto: "braccio_arm")
- `scale_linear`: Escala para movimientos lineales (por defecto: 0.1)
- `scale_angular`: Escala para rotaciones (por defecto: 0.5)
- `deadzone`: Zona muerta para los joysticks (por defecto: 0.1)

Ejemplo con parámetros personalizados:
```bash
ros2 run braccio_gamepad_teleop gamepad_teleop --ros-args -p scale_linear:=0.05 -p scale_angular:=0.3
```

## Solución de Problemas

### El gamepad no se detecta
- Verificar que el dispositivo esté en `/dev/input/js0`
- Cambiar el parámetro `dev` en el launch file si está en otro dispositivo

### Movimientos muy rápidos o lentos
- Ajustar los parámetros `scale_linear` y `scale_angular`
- Valores más bajos = movimientos más lentos y precisos

### El brazo no responde
- Verificar que MoveIt esté ejecutándose correctamente
- Comprobar que el nombre del grupo de planificación sea correcto
- Verificar los tópicos:
```bash
ros2 topic list | grep joy
ros2 topic echo /joy
```

## Temas ROS2

- **Suscribe a**: `/joy` (sensor_msgs/Joy) - Datos del gamepad
- **Publica en**: MoveIt a través de la interfaz MoveGroupInterface

## Seguridad

- El sistema incluye una zona muerta en los joysticks para evitar movimientos accidentales
- Los movimientos son relativos y graduales
- Siempre mantener el botón de activación (A) presionado para que el brazo se mueva
- **IMPORTANTE**: Supervisar siempre el movimiento del brazo y estar listo para detener el sistema en caso de emergencia

## Limitaciones

- Los movimientos son en el espacio cartesiano, no en el espacio de las articulaciones
- La planificación de trayectorias puede fallar si se solicita una pose inalcanzable
- El sistema no incluye control directo de la pinza (gripper)

## Mapeo de Botones por Defecto (Xbox)

```
     Y
     |
X ---+--- B
     |
     A

Stick Izquierdo: Movimiento X,Y
Stick Derecho: Movimiento Z, Rotación Z
LT/RT: Rotación X,Y
Botón A: Activar/Desactivar
```

Para otros tipos de gamepad, el mapeo puede variar. Usar `ros2 topic echo /joy` para ver los índices de botones y ejes específicos.
