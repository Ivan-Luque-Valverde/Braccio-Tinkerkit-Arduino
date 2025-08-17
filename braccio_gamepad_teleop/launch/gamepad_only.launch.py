from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch simple que solo agrega gamepad control.
    Usar después de ejecutar: ros2 launch braccio_bringup bringup.launch.py sim:=false
    """
    
    # Nodo para el driver del gamepad (Joy)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',  # Dispositivo del gamepad
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )
    
    # Nodo para la teleoperación con gamepad
    gamepad_teleop_node = Node(
        package='braccio_gamepad_teleop',
        executable='gamepad_teleop',
        name='gamepad_teleop',
        output='screen',
        parameters=[{
            'planning_group': 'arm',
            'scale_linear': 0.1,      # Escala para movimientos lineales
            'scale_angular': 0.5,     # Escala para rotaciones
            'deadzone': 0.1           # Zona muerta para los joysticks
        }]
    )
    
    return LaunchDescription([
        joy_node,
        gamepad_teleop_node
    ])
