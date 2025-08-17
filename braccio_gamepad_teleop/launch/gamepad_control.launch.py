from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Ruta al paquete de bringup para el brazo físico
    bringup_package = get_package_share_directory('braccio_bringup')
    
    # Incluir el launch del brazo físico (no simulación)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_package, 'launch', 'bringup.launch.py')
        ]),
        launch_arguments={'sim': 'false'}.items()
    )
    
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
        bringup_launch,
        joy_node,
        gamepad_teleop_node
    ])
