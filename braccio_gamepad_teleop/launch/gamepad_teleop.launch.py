from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo del joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'dev': '/dev/input/js0'}]  # cambia si tu mando aparece distinto
        ),
        # Nodo teleop (Python script)
        Node(
            package='braccio_gamepad_teleop',
            executable='gamepad_teleop.py',
            name='gamepad_teleop',
            output='screen',
            parameters=[],
            arguments=[],
            prefix='python3 '
        )
    ])
