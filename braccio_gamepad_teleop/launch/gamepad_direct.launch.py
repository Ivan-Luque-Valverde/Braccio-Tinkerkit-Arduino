from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo Joy para el gamepad
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),
        
        # Nodo de control directo del gamepad (SIN MoveIt)
        Node(
            package='braccio_gamepad_teleop',
            executable='gamepad_direct',
            name='gamepad_direct',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
    ])
