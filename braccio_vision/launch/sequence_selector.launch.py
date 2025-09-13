#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Nodo selector de secuencias
    sequence_selector_node = Node(
        package='braccio_vision',
        executable='sequence_selector.py',
        name='sequence_selector',
        output='screen',
        parameters=[
            {'use_sim_time': False}
        ]
    )

    return LaunchDescription([
        sequence_selector_node
    ])
