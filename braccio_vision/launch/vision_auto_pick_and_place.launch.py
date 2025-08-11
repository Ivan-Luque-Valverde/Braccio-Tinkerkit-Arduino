#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='braccio_vision',
            executable='vision_auto_pick_and_place.py',
            name='vision_auto_pick_and_place',
            output='screen'
        )
    ])
