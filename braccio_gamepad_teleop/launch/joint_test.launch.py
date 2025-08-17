#!/usr/bin/env python3

import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='braccio_gamepad_teleop',
            executable='joint_test',
            name='joint_test',
            output='screen',
            parameters=[],
        )
    ])
