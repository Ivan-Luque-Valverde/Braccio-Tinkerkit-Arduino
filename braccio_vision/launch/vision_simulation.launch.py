#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Obtener directorios de paquetes
    braccio_bringup_dir = get_package_share_directory('braccio_bringup')
    vision_package_dir = get_package_share_directory('braccio_vision')
    
    return LaunchDescription([
        
        # Lanzar la simulación completa del robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(braccio_bringup_dir, 'launch', 'bringup.launch.py')
            ]),
            launch_arguments={
                'sim': 'true',
                'verbose': 'true'
            }.items()
        ),
        
        # Nodo para generar la cámara cenital (con retardo)
        TimerAction(
            period=7.0,  # segundos de espera antes de lanzar la cámara
            actions=[
                Node(
                    package='braccio_vision',
                    executable='camera_spawner.py',
                    name='camera_spawner',
                    output='screen'
                )
            ]
        ),
        # Nodo para generar objetos de prueba (unos segundos después de la cámara)
        TimerAction(
            period=10.0,  # segundos de espera antes de lanzar el spawner
            actions=[
                Node(
                    package='braccio_vision',
                    executable='object_spawner.py',
                    name='object_spawner',
                    output='screen'
                )
            ]
        ),
        
    ])
