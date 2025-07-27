#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Ruta del paquete de visión
    vision_package_dir = get_package_share_directory('braccio_vision')
    
    # Ruta del paquete de bringup
    bringup_package_dir = get_package_share_directory('braccio_bringup')
    
    return LaunchDescription([
        
        # 1. Lanzar simulación básica de Braccio
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(bringup_package_dir, 'launch', 'bringup.launch.py')
            ]),
            launch_arguments={'sim': 'true'}.items()
        ),
        
        # 2. Lanzar nodos de visión después de un delay
        TimerAction(
            period=5.0,
            actions=[
                # Detector de objetos
                Node(
                    package='braccio_vision',
                    executable='object_detector.py',
                    name='object_detector',
                    output='screen'
                ),
                
                # Visor de cámara
                Node(
                    package='braccio_vision',
                    executable='camera_viewer.py',
                    name='camera_viewer',
                    output='screen'
                ),
                
                # Pick and place con visión
                Node(
                    package='braccio_vision',
                    executable='vision_pick_and_place.py',
                    name='vision_pick_and_place',
                    output='screen'
                )
            ]
        ),
        
        # 3. Mensaje informativo
        ExecuteProcess(
            cmd=['echo', '🚀 Sistema completo de visión iniciado! Coloca objetos rojos en el área de trabajo.'],
            output='screen'
        )
    ])
