#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Rutas de paquetes
    bringup_package_dir = get_package_share_directory('braccio_bringup')
    vision_package_dir = get_package_share_directory('braccio_vision')
    
    # Ruta al mundo de visión
    world_file = os.path.join(vision_package_dir, 'urdf', 'vision_world.sdf')
    
    return LaunchDescription([
        
        # 1. Lanzar simulación básica de Braccio con mundo personalizado
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(bringup_package_dir, 'launch', 'bringup.launch.py')
            ]),
            launch_arguments={
                'sim': 'true',
                'world': world_file
            }.items()
        ),
        
        # 2. Esperar y lanzar detector de objetos
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='braccio_vision',
                    executable='object_detector.py',
                    name='object_detector',
                    output='screen'
                )
            ]
        ),
        
        # 3. Lanzar visor de cámara
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='braccio_vision',
                    executable='camera_viewer.py',
                    name='camera_viewer',
                    output='screen'
                )
            ]
        ),
        
        # 4. Pick and place con visión
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='braccio_vision',
                    executable='vision_pick_and_place.py',
                    name='vision_pick_and_place',
                    output='screen'
                )
            ]
        ),
        
        # 5. Mensaje informativo
        ExecuteProcess(
            cmd=['echo', '🚀 Sistema de visión con cámara cenital iniciado!'],
            output='screen'
        )
        
    ])
