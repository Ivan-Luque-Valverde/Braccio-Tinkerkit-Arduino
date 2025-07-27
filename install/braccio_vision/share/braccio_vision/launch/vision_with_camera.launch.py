#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Rutas de paquetes
    braccio_description_dir = get_package_share_directory('braccio_description')
    vision_package_dir = get_package_share_directory('braccio_vision')
    
    # Ruta al mundo de visión
    world_file = os.path.join(vision_package_dir, 'urdf', 'vision_world.sdf')
    
    # Ruta al URDF del robot
    urdf_file = os.path.join(braccio_description_dir, 'urdf', 'braccio.urdf.xacro')
    
    return LaunchDescription([
        
        # 1. Lanzar Gazebo con el mundo de visión
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file],
            output='screen'
        ),
        
        # 2. Lanzar simulación básica de Braccio (sin mundo específico)
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(braccio_description_dir, 'launch', 'bringup.launch.py')
                    ]),
                    launch_arguments={'sim': 'true'}.items()
                )
            ]
        ),
        
        # 3. Detector de objetos
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='braccio_vision',
                    executable='object_detector.py',
                    name='object_detector',
                    output='screen'
                )
            ]
        ),
        
        # 4. Visor de cámara
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='braccio_vision',
                    executable='camera_viewer.py',
                    name='camera_viewer',
                    output='screen'
                )
            ]
        ),
        
        # 5. Pick and place con visión
        TimerAction(
            period=14.0,
            actions=[
                Node(
                    package='braccio_vision',
                    executable='vision_pick_and_place.py',
                    name='vision_pick_and_place',
                    output='screen'
                )
            ]
        ),
        
        # 6. Mensaje informativo
        ExecuteProcess(
            cmd=['echo', '🎥 Sistema completo con cámara cenital iniciado!'],
            output='screen'
        )
        
    ])
