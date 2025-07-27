#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Argumentos de launch
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Si usar simulación o robot real'
    )
    
    vision_arg = DeclareLaunchArgument(
        'vision',
        default_value='true',
        description='Si activar el sistema de visión'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Si activar modo debug de visión'
    )
    
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='1.0',
        description='Altura de la cámara cenital'
    )

    # Incluir el launch del robot Braccio original con el URDF modificado
    braccio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('braccio_bringup'),
                'launch',
                'bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'sim': LaunchConfiguration('sim'),
            'robot_description_file': PathJoinSubstitution([
                FindPackageShare('braccio_vision'),
                'urdf',
                'braccio_with_vision.urdf.xacro'
            ])
        }.items()
    )

    # Nodo de detección de objetos
    object_detector_node = Node(
        package='braccio_vision',
        executable='object_detector.py',
        name='object_detector',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('braccio_vision'),
                'config',
                'vision_config.yaml'
            ]),
            {'debug_mode': LaunchConfiguration('debug')}
        ],
        condition=IfCondition(LaunchConfiguration('vision')),
        output='screen'
    )

    # Nodo de visualización de cámara
    camera_viewer_node = Node(
        package='braccio_vision',
        executable='camera_viewer.py',
        name='camera_viewer',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('braccio_vision'),
                'config',
                'vision_config.yaml'
            ])
        ],
        condition=IfCondition(LaunchConfiguration('debug')),
        output='screen'
    )

    # Nodo de pick and place con visión
    vision_pick_and_place_node = Node(
        package='braccio_vision',
        executable='vision_pick_and_place.py',
        name='vision_pick_and_place',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('braccio_vision'),
                'config',
                'vision_config.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare('braccio_moveit_config'),
                'config',
                'pick_and_place_config.yaml'
            ])
        ],
        condition=IfCondition(LaunchConfiguration('vision')),
        output='screen'
    )

    return LaunchDescription([
        sim_arg,
        vision_arg,
        debug_arg,
        camera_height_arg,
        braccio_launch,
        object_detector_node,
        camera_viewer_node,
        vision_pick_and_place_node
    ])
