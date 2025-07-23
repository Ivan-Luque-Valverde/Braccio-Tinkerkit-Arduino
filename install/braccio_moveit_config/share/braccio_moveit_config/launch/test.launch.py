# Contenido FINAL y CORRECTO para test.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

def generate_launch_description():
    # --- Creación de robot_description (LA FORMA CORRECTA) ---
    # Simplemente procesamos el XACRO pasándole el parámetro 'sim:=true'
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([get_package_share_directory("braccio_description"), "urdf", "braccio.urdf.xacro"]), " ",
            "sim:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # --- NODOS DEL TEST (igual que antes) ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        )
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "braccio"],
        output="screen",
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "300"],
    )
    delay_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_entity, on_exit=[joint_state_broadcaster_spawner])
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delay_spawner,
    ])