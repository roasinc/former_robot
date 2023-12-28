import os
from os import environ
from os import pathsep

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, DeclareLaunchArgument, Shutdown, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    world_name = DeclareLaunchArgument("world_name", default_value="default.sdf")

    environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '1'
    resource_path = get_package_share_directory('former_description')
    resource_path = resource_path[:len(resource_path) - len('former_description')]

    model_path = get_package_share_directory('former_gazebo')
    environ['IGN_GAZEBO_RESOURCE_PATH'] = resource_path + pathsep + model_path + "/models"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            "gz_args": [
                '-r -v3',
                ' ',
                PathJoinSubstitution([
                    FindPackageShare('former_gazebo'),
                    "worlds",
                    LaunchConfiguration('world_name')]
                )
            ]
        }.items()
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        name="ros_gz_bridge_clock",
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([
        world_name,
        gz_bridge,
        gz_sim,
    ])
