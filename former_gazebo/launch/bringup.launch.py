import os
from os import environ
from os import pathsep

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, Shutdown, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    robot_name = DeclareLaunchArgument("robot_name", default_value="former")
    world_name = DeclareLaunchArgument("world_name", default_value="empty.world")

    environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
    environ['GAZEBO_MODEL_DATABASE_URI'] = ''

    gazebo_model_paths = '/usr/share/gazebo-11/models'

    package_path = get_package_share_directory('former_gazebo')
    gazebo_model_paths += pathsep + package_path + "/models"

    environ['GAZEBO_MODEL_PATH'] = gazebo_model_paths

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gzserver.launch.py'
        ]),
        launch_arguments={
            "verbose": "true",
            "world": PathJoinSubstitution([
                        get_package_share_directory('former_gazebo'),
                        'worlds',
                        LaunchConfiguration('world_name'),
            ])
        }.items(),
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gzclient.launch.py'
        ]),
    )

    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('former_description'),
            '/launch/upload_robot.launch.py']
        ),
        launch_arguments = {
            'use_gazebo_sim' : 'true'
        }.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-timeout', '20.0',
            '-x', '-5.0',
            '-y', '-3.0',
            '-package_to_model'
        ],
        parameters=[{
            "use_sim_time": True
        }],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )

    load_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'base_controller'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_base_controller],
            )
        ),
        robot_name,
        world_name,
        gz_server,
        gz_client,
        upload_robot,
        spawn_robot,
    ])
