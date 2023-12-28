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
    robot_name = DeclareLaunchArgument("robot_name", default_value="former")
    world_name = DeclareLaunchArgument("world_name", default_value="office_building.sdf")
    tf_prefix = DeclareLaunchArgument("tf_prefix", default_value="")

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

    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('former_description'),
            '/launch/upload_robot.launch.py']
        ),
        launch_arguments = {
            'use_gazebo_sim' : 'true',
            'tf_prefix': LaunchConfiguration('tf_prefix'),
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='both',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '-1.0',
        ],
        parameters=[{
            "use_sim_time": True
        }],
    )

    # robot_localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[
    #         os.path.join(
    #             get_package_share_directory('former_bringup'),
    #             'config/ekf.yaml'
    #         ),
    #         {'use_sim_time': True}
    #     ],
    # )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
        ],
        output='screen'
    )

    gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/camera/image',
            '/camera/depth_image'
        ],
        output='screen'
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
        robot_name,
        world_name,
        tf_prefix,
        gz_bridge,
        upload_robot,
        gz_sim,
        spawn_robot,
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
        gz_image_bridge,
        # robot_localization_node,

    ])
