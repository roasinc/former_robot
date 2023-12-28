import os
from os import environ
from os import pathsep

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import ExecuteProcess, Shutdown, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, LogInfo
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    use_sim_time_cmd = DeclareLaunchArgument("use_sim_time", default_value="true")
    use_namespace_cmd = DeclareLaunchArgument("use_namespace", default_value="false")
    namespace_cmd = DeclareLaunchArgument("namespace", default_value="")
    tf_prefix_cmd = DeclareLaunchArgument("tf_prefix", default_value="")


    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('former_description'),
            '/launch/upload_robot.launch.py']
        ),
        launch_arguments = {
            'use_gazebo_sim' : LaunchConfiguration('use_sim_time'),
            'tf_prefix': LaunchConfiguration('tf_prefix'),
            'namespace': LaunchConfiguration('namespace'),
        }.items()
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

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        name="ros_gz_bridge_sensors",
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
        ],
        output='screen'
    )

    ros_gz_image = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/camera/image',
            '/camera/depth_image'
        ],
        output='screen'
    )


    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='both',
        arguments=[
            '-name', LaunchConfiguration('namespace'),
            '-topic', [LaunchConfiguration('namespace'), '/robot_description'],
            '-x', '0.0',
            '-y', '-1.0',
        ],
        parameters=[{
            "use_sim_time": LaunchConfiguration('use_sim_time')
        }],
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
    )

    load_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "base_controller",
            "--controller-manager", "/controller_manager"
        ],
    )

    return LaunchDescription([
        use_sim_time_cmd,
        use_namespace_cmd,
        namespace_cmd,
        tf_prefix_cmd,
        upload_robot,
        spawn_robot,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_broadcaster]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_base_controller],
            )
        ),
        # robot_localization_node,

    ])
