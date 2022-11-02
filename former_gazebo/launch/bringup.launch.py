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

    return LaunchDescription([
        robot_name,
        world_name,
        gz_server,
        gz_client,
        upload_robot,
        spawn_robot,
    ])

    # robot_path = get_package_share_directory('former_description')
    # robot_path = robot_path[:len(robot_path) - len('former_description')]
    # robot_path = get_package_share_directory('former_description')
    # robot_path = robot_path[:len(robot_path) - len('former_description')]
    # gz_resource_paths += pathsep + robot_path

    # realsense2_path = get_package_share_directory('realsense2_description')
    # realsense2_path = realsense2_path[:len(realsense2_path) - len('realsense2_description')]
    # realsense2_path = get_package_share_directory('realsense2_description')
    # realsense2_path = realsense2_path[:len(realsense2_path) - len('realsense2_description')]
    # gz_resource_paths += pathsep + realsense2_path

    # # environ['IGN_MODEL_PATH'] = gz_resource_paths
    # environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_paths

    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('ros_gz_sim'),
    #         '/launch/gz_sim.launch.py'
    #     ]),
    #     launch_arguments={
    #         'gz_args': [
    #                         '-r -v2 ',
    #                         PathJoinSubstitution([
    #                                 get_package_share_directory('former_gazebo'),
    #                                 'worlds',
    #                                 LaunchConfiguration('world_name')
    #                         ])
    #                     ]
    #     }.items(),
    # )

    # upload_robot = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('former_description'),
    #         '/launch/upload_robot.launch.py']
    #     ),
    #     launch_arguments = {
    #         'use_gazebo_sim' : 'true'
    #     }.items()
    # )

    # spawn_node = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     output='screen',
    #     arguments=[
    #         '-name', LaunchConfiguration('robot_name'),
    #         '-topic', 'robot_description',
    #         '-x', '-5.0',
    #         '-y', '-3.0'
    #     ],
    #     parameters=[{
    #         "use_sim_time": True
    #     }],
    # )

    # ign_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[ '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
    #                 '/laser_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
    #                 '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
    #                 '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
    #     ],
    #     parameters=[{
    #         "use_sim_time": True
    #     }],
    #     output='screen'
    # )

    # ign_image_bridge = Node(
    #     package='ros_gz_image',
    #     executable="image_bridge",
    #     arguments=[ 'camera/image',
    #                 'camera/depth_image'
    #     ]
    # )

    # static_tf_lidar = Node(
    #     package='tf2_ros',
    #     name="static_tf_lidar",
    #     executable='static_transform_publisher',
    #     arguments=[ "0.0", "0.0", "0.0", "0.0", "0.0", "0.0",
    #                 "laser_link",
    #                 [LaunchConfiguration('robot_name'), "/base_footprint/laser_link"]
    #     ],
    #     parameters=[{
    #         "use_sim_time": True
    #     }],
    #     output='screen'
    # )

    # static_tf_camera = Node(
    #     package='tf2_ros',
    #     name="static_tf_lidar",
    #     executable='static_transform_publisher',
    #     arguments=[ "0.0", "0.0", "0.0", "0.0", "0.0", "0.0",
    #                 "camera_link",
    #                 [LaunchConfiguration('robot_name'), "/base_footprint/rgbd_camera"]
    #     ],
    #     parameters=[{
    #         "use_sim_time": True
    #     }],
    #     output='screen'
    # )

    # static_tf_imu = Node(
    #     package='tf2_ros',
    #     name="static_tf_lidar",
    #     executable='static_transform_publisher',
    #     arguments=[ "0.0", "0.0", "0.0", "0.0", "0.0", "0.0",
    #                 "imu_link",
    #                 [LaunchConfiguration('robot_name'), "/base_footprint/imu"]
    #     ],
    #     parameters=[{
    #         "use_sim_time": True
    #     }],
    #     output='screen'
    # )

    # jsp_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{
    #         "source_list": ['joint_states'],
    #         "rate": 100.0,
    #         "use_sim_time": True
    #     }],
    #     output='screen'
    # )

    # load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #             'joint_state_broadcaster'],
    #     output='screen'
    # )

    # load_base_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #             'base_controller'],
    #     output='screen'
    # )

    # return LaunchDescription([
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=spawn_node,
    #             on_exit=[load_joint_state_controller],
    #         )
    #     ),
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=load_joint_state_controller,
    #             on_exit=[load_base_controller],
    #         )
    #     ),
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=load_base_controller,gazebo_ros
    #     gz_sim,
    #     ign_image_bridge,
    #     spawn_node,
    #     jsp_node,
    #     static_tf_lidar,
    #     static_tf_camera,
    #     static_tf_imu,
    # ])

