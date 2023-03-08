import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command

def generate_launch_description():
    ld = LaunchDescription()
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")

    upload_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('former_description'),
            '/launch/upload_robot.launch.py']
        ),
        launch_arguments = {
            'use_gazebo_sim': 'false'
        }.items()
    )

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('former_description'),
            'urdf/robot.urdf.xacro',
        ]),
        ' use_gazebo_sim:=', 'false'
    ])

    robot_description = {"robot_description": robot_description_content}

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('former_bringup'),
                'config/ekf.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    robot_controllers = PathJoinSubstitution([
            FindPackageShare('former_bringup'),
            "config",
            "controllers_former.yaml"
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            robot_controllers
            ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        respawn=True,
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )

    load_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'base_controller'],
        output='screen'
    )

    lidar_bringup = Node(
        package="sick_scan",
        executable="sick_generic_caller",
        respawn=True,
        arguments=[
            'scanner_type:=sick_tim_5xx',
            'hostname:=192.168.10.11',
            'frame_id:=laser_link',
            'min_ang:=-2.0',
            'max_ang:=2.0',
            'nodename:=front_lidar',
            'range_min:=0.05',
            'sw_pll_only_publish:=false',
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        remappings=[
            ('front_lidar/scan', 'scan')
        ]
    )

    imu_bringup = Node(
        package="imu_xg6000_ros2",
        executable="main_node",
        respawn=True,
        parameters=[
            {'port_name': '/dev/ttyIMU'},
            {'baudrate': 38400},
            {'frame_id': 'imu_link'},
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        }
    )


    ld.add_action(use_sim_time)
    ld.add_action(upload_robot)
    ld.add_action(control_node)
    ld.add_action(load_joint_state_broadcaster)
    # ld.add_action(robot_localization_node)
    ld.add_action(load_base_controller)
    # ld.add_action(lidar_bringup)
    # ld.add_action(imu_bringup)
    return ld