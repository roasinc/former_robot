import os

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnExecutionComplete, OnProcessStart, OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    use_sim_time_cmd = DeclareLaunchArgument("use_sim_time", default_value="false")
    use_namespace_cmd = DeclareLaunchArgument("use_namespace", default_value="false")
    namespace_cmd = DeclareLaunchArgument("namespace", default_value="")
    tf_prefix = DeclareLaunchArgument("tf_prefix", default_value="")

    robot_controllers = PathJoinSubstitution([
            FindPackageShare('former_bringup'),
            "config",
            "controllers_former.yaml"
        ]
    )

    bringup_robot_group = GroupAction([
            PushRosNamespace(
                condition=IfCondition(LaunchConfiguration('use_namespace')),
                namespace=LaunchConfiguration('namespace')
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('former_description'), '/launch/upload_robot.launch.py']
                ),
                launch_arguments = {
                    'use_gazebo_sim': 'false',
                    'tf_prefix': [LaunchConfiguration('namespace'), "_"]

                }.items()
            ),
            # Node(
            #     package='robot_localization',
            #     executable='ekf_node',
            #     name='ekf_filter_node',
            #     output='screen',
            #     parameters=[
            #         PathJoinSubstitution([
            #             FindPackageShare('former_bringup'),
            #             'config/ekf.yaml'
            #         ]),
            #         {
            #             'use_sim_time': LaunchConfiguration('use_sim_time')
            #         }
            #     ],
            # ),
            Node(
                condition=IfCondition(LaunchConfiguration('use_namespace')),
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    ParameterFile(
                        RewrittenYaml(
                            source_file=robot_controllers,
                            param_rewrites={},
                            root_key=LaunchConfiguration('namespace')
                        ),
                        allow_substs=True
                    )
                ],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
                respawn=False,
                remappings=[
                    ('~/robot_description', ['/', LaunchConfiguration('namespace'),'/robot_description'])
                ]
            ),
            Node(
                condition=UnlessCondition(LaunchConfiguration('use_namespace')),
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    robot_controllers
                ],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
                respawn=False,
                remappings=[
                    ('~/robot_description', '/robot_description')
                ]
            ),

        ]
    )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_cmd)
    ld.add_action(use_namespace_cmd)
    ld.add_action(namespace_cmd)
    ld.add_action(tf_prefix)
    ld.add_action(bringup_robot_group)

    return ld



    # load_joint_state_broadcaster = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    # load_base_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["base_controller", "--controller-manager", "/controller_manager"],
    # )

    # load_former_io_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["former_io_controller", "--controller-manager", "/controller_manager"],
    # )

    # lidar_bringup = Node(
    #     package="sick_scan_xd",
    #     executable="sick_generic_caller",
    #     respawn=True,
    #     arguments=[
    #         PathJoinSubstitution([
    #             FindPackageShare('sick_scan_xd'),
    #             'launch/sick_tim_5xx.launch'
    #         ]),
    #         'hostname:=192.168.10.11',
    #         'frame_id:=laser_link',
    #         'min_ang:=-1.85',
    #         'max_ang:=1.85',
    #         'nodename:=front_lidar',
    #         'range_min:=0.05',
    #         'sw_pll_only_publish:=false',
    #     ],
    #     output={
    #         "stdout": "screen",
    #         "stderr": "screen",
    #     },
    #     remappings=[
    #         ('front_lidar/scan', 'scan')
    #     ]
    # )

    # auto_docking_bringup = Node(
    #     package="former_auto_docking",
    #     executable="auto_docking_node",
    #     respawn=True,
    #     parameters=[
    #         {"distance_approach": 0.260},
    #     ],
    #     remappings=[
    #         ('odom', 'base_controller/odom'),
    #         ('cmd_vel', 'base_controller/cmd_vel_unstamped')
    #     ],
    #     output={
    #         "stdout": "screen",
    #         "stderr": "screen",
    #     }
    # )

    # gpio_board_bringup = Node(
    #     package="former_gpio_board",
    #     executable="main_node",
    #     respawn=True,
    #     parameters=[
    #         {"port_name": "/dev/ttyARDUINO"},
    #         {"baudrate": 115200},
    #     ],
    #     output={
    #         "stdout": "screen",
    #         "stderr": "screen",
    #     }
    # )

    # imu_bringup = Node(
    #     package="mw_ahrs_ros2",
    #     executable="main_node",
    #     name='imu_node',
    #     respawn=True,
    #     parameters=[
    #         {'port_name': '/dev/ttyIMU'},
    #         {'baudrate': 921600},
    #         {'frame_id': 'imu_link'},
    #         {'rate': 100.0},
    #     ],
    #     output={
    #         "stdout": "screen",
    #         "stderr": "screen",
    #     }
    # )

    # realsense2_bringup = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('realsense2_camera'),
    #         '/launch/rs_launch.py']
    #     ),
    #     launch_arguments = {
    #         'enable_sync': 'true',
    #         'pointcloud.enable': 'true',
    #         'decimation_filter.enable': 'true',
    #         'rgb_camera.profile': '640,480,30',
    #         'depth_module.profile': '640,480,30',
    #     }.items()
    # )

    # joy_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     name='joy_node',
    #     parameters=[{
    #         'dev': '/dev/js0',
    #         'deadzone': 0.3,
    #         'autorepeat_rate': 20.0,
    #     }],
    # )

    # teleop_joy_node = Node(
    #     package='teleop_twist_joy',
    #     executable='teleop_node',
    #     name='teleop_twist_joy_node',
    #     parameters=[
    #         PathJoinSubstitution([
    #             FindPackageShare('former_bringup'),
    #             'config/ps5.config.yaml'
    #         ]),
    #     ],
    #     remappings=[
    #         ('cmd_vel', 'base_controller/cmd_vel_unstamped')
    #     ]
    # )

    # return LaunchDescription([
    #     use_sim_time,
    #     upload_robot,
    #     control_manager_node,
    #     RegisterEventHandler(
    #         event_handler=OnProcessStart(
    #             target_action=control_manager_node,
    #             on_start=[
    #                 LogInfo(msg='control_manager_node started, spawn controllers'),
    #                 load_joint_state_broadcaster,
    #             ]
    #         )
    #     ),
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=load_joint_state_broadcaster,
    #             on_exit=[load_base_controller],
    #         )
    #     ),
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=load_base_controller,
    #             on_exit=[load_former_io_controller],
    #         )
    #     ),
    #     # # robot_localization_node,
    #     lidar_bringup,
    #     imu_bringup,
    #     gpio_board_bringup,
    #     auto_docking_bringup,
    #     realsense2_bringup,
    #     joy_node,
    #     teleop_joy_node
    # ])
