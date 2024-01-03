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
    tf_prefix_cmd = DeclareLaunchArgument("tf_prefix", default_value="")

    robot_controllers = PathJoinSubstitution([
            FindPackageShare('former_bringup'),
            "config",
            "controllers_former.yaml"
        ]
    )

    upload_robot_group = GroupAction([
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
                'namespace': LaunchConfiguration('namespace'),
                'tf_prefix': LaunchConfiguration('tf_prefix'),
            }.items()
        )
    ])

    # robot_localization_ekf = Node(
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

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            ParameterFile(
                RewrittenYaml(
                    source_file=robot_controllers,
                    param_rewrites={},
                    root_key=LaunchConfiguration('namespace')
                ),
                allow_substs=True
            ),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        respawn=False,
        remappings=[
            ('~/robot_description',  'robot_description')
        ]
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration('namespace'),
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
    )

    load_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration('namespace'),
        arguments=["base_controller", "--controller-manager", "controller_manager"],
    )

    load_former_io_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration('namespace'),
        arguments=["former_io_controller", "--controller-manager", "controller_manager"],
    )

    lidar_bringup = Node(
        package="sick_scan_xd",
        executable="sick_generic_caller",
        namespace=LaunchConfiguration('namespace'),
        respawn=True,
        arguments=[
            PathJoinSubstitution([
                FindPackageShare('sick_scan_xd'),
                'launch/sick_tim_5xx.launch'
            ]),
            'hostname:=192.168.10.11',
            'frame_id:=laser_link',
            'min_ang:=-1.85',
            'max_ang:=1.85',
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

    auto_docking_bringup = Node(
        package="former_auto_docking",
        executable="auto_docking_node",
        name="former_auto_docking",
        namespace=LaunchConfiguration('namespace'),
        respawn=True,
        parameters=[
            {"distance_approach": 0.260},
        ],
        remappings=[
            ('odom', 'base_controller/odom'),
            ('cmd_vel', 'base_controller/cmd_vel_unstamped')
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        }
    )

    gpio_board_bringup = Node(
        package="former_gpio_board",
        executable="main_node",
        name="former_gpio_board",
        namespace=LaunchConfiguration('namespace'),
        respawn=True,
        parameters=[
            {"port_name": "/dev/ttyARDUINO"},
            {"baudrate": 115200},
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        }
    )

    imu_bringup = Node(
        package="mw_ahrs_ros2",
        executable="main_node",
        name='imu_node',
        namespace=LaunchConfiguration('namespace'),
        respawn=True,
        parameters=[
            {'port_name': '/dev/ttyIMU'},
            {'baudrate': 921600},
            {'frame_id': [LaunchConfiguration('namespace'), '/imu_link']},
            {'rate': 100.0},
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        }
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'dev': '/dev/js0',
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }],
    )

    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('former_bringup'),
                'config/ps5.config.yaml'
            ]),
        ],
        remappings=[
            ('cmd_vel', 'base_controller/cmd_vel_unstamped')
        ]
    )

    return LaunchDescription([
        use_sim_time_cmd,
        use_namespace_cmd,
        namespace_cmd,
        tf_prefix_cmd,
        # robot_localization_ekf,
        upload_robot_group,
        controller_manager,
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager,
                on_start=[
                    LogInfo(msg='controller_manager started, spawn controllers'),
                    load_joint_state_broadcaster,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[
                    LogInfo(msg='controller_manager started, spawn controllers'),
                    load_base_controller,
                    load_former_io_controller,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_former_io_controller,
                on_exit=[
                    LogInfo(msg='controllers are spawn completed. run applications'),
                    auto_docking_bringup,
                    gpio_board_bringup,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=gpio_board_bringup,
                on_start=[
                    LogInfo(msg='application are started, run sensors'),
                    lidar_bringup,
                    imu_bringup,
                    joy_node,
                    teleop_joy_node
                ]
            )
        )
    ])
