import os
from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    use_gazebo_sim = DeclareLaunchArgument("use_gazebo_sim", default_value="false")

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'ignore_timestamp': False,
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        get_package_share_directory('former_description'),
                        'urdf/robot.urdf.xacro',
                    ]),
                    ' use_gazebo_sim:=', LaunchConfiguration('use_gazebo_sim')
                ]),
        }]
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            "source_list": ['joint_states'],
            "rate": 100.0,
            "use_sim_time": False
        }],
        output='screen'
    )

    ld.add_action(use_gazebo_sim)
    ld.add_action(rsp_node)
    ld.add_action(jsp_node)

    return ld