import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wheel_legged_odometry')
    default_config_dir = '/external/config/wheel_legged_odometry'
    default_urdf = os.path.join(pkg_share, 'urdf', 'go2w', 'go2w.urdf')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    lowstate_topic = LaunchConfiguration('lowstate_topic')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    with open(default_urdf, 'r', encoding='utf-8') as f:
        default_robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_path', default_value=default_config_dir),
        DeclareLaunchArgument('config_file', default_value='go2w.yaml'),
        DeclareLaunchArgument('lowstate_topic', default_value='lowstate'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_cfg',
            default_value=os.path.join(
                default_config_dir, 'wheel_legged_odometry.rviz')),

        Node(
            package='wheel_legged_odometry',
            executable='wheel_legged_odometry_node',
            name='wheel_legged_odometry',
            output='screen',
            parameters=[
                PathJoinSubstitution([config_path, config_file]),
                {
                    'use_sim_time': use_sim_time,
                    'lowstate_topic': lowstate_topic,
                },
            ],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='wheel_legged_robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': default_robot_description,
                },
            ],
            remappings=[
                ('joint_states', '/wheel_legged_odometry/joint_states'),
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                },
            ],
            condition=IfCondition(rviz_use),
        ),
    ])
