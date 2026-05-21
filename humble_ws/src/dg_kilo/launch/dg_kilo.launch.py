import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('dg_kilo')
    default_config_dir = '/external/config/dgkilo'

    use_sim_time     = LaunchConfiguration('use_sim_time')
    config_path      = LaunchConfiguration('config_path')
    config_file      = LaunchConfiguration('config_file')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    imu_topic        = LaunchConfiguration('imu_topic')
    lowstate_topic   = LaunchConfiguration('lowstate_topic')
    rviz_use         = LaunchConfiguration('rviz')
    rviz_cfg         = LaunchConfiguration('rviz_cfg')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',     default_value='false'),
        DeclareLaunchArgument('config_path',      default_value=default_config_dir),
        DeclareLaunchArgument('config_file',      default_value='go2w.yaml'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/points_raw'),
        DeclareLaunchArgument('imu_topic',        default_value='/go2w/imu'),
        DeclareLaunchArgument('lowstate_topic',   default_value='lowstate'),
        DeclareLaunchArgument('rviz',             default_value='true'),
        DeclareLaunchArgument('rviz_cfg',
            default_value=os.path.join(default_config_dir, 'dg_kilo.rviz')),

        Node(
            package='dg_kilo',
            executable='dg_kilo_node',
            name='dg_kilo',
            output='screen',
            parameters=[
                PathJoinSubstitution([config_path, config_file]),
                {
                    'use_sim_time':     use_sim_time,
                    'pointcloud_topic': pointcloud_topic,
                    'imu_topic':        imu_topic,
                    'lowstate_topic':   lowstate_topic,
                },
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(rviz_use),
        ),
    ])
