"""Launch SLAM Toolbox and RViz for map creation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the online mapping launch description."""
    bringup_share = get_package_share_directory('care_robot_bringup')
    slam_share = get_package_share_directory('slam_toolbox')

    default_params = os.path.join(
        bringup_share, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(
        bringup_share, 'rviz', 'follow_config.rviz')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Absolute path to SLAM Toolbox parameters.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    slam_share, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': params_file,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
