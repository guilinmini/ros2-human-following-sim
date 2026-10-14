"""Launch Nav2 localization/navigation and the project RViz view."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Build localization and navigation launch actions."""
    bringup_share = get_package_share_directory('care_robot_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(bringup_share, 'maps', 'follow_map.yaml')
    default_params = os.path.join(
        bringup_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(
        bringup_share, 'rviz', 'follow_config.rviz')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'default_nav_to_pose_bt_xml': os.path.join(
                bringup_share, 'config', 'navigate_to_pose_ackermann.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(
                bringup_share, 'config', 'navigate_through_poses_ackermann.xml'),
        },
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Absolute path to the occupancy map YAML file.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Absolute path to Nav2 parameters.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'params_file': configured_params,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                # Nav2 evaluates these values in PythonExpression.
                'slam': 'False',
                'use_composition': 'False',
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
