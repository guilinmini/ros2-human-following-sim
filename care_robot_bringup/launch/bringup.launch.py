"""Launch the complete simulated human-following system."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Compose simulation, Nav2, and the follower into one entry point."""
    bringup_share = get_package_share_directory('care_robot_bringup')
    simulation_share = get_package_share_directory('care_robot_gazebo')

    default_world = os.path.join(
        simulation_share, 'worlds', 'maze4.world')
    default_map = os.path.join(bringup_share, 'maps', 'follow_map.yaml')
    default_nav2_params = os.path.join(
        bringup_share, 'config', 'nav2_params.yaml')
    default_follower_params = os.path.join(
        bringup_share, 'config', 'follower.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('nav2_params')
    follower_params = LaunchConfiguration('follower_params')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    start_simulation = LaunchConfiguration('start_simulation')
    start_navigation = LaunchConfiguration('start_navigation')
    start_follower = LaunchConfiguration('start_follower')
    track_actor = LaunchConfiguration('track_actor')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'nav2_params', default_value=default_nav2_params),
        DeclareLaunchArgument(
            'follower_params', default_value=default_follower_params),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('start_simulation', default_value='true'),
        DeclareLaunchArgument('start_navigation', default_value='true'),
        DeclareLaunchArgument('start_follower', default_value='true'),
        DeclareLaunchArgument('track_actor', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    simulation_share, 'launch', 'simulation.launch.py')),
            condition=IfCondition(start_simulation),
            launch_arguments={
                'world': world,
                'gui': gui,
                'use_sim_time': use_sim_time,
                'track_actor': track_actor,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    bringup_share, 'launch', 'localization.launch.py')),
            condition=IfCondition(start_navigation),
            launch_arguments={
                'map': map_file,
                'params_file': nav2_params,
                'use_sim_time': use_sim_time,
                'use_rviz': use_rviz,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    bringup_share, 'launch', 'following.launch.py')),
            condition=IfCondition(start_follower),
            launch_arguments={
                'params_file': follower_params,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])
