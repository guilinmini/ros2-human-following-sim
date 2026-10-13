"""Launch Gazebo Sim Harmonic, the robot, and ROS-Gazebo bridges."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the Gazebo Sim launch description."""
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    simulation_share = get_package_share_directory('care_robot_gazebo')
    description_share = get_package_share_directory('care_robot_description')

    default_world = os.path.join(
        simulation_share, 'worlds', 'maze4.world')
    robot_file = os.path.join(
        description_share, 'urdf', 'care_robot.urdf')
    models_path = os.path.join(simulation_share, 'models')
    with open(robot_file, encoding='utf-8') as robot_stream:
        robot_description = robot_stream.read()

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    track_actor = LaunchConfiguration('track_actor')

    bridge_arguments = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/model/care_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/model/care_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/model/care_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        (
            '/world/default/model/care_robot/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model'
        ),
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        (
            '/robot_camera_sensor/image'
            '@sensor_msgs/msg/Image[gz.msgs.Image'
        ),
        (
            '/robot_camera_sensor/camera_info'
            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ),
        (
            '/robot_camera_sensor/depth_image'
            '@sensor_msgs/msg/Image[gz.msgs.Image'
        ),
        (
            '/world/default/set_pose'
            '@ros_gz_interfaces/srv/SetEntityPose'
        ),
    ]

    bridge_remappings = [
        ('/model/care_robot/cmd_vel', '/cmd_vel'),
        ('/model/care_robot/odometry', '/odom'),
        ('/model/care_robot/tf', '/tf'),
        (
            '/world/default/model/care_robot/joint_state',
            '/joint_states',
        ),
    ]

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            [
                models_path,
                os.pathsep,
                EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ],
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Absolute path to a Gazebo Sim world file.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo Sim graphical client.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo simulation clock.',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='care_robot',
            description='Spawned entity name; bridge paths expect care_robot.',
        ),
        DeclareLaunchArgument('spawn_x', default_value='-5.5'),
        DeclareLaunchArgument('spawn_y', default_value='2.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.11'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'track_actor',
            default_value='true',
            description='Keep the actor non-contact proxy synchronized.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
            condition=IfCondition(gui),
            launch_arguments={
                'gz_args': ['-r -v 3 ', world],
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
            condition=UnlessCondition(gui),
            launch_arguments={
                'gz_args': ['-r -s --headless-rendering -v 3 ', world],
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    robot_description, value_type=str),
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_care_robot',
            output='screen',
            arguments=[
                '-name', robot_name,
                '-topic', 'robot_description',
                '-x', spawn_x,
                '-y', spawn_y,
                '-z', spawn_z,
                '-Y', spawn_yaw,
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            arguments=bridge_arguments,
            remappings=bridge_remappings,
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_gazebo_frame',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'lidar',
                '--child-frame-id', 'care_robot/base_link/lidar_sensor',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_gazebo_frame',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'camera_link_optical',
                '--child-frame-id',
                'care_robot/base_link/robot_camera_sensor',
            ],
        ),
        Node(
            package='care_robot',
            executable='actor_proxy_follower',
            name='actor_proxy_follower',
            output='screen',
            condition=IfCondition(track_actor),
            parameters=[{
                'use_sim_time': use_sim_time,
                'world_file': ParameterValue(world, value_type=str),
                'set_pose_service': '/world/default/set_pose',
            }],
        ),
    ])
