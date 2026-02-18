"""
Full G1 SLAM + Navigation + Web Interface launch.
Usage: ros2 launch g1_bringup g1_full.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sensor_bridge_dir = get_package_share_directory('g1_sensor_bridge')
    slam_dir = get_package_share_directory('g1_slam')
    nav_dir = get_package_share_directory('g1_navigation')
    web_dir = get_package_share_directory('g1_web_interface')

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='g1'),
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        DeclareLaunchArgument('lidar_config', default_value=''),
        DeclareLaunchArgument('enable_loop_closure', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('ws_port', default_value='9090'),
        DeclareLaunchArgument('http_port', default_value='8080'),

        # 1. Sensor Bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sensor_bridge_dir, 'launch', 'sensor_bridge.launch.py')
            ),
            launch_arguments={
                'robot_type': LaunchConfiguration('robot_type'),
                'network_interface': LaunchConfiguration('network_interface'),
                'lidar_config': LaunchConfiguration('lidar_config'),
                'flip_lidar': 'true',
            }.items(),
        ),

        # 2. SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, 'launch', 'slam.launch.py')
            ),
            launch_arguments={
                'enable_loop_closure': LaunchConfiguration('enable_loop_closure'),
            }.items(),
        ),

        # 3. Navigation (Nav2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_dir, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),

        # 4. Web Interface
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(web_dir, 'launch', 'web_interface.launch.py')
            ),
            launch_arguments={
                'ws_port': LaunchConfiguration('ws_port'),
                'http_port': LaunchConfiguration('http_port'),
            }.items(),
        ),
    ])
