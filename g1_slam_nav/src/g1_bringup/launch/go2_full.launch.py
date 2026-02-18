"""
Full Go2 SLAM + Navigation launch.
Parameterized for Go2's 12-joint quadruped configuration.
Usage: ros2 launch g1_bringup go2_full.launch.py
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
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        DeclareLaunchArgument('lidar_config', default_value=''),

        # Sensors (Go2 config: 12 joints, LiDAR not flipped)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sensor_bridge_dir, 'launch', 'sensor_bridge.launch.py')
            ),
            launch_arguments={
                'robot_type': 'go2',
                'network_interface': LaunchConfiguration('network_interface'),
                'lidar_config': LaunchConfiguration('lidar_config'),
                'flip_lidar': 'false',
            }.items(),
        ),

        # SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, 'launch', 'slam.launch.py')
            ),
        ),

        # Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_dir, 'launch', 'navigation.launch.py')
            ),
        ),

        # Web
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(web_dir, 'launch', 'web_interface.launch.py')
            ),
        ),
    ])
