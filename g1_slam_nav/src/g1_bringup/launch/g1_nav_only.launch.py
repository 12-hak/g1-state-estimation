"""
Navigation-only mode: localize against a saved map and navigate.
Usage: ros2 launch g1_bringup g1_nav_only.launch.py map_file:=/path/to/map.pcd
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
        DeclareLaunchArgument('map_file', description='Path to saved PCD map'),
        DeclareLaunchArgument('map_yaml_file', default_value='',
                              description='Path to occupancy grid YAML for Nav2'),

        # Sensors
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sensor_bridge_dir, 'launch', 'sensor_bridge.launch.py')
            ),
            launch_arguments={
                'robot_type': LaunchConfiguration('robot_type'),
                'network_interface': LaunchConfiguration('network_interface'),
            }.items(),
        ),

        # SLAM in localization mode (loop closure off, loads prior map)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, 'launch', 'slam.launch.py')
            ),
            launch_arguments={
                'enable_loop_closure': 'false',
            }.items(),
        ),

        # Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_dir, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={
                'map_yaml_file': LaunchConfiguration('map_yaml_file'),
            }.items(),
        ),

        # Web
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(web_dir, 'launch', 'web_interface.launch.py')
            ),
        ),
    ])
