"""
SLAM-only mode: sensors + SLAM + web viz (no navigation).
Use this for mapping exploration before deploying autonomous nav.
Usage: ros2 launch g1_bringup g1_slam_only.launch.py
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
    web_dir = get_package_share_directory('g1_web_interface')

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='g1'),
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        DeclareLaunchArgument('lidar_config', default_value=''),
        DeclareLaunchArgument('enable_loop_closure', default_value='true'),
        DeclareLaunchArgument('start_unitree_bridge', default_value='true',
                              description='Set false to skip unitree/leg_odometry (e.g. if unitree_bridge_node causes bad_alloc)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sensor_bridge_dir, 'launch', 'sensor_bridge.launch.py')
            ),
            launch_arguments={
                'robot_type': LaunchConfiguration('robot_type'),
                'network_interface': LaunchConfiguration('network_interface'),
                'lidar_config': LaunchConfiguration('lidar_config'),
                'start_unitree_bridge': LaunchConfiguration('start_unitree_bridge'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, 'launch', 'slam.launch.py')
            ),
            launch_arguments={
                'enable_loop_closure': LaunchConfiguration('enable_loop_closure'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(web_dir, 'launch', 'web_interface.launch.py')
            ),
        ),
    ])
