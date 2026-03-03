"""
Launch Nav2 + nav_command_node + web interface for use with Point-LIO.
Expects /map (OccupancyGrid, frame camera_init) and tf (camera_init -> base_link) from the robot
(e.g. from point_lio_mapping.launch.py with use_occ_grid:=true on the Jetson, or from another client).

Usage:
  # On robot (Jetson): Point-LIO + occupancy grid + map manager + web (optional)
  ros2 launch g1_bringup point_lio_mapping.launch.py use_occ_grid:=true

  # On this machine (same network, same ROS_DOMAIN_ID): Nav2 + web for waypoint navigation
  ros2 launch g1_bringup point_lio_navigation.launch.py

  # Or run Nav2 on the same machine as Point-LIO (single machine)
  ros2 launch g1_bringup point_lio_mapping.launch.py use_occ_grid:=true
  ros2 launch g1_bringup point_lio_navigation.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    nav_dir = get_package_share_directory('g1_navigation')
    point_lio_params = os.path.join(nav_dir, 'params', 'nav2_params_point_lio.yaml')
    nav_launch_path = os.path.join(nav_dir, 'launch', 'navigation.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('use_web', default_value='true',
                              description='Launch web interface for waypoint/navigate commands.'),
        DeclareLaunchArgument('params_file', default_value=point_lio_params),
        DeclareLaunchArgument('map_frame', default_value='camera_init'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_path),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'map_frame': LaunchConfiguration('map_frame'),
            }.items(),
        ),
        Node(
            package='g1_web_interface',
            executable='web_bridge_node',
            name='web_bridge',
            output='screen',
            parameters=[{
                'ws_port': 9090,
                'http_port': 8080,
                'use_point_lio': True,
            }],
            condition=IfCondition(LaunchConfiguration('use_web')),
        ),
    ])
