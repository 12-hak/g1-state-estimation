from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    g1_nav_dir = get_package_share_directory('g1_navigation')
    nav2_params = os.path.join(g1_nav_dir, 'params', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map_yaml_file', default_value='',
                              description='Path to map YAML for localization mode'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),

        # Nav2 full stack via its bringup launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
            }.items(),
        ),

        # Our navigation command node (bridges web UI to Nav2 actions)
        Node(
            package='g1_navigation',
            executable='nav_command_node',
            name='nav_command_node',
            output='screen',
            parameters=[{
                'map_frame': 'map',
            }],
        ),
    ])
