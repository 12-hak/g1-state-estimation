from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    g1_nav_dir = get_package_share_directory('g1_navigation')
    install_prefix = os.path.dirname(os.path.dirname(g1_nav_dir))
    nav_command_path = os.path.join(install_prefix, 'lib', 'g1_navigation', 'nav_command_node')
    have_nav_command = os.path.isfile(nav_command_path)

    ld = [
        DeclareLaunchArgument('map_yaml_file', default_value='',
                              description='Path to map YAML for localization mode'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
    ]

    try:
        nav2_bringup_dir = get_package_share_directory('nav2_bringup')
        nav2_params = os.path.join(g1_nav_dir, 'params', 'nav2_params.yaml')
        ld.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'params_file': nav2_params,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': LaunchConfiguration('autostart'),
                }.items(),
            )
        )
    except Exception:
        ld.append(LogInfo(msg='nav2_bringup not found; skipping Nav2 stack. Install ros-humble-nav2-bringup if needed.'))

    if have_nav_command:
        ld.append(
            Node(
                package='g1_navigation',
                executable='nav_command_node',
                name='nav_command_node',
                output='screen',
                parameters=[{'map_frame': 'map'}],
            )
        )
    else:
        ld.append(LogInfo(msg='nav_command_node not built (nav2_msgs not found). Install ros-humble-nav2-msgs and rebuild for web->Nav2 command bridge.'))

    return LaunchDescription(ld)
