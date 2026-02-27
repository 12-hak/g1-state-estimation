import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('g1_bringup')
    # Try rviz folder in share, fallback to relative src if running from devel
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'g1_default.rviz')
    
    # Fallback to source directory if running locally
    if not os.path.exists(rviz_config_path):
        # Assuming we are in a dev environment and can find the file relative to current path
        # or from a known structure
        current_dir = os.path.dirname(os.path.realpath(__file__))
        rviz_config_path = os.path.abspath(os.path.join(current_dir, '..', 'rviz', 'g1_default.rviz'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_path,
            description='Full path to the RViz config file to use'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen'
        )
    ])
