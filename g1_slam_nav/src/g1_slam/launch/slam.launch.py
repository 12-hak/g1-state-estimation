from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('g1_slam')
    install_prefix = os.path.dirname(
        os.path.dirname(pkg_dir))
    slam_node_path = os.path.join(
        install_prefix, 'lib', 'g1_slam', 'slam_node')
    if not os.path.isfile(slam_node_path):
        return LaunchDescription([
            LogInfo(
                msg='g1_slam not built (PCL missing). '
                    'Skipping SLAM nodes.'),
        ])

    config_file = os.path.join(pkg_dir, 'config', 'slam_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_loop_closure', default_value='true'),
        DeclareLaunchArgument(
            'map_frame', default_value='map'),
        DeclareLaunchArgument(
            'flip_lidar', default_value='true'),

        Node(
            package='g1_slam',
            executable='slam_node',
            name='slam_node',
            output='screen',
            parameters=[config_file, {
                'enable_loop_closure': LaunchConfiguration(
                    'enable_loop_closure'),
                'map_frame': LaunchConfiguration('map_frame'),
                'flip_lidar': LaunchConfiguration('flip_lidar'),
                'lidar_offset_x': 0.10,
                'lidar_offset_y': 0.0,
                'lidar_offset_z': 0.60,
            }],
        ),

        Node(
            package='g1_slam',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[config_file, {
                'map_frame': LaunchConfiguration('map_frame'),
            }],
        ),

        Node(
            package='g1_slam',
            executable='map_manager_node',
            name='map_manager_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
