"""
Point-LIO mapping with Livox Mid-360.
Requires livox_ros_driver2 to be built and sourced; run msg_MID360_launch.py first (or in another terminal).

Usage:
  # Terminal 1 (after sourcing livox_ros_driver2 install):
  ros2 launch livox_ros_driver2 msg_MID360_launch.py
  # Terminal 2 (after sourcing this workspace):
  ros2 launch g1_bringup point_lio_mapping.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os


def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare('g1_bringup'),
        'config', 'point_lio_mid360.yaml'
    ])

    laser_mapping_params = [
        config_path,
        {
            'use_imu_as_input': False,
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 1,
            'space_down_sample': True,
            'filter_size_surf': 0.3,
            'filter_size_map': 0.2,
            'cube_side_length': 2000.0,
            'runtime_pos_log_enable': False,
        },
    ]

    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz (use point_lio rviz config if true).'
    )
    use_web_arg = DeclareLaunchArgument(
        'use_web', default_value='true',
        description='Launch web interface (pointcloud, path, trajectory) at http://<robot-ip>:8080'
    )
    use_occ_grid_arg = DeclareLaunchArgument(
        'use_occ_grid', default_value='false',
        description='Run Point-LIO occupancy grid node (Laser_map -> /map for Nav2).'
    )

    point_lio_occ_grid_node = Node(
        package='g1_slam',
        executable='point_lio_occupancy_grid_node',
        name='point_lio_occupancy_grid_node',
        output='screen',
        parameters=[{
            'map_topic': 'Laser_map',
            'odom_topic': 'aft_mapped_to_init',
            'resolution': 0.05,
            'width': 200.0,
            'height': 200.0,
            'map_frame': 'camera_init',
        }],
        condition=IfCondition(LaunchConfiguration('use_occ_grid')),
    )

    point_lio_map_manager_node = Node(
        package='g1_slam',
        executable='map_manager_node',
        name='map_manager_node',
        output='screen',
        parameters=[{
            'map_directory': '/home/unitree/maps',
            'map_frame': 'camera_init',
            'map_topic': 'Laser_map',
            'grid_topic': 'map',
            'map_pub_topic': 'loaded_map',
            'grid_pub_topic': 'map',
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('g1_bringup'),
            'rviz', 'point_lio.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    web_bridge_node = Node(
        package='g1_web_interface',
        executable='web_bridge_node',
        name='web_bridge',
        output='screen',
        parameters=[{
            'ws_port': 9090,
            'http_port': 8080,
            'map_downsample': 4,
            'map_publish_rate': 1.0,
            'pose_publish_rate': 10.0,
            'use_point_lio': True,
        }],
        condition=IfCondition(LaunchConfiguration('use_web')),
    )

    return LaunchDescription([
        rviz_arg,
        use_web_arg,
        use_occ_grid_arg,
        laser_mapping_node,
        point_lio_occ_grid_node,
        point_lio_map_manager_node,
        rviz_node,
        web_bridge_node,
    ])
