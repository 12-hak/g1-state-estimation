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

    return LaunchDescription([
        rviz_arg,
        laser_mapping_node,
        rviz_node,
    ])
