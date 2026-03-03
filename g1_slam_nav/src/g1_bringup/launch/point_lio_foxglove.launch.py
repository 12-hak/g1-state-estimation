"""
Point-LIO mapping + Foxglove bridge for browser/desktop visualization (point cloud, path, G1 model).

Requires: ros-$ROS_DISTRO-foxglove-bridge
  sudo apt install ros-humble-foxglove-bridge

Usage:
  # Terminal 1: Livox driver
  ros2 launch livox_ros_driver2 msg_MID360_launch.py
  # Terminal 2: Point-LIO + Foxglove
  ros2 launch g1_bringup point_lio_foxglove.launch.py

Then open https://app.foxglove.dev → Open connection → Foxglove WebSocket →
  ws://<robot-ip>:8765

Layout: Add 3D panel; add topics /Laser_map, /cloud_registered (PointCloud2), /path (Path).
Add URDF: set source to "Topic" and topic /robot_description. Fixed frame: camera_init.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix


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

    foxglove_arg = DeclareLaunchArgument(
        'foxglove', default_value='true',
        description='Start Foxglove bridge for app.foxglove.dev'
    )
    use_web_arg = DeclareLaunchArgument(
        'use_web', default_value='true',
        description='Start web UI (point cloud, path) at http://<robot-ip>:8080'
    )

    # Topic whitelist: point clouds, path, odom, TF, robot_description (for G1 model)
    topic_whitelist = [
        '/Laser_map',
        '/cloud_registered',
        '/path',
        '/aft_mapped_to_init',
        '/tf',
        '/tf_static',
        '/robot_description',
    ]

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'topic_whitelist': topic_whitelist,
        }],
        condition=IfCondition(LaunchConfiguration('foxglove')),
    )

    # Publish G1 minimal URDF to /robot_description so Foxglove 3D panel can show the robot
    pkg_share = get_package_share_directory('g1_bringup')
    urdf_path = os.path.join(pkg_share, 'urdf', 'g1_minimal.urdf')
    pkg_prefix = get_package_prefix('g1_bringup')
    script_path = os.path.join(pkg_prefix, 'lib', 'g1_bringup', 'publish_robot_description.py')
    robot_description_node = ExecuteProcess(
        cmd=['python3', script_path, urdf_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('foxglove')),
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
        foxglove_arg,
        use_web_arg,
        laser_mapping_node,
        foxglove_bridge_node,
        robot_description_node,
        web_bridge_node,
    ])
