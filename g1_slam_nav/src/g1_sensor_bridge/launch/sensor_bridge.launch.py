from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='g1',
                              description='Robot type: g1 or go2'),
        DeclareLaunchArgument('network_interface', default_value='eth0',
                              description='Network interface for Unitree SDK2 DDS'),
        DeclareLaunchArgument('lidar_config', default_value='',
                              description='Path to Livox Mid-360 config JSON'),
        DeclareLaunchArgument('flip_lidar', default_value='true',
                              description='Flip LiDAR data for upside-down mount (G1)'),

        Node(
            package='g1_sensor_bridge',
            executable='livox_bridge_node',
            name='livox_bridge',
            output='screen',
            parameters=[{
                'frame_id': 'lidar_link',
                'lidar_config_path': LaunchConfiguration('lidar_config'),
                'flip_lidar': LaunchConfiguration('flip_lidar'),
                'body_filter_radius': 0.3,
                'max_range': 15.0,
                'min_range': 0.15,
                'publish_rate': 10.0,
            }],
        ),

        Node(
            package='g1_sensor_bridge',
            executable='unitree_bridge_node',
            name='unitree_bridge',
            output='screen',
            parameters=[{
                'network_interface': LaunchConfiguration('network_interface'),
                'robot_type': LaunchConfiguration('robot_type'),
                'publish_rate': 50.0,
            }],
        ),

        Node(
            package='g1_sensor_bridge',
            executable='leg_odometry_node',
            name='leg_odometry',
            output='screen',
            parameters=[{
                'robot_type': LaunchConfiguration('robot_type'),
                'step_scale': 0.6,
                'lateral_scale': 0.4,
                'velocity_deadband': 0.10,
                'publish_tf': False,  # SLAM will publish the authoritative tf
                'publish_rate': 50.0,
            }],
        ),
    ])
