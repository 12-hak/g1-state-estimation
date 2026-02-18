from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ws_port', default_value='9090'),
        DeclareLaunchArgument('http_port', default_value='8080'),
        DeclareLaunchArgument('frontend_path', default_value=''),

        Node(
            package='g1_web_interface',
            executable='web_bridge_node',
            name='web_bridge',
            output='screen',
            parameters=[{
                'ws_port': 9090,
                'http_port': 8080,
                'frontend_path': LaunchConfiguration('frontend_path'),
                'map_downsample': 4,
                'map_publish_rate': 1.0,
                'pose_publish_rate': 10.0,
            }],
        ),
    ])
