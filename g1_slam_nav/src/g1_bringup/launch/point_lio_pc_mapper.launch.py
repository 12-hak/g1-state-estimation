from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare('g1_bringup'),
        'config',
        'point_lio_mid360.yaml',
    ])

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz with Point-LIO config.',
    )
    use_web_arg = DeclareLaunchArgument(
        'use_web',
        default_value='true',
        description='Launch web bridge/frontend.',
    )
    use_occ_grid_arg = DeclareLaunchArgument(
        'use_occ_grid',
        default_value='true',
        description='Run occupancy grid publisher from Point-LIO map.',
    )
    ws_port_arg = DeclareLaunchArgument(
        'ws_port',
        default_value='9090',
        description='WebSocket port for web bridge.',
    )
    http_port_arg = DeclareLaunchArgument(
        'http_port',
        default_value='8080',
        description='HTTP port for web bridge.',
    )
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/livox/lidar',
        description='Input LiDAR topic from Jetson.',
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/livox/imu',
        description='Input IMU topic from Jetson.',
    )

    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
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
        ],
        remappings=[
            ('/livox/lidar', LaunchConfiguration('lidar_topic')),
            ('/livox/imu', LaunchConfiguration('imu_topic')),
        ],
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

    web_bridge_node = Node(
        package='g1_web_interface',
        executable='web_bridge_node',
        name='web_bridge',
        output='screen',
        parameters=[{
            'ws_port': LaunchConfiguration('ws_port'),
            'http_port': LaunchConfiguration('http_port'),
            'map_downsample': 4,
            'map_publish_rate': 1.0,
            'pose_publish_rate': 10.0,
            'use_point_lio': True,
        }],
        condition=IfCondition(LaunchConfiguration('use_web')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('g1_bringup'),
            'rviz',
            'point_lio.rviz',
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        use_web_arg,
        use_occ_grid_arg,
        ws_port_arg,
        http_port_arg,
        lidar_topic_arg,
        imu_topic_arg,
        laser_mapping_node,
        point_lio_occ_grid_node,
        web_bridge_node,
        rviz_node,
    ])
