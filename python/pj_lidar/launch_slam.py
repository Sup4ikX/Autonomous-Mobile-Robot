
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # объявить аргументы запуска (пользователь может переопределить)
    # THESE LAUNCH ARGS SHOULD BE SET BY THE USER (via setup_config.py -> config.yaml or CLI)
    declare_robot_ip = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.4.2',
        description='ESP32 robot IP address (set by user via config or CLI)'
    )
    declare_tcp_port = DeclareLaunchArgument(
        'tcp_port',
        default_value='3333',
        description='TCP port for lidar_processor and ESP32 (set by user via config or CLI)'
    )

    # Пути к конфигам
    slam_config_dir = get_package_share_directory('pj_lidar')
    slam_params = os.path.join(slam_config_dir, 'slam_toolbox_config.yaml')

    robot_ip_cfg = LaunchConfiguration('robot_ip')
    tcp_port_cfg = LaunchConfiguration('tcp_port')

    return LaunchDescription([
        declare_robot_ip,
        declare_tcp_port,

        # ===== LiDAR обработчик (C++) =====
        Node(
            package='pj_lidar',
            executable='lidar_processor',
            name='lidar_data_handler',
            output='screen',
            parameters=[{
                'tcp_port': tcp_port_cfg,
                'serial_port': '/dev/ttyUSB0',
                'target_rpm': 300.0,
                'frame_id': 'laser',
                'base_frame_id': 'base_link',
            }]
        ),

        # ===== SLAM Toolbox (асинхронный режим) =====
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        ),

        # ===== SLAM Monitor (Python) =====
        Node(
            package='pj_lidar',
            executable='slam_monitor',
            name='slam_monitor',
            output='screen',
            parameters=[{
                'min_rotations': 3,
                'avg_distance_min': 0.5,
                'avg_distance_max': 10.0,
            }]
        ),

        # ===== TCP клиент управления (Python) =====
        Node(
            package='pj_lidar',
            executable='esp32_tcp_client',
            name='esp32_tcp_client',
            output='screen',
            parameters=[{
                'robot_ip': robot_ip_cfg,
                'tcp_port': tcp_port_cfg,
            }]
        ),

        # ===== State Machine (Python) =====
        Node(
            package='pj_lidar',
            executable='scan_state_machine',
            name='scan_state_machine',
            output='screen',
            parameters=[{
                'obstacle_distance': 0.2,
                'max_forward_speed': 0.3,
            }]
        ),

        # ===== Static TF Broadcaster (для построения карты на месте) =====
        Node(
            package='pj_lidar',
            executable='static_tf_broadcaster',
            name='static_tf_broadcaster',
            output='screen',
            parameters=[{
                'laser_height': 0.1,
                'publish_rate': 10.0,
            }]
        ),

        # ===== RViz2 для визуализации =====
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(slam_config_dir, 'rviz_config.rviz')],
            output='screen'
        ),
    ])
