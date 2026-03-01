from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Запуск TCP клиента
        Node(
            package='pj_lidar',
            executable='esp32_tcp_client',
            name='tcp_client',
            parameters=[{
                'robot_ip': '192.168.4.2',
                'tcp_port': 3333,
            }]
        ),
        
        # Запуск State Machine
        Node(
            package='pj_lidar',
            executable='scan_state_machine',
            name='state_machine',
            parameters=[{
                'obstacle_distance': 0.2,
                'max_forward_speed': 0.5,
            }]
        ),
        
        # Запуск контроллера
        Node(
            package='pj_lidar',
            executable='robot_controller',
            name='controller',
        ),
    ])
