# Main launch file for SLAM and robot control
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from launch.actions import TimerAction, ExecuteProcess

PYTHON_PATH = "/home/abodik345/Desktop/VSC (2)/install/pj_lidar/share/pj_lidar/python"
def generate_launch_description():
    pkg_share = FindPackageShare('pj_lidar')

    slam_config = PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox_config.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'rviz_config.rviz'])

    env = os.environ.copy()
    env['PYTHONPATH'] = PYTHON_PATH + ':' + env.get('PYTHONPATH', '')

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='auto'),
        DeclareLaunchArgument('tcp_port', default_value='3333'),

# TCP клиент
        ExecuteProcess(
            cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/lib/pj_lidar/esp32_tcp_client.py', '--ros-args',
                 '-p', 'robot_ip:=auto', '-p', 'tcp_port:=3333'],
            name='esp32_tcp_client',
            output='screen',
            additional_env=env
        ),
# LiDAR UDP сервер
        ExecuteProcess(
            cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/lib/pj_lidar/lidar_udp_server_new.py', '--ros-args', '-p', 'udp_port:=4444'],
            name='lidar_udp_server',
            output='screen',
            additional_env=env
        ),
# State machine
        ExecuteProcess(
            cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/lib/pj_lidar/scan_state_machine.py'],
            name='scan_state_machine',
            output='screen',
            additional_env=env
        ),
# Robot monitor
        ExecuteProcess(
            cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/lib/pj_lidar/robot_monitor.py'],
            name='robot_monitor',
            output='screen',
            additional_env=env
        ),
# LiDAR Motor Controller
        ExecuteProcess(
            cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/lib/pj_lidar/lidar_motor_controller.py'],
            name='lidar_motor_controller',
            output='screen',
            additional_env=env
        ),
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=['/home/abodik345/Desktop/VSC (2)/install/pj_lidar/share/pj_lidar/config/slam_toolbox_config.yaml'],
            output='screen'
        ),

            # Конфигурация и активация SLAM Toolbox
    TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                output='screen'
            )
        ]
    ),
    TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                output='screen'
            )
        ]
    ),
    
# SLAM Monitor
        ExecuteProcess(
            cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/lib/pj_lidar/slam_monitor.py'],
            name='slam_monitor',
            output='screen',
            additional_env=env
        ),
# Static TF Broadcaster - ВКЛЮЧЕН для корректной работы SLAM
        ExecuteProcess(
            cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/lib/pj_lidar/static_tf_broadcaster.py'],
            name='static_tf_broadcaster',
            output='both',
            emulate_tty=True,
            additional_env=env
        ),
# Wheel odometry (encoder -> odom and static laser_frame)
        ExecuteProcess(
            cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/lib/pj_lidar/wheel_odometry.py', '--ros-args',
                 '-p', 'wheel_diameter:=0.051',
                 '-p', 'wheel_base:=0.135',
                 '-p', 'ticks_per_rev:=4',
                 '-p', 'left_ticks_topic:=/left_ticks',
                 '-p', 'right_ticks_topic:=/right_ticks',
                 '-p', 'laser_frame:=laser',
                 '-p', 'laser_z:=0.06',
                 '-p', 'publish_rate:=50.0'],
            name='wheel_odometry',
            output='both',
            emulate_tty=True,
            additional_env=env
        ),
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])