# Main launch file for SLAM and robot control
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

PYTHON_PATH = "/home/abodik345/Desktop/VSC (2)/install/pj_lidar/share/pj_lidar/python"

def generate_launch_description():
    pkg_share = FindPackageShare('pj_lidar')

    slam_config = PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox_config.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'rviz_config.rviz'])

    env = os.environ.copy()
    env['PYTHONPATH'] = PYTHON_PATH + ':' + env.get('PYTHONPATH', '')

    # Базовые аргументы для подавления INFO-логов (пока убираем, чтобы видеть ошибки)
    # log_level_args = ['--ros-args', '--log-level', 'WARN']
    log_level_args = []  # временно без фильтра

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='auto'),
        DeclareLaunchArgument('tcp_port', default_value='3333'),

        # TCP клиент
        ExecuteProcess(
            cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/lib/pj_lidar/esp32_tcp_client.py'] + log_level_args +
                ['--ros-args', '-p', 'robot_ip:=auto', '-p', 'tcp_port:=3333', '-p', 'udp_port:=4444'],
            name='esp32_tcp_client',
            output='screen',
            additional_env=env
        ),
        # LiDAR UDP сервер
        ExecuteProcess(
            cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/lib/pj_lidar/lidar_udp_server_new.py'] + log_level_args +
                ['--ros-args', '-p', 'udp_port:=4444'],
            name='lidar_udp_server',
            output='screen',
            additional_env=env
        ),
        # Frontier explorer (исследование границ) – исправленный путь
        ExecuteProcess(
            cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/share/pj_lidar/python/pj_lidar/frontier_explorer.py'] + log_level_args +
                ['--ros-args'],
            name='frontier_explorer',
            output='screen',
            additional_env=env
        ),
        # Robot monitor
        ExecuteProcess(
            cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/lib/pj_lidar/robot_monitor.py'] + log_level_args +
                ['--ros-args'],
            name='robot_monitor',
            output='screen',
            additional_env=env
        ),
        # LiDAR Motor Controller
        ExecuteProcess(
            cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/lib/pj_lidar/lidar_motor_controller.py'] + log_level_args +
                ['--ros-args'],
            name='lidar_motor_controller',
            output='screen',
            additional_env=env
        ),
        # SLAM Toolbox (C++ node)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=['/home/abodik345/Desktop/VSC (2)/install/pj_lidar/share/pj_lidar/config/slam_toolbox_config.yaml'],
            output='screen',
            arguments=log_level_args
        ),

        # Активация SLAM Toolbox через lifecycle (configure → activate)
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                    output='screen'
                ),
                TimerAction(
                    period=3.0,
                    actions=[
                        ExecuteProcess(
                            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                            output='screen'
                        )
                    ]
                )
            ]
        ),

        # SLAM Monitor
        ExecuteProcess(
            cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/lib/pj_lidar/slam_monitor.py'] + log_level_args +
                ['--ros-args'],
            name='slam_monitor',
            output='screen',
            additional_env=env
        ),
        # Static TF Broadcaster отключён – трансформации публикует wheel_odometry
        # ExecuteProcess(
        #     cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/lib/pj_lidar/static_tf_broadcaster.py'] + log_level_args +
        #         ['--ros-args'],
        #     name='static_tf_broadcaster',
        #     output='screen',
        #     emulate_tty=True,
        #     additional_env=env
        # ),

        # Wheel odometry
        ExecuteProcess(
            cmd=['python3', '/home/abodik345/Desktop/VSC (2)/install/pj_lidar/share/pj_lidar/python/pj_lidar/wheel_odometry.py'] + log_level_args +
                ['--ros-args',
                 '-p', 'wheel_diameter:=0.051',
                 '-p', 'wheel_base:=0.135',
                 '-p', 'ticks_per_rev:=4',
                 '-p', 'left_ticks_topic:=/left_ticks',
                 '-p', 'right_ticks_topic:=/right_ticks',
                 '-p', 'laser_frame:=laser',
                 '-p', 'laser_z:=0.06',
                 '-p', 'publish_rate:=50.0'],
            name='wheel_odometry',
            output='screen',
            emulate_tty=True,
            additional_env=env
        ),

        # Запуск RViz2 с задержкой (после активации SLAM)
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen'
                )
            ]
        ),

        # ========== ДИАГНОСТИКА (выводится через 3 секунды) ==========
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="\n=== DIAGNOSTICS: Checking ROS2 nodes and topics ==="),
                ExecuteProcess(
                    cmd=['bash', '-c', 'echo "--- ROS2 Nodes ---" && ros2 node list'],
                    name='ros2_node_list',
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['bash', '-c', 'echo "--- ROS2 Topics ---" && ros2 topic list'],
                    name='ros2_topic_list',
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['bash', '-c', 'journalctl --since "10 seconds ago" | grep -i "error\\|fatal\\|exception" || echo "No errors found in recent logs"'],
                    name='error_check',
                    output='screen'
                ),
                LogInfo(msg="=== Diagnostics completed ===")
            ]
        ),
    ])