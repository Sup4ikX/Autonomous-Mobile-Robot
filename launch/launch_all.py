# Main launch file for SLAM and robot control
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('pj_lidar')
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'rviz_config.rviz'])

    # Прямой путь к исходным файлам (временное решение до настройки entry points)
    SRC_PYTHON_PATH = "/home/abodik345/Desktop/VSC (2)/python/pj_lidar"

    return LaunchDescription([
        # TCP клиент
        ExecuteProcess(
            cmd=['python3', f'{SRC_PYTHON_PATH}/esp32_tcp_client.py',
                 '--ros-args',
                 '-p', 'robot_ip:=auto',
                 '-p', 'tcp_port:=3333',
                 '-p', 'udp_port:=4444'],
            name='esp32_tcp_client',
            output='screen'
        ),

        # LiDAR UDP сервер
        ExecuteProcess(
            cmd=['python3', f'{SRC_PYTHON_PATH}/lidar_udp_server_new.py',
                 '--ros-args',
                 '-p', 'udp_port:=4444'],
            name='lidar_udp_server',
            output='screen'
        ),

        # Frontier explorer
        ExecuteProcess(
            cmd=['python3', f'{SRC_PYTHON_PATH}/frontier_explorer.py',
                 '--ros-args'],
            name='frontier_explorer',
            output='screen'
        ),

        # Robot monitor
        ExecuteProcess(
            cmd=['python3', f'{SRC_PYTHON_PATH}/robot_monitor.py',
                 '--ros-args'],
            name='robot_monitor',
            output='screen'
        ),

        # LiDAR Motor Controller
        ExecuteProcess(
            cmd=['python3', f'{SRC_PYTHON_PATH}/lidar_motor_controller.py',
                 '--ros-args'],
            name='lidar_motor_controller',
            output='screen'
        ),

        # SLAM Toolbox (C++ node)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox_config.yaml'])],
            output='screen'
        ),

        # Активация SLAM Toolbox через lifecycle
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
            cmd=['python3', f'{SRC_PYTHON_PATH}/slam_monitor.py',
                 '--ros-args'],
            name='slam_monitor',
            output='screen'
        ),

        # Wheel odometry
        ExecuteProcess(
            cmd=['python3', f'{SRC_PYTHON_PATH}/wheel_odometry.py',
                 '--ros-args',
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
            emulate_tty=True
        ),

        # Запуск RViz2 с задержкой
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

        # Диагностика
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