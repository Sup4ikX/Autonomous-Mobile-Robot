import os
from setuptools import setup, find_packages

package_name = 'pj_lidar'

# Директория пакета
pkg_dir = os.path.dirname(os.path.abspath(__file__))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_all.py']),
        ('share/' + package_name + '/config', [
            os.path.join(pkg_dir, 'python/pj_lidar/config/slam_toolbox_config.yaml'),
            os.path.join(pkg_dir, 'python/pj_lidar/config/rviz_config.rviz'),
            os.path.join(pkg_dir, 'python/pj_lidar/config/nav2_params.yaml'),
            os.path.join(pkg_dir, 'python/pj_lidar/config/navigate_w_replanning_and_recovery.xml')
        ]),
        ('share/' + package_name + '/doc', [
            os.path.join(pkg_dir, 'docs/README.md'),
            os.path.join(pkg_dir, 'docs/QUICKSTART.md'),
            os.path.join(pkg_dir, 'docs/INSTALL.md'),
            os.path.join(pkg_dir, 'docs/CHEATSHEET.md'),
            os.path.join(pkg_dir, 'docs/CHANGELOG.md')
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sup4ikX',
    maintainer_email='sup4ikX@example.com',
    description='ESP32 LiDAR Robot Mapper - Full-featured autonomous room mapping system',
    license='AGPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_tcp_client = pj_lidar.esp32_tcp_client:main',
            'lidar_udp_server = pj_lidar.lidar_udp_server_new:main',
            'scan_state_machine = pj_lidar.scan_state_machine:main',
            'robot_monitor = pj_lidar.robot_monitor:main',
            'robot_controller = pj_lidar.robot_controller:main',
            'start_scan = pj_lidar.start_scan:main',
            'setup_config = pj_lidar.setup_config:main',
            'static_tf_broadcaster = pj_lidar.static_tf_broadcaster:main',
            'slam_monitor = pj_lidar.slam_monitor:main',
            'wheel_odometry = pj_lidar.wheel_odometry:main',
            'system_diagnostics = pj_lidar.system_diagnostics:main',
            'lidar_motor_controller = pj_lidar.lidar_motor_controller:main',
        ],
    },
)