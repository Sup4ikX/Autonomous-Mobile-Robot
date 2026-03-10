import os
from setuptools import setup, find_packages

package_name = 'pj_lidar'

# Корневая директория пакета (где лежит setup.py)
pkg_dir = os.path.dirname(os.path.abspath(__file__))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages('python'),           # ищем пакеты в папке python
    package_dir={'': 'python'},                 # корень пакетов – папка python
        data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', ['launch/launch_all.py']),

        # Конфигурационные файлы (только существующие)
        ('share/' + package_name + '/config', [
            'python/pj_lidar/config/slam_toolbox_config.yaml',
            'python/pj_lidar/config/rviz_config.rviz',
        ]),

        # Документация (если нужна, оставляем)
        ('share/' + package_name + '/doc', [
            'docs/README.md',
            'docs/QUICKSTART.md',
            'docs/INSTALL.md',
            'docs/CHEATSHEET.md',
            'docs/CHANGELOG.md'
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
        'frontier_explorer = pj_lidar.frontier_explorer:main',
        'robot_monitor = pj_lidar.robot_monitor:main',
        'start_scan = pj_lidar.start_scan:main',
        'setup_config = pj_lidar.setup_config:main',
        'slam_monitor = pj_lidar.slam_monitor:main',
        'wheel_odometry = pj_lidar.wheel_odometry:main',
        'system_diagnostics = pj_lidar.system_diagnostics:main',
        'lidar_motor_controller = pj_lidar.lidar_motor_controller:main',
        ],
    },
)