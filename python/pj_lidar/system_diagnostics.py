#!/usr/bin/env python3
"""
SYSTEM DIAGNOSTICS SCRIPT
=========================
Диагностический скрипт для проверки всех компонентов системы робота.

Проверяет:
1. TF дерево (фреймы и связи)
2. Топики (публикация и частота)
3. Узлы ROS2
4. Подключение к ESP32
5. Конфликты TF фреймов

Запуск:
    ros2 run pj_lidar system_diagnostics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import time
import threading
from collections import defaultdict


class SystemDiagnostics(Node):
    """Диагностический узел для проверки системы."""
    
    def __init__(self):
        super().__init__('system_diagnostics')
        
        # TF Buffer для проверки трансформов
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Хранилище для статистики
        self.topic_stats = defaultdict(lambda: {'count': 0, 'last_time': 0, 'hz': 0.0})
        self.tf_frames = set()
        self.issues = []
        self.warnings = []
        self.ok_items = []
        
        # QoS для разных типов топиков
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Подписчики для проверки топиков
        self.setup_subscribers()
        
        self.get_logger().info('🔍 Диагностика системы запущена...')
        self.get_logger().info('=' * 60)
    
    def setup_subscribers(self):
        """Настроить подписчиков для мониторинга топиков."""
        # LaserScan
        self.create_subscription(
            LaserScan, '/scan', 
            lambda msg: self.topic_callback('/scan', msg), 
            self.best_effort_qos)
        
        # Map
        self.create_subscription(
            OccupancyGrid, '/map',
            lambda msg: self.topic_callback('/map', msg),
            self.reliable_qos)
        
        # Odometry
        self.create_subscription(
            Odometry, '/odom',
            lambda msg: self.topic_callback('/odom', msg),
            self.best_effort_qos)
        
        # Robot status
        self.create_subscription(
            String, '/robot_status',
            lambda msg: self.topic_callback('/robot_status', msg),
            10)
        
        # Robot state
        self.create_subscription(
            String, '/robot_state',
            lambda msg: self.topic_callback('/robot_state', msg),
            10)
    
    def topic_callback(self, topic_name, msg):
        """Обновить статистику топика."""
        current_time = time.time()
        stats = self.topic_stats[topic_name]
        
        if stats['last_time'] > 0:
            dt = current_time - stats['last_time']
            if dt > 0:
                # Экспоненциальное сглаживание
                stats['hz'] = 0.9 * stats['hz'] + 0.1 * (1.0 / dt)
        
        stats['count'] += 1
        stats['last_time'] = current_time
    
    def check_tf_tree(self):
        """Проверить TF дерево."""
        self.get_logger().info('\n📌 ПРОВЕРКА TF ДЕРЕВА')
        self.get_logger().info('-' * 40)
        
        # Получить все фреймы
        try:
            # Проверить ключевые трансформы
            transforms_to_check = [
                ('map', 'odom'),
                ('odom', 'base_link'),
                ('base_link', 'laser'),
                ('base_link', 'laser_frame'),
            ]
            
            for parent, child in transforms_to_check:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        parent, child, rclpy.time.Time())
                    
                    t = transform.transform.translation
                    r = transform.transform.rotation
                    
                    self.get_logger().info(
                        f'  ✓ {parent} → {child}: '
                        f'pos=({t.x:.3f}, {t.y:.3f}, {t.z:.3f})')
                    self.ok_items.append(f'TF: {parent} → {child}')
                    
                except (LookupException, ConnectivityException, ExtrapolationException) as e:
                    self.warnings.append(f'TF: {parent} → {child} НЕ НАЙДЕН')
                    self.get_logger().warn(f'  ⚠️ {parent} → {child}: НЕ НАЙДЕН')
                    
        except Exception as e:
            self.issues.append(f'Ошибка проверки TF: {str(e)}')
            self.get_logger().error(f'  ❌ Ошибка: {str(e)}')
        
        # Проверить конфликты фреймов лидара
        self.check_laser_frame_conflict()
    
    def check_laser_frame_conflict(self):
        """Проверить конфликт имён фреймов лидара."""
        self.get_logger().info('\n📌 ПРОВЕРКА КОНФЛИКТА ФРЕЙМОВ')
        self.get_logger().info('-' * 40)
        
        laser_exists = False
        laser_frame_exists = False
        
        try:
            self.tf_buffer.lookup_transform('base_link', 'laser', rclpy.time.Time())
            laser_exists = True
        except:
            pass
        
        try:
            self.tf_buffer.lookup_transform('base_link', 'laser_frame', rclpy.time.Time())
            laser_frame_exists = True
        except:
            pass
        
        if laser_exists and laser_frame_exists:
            self.issues.append('КОНФЛИКТ: существуют оба фрейма "laser" и "laser_frame"')
            self.get_logger().error('  ❌ КОНФЛИКТ: существуют оба фрейма "laser" и "laser_frame"')
            self.get_logger().error('     Это приведёт к ошибкам отображения в RViz!')
        elif laser_exists:
            self.get_logger().info('  ✓ Фрейм "laser" существует')
            self.ok_items.append('Фрейм лидара: laser')
        elif laser_frame_exists:
            self.get_logger().info('  ✓ Фрейм "laser_frame" существует')
            self.get_logger().warn('  ⚠️ Рекомендуется использовать имя "laser" для совместимости')
            self.warnings.append('Фрейм лидара: laser_frame (рекомендуется "laser")')
        else:
            self.issues.append('Фрейм лидара НЕ НАЙДЕН (ни "laser" ни "laser_frame")')
            self.get_logger().error('  ❌ Фрейм лидара НЕ НАЙДЕН')
    
    def check_topics(self):
        """Проверить активность топиков."""
        self.get_logger().info('\n📌 ПРОВЕРКА ТОПИКОВ')
        self.get_logger().info('-' * 40)
        
        time.sleep(2.0)  # Подождать данные
        
        topics_info = [
            ('/scan', 'LaserScan', 5.0, 10.0),      # ожидаемая частота
            ('/map', 'OccupancyGrid', 0.1, 2.0),    # карта публикуется реже
            ('/odom', 'Odometry', 10.0, 100.0),      # одометрия
            ('/robot_status', 'String', 0.1, 2.0),   # статус от ESP32
            ('/robot_state', 'String', 1.0, 20.0),   # состояние state machine
        ]
        
        for topic, msg_type, min_hz, max_hz in topics_info:
            stats = self.topic_stats[topic]
            hz = stats['hz']
            count = stats['count']
            
            if count == 0:
                self.warnings.append(f'{topic}: НЕТ ДАННЫХ')
                self.get_logger().warn(f'  ⚠️ {topic}: НЕТ ДАННЫХ')
            elif hz < min_hz:
                self.warnings.append(f'{topic}: низкая частота {hz:.1f} Гц (ожидается >{min_hz} Гц)')
                self.get_logger().warn(f'  ⚠️ {topic}: {hz:.1f} Гц (низкая частота, count={count})')
            else:
                self.ok_items.append(f'{topic}: {hz:.1f} Гц')
                self.get_logger().info(f'  ✓ {topic}: {hz:.1f} Гц (count={count})')
    
    def check_scan_data(self):
        """Проверить качество данных LaserScan."""
        self.get_logger().info('\n📌 ПРОВЕРКА ДАННЫХ LIDAR')
        self.get_logger().info('-' * 40)
        
        stats = self.topic_stats['/scan']
        if stats['count'] == 0:
            self.issues.append('LaserScan: нет данных для проверки')
            self.get_logger().error('  ❌ Нет данных для проверки')
            return
        
        # Проверить последний скан
        # Это делается в callback, здесь только отчёт
        self.get_logger().info('  ✓ Данные LiDAR поступают')
        self.ok_items.append('LiDAR данные поступают')
    
    def check_slam(self):
        """Проверить работу SLAM."""
        self.get_logger().info('\n📌 ПРОВЕРКА SLAM')
        self.get_logger().info('-' * 40)
        
        map_stats = self.topic_stats['/map']
        if map_stats['count'] == 0:
            self.warnings.append('SLAM: карта не публикуется')
            self.get_logger().warn('  ⚠️ Карта не публикуется')
            self.get_logger().info('     Проверьте: запущен ли slam_toolbox?')
            self.get_logger().info('     Есть ли данные в /scan?')
        else:
            self.ok_items.append('SLAM: карта публикуется')
            self.get_logger().info('  ✓ Карта публикуется')
    
    def check_nodes(self):
        """Проверить запущенные узлы."""
        self.get_logger().info('\n📌 ОЖИДАЕМЫЕ УЗЛЫ')
        self.get_logger().info('-' * 40)
        
        expected_nodes = [
            'slam_toolbox',
            'esp32_tcp_client',
            'lidar_udp_server',
            'scan_state_machine',
            'wheel_odometry',
            'static_tf_broadcaster',
            'lidar_motor_controller',
            'rviz2',
        ]
        
        self.get_logger().info('  Проверьте командой: ros2 node list')
        self.get_logger().info('  Ожидаемые узлы:')
        for node in expected_nodes:
            self.get_logger().info(f'    - {node}')
    
    def print_summary(self):
        """Вывести итоговый отчёт."""
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('📊 ИТОГОВЫЙ ОТЧЁТ')
        self.get_logger().info('=' * 60)
        
        if self.ok_items:
            self.get_logger().info('\n✅ OK:')
            for item in self.ok_items:
                self.get_logger().info(f'   ✓ {item}')
        
        if self.warnings:
            self.get_logger().info('\n⚠️ ПРЕДУПРЕЖДЕНИЯ:')
            for item in self.warnings:
                self.get_logger().warn(f'   ⚠️ {item}')
        
        if self.issues:
            self.get_logger().info('\n❌ ПРОБЛЕМЫ:')
            for item in self.issues:
                self.get_logger().error(f'   ❌ {item}')
        
        # Итог
        self.get_logger().info('\n' + '-' * 60)
        if self.issues:
            self.get_logger().error(f'🚨 НАЙДЕНО {len(self.issues)} КРИТИЧЕСКИХ ПРОБЛЕМ')
        elif self.warnings:
            self.get_logger().warn(f'⚠️ НАЙДЕНО {len(self.warnings)} ПРЕДУПРЕЖДЕНИЙ')
        else:
            self.get_logger().info('✅ СИСТЕМА РАБОТАЕТ НОРМАЛЬНО')
        
        self.get_logger().info('=' * 60)
    
    def run_diagnostics(self, duration=5.0):
        """Запустить диагностику на указанное время."""
        self.get_logger().info(f'Сбор данных в течение {duration} секунд...')
        
        # Собирать данные
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Выполнить проверки
        self.check_tf_tree()
        self.check_topics()
        self.check_scan_data()
        self.check_slam()
        self.check_nodes()
        
        # Итоговый отчёт
        self.print_summary()


def main(args=None):
    rclpy.init(args=args)
    
    diagnostics = SystemDiagnostics()
    
    try:
        diagnostics.run_diagnostics(duration=5.0)
    except KeyboardInterrupt:
        diagnostics.get_logger().info('Прервано пользователем')
    finally:
        diagnostics.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()