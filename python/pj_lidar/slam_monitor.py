import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
import math
import threading
import subprocess
import os
from datetime import datetime
from nav_msgs.msg import Odometry

class SlamMonitor(Node):
    def __init__(self):
        super().__init__('slam_monitor')
        
        # Параметры
        self.declare_parameter('min_rotations', 3)
        self.declare_parameter('avg_distance_min', 0.5)
        self.declare_parameter('avg_distance_max', 10.0)
        
        self.min_rotations = self.get_parameter('min_rotations').value
        self.avg_distance_min = self.get_parameter('avg_distance_min').value
        self.avg_distance_max = self.get_parameter('avg_distance_max').value
        
        # Подписчики
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        
        self.lidar_rpm_sub = self.create_subscription(
            String, 'lidar_rpm', self.rpm_callback, 10)
        
        # Одометрия для отслеживания позиции
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Издатели
        self.scan_complete_pub = self.create_publisher(
            Bool, 'scan_complete', 10)
        
        self.slam_status_pub = self.create_publisher(
            String, 'slam_status', 10)
        
        # Переменные отслеживания
        self.scan_count = 0
        self.last_angle = 0.0
        self.rotation_count = 0
        self.scan_history = []
        self.map_size = 0
        self.current_rpm = 0.0
        self.scan_complete = False
        self.max_consecutive_stable = 0
        
        # Добавить отслеживание посещённых зон:
        self.visited_zones = []  # Сетка 1м x 1м
        
        # Таймер для проверки завершения
        self.check_timer = self.create_timer(1.0, self.check_scan_complete)
        
        self.get_logger().info('SLAM Monitor инициализирован')
    
    def scan_callback(self, msg):
        """Обработка сканирований LiDAR"""
        self.scan_count += 1
        
        # Подсчет полных оборотов
        ranges = msg.ranges
        if ranges:
            current_angle = len(ranges)
            if current_angle < len(msg.ranges) * 0.1:  # Новый оборот
                self.rotation_count += 1
        
        # Сохранение статистики сканирования
        avg_distance = self.calculate_avg_distance(ranges)
        valid_points = sum(1 for r in ranges if not math.isinf(r) and r > 0)
        
        scan_data = {
            'rotation': self.rotation_count,
            'avg_distance': avg_distance,
            'valid_points': valid_points,
            'timestamp': self.get_clock().now().to_msg()
        }
        
        self.scan_history.append(scan_data)
        
        # Удалить старые данные (более 60 секунд)
        if len(self.scan_history) > 100:
            self.scan_history.pop(0)
        
        # Публикация статуса
        status = f"Rotations: {self.rotation_count}, Avg: {avg_distance:.2f}m, Points: {valid_points}"
        self.publish_status(status)
    
    def map_callback(self, msg):
        """Получение размера карты от SLAM"""
        self.map_size = len(msg.data)
        
        # Подсчет занятых ячеек
        occupied = sum(1 for cell in msg.data if cell > 50)
        self.get_logger().debug(f"Map size: {self.map_size}, Occupied: {occupied}")
    
    def rpm_callback(self, msg):
        """Получение RPM лидара"""
        try:
            self.current_rpm = float(msg.data)
        except:
            pass
    
    def odom_callback(self, msg):
        """Получить одометрию для отслеживания позиции."""
        self.odometry_x = msg.pose.pose.position.x
        self.odometry_y = msg.pose.pose.position.y
    
    def calculate_avg_distance(self, ranges):
        """Расчет среднего расстояния"""
        valid_ranges = [r for r in ranges if not math.isinf(r) and r > 0]
        if valid_ranges:
            return sum(valid_ranges) / len(valid_ranges)
        return 0.0
    
    def check_scan_complete(self):
        """Проверка завершения сканирования комнаты"""
        
        if self.rotation_count < self.min_rotations:
            return
        
        if len(self.scan_history) < 10:
            return
        
        # Анализ последних 10 сканирований
        recent_scans = self.scan_history[-10:]
        
        # Проверка стабильности расстояния
        distances = [s['avg_distance'] for s in recent_scans]
        avg_dist = sum(distances) / len(distances)
        distance_variance = sum((d - avg_dist) ** 2 for d in distances) / len(distances)
        
        # Проверка количества точек
        points = [s['valid_points'] for s in recent_scans]
        avg_points = sum(points) / len(points)
        
        self.get_logger().info(
            f"Анализ SLAM:\n"
            f"  Оборотов: {self.rotation_count}\n"
            f"  Avg расстояние: {avg_dist:.2f}м\n"
            f"  Вариация: {math.sqrt(distance_variance):.2f}м\n"
            f"  Avg точек: {avg_points:.0f}\n"
            f"  RPM: {self.current_rpm:.1f}"
        )
        
        # Условия завершения сканирования
        conditions_met = 0
        
        # 1. Достаточно оборотов
        if self.rotation_count >= self.min_rotations:
            conditions_met += 1
        
        # 2. Расстояние в диапазоне стен
        if self.avg_distance_min <= avg_dist <= self.avg_distance_max:
            conditions_met += 1
        
        # 3. Стабильное расстояние (низкая вариация)
        if math.sqrt(distance_variance) < 0.5:
            conditions_met += 1
        
        # 4. Много точек в каждом сканировании
        if avg_points > 300:
            conditions_met += 1
        
        # 5. Карта имеет данные
        if self.map_size > 1000:
            conditions_met += 1
        
        # Если 4+ условия выполнены - сканирование завершено
        if conditions_met >= 4 and not self.scan_complete:
            self.scan_complete = True
            self.publish_scan_complete()
    
    def publish_scan_complete(self):
        """Публикация сигнала завершения сканирования"""
        msg = Bool()
        msg.data = True
        self.scan_complete_pub.publish(msg)
        
        self.get_logger().warn(
            "✓✓✓ СКАНИРОВАНИЕ КОМНАТЫ ЗАВЕРШЕНО! ✓✓✓"
        )
        
        # Лог и подробная информация
        info = String()
        info.data = (
            f"Scan Complete!\n"
            f"Rotations: {self.rotation_count}\n"
            f"Avg Distance: {self.scan_history[-1]['avg_distance']:.2f}m\n"
            f"Total Scans: {self.scan_count}\n"
            f"Map Size: {self.map_size}"
        )
        self.slam_status_pub.publish(info)

        # Записать в локальный лог файл
        try:
            log_dir = os.path.expanduser("~")
            fname = os.path.join(log_dir, f"scan_complete_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
            with open(fname, "w") as f:
                f.write(f"{datetime.now().isoformat()} - SCAN_COMPLETE\n")
                f.write(info.data + "\n")
            self.get_logger().info(f"Scan completion logged to {fname}")
        except Exception as e:
            self.get_logger().error(f"Ошибка записи лога: {e}")

        # Попытаться сохранить карту с помощью map_saver_cli в фоне (если установлен)
        def save_map_background():
            try:
                maps_dir = os.path.expanduser("~/maps")
                os.makedirs(maps_dir, exist_ok=True)
                map_fname = os.path.join(maps_dir, f"map_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
                cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_fname]
                self.get_logger().info(f"Запуск сохранения карты: {' '.join(cmd)}")
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                out, err = proc.communicate(timeout=60)
                if proc.returncode == 0:
                    self.get_logger().info(f"Карта сохранена: {map_fname}.yaml/png")
                else:
                    self.get_logger().error(f"Ошибка сохранения карты: {err.decode('utf-8')}")
            except Exception as e:
                self.get_logger().error(f"Failed to save map: {e}")

        # Запустить в фоне
        threading.Thread(target=save_map_background, daemon=True).start()
    
    def publish_status(self, status):
        """Публикация статуса"""
        msg = String()
        msg.data = status
        self.slam_status_pub.publish(msg)

    def check_coverage(self):
        """Проверить покрытие комнаты на основе сетки 1м x 1м."""
        # Проверить наличие одометрии
        if not hasattr(self, 'odometry_x') or not hasattr(self, 'odometry_y'):
            return False
        
        zone = (int(self.odometry_x), int(self.odometry_y))
        
        if zone not in self.visited_zones:
            self.visited_zones.append(zone)
        
        # Оценочное количество зон в помещении 5м × 4м = 20 зон
        estimated_room_zones = 20
        coverage_percent = (len(self.visited_zones) / estimated_room_zones) * 100
        
        self.get_logger().info(f"📊 Покрытие: {len(self.visited_zones)}/{estimated_room_zones} зон ({coverage_percent:.1f}%)")
        
        # Возвращает True если покрытие >= 80%
        return coverage_percent >= 80.0

def main(args=None):
    rclpy.init(args=args)
    monitor = SlamMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Остановка...')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
