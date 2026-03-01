#!/usr/bin/env python3

# LIDAR UDP SERVER (НОВЫЙ ФОРМАТ)
# ЗАДАЧА:
# - Принимать UDP-пакеты LiDAR в новом формате и публиковать sensor_msgs/LaserScan.
# - Обрабатывать пакеты с заголовком 0xAA и контрольной суммой.
# ФОРМАТ ПАКЕТА:
# - Заголовок: 0xAA
# - Длина пакета (2 байта, little endian)
# - Версия протокола (1 байт)
# - Тип пакета (1 байт)
# - Заголовок данных: 0xAD
# - Длина данных (2 байта, little endian)
# - Скорость мотора (1 байт)
# - Сдвиг нуля (2 байта, little endian, со знаком)
# - Стартовый угол (2 байта, little endian)
# - Данные: [уровень сигнала (1 байт) + расстояние (2 байта)] * N
# - Контрольная сумма (2 байта, little endian)
#
# ВХОДЫ:
# - UDP порт (параметр udp_port)
# ВЫХОДЫ:
# - Топик: /scan (sensor_msgs/LaserScan)
# - Топик: /robot_status (std_msgs/String) — для служебных сообщений


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import socket
import threading
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from pj_lidar.udp_structs import LidarHeader, LidarMeasurement, get_checksum, calc_checksum

try:
    from pj_lidar.config_loader import ConfigLoader
    CONFIG_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    CONFIG_AVAILABLE = False

class LidarUdpServer(Node):
    """
    ROS2 узел для приема LiDAR данных через UDP в новом формате и публикации в LaserScan формате.
    """
    
    def __init__(self):
        super().__init__('lidar_udp_server')
        
        # Параметры - базовые значения берём из config.yaml (если есть),
        # а launch/ROS2 параметры могут их переопределить.
        udp_cfg = {}
        if CONFIG_AVAILABLE:
            try:
                ConfigLoader.load()
                udp_cfg = ConfigLoader.get_udp_config() or {}
                self.get_logger().info('✓ Параметры LiDAR UDP сервера загружены из config.yaml')
            except Exception as e:
                self.get_logger().warn(f'Ошибка загрузки конфига для lidar_udp_server: {e}')
                udp_cfg = {}

        self.declare_parameter('udp_port', udp_cfg.get('udp_port', 8888))  # UDP порт прослушивания
        self.declare_parameter('angle_min', -math.pi)  # минимальный угол
        self.declare_parameter('angle_max', math.pi)  # максимальный угол
        self.declare_parameter('range_min', udp_cfg.get('range_min', 0.15))  # минимальное расстояние (метры)
        self.declare_parameter('range_max', udp_cfg.get('range_max', 12.0))  # максимальное расстояние (метры)
        self.declare_parameter('frame_id', udp_cfg.get('frame_id', 'laser'))  # TF frame имя
        # Масштаб расстояния: во сколько метров превращается единица raw_distance.
        # Для Delta2A по умолчанию 1 unit = 0.25 мм = 0.00025 м.
        self.declare_parameter(
            'distance_scale',
            udp_cfg.get('distance_scale', 0.00025)
        )
        
        # Загрузить параметры
        self.udp_port = self.get_parameter('udp_port').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.frame_id = self.get_parameter('frame_id').value
        self.distance_scale = self.get_parameter('distance_scale').value
        
        # Издатели для LaserScan и статуса
        self.laser_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        
        # TF Broadcaster УДАЛЁН - TF должен публиковаться из одного источника
        # (wheel_odometry.py или static_tf_broadcaster.py)
        # self.tf_broadcaster = TransformBroadcaster(self)
        self.base_frame_id = 'base_link'  # для справки
        
        # Создать UDP сокет
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Разрешить принимать broadcast пакеты
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            self.socket.bind(('0.0.0.0', self.udp_port))
            self.get_logger().info(f'✓ UDP сервер слушает на порту {self.udp_port}')
        except Exception as e:
            self.get_logger().error(f'Ошибка привязки UDP сокета: {str(e)}')
        
        # ── Накопительный буфер для полного оборота (360 проб) ───
        self._accum_ranges      = [float('inf')] * 360
        self._accum_intensities = [0.0] * 360
        self._last_start_angle  = None   # стартовый угол предыдущего пакета
        self._packets_in_rev    = 0      # кол-во пакетов в текущем обороте

        # Флаг для корректного завершения потока при shutdown
        self._running = True

        # Запустить поток приема в фоне
        self.receive_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.receive_thread.start()

        self.get_logger().info('LiDAR UDP сервер готов к приему (аккумуляция по обороту)')
    
    def receive_loop(self):
        """
        Основной цикл приема UDP пакетов. Работает в отдельном потоке.
        """
        while rclpy.ok() and self._running:
            try:
                # Получить UDP пакет и адрес отправителя
                data, addr = self.socket.recvfrom(2048)
                
                # Лог о полученных данных
                hex_data = data.hex()[:60] if len(data) > 60 else data.hex()
                self.get_logger().info(f'📡 UDP: {len(data)} байт от {addr[0]} | Первые байты: {hex_data}')

                # Проверить тип данных - служебный статус, Telemetry или LiDAR?
                if data.startswith(b'ROBOT_STATUS'):
                    self.process_status(data)
                elif data and data[0] == 0xBB and len(data) >= 23:
                    self.process_telemetry_packet(data)
                else:
                    # Обработать как LiDAR данные в новом формате
                    self.process_lidar_data_new_format(data)

            except Exception as e:
                self.get_logger().error(f'Ошибка приема: {str(e)}')

    def process_telemetry_packet(self, data):
        """
        Обработать TelemetryPacket (0xBB) UDP пакет.
        Args:
            data (bytes): UDP пакет (23 байта)
        """
        from pj_lidar.udp_structs import TelemetryPacket
        try:
            pkt = TelemetryPacket.from_bytes(data[:23])
            ip_str = '.'.join(str(b) for b in pkt.ip)
            msg = f"[TELEMETRY] IP={ip_str} TCP={pkt.tcp_port} LEFT={pkt.left_hall} RIGHT={pkt.right_hall}"
            self.get_logger().info(msg)
            # Можно публиковать в отдельный топик, если нужно
            status_msg = String()
            status_msg.data = msg
            self.status_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Ошибка парсинга TelemetryPacket: {e}')
    
    def process_status(self, data):
        """
        Обработать служебное сообщение о статусе робота.
        
        Args:
            data (bytes): служебное сообщение от ESP32
        """
        try:
            status_str = data.decode('utf-8')
            self.get_logger().debug(f'Статус: {status_str}')
            
            # Опубликовать статус
            status_msg = String()
            status_msg.data = status_str
            self.status_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Ошибка обработки статуса: {str(e)}')
    
    def process_lidar_data_new_format(self, data):
        """
        Накапливаем секторные пакеты LiDAR (~22.5° каждый) и публикуем
        полный LaserScan (360°) один раз за оборот лидара.

        Оборот определяется по обратному переходу стартового угла:
        когда текущий start_angle < предыдущего более чем на 180°,
        значит началась новая революция — публикуем накопленные данные.
        """
        try:
            if len(data) < 15 or data[0] != 0xAA:
                return
            if len(data) > 4 and data[4] != 0x61:
                return

            # Стартовый угол текущего пакета (BIG-ENDIAN)
            start_angle_raw = int.from_bytes(data[11:13], 'big')
            start_angle_deg = start_angle_raw * 0.01

            # ── Определяем начало нового оборота ─────────────────
            if self._last_start_angle is not None:
                delta = start_angle_deg - self._last_start_angle
                # Угол убывает значительно → прошли 0°, начался новый оборот
                if delta < -180.0 and self._packets_in_rev >= 4:
                    self._publish_full_scan()
                    # Сброс буфера для нового оборота
                    self._accum_ranges      = [float('inf')] * 360
                    self._accum_intensities = [0.0] * 360
                    self._packets_in_rev    = 0

            self._last_start_angle = start_angle_deg

            # ── Заполнить накопительный буфер данными пакета ─────
            self.processLidarPacket(data,
                                    self._accum_ranges,
                                    self._accum_intensities)
            self._packets_in_rev += 1

        except Exception as e:
            self.get_logger().error(f'Ошибка обработки LiDAR данных: {str(e)}')

    def _publish_full_scan(self):
        """Опубликовать накопленный полный LaserScan (360°) и TF."""
        scan_msg = LaserScan()
        scan_msg.header.stamp        = self.get_clock().now().to_msg()
        scan_msg.header.frame_id     = self.frame_id
        scan_msg.angle_min           = self.angle_min          # -π
        scan_msg.angle_max           = self.angle_max          # +π
        scan_msg.angle_increment     = (self.angle_max - self.angle_min) / 360
        scan_msg.time_increment      = 0.0
        scan_msg.scan_time           = 1.0 / 6.0               # ~6 об/с
        scan_msg.range_min           = self.range_min
        scan_msg.range_max           = self.range_max
        scan_msg.ranges              = list(self._accum_ranges)
        scan_msg.intensities         = list(self._accum_intensities)

        self.laser_pub.publish(scan_msg)
        # TF публикуется wheel_odometry.py или static_tf_broadcaster.py

        valid = sum(1 for r in self._accum_ranges if math.isfinite(r))
        self.get_logger().info(
            f'✓ Полный скан опубликован: {valid}/360 валидных точек '
            f'({self._packets_in_rev} пакетов)')
    
    def processLidarPacket(self, packet, ranges, intensities):
        """
        Обработать полный LiDAR пакет.

        Формат (все многобайтовые значения — BIG-ENDIAN):
          байт  0    : 0xAA  — заголовок пакета
          байты 1-2  : длина пакета (big-endian)
          байт  3    : версия протокола
          байт  4    : 0x61  — тип пакета
          байт  5    : 0xAD  — заголовок данных
          байты 6-7  : длина данных (big-endian)
          байт  8    : скорость мотора * 0.05 об/с
          байты 9-10 : сдвиг нулевого градуса * 0.01° (знаковый, big-endian)
          байты 11-12: стартовый угол * 0.01° (big-endian)
          байты 13+  : [сигнал(1 байт) + расстояние(2 байта big-endian)] * N
          последние 2 байта: контрольная сумма (не проверяем)

        Расчёты:
          N = (длина_данных - 5) / 3
          угол_пробы_i = стартовый_угол + 22.5° * i / N   (i = 0..N-1)
          расстояние_м = raw_distance * 0.00025            (0.25 мм / unit)
        """
        length = len(packet)
        if length < 15:
            self.get_logger().debug('Пакет слишком короткий')
            return

        # Проверка заголовка
        if packet[0] != 0xAA:
            self.get_logger().debug('Неверный заголовок 0xAA')
            return

        # Проверка типа пакета (0x61)
        if length > 4 and packet[4] != 0x61:
            self.get_logger().debug(f'Неверный тип пакета: 0x{packet[4]:02X} (ожидается 0x61)')
            return

        # ── Длина данных — BIG-ENDIAN ─────────────────────────────
        data_length = int.from_bytes(packet[6:8], 'big')

        # ── Стартовый угол — BIG-ENDIAN ───────────────────────────
        start_angle_raw = int.from_bytes(packet[11:13], 'big')
        start_angle_deg = start_angle_raw * 0.01      # в градусах (0.0 – 360.0)

        # ── Количество проб ───────────────────────────────────────
        num_samples = (data_length - 5) // 3
        if num_samples <= 0:
            self.get_logger().debug('num_samples <= 0, пропускаем пакет')
            return

        # Угловой шаг между пробами в этом пакете
        angle_step_deg = 22.5 / num_samples           # °/проба

        self.get_logger().debug(
            f'Пакет: data_len={data_length} N={num_samples} '
            f'start={start_angle_deg:.2f}°  step={angle_step_deg:.4f}°')

        # ── Данные проб (начиная с байта 13) ──────────────────────
        data_start = 13
        for i in range(num_samples):
            offset = data_start + i * 3
            if offset + 3 > length - 2:   # не читать контрольную сумму
                break

            signal       = packet[offset]
            # Расстояние — BIG-ENDIAN
            distance_raw = int.from_bytes(packet[offset + 1: offset + 3], 'big')

            # Расстояние в метрах: distance_raw * distance_scale.
            # По умолчанию distance_scale = 0.00025 (0.25 мм / unit) —
            # при необходимости можно переопределить параметром ROS2
            # или через config.yaml (lidar_udp_server.distance_scale).
            distance_m = distance_raw * float(self.distance_scale)

            # Угол пробы (формула: стартовый_угол + 22.5° * i / N)
            angle_deg   = start_angle_deg + angle_step_deg * i
            # Сдвиг +180°: физический 0° (фронт робота) → индекс 180
            # (front_clear() проверяет индексы 135–225, центр = 180)
            angle_index = int(round(angle_deg + 180.0)) % 360

            if signal > 0 and self.range_min <= distance_m <= self.range_max:
                ranges[angle_index]      = distance_m
                intensities[angle_index] = float(signal)

        self.get_logger().debug(
            f'✓ Обработано {num_samples} точек, start={start_angle_deg:.2f}°')
    
    def destroy_node(self):
        """Корректное завершение - остановить поток и закрыть UDP сокет."""
        self._running = False
        if self.socket:
            try:
                self.socket.close()
            except Exception:
                pass
        if hasattr(self, 'receive_thread') and self.receive_thread.is_alive():
            try:
                self.receive_thread.join(timeout=1.0)
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    """Точка входа - инициализация ROS2 и запуск узла."""
    rclpy.init(args=args)
    
    server = LidarUdpServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Остановка сервера...')
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()