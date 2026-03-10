#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import socket
import threading
import math

try:
    from pj_lidar.config_loader import ConfigLoader
    from pj_lidar.udp_structs import TelemetryPacket
    CONFIG_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    CONFIG_AVAILABLE = False

class LidarUdpServer(Node):
    def __init__(self):
        super().__init__('lidar_udp_server')
        
        udp_cfg = {}
        if CONFIG_AVAILABLE:
            try:
                ConfigLoader.load()
                udp_cfg = ConfigLoader.get_udp_config() or {}
                self.get_logger().info('✓ Параметры LiDAR UDP сервера загружены из config.yaml')
            except Exception as e:
                self.get_logger().warn(f'Ошибка загрузки конфига: {e}')

        self.declare_parameter('udp_port', udp_cfg.get('udp_port', 4444))
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('range_min', udp_cfg.get('range_min', 0.15))
        self.declare_parameter('range_max', udp_cfg.get('range_max', 12.0))
        self.declare_parameter('frame_id', udp_cfg.get('frame_id', 'laser'))
        self.declare_parameter('distance_scale', udp_cfg.get('distance_scale', 0.00025))

        self.udp_port = self.get_parameter('udp_port').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.frame_id = self.get_parameter('frame_id').value
        self.distance_scale = self.get_parameter('distance_scale').value

        self.laser_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        from std_msgs.msg import Int64
        self.left_ticks_pub = self.create_publisher(Int64, 'left_ticks', 10)
        self.right_ticks_pub = self.create_publisher(Int64, 'right_ticks', 10)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            self.socket.bind(('0.0.0.0', self.udp_port))
            self.get_logger().info(f'✓ UDP сервер слушает на порту {self.udp_port}')
        except Exception as e:
            self.get_logger().error(f'Ошибка привязки UDP сокета: {str(e)}')

        self._accum_ranges = [float('inf')] * 360
        self._accum_intensities = [0.0] * 360
        self._last_start_angle = None
        self._packets_in_rev = 0
        self._running = True
        self._packet_counter = 0  # DEBUG: счётчик пакетов

        self.receive_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.receive_thread.start()

        self.get_logger().info('LiDAR UDP сервер готов к приему (аккумуляция по обороту)')

    def receive_loop(self):
        while rclpy.ok() and self._running:
            try:
                data, addr = self.socket.recvfrom(2048)
                self._packet_counter += 1
                # Детальный лог каждого пакета
                #self.get_logger().info(f'[PKT {self._packet_counter}] 📡 UDP: {len(data)} байт от {addr[0]}, первые 30 байт: {data.hex()[:60]}')
                
                if data.startswith(b'ROBOT_STATUS'):
                    self.get_logger().debug('Обработка ROBOT_STATUS')
                    self.process_status(data)
                elif data and data[0] == 0xBB:
                    self.get_logger().debug('Обработка телеметрии (0xBB)')
                    self.process_telemetry_packet(data)
                elif data and data[0] == 0xAA:
                    self.get_logger().debug('Обработка LiDAR пакета (0xAA)')
                    self.process_lidar_packet(data)
                else:
                    self.get_logger().warn(f'Неизвестный маркер пакета: 0x{data[0]:02X}')
            except Exception as e:
                self.get_logger().error(f'Ошибка приема: {str(e)}')

    def process_telemetry_packet(self, data):
        try:
            pkt = TelemetryPacket.from_bytes(data[:15])
            ip_str = '.'.join(str(b) for b in pkt.ip)
            msg = f"[TELEMETRY] IP={ip_str} TCP={pkt.tcp_port} LEFT={pkt.left_hall} RIGHT={pkt.right_hall}"
            self.get_logger().debug(msg)
            status_msg = String()
            status_msg.data = msg
            self.status_pub.publish(status_msg)
            # Публикация энкодеров
            from std_msgs.msg import Int64
            left_msg = Int64()
            right_msg = Int64()
            left_msg.data = pkt.left_hall
            right_msg.data = pkt.right_hall
            self.left_ticks_pub.publish(left_msg)
            self.right_ticks_pub.publish(right_msg)
        except Exception as e:
            self.get_logger().error(f'Ошибка парсинга TelemetryPacket: {e}')

    def process_status(self, data):
        try:
            status_str = data.decode('utf-8')
            self.get_logger().debug(f'Статус: {status_str}')
            status_msg = String()
            status_msg.data = status_str
            self.status_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Ошибка обработки статуса: {str(e)}')

    def process_lidar_packet(self, data):
        if len(data) < 9:
            self.get_logger().debug('Пакет слишком короткий')
            return

        packet_type = data[4]
        indicator = data[5] if len(data) > 5 else 0
        #self.get_logger().info(f'LiDAR пакет: тип=0x{packet_type:02X}, индикатор=0x{indicator:02X}, длина={len(data)}')

        if packet_type == 0x61 and indicator == 0xAD:
            #self.get_logger().info('✅ Обнаружен пакет с данными сканирования (0x61,0xAD)')
            self.process_scan_packet(data)
        elif packet_type == 0xAE:
            self.get_logger().debug('Получен пакет скорости LiDAR (0xAE)')
        #else:
        #   self.get_logger().warn(f'Неизвестный тип пакета LiDAR: тип=0x{packet_type:02X}, индикатор=0x{indicator:02X}')

    def process_scan_packet(self, data):
        try:
            if len(data) < 15:
                self.get_logger().warn('process_scan_packet: пакет слишком короткий')
                return

            pkt_len = int.from_bytes(data[1:3], 'big')
            if len(data) < pkt_len:
                self.get_logger().warn(f'process_scan_packet: пакет неполный (ожидалось {pkt_len}, получено {len(data)})')
                return

            version = data[3]
            pkt_type = data[4]
            indicator = data[5]
            data_len = int.from_bytes(data[6:8], 'big')
            speed_raw = data[8]
            angle_offset_raw = int.from_bytes(data[9:11], 'big')
            start_angle_raw = int.from_bytes(data[11:13], 'big')
            #self.get_logger().info(f'process_scan_packet: ver={version}, pkt_type=0x{pkt_type:02X}, indicator=0x{indicator:02X}, data_len={data_len}, speed_raw={speed_raw}, angle_offset_raw={angle_offset_raw}, start_angle_raw={start_angle_raw}')

            remaining = data_len - 5
            if remaining < 0 or remaining % 3 != 0:
                self.get_logger().warn(f'process_scan_packet: некорректная длина данных: data_len={data_len}, remaining={remaining}')
                return
            num_points = remaining // 3
            #self.get_logger().info(f'process_scan_packet: num_points={num_points}')

            start_angle_deg = start_angle_raw * 0.01
            angle_step_deg = 22.5 / num_points if num_points > 0 else 0
            #self.get_logger().info(f'process_scan_packet: start_angle={start_angle_deg:.2f}°, angle_step={angle_step_deg:.2f}°')

            # Определение нового оборота
            if self._last_start_angle is not None:
                delta = start_angle_deg - self._last_start_angle
                if delta < -180.0 and self._packets_in_rev >= 4:
                    #elf.get_logger().info(f'Новый оборот! delta={delta:.1f}°, packets_in_rev={self._packets_in_rev}')
                    self._publish_full_scan()
                    self._accum_ranges = [float('inf')] * 360
                    self._accum_intensities = [0.0] * 360
                    self._packets_in_rev = 0
            else:
                self.get_logger().info('Первый пакет в обороте')

            self._last_start_angle = start_angle_deg

            # Обработка точек
            offset = 13
            valid_points_in_packet = 0
            for i in range(num_points):
                if offset + 2 >= pkt_len - 2:
                    #self.get_logger().warn(f'process_scan_packet: выход за границы данных на точке {i}, offset={offset}')
                    break
                signal = data[offset]
                dist_raw = int.from_bytes(data[offset+1:offset+3], 'big')
                angle_deg = start_angle_deg + angle_step_deg * i
                distance_m = dist_raw * self.distance_scale
                angle_index = int(round(angle_deg + 180.0)) % 360

                if signal > 0 and self.range_min <= distance_m <= self.range_max:
                    self._accum_ranges[angle_index] = distance_m
                    self._accum_intensities[angle_index] = float(signal)
                    valid_points_in_packet += 1
                else:
                    self.get_logger().debug(f'Точка {i}: signal={signal}, dist={dist_raw} -> {distance_m:.3f} м, угол={angle_deg:.2f}° -> индекс {angle_index} (игнорируется)')
                offset += 3

            self._packets_in_rev += 1
            #self.get_logger().info(f'process_scan_packet: обработано {valid_points_in_packet}/{num_points} точек, всего пакетов в обороте={self._packets_in_rev}')

        except Exception as e:
            self.get_logger().error(f'Ошибка обработки пакета сканирования: {e}')

    def _publish_full_scan(self):
        valid = sum(1 for r in self._accum_ranges if math.isfinite(r))
        #self.get_logger().info(f'Публикация полного скана: {valid}/360 валидных точек, пакетов в обороте={self._packets_in_rev}')
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = (self.angle_max - self.angle_min) / 360
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / 6.0
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        scan_msg.ranges = list(self._accum_ranges)
        scan_msg.intensities = list(self._accum_intensities)

        self.laser_pub.publish(scan_msg)
        #self.get_logger().info(f'✓ Полный скан опубликован')

    def destroy_node(self):
        self._running = False
        if self.socket:
            self.socket.close()
        if hasattr(self, 'receive_thread') and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
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