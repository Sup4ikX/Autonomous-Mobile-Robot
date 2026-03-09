#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import socket
import threading
import time
import signal
import sys
from std_msgs.msg import Int32
import re
import select
import queue
import os
import errno

try:
    from pj_lidar.config_loader import ConfigLoader
    CONFIG_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    CONFIG_AVAILABLE = False

class Esp32TcpClient(Node):
    def signal_handler(self, sig, frame):
        if self._shutdown_flag:
            return
        self._shutdown_flag = True
        self.get_logger().warn(f'⚠️ Получен сигнал {sig}, экстренная остановка моторов!')
        try:
            if self.socket and self.connected:
                self.socket.sendall(b'disable_motors\n')
                self.socket.sendall(b'lidar_off\n')
                time.sleep(0.1)
        except:
            pass
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    def __init__(self):
        super().__init__('esp32_tcp_client')
        tcp_config = {}
        if CONFIG_AVAILABLE:
            try:
                ConfigLoader.load()
                tcp_config = ConfigLoader.get_tcp_config() or {}
                self.get_logger().info("✓ TCP параметры загружены из config.yaml")
            except Exception as e:
                self.get_logger().warn(f"Ошибка загрузки TCP конфига: {e}")
                tcp_config = {}

        self.declare_parameter('robot_ip', tcp_config.get('robot_ip', 'auto'))
        self.declare_parameter('tcp_port', tcp_config.get('tcp_port', 3333))
        self.declare_parameter('timeout', tcp_config.get('timeout', 5.0))
        self.declare_parameter('udp_port', tcp_config.get('udp_port', 4444))

        robot_ip = self.get_parameter('robot_ip').value
        tcp_port = self.get_parameter('tcp_port').value
        timeout = self.get_parameter('timeout').value
        self.udp_port = self.get_parameter('udp_port').value

        self.use_telemetry_discovery = (robot_ip == 'auto' or robot_ip == 'discover')
        self.discovery_done = False

        self.robot_ip = None
        self.tcp_port = None
        self.timeout = timeout

        self.socket = None
        self.connected = False
        self.lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.send_queue = queue.Queue(maxsize=512)
        self._send_thread = threading.Thread(target=self._send_worker, daemon=True)
        self._send_thread_stop = False
        self.recv_buffer = ""

        self.last_left_speed = 0
        self.last_right_speed = 0
        self.last_disabled = False
        self.last_lidar_command = None
        self.last_lidar_time = 0.0
        self.lidar_cooldown = 2.0

        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_manual_callback, 10)
        self.cmd_vel_auto_sub = self.create_subscription(Twist, 'cmd_vel_auto', self.cmd_vel_auto_callback, 10)
        self.lidar_cmd_sub = self.create_subscription(String, 'lidar_motor_cmd', self.lidar_motor_cmd_callback, 10)
        self.robot_state_sub = self.create_subscription(String, 'robot_state', self.robot_state_callback, 10)
        self.setspeed_cmd_sub = self.create_subscription(String, 'setspeed_cmd', self.setspeed_callback, 10)

        self.auto_mode_active = False

        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.left_ticks_pub = self.create_publisher(Int32, 'left_ticks', 10)
        self.right_ticks_pub = self.create_publisher(Int32, 'right_ticks', 10)

        self.reconnect_timer = self.create_timer(5.0, self.reconnect_callback)
        self.reconnect_attempts = 0
        self.next_reconnect_time = 0.0
        self.reconnect_backoff_base = 1.0
        self.reconnect_backoff_max = 60.0
        self._last_connect_res = None

        self.receive_thread = None
        self.stop_receive = False

        self.last_auto_time = 0.0
        self.last_nonzero_time = 0.0
        self.auto_precedence_timeout = 0.2

        if self.use_telemetry_discovery:
            self.get_logger().info("🔍 Ожидание телеметрии для определения IP и порта...")
            self.discover_from_telemetry()
        else:
            self.robot_ip = robot_ip
            self.tcp_port = tcp_port
            self.get_logger().info(f'Подключение к {self.robot_ip}:{self.tcp_port} (из параметров)')
            self.connect()

        self._send_thread.start()
        self.get_logger().info('TCP клиент инициализирован и готов')
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        self._shutdown_flag = False

    def discover_from_telemetry(self):
        if self.discovery_done:
            self.connect()
            return
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp_socket.settimeout(10.0)
        try:
            udp_socket.bind(('0.0.0.0', self.udp_port))
        except Exception as e:
            self.get_logger().error(f'Не удалось открыть UDP порт {self.udp_port}: {e}')
            self.get_logger().info("Пробуем автоматическое сканирование сети...")
            self.robot_ip = self.discover_esp32()
            if self.robot_ip:
                self.tcp_port = self.get_parameter('tcp_port').value
                self.discovery_done = True
                self.connect()
            else:
                self.get_logger().error("❌ Не удалось определить IP робота")
            return

        self.get_logger().info(f"Ожидание телеметрии на порту {self.udp_port}...")
        start_time = time.time()
        found = False
        while not found and time.time() - start_time < 10.0:
            try:
                data, addr = udp_socket.recvfrom(256)
                if len(data) >= 15 and data[0] == 0xBB:
                    ip_bytes = data[1:5]
                    port_bytes = data[5:7]
                    tcp_port = int.from_bytes(port_bytes, 'big')
                    ip = '.'.join(str(b) for b in ip_bytes)
                    self.get_logger().info(f"Получена телеметрия: IP={ip}, TCP порт={tcp_port}")
                    self.robot_ip = ip
                    self.tcp_port = tcp_port
                    found = True
                    self.discovery_done = True
                    break
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Ошибка приёма UDP: {e}")
                break

        udp_socket.close()
        if found:
            self.connect()
        else:
            self.get_logger().error("❌ Телеметрия не получена, пробуем автоматическое сканирование...")
            self.robot_ip = self.discover_esp32()
            if self.robot_ip:
                self.tcp_port = self.get_parameter('tcp_port').value
                self.discovery_done = True
                self.connect()
            else:
                self.get_logger().error("❌ Не удалось определить IP робота")

    def connect(self):
        if self.robot_ip is None or self.tcp_port is None:
            self.get_logger().error("Невозможно подключиться: IP или порт не определены")
            return False
        try:
            if self.socket:
                self.socket.close()
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.get_logger().info(f'Подключение к {self.robot_ip}:{self.tcp_port}...')
            try:
                local_ip = self._get_local_ip()
                if local_ip and not local_ip.startswith('127.'):
                    try:
                        self.socket.bind((local_ip, 0))
                    except:
                        pass
            except:
                pass
            self.socket.connect((self.robot_ip, self.tcp_port))
            try:
                self.socket.settimeout(None)
            except:
                pass
            self.connected = True
            try:
                local = self.socket.getsockname()
            except:
                local = None
            try:
                peer = self.socket.getpeername()
            except:
                peer = None
            self.get_logger().info(f'✓ Подключено к ESP32! local={local} peer={peer}')

            try:
                self.send_command('lidar_on')
                self.get_logger().info('Команда lidar_on отправлена после подключения')
            except Exception as e:
                self.get_logger().error(f'Ошибка отправки lidar_on: {e}')

            if self.receive_thread is None or not self.receive_thread.is_alive():
                self.stop_receive = False
                self.receive_thread = threading.Thread(target=self.receive_loop, daemon=True)
                self.receive_thread.start()
            return True
        except Exception as e:
            # обработка ошибок
            self.connected = False
            if self.socket:
                self.socket.close()
            return False

    def send_command(self, command):
        if not isinstance(command, str):
            try:
                command = str(command)
            except:
                return False
        msg = command + '\n'
        self.get_logger().debug(f'📤 Постановка в очередь: {command}')
        try:
            try:
                self.send_queue.put(msg, block=False)
                return True
            except queue.Full:
                try:
                    self.send_queue.put(msg, block=True, timeout=0.05)
                    return True
                except queue.Full:
                    self.get_logger().warn('Очередь отправки полна — команда отброшена')
                    return False
        except Exception as e:
            self.get_logger().error(f'Ошибка постановки в очередь отправки: {e}')
            return False

    def _send_worker(self):
        import time as _time
        while rclpy.ok() and not self._send_thread_stop:
            try:
                try:
                    msg = self.send_queue.get(timeout=0.2)
                except queue.Empty:
                    continue
                if not msg:
                    continue
                if not self.connected or not self.socket:
                    try:
                        self.send_queue.put(msg, block=False)
                    except queue.Full:
                        pass
                    _time.sleep(0.1)
                    continue
                t0 = _time.time()
                try:
                    with self.send_lock:
                        self.socket.sendall(msg.encode('utf-8'))
                    elapsed = _time.time() - t0
                    cmd_clean = msg.strip()
                    self.get_logger().debug(f'📤➡️ Отправлено по TCP: {cmd_clean} (за {elapsed*1000:.1f} мс)')
                    if elapsed > 0.1:
                        self.get_logger().warn(f'Задержка при отправке команды {cmd_clean}: {elapsed:.3f}s')
                except Exception as e:
                    self.get_logger().error(f'Ошибка в _send_worker при отправке: {e}')
                    self.connected = False
                    try:
                        self.send_queue.put(msg, block=False)
                    except:
                        pass
                    _time.sleep(0.2)
                    continue
            except Exception as e:
                self.get_logger().error(f'Ошибка в потоке отправки: {e}')
                _time.sleep(0.1)

    def receive_loop(self):
        while rclpy.ok() and self.connected and not self.stop_receive:
            try:
                rlist, _, _ = select.select([self.socket], [], [], 0.5)
                if not rlist:
                    continue
                data = self.socket.recv(4096)
                if not data:
                    try:
                        peer = self.socket.getpeername()
                    except:
                        peer = None
                    self.get_logger().warn(f'ESP32 закрыл соединение (peer={peer})')
                    self.connected = False
                    break
                with self.lock:
                    self.recv_buffer += data.decode('utf-8', errors='ignore')
                while True:
                    with self.lock:
                        if '\n' not in self.recv_buffer:
                            break
                        line, self.recv_buffer = self.recv_buffer.split('\n', 1)
                    if line:
                        status_msg = String()
                        status_msg.data = line.strip()
                        self.status_pub.publish(status_msg)
                        self.parse_status(line.strip())
            except Exception as e:
                try:
                    if isinstance(e, OSError):
                        self._last_connect_res = e.errno
                except:
                    pass
                self.get_logger().error(f'Ошибка приема: {repr(e)}')
                self.connected = False
                break

    def parse_status(self, line):
        match = re.search(r'L=(-?\d+),R=(-?\d+)', line)
        if match:
            left_ticks = int(match.group(1))
            right_ticks = int(match.group(2))
            left_msg = Int32()
            left_msg.data = left_ticks
            self.left_ticks_pub.publish(left_msg)
            right_msg = Int32()
            right_msg.data = right_ticks
            self.right_ticks_pub.publish(right_msg)
        else:
            self.get_logger().debug(f'Не удалось распарсить энкодеры из: {line}')

    def cmd_vel_auto_callback(self, msg):
        self.last_auto_time = time.time()
        self.auto_mode_active = True
        self._process_cmd_vel(msg)

    def setspeed_callback(self, msg):
        command = msg.data
        if not command or not command.startswith('setspeed:'):
            return
        try:
            params = command[len('setspeed:'):].strip().split(',')
            if len(params) != 2:
                self.get_logger().warn(f'Неверный формат setspeed: {command}')
                return
            left = int(params[0].strip())
            right = int(params[1].strip())
            self.send_command(f'set_motor_left:{left}')
            self.send_command(f'set_motor_right:{right}')
            self.get_logger().info(f'📥 Команда setspeed обработана: {left}, {right}')
        except ValueError as e:
            self.get_logger().warn(f'Ошибка разбора setspeed: {e}')

    def cmd_vel_manual_callback(self, msg):
        if self.auto_mode_active or (time.time() - self.last_auto_time < self.auto_precedence_timeout):
            return
        self._process_cmd_vel(msg)

    def _process_cmd_vel(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        left = int((-linear - angular) * 100)
        right = int((-linear + angular) * 100)

        left = max(-100, min(100, left))
        right = max(-100, min(100, right))

        speed_left = int(left * 2.55)
        speed_right = int(right * 2.55)

        MIN_SPEED = 95
        if speed_left != 0:
            if speed_left > 0:
                speed_left = max(MIN_SPEED, speed_left)
            else:
                speed_left = min(-MIN_SPEED, speed_left)
        if speed_right != 0:
            if speed_right > 0:
                speed_right = max(MIN_SPEED, speed_right)
            else:
                speed_right = min(-MIN_SPEED, speed_right)

        if speed_left != self.last_left_speed:
            self.send_command(f'set_motor_left:{speed_left}')
            self.last_left_speed = speed_left
        if speed_right != self.last_right_speed:
            self.send_command(f'set_motor_right:{speed_right}')
            self.last_right_speed = speed_right

        if left != 0 or right != 0:
            self.last_nonzero_time = time.time()
            if self.last_disabled:
                self.last_disabled = False

        if left == 0 and right == 0:
            if not self.auto_mode_active and time.time() - self.last_nonzero_time > 0.15:
                if not self.last_disabled:
                    self.send_command('disable_motors')
                    self.last_disabled = True
        else:
            self.last_disabled = False

        if time.time() - self.last_auto_time > 1.0:
            self.auto_mode_active = False

    def robot_state_callback(self, msg):
        try:
            state = msg.data.upper() if msg.data else ''
            if 'MOVING_FORWARD' in state or 'ROTATING' in state or 'SPIRAL' in state or 'OBSTACLE' in state:
                self.auto_mode_active = True
            elif 'IDLE' in state or 'SCAN_COMPLETE' in state or '✓✓✓' in state:
                self.auto_mode_active = False
        except:
            pass

    def lidar_motor_cmd_callback(self, msg):
        command = msg.data
        if not command:
            return
        now = time.time()
        if command == self.last_lidar_command and (now - self.last_lidar_time) < self.lidar_cooldown:
            self.get_logger().debug(f'Команда {command} повторяется слишком часто, пропускаем')
            return
        self.get_logger().info(f'📥 Получена команда LiDAR: {command}')
        self.send_command(command)
        self.last_lidar_command = command
        self.last_lidar_time = now

    def reconnect_callback(self):
        now = time.time()
        if self.connected:
            self.reconnect_attempts = 0
            self.next_reconnect_time = 0.0
            return
        if now < self.next_reconnect_time:
            return
        self.get_logger().info('Попытка переподключения...')
        if self.robot_ip and self.tcp_port and self.discovery_done:
            self.connect()
        else:
            self.discover_from_telemetry()
        if not self.connected:
            self.reconnect_attempts += 1
            backoff = min(self.reconnect_backoff_base * (2 ** (self.reconnect_attempts - 1)), self.reconnect_backoff_max)
            self.next_reconnect_time = now + backoff

    def request_status(self):
        if self.connected:
            self.send_command('status')

    def discover_esp32(self):
        self.get_logger().info("🔍 Поиск ESP32 в сети (сканирование)...")
        local_ip = self._get_local_ip()
        if local_ip:
            subnet = local_ip.rsplit('.', 1)[0]
            self.get_logger().info(f"🔍 Сканирование подсети {subnet}.0/24...")
            import concurrent.futures
            with concurrent.futures.ThreadPoolExecutor(max_workers=20) as executor:
                futures = [executor.submit(self._try_connect, f"{subnet}.{i}", self.tcp_port) for i in range(1, 255)]
                for future in concurrent.futures.as_completed(futures):
                    if future.result():
                        return future.result()
        try:
            result = subprocess.run(['nslookup', 'esp32.local'], capture_output=True, timeout=3)
            if result.returncode == 0:
                for line in result.stdout.decode().split('\n'):
                    if 'Address:' in line and not line.endswith(':#'):
                        ip = line.split(':')[-1].strip()
                        if '.' in ip and not ip.startswith('127'):
                            return ip
        except:
            pass
        self.get_logger().error("❌ ESP32 не найден в сети")
        return None

    def _try_connect(self, ip, port):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.5)
            result = sock.connect_ex((ip, port))
            sock.close()
            return result == 0
        except:
            return False

    def _get_local_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(1.0)
            s.connect(('8.8.8.8', 80))
            local_ip = s.getsockname()[0]
            s.close()
            return local_ip
        except:
            return None

    def destroy_node(self):
        self.stop_receive = True
        self._send_thread_stop = True
        if self.connected and self.socket:
            try:
                self.get_logger().info('🛑 Завершение: останавливаем моторы и LiDAR...')
                try:
                    self.send_queue.put_nowait('disable_motors\n')
                    self.send_queue.put_nowait('lidar_off\n')
                except:
                    pass
                time.sleep(0.15)
            except Exception as e:
                self.get_logger().warn(f'Ошибка при отправке команд остановки: {e}')
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        try:
            if self._send_thread.is_alive():
                self._send_thread.join(timeout=0.5)
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    client = Esp32TcpClient()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(client)
    try:
        executor.spin()
    except KeyboardInterrupt:
        client.get_logger().info('Завершение...')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()