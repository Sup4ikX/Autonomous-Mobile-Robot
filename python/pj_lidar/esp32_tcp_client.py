#!/usr/bin/env python3
# ESP32 TCP CLIENT
# ЗАДАЧА:
# - ROS2 узел для связи с ESP32 по TCP.
# - Принимает команды cmd_vel (Twist) и переводит их в TCP-команды для ESP32.
# - Публикует полученные от ESP32 строки в топик 'robot_status'.
# ВХОДЫ:
# - Параметры: robot_ip, tcp_port, timeout
# - Топик: /cmd_vel (geometry_msgs/Twist)
# ВЫХОДЫ:
# - Топик: /robot_status (std_msgs/String)
# ОСНОВНЫЕ ФУНКЦИИ:
# - connect(), send_command(), receive_loop(), cmd_vel_callback()
# ПРИМЕЧАНИЕ:
# - Для запуска: ros2 run pj_lidar esp32_tcp_client (или через launch)
#

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import socket
import threading
import time
import signal
import sys
import threading
from std_msgs.msg import Int64
import re
import select
import queue
import os
import errno


# Импортировать конфиг-лоадер с fallback
try:
    from pj_lidar.config_loader import ConfigLoader
    CONFIG_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    CONFIG_AVAILABLE = False

# Импорты для автоматического обнаружения
import socket
import subprocess
import netifaces
import struct

class Esp32TcpClient(Node):
    """ROS2 узел для управления ESP32 роботом через TCP соединение."""

    def signal_handler(self, sig, frame):
        if self._shutdown_flag:
            return
        self._shutdown_flag = True
        self.get_logger().warn(f'⚠️ Получен сигнал {sig}, экстренная остановка моторов!')
        # Отправляем команды остановки напрямую, минуя очередь (если сокет жив)
        try:
            if self.socket and self.connected:
                self.socket.sendall(b'disable_motors\n')
                self.socket.sendall(b'set_lidar_pwm:0\n')
                time.sleep(0.1)
        except:
            pass
        # Завершаем узел
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


    def __init__(self):
        super().__init__('esp32_tcp_client')
        
        # СНАЧАЛА загрузить значения по умолчанию из config.yaml (если есть),
        # а затем разрешить их переопределение через ROS2 параметры / launch.
        tcp_config = {}
        if CONFIG_AVAILABLE:
            try:
                ConfigLoader.load()
                tcp_config = ConfigLoader.get_tcp_config() or {}
                self.get_logger().info("✓ TCP параметры загружены из config.yaml")
            except Exception as e:
                self.get_logger().warn(f"Ошибка загрузки TCP конфига: {e}")
                tcp_config = {}

        # Параметры ROS2 с дефолтами из конфига
        self.declare_parameter('robot_ip', tcp_config.get('robot_ip', 'auto'))
        self.declare_parameter('tcp_port', tcp_config.get('tcp_port', 3333))
        self.declare_parameter('bind_local_ip', tcp_config.get('bind_local_ip', ''))
        self.declare_parameter('timeout', tcp_config.get('timeout', 5.0))
        self.declare_parameter('udp_port', tcp_config.get('udp_port', 4444))
        
        robot_ip = self.get_parameter('robot_ip').value
        tcp_port = self.get_parameter('tcp_port').value
        timeout = self.get_parameter('timeout').value
        self.bind_local_ip = self.get_parameter('bind_local_ip').value
        
        # Сохраняем tcp_port для использования в discover_esp32
        self.tcp_port = tcp_port
        
        # Если robot_ip = "auto", запустить автообнаружение
        if not robot_ip or robot_ip == 'auto' or robot_ip == 'discover':
            self.get_logger().info("🔍 Автоматическое обнаружение ESP32...")
            found_ip = self.discover_esp32()
            if found_ip:
                robot_ip = found_ip
                self.get_logger().info(f"✓ ESP32 найден: {robot_ip}")
            else:
                self.get_logger().warn("⚠️ ESP32 не найден")
                robot_ip = None
        
        # Если IP не определён после автообнаружения — ещё раз попробовать взять из конфига
        if not robot_ip and tcp_config:
            robot_ip = tcp_config.get('robot_ip', None)
            if not robot_ip or robot_ip == 'auto':
                self.get_logger().warn("⚠️ IP не задан в config.yaml")
        
        # Final fallback
        if not robot_ip:
            robot_ip = '192.168.0.23'
        
        self.robot_ip = robot_ip
        self.tcp_port = tcp_port
        self.timeout = timeout
        self.udp_port = self.get_parameter('udp_port').value
        
        self.get_logger().info(f'Подключение к {self.robot_ip}:{self.tcp_port}')
        
        # Переменные подключения
        self.socket = None
        self.connected = False
        # lock для буфера приёма и отдельный lock для отправки
        self.lock = threading.Lock()
        self.send_lock = threading.Lock()
        # Очередь исходящих команд + поток отправки
        self.send_queue = queue.Queue(maxsize=512)
        self._send_thread = threading.Thread(target=self._send_worker, daemon=True)
        self._send_thread_stop = False
        self.recv_buffer = ""
        # UDP listener
        self.udp_socket = None
        self.udp_thread = None
        self._udp_thread_stop = False
        
        self.last_left_speed = None
        self.last_right_speed = None
        self.last_disabled = False

        # Подписчик на команды скорости от ROS2 навигации
        # Подписчики: ручные и автокоманды отдельно
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_manual_callback, 10)

        # Подписчик на команды от scan_state_machine (автосканирование)
        self.cmd_vel_auto_sub = self.create_subscription(
            Twist, 'cmd_vel_auto', self.cmd_vel_auto_callback, 10)
        
        # Подписчик на команды LiDAR мотора от lidar_motor_controller
        self.lidar_cmd_sub = self.create_subscription(
            String, 'lidar_motor_cmd', self.lidar_motor_cmd_callback, 10)

        # Подписчик на состояние робота (чтобы знать, запущен ли автоскан)
        self.robot_state_sub = self.create_subscription(
            String, 'robot_state', self.robot_state_callback, 10)

        # Флаг автопилота
        self.auto_mode_active = False
        
        # Издатель статуса робота для мониторинга
        self.status_pub = self.create_publisher(
            String, 'robot_status', 10)
        
        self.left_ticks_pub = self.create_publisher(Int64, 'left_ticks', 10)
        self.right_ticks_pub = self.create_publisher(Int64, 'right_ticks', 10)

        # Таймеры для переподключения и запроса статуса
        self.reconnect_timer = self.create_timer(5.0, self.reconnect_callback)
        self.status_timer = self.create_timer(2.0, self.request_status)
        # экспоненциальный бэкофф
        self.reconnect_attempts = 0
        self.next_reconnect_time = 0.0
        self.reconnect_backoff_base = 1.0
        self.reconnect_backoff_max = 60.0
        # последний код ошибки connect_ex (int) или None
        self._last_connect_res = None
        
        # Поток приема данных в фоне
        self.receive_thread = None
        self.stop_receive = False

        # Тайминги для приоритета команд (auto > manual)
        self.last_auto_time = 0.0
        self.last_nonzero_time = 0.0
        self.auto_precedence_timeout = 0.2  # секунды, игнорировать manual если были auto
        
        # Попытка подключиться к ESP32
        self.connect()
        # Запустить поток отправки (он будет ждать подключения)
        self._send_thread.start()
        # Запустить UDP слушатель в отдельном потоке (приём телеметрии)
        try:
            self.udp_thread = threading.Thread(target=self.udp_listener_loop, daemon=True)
            self.udp_thread.start()
            self.get_logger().info(f'UDP listener запущен на порту {self.udp_port}')
        except Exception as e:
            self.get_logger().warn(f'Не удалось запустить UDP listener: {e}')
        self.get_logger().info('TCP клиент инициализирован и готов')
                # Регистрируем обработчик сигналов для гарантированной остановки
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        self._shutdown_flag = False

    def connect(self):
        """
        Подключение к TCP серверу на ESP32.
        При успешном подключении запускает поток для приема данных.
        """
        try:
            # Закрыть старое соединение если оно было
            if self.socket:
                self.socket.close()
            
            # Создать новый TCP сокет
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            
            self.get_logger().info(f'Подключение к {self.robot_ip}:{self.tcp_port}...')
            # Попытаемся привязать исходящий сокет к локальному интерфейсу,
            # чтобы система использовала корректный адрес/маршрут.
            try:
                local_ip = self._get_local_ip()
                if local_ip and not local_ip.startswith('127.'):
                    try:
                        self.get_logger().debug(f'Привязываю сокет к локальному IP {local_ip}')
                        self.socket.bind((local_ip, 0))
                    except Exception as e:
                        self.get_logger().debug(f'Не удалось привязать сокет к {local_ip}: {e}')
            except Exception:
                pass

            self.socket.connect((self.robot_ip, self.tcp_port))
            
            # Успешное подключение
            # После соединения используем select() для приёма — снимем общий timeout
            try:
                self.socket.settimeout(None)
            except Exception:
                pass
            self.connected = True
            try:
                local = self.socket.getsockname()
            except Exception:
                local = None
            try:
                peer = self.socket.getpeername()
            except Exception:
                peer = None
            self.get_logger().info(f'✓ Подключено к ESP32! local={local} peer={peer}')
            # diagnostic info
            try:
                neigh = subprocess.check_output(['ip', 'neigh', 'show', str(self.robot_ip)], stderr=subprocess.DEVNULL)
                self.get_logger().info(f'ARP entry at connect: {neigh.decode().strip()}')
            except Exception:
                pass
            try:
                arp = subprocess.check_output(['arp', '-n', str(self.robot_ip)], stderr=subprocess.DEVNULL)
                self.get_logger().info(f'arp -n:\n{arp.decode().strip()}')
            except Exception:
                pass
            
            # Запустить поток приема данных если его еще нет
            if self.receive_thread is None or not self.receive_thread.is_alive():
                self.stop_receive = False
                self.receive_thread = threading.Thread(
                    target=self.receive_loop, daemon=True)
                self.receive_thread.start()
            
        except Exception as e:
            # Дополнительная диагностика: попробовать connect_ex на том же IP:port
            try:
                test_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                test_sock.settimeout(1.0)
                res = test_sock.connect_ex((self.robot_ip, self.tcp_port))
                test_sock.close()
            except Exception:
                res = None

            local_ip = self._get_local_ip()
            if res is None:
                self.get_logger().error(f'Ошибка подключения: {e} (connect_ex недоступен). Локальный IP: {local_ip}')
            else:
                err_str = os.strerror(res) if isinstance(res, int) and res != 0 else 'OK'
                self.get_logger().error(
                    f'Ошибка подключения: {e} ; connect_ex -> {res} ({err_str}). Локальный IP: {local_ip}'
                )
            # дополнительная сетевая диагностика
            try:
                neigh = subprocess.check_output(['ip', 'neigh', 'show', str(self.robot_ip)], stderr=subprocess.DEVNULL)
                self.get_logger().info(f'ARP entry for {self.robot_ip}: {neigh.decode().strip()}')
            except Exception:
                pass
            try:
                arp = subprocess.check_output(['arp', '-n', str(self.robot_ip)], stderr=subprocess.DEVNULL)
                self.get_logger().info(f'arp -n output:\n{arp.decode().strip()}')
            except Exception:
                pass
            try:
                # arping may require root; ignore failures
                arping = subprocess.check_output(['arping', '-c', '2', '-I', 'eno1', str(self.robot_ip)], stderr=subprocess.DEVNULL)
                self.get_logger().info(f'arping result:\n{arping.decode().strip()}')
            except Exception:
                pass
            self.connected = False
            if self.socket:
                self.socket.close()
    
    def send_command(self, command):
        """
        Отправить команду на ESP32 через TCP.
        
        Args:
            command (str): команда для отправки (без \n)
            
        Returns:
            bool: успешность отправки
        """
        # Помещаем команду в очередь для асинхронной отправки.
        if not isinstance(command, str):
            try:
                command = str(command)
            except Exception:
                return False

        msg = command + '\n'
        try:
            # Небольшая задержка при наполненности очереди — drop старых команд
            try:
                self.send_queue.put(msg, block=False)
                return True
            except queue.Full:
                # Если очередь полная — попытаться аккуратно поставить с небольшим таймаутом
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
        """Поток, читающий команды из очереди и отправляющий по TCP.
        Защищает отправку отдельным lock и восстанавливает соединение при ошибках.
        """
        import time as _time
        while rclpy.ok() and not self._send_thread_stop:
            try:
                try:
                    msg = self.send_queue.get(timeout=0.2)
                except queue.Empty:
                    # ничего нет — продолжить
                    continue

                if not msg:
                    continue

                # Ждать пока сокет будет доступен
                if not self.connected or not self.socket:
                    # пробуем подождать небольшое время, если нет соединения — переопубликовать/отбросить
                    # пересадим обратно в очередь на повтор через короткий промежуток
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
                    if elapsed > 0.1:
                        self.get_logger().warn(f'Задержка при отправке команды {msg.strip()}: {elapsed:.3f}s')
                except Exception as e:
                    self.get_logger().error(f'Ошибка в _send_worker при отправке: {e}')
                    # Пометка о разрыве и попытка переподключиться — команда будет переотправлена позже
                    self.connected = False
                    try:
                        self.send_queue.put(msg, block=False)
                    except Exception:
                        pass
                    _time.sleep(0.2)
                    continue

            except Exception as e:
                self.get_logger().error(f'Ошибка в потоке отправки: {e}')
                _time.sleep(0.1)
    
    def receive_loop(self):
        """
        Цикл приема данных от ESP32 в отдельном потоке.
        Обрабатывает строки одну за другой по мере получения.
        """
        # Используем select.select() чтобы избежать блокирующих recv при высокой частоте телеметрии.
        while rclpy.ok() and self.connected and not self.stop_receive:
            try:
                rlist, _, _ = select.select([self.socket], [], [], 0.5)
                if not rlist:
                    continue

                data = self.socket.recv(4096)

                # Если получены пустые данные - соединение закрыто
                if not data:
                    try:
                        peer = self.socket.getpeername()
                    except Exception:
                        peer = None
                    self.get_logger().warn(f'ESP32 закрыл соединение (peer={peer})')
                    self.connected = False
                    break

                # Добавить в буфер (кратко блокируя общий буфер)
                with self.lock:
                    self.recv_buffer += data.decode('utf-8', errors='ignore')

                # Обрабатывать полные строки по одной, не удерживая долго общую блокировку
                while True:
                    with self.lock:
                        if '\n' not in self.recv_buffer:
                            break
                        line, self.recv_buffer = self.recv_buffer.split('\n', 1)
                    if line:
                        status_msg = String()
                        status_msg.data = line.strip()
                        self.status_pub.publish(status_msg)
                        self.get_logger().info(f"← Получено: {line.strip()}")
                        self.parse_status(line.strip())

            except Exception as e:
                # Игнорируем временные ошибки чтения и продолжаем попытки переподключения
                err_repr = repr(e)
                # сохранение кода ошибки для reconnect_callback
                try:
                    if isinstance(e, OSError):
                        self._last_connect_res = e.errno
                except Exception:
                    pass
                self.get_logger().error(f'Ошибка приема: {err_repr}')
                self.connected = False
                break
    
    def parse_status(self, line):
        """Парсит строку статуса, извлекая значения энкодеров и публикуя их."""
        # Ищем паттерн L=значение,R=значение
        match = re.search(r'L=(-?\d+),R=(-?\d+)', line)
        if match:
            left_ticks = int(match.group(1))
            right_ticks = int(match.group(2))
            left_msg = Int64()
            left_msg.data = left_ticks
            self.left_ticks_pub.publish(left_msg)
            right_msg = Int64()
            right_msg.data = right_ticks
            self.right_ticks_pub.publish(right_msg)
            self.get_logger().debug(f'Опубликованы энкодеры: L={left_ticks}, R={right_ticks}')
        else:
            self.get_logger().debug(f'Не удалось распарсить энкодеры из: {line}')

    def udp_listener_loop(self):
        """
        Простой UDP listener для приёма телеметрии от ESP32.
        Запускается в отдельном потоке, публикует полученные строки в `robot_status`.
        """
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Разрешаем переиспользование адреса и приём широковещательных пакетов
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            except Exception:
                pass
            self.udp_socket.bind(('', int(self.udp_port)))
            self.udp_socket.settimeout(1.0)
        except Exception as e:
            self.get_logger().error(f'Ошибка при создании UDP сокета: {e}')
            self.udp_socket = None
            return

        self.get_logger().debug(f'UDP listener listening on port {self.udp_port}')

        while rclpy.ok() and not self._udp_thread_stop:
            try:
                try:
                    data, addr = self.udp_socket.recvfrom(8192)
                except socket.timeout:
                    continue
                if not data:
                    continue
                text = data.decode('utf-8', errors='ignore').strip()
                # Публикуем в topic robot_status и логируем
                try:
                    msg = String()
                    msg.data = text
                    self.status_pub.publish(msg)
                    self.get_logger().debug(f'UDP ← {addr}: {text}')
                except Exception as e:
                    self.get_logger().warn(f'Ошибка при публикации UDP статуса: {e}')

            except Exception as e:
                self.get_logger().error(f'Ошибка в UDP listener: {e}')
                time.sleep(0.1)

    def cmd_vel_auto_callback(self, msg):
        self.get_logger().info(f"Получена авто-команда: linear={msg.linear.x}, angular={msg.angular.z}")
        self.last_auto_time = time.time()
        self.auto_mode_active = True
        self._process_cmd_vel(msg)

    def cmd_vel_manual_callback(self, msg):
        """Обработка ручных команд. Игнорируется, если недавно были авто-команды."""
        if self.auto_mode_active or (time.time() - self.last_auto_time < self.auto_precedence_timeout):
            self.get_logger().debug('Игнорирую ручную команду: автопилот активен или недавняя авто-команда')
            return
        self._process_cmd_vel(msg)

    def _process_cmd_vel(self, msg):
        """Общая обработка Twist в команды моторов с защитой от ложного disable."""
        linear = msg.linear.x
        angular = msg.angular.z

        left = int((-linear - angular) * 100)
        right = int((-linear + angular) * 100)

        left = max(-100, min(100, left))
        right = max(-100, min(100, right))

        speed_left = int(left * 2.55)
        speed_right = int(right * 2.55)

        # Минимальная скорость 95 (кроме полной остановки)
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

        self.get_logger().info(f'⬅️ Левый мотор: {speed_left}, ➡️ Правый мотор: {speed_right}')

        # Отправляем команды только при изменении скорости
        if speed_left != self.last_left_speed:
            self.send_command(f'set_motor_left:{speed_left}')
            self.last_left_speed = speed_left
        if speed_right != self.last_right_speed:
            self.send_command(f'set_motor_right:{speed_right}')
            self.last_right_speed = speed_right

        # Обновить время последней ненулевой команды и сбросить флаг disable
        if left != 0 or right != 0:
            self.last_nonzero_time = time.time()
            if self.last_disabled:
                self.last_disabled = False

        # Если оба мотора на нуле - отправлять disable только при переходе
        if left == 0 and right == 0:
            if not self.auto_mode_active and time.time() - self.last_nonzero_time > 0.15:
                if not self.last_disabled:
                    self.get_logger().info('🛑 Моторы остановлены (disable)')
                    self.send_command('disable_motors')
                    self.last_disabled = True
        else:
            self.last_disabled = False

        # Если автопилот больше не посылал команды длительное время — снять флаг
        if time.time() - self.last_auto_time > 1.0:
            self.auto_mode_active = False

    def robot_state_callback(self, msg):
        """Отслеживаем состояние робота из state machine для явного контроля автопилота."""
        try:
            state = msg.data.upper() if msg.data else ''
            # Если state содержит признаки движения автопилота — включаем режим
            if 'MOVING_FORWARD' in state or 'ROTATING' in state or 'SPIRAL' in state or 'OBSTACLE' in state:
                self.auto_mode_active = True
            elif 'IDLE' in state or 'SCAN_COMPLETE' in state or '✓✓✓' in state:
                self.auto_mode_active = False
        except Exception:
            pass
    
    def lidar_motor_cmd_callback(self, msg):
        """
        Обработка команды LiDAR мотора от lidar_motor_controller.
        
        Args:
            msg (String): команда типа 'set_lidar_pwm:<value>'
        """
        command = msg.data
        if command:
            self.get_logger().debug(f'Получена команда LiDAR: {command}')
            self.send_command(command)
    
    def reconnect_callback(self):
        """Попытка переподключения с экспоненциальным бэкоффом.

        При ошибках маршрутизации будет инициировано повторное автообнаружение ESP32.
        """
        now = time.time()
        if self.connected:
            # сброс счётчика попыток при удачном подключении
            self.reconnect_attempts = 0
            self.next_reconnect_time = 0.0
            return

        if now < self.next_reconnect_time:
            return

        self.get_logger().info('Попытка переподключения...')

        # Если предыдущая ошибка указала на проблему маршрута — попробуем автообнаружение
        if self._last_connect_res in (errno.ENETUNREACH, errno.EHOSTUNREACH):
            self.get_logger().warn('Предыдущая попытка вернула ошибку маршрутизации — запускаю discover_esp32()')
            found = self.discover_esp32()
            if found:
                self.get_logger().info(f'Попробую подключиться к вновь найденному ESP32: {found}')
                self.robot_ip = found

        # Попытка подключиться
        self.connect()

        # Если не подключились — увеличиваем бэкофф
        if not self.connected:
            self.reconnect_attempts += 1
            backoff = min(self.reconnect_backoff_base * (2 ** (self.reconnect_attempts - 1)), self.reconnect_backoff_max)
            self.next_reconnect_time = now + backoff
            self.get_logger().info(f'Следующая попытка переподключения через {backoff:.1f}s')
    
    def request_status(self):
        """Периодический запрос статуса робота для мониторинга."""
        if self.connected:
            self.send_command('status')
    
    # 👇👇👇 ДОБАВЛЯЙ СВОИ МЕТОДЫ УПРАВЛЕНИЯ ЗДЕСЬ 👇👇👇
    
    def my_custom_control(self, param1, param2):
        """Пользовательская функция управления."""
        command = f'my_custom_command:{param1}:{param2}'
        self.send_command(command)
        self.get_logger().info(f'Выполнена команда: {command}')
    
    def move_forward_distance(self, distance_m):
        """Движение вперед на заданное расстояние."""
        # distance_m: расстояние в метрах
        self.send_command(f'move_forward:{distance_m}')
    
    def rotate_degrees(self, degrees):
        """Вращение на угол в градусах."""
        self.send_command(f'rotate:{degrees}')
    
    def emergency_stop(self):
        """Аварийная остановка."""
        self.send_command('emergency_stop')
        self.get_logger().warn('⚠️ АВАРИЙНАЯ ОСТАНОВКА!')
    
    # 👆👆👆 КОНЕЦ ТВОИХ МЕТОДОВ 👆👆👆
    
    def discover_esp32(self):
        """Автоматическое обнаружение ESP32 в локальной сети."""
        self.get_logger().info("🔍 Поиск ESP32 в сети...")
        
        # Метод 1: Попробовать подключиться к известным IP диапазонам
        common_ips = [
            '192.168.0.23',   # Частый IP
            '192.168.0.100',  # DHCP диапазон
            '192.168.0.101',
            '192.168.0.102',
            '192.168.1.23',
            '192.168.1.100',
            '192.168.4.1',    # ESP AP mode
            '192.168.4.2',
            '192.168.4.10',
            '192.168.4.20',
            '192.168.4.30',
            '192.168.5.1',
            '192.168.5.2',
            '192.168.8.1',
            '192.168.8.2',
        ]
        
        for ip in common_ips:
            if self._try_connect(ip, self.tcp_port):
                self.get_logger().info(f"✓ ESP32 найден по адресу: {ip}")
                return ip
        
        # Метод 2: Сканирование локальной подсети
        local_ip = self._get_local_ip()
        if local_ip:
            subnet = local_ip.rsplit('.', 1)[0]
            self.get_logger().info(f"🔍 Сканирование подсети {subnet}.0/24...")
            
            # Параллельное сканирование порта
            import concurrent.futures
            with concurrent.futures.ThreadPoolExecutor(max_workers=20) as executor:
                futures = [executor.submit(self._try_connect, f"{subnet}.{i}", self.tcp_port) 
                          for i in range(1, 255)]
                for i, future in enumerate(concurrent.futures.as_completed(futures)):
                    if future.result():
                        found_ip = f"{subnet}.{i}"
                        self.get_logger().info(f"✓ ESP32 найден: {found_ip}")
                        return found_ip
        
        # Метод 3: mDNS/DNS-SD (если доступно)
        try:
            self.get_logger().info("🔍 Пробуем mDNS (esp32.local)...")
            result = subprocess.run(['nslookup', 'esp32.local'], 
                                 capture_output=True, timeout=3)
            if result.returncode == 0:
                # Извлечь IP из nslookup
                for line in result.stdout.decode().split('\n'):
                    if 'Address:' in line and not line.endswith(':#'):
                        ip = line.split(':')[-1].strip()
                        if '.' in ip and not ip.startswith('127'):
                            self.get_logger().info(f"✓ ESP32 найден через mDNS: {ip}")
                            return ip
        except Exception as e:
            self.get_logger().debug(f"mDNS недоступен: {e}")
        
        # Метод 4: UDP broadcast для поиска
        try:
            self.get_logger().info("🔍 Отправка UDP broadcast...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.sendto(b'FIND_ROBOT', ('<broadcast>', 8889))
            sock.settimeout(3.0)
            
            while True:
                try:
                    data, addr = sock.recvfrom(1024)
                    response = data.decode('utf-8', errors='ignore')
                    if 'ROBOT_IP' in response or 'ESP32' in response:
                        self.get_logger().info(f"✓ ESP32 ответил: {addr[0]}")
                        sock.close()
                        return addr[0]
                except socket.timeout:
                    break
            sock.close()
        except Exception as e:
            self.get_logger().debug(f"UDP broadcast недоступен: {e}")
        
        self.get_logger().warn("❌ ESP32 не найден в сети")
        return None
    
    def _try_connect(self, ip, port):
        """Попытка подключения к IP:port."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.5)
            result = sock.connect_ex((ip, port))
            sock.close()
            return result == 0
        except:
            return False
    
    def _get_local_ip(self):
        """Получить локальный IP адрес."""
        try:
            # Создать сокет для определения локального IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(1.0)
            # Не нужно реально отправлять данные
            s.connect(('8.8.8.8', 80))
            local_ip = s.getsockname()[0]
            s.close()
            return local_ip
        except:
            return None
    
    def destroy_node(self):
        """Корректное завершение - остановить моторы, выключить LiDAR, закрыть соединение."""
        # Остановим потоки при завершении
        self.stop_receive = True
        self._send_thread_stop = True
        # Отправить команды остановки перед закрытием соединения
        if self.connected and self.socket:
            try:
                self.get_logger().info('🛑 Завершение: останавливаем моторы и LiDAR...')
                # Остановить двигатели робота
                # Ставим в очередь команды остановки, даём им шанс уйти
                try:
                    self.send_queue.put_nowait('disable_motors\n')
                    self.send_queue.put_nowait('set_lidar_pwm:0\n')
                except Exception:
                    pass
                import time as _time
                _time.sleep(0.15)  # Дать время на отправку команд
            except Exception as e:
                self.get_logger().warn(f'Ошибка при отправке команд остановки: {e}')
        if self.socket:
            try:
                self.socket.close()
            except Exception:
                pass
        # Остановить UDP поток и закрыть UDP сокет
        try:
            self._udp_thread_stop = True
            if self.udp_socket:
                try:
                    self.udp_socket.close()
                except Exception:
                    pass
            if self.udp_thread and self.udp_thread.is_alive():
                try:
                    self.udp_thread.join(timeout=0.5)
                except Exception:
                    pass
        except Exception:
            pass
        # Дать потокам время корректно завершиться
        try:
            if self._send_thread.is_alive():
                self._send_thread.join(timeout=0.5)
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    """Точка входа - инициализация ROS2 и запуск узла."""
    rclpy.init(args=args)
    client = Esp32TcpClient()
    
    # Использовать MultiThreadedExecutor для одновременной обработки:
    # 1. ROS2 callbacks (cmd_vel подписка)
    # 2. Приема TCP данных в фоновом потоке
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