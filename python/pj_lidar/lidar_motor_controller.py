#!/usr/bin/env python3
# LIDAR MOTOR CONTROLLER
# ЗАДАЧА:
# - Управление скоростью LiDAR мотора на ESP32.
# - Использует параметры из config.yaml для установки PWM.
# ВХОДЫ:
# - Топик: /scan_state (std_msgs/String) - для определения состояния
# ВЫХОДЫ:
# - TCP команды: set_lidar_pwm:<value>
# ПРИМЕЧАНИЕ:
# - Автоматически устанавливает PWM в зависимости от состояния сканирования.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import time

# Импортировать конфиг-лоадер
try:
    from pj_lidar.config_loader import ConfigLoader
    CONFIG_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    CONFIG_AVAILABLE = False

class LidarMotorController(Node):
    """ROS2 узел для управления LiDAR мотором."""
    
    def __init__(self):
        super().__init__('lidar_motor_controller')
        
        # Загрузить конфиг
        if CONFIG_AVAILABLE:
            try:
                ConfigLoader.load()
                tcp_config = ConfigLoader.get_tcp_config()
                motor_config = ConfigLoader.get('lidar_motor', {})
                
                self.robot_ip = tcp_config.get('robot_ip', '192.168.0.23')
                self.tcp_port = tcp_config.get('tcp_port', 3333)
                self.lidar_min_pwm = motor_config.get('min_pwm', 130)
                self.lidar_max_pwm = motor_config.get('max_pwm', 170)
                self.lidar_default_pwm = motor_config.get('default_pwm', 145)
                self.get_logger().info("✓ Конфиг загружен из config.yaml")
            except Exception as e:
                self.get_logger().warn(f"Ошибка загрузки конфига: {e}")
                self._use_default_params()
        else:
            self._use_default_params()
        
        # Общая инициализация
        self._init_common()
    
    def _use_default_params(self):
        """Использовать параметры по умолчанию."""
        self.declare_parameter('lidar_motor.min_pwm', 130)
        self.declare_parameter('lidar_motor.max_pwm', 170)
        self.declare_parameter('lidar_motor.default_pwm', 145)
        self.declare_parameter('esp32_tcp_client.robot_ip', '192.168.0.23')
        self.declare_parameter('esp32_tcp_client.tcp_port', 3333)
        
        self.lidar_min_pwm = self.get_parameter('lidar_motor.min_pwm').value
        self.lidar_max_pwm = self.get_parameter('lidar_motor.max_pwm').value
        self.lidar_default_pwm = self.get_parameter('lidar_motor.default_pwm').value
        self.robot_ip = self.get_parameter('esp32_tcp_client.robot_ip').value
        self.tcp_port = self.get_parameter('esp32_tcp_client.tcp_port').value
    
    def _init_common(self):
        """Общая инициализация для всех режимов."""
        # Подписчик на состояние сканирования
        self.state_sub = self.create_subscription(
            String, 'robot_state', self.state_callback, 10)
        
        # Публикатор команд LiDAR - esp32_tcp_client подписывается на этот топик
        self.cmd_pub = self.create_publisher(String, 'lidar_motor_cmd', 10)
        
        # Таймер для периодической отправки команд
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Переменные состояния
        self.current_state = "IDLE"
        self.last_command_time = 0
        self.command_interval = 2.0  # Интервал между командами (сек)
        self._last_pwm_sent = None  # PWM, последний реально отправленный
        # TCP сокет - НЕ используем, esp32_tcp_client уже подключен!
        self.tcp_socket = None
        self.get_logger().info('Lidar Motor Controller инициализирован')
        self.get_logger().info(f'LiDAR PWM: min={self.lidar_min_pwm}, max={self.lidar_max_pwm}, default={self.lidar_default_pwm}')
        self.get_logger().info('Команды отправляются через топик /lidar_motor_cmd')
    
    def state_callback(self, msg):
        """Получить состояние сканирования."""
        self.current_state = msg.data
        self.get_logger().debug(f'Получено состояние: {self.current_state}')
        
        # Отправить команду немедленно при изменении состояния
        self.send_lidar_command()
    
    def timer_callback(self):
        """Периодическая отправка команды (если прошло достаточно времени)."""
        if self.current_state != "IDLE":
            self.send_lidar_command()
    
    def send_lidar_command(self):
        """Отправить команду управления LiDAR мотором только при изменении PWM."""
        current_time = time.time()
        # Определить PWM значение в зависимости от состояния робота
        if "ROTATING" in self.current_state or "MOVING_FORWARD" in self.current_state:
            pwm_value = self.lidar_max_pwm
            self.get_logger().info(f'🔄 LiDAR: максимальная скорость PWM = {pwm_value}')
        elif "OBSTACLE_DETECTED" in self.current_state:
            pwm_value = self.lidar_min_pwm
            self.get_logger().info(f'⚠️ LiDAR: минимальная скорость PWM = {pwm_value}')
        elif "SCAN_COMPLETE" in self.current_state:
            pwm_value = 0
            self.get_logger().info(f'✅ LiDAR: выключен')
        else:
            pwm_value = self.lidar_default_pwm
            self.get_logger().debug(f'⏸️ LiDAR: стандартная скорость PWM = {pwm_value}')

        # Отправлять команду только если PWM изменился
        if pwm_value != self._last_pwm_sent:
            if pwm_value == 0:
                self.send_tcp_command("lidar_off")
            else:
                self.send_tcp_command("lidar_on")
            self._last_pwm_sent = pwm_value
            self.last_command_time = current_time
        else:
            self.get_logger().debug(f'PWM {pwm_value} уже установлен, команда не отправляется')
    
    def send_tcp_command(self, command):
        """Отправить команду LiDAR через ROS2 топик."""
        try:
            # Публикуем команду в топик - esp32_tcp_client получит её
            msg = String()
            msg.data = command
            self.cmd_pub.publish(msg)
            self.get_logger().debug(f'📡 Команда отправлена: {command}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка отправки: {str(e)}')
    
    def destroy_node(self):
        """Остановить LiDAR мотор при завершении."""
        try:
            # Публикуем команду остановки лидара через топик
            stop_msg = String()
            stop_msg.data = 'set_lidar_pwm:0'
            self.cmd_pub.publish(stop_msg)
            self.get_logger().info('🛑 LiDAR мотор остановлен при завершении')
            import time as _time
            _time.sleep(0.1)
        except Exception:
            pass
        if self.tcp_socket:
            try:
                self.tcp_socket.close()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = LidarMotorController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Остановка...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()