#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

try:
    from pj_lidar.config_loader import ConfigLoader
    CONFIG_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    CONFIG_AVAILABLE = False

class LidarMotorController(Node):
    def __init__(self):
        super().__init__('lidar_motor_controller')
        
        if CONFIG_AVAILABLE:
            try:
                ConfigLoader.load()
                tcp_config = ConfigLoader.get_tcp_config()
                motor_config = ConfigLoader.get('lidar_motor', {})
                self.robot_ip = tcp_config.get('robot_ip', '192.168.0.23')
                self.tcp_port = tcp_config.get('tcp_port', 3333)
                self.get_logger().info("✓ Конфиг загружен из config.yaml")
            except Exception as e:
                self.get_logger().warn(f"Ошибка загрузки конфига: {e}")
                self._use_default_params()
        else:
            self._use_default_params()
        
        self._init_common()
    
    def _use_default_params(self):
        self.declare_parameter('esp32_tcp_client.robot_ip', '192.168.0.23')
        self.declare_parameter('esp32_tcp_client.tcp_port', 3333)
        
        self.robot_ip = self.get_parameter('esp32_tcp_client.robot_ip').value
        self.tcp_port = self.get_parameter('esp32_tcp_client.tcp_port').value
    
    def _init_common(self):
        self.state_sub = self.create_subscription(String, 'robot_state', self.state_callback, 10)
        self.cmd_pub = self.create_publisher(String, 'lidar_motor_cmd', 10)
        
        self.current_state = "IDLE"
        self.last_command = None
        self.last_command_time = 0.0
        self.command_cooldown = 5.0
        
        self.get_logger().info('Lidar Motor Controller инициализирован (включение LiDAR выполняется TCP-клиентом)')
        self.get_logger().info('Команды отправляются через топик /lidar_motor_cmd')
    
    def state_callback(self, msg):
        new_state = msg.data
        if new_state == self.current_state:
            return
        self.current_state = new_state
        self.get_logger().info(f'Получено состояние: {new_state}')
        
        # Выключаем LiDAR при завершении сканирования
        if "SCAN_COMPLETE" in self.current_state:
            command = "lidar_off"
            now = time.time()
            if command != self.last_command or (now - self.last_command_time) > self.command_cooldown:
                self.send_command(command)
                self.last_command = command
                self.last_command_time = now
    
    def send_command(self, command):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.get_logger().debug(f'📡 Команда отправлена: {command}')
    
    def destroy_node(self):
        stop_msg = String()
        stop_msg.data = 'lidar_off'
        self.cmd_pub.publish(stop_msg)
        self.get_logger().info('🛑 LiDAR выключен при завершении')
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