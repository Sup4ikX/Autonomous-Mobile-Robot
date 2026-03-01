import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from threading import Thread

class ROS2Watchdog(Node):
    """Watchdog для мониторинга и восстановления узлов."""
    
    def __init__(self):
        super().__init__('ros2_watchdog')
        
        # Параметры
        self.declare_parameter('heartbeat_timeout', 5.0)
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        
        # Издатель для аварийной остановки
        self.emergency_pub = self.create_publisher(
            String, '/emergency_stop', 10)
        
        # Отслеживание узлов
        self.last_heartbeat = {}
        self.nodes_to_monitor = [
            'esp32_tcp_client',
            'lidar_udp_server',
            'scan_state_machine',
            'slam_toolbox'
        ]
        
        # Инициализировать heartbeat
        for node in self.nodes_to_monitor:
            self.last_heartbeat[node] = time.time()
        
        # Таймер проверки (1 Hz)
        self.timer = self.create_timer(1.0, self.check_heartbeat)
        
        self.get_logger().info('✓ ROS2 Watchdog инициализирован')
    
    def check_heartbeat(self):
        """Проверить heartbeat узлов."""
        current_time = time.time()
        
        for node_name in self.nodes_to_monitor:
            last_beat = self.last_heartbeat.get(node_name, current_time)
            elapsed = current_time - last_beat
            
            if elapsed > self.heartbeat_timeout:
                self.get_logger().error(
                    f"⚠️ Узел {node_name} не отвечает {elapsed:.1f}s")
                
                # Отправить сигнал аварийной остановки
                self.emergency_stop(node_name)
    
    def emergency_stop(self, node_name: str):
        """Отправить сигнал аварийной остановки."""
        msg = String()
        msg.data = f"EMERGENCY_STOP:{node_name}"
        self.emergency_pub.publish(msg)
        
        self.get_logger().warn("🚨 АВАРИЙНАЯ ОСТАНОВКА!")
    
    def update_heartbeat(self, node_name: str):
        """Обновить heartbeat узла."""
        self.last_heartbeat[node_name] = time.time()

def main(args=None):
    rclpy.init(args=args)
    watchdog = ROS2Watchdog()
    rclpy.spin(watchdog)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
