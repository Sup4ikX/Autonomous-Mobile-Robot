import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import String
import time

class DiagnosticsAggregator(Node):
    """Агрегатор диагностики для ROS2."""
    
    def __init__(self):
        super().__init__('diagnostics_aggregator')
        
        # Подписчики
        self.status_sub = self.create_subscription(
            String, '/robot_status', self.status_callback, 10)
        
        self.scan_complete_sub = self.create_subscription(
            String, '/slam_status', self.slam_callback, 10)
        
        # Издатель
        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics_agg', 1)
        
        # Таймер публикации (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_diagnostics)
        
        # Состояние компонентов
        self.components = {
            'robot_main': {'level': DiagnosticStatus.UNKNOWN, 'message': 'No data'},
            'lidar': {'level': DiagnosticStatus.UNKNOWN, 'message': 'No data'},
            'slam': {'level': DiagnosticStatus.UNKNOWN, 'message': 'No data'},
            'motors': {'level': DiagnosticStatus.UNKNOWN, 'message': 'No data'},
        }
        
        self.get_logger().info('✓ Diagnostics Aggregator инициализирован')
    
    def status_callback(self, msg):
        """Получить статус робота."""
        status = msg.data.upper()
        
        if 'ERROR' in status:
            self.components['robot_main']['level'] = DiagnosticStatus.ERROR
            self.components['robot_main']['message'] = status
        elif 'WARN' in status:
            self.components['robot_main']['level'] = DiagnosticStatus.WARN
            self.components['robot_main']['message'] = status
        else:
            self.components['robot_main']['level'] = DiagnosticStatus.OK
            self.components['robot_main']['message'] = 'Running'
    
    def slam_callback(self, msg):
        """Получить статус SLAM."""
        status = msg.data.upper()
        
        if 'COMPLETE' in status:
            self.components['slam']['level'] = DiagnosticStatus.OK
            self.components['slam']['message'] = 'Scan complete'
        elif 'RUNNING' in status:
            self.components['slam']['level'] = DiagnosticStatus.OK
            self.components['slam']['message'] = 'Scanning'
    
    def publish_diagnostics(self):
        """Опубликовать диагностику."""
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        
        for name, data in self.components.items():
            status = DiagnosticStatus()
            status.name = name
            status.hardware_id = 'ESP32'
            status.level = data['level']
            status.message = data['message']
            
            array.status.append(status)
        
        self.diag_pub.publish(array)

def main(args=None):
    rclpy.init(args=args)
    aggregator = DiagnosticsAggregator()
    rclpy.spin(aggregator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
