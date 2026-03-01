#!/usr/bin/env python3
# ===============================================
# МОДУЛЬ: ROBOT MONITOR
# ===============================================
#
# ЗАДАЧА:
# - Подписывается на /robot_status (String), парсит строки статуса от ESP32.
# - Публикует статус в лог и простой статус.
#
# ВХОДЫ:
# - Топик: /robot_status (std_msgs/String)
#
# ВЫХОДЫ:
# - Топик: /robot_diagnostics (std_msgs/String) - упрощенная диагностика
#

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotMonitor(Node):
    """
    Узел мониторинга здоровья системы.
    Парсит статус от ESP32 и публикует упрощенную диагностику.
    """
    
    def __init__(self):
        super().__init__('robot_monitor')
        
        # Подписчик на статус
        self.status_sub = self.create_subscription(
            String, 'robot_status', self.status_callback, 10)
        
        # Издатель упрощенной диагностики
        self.diag_pub = self.create_publisher(
            String, 'robot_diagnostics', 10)
        
        self.get_logger().info('🔍 Robot Monitor инициализирован')
    
    def status_callback(self, msg):
        """Получить статус и обработать."""
        status_str = msg.data
        
        # Простой парсинг и логирование
        if "STATUS:" in status_str:
            self.get_logger().info(f'📟 Статус: {status_str}')
            
            # Публиковать упрощенную диагностику
            diag_msg = String()
            diag_msg.data = status_str
            self.diag_pub.publish(diag_msg)
        elif "Lidar" in status_str or "Motor" in status_str:
            self.get_logger().debug(f'🔧 {status_str}')
        else:
            # Прокидываем все сообщения дальше
            diag_msg = String()
            diag_msg.data = status_str
            self.diag_pub.publish(diag_msg)

def main(args=None):
    """Точка входа."""
    rclpy.init(args=args)
    monitor = RobotMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Остановка монитора...')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
