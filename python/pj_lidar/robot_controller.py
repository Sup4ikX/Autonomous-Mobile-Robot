# ===============================================
# КОНТРОЛЛЕР ДЛЯ УПРАВЛЕНИЯ РОБОТОМ
# ===============================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class RobotController(Node):
    """Узел для отправки команд роботу."""
    
    def __init__(self):
        super().__init__('robot_controller')
        
        # Издатель команд движения
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Подписчик на статус
        self.status_sub = self.create_subscription(
            String, 'robot_status', self.status_callback, 10)
        
        self.status = ""
    
    def status_callback(self, msg):
        """Получить статус робота."""
        self.status = msg.data
        self.get_logger().info(f"Статус: {self.status}")
    
    # 👇👇👇 ОТПРАВЛЯЙ КОМАНДЫ ЗДЕСЬ 👇👇👇
    
    def move_forward(self, speed=0.5):
        """Движение вперед."""
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Движение вперед: {speed} м/с")
    
    def move_backward(self, speed=0.5):
        """Движение назад."""
        msg = Twist()
        msg.linear.x = -speed
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Движение назад: {speed} м/с")
    
    def rotate_left(self, angular_speed=0.5):
        """Вращение влево."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular_speed
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Вращение влево: {angular_speed} рад/с")
    
    def rotate_right(self, angular_speed=0.5):
        """Вращение вправо."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -angular_speed
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Вращение вправо: {angular_speed} рад/с")
    
    def stop(self):
        """Остановка."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)
        self.get_logger().info("Остановка")
    
    def move_in_circle(self, radius=1.0, linear_speed=0.3):
        """Движение по кругу."""
        # Вычисление угловой скорости: omega = v / r
        angular_speed = linear_speed / radius
        
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Движение по кругу, радиус={radius}м")
    
    def move_in_square(self, side_length=1.0, speed=0.3):
        """Движение по квадрату."""
        self.get_logger().info(f"Начинаю движение по квадрату, сторона={side_length}м")
        
        for i in range(4):
            # Движение вперед
            self.move_forward(speed)
            time.sleep(side_length / speed)  # Время в пути
            
            # Поворот на 90 градусов
            self.rotate_left(0.5)
            time.sleep(3.14 / 2 / 0.5)  # 90 градусов = π/2 радиан
        
        self.stop()
        self.get_logger().info("Квадрат завершен!")
    
    # 👆👆👆 КОНЕЦ КОМАНД 👆👆👆

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    
    # Примеры использования:
    time.sleep(1)
    
    # Движение вперед
    controller.move_forward(0.5)
    time.sleep(2)
    
    # Вращение влево
    controller.rotate_left(0.3)
    time.sleep(2)
    
    # Остановка
    controller.stop()
    time.sleep(1)
    
    # Движение по кругу
    controller.move_in_circle(1.0, 0.3)
    time.sleep(10)
    
    # Движение по квадрату
    controller.move_in_square(1.0, 0.3)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
