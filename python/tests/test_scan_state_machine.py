import pytest
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import rclpy

# Mock для ROS2
class MockNode:
    def get_logger(self):
        return MockLogger()
    
    def get_clock(self):
        return MockClock()
    
    def create_subscription(self, *args, **kwargs):
        return None
    
    def create_publisher(self, *args, **kwargs):
        return MockPublisher()
    
    def create_timer(self, interval, callback):
        return None

class MockLogger:
    def info(self, msg):
        print(f"[INFO] {msg}")
    def warn(self, msg):
        print(f"[WARN] {msg}")

class MockClock:
    def now(self):
        return None

class MockPublisher:
    def publish(self, msg):
        pass

class TestScanStateMachine:
    """Тесты для State Machine."""
    
    def test_state_transitions(self):
        """Тест переходов состояний."""
        from pj_lidar.scan_state_machine import ScanStateMachine
        
        # Создать ноду (mock)
        rclpy.init() if not rclpy.ok() else None
        machine = ScanStateMachine()
        
        # Изначально в IDLE
        assert machine.current_state == machine.STATE_IDLE
        
        # Переход в ROTATING при start_auto_scan
        machine.start_auto_scan()
        assert machine.current_state == machine.STATE_ROTATING
        assert machine.auto_mode_enabled == True
        
        # Остановка
        machine.stop_auto_scan()
        assert machine.current_state == machine.STATE_IDLE
        assert machine.auto_mode_enabled == False
    
    def test_obstacle_detection(self):
        """Тест обнаружения препятствий."""
        from pj_lidar.scan_state_machine import ScanStateMachine
        
        rclpy.init() if not rclpy.ok() else None
        machine = ScanStateMachine()
        machine.obstacle_distance = 0.5
        
        # Создать LaserScan с близким препятствием
        scan = LaserScan()
        scan.ranges = [0.3] * 360  # Все точки 30см
        
        machine.scan_data = scan
        machine.check_obstacles()
        
        # Должно обнаружить препятствие
        assert machine.current_state == machine.STATE_OBSTACLE_DETECTED
