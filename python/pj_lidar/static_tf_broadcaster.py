#!/usr/bin/env python3
# STATIC TF BROADCASTER
# ЗАДАЧА:
# - Публиковать статические TF трансформы для работы SLAM без реального движения робота.
# - Это позволяет строить карту когда робот стоит на месте (LiDAR вращается).
# ИСПОЛЬЗОВАНИЕ:
# - ros2 run pj_lidar static_tf_broadcaster.py

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class StaticTFBroadcaster(Node):
    """Публикует статические TF трансформы для SLAM."""

    def __init__(self):
        super().__init__('static_tf_broadcaster')

        # Используем StaticTransformBroadcaster для статических трансформов
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Параметры трансформов
        self.declare_parameter('laser_height', 0.1)  # Высота LiDAR от земли
        self.declare_parameter('publish_rate', 10.0)  # Частота публикации

        self.laser_height = self.get_parameter('laser_height').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Таймер для периодической публикации
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_transforms)

        self.get_logger().info('Static TF Broadcaster инициализирован')
        self.get_logger().info(f'Высота LiDAR: {self.laser_height} м')

    def create_static_transform(self, parent_frame, child_frame, x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1):
        """Создать статический трансформ."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        return t

    def publish_transforms(self):
        """Публиковать трансформы.

        ВАЖНО:
        - map → odom публикует SLAM Toolbox
        - odom → base_link публикует wheel_odometry.py
        - base_link → laser публикует wheel_odometry.py (static)

        Этот узел НЕ публикует TF, чтобы избежать конфликтов!
        Он оставлен только для совместимости - TF публикуется
        исключительно из wheel_odometry.py

        Если wheel_odometry не используется - раскомментируйте блоки ниже.
        """
        # ВСЕ TF ОТКЛЮЧЕНЫ - публикуются из wheel_odometry.py
        # для избежания конфликтов трансформов
        pass

        # # odom → base_link (fallback — только если НЕТ wheel_odometry)
        # t2 = TransformStamped()
        # t2.header.stamp = self.get_clock().now().to_msg()
        # t2.header.frame_id = 'odom'
        # t2.child_frame_id = 'base_link'
        # t2.transform.translation.x = 0.0
        # t2.transform.translation.y = 0.0
        # t2.transform.translation.z = 0.0
        # t2.transform.rotation.x = 0.0
        # t2.transform.rotation.y = 0.0
        # t2.transform.rotation.z = 0.0
        # t2.transform.rotation.w = 1.0
        # transforms.append(t2)

        # # base_link → laser (позиция лидара относительно центра робота)
        transforms = []
        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'base_link'
        t3.child_frame_id = 'laser'
        t3.transform.translation.x = 0.0
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = self.laser_height
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0
        transforms.append(t3)

        self.tf_broadcaster.sendTransform(transforms)

def main(args=None):
    rclpy.init(args=args)
    broadcaster = StaticTFBroadcaster()

    try:
        rclpy.spin(broadcaster)
    except KeyboardInterrupt:
        broadcaster.get_logger().info('Остановка...')
    finally:
        broadcaster.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()