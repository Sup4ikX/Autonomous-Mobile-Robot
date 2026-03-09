import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
import tf_transformations
import tf2_ros
import math  # обязательно для cos/sin

class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_diameter', 0.051),
                ('wheel_base', 0.135),
                ('ticks_per_rev', 4),
                ('left_ticks_topic', '/left_ticks'),
                ('right_ticks_topic', '/right_ticks'),
                ('laser_frame', 'laser'),
                ('laser_z', 0.06),
                ('publish_rate', 50.0),
            ]
        )
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.left_ticks_topic = self.get_parameter('left_ticks_topic').value
        self.right_ticks_topic = self.get_parameter('right_ticks_topic').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.laser_z = self.get_parameter('laser_z').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Подписка на Int32 (как публикует esp32_tcp_client)
        self.create_subscription(Int32, self.left_ticks_topic, self.left_ticks_callback, 10)
        self.create_subscription(Int32, self.right_ticks_topic, self.right_ticks_callback, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.update_odometry)

    def left_ticks_callback(self, msg):
        self.left_ticks = msg.data

    def right_ticks_callback(self, msg):
        self.right_ticks = msg.data

    def update_odometry(self):
        d_left = (self.left_ticks - self.last_left_ticks) * (self.wheel_diameter * 3.141592653589793 / self.ticks_per_rev)
        d_right = (self.right_ticks - self.last_right_ticks) * (self.wheel_diameter * 3.141592653589793 / self.ticks_per_rev)
        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks
        d = (d_left + d_right) / 2.0
        th = (d_right - d_left) / self.wheel_base
        # Используем math.cos и math.sin
        self.x += d * math.cos(self.th + th / 2.0)
        self.y += d * math.sin(self.th + th / 2.0)
        self.th += th

        # Публикация одометрии (odom -> base_link)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'          # важно для SLAM
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.th)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom_msg)

        # Публикация трансформации odom -> base_link
        t_odom = TransformStamped()
        t_odom.header.stamp = odom_msg.header.stamp
        t_odom.header.frame_id = 'odom'
        t_odom.child_frame_id = 'base_link'
        t_odom.transform.translation.x = self.x
        t_odom.transform.translation.y = self.y
        t_odom.transform.translation.z = 0.0
        t_odom.transform.rotation.x = q[0]
        t_odom.transform.rotation.y = q[1]
        t_odom.transform.rotation.z = q[2]
        t_odom.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t_odom)

        # Публикация статической трансформации base_link -> laser_frame
        t_laser = TransformStamped()
        t_laser.header.stamp = odom_msg.header.stamp
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = self.laser_frame
        t_laser.transform.translation.x = 0.0
        t_laser.transform.translation.y = 0.0
        t_laser.transform.translation.z = self.laser_z
        t_laser.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_laser)

def main():
    rclpy.init()
    node = WheelOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()