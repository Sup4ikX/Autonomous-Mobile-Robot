#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String  # NEW
import math
import numpy as np
from scipy import ndimage
from tf2_ros import Buffer, TransformListener

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self.marker_pub = self.create_publisher(Marker, '/frontier_marker', 10)
        self.frontier_markers_pub = self.create_publisher(MarkerArray, '/frontier_clusters', 10)
        # NEW: подписка на команды
        self.cmd_sub = self.create_subscription(String, '/scan_command', self.cmd_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.max_angular_speed = 0.3   # рад/с, ограничение угловой скорости
        self.map = None
        self.map_info = None
        self.robot_pose_map = None
        self.robot_yaw = 0.0
        self.current_target = None
        self.exploration_active = False  # NEW: изначально выключено

        # Параметры
        self.max_speed = 0.05
        self.kp_angle = 0.3
        self.goal_tolerance = 0.3
        self.min_frontier_size = 5
        self.search_range = 10.0
        self.exploration_rate = 1.0

        self.timer = self.create_timer(1.0 / self.exploration_rate, self.update_exploration)

    # NEW: обработчик команд
    def cmd_callback(self, msg):
        cmd = msg.data.strip().upper()
        if cmd == 'START_SCAN':
            if not self.exploration_active:
                self.exploration_active = True
                self.get_logger().info('Исследование запущено')
        elif cmd == 'STOP_SCAN':
            if self.exploration_active:
                self.exploration_active = False
                self.current_target = None
                self.cmd_pub.publish(Twist())
                self.get_logger().info('Исследование остановлено')
        else:
            self.get_logger().warn(f'Неизвестная команда: {cmd}')

    # остальные методы без изменений
    def odom_callback(self, msg):
        pass

    def map_callback(self, msg):
        self.map = msg
        self.map_info = msg.info

    def get_robot_pose_in_map(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (x, y), yaw
        except Exception as e:
            self.get_logger().warn(f"TF error: {e}")
            return None, None

    def find_frontiers(self):
        if self.map is None:
            return []
        data = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        unknown = (data == -1).astype(np.uint8)
        free = (data == 0).astype(np.uint8)
        kernel = np.ones((3,3), dtype=np.uint8)
        free_dilated = ndimage.binary_dilation(free, kernel).astype(np.uint8)
        frontier_cells = (unknown * free_dilated).astype(np.uint8)
        labeled, num_features = ndimage.label(frontier_cells)
        clusters = []
        for i in range(1, num_features+1):
            cluster_indices = np.argwhere(labeled == i)
            if len(cluster_indices) < self.min_frontier_size:
                continue
            world_coords = []
            for idx in cluster_indices:
                r, c = idx
                x = self.map.info.origin.position.x + c * self.map.info.resolution
                y = self.map.info.origin.position.y + r * self.map.info.resolution
                world_coords.append((x, y))
            clusters.append(world_coords)
        return clusters

    def select_target(self, clusters):
        if not clusters:
            return None
        robot_pose_map, _ = self.get_robot_pose_in_map()
        if robot_pose_map is None:
            return None
        best_cluster = None
        best_cost = float('inf')
        for cluster in clusters:
            center_x = sum(p[0] for p in cluster) / len(cluster)
            center_y = sum(p[1] for p in cluster) / len(cluster)
            dist = math.hypot(center_x - robot_pose_map[0], center_y - robot_pose_map[1])
            size = len(cluster)
            cost = dist - 0.1 * size
            if cost < best_cost:
                best_cost = cost
                best_cluster = (center_x, center_y, cluster)
        return best_cluster

    def publish_markers(self, clusters, target):
        marker_array = MarkerArray()
        for i, cluster in enumerate(clusters):
            center_x = sum(p[0] for p in cluster) / len(cluster)
            center_y = sum(p[1] for p in cluster) / len(cluster)
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = 0.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker_array.markers.append(marker)
        if target:
            target_marker = Marker()
            target_marker.header.frame_id = 'map'
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.ns = 'target'
            target_marker.id = 0
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = target[0]
            target_marker.pose.position.y = target[1]
            target_marker.pose.position.z = 0.0
            target_marker.scale.x = 0.4
            target_marker.scale.y = 0.4
            target_marker.scale.z = 0.4
            target_marker.color.a = 1.0
            target_marker.color.r = 1.0
            target_marker.color.g = 0.0
            target_marker.color.b = 0.0
            marker_array.markers.append(target_marker)
        self.frontier_markers_pub.publish(marker_array)

    def drive_to_target(self, target):
        robot_pose_map, robot_yaw = self.get_robot_pose_in_map()
        if robot_pose_map is None:
            return
        dx = target[0] - robot_pose_map[0]
        dy = target[1] - robot_pose_map[1]
        dist = math.hypot(dx, dy)
        if dist < self.goal_tolerance:
            self.get_logger().info('Цель достигнута')
            self.current_target = None
            self.cmd_pub.publish(Twist())
            return
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - robot_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        twist = Twist()
        if abs(angle_error) > 0.1:
            # ограничиваем угловую скорость
            desired_angular = self.kp_angle * angle_error
            twist.angular.z = max(min(desired_angular, self.max_angular_speed), -self.max_angular_speed)
            twist.linear.x = 0.0
        else:
            twist.linear.x = min(self.max_speed, dist)
            twist.angular.z = 0.0
    def update_exploration(self):
        if not self.exploration_active:
            return
        clusters = self.find_frontiers()
        if not clusters:
            self.get_logger().info('Исследование завершено: границ не найдено')
            self.exploration_active = False
            self.cmd_pub.publish(Twist())
            return
        target_info = self.select_target(clusters)
        if target_info is None:
            return
        target_x, target_y, cluster = target_info
        self.current_target = (target_x, target_y)
        self.publish_markers(clusters, self.current_target)
        self.drive_to_target(self.current_target)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Остановка...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()