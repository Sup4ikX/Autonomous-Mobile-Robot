#!/usr/bin/env python3
# SCAN STATE MACHINE — Wall Following Room Exploration
#
# АЛГОРИТМ:
#   1. INITIAL_SCAN — робот стоит, сканирует, строит первичную карту
#   2. FIND_WALL — найти ближайшую стену
#   3. TURN_TO_WALL — повернуться носом к стене
#   4. APPROACH_WALL — подъехать к стене (целевая дистанция)
#   5. FOLLOW_WALL — ехать вдоль стены, сканируя местность
#   6. CHECK_RETURN — вернулся к началу → завершить
#
# ВХОДЫ:
#   /scan (LaserScan), /map (OccupancyGrid), /scan_command
# ВЫХОДЫ:
#   /cmd_vel_auto (Twist), /robot_state (String)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker
import math
import time
import numpy as np

try:
    from pj_lidar.config_loader import ConfigLoader
    CONFIG_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    CONFIG_AVAILABLE = False


class ScanStateMachine(Node):
    """Wall-following автопилот для исследования замкнутой комнаты."""

    STATE_IDLE = 0
    STATE_INITIAL_SCAN = 1
    STATE_FIND_WALL = 2
    STATE_APPROACH_WALL = 3
    STATE_FOLLOW_WALL = 4
    STATE_CHECK_RETURN = 5
    STATE_TURN_90 = 6
    STATE_TURN_TO_WALL = 7

    def __init__(self):
        super().__init__('scan_state_machine')

        cfg = {}
        if CONFIG_AVAILABLE:
            try:
                ConfigLoader.load()
                cfg = ConfigLoader.get_state_machine_config() or {}
                self.get_logger().info('✓ Параметры ScanStateMachine загружены из config.yaml')
            except Exception as e:
                self.get_logger().warn(f'Ошибка загрузки конфига для ScanStateMachine: {e}')
                cfg = {}

        self.declare_parameter('initial_scan_time', cfg.get('initial_scan_time', 8.0))
        self.declare_parameter('target_wall_distance', cfg.get('target_wall_distance', 0.30))
        self.declare_parameter('wall_follow_distance', cfg.get('wall_follow_distance', 0.35))
        self.declare_parameter('obstacle_distance', cfg.get('obstacle_distance', 0.25))
        self.declare_parameter('max_forward_speed', cfg.get('max_forward_speed', 0.15))
        self.declare_parameter('rotation_speed', cfg.get('rotation_speed', 0.40))
        self.declare_parameter('return_threshold', cfg.get('return_threshold', 0.50))
        self.declare_parameter('enable_auto_mode', cfg.get('enable_auto_mode', False))
        self.declare_parameter('lidar_yaw_offset_deg', cfg.get('lidar_yaw_offset_deg', 90.0))
        self.declare_parameter('turn_distance', cfg.get('turn_distance', 0.30))
        self.declare_parameter('turn_clear_distance', cfg.get('turn_clear_distance', 0.45))
        self.declare_parameter('turn_timeout', cfg.get('turn_timeout', 6.0))
        # Параметры безопасности
        self.declare_parameter('emergency_stop_distance', cfg.get('emergency_stop_distance', 0.30))
        self.declare_parameter('slowdown_margin', cfg.get('slowdown_margin', 0.15))
        self.declare_parameter('front_history_len', cfg.get('front_history_len', 3))

        self.initial_scan_time = self.get_parameter('initial_scan_time').value
        self.target_wall_dist = self.get_parameter('target_wall_distance').value
        self.wall_follow_dist = self.get_parameter('wall_follow_distance').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.max_forward_speed = self.get_parameter('max_forward_speed').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.return_threshold = self.get_parameter('return_threshold').value
        self.auto_mode_enabled = self.get_parameter('enable_auto_mode').value
        self.lidar_yaw_offset = math.radians(float(self.get_parameter('lidar_yaw_offset_deg').value))
        self.turn_distance = float(self.get_parameter('turn_distance').value)
        self.turn_clear_distance = float(self.get_parameter('turn_clear_distance').value)
        self.turn_timeout = float(self.get_parameter('turn_timeout').value)
        self.emergency_stop_dist = float(self.get_parameter('emergency_stop_distance').value)
        self.slowdown_margin = float(self.get_parameter('slowdown_margin').value)
        self.front_history_len = int(self.get_parameter('front_history_len').value)

        # Переменные состояния
        self.current_state = self.STATE_IDLE
        self.scan_data = None
        self.map_data = None
        self.map_info = None
        self.scan_start_time = None
        self.robot_pose = (0.0, 0.0, 0.0)
        self.start_position = None
        self.wall_side = None
        self.closest_wall_angle = None
        self.closest_wall_dist = None
        self.distance_traveled = 0.0
        self.last_pose = None
        self.wall_follow_started = False
        self.return_circle_radius = self.return_threshold
        self.turn_target_yaw = None
        self.turn_start_time = None
        self._cmd_published = False

        # Для INITIAL_SCAN
        self.scan_count_initial = 0
        # Минимальное количество LIDAR сканов, требуемых для завершения начального скана.
        # Делать настраиваемым в config.yaml: параметр 'min_scans_required'.
        self.declare_parameter('min_scans_required', cfg.get('min_scans_required', 3))
        self.min_scans_required = int(self.get_parameter('min_scans_required').value)

        # Для TURN_TO_WALL
        self._turn_to_wall_start = None

        # Для FIND_WALL
        self._find_wall_start = None

        # Для фильтрации фронтального расстояния
        self.front_history = []

        # Подписчики
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.manual_cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.manual_cmd_callback, 10)
        self.scan_cmd_sub = self.create_subscription(String, 'scan_command', self.scan_command_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Издатели
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_auto', 10)
        self.state_pub = self.create_publisher(String, 'robot_state', 10)
        self.marker_pub = self.create_publisher(Marker, 'return_circle_marker', 10)
        self.state_timer = self.create_timer(0.1, self.state_machine_callback)
        self.get_logger().info('✓ ScanStateMachine (Wall Following) инициализирована')

    def _publish_return_circle(self):
        if self.start_position is None:
            return
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'return_circle'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.start_position[0]
        marker.pose.position.y = self.start_position[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.return_circle_radius * 2
        marker.scale.y = self.return_circle_radius * 2
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.lifetime.sec = 0
        self.marker_pub.publish(marker)

    def scan_callback(self, msg):
        self.scan_data = msg
        if self.current_state == self.STATE_INITIAL_SCAN:
            self.scan_count_initial += 1
        self._find_closest_wall()

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def manual_cmd_callback(self, msg):
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            if self.auto_mode_enabled:
                self.get_logger().info('⏸ Авторежим отключен из-за ручной команды')
                self.auto_mode_enabled = False
                self.current_state = self.STATE_IDLE
                self.cmd_vel_pub.publish(Twist())

    def scan_command_callback(self, msg):
        cmd = (msg.data or '').strip().upper()
        if cmd == 'START_SCAN':
            self.start_auto_scan()
        elif cmd == 'STOP_SCAN':
            self.stop_auto_scan()

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        if self.last_pose is not None:
            dx = x - self.last_pose[0]
            dy = y - self.last_pose[1]
            self.distance_traveled += math.sqrt(dx*dx + dy*dy)
        self.last_pose = (x, y, theta)
        self.robot_pose = (x, y, theta)

    def _find_closest_wall(self):
        if self.scan_data is None:
            return
        ranges = self.scan_data.ranges
        n = len(ranges)
        if n == 0:
            return
        valid = [(i, r) for i, r in enumerate(ranges) if math.isfinite(r) and r > 0.05]
        if not valid:
            return
        min_idx, min_dist = min(valid, key=lambda x: x[1])
        angle_min = self.scan_data.angle_min
        angle_inc = self.scan_data.angle_increment
        scan_angle = angle_min + min_idx * angle_inc
        self.closest_wall_angle = self._normalize_angle(scan_angle - self.lidar_yaw_offset)
        self.closest_wall_dist = min_dist

    def _sector_min(self, center_robot_angle, half_width_rad):
        if self.scan_data is None:
            return float('inf')
        ranges = self.scan_data.ranges
        n = len(ranges)
        if n == 0:
            return float('inf')

        angle_min = self.scan_data.angle_min
        angle_inc = self.scan_data.angle_increment
        center_scan = self._normalize_angle(center_robot_angle + self.lidar_yaw_offset)

        vals = []
        for i, r in enumerate(ranges):
            if not (math.isfinite(r) and r > 0.05):
                continue
            a_scan = angle_min + i * angle_inc
            da = self._normalize_angle(a_scan - center_scan)
            if abs(da) <= half_width_rad:
                vals.append(r)

        return min(vals) if vals else float('inf')

    def _get_front_distance(self):
        return self._sector_min(0.0, math.radians(12.0))

    def _get_back_distance(self):
        return self._sector_min(math.pi, math.radians(12.0))

    def _get_wall_distance_left(self):
        return self._sector_min(math.pi / 2.0, math.radians(15.0))

    def _get_wall_distance_right(self):
        return self._sector_min(-math.pi / 2.0, math.radians(15.0))

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _angle_diff(target, current):
        return ScanStateMachine._normalize_angle(target - current)

    def publish_state(self, name):
        msg = String()
        msg.data = name
        self.state_pub.publish(msg)

    def state_machine_callback(self):
        # Отслеживание изменений auto_mode_enabled
        if not hasattr(self, '_last_auto_state'):
            self._last_auto_state = self.auto_mode_enabled
        if self.auto_mode_enabled != self._last_auto_state:
            self.get_logger().warn(f'⚡ auto_mode_enabled изменился: {self._last_auto_state} -> {self.auto_mode_enabled}')
            self._last_auto_state = self.auto_mode_enabled

        # Отладочная печать каждые 0.5 сек (даже если авторежим выключен)
        now = time.time()
        if not hasattr(self, '_last_debug_time') or now - self._last_debug_time > 0.5:
            front = self._get_front_distance()
            back = self._get_back_distance()
            left = self._get_wall_distance_left()
            right = self._get_wall_distance_right()
            self.get_logger().info(
                f"📐 Секторы: front={front:.2f} back={back:.2f} left={left:.2f} right={right:.2f}"
            )
            self._last_debug_time = now

        if not self.auto_mode_enabled:
            self.publish_state('IDLE')
            self.cmd_vel_pub.publish(Twist())
            return

        # Получаем текущее расстояние спереди и применяем фильтр скользящего среднего
        raw_front = self._get_front_distance()
        self.front_history.append(raw_front)
        if len(self.front_history) > self.front_history_len:
            self.front_history.pop(0)
        front_filtered = sum(self.front_history) / len(self.front_history) if self.front_history else raw_front

        # Аварийная остановка с гистерезисом
        if not hasattr(self, '_emergency_count'):
            self._emergency_count = 0
        if raw_front < self.emergency_stop_dist:
            self._emergency_count += 1
            if self._emergency_count >= 5:
                self.get_logger().error(f'🚨 АВАРИЯ! Стена {raw_front:.2f} м в течение нескольких циклов! Остановка!')
                self.cmd_vel_pub.publish(Twist())
                self.auto_mode_enabled = False
                self.current_state = self.STATE_IDLE
                return
        else:
            self._emergency_count = 0

        # Дальше везде используем отфильтрованное значение для плавности
        front = front_filtered

        # Подробная отладочная печать в авторежиме
        now = time.time()
        if not hasattr(self, '_last_debug_time2') or now - self._last_debug_time2 > 0.5:
            back = self._get_back_distance()
            left = self._get_wall_distance_left()
            right = self._get_wall_distance_right()
            self.get_logger().info(
                f"📐 Секторы: front={front:.2f} back={back:.2f} left={left:.2f} right={right:.2f} | "
                f"closest_wall: dist={self.closest_wall_dist} angle={self.closest_wall_angle} | "
                f"state={self.current_state} traveled={self.distance_traveled:.2f}m"
            )
            self._last_debug_time2 = now

        cmd = Twist()

        if self.current_state == self.STATE_IDLE:
            self.publish_state('IDLE')

        elif self.current_state == self.STATE_INITIAL_SCAN:
            elapsed = time.time() - self.scan_start_time if self.scan_start_time else 0
            progress = min(100, int(elapsed / self.initial_scan_time * 100))
            self.publish_state(f'INITIAL_SCAN {progress}%')
            cmd.angular.z = self.rotation_speed * 0.8

            if elapsed >= self.initial_scan_time:
                # Завершаем начальный скан по времени, даже если сканов получилось мало.
                # старые проверки приводили к зацикливанию, поскольку scan_count_initial
                # мог оставаться ниже порога (например, при плохих данных).
                if self.scan_count_initial < self.min_scans_required:
                    # предупреждение выводим, чтобы в логах видеть причину
                    self.get_logger().warn(
                        f'⚠️ Начальный скан занял {elapsed:.1f}s, но получено только '
                        f'{self.scan_count_initial} сканов (требовалось {self.min_scans_required}).'
                        ' Переходим к следующему этапу.'
                    )
                else:
                    self.get_logger().info(f'✓ Начальный скан завершён (получено {self.scan_count_initial} сканов)')

                self.current_state = self.STATE_FIND_WALL
                # Сброс счётчика, он больше не нужен
                self.scan_count_initial = 0
            self.cmd_vel_pub.publish(cmd)

        elif self.current_state == self.STATE_FIND_WALL:
            self.publish_state('FIND_WALL')
            if self._find_wall_start is None:
                self._find_wall_start = time.time()
            # Если долго не можем найти стену, снижаем скорость вращения
            if time.time() - self._find_wall_start > 10.0:
                self.get_logger().warn('⚠️ Долго не можем найти стену, снижаем скорость вращения')
                self._find_wall_start = time.time()  # сброс
            if self.closest_wall_dist is None:
                cmd.angular.z = self.rotation_speed * 0.3
                self.cmd_vel_pub.publish(cmd)
                return
            left_dist = self._get_wall_distance_left()
            right_dist = self._get_wall_distance_right()
            self.wall_side = 'left' if left_dist < right_dist else 'right'
            self.get_logger().info(f'🎯 Стена: {self.closest_wall_dist:.2f}м, сторона: {self.wall_side}')
            self.start_position = self.robot_pose
            self.distance_traveled = 0.0
            self.wall_follow_started = False
            self.current_state = self.STATE_TURN_TO_WALL
            self._find_wall_start = None

        elif self.current_state == self.STATE_TURN_TO_WALL:
            self.publish_state('TURN_TO_WALL')

            if self._turn_to_wall_start is None:
                self._turn_to_wall_start = time.time()

            if time.time() - self._turn_to_wall_start > 8.0:
                self.get_logger().warn('⚠️ TURN_TO_WALL timeout — принудительный переход к APPROACH_WALL')
                self._turn_to_wall_start = None
                self.current_state = self.STATE_APPROACH_WALL
                self.cmd_vel_pub.publish(Twist())
                return

            current_angle = self.closest_wall_angle
            if current_angle is None:
                cmd.angular.z = self.rotation_speed * 0.5
                self.cmd_vel_pub.publish(cmd)
                return

            if current_angle > math.pi:
                current_angle -= 2*math.pi
            elif current_angle < -math.pi:
                current_angle += 2*math.pi

            self.get_logger().info(f"🔄 TURN_TO_WALL: angle={current_angle:.3f} rad")

            if abs(current_angle) < 0.2:
                self.get_logger().info('✓ Поворот к стене завершён')
                self._turn_to_wall_start = None
                self.current_state = self.STATE_APPROACH_WALL
                self.cmd_vel_pub.publish(Twist())
                return

            direction = 1.0 if current_angle > 0 else -1.0
            cmd.angular.z = direction * self.rotation_speed
            cmd.linear.x = 0.0
            self.cmd_vel_pub.publish(cmd)

        elif self.current_state == self.STATE_APPROACH_WALL:
            front_dist = front
            self.publish_state(f'APPROACH_WALL front={front_dist:.2f}m')

            # Если стена потеряна (inf или очень далеко)
            if not math.isfinite(front_dist) or front_dist > 2.5:
                self.get_logger().warn('⚠️ Стена потеряна, пытаемся её найти, медленно двигаясь и поворачивая')
                # Медленно едем и поворачиваем в сторону, где последний раз была стена
                if self.closest_wall_angle is not None:
                    direction = 1.0 if self.closest_wall_angle > 0 else -1.0
                    cmd.angular.z = direction * self.rotation_speed * 0.3
                else:
                    cmd.angular.z = self.rotation_speed * 0.2
                cmd.linear.x = self.max_forward_speed * 0.2
                self.cmd_vel_pub.publish(cmd)
                return

            target = self.target_wall_dist
            # Если уже достигли целевой дистанции (с допуском)
            if front_dist <= target + 0.1:
                self.get_logger().info(f'✓ Достигнута дистанция спереди: {front_dist:.2f}м')
                self.wall_follow_started = True
                self.distance_traveled = 0.0
                self.start_position = self.robot_pose
                self._publish_return_circle()
                self.get_logger().info(f'🎯 Создана окружность возврата (диаметр {self.return_circle_radius*2:.1f}м)')
                self.current_state = self.STATE_FOLLOW_WALL
                self.cmd_vel_pub.publish(Twist())
                return

            # Иначе двигаемся вперёд с очень малой скоростью и лёгкой коррекцией курса
            cmd.linear.x = self.max_forward_speed * 0.5
            if self.closest_wall_angle is not None and abs(self.closest_wall_angle) > 0.1:
                cmd.angular.z = self.rotation_speed * 0.2 * (1.0 if self.closest_wall_angle > 0 else -1.0)
            self.cmd_vel_pub.publish(cmd)

        elif self.current_state == self.STATE_FOLLOW_WALL:
            self.publish_state(f'FOLLOW_WALL traveled={self.distance_traveled:.1f}m')

            front_dist = front
            side_dist = self._get_wall_distance_left() if self.wall_side == 'left' else self._get_wall_distance_right()
            self._publish_return_circle()

            # Проверка возвращения в окружность
            if self.distance_traveled > 2.0 and self.wall_follow_started and self.start_position is not None:
                dx = self.robot_pose[0] - self.start_position[0]
                dy = self.robot_pose[1] - self.start_position[1]
                dist_to_start = math.sqrt(dx*dx + dy*dy)
                if dist_to_start < self.return_circle_radius:
                    self.get_logger().info(f'🎯 Вернулись к началу (дистанция {dist_to_start:.2f}м)')
                    self.current_state = self.STATE_CHECK_RETURN
                    self.cmd_vel_pub.publish(Twist())
                    return

            # Если стена спереди слишком близко — поворачиваем
            if front_dist < self.turn_distance:
                self.get_logger().info(f'⬛ Стена впереди: {front_dist:.2f}м < {self.turn_distance:.2f}м → поворот')
                # защита от бесконечных чередующихся поворотов
                self._turn_90_attempts += 1
                if self._turn_90_attempts >= 3:
                    self.get_logger().warn('⚠️ Слишком много поворотов подряд — считаю обход завершённым')
                    self.current_state = self.STATE_CHECK_RETURN
                    self.cmd_vel_pub.publish(Twist())
                    return
                self.current_state = self.STATE_TURN_90
                self.turn_start_time = time.time()
                self.cmd_vel_pub.publish(Twist())
                return

            # Усиленное торможение при приближении к стене
            if front_dist < self.obstacle_distance + self.slowdown_margin:
                cmd.linear.x = 0.02
            else:
                denom = self.turn_distance - self.obstacle_distance
                if denom <= 0:
                    denom = 0.01
                speed_factor = max(0.0, min(1.0, (front_dist - self.obstacle_distance) / denom))
                cmd.linear.x = self.max_forward_speed * max(0.1, speed_factor)

            # Коррекция по боковой стене
            if math.isfinite(side_dist):
                error = side_dist - self.wall_follow_dist
                k_p = 1.2
                if self.wall_side == 'right':
                    cmd.angular.z = -k_p * error
                else:
                    cmd.angular.z = k_p * error
            else:
                # Если стена потеряна, едем прямо, но очень медленно, слегка подруливая в сторону предполагаемой стены
                self.get_logger().warn('⚠️ Потеряна боковая стена, ищем её...')
                cmd.linear.x = self.max_forward_speed * 0.3
                if self.wall_side == 'left':
                    cmd.angular.z = 0.1
                else:
                    cmd.angular.z = -0.1

            self.cmd_vel_pub.publish(cmd)

        elif self.current_state == self.STATE_TURN_90:
            # сбрасываем попытки, мы уже в процессе поворота
            self._turn_90_attempts = 0
            self.publish_state('TURN_90')

            front_dist = self._get_front_distance()
            # диагностическое логирование расстояния перед роботом
            self.get_logger().debug(f'📐 TURN_90 front_dist={front_dist:.2f}м')
            if self.turn_start_time is not None and (time.time() - self.turn_start_time) > self.turn_timeout:
                self.get_logger().warn('⚠️ TURN_90 timeout — выходим в FOLLOW_WALL')
                self.turn_start_time = None
                self.current_state = self.STATE_FOLLOW_WALL
                self.cmd_vel_pub.publish(Twist())
                return

            if front_dist > self.turn_clear_distance:
                self.get_logger().info(f'✓ Поворот завершён: front={front_dist:.2f}м')
                self.turn_start_time = None
                self.current_state = self.STATE_FOLLOW_WALL
                self.cmd_vel_pub.publish(Twist())
                return

            direction = -1.0 if self.wall_side == 'left' else 1.0
            cmd.angular.z = direction * self.rotation_speed
            cmd.linear.x = 0.0
            self.cmd_vel_pub.publish(cmd)

        elif self.current_state == self.STATE_CHECK_RETURN:
            # любая проверка возврата очищает счётчик поворотов
            self._turn_90_attempts = 0
            self.publish_state('CHECK_RETURN')
            self.cmd_vel_pub.publish(Twist())
            dx = self.robot_pose[0] - self.start_position[0]
            dy = self.robot_pose[1] - self.start_position[1]
            dist_to_start = math.sqrt(dx*dx + dy*dy)
            self.get_logger().info(f'📏 Расстояние до центра окружности: {dist_to_start:.2f}м (радиус: {self.return_circle_radius:.2f}м)')
            if dist_to_start < self.return_circle_radius:
                self.get_logger().info('✓✓✓ АВТОСКАНИРОВАНИЕ КОМНАТЫ ЗАВЕРШЕНО! ✓✓✓')
                self.stop_auto_scan()
                self.cmd_vel_pub.publish(Twist())
                return
            else:
                self.current_state = self.STATE_FOLLOW_WALL

        # Публикация команды, если ещё не опубликована
        if not hasattr(self, '_cmd_published') or not self._cmd_published:
            self.cmd_vel_pub.publish(cmd)
        self._cmd_published = False

    def start_auto_scan(self):
        self.auto_mode_enabled = True
        self.current_state = self.STATE_INITIAL_SCAN
        self.scan_start_time = time.time()
        self.distance_traveled = 0.0
        self.start_position = None
        self.wall_follow_started = False
        self.last_pose = None
        # очистка счётчика поворотов на случай повторного запуска
        self._turn_90_attempts = 0
        self.get_logger().info('▶️ АВТОСКАНИРОВАНИЕ ЗАПУЩЕНО!')
        self.get_logger().info('  1. Начальный скан')
        self.get_logger().info('  2. Поиск стены')
        self.get_logger().info('  3. Поворот к стене')
        self.get_logger().info('  4. Подъезд к стене')
        self.get_logger().info('  5. Движение вдоль стены')
        self.get_logger().info('  6. Завершение при возвращении')

    def stop_auto_scan(self):
        self.get_logger().info('⏹ Автосканирование остановлено по команде')
        self.auto_mode_enabled = False
        self.current_state = self.STATE_IDLE
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('⏹ АВТОСКАНИРОВАНИЕ ОСТАНОВЛЕНО')

    def destroy_node(self):
        try:
            if self.auto_mode_enabled:
                self.auto_mode_enabled = False
                self.cmd_vel_pub.publish(Twist())
                time.sleep(0.1)
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScanStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Остановка...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()