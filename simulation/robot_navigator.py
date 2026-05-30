#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist          # ← plain Twist, not TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import numpy as np
import tf_transformations

# ─── Waypoints: (x, y, heading_rad) ───────────────────────────────────────────
WAYPOINTS = [
    (0.0,  0.0,   0.0),
    (1.5,  0.0,   math.pi / 2),
    (1.5,  1.5,   math.pi),
    (0.0,  1.5,  -math.pi / 2),
]
WAYPOINT_LABELS = ['A (start)', 'B', 'C', 'D']

# ─── Tuning constants ──────────────────────────────────────────────────────────
SAFE_DISTANCE      = 0.5
MIN_VALID_RANGE    = 0.38
STATIC_TIMEOUT     = 5.0
WAYPOINT_STOP_TIME = 10.0
LINEAR_SPEED       = 0.15
ANGULAR_SPEED      = 0.4
DIST_TOLERANCE     = 0.20
FINAL_APPROACH     = 0.40
ANGLE_TOLERANCE    = 0.15
OBSTACLE_CONFIRM   = 5


class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')

        # ── Publisher: plain Twist to match use_stamped_vel: false ────────────
        self.cmd_pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel', 10)

        self.create_subscription(LaserScan, '/scan',       self.scan_left_cb,  10)
        self.create_subscription(LaserScan, '/scan_right', self.scan_right_cb, 10)
        self.create_subscription(Imu,       '/imu',        self.imu_cb,        10)
        self.create_subscription(Odometry,  '/diff_drive_controller/odom',
                                 self.odom_cb, 10)

        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.current_wp      = 1
        self.nav_state       = 'rotate_to_target'
        self.stop_start_time = None

        self.obstacle_ahead         = False
        self.obstacle_first_seen    = None
        self.obstacle_last_range    = None
        self.obstacle_is_static     = False
        self.obstacle_confirm_count = 0

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('RobotNavigator started')
        self.get_logger().info(
            f'First target: {WAYPOINT_LABELS[self.current_wp]} '
            f'at {WAYPOINTS[self.current_wp]}')

    # ══════════════════════════════════════════════════════════════════════════
    # Sensor callbacks
    # ══════════════════════════════════════════════════════════════════════════

    def _check_scan(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0)
        if len(ranges) == 0:
            return False, 10.0
        front = np.concatenate((ranges[:30], ranges[-30:]))
        front = front[front > MIN_VALID_RANGE]
        if len(front) == 0:
            return False, 10.0
        min_r = float(np.min(front))
        return min_r < SAFE_DISTANCE, min_r

    def scan_left_cb(self, msg):
        self._update_obstacle(*self._check_scan(msg))

    def scan_right_cb(self, msg):
        self._update_obstacle(*self._check_scan(msg))

    def _update_obstacle(self, detected, dist):
        now = self.get_clock().now().nanoseconds / 1e9
        if detected:
            self.obstacle_confirm_count += 1
            if self.obstacle_confirm_count < OBSTACLE_CONFIRM:
                return
            if self.obstacle_first_seen is None:
                self.obstacle_first_seen = now
                self.obstacle_last_range = dist
                self.get_logger().warn(f'Obstacle confirmed at {dist:.2f} m')
            age          = now - self.obstacle_first_seen
            range_change = abs(dist - self.obstacle_last_range)
            self.obstacle_last_range = dist
            if age >= STATIC_TIMEOUT and range_change < 0.05:
                if not self.obstacle_is_static:
                    self.get_logger().warn('Obstacle is STATIC — will manoeuvre')
                self.obstacle_is_static = True
            else:
                self.obstacle_is_static = False
            self.obstacle_ahead = True
        else:
            self.obstacle_confirm_count = 0
            if self.obstacle_ahead:
                self.get_logger().info('Obstacle cleared — resuming path')
            self.obstacle_ahead      = False
            self.obstacle_first_seen = None
            self.obstacle_last_range = None
            self.obstacle_is_static  = False

    def imu_cb(self, msg):
        q = msg.orientation
        _, _, self.yaw = tf_transformations.euler_from_quaternion(
            (q.x, q.y, q.z, q.w))

    def odom_cb(self, msg):
        # Read position directly from odometry
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    # ══════════════════════════════════════════════════════════════════════════
    # Helpers
    # ══════════════════════════════════════════════════════════════════════════

    @staticmethod
    def normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    def publish(self, linear=0.0, angular=0.0):
        msg = Twist()                        # ← plain Twist
        msg.linear.x  = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop(self):
        self.publish(0.0, 0.0)

    # ══════════════════════════════════════════════════════════════════════════
    # Main control loop (20 Hz)
    # ══════════════════════════════════════════════════════════════════════════

    def control_loop(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9

        # ── Obstacle handling (overrides navigation except at waypoint stop) ──
        if self.obstacle_ahead and self.nav_state != 'waypoint_stop':
            if self.obstacle_is_static:
                self.nav_state = 'avoid_static'
                self.get_logger().info('Static obstacle — manoeuvring around',
                                       throttle_duration_sec=2.0)
                self.publish(linear=0.1, angular=0.6)
            else:
                self.nav_state = 'wait_dynamic'
                self.get_logger().info('Dynamic obstacle — waiting...',
                                       throttle_duration_sec=2.0)
                self.stop()
            return

        # ── Compute errors fresh every tick ───────────────────────────────────
        tx, ty, t_yaw = WAYPOINTS[self.current_wp]
        dx       = tx - self.x
        dy       = ty - self.y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_error     = self.normalize_angle(angle_to_target - self.yaw)
        final_yaw_error = self.normalize_angle(t_yaw - self.yaw)

        # ── State machine ──────────────────────────────────────────────────────

        if self.nav_state == 'rotate_to_target':
            if distance < DIST_TOLERANCE:
                self.stop()
                self.nav_state = 'rotate_to_final'
                self.get_logger().info(
                    f'At {WAYPOINT_LABELS[self.current_wp]} — aligning heading')
                return
            if abs(angle_error) > ANGLE_TOLERANCE:
                angular = max(-ANGULAR_SPEED,
                              min(ANGULAR_SPEED, -0.8 * angle_error))
                self.publish(angular=angular)
            else:
                self.nav_state = 'move_forward'
                self.get_logger().info(
                    f'Moving toward {WAYPOINT_LABELS[self.current_wp]}')

        elif self.nav_state == 'move_forward':
            if distance <= DIST_TOLERANCE:
                self.stop()
                self.nav_state = 'rotate_to_final'
                self.get_logger().info(
                    f'Reached {WAYPOINT_LABELS[self.current_wp]} '
                    f'at dist={distance:.3f}m — aligning to final heading')
            elif distance < FINAL_APPROACH:
                self.publish(linear=max(0.05, distance * 0.3), angular=0.0)
            else:
                speed = min(LINEAR_SPEED, distance * 0.4)
                steer = max(-0.3, min(0.3, -0.8 * angle_error))
                self.publish(linear=speed, angular=steer)

        elif self.nav_state == 'rotate_to_final':
            if abs(final_yaw_error) > ANGLE_TOLERANCE:
                angular = max(-ANGULAR_SPEED,
                              min(ANGULAR_SPEED, -0.8 * final_yaw_error))
                self.publish(angular=angular)
            else:
                self.stop()
                self.nav_state       = 'waypoint_stop'
                self.stop_start_time = now_sec
                self.get_logger().info(
                    f'Arrived at {WAYPOINT_LABELS[self.current_wp]} '
                    f'— stopping for {WAYPOINT_STOP_TIME}s')

        elif self.nav_state == 'waypoint_stop':
            self.stop()
            remaining = WAYPOINT_STOP_TIME - (now_sec - self.stop_start_time)
            if remaining <= 0:
                self.current_wp = (self.current_wp + 1) % len(WAYPOINTS)
                self.nav_state  = 'rotate_to_target'
                self.get_logger().info(
                    f'Departing toward {WAYPOINT_LABELS[self.current_wp]}')
            else:
                self.get_logger().info(
                    f'Waiting at {WAYPOINT_LABELS[self.current_wp]}... '
                    f'{remaining:.1f}s remaining',
                    throttle_duration_sec=2.0)

        elif self.nav_state == 'avoid_static':
            self.publish(linear=0.1, angular=0.6)

        elif self.nav_state == 'wait_dynamic':
            self.stop()

        # ── Status log ────────────────────────────────────────────────────────
        self.get_logger().info(
            f'[{self.nav_state}] pos=({self.x:.2f}, {self.y:.2f}) '
            f'yaw={math.degrees(self.yaw):.1f}° '
            f'wp={WAYPOINT_LABELS[self.current_wp]} dist={distance:.2f}m',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
