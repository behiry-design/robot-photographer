#!/usr/bin/env python3
"""
Unified Robot Controller
========================
Combines waypoint following with intelligent obstacle handling:
- Follows 4 waypoints (A -> B -> C -> D) in a smooth 2x2m path
- Stops 10 seconds at each waypoint
- If obstacle is detected:
    * Static for >= 5s  -> manoeuvre around it and resume path
    * Moving / clears   -> wait for it to clear then resume
- Uses /scan (left) and /scan_right (right) for obstacle detection
- Publishes TwistStamped to /diff_drive_controller/cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from rosgraph_msgs.msg import Clock
import math
import numpy as np
import tf_transformations
from enum import Enum, auto


# ─────────────────────────────────────────────
#  Waypoints  (x, y, heading_rad)
#  Smooth rectangular loop, corners rounded by
#  approach angle — 2 x 2 m area
# ─────────────────────────────────────────────
WAYPOINTS = [
    (0.0,  0.0,  0.0),          # A  — start
    (1.8,  0.0,  math.pi / 2),  # B  — front-right corner
    (1.8,  1.8,  math.pi),      # C  — back-right corner
    (0.0,  1.8,  -math.pi / 2), # D  — back-left corner
]

WAYPOINT_STOP_TIME   = 10.0   # seconds to pause at each waypoint
OBSTACLE_STATIC_TIME =  5.0   # seconds before declaring obstacle static
SAFE_DISTANCE        =  0.5   # metres — obstacle detection threshold
MANOEUVRE_DISTANCE   =  0.6   # metres — how far to side-step
FRONT_CONE_DEG       =  40    # degrees each side for front detection

# Controller gains
KP_LINEAR  = 0.5
KP_ANGULAR = 1.5
MAX_LINEAR  = 0.35   # m/s  — keep turns gentle
MAX_ANGULAR = 0.8    # rad/s


class State(Enum):
    ROTATE_TO_TARGET  = auto()
    MOVE_FORWARD      = auto()
    ROTATE_TO_FINAL   = auto()
    WAYPOINT_STOP     = auto()
    OBSTACLE_WAIT     = auto()
    MANOEUVRE_LEFT    = auto()
    MANOEUVRE_RIGHT   = auto()
    MANOEUVRE_FORWARD = auto()
    MANOEUVRE_REALIGN = auto()


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        # ── publishers ──────────────────────────────────────
        self.cmd_pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)

        # ── subscribers ─────────────────────────────────────
        self.create_subscription(Imu,      '/imu',                       self._imu_cb,   10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self._odom_cb,  10)
        self.create_subscription(LaserScan,'/scan',                       self._scan_l_cb,10)
        self.create_subscription(LaserScan,'/scan_right',                 self._scan_r_cb,10)
        self.create_subscription(Clock,    '/clock',                      self._clock_cb, 10)

        # ── robot state ─────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.linear_vel  = 0.0
        self.angular_vel = 0.0
        self.last_time   = None

        # ── waypoint state ───────────────────────────────────
        self.current_wp  = 0
        self.state       = State.ROTATE_TO_TARGET
        self.stop_timer  = 0.0          # counts down waypoint pause

        # ── obstacle state ───────────────────────────────────
        self.obstacle_front      = False
        self.obstacle_first_seen = None  # wall-clock time
        self.obstacle_is_static  = False
        self.pre_obstacle_state  = None  # state to resume after manoeuvre
        self.manoeuvre_timer     = 0.0
        self.manoeuvre_phase     = 0

        # ── saved pre-obstacle pose (for realign after manoeuvre) ──
        self.saved_x   = 0.0
        self.saved_y   = 0.0
        self.saved_yaw = 0.0

        # ── control timer 20 Hz ─────────────────────────────
        self.create_timer(0.05, self._control_loop)
        self.get_logger().info('Robot controller started — waiting for sensor data')

    # ════════════════════════════════════════════
    #  Callbacks
    # ════════════════════════════════════════════

    def _imu_cb(self, msg):
        q = msg.orientation
        _, _, self.yaw = tf_transformations.euler_from_quaternion(
            (q.x, q.y, q.z, q.w))

    def _odom_cb(self, msg):
        self.linear_vel  = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z

    def _clock_cb(self, msg):
        # reset odometry integration at t=0
        if msg.clock.sec == 0:
            self.x = self.y = self.yaw = 0.0

    def _process_scan(self, msg, side: str):
        """Shared logic for both lidar callbacks."""
        ranges = np.array(msg.ranges, dtype=float)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=10.0)
        if len(ranges) == 0:
            return

        n = len(ranges)
        # angular resolution
        step = (msg.angle_max - msg.angle_min) / max(n - 1, 1)
        cone = int(math.radians(FRONT_CONE_DEG) / step) if step > 0 else 30
        cone = min(cone, n // 2)

        front = np.concatenate((ranges[:cone], ranges[n - cone:]))
        min_dist = float(np.min(front))

        now = self.get_clock().now()

        if min_dist < SAFE_DISTANCE:
            if self.obstacle_first_seen is None:
                self.obstacle_first_seen = now
                self.obstacle_is_static  = False
                self.get_logger().info(
                    f'[{side}] Obstacle at {min_dist:.2f} m — monitoring')
            else:
                elapsed = (now - self.obstacle_first_seen).nanoseconds / 1e9
                if elapsed >= OBSTACLE_STATIC_TIME and not self.obstacle_is_static:
                    self.obstacle_is_static = True
                    self.get_logger().info(
                        f'[{side}] Obstacle STATIC for {elapsed:.1f}s — will manoeuvre')
            self.obstacle_front = True
        else:
            if self.obstacle_front:
                self.get_logger().info(f'[{side}] Obstacle cleared')
            self.obstacle_front      = False
            self.obstacle_first_seen = None
            self.obstacle_is_static  = False

    def _scan_l_cb(self, msg):
        self._process_scan(msg, 'LEFT')

    def _scan_r_cb(self, msg):
        self._process_scan(msg, 'RIGHT')

    # ════════════════════════════════════════════
    #  Odometry integration
    # ════════════════════════════════════════════

    def _integrate_odom(self):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0 or dt > 1.0:
            return
        self.x   += self.linear_vel * math.cos(self.yaw) * dt
        self.y   += self.linear_vel * math.sin(self.yaw) * dt
        # yaw already comes from IMU

    # ════════════════════════════════════════════
    #  Publishing helpers
    # ════════════════════════════════════════════

    def _publish(self, linear: float, angular: float):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'Main_Body'
        cmd.twist.linear.x  = clamp(linear,  -MAX_LINEAR,  MAX_LINEAR)
        cmd.twist.angular.z = clamp(angular, -MAX_ANGULAR, MAX_ANGULAR)
        self.cmd_pub.publish(cmd)

    def _stop(self):
        self._publish(0.0, 0.0)

    # ════════════════════════════════════════════
    #  Main control loop
    # ════════════════════════════════════════════

    def _control_loop(self):
        self._integrate_odom()

        # ── obstacle interrupt (only during normal navigation) ──
        nav_states = {State.ROTATE_TO_TARGET,
                      State.MOVE_FORWARD,
                      State.ROTATE_TO_FINAL}

        if self.obstacle_front and self.state in nav_states:
            if self.obstacle_is_static:
                # save context and start manoeuvre
                if self.state not in (State.MANOEUVRE_LEFT,
                                      State.MANOEUVRE_RIGHT,
                                      State.MANOEUVRE_FORWARD,
                                      State.MANOEUVRE_REALIGN):
                    self.pre_obstacle_state = self.state
                    self.saved_x   = self.x
                    self.saved_y   = self.y
                    self.saved_yaw = self.yaw
                    self.manoeuvre_phase = 0
                    self.manoeuvre_timer = 0.0
                    self.state = State.MANOEUVRE_LEFT
                    self.get_logger().info('Starting manoeuvre')
            else:
                # dynamic obstacle — wait
                if self.state != State.OBSTACLE_WAIT:
                    self.pre_obstacle_state = self.state
                    self.state = State.OBSTACLE_WAIT
                    self.get_logger().info('Waiting for obstacle to move')

        # ── resume from wait if obstacle cleared ──
        if (self.state == State.OBSTACLE_WAIT
                and not self.obstacle_front):
            self.state = self.pre_obstacle_state
            self.get_logger().info('Obstacle gone — resuming')

        # ── dispatch ────────────────────────────────────────
        {
            State.ROTATE_TO_TARGET:  self._do_rotate_to_target,
            State.MOVE_FORWARD:      self._do_move_forward,
            State.ROTATE_TO_FINAL:   self._do_rotate_to_final,
            State.WAYPOINT_STOP:     self._do_waypoint_stop,
            State.OBSTACLE_WAIT:     self._do_obstacle_wait,
            State.MANOEUVRE_LEFT:    self._do_manoeuvre_left,
            State.MANOEUVRE_RIGHT:   self._do_manoeuvre_right,
            State.MANOEUVRE_FORWARD: self._do_manoeuvre_forward,
            State.MANOEUVRE_REALIGN: self._do_manoeuvre_realign,
        }[self.state]()

    # ════════════════════════════════════════════
    #  Navigation states
    # ════════════════════════════════════════════

    def _target(self):
        return WAYPOINTS[self.current_wp]

    def _do_rotate_to_target(self):
        tx, ty, _ = self._target()
        dx, dy = tx - self.x, ty - self.y
        if math.hypot(dx, dy) < 0.05:
            self.state = State.ROTATE_TO_FINAL
            return
        target_angle = math.atan2(dy, dx)
        err = normalize_angle(target_angle - self.yaw)
        if abs(err) > 0.05:
            self._publish(0.0, KP_ANGULAR * err)
        else:
            self.state = State.MOVE_FORWARD
            self.get_logger().info(
                f'WP {self.current_wp}: aligned — moving forward')

    def _do_move_forward(self):
        tx, ty, _ = self._target()
        dx, dy  = tx - self.x, ty - self.y
        dist    = math.hypot(dx, dy)
        if dist > 0.05:
            # gentle steering correction while moving
            target_angle = math.atan2(dy, dx)
            err = normalize_angle(target_angle - self.yaw)
            self._publish(KP_LINEAR * dist,
                          KP_ANGULAR * err * 0.5)
        else:
            self._stop()
            self.state = State.ROTATE_TO_FINAL
            self.get_logger().info(
                f'WP {self.current_wp}: arrived — aligning final heading')

    def _do_rotate_to_final(self):
        _, _, tyaw = self._target()
        err = normalize_angle(tyaw - self.yaw)
        if abs(err) > 0.05:
            self._publish(0.0, KP_ANGULAR * err)
        else:
            self._stop()
            self.stop_timer = WAYPOINT_STOP_TIME
            self.state = State.WAYPOINT_STOP
            self.get_logger().info(
                f'WP {self.current_wp}: reached — stopping {WAYPOINT_STOP_TIME:.0f}s')

    def _do_waypoint_stop(self):
        self._stop()
        self.stop_timer -= 0.05          # 20 Hz tick = 0.05 s
        if self.stop_timer <= 0:
            self.current_wp = (self.current_wp + 1) % len(WAYPOINTS)
            self.state = State.ROTATE_TO_TARGET
            self.get_logger().info(
                f'Moving to WP {self.current_wp}: {WAYPOINTS[self.current_wp]}')

    # ════════════════════════════════════════════
    #  Obstacle states
    # ════════════════════════════════════════════

    def _do_obstacle_wait(self):
        self._stop()

    # ── Manoeuvre sequence ───────────────────────
    # Phase 0: turn left 90°
    # Phase 1: drive forward past obstacle
    # Phase 2: turn right 90°
    # Phase 3: drive forward (clear obstacle width)
    # Phase 4: turn right 90°
    # Phase 5: drive forward to original track
    # Phase 6: turn left 90° — realign to saved heading
    # Then resume pre-obstacle state

    MANOEUVRE_SIDE_DIST    = 0.6   # m — step sideways
    MANOEUVRE_FORWARD_DIST = 0.8   # m — pass obstacle length
    TURN_90_TIME           = 1.05  # seconds at MAX_ANGULAR to turn ~90°

    def _do_manoeuvre_left(self):
        """Phase 0 — turn left 90 degrees."""
        target_yaw = normalize_angle(self.saved_yaw + math.pi / 2)
        err = normalize_angle(target_yaw - self.yaw)
        if abs(err) > 0.08:
            self._publish(0.0, KP_ANGULAR * err)
        else:
            self.manoeuvre_timer = 0.0
            self.state = State.MANOEUVRE_FORWARD
            self.manoeuvre_phase = 1
            self.get_logger().info('Manoeuvre: stepping sideways')

    def _do_manoeuvre_forward(self):
        """Drives forward a fixed distance based on manoeuvre phase."""
        self.manoeuvre_timer += 0.05
        distances = {
            1: self.MANOEUVRE_SIDE_DIST,
            3: self.MANOEUVRE_FORWARD_DIST,
            5: self.MANOEUVRE_SIDE_DIST,
        }
        target_dist = distances.get(self.manoeuvre_phase,
                                    self.MANOEUVRE_SIDE_DIST)
        speed = 0.2
        time_needed = target_dist / speed
        if self.manoeuvre_timer < time_needed:
            self._publish(speed, 0.0)
        else:
            self._stop()
            next_phase = self.manoeuvre_phase + 1
            if next_phase == 2:
                # turn right after first sideways move
                self.state = State.MANOEUVRE_RIGHT
                self.manoeuvre_phase = 2
            elif next_phase == 4:
                # turn right again after passing obstacle
                self.state = State.MANOEUVRE_RIGHT
                self.manoeuvre_phase = 4
            elif next_phase == 6:
                # realign to original heading
                self.state = State.MANOEUVRE_REALIGN
                self.manoeuvre_phase = 6
            self.manoeuvre_timer = 0.0

    def _do_manoeuvre_right(self):
        """Turns right 90 degrees (phase 2 or 4)."""
        if self.manoeuvre_phase == 2:
            target_yaw = self.saved_yaw          # back to original heading
        else:
            target_yaw = normalize_angle(self.saved_yaw - math.pi / 2)
        err = normalize_angle(target_yaw - self.yaw)
        if abs(err) > 0.08:
            self._publish(0.0, KP_ANGULAR * err)
        else:
            self.manoeuvre_timer = 0.0
            if self.manoeuvre_phase == 2:
                # drive forward past obstacle
                self.state = State.MANOEUVRE_FORWARD
                self.manoeuvre_phase = 3
                self.get_logger().info('Manoeuvre: passing obstacle')
            else:
                # drive back to original track
                self.state = State.MANOEUVRE_FORWARD
                self.manoeuvre_phase = 5
                self.get_logger().info('Manoeuvre: returning to track')

    def _do_manoeuvre_realign(self):
        """Final phase — turn left to restore original heading."""
        err = normalize_angle(self.saved_yaw - self.yaw)
        if abs(err) > 0.08:
            self._publish(0.0, KP_ANGULAR * err)
        else:
            self.get_logger().info('Manoeuvre complete — resuming navigation')
            self.obstacle_front      = False
            self.obstacle_first_seen = None
            self.obstacle_is_static  = False
            self.state = self.pre_obstacle_state or State.ROTATE_TO_TARGET


# ════════════════════════════════════════════
#  Entry point
# ════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
