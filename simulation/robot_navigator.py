#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import numpy as np
from collections import deque
import tf_transformations
import subprocess
import threading
import time

WAYPOINTS = [
    (0.0, 0.0,  0.0),
    (5.0, 0.0,  math.pi / 2),
    (5.0, 2.5,  math.pi),
    (-0.25, 2.0, -math.pi / 2),
]

WAYPOINTS_visual = [
    (0.0, 0.0,  0.0),
    (5.0, 0.0,  math.pi / 2),
    (5.0, 2.0,  math.pi),
    (0.0, 2.0, -math.pi / 2),
]

WAYPOINT_LABELS  = ['A', 'B', 'C', 'D']
WAYPOINT_COLORS  = [
    '0 0.8 0 1',
    '0 0 0.8 1',
    '0.8 0.8 0 1',
    '0.8 0 0 1',
]

WAYPOINT_MARKER_POSITIONS = [
    (-1.0, -1.0, 0.0),
    (6.0, -1.0, 0.0),
    (6.0, 3.0, 0.0),
    (-1.0, 3.0, 0.0),
]

WAYPOINT_MARKER_NAMES = [
    'origin',
    'object_detected_B',
    'object_detected_C',
    'object_detected_D',
]

LEG_FIXED_AXIS = [
    {'axis': 'y', 'value': 0.0,  'heading': 0.0},
    {'axis': 'x', 'value': 5.0,  'heading': math.pi / 2},
    {'axis': 'y', 'value': 2.5,  'heading': math.pi},
    {'axis': 'x', 'value': 0.0,  'heading': -math.pi / 2},
]

DIST_TOLERANCE     = 0.35
ANGLE_TOLERANCE    = 0.04
LINEAR_SPEED       = 0.15
ANGULAR_SPEED_NAV  = 0.5
WAYPOINT_STOP_TIME = 10.0

STEER_KP    = 3.0
STEER_KI    = 1.5
STEER_KD    = 0.05
STEER_I_MAX = 1.0

YAW_KP    = 0.5
YAW_KI    = 0.05
YAW_KD    = 0.05
YAW_I_MAX = 0.15

THRESHOLD          = 0.6
LINEAR_SPD         = 0.15
ANGULAR_SPD        = 0.18  # Less aggressive turning (was 0.3)
SAFE_SIDE_DIST     = 0.45
MIN_VALID          = 0.50
SMOOTH_WINDOW      = 5
STUCK_TIMEOUT      = 3.0
ROBOT_WIDTH        = 0.35
SAFETY_MARGIN      = 0.15
MIN_GAP_WIDTH      = ROBOT_WIDTH + 2 * SAFETY_MARGIN

LEFT_SPLIT         = 15
RIGHT_SPLIT        = 75
CLEAR_CONFIRM_TIME = 5.0  # Wait 5 seconds for path to be clear
TREND_THRESHOLD    = 0.03  # More sensitive to small trends
STABILITY_THRESHOLD = 0.008  # More sensitive to small variance

STOP_THRESHOLD     = 0.45  # Stop and back up if obstacle < 0.45m
SLOW_THRESHOLD     = 1.0   # Start slowing down if obstacle < 1.0m


class PID:
    def __init__(self, kp, ki, kd, i_max, out_max):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.i_max = i_max; self.out_max = out_max
        self.reset()

    def reset(self):
        self.integral   = 0.0
        self.prev_error = None
        self.prev_time  = None

    def compute(self, error, now):
        if self.prev_time is None:
            self.prev_error = error
            self.prev_time  = now
            return max(-self.out_max, min(self.out_max, self.kp * error))
        dt = max(now - self.prev_time, 0.001)
        p  = self.kp * error
        self.integral = max(-self.i_max,
                        min(self.i_max, self.integral + error * dt))
        i  = self.ki * self.integral
        d  = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        self.prev_time  = now
        return max(-self.out_max, min(self.out_max, p + i + d))


class GearMover:
    """Controls camera gears only when robot is stopped at waypoints"""
    
    def __init__(self, node):
        self.node = node
        self.pub = None
        self.timer = None
        self.gear_active = False
        self.positions = [
            (0.5, -0.5),
            (1.0, -1.0),
            (0.0, 0.0),
            (-0.5, 0.5),
        ]
        self.index = 0
        
    def start(self):
        """Start the gear mover (creates publisher and timer)"""
        if self.gear_active:
            return
        self.pub = self.node.create_publisher(
            JointTrajectory, '/camera_gear_controller/joint_trajectory', 10)
        self.timer = self.node.create_timer(1.0, self.move_gears)
        self.gear_active = True
        self.node.get_logger().info('ð¥ Camera gear mover started')
    
    def stop(self):
        """Stop the gear mover (cancels timer)"""
        if not self.gear_active:
            return
        if self.timer:
            self.timer.cancel()
            self.timer = None
        self.gear_active = False
        self.node.get_logger().info('🎥 Camera gear mover stopped')
    
    def move_gears(self):
        """Move the camera gears to next position"""
        if not self.gear_active:
            return
        
        traj = JointTrajectory()
        traj.joint_names = ['Large_Gear_Joint', 'Camera_Joint']
        point = JointTrajectoryPoint()
        point.positions = list(self.positions[self.index])
        point.time_from_start.sec = 1
        traj.points.append(point)
        self.pub.publish(traj)
        self.node.get_logger().info(
            f'🎥 Moving gears: Large_Gear={point.positions[0]:.2f}, Camera={point.positions[1]:.2f}',
            throttle_duration_sec=1.0)
        self.index = (self.index + 1) % len(self.positions)


class DynamicObstacleController:
    """Manages dynamic obstacle that activates at waypoint B"""

    def __init__(self, node):
        self.node = node
        self.obstacle_active = False
        self.obstacle_name = "moving_box"
        self.obstacle_spawned = False

        self.y_fixed = 1.25
        self.x_min = 3.5
        self.x_max = 7.5
        self.speed = 0.37
        self.direction = 1
        self.current_x = 2.5

        self.move_timer = None

    def spawn_obstacle(self):
        """Spawn the moving box in Gazebo"""
        if self.obstacle_spawned:
            return

        sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{self.obstacle_name}">
    <static>false</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <emissive>0.5 0 0 1</emissive>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>"""

        filename = f'/tmp/{self.obstacle_name}.sdf'
        with open(filename, 'w') as f:
            f.write(sdf)

        result = subprocess.run([
            'gz', 'service', '-s', '/world/my_world/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000', '--req',
            f'sdf_filename: "{filename}" pose: {{position: {{x: {self.current_x} y: {self.y_fixed} z: 0.25}}}}'
        ], capture_output=True, text=True)

        if 'true' in result.stdout.lower():
            self.obstacle_spawned = True
            self.node.get_logger().info(f'🎲 Dynamic obstacle spawned at ({self.current_x}, {self.y_fixed})')
            return True
        else:
            self.node.get_logger().error(f'Failed to spawn dynamic obstacle')
            return False

    def start_moving(self):
        """Start the obstacle moving back and forth horizontally"""
        if not self.obstacle_spawned:
            if not self.spawn_obstacle():
                return

        self.obstacle_active = True
        self.move_timer = self.node.create_timer(0.05, self.update_position)
        self.node.get_logger().info('🚀 Dynamic obstacle activated and moving horizontally!')
        self.node.get_logger().info(f'   Moving from x={self.x_min} to x={self.x_max} at y={self.y_fixed}')

    def update_position(self):
        """Update obstacle position"""
        if not self.obstacle_active:
            return

        step = self.speed * 0.05
        self.current_x += self.direction * step

        if self.current_x >= self.x_max:
            self.current_x = self.x_max
            self.direction = -1
            self.node.get_logger().info('🔄 Obstacle reversing direction (moving LEFT)')
        elif self.current_x <= self.x_min:
            self.current_x = self.x_min
            self.direction = 1
            self.node.get_logger().info('🔄 Obstacle reversing direction (moving RIGHT)')

        subprocess.run([
            'gz', 'service', '-s', '/world/my_world/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000', '--req',
            f'name: "{self.obstacle_name}" position: {{x: {self.current_x} y: {self.y_fixed} z: 0.25}}'
        ], capture_output=True)

    def stop_and_remove(self):
        """Stop movement and remove the obstacle"""
        self.obstacle_active = False
        if self.move_timer:
            self.move_timer.cancel()

        if self.obstacle_spawned:
            subprocess.run([
                'gz', 'service', '-s', '/world/my_world/remove',
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000', '--req',
                f'name: "{self.obstacle_name}"'
            ], capture_output=True)
            self.obstacle_spawned = False


class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')

        self.cmd_pub = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan',       self.scan_left_cb,  10)
        self.create_subscription(LaserScan, '/scan_right', self.scan_right_cb, 10)
        self.create_subscription(Imu,       '/imu',        self.imu_cb,        10)
        self.create_subscription(Odometry,  '/diff_drive_controller/odom',
                                 self.odom_cb, 10)

        self.create_subscription(String, '/robot_command', self.command_callback, 10)

        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.mission_active = False
        self.stop_requested = False

        self.avoid_start_x = 0.0
        self.avoid_start_y = 0.0
        self.obstacle_side = None

        self.scan_left_ranges  = None
        self.scan_right_ranges = None
        self.buf_L = deque(maxlen=SMOOTH_WINDOW)
        self.buf_F = deque(maxlen=SMOOTH_WINDOW)
        self.buf_R = deque(maxlen=SMOOTH_WINDOW)

        self.case7_start      = None
        self.escape_active    = False
        self.escape_direction = 1.0

        self.mode            = 'navigate'
        self.all_clear_since = None

        self.current_wp      = 1
        self.nav_state       = 'rotate_to_target'
        self.stop_start_time = None

        self.recovery_target = None
        self.recovery_start_time = None

        self.dynamic_obstacle_activated = False
        self.dynamic_obstacle = DynamicObstacleController(self)

        self.waiting_start_time = None
        self.waiting_initial_dist = None
        self.waiting_classified = None
        self.waiting_elapsed = 0.0
        self.waiting_readings_F = []
        self.waiting_readings_L = []
        self.waiting_readings_R = []

        self.gear_mover = GearMover(self)

        self.loop_completion_at_a = False

        self.steer_pid = PID(STEER_KP, STEER_KI, STEER_KD,
                             STEER_I_MAX, ANGULAR_SPEED_NAV)
        self.yaw_pid   = PID(YAW_KP,   YAW_KI,   YAW_KD,
                             YAW_I_MAX,  ANGULAR_SPEED_NAV)

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('RobotNavigator started - waiting for START signal')
        self.get_logger().info('Send: ros2 topic pub /robot_command std_msgs/msg/String "{data: \'start\'}" --once')
        self.get_logger().info(
            f'Target: {WAYPOINT_LABELS[self.current_wp]} '
            f'at {WAYPOINTS[self.current_wp]}')
        self.waiting_cooldown_until = None  # Prevent immediate re-detection after clear

    def command_callback(self, msg):
        command = msg.data.lower().strip()

        if command == 'start':
            if not self.mission_active:
                self.mission_active = True
                self.stop_requested = False
                self.get_logger().info('🚀 START signal received - Robot mission started!')
                self.get_logger().info(f'Continuing to waypoint {WAYPOINT_LABELS[self.current_wp]}')
            else:
                self.get_logger().info('Robot already active - ignoring START command')

        elif command == 'stop':
            if self.mission_active:
                self.mission_active = False
                self.stop_requested = True
                self._stop()
                self.gear_mover.stop()
                self.get_logger().warn('🛑 STOP signal received - Robot halted immediately!')
                self.get_logger().info(f'Position: ({self.x:.2f}, {self.y:.2f}) - Use START to resume')
            else:
                self.get_logger().info('Robot already stopped - ignoring STOP command')

        elif command == 'resume':
            if not self.mission_active:
                self.mission_active = True
                self.stop_requested = False
                self.get_logger().info('▶️ RESUME signal received - Mission continuing!')
            else:
                self.get_logger().info('Robot already active - ignoring RESUME command')

        elif command == 'status':
            self.get_logger().info(f'Status: {"ACTIVE" if self.mission_active else "STOPPED"}')
            self.get_logger().info(f'Position: ({self.x:.2f}, {self.y:.2f}) Yaw: {math.degrees(self.yaw):.0f}°')
            self.get_logger().info(f'Waypoint: {WAYPOINT_LABELS[self.current_wp]}')
            self.get_logger().info(f'Mode: {self.mode}, Nav State: {self.nav_state}')
            self.get_logger().info(f'Dynamic Obstacle: {"ACTIVE" if self.dynamic_obstacle.obstacle_active else "INACTIVE"}')
            self.get_logger().info(f'Camera Gears: {"ACTIVE" if self.gear_mover.gear_active else "INACTIVE"}')
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def _clean(self, ranges):
        r = np.array(ranges, dtype=float)
        r = np.nan_to_num(r, nan=10.0, posinf=10.0)
        r[r < MIN_VALID] = 10.0
        return r

    def _min(self, arr):
        return float(np.min(arr)) if len(arr) else 10.0

    def scan_left_cb(self, msg):
        self.scan_left_ranges = self._clean(msg.ranges)

    def scan_right_cb(self, msg):
        self.scan_right_ranges = self._clean(msg.ranges)

    def imu_cb(self, msg):
        q = msg.orientation
        _, _, self.yaw = tf_transformations.euler_from_quaternion(
            (q.x, q.y, q.z, q.w))

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.current_wp == 1 and not self.dynamic_obstacle_activated and self.mission_active:
            if math.hypot(5.0 - self.x, 0.0 - self.y) < 0.5:
                self.dynamic_obstacle_activated = True
                self.get_logger().info('🎯 Reached waypoint B - Activating dynamic obstacle!')
                threading.Thread(target=self.dynamic_obstacle.start_moving, daemon=True).start()

    def _get_regions(self):
        if self.scan_left_ranges is None or self.scan_right_ranges is None:
            return None, None, None
        self.buf_L.append(self._min(self.scan_left_ranges[LEFT_SPLIT:]))
        self.buf_F.append(min(
            self._min(self.scan_left_ranges[0:LEFT_SPLIT]),
            self._min(self.scan_right_ranges[RIGHT_SPLIT:])))
        self.buf_R.append(self._min(self.scan_right_ranges[0:RIGHT_SPLIT]))
        return (float(np.mean(self.buf_L)),
                float(np.mean(self.buf_F)),
                float(np.mean(self.buf_R)))

    def _find_best_passable_gap(self):
        if self.scan_left_ranges is None or self.scan_right_ranges is None:
            return 1.0, False
        combined = np.full(150, 10.0)
        angles   = np.arange(-75.0, 75.0, 1.0)
        for i, v in enumerate(self.scan_right_ranges):
            if i < 150: combined[i] = v
        for i, v in enumerate(self.scan_left_ranges):
            idx = 60 + i
            if idx < 150: combined[idx] = v
        clear = combined > THRESHOLD
        gaps = []; in_gap = False; gs = 0
        for i in range(150):
            if clear[i] and not in_gap:   in_gap = True;  gs = i
            elif not clear[i] and in_gap: in_gap = False; gaps.append((gs, i-1))
        if in_gap: gaps.append((gs, 149))
        if not gaps: return 1.0, False
        passable = []; all_gaps = []
        for (s, e) in gaps:
            md = float(np.min(combined[s:e+1]))
            c  = float(np.mean(angles[s:e+1]))
            pw = 2.0 * md * np.tan(np.deg2rad(e-s+1) / 2.0)
            all_gaps.append({'centre': c, 'width': pw})
            if pw >= MIN_GAP_WIDTH: passable.append(all_gaps[-1])
        if passable:
            best = min(passable, key=lambda g: abs(g['centre']))
            return (1.0 if best['centre'] >= 0 else -1.0), True
        best = max(all_gaps, key=lambda g: g['width'])
        return (1.0 if best['centre'] >= 0 else -1.0), False

    def _publish(self, linear=0.0, angular=0.0):
        if not self.mission_active:
            msg = TwistStamped()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'Main_Body'
            msg.twist.linear.x  = 0.0
            msg.twist.angular.z = 0.0
            self.cmd_pub.publish(msg)
            return

        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'Main_Body'
        msg.twist.linear.x  = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)

    def _stop(self):
        self._publish(0.0, 0.0)

    @staticmethod
    def _norm(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _run_avoidance(self, dist_L, dist_F, dist_R, now):
        t = THRESHOLD
        F = dist_F < t
        L = dist_L < t
        R = dist_R < t

        # Gentle backup if too close
        if dist_F < STOP_THRESHOLD or dist_L < STOP_THRESHOLD or dist_R < STOP_THRESHOLD:
            self.get_logger().warn('[avoid] Too close! Backing up (gentler)')
            self._publish(linear=-0.045, angular=0.0)  # gentler backup
            return

        if self.escape_active:
            if not F:
                self.escape_active = False
                self.case7_start   = None
                self.get_logger().info('[escape] front clear')
            else:
                self._publish(0.0, ANGULAR_SPD * self.escape_direction)
            return

        lin, ang, label = 0.0, 0.0, ''

        if L and not R:
            self.obstacle_side = 'left'
        elif R and not L:
            self.obstacle_side = 'right'

        if not F and not L and not R:
            lin, ang, label = LINEAR_SPD, 0.0, 'case 1 — clear'
            self.case7_start = None

        elif F and not L and not R:
            lin, ang, label = 0.0, ANGULAR_SPD, 'case 2 — front'
            self.case7_start = None

        elif not F and not L and R:
            lin, ang, label = 0.0, ANGULAR_SPD, 'case 3 — right'
            self.case7_start = None

        elif not F and L and not R:
            lin, ang, label = 0.0, -ANGULAR_SPD, 'case 4 — left'
            self.case7_start = None

        elif F and not L and R:
            lin, ang, label = 0.0, ANGULAR_SPD, 'case 5 — F+R'
            self.case7_start = None

        elif F and L and not R:
            lin, ang, label = 0.0, -ANGULAR_SPD, 'case 6 — F+L'
            self.case7_start = None

        elif F and L and R:
            if self.case7_start is None:
                self.case7_start = now
                self.get_logger().warn('Surrounded — escape timer')
            stuck = now - self.case7_start
            if stuck >= STUCK_TIMEOUT:
                direction, found = self._find_best_passable_gap()
                self.escape_direction = direction
                self.escape_active    = True
                self.get_logger().warn(
                    f'Escape — {"left" if direction>0 else "right"}')
                self._publish(0.0, ANGULAR_SPD * direction)
                return
            lin, ang, label = 0.0, ANGULAR_SPD, f'case 7 ({stuck:.1f}s)'

        elif not F and L and R:
            # Both sides blocked but front clear — do NOT move forward
            # Robot is straddling obstacle — moving forward pushes into it
            # Instead rotate left to find a gap
            lin, ang, label = 0.0, ANGULAR_SPD, 'case 8 — sides, rotate left'
            self.case7_start = None

        self.get_logger().info(
            f'[avoid] {label} | L={dist_L:.2f} F={dist_F:.2f} R={dist_R:.2f}',
            throttle_duration_sec=1.0)
        self._publish(lin, ang)

    def _run_waiting(self, dist_L, dist_F, dist_R, now):
        elapsed = now - self.waiting_start_time

        if elapsed < 5.0:
            self._stop()
            remaining = 5.0 - elapsed
            self.get_logger().info(
                f'[observe] Classifying... {remaining:.1f}s remaining '
                f'| F={dist_F:.2f} L={dist_L:.2f} R={dist_R:.2f}',
                throttle_duration_sec=1.0)
            return

        if self.waiting_classified is None:
            var_F = max(self.waiting_readings_F) - min(self.waiting_readings_F)
            var_L = max(self.waiting_readings_L) - min(self.waiting_readings_L)
            var_R = max(self.waiting_readings_R) - min(self.waiting_readings_R)

            # Classification uses FRONT sensor primarily:
            # - If front was triggered (min_F < THRESHOLD+margin): classify by front only
            #   The margin accounts for corner approaches where F briefly spikes then clears
            #   Side sensors ignored — dynamic box passing nearby corrupts side readings
            # - If front was NOT triggered (side-only obstacle):
            #   Only classify as DYNAMIC if the close side reading is ALSO varying
            #   A stable close reading on the side = static obstacle at an angle
            FRONT_TRIGGER_MARGIN = 0.15  # catch brief front spikes from corner approach
            front_triggered = min(self.waiting_readings_F) < (THRESHOLD + FRONT_TRIGGER_MARGIN)

            if front_triggered:
                classification_var = var_F
                self.get_logger().info(
                    f'[classify] Front obstacle: varF={var_F:.2f} '
                    f'(varL={var_L:.2f} varR={var_R:.2f} ignored)')
            else:
                # Side-only obstacle — only use the side that stayed close
                # If a side sensor stayed consistently close → static obstacle at angle
                # If a side sensor varied a lot → dynamic obstacle passing
                min_L = min(self.waiting_readings_L)
                min_R = min(self.waiting_readings_R)
                if min_L < THRESHOLD and min_R < THRESHOLD:
                    # Both sides close — use whichever varied more
                    classification_var = max(var_L, var_R)
                elif min_L < THRESHOLD:
                    # Only left was consistently close
                    classification_var = var_L
                elif min_R < THRESHOLD:
                    # Only right was consistently close
                    classification_var = var_R
                else:
                    classification_var = max(var_L, var_R)
                self.get_logger().info(
                    f'[classify] Side obstacle: varL={var_L:.2f} varR={var_R:.2f} '
                    f'used={classification_var:.2f}m (minL={min_L:.2f} minR={min_R:.2f})')

            if classification_var > 0.1:
                self.waiting_classified = 'dynamic'
                self.get_logger().info(
                    f'🔍 Obstacle classified as DYNAMIC '
                    f'(variation={classification_var:.2f}m) — waiting to clear')
            else:
                self.waiting_classified = 'static'
                self.get_logger().info(
                    f'🔍 Obstacle classified as STATIC '
                    f'(variation={classification_var:.2f}m) — starting avoidance')
                self.mode            = 'avoid'
                self.avoid_start_x   = self.x
                self.avoid_start_y   = self.y
                self.waiting_start_time = None
                self.waiting_classified = None
                return

        F = dist_F < THRESHOLD
        L = dist_L < THRESHOLD
        R = dist_R < THRESHOLD
        all_clear = not F and not L and not R

        if all_clear:
            if self.all_clear_since is None:
                self.all_clear_since = now
            clear_dur = now - self.all_clear_since
            self.get_logger().info(
                f'[wait_dynamic] Clearing... {clear_dur:.1f}s/{CLEAR_CONFIRM_TIME}s',
                throttle_duration_sec=1.0)
            if clear_dur >= CLEAR_CONFIRM_TIME:
                self.mode               = 'navigate'
                self.nav_state          = 'move_forward'
                self.all_clear_since    = None
                self.waiting_start_time = None
                self.waiting_classified = None
                self.steer_pid.reset()
                self.yaw_pid.reset()
                self.get_logger().info(
                    '[wait_dynamic] Path clear — resuming move_forward')
        else:
            self.all_clear_since = None
            self._stop()
            self.get_logger().info(
                f'[wait_dynamic] Obstacle present '
                f'| F={dist_F:.2f} L={dist_L:.2f} R={dist_R:.2f}',
                throttle_duration_sec=1.0)

    def _run_recovery(self, dist_L, dist_F, dist_R, now):
        t = THRESHOLD

        if dist_F < t:
            self.get_logger().warn('[recovery] Front blocked — back to avoid')
            self.mode = 'avoid'
            self.recovery_target = None
            self.recovery_start_time = None
            self.steer_pid.reset()
            self.yaw_pid.reset()
            return

        if self.current_wp == 1:
            tx = max(self.x + 0.5, 5.0)
            ty = 0.0
        elif self.current_wp == 2:
            tx = 5.0
            ty = max(self.y + 0.5, 2.5)
        elif self.current_wp == 3:
            tx = min(self.x - 0.5, 0.0)
            ty = 2.5
        else:
            tx = 0.0
            ty = min(self.y - 0.5, 0.0)

        if self.recovery_target is None:
            self.recovery_target = (tx, ty)
            self.recovery_start_time = now
            self.get_logger().info(
                f'[recovery] Target: ({tx:.2f},{ty:.2f})')

        rx, ry = self.recovery_target
        dx = rx - self.x
        dy = ry - self.y
        dist_to_target = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_error = self._norm(angle_to_target - self.yaw)

        if dist_to_target < 0.15:
            # Align to actual angle toward waypoint, not just leg heading
            # This prevents large angle_error jumps in move_forward
            tx_wp, ty_wp, _ = WAYPOINTS[self.current_wp]
            angle_to_wp = math.atan2(ty_wp - self.y, tx_wp - self.x)
            heading_error = self._norm(angle_to_wp - self.yaw)
            if abs(heading_error) > ANGLE_TOLERANCE:
                angular = self.yaw_pid.compute(heading_error, now)
                self._publish(angular=angular)
                self.get_logger().info(
                    f'[recovery] Aligning to waypoint {math.degrees(angle_to_wp):.0f}° '
                    f'err={math.degrees(heading_error):.1f}°')
                return

            self.mode = 'navigate'
            self.recovery_target = None
            self.recovery_start_time = None
            self.nav_state = 'rotate_to_target'
            self.steer_pid.reset()
            self.yaw_pid.reset()
            self.get_logger().info(
                f'[recovery] Complete — back on path at ({self.x:.2f},{self.y:.2f})')
            return

        speed = min(0.20, max(0.08, dist_to_target * 0.5))
        steer = self.steer_pid.compute(angle_error, now)
        steer = max(-0.3, min(0.3, steer))

        self._publish(linear=speed, angular=steer)
        self.get_logger().info(
            f'[recovery] Moving to ({rx:.2f},{ry:.2f}) '
            f'dist={dist_to_target:.3f}m err={math.degrees(angle_error):.1f}°',
            throttle_duration_sec=1.0)

    def _run_navigation(self, now):
        tx, ty, t_yaw = WAYPOINTS[self.current_wp]
        dx = tx - self.x
        dy = ty - self.y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_error = self._norm(angle_to_target - self.yaw)
        final_yaw_error = self._norm(t_yaw - self.yaw)

        if self.nav_state == 'rotate_to_target':
            if distance < DIST_TOLERANCE:
                self._stop()
                self.yaw_pid.reset()
                self.nav_state = 'waypoint_stop_at_arrival'
                self.stop_start_time = now
                self.gear_mover.start()
            else:
                if distance < 0.3:
                    leg_heading = LEG_FIXED_AXIS[self.current_wp - 1]['heading']
                    angle_error = self._norm(leg_heading - self.yaw)

                if abs(angle_error) > ANGLE_TOLERANCE:
                    angular = self.yaw_pid.compute(angle_error, now)
                    self._publish(angular=angular)
                else:
                    self.yaw_pid.reset()
                    self.steer_pid.reset()
                    self.nav_state = 'move_forward'
                    self.get_logger().info(
                        f'Moving to {WAYPOINT_LABELS[self.current_wp]}')

        elif self.nav_state == 'move_forward':
            if distance <= 0.35:
                self._stop()
                self.steer_pid.reset()
                self.yaw_pid.reset()

                if self.loop_completion_at_a and self.current_wp == 1:
                    self.get_logger().info(
                        '🎯 Special case: A after loop completion - rotating first')
                    self.loop_completion_at_a = False
                    self.nav_state = 'rotate_to_final_special'
                    self.stop_start_time = now
                    return

                self.nav_state = 'waypoint_stop_at_arrival'
                self.stop_start_time = now
                self.gear_mover.start()
                self.get_logger().info(
                    f'Reached {WAYPOINT_LABELS[self.current_wp]} '
                    f'pos=({self.x:.2f},{self.y:.2f}) - Stopping for {WAYPOINT_STOP_TIME}s')
                return

            # Use leg_heading as steering reference — same as rotate_to_target
            # Using raw angle_to_waypoint causes sudden jump when transitioning
            # from rotate_to_target (which aligned to leg_heading)
            leg_heading  = LEG_FIXED_AXIS[self.current_wp - 1]['heading']
            angle_error  = self._norm(leg_heading - self.yaw)

            if abs(angle_error) > math.radians(10):
                self._stop()
                angular = self.yaw_pid.compute(angle_error, now)
                self._publish(angular=angular)
                self.get_logger().info(
                    f'[realign] err={math.degrees(angle_error):.0f}°',
                    throttle_duration_sec=1.0)
            else:
                speed = min(LINEAR_SPEED, max(0.03, distance * 0.3))
                steer = self.steer_pid.compute(angle_error, now)
                self._publish(linear=speed, angular=steer)
                self.get_logger().info(
                    f'[PID] err={math.degrees(angle_error):.1f}° '
                    f'I={self.steer_pid.integral:.3f} steer={steer:.3f}',
                    throttle_duration_sec=1.0)

        elif self.nav_state == 'waypoint_stop_at_arrival':
            remaining = WAYPOINT_STOP_TIME - (now - self.stop_start_time)
            self._stop()

            if remaining <= 0:
                self.gear_mover.stop()
                self.nav_state = 'rotate_to_final'
                self.get_logger().info(
                    f'Stop complete at {WAYPOINT_LABELS[self.current_wp]} - '
                    f'Aligning to {math.degrees(t_yaw):.0f}° for next waypoint')
            else:
                self.get_logger().info(
                    f'Stopped at {WAYPOINT_LABELS[self.current_wp]} '
                    f'{remaining:.0f}s remaining',
                    throttle_duration_sec=1.0)

        elif self.nav_state == 'rotate_to_final':
            if abs(final_yaw_error) > ANGLE_TOLERANCE:
                angular = self.yaw_pid.compute(final_yaw_error, now)
                self._publish(angular=angular)
            else:
                self._stop()
                self.yaw_pid.reset()
                old_wp = self.current_wp
                self.current_wp = (self.current_wp + 1) % len(WAYPOINTS)
                self.nav_state = 'rotate_to_target'
                self.recovery_target = None

                if old_wp == 3 and self.current_wp == 1:
                    self.get_logger().info(
                        '🎯 Completed full loop - setting special flag for A')
                    self.loop_completion_at_a = True
                    self._reset_for_new_loop()

                self.get_logger().info(
                    f'Aligned at {WAYPOINT_LABELS[self.current_wp]} - '
                    f'Departing to {WAYPOINT_LABELS[self.current_wp]}')

        elif self.nav_state == 'rotate_to_final_special':
            if abs(final_yaw_error) > ANGLE_TOLERANCE:
                angular = self.yaw_pid.compute(final_yaw_error, now)
                self._publish(angular=angular)
                self.get_logger().info(
                    f'Special: Aligning A to face B: yaw={math.degrees(self.yaw):.0f}° '
                    f'target={math.degrees(t_yaw):.0f}° '
                    f'err={math.degrees(final_yaw_error):.0f}°',
                    throttle_duration_sec=1.0)
            else:
                self._stop()
                self.yaw_pid.reset()
                self.nav_state = 'waypoint_stop_special'
                self.stop_start_time = now
                self.get_logger().info(
                    f'🎯 A aligned to {math.degrees(t_yaw):.0f}° - '
                    f'Now stopping for {WAYPOINT_STOP_TIME}s')
                return

        elif self.nav_state == 'waypoint_stop_special':
            remaining = WAYPOINT_STOP_TIME - (now - self.stop_start_time)
            self._stop()

            if remaining <= 0:
                self.gear_mover.stop()
                self.nav_state = 'rotate_to_target'
                self.current_wp = (self.current_wp + 1) % len(WAYPOINTS)
                self.get_logger().info(
                    f'Stop complete at A - Departing to {WAYPOINT_LABELS[self.current_wp]}')
            else:
                self.get_logger().info(
                    f'Stopped at A (facing B) {remaining:.0f}s remaining',
                    throttle_duration_sec=1.0)

        self.get_logger().info(
            f'[nav:{self.nav_state}] ({self.x:.2f},{self.y:.2f}) '
            f'yaw={math.degrees(self.yaw):.0f}° '
            f'err={math.degrees(angle_error):.0f}° '
            f'→{WAYPOINT_LABELS[self.current_wp]} {distance:.2f}m',
            throttle_duration_sec=1.0)

    def _reset_for_new_loop(self):
        self.get_logger().info('🔄 Resetting for new loop - clearing accumulated errors')
        
        self.steer_pid.reset()
        self.yaw_pid.reset()
        
        self.recovery_target = None
        self.recovery_start_time = None
        
        self.waiting_start_time = None
        self.waiting_initial_dist = None
        self.waiting_classified = None
        self.waiting_readings_F = []
        self.waiting_readings_L = []
        self.waiting_readings_R = []
        self.all_clear_since = None
        
        self.avoid_start_x = 0.0
        self.avoid_start_y = 0.0
        self.obstacle_side = None
        
        self.case7_start = None
        self.escape_active = False
        
        self.dynamic_obstacle_activated = False
        
        self.dynamic_obstacle.stop_and_remove()
        
        self.get_logger().info('✅ Reset complete - ready for new loop')

    def control_loop(self):
        if not self.mission_active:
            self._publish(0.0, 0.0)
            if hasattr(self, '_last_stop_log') and (self.get_clock().now().nanoseconds / 1e9 - self._last_stop_log) > 5.0:
                self.get_logger().info('⏸️ Robot STOPPED - Send START to begin mission')
                self._last_stop_log = self.get_clock().now().nanoseconds / 1e9
            elif not hasattr(self, '_last_stop_log'):
                self._last_stop_log = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info('⏸️ Robot STOPPED - Send START to begin mission')
            return

        dist_L, dist_F, dist_R = self._get_regions()
        if dist_L is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        t = THRESHOLD
        F = dist_F < t
        L = dist_L < t
        R = dist_R < t
        any_obstacle = F or L or R
        all_clear    = not any_obstacle

        if self.mode == 'waiting':
            self.waiting_readings_F.append(dist_F)
            self.waiting_readings_L.append(dist_L)
            self.waiting_readings_R.append(dist_R)
            self._run_waiting(dist_L, dist_F, dist_R, now)
            return

        if self.mode == 'navigate':
            if any_obstacle:
                self.get_logger().warn(
                    f'Obstacle detected → entering WAITING state'
                    f' | L={dist_L:.2f} F={dist_F:.2f} R={dist_R:.2f}')
                self.mode = 'waiting'
                self.waiting_start_time = now
                self.waiting_initial_dist = dist_F
                self.waiting_classified = None
                self.waiting_readings_F = []
                self.waiting_readings_L = []
                self.waiting_readings_R = []
                self._stop()
                return

        elif self.mode == 'avoid':
            if all_clear:
                if self.all_clear_since is None:
                    self.all_clear_since = now
                clear_dur = now - self.all_clear_since
                dist_moved = math.hypot(
                    self.x - self.avoid_start_x,
                    self.y - self.avoid_start_y
                )
                if clear_dur >= CLEAR_CONFIRM_TIME and dist_moved > 0.3:
                    self.mode = 'recover'
                    self.recovery_target = None
                    self.all_clear_since = None
                    self.steer_pid.reset()
                    self.yaw_pid.reset()
                    self.get_logger().info(
                        f'Avoid → recover pos=({self.x:.2f},{self.y:.2f})')
            else:
                self.all_clear_since = None

        elif self.mode == 'recover':
            if dist_F < THRESHOLD:
                self.get_logger().warn('Front blocked during recovery → avoid')
                self.mode = 'avoid'
                self.recovery_target = None
                self.all_clear_since = None
                self.steer_pid.reset()
                self.yaw_pid.reset()

        if self.mode == 'avoid':
            self._run_avoidance(dist_L, dist_F, dist_R, now)
        elif self.mode == 'recover':
            self._run_recovery(dist_L, dist_F, dist_R, now)
        elif self.mode == 'navigate':
            self._run_navigation(now)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
