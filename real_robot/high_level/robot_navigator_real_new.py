
import rclpy
try:
    from gpiozero import LED as GPIOLED
    led_g = GPIOLED(17)
    led_y = GPIOLED(27)
    led_r = GPIOLED(22)
    GPIO_AVAILABLE = True
    print("GPIO LEDs initialized")
except Exception as e:
    GPIO_AVAILABLE = False
    print(f"GPIO not available: {e}")
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

import math
import json
import tf_transformations

# ── Waypoints: (x_m, y_m, yaw_deg) ─────────────────────────
WAYPOINTS = [
    (0.0, 0.0, 0.0),    # A
    (2.0, 0.0, 0.0),    # B
    (2.0, 3.0, 90.0),   # C
    (0.0, 3.0, 180.0)   # D
]
WAYPOINT_LABELS = ['A', 'B', 'C', 'D']

# ── QoS profiles ────────────────────────────────────────────
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10)

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10)


class RobotNavigator(Node):

    def __init__(self):
        super().__init__('robot_navigator')

        # ── Publishers → ESP32 ───────────────────────────────
        self.cmd_pos_pub  = self.create_publisher(
            Pose2D,  '/cmd_position',   RELIABLE_QOS)
        self.mode_pub     = self.create_publisher(
            String,  '/robot_mode',     RELIABLE_QOS)
        self.cam_next_pub = self.create_publisher(
            Bool,    '/cam_next',       RELIABLE_QOS)
        self.cmd_dir_pub  = self.create_publisher(
            String,  '/cmd_direction',  RELIABLE_QOS)

        # ── Publishers → GUI ─────────────────────────────────
        self.mission_state_pub  = self.create_publisher(
            String,  '/mission_state',      10)
        self.robot_pose_pub     = self.create_publisher(
            Pose2D,  '/robot_pose',         10)
        self.mission_log_pub    = self.create_publisher(
            String,  '/mission_log',        10)
        self.cam_trig_pub       = self.create_publisher(
            Int32,   '/cam_ready_trigger',  10)

        # ── Subscribers ← ESP32 ──────────────────────────────
        self.create_subscription(
            Odometry, '/odom',
            self.odom_cb, BEST_EFFORT_QOS)
        self.create_subscription(
            Bool, '/waypoint_reached',
            self.waypoint_reached_cb, RELIABLE_QOS)
        self.create_subscription(
            Int32, '/cam_ready',
            self.cam_ready_cb, RELIABLE_QOS)
        self.create_subscription(
            Bool, '/cam_done',
            self.cam_done_cb, RELIABLE_QOS)
        self.create_subscription(
            String, '/obstacle_state',
            self.obstacle_state_cb, RELIABLE_QOS)

        # ── Subscribers ← GUI ────────────────────────────────
        self.create_subscription(
            String, '/gui_command',
            self.gui_command_cb, 10)
        self.create_subscription(
            Bool, '/vision_done',
            self.vision_done_cb, RELIABLE_QOS)

        # ── Robot pose ───────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0   # degrees, for display

        # ── Mission state ────────────────────────────────────
        # mode:      idle / auto / manual / returning_home
        # nav_state: idle / navigating / vision / complete
        self.mode           = 'idle'
        self.nav_state      = 'idle'
        self.current_wp     = 0       # index into WAYPOINTS

        # ── Vision handshake state ───────────────────────────
        self.current_cam_angle  = None   # angle ESP32 just reported ready
        self.waiting_for_vision = False  # True while vision_node capturing

        # ── Obstacle state (display only — avoidance on ESP32) ─
        self.obstacle_state = 'CLEAR'

        # ── State publish timer (2 Hz) ───────────────────────
        self.create_timer(0.5, self.publish_state)

        self.log('RobotNavigator ready — waiting for GUI command')
        self.log('  start  → begin autonomous mission')
        self.log('  manual → switch to manual drive mode')

    # ════════════════════════════════════════════════════════
    #  ODOMETRY — pose from ESP32 encoders + IMU
    # ════════════════════════════════════════════════════════
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, yaw_rad = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        self.yaw = math.degrees(yaw_rad)

        # Forward to GUI map
        pose = Pose2D()
        pose.x     = self.x
        pose.y     = self.y
        pose.theta = yaw_rad
        self.robot_pose_pub.publish(pose)

    # ════════════════════════════════════════════════════════
    #  WAYPOINT REACHED — ESP32 signals it arrived
    # ════════════════════════════════════════════════════════
    def waypoint_reached_cb(self, msg: Bool):
        if not msg.data:
            return

        label = WAYPOINT_LABELS[self.current_wp]
        self.log(f'Waypoint {label} reached '
                 f'pos=({self.x:.2f},{self.y:.2f}) yaw={self.yaw:.1f}°')

        if self.mode == 'returning_home' or self.nav_state == 'returning_home':
            # Arrived at A — wait for new START command
            self.log('Home reached — mission complete. Press START to begin new loop.')
            self.mode      = 'idle'
            self.nav_state = 'idle'
            self._send_mode('STOP')
            return

        if self.mode == 'auto':
            # Arrived at waypoint — ESP32 will now run camera sequence
            # and send CAM_READY for each angle automatically
            self.nav_state = 'vision'
            self.log(f'Waiting for camera sequence at {label}')

    # ════════════════════════════════════════════════════════
    #  CAM_READY — ESP32 camera motor settled at an angle
    #  Trigger vision_node to capture + run YOLO
    # ════════════════════════════════════════════════════════
    def cam_ready_cb(self, msg: Int32):
        if self.mode != 'auto' or self.nav_state != 'vision':
            return

        angle = msg.data
        self.current_cam_angle  = angle
        self.waiting_for_vision = True

        self.log(f'Camera at {angle}° — triggering vision node')

        # Trigger vision_node with the current angle
        trig = Int32()
        trig.data = angle
        self.cam_trig_pub.publish(trig)

    # ════════════════════════════════════════════════════════
    #  VISION DONE — vision_node finished YOLO at this angle
    #  Send CAM_NEXT to ESP32 so it moves to next angle
    # ════════════════════════════════════════════════════════
    def vision_done_cb(self, msg: Bool):
        if not self.waiting_for_vision:
            return

        self.waiting_for_vision = False
        self.log(f'Vision done at {self.current_cam_angle}° '
                 f'— sending CAM_NEXT to ESP32')

        next_msg = Bool()
        next_msg.data = True
        self.cam_next_pub.publish(next_msg)

    # ════════════════════════════════════════════════════════
    #  CAM_DONE — ESP32 finished all angles at this waypoint
    #  Advance to next waypoint
    # ════════════════════════════════════════════════════════
    def cam_done_cb(self, msg: Bool):
        if not msg.data:
            return

        label = WAYPOINT_LABELS[self.current_wp]
        self.log(f'All camera angles done at {label}')

        self.current_wp += 1

        if self.current_wp >= len(WAYPOINTS):
            # Completed full loop — return to A first
            self.log('Full loop complete — returning to A')
            self._reset_for_new_loop()
            self.current_wp = 0   # go to A
            self.nav_state = 'returning_home'
            self._send_waypoint()
        else:
            self.nav_state = 'navigating'
            self._send_waypoint()

    # ════════════════════════════════════════════════════════
    #  OBSTACLE STATE — forwarded to GUI display only
    #  ESP32 handles all avoidance internally
    # ════════════════════════════════════════════════════════
    def obstacle_state_cb(self, msg: String):
        prev = self.obstacle_state
        self.obstacle_state = msg.data
        if self.obstacle_state != prev:
            self.log(f'Obstacle state: {self.obstacle_state}')

    # ════════════════════════════════════════════════════════
    #  GUI COMMAND — from gui_node.py over WiFi
    # ════════════════════════════════════════════════════════
    def gui_command_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        self.log(f'GUI → {cmd}')

        # ── Start autonomous mission ─────────────────────────
        if cmd == 'start' and self.mode == 'idle':
            self.mode = 'armed'
            self.log('System armed — select AUTO or MANUAL')

        elif cmd == 'auto' and self.mode == 'armed':
            self.mode       = 'auto'
            self.current_wp = 1
            self.nav_state  = 'navigating'
            self._send_mode('AUTO')
            self._send_waypoint()

        # ── Stop mission ─────────────────────────────────────
        elif cmd == 'stop':
            self.mode      = 'idle'
            self.nav_state = 'idle'
            self._send_mode('STOP')
            self.log('Mission stopped')

        # ── Emergency stop ───────────────────────────────────
        elif cmd == 'estop':
            self.mode      = 'idle'
            self.nav_state = 'idle'
            self._send_mode('STOP')
            self.log('EMERGENCY STOP')

        # ── Switch to manual mode ────────────────────────────
        elif cmd == 'manual' and self.mode in ['idle', 'auto', 'armed']:
            self.mode = 'manual'
            self._send_mode('MANUAL')
            self.log('MANUAL mode — use direction buttons')

        # ── Return to auto — go home first ───────────────────
        elif cmd == 'auto' and self.mode == 'manual':
            self.mode      = 'returning_home'
            self.nav_state = 'navigating'
            self._send_mode('AUTO')
            self.log('Returning to (0,0) before resuming AUTO')
            self._send_position(0.0, 0.0, 0.0)

        # ── Manual direction buttons ─────────────────────────
        # Sent while button held, d_s sent on release
        elif cmd.startswith('d_') and self.mode == 'manual':
            direction = cmd.split('_')[1].upper()   # F / B / L / R / S
            dir_msg = String()
            dir_msg.data = direction
            self.cmd_dir_pub.publish(dir_msg)

        # ── Reset odometry ───────────────────────────────────
        elif cmd == 'reset':
            self._send_mode('RESET')
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.log('Odometry reset to (0, 0, 0°)')

        else:
            self.log(f'Unknown or invalid command: {cmd} (mode={self.mode})')

    # ════════════════════════════════════════════════════════
    #  SEND HELPERS
    # ════════════════════════════════════════════════════════
    def _send_waypoint(self):
        wp    = WAYPOINTS[self.current_wp]
        label = WAYPOINT_LABELS[self.current_wp]
        # Only A (home) gets a real heading to rotate to on arrival.
        # B, C, D send NaN so the ESP32 skips the forced rotation and
        # keeps whatever heading the last straight leg left it facing.
        theta_to_send = wp[2]
        self._send_position(wp[0], wp[1], theta_to_send)
        self.log(f'Sending waypoint {label}: '
                 f'({wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.0f} deg)')

    def _send_position(self, x: float, y: float, yaw_deg: float):
        msg       = Pose2D()
        msg.x     = float(x)
        msg.y     = float(y)
        msg.theta = math.radians(yaw_deg)   # ESP32 expects radians
        self.cmd_pos_pub.publish(msg)

    def _send_mode(self, mode: str):
        msg      = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    # ════════════════════════════════════════════════════════
    #  STATE PUBLISHING → GUI at 2 Hz
    # ════════════════════════════════════════════════════════
    def publish_state(self):
        state = {
            'mode':           self.mode,
            'nav_state':      self.nav_state,
            'current_wp':      WAYPOINT_LABELS[self.current_wp],
            'x':              round(self.x, 3),
            'y':              round(self.y, 3),
            'yaw':            round(self.yaw, 1),
            'obstacle_state': self.obstacle_state,
            'cam_angle':      self.current_cam_angle,
            'waiting_vision': self.waiting_for_vision,
        }
        out      = String()
        out.data = json.dumps(state)
        self.mission_state_pub.publish(out)
        self.update_leds()

    # ════════════════════════════════════════════════════════
    #  LOOP RESET — called after completing D → back to B
    # ════════════════════════════════════════════════════════
    def _reset_for_new_loop(self):
        self.log('Resetting state for new loop')
        self.nav_state          = 'navigating'
        self.current_cam_angle  = None
        self.waiting_for_vision = False
        self.obstacle_state     = 'CLEAR'

    # ════════════════════════════════════════════════════════
    #  LOG HELPER — console + /mission_log topic
    # ════════════════════════════════════════════════════════
    def update_leds(self):
        if not GPIO_AVAILABLE:
            return
        obs = self.obstacle_state
        mode = self.mode
        if obs in ['STATIC', 'RECOVERING']:
            led_g.off(); led_y.off(); led_r.on()
        elif mode == 'manual':
            led_g.on(); led_y.on(); led_r.off()
        elif mode in ['auto', 'armed']:
            led_g.off(); led_y.on(); led_r.off()
        else:
            led_g.on(); led_y.off(); led_r.off()

    def log(self, text: str):
        self.get_logger().info(text)
        msg      = String()
        msg.data = f'[{self._ts()}] {text}'
        self.mission_log_pub.publish(msg)

    def _ts(self) -> str:
        ns = self.get_clock().now().nanoseconds
        t  = ns / 1e9
        h  = int(t // 3600) % 24
        m  = int(t //   60) % 60
        s  = int(t)         % 60
        return f'{h:02d}:{m:02d}:{s:02d}'


# ════════════════════════════════════════════════════════════
#  ENTRY POINT
# ════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.log('Shutdown — sending STOP to ESP32')
        node._send_mode('STOP')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
