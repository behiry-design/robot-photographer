#!/usr/bin/env python3
# ============================================================
#  serial_node.py — Raspberry Pi UART ↔ ROS2 bridge
#
#  Receives odometry from ESP32 → publishes /odom + /imu
#  Receives /cmd_vel from robot_navigator.py → sends to ESP32
#
#  Install dependencies:
#    pip3 install pyserial
#    sudo apt install ros-jazzy-tf2-ros
#
#  Run:
#    python3 serial_node.py --port /dev/ttyAMA0 --baud 115200
#
#  /dev/ttyAMA0  = hardware UART on RPi GPIO 14/15 (TX/RX)
#  /dev/ttyUSB0  = USB-serial adapter (for testing)
# ============================================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import serial
import math
import argparse
import threading
import time

class SerialBridgeNode(Node):

    def __init__(self, port: str, baud: int):
        super().__init__('serial_bridge')

        # ── Publishers → ROS2 ───────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/diff_drive_controller/odom', 10)
        self.imu_pub  = self.create_publisher(Imu,      '/imu', 10)

        # ── Subscriber ← robot_navigator.py ─────────────────
        self.create_subscription(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            self.cmd_vel_cb,
            10)

        # ── Serial port ──────────────────────────────────────
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.get_logger().info(f'Serial bridge opened: {port} @ {baud}')

        # ── Odometry state ───────────────────────────────────
        # ESP32 sends cumulative dist_L, dist_R in metres.
        # We differentiate here to get incremental displacement.
        self.prev_dist_L = 0.0
        self.prev_dist_R = 0.0
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0   # radians — taken directly from IMU

        # ── Physical parameters (must match ESP32) ───────────
        self.WHEEL_BASE_M = 0.372

        # ── Start serial read thread ─────────────────────────
        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

        self.get_logger().info('Serial bridge ready')

    # ── Receive cmd_vel from robot_navigator → send to ESP32 ─
    def cmd_vel_cb(self, msg: TwistStamped):
        linear  = msg.twist.linear.x
        angular = msg.twist.angular.z
        line = f'V {linear:.4f} {angular:.4f}\n'
        try:
            self.ser.write(line.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    # ── Read thread: parse odom packets from ESP32 ───────────
    def _read_loop(self):
        while self._running:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='ignore').strip()
                if line.startswith('O '):
                    self._parse_odom(line)
            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')
                time.sleep(0.01)

    # ── Parse "O dist_L dist_R yaw_deg vel_L_ms vel_R_ms" ────
    def _parse_odom(self, line: str):
        parts = line.split()
        if len(parts) != 6:
            return
        try:
            dist_L   = float(parts[1])
            dist_R   = float(parts[2])
            yaw_deg  = float(parts[3])
            vel_L_ms = float(parts[4])
            vel_R_ms = float(parts[5])
        except ValueError:
            return

        # ── Differential drive odometry ──────────────────────
        # Incremental distances since last packet
        d_dist_L = dist_L - self.prev_dist_L
        d_dist_R = dist_R - self.prev_dist_R
        self.prev_dist_L = dist_L
        self.prev_dist_R = dist_R

        # Centre displacement and heading from IMU (authoritative)
        dc   = (d_dist_L + d_dist_R) / 2.0
        self.yaw = yaw_deg * math.pi / 180.0

        # Update position
        self.x += dc * math.cos(self.yaw)
        self.y += dc * math.sin(self.yaw)

        # Linear and angular velocity from wheel speeds
        vel_linear  = (vel_L_ms + vel_R_ms) / 2.0
        vel_angular = (vel_R_ms - vel_L_ms) / self.WHEEL_BASE_M

        now = self.get_clock().now().to_msg()

        # ── Publish /odom ─────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy

        odom.twist.twist.linear.x  = vel_linear
        odom.twist.twist.angular.z = vel_angular

        self.odom_pub.publish(odom)

        # ── Publish /imu ──────────────────────────────────────
        imu = Imu()
        imu.header.stamp    = now
        imu.header.frame_id = 'imu_link'

        imu.orientation.w = cy
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = sy

        # Orientation covariance — reasonable for gyro-integrated yaw
        imu.orientation_covariance[0] = 0.01
        imu.orientation_covariance[4] = 0.01
        imu.orientation_covariance[8] = 0.05

        self.imu_pub.publish(imu)

    # ── Convenience: send mode commands to ESP32 ─────────────
    def send_mode(self, mode: str):
        """mode: 'H' hold, 'F' free, 'S' stop, 'R' reset"""
        try:
            self.ser.write(f'M {mode}\n'.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def destroy_node(self):
        self._running = False
        self.send_mode('S')   # stop robot on shutdown
        time.sleep(0.1)
        self.ser.close()
        super().destroy_node()


# ============================================================
#  Entry point
# ============================================================
def main():
    parser = argparse.ArgumentParser(description='ESP32 ↔ ROS2 serial bridge')
    parser.add_argument('--port', default='/dev/ttyAMA0',
                        help='Serial port (default /dev/ttyAMA0)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Baud rate (default 115200)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = SerialBridgeNode(args.port, args.baud)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
