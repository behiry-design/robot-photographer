#!/usr/bin/env python3
# ============================================================
#  gui_node.py — Upgraded PyQt5 Mission Control Dashboard
#  Runs on Laptop (Linux + ROS2 Jazzy)
# ============================================================

import sys
import math
import json
import os
import urllib.request

import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QHBoxLayout, QVBoxLayout, QGridLayout,
    QPushButton, QLabel, QTextEdit, QFrame,
    QSizePolicy
)
from PyQt5.QtCore import (
    Qt, QThread, pyqtSignal, QObject, pyqtSlot
)
from PyQt5.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QPixmap
)

VISION_STREAM_URL = "http://172.20.10.5:5000/video"
MAP_X_MIN, MAP_X_MAX = -0.5, 3.0
MAP_Y_MIN, MAP_Y_MAX = -0.5, 4.0
WAYPOINTS = {'A': (0.0, 0.0), 'B': (2.0, 0.0), 'C': (2.0, 3.0), 'D': (0.0, 3.0)}
WP_COLORS = {'A': '#27AE60', 'B': '#2980B9', 'C': '#F39C12', 'D': '#C0392B'}

class VideoStreamThread(QThread):
    frame_ready = pyqtSignal(QPixmap)

    def __init__(self, url):
        super().__init__()
        self.url = url
        self._running = True

    def run(self):
        while self._running:
            try:
                req = urllib.request.Request(self.url)
                stream = urllib.request.urlopen(req, timeout=4)
                data = b''
                while self._running:
                    chunk = stream.read(4096)
                    if not chunk:
                        break
                    data += chunk
                    while True:
                        start = data.find(b'\xff\xd8')
                        end = data.find(b'\xff\xd9')
                        if start != -1 and end != -1 and start < end:
                            jpg = data[start:end + 2]
                            data = data[end + 2:]
                            pixmap = QPixmap()
                            if pixmap.loadFromData(jpg):
                                self.frame_ready.emit(pixmap)
                        else:
                            break
            except Exception:
                self.msleep(1000)

    def stop(self):
        self._running = False

class RosWorker(QObject):
    state_received = pyqtSignal(dict)
    log_received = pyqtSignal(str)
    pose_received = pyqtSignal(float, float, float)
    detection_received = pyqtSignal(dict)
    obstacle_received = pyqtSignal(str)
    finished = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.node = None
        self.publisher = None
        self._running = True
        import queue
        self._cmd_queue = queue.Queue()

    @pyqtSlot()
    def start_ros(self):
        rclpy.init()
        self.node = rclpy.create_node('gui_node')
        self.publisher = self.node.create_publisher(String, '/gui_command', 10)
        self.node.create_subscription(String, '/mission_state', self._state_cb, 10)
        self.node.create_subscription(String, '/mission_log', self._log_cb, 10)
        self.node.create_subscription(Pose2D, '/robot_pose', self._pose_cb, 10)
        self.node.create_subscription(String, '/detection_result', self._detection_cb, 10)
        self.node.create_subscription(String, '/obstacle_state', self._obstacle_cb, 10)
        self.node.get_logger().info('GUI node connected to ROS2 network')
        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            while not self._cmd_queue.empty():
                cmd = self._cmd_queue.get_nowait()
                if self.publisher:
                    msg = String()
                    msg.data = cmd
                    self.publisher.publish(msg)
                    print(f"Published: {cmd}")
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()
        self.finished.emit()

    def send_command(self, cmd: str):
        self._cmd_queue.put(cmd)

    def stop(self):
        self._running = False

    def _state_cb(self, msg):
        try:
            self.state_received.emit(json.loads(msg.data))
        except Exception:
            pass

    def _log_cb(self, msg):
        self.log_received.emit(msg.data)

    def _pose_cb(self, msg):
        self.pose_received.emit(msg.x, msg.y, msg.theta)

    def _detection_cb(self, msg):
        try:
            self.detection_received.emit(json.loads(msg.data))
        except Exception:
            pass

    def _obstacle_cb(self, msg):
        self.obstacle_received.emit(msg.data)

class MapWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(380, 280)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet('background:#1A1A2E; border:2px solid #00D4FF; border-radius:6px;')
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.path = []

    def world_to_px(self, wx, wy):
        px = int((wx - MAP_X_MIN) / (MAP_X_MAX - MAP_X_MIN) * self.width())
        py = int((1.0 - (wy - MAP_Y_MIN) / (MAP_Y_MAX - MAP_Y_MIN)) * self.height())
        return px, py

    def update_pose(self, x, y, yaw):
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = yaw
        self.path.append((x, y))
        if len(self.path) > 500:
            self.path = self.path[-500:]
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#1A1A2E'))
        p.setPen(QPen(QColor('#2A2A4A'), 1))
        for gx in range(int(MAP_X_MIN), int(MAP_X_MAX) + 1):
            px, _ = self.world_to_px(gx, 0)
            p.drawLine(px, 0, px, self.height())
        for gy in range(int(MAP_Y_MIN), int(MAP_Y_MAX) + 1):
            _, py = self.world_to_px(0, gy)
            p.drawLine(0, py, self.width(), py)
        path_pts = [WAYPOINTS['A'], WAYPOINTS['B'], WAYPOINTS['C'], WAYPOINTS['D'], WAYPOINTS['A']]
        p.setPen(QPen(QColor('#FFFFFF'), 1, Qt.DashLine))
        for i in range(len(path_pts) - 1):
            x1, y1 = self.world_to_px(*path_pts[i])
            x2, y2 = self.world_to_px(*path_pts[i + 1])
            p.drawLine(x1, y1, x2, y2)
        if len(self.path) > 1:
            p.setPen(QPen(QColor('#00D4FF'), 2))
            for i in range(len(self.path) - 1):
                x1, y1 = self.world_to_px(*self.path[i])
                x2, y2 = self.world_to_px(*self.path[i + 1])
                p.drawLine(x1, y1, x2, y2)
        for label, (wx, wy) in WAYPOINTS.items():
            px, py = self.world_to_px(wx, wy)
            color = QColor(WP_COLORS[label])
            p.setPen(QPen(color, 2))
            p.setBrush(QBrush(color))
            p.drawEllipse(px - 6, py - 6, 12, 12)
            p.setPen(QPen(QColor('white'), 1))
            p.setFont(QFont('Arial', 9, QFont.Bold))
            p.drawText(px + 8, py + 4, label)
        rx, ry = self.world_to_px(self.robot_x, self.robot_y)
        p.setPen(QPen(QColor('#FF4444'), 2))
        p.setBrush(QBrush(QColor('#FF4444')))
        p.drawEllipse(rx - 7, ry - 7, 14, 14)
        arrow_len = 18
        ax = rx + int(arrow_len * math.cos(self.robot_yaw))
        ay = ry - int(arrow_len * math.sin(self.robot_yaw))
        p.setPen(QPen(QColor('#FFFF00'), 2))
        p.drawLine(rx, ry, ax, ay)
        p.end()

class DetectionCard(QFrame):
    def __init__(self, waypoint_label: str, parent=None):
        super().__init__(parent)
        self.waypoint_label = waypoint_label
        self.setMinimumSize(220, 240)
        self.setStyleSheet('QFrame { background:#16213E; border:2px solid #00D4FF; border-radius:8px; }')
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)
        self.wp_label = QLabel(f'Waypoint {waypoint_label} Capture')
        self.wp_label.setAlignment(Qt.AlignCenter)
        self.wp_label.setStyleSheet(f'color:{WP_COLORS[waypoint_label]};font-size:13px;font-weight:bold;border:none;')
        layout.addWidget(self.wp_label)
        self.img_label = QLabel('Awaiting arrival...')
        self.img_label.setMinimumSize(180, 120)
        self.img_label.setAlignment(Qt.AlignCenter)
        self.img_label.setStyleSheet('color:#666;font-size:11px;background:#0F3460;border:1px solid #333;border-radius:4px;')
        layout.addWidget(self.img_label, stretch=1)
        self.obj_label = QLabel('—')
        self.obj_label.setAlignment(Qt.AlignCenter)
        self.obj_label.setStyleSheet('color:#A8FF78;font-size:13px;font-weight:bold;border:none;')
        layout.addWidget(self.obj_label)
        self.conf_label = QLabel('conf: —')
        self.conf_label.setAlignment(Qt.AlignCenter)
        self.conf_label.setStyleSheet('color:#00D4FF;font-size:11px;border:none;')
        layout.addWidget(self.conf_label)
        self.angle_label = QLabel('angle: —')
        self.angle_label.setAlignment(Qt.AlignCenter)
        self.angle_label.setStyleSheet('color:#AAA;font-size:11px;border:none;')
        layout.addWidget(self.angle_label)

    def update_detection(self, result: dict):
        best_obj = result.get('best_object', 'none')
        best_conf = result.get('best_conf', 0.0)
        angle = result.get('angle', '—')
        img_path = result.get('image_path', '')
        self.obj_label.setText(best_obj.upper() if best_obj != 'none' else 'Nothing Detected')
        self.conf_label.setText(f'Confidence: {best_conf*100:.1f}%' if best_conf > 0 else 'conf: —')
        self.angle_label.setText(f'angle: {angle}°' if angle != '—' else 'angle: —')
        if img_path and img_path.startswith('http'):
            try:
                import urllib.request
                data = urllib.request.urlopen(img_path, timeout=3).read()
                pixmap = QPixmap()
                pixmap.loadFromData(data)
                if not pixmap.isNull():
                    pixmap = pixmap.scaled(self.img_label.width(), self.img_label.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                    self.img_label.setPixmap(pixmap)
                    self.img_label.setText('')
            except Exception as e:
                self.img_label.setText(f'[Image Error]')
        elif img_path and os.path.exists(img_path):
            pixmap = QPixmap(img_path)
            if not pixmap.isNull():
                pixmap = pixmap.scaled(self.img_label.width(), self.img_label.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.img_label.setPixmap(pixmap)
                self.img_label.setText('')
        else:
            self.img_label.setText('[Awaiting arrival...]')

class MainWindow(QMainWindow):
    def __init__(self, ros_worker: RosWorker):
        super().__init__()
        self.ros = ros_worker
        self.setWindowTitle('Robot Photographer — Mission Control Console')
        self.setMinimumSize(1280, 760)
        self.setStyleSheet('background:#0F0F1A; color:white;')
        self.manual_mode = False
        self.armed = False
        self._build_ui()
        self._connect_signals()
        self._start_video_stream()

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)
        top = QHBoxLayout()
        top.setSpacing(12)
        map_wrap = QVBoxLayout()
        map_title = QLabel('Telemetry Grid Map')
        map_title.setStyleSheet('color:#00D4FF;font-size:13px;font-weight:bold;')
        map_wrap.addWidget(map_title)
        self.map_widget = MapWidget()
        map_wrap.addWidget(self.map_widget, stretch=1)
        self.pose_label = QLabel('x: 0.000   y: 0.000   yaw: 0.0°')
        self.pose_label.setStyleSheet('color:#888;font-size:11px;')
        map_wrap.addWidget(self.pose_label)
        top.addLayout(map_wrap, stretch=4)
        stream_wrap = QVBoxLayout()
        stream_title = QLabel('Live YOLOv8 Target Vision Feed')
        stream_title.setStyleSheet('color:#A8FF78;font-size:13px;font-weight:bold;')
        stream_wrap.addWidget(stream_title)
        self.stream_view = QLabel('Establishing connection to vision server...')
        self.stream_view.setAlignment(Qt.AlignCenter)
        self.stream_view.setMinimumSize(400, 280)
        self.stream_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.stream_view.setStyleSheet('background:#1A1A2E; border:2px solid #A8FF78; border-radius:6px; color:#666;')
        stream_wrap.addWidget(self.stream_view, stretch=1)
        top.addLayout(stream_wrap, stretch=5)
        right = QVBoxLayout()
        right.setSpacing(8)
        state_frame = QFrame()
        state_frame.setStyleSheet('background:#16213E;border:2px solid #00D4FF;border-radius:8px;')
        state_layout = QGridLayout(state_frame)
        state_layout.setContentsMargins(12, 12, 12, 12)
        def make_state_row(label_text, row):
            lbl = QLabel(label_text)
            lbl.setStyleSheet('color:#AAA;font-size:12px;')
            val = QLabel('—')
            val.setStyleSheet('color:#A8FF78;font-size:13px;font-weight:bold;')
            state_layout.addWidget(lbl, row, 0)
            state_layout.addWidget(val, row, 1)
            return val
        self.mode_val = make_state_row('Mode:', 0)
        self.state_val = make_state_row('State:', 1)
        self.wp_val = make_state_row('Waypoint:', 2)
        self.cam_val = make_state_row('Camera:', 3)
        obs_lbl = QLabel('Obstacle:')
        obs_lbl.setStyleSheet('color:#AAA;font-size:12px;')
        self.obs_dot = QLabel('● CLEAR')
        self.obs_dot.setStyleSheet('color:#27AE60;font-size:13px;font-weight:bold;')
        state_layout.addWidget(obs_lbl, 4, 0)
        state_layout.addWidget(self.obs_dot, 4, 1)
        right.addWidget(state_frame)
        ctrl_frame = QFrame()
        ctrl_frame.setStyleSheet('background:#16213E;border:2px solid #534AB7;border-radius:8px;')
        ctrl_layout = QGridLayout(ctrl_frame)
        ctrl_layout.setContentsMargins(10, 10, 10, 10)
        ctrl_layout.setSpacing(6)
        def btn(text, color, handler):
            b = QPushButton(text)
            b.setFixedHeight(34)
            b.setStyleSheet(f"QPushButton {{ background:{color};color:white;font-size:12px;font-weight:bold;border-radius:6px;border:none; }} QPushButton:hover {{ opacity:0.9; }} QPushButton:pressed {{ background:#333; }}")
            b.clicked.connect(handler)
            b.setCursor(Qt.PointingHandCursor)
            return b
        ctrl_layout.addWidget(btn('START', '#27AE60', self.on_start), 0, 0)
        ctrl_layout.addWidget(btn('STOP', '#E74C3C', self.on_stop), 0, 1)
        ctrl_layout.addWidget(btn('AUTO', '#2980B9', self.on_auto), 1, 0)
        ctrl_layout.addWidget(btn('MANUAL', '#8E44AD', self.on_manual), 1, 1)
        ctrl_layout.addWidget(btn('⚠ EMERGENCY STOP', '#C0392B', self.on_estop), 2, 0, 1, 2)
        ctrl_layout.addWidget(btn('RESET ODOM', '#555', self.on_reset), 3, 0, 1, 2)
        right.addWidget(ctrl_frame)
        drive_frame = QFrame()
        drive_frame.setStyleSheet('background:#16213E;border:2px solid #8E44AD;border-radius:8px;')
        drive_layout = QGridLayout(drive_frame)
        drive_layout.setContentsMargins(10, 8, 10, 8)
        drive_layout.setSpacing(4)
        drive_title = QLabel('Manual Override Axis')
        drive_title.setStyleSheet('color:#8E44AD;font-size:11px;font-weight:bold;')
        drive_layout.addWidget(drive_title, 0, 0, 1, 3)
        def drive_btn(text, cmd):
            b = QPushButton(text)
            b.setFixedSize(65, 34)
            b.setStyleSheet('QPushButton { background:#2C2C4A;color:#45f3ff;font-size:18px;font-weight:bold;border-radius:6px;border:1px solid #8E44AD; } QPushButton:pressed { background:#8E44AD; }')
            b.pressed.connect(lambda c=cmd: self.ros.send_command(c) if self.manual_mode else None)
            b.released.connect(lambda: self.ros.send_command('d_s') if self.manual_mode else None)
            return b
        drive_layout.addWidget(drive_btn('↑', 'd_f'), 1, 1)
        drive_layout.addWidget(drive_btn('←', 'd_l'), 2, 0)
        drive_layout.addWidget(drive_btn('↓', 'd_b'), 2, 1)
        drive_layout.addWidget(drive_btn('→', 'd_r'), 2, 2)
        right.addWidget(drive_frame)
        top.addLayout(right, stretch=3)
        root.addLayout(top, stretch=7)
        det_title = QLabel('Historic Waypoint Capture Logs (Target Classification Profiles)')
        det_title.setStyleSheet('color:#00D4FF;font-size:13px;font-weight:bold;')
        root.addWidget(det_title)
        cards_row = QHBoxLayout()
        cards_row.setSpacing(12)
        self.cards = {}
        for label in ['B', 'C', 'D']:
            card = DetectionCard(label)
            self.cards[label] = card
            cards_row.addWidget(card)
        cards_row.addStretch()
        root.addLayout(cards_row, stretch=3)
        log_title = QLabel('System Activity Logs')
        log_title.setStyleSheet('color:#00D4FF;font-size:13px;font-weight:bold;')
        root.addWidget(log_title)
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setFixedHeight(110)
        self.log_box.setStyleSheet('QTextEdit { background:#0F3460; color:#A8FF78; font-family:monospace; font-size:11px; border:2px solid #00D4FF; border-radius:6px; }')
        root.addWidget(self.log_box, stretch=2)

    def _connect_signals(self):
        self.ros.state_received.connect(self.on_state)
        self.ros.log_received.connect(self.on_log)
        self.ros.pose_received.connect(self.on_pose)
        self.ros.detection_received.connect(self.on_detection)
        self.ros.obstacle_received.connect(self.on_obstacle)

    def _start_video_stream(self):
        self.video_thread = VideoStreamThread(VISION_STREAM_URL)
        self.video_thread.frame_ready.connect(self.on_video_frame_received)
        self.video_thread.start()

    @pyqtSlot(QPixmap)
    def on_video_frame_received(self, pixmap: QPixmap):
        scaled_pixmap = pixmap.scaled(self.stream_view.width(), self.stream_view.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.stream_view.setPixmap(scaled_pixmap)

    @pyqtSlot(dict)
    def on_state(self, state: dict):
        self.mode_val.setText(state.get('mode', '—').upper())
        self.state_val.setText(state.get('nav_state', '—'))
        self.wp_val.setText(state.get('current_wp', '—'))
        angle = state.get('cam_angle')
        self.cam_val.setText(f'{angle}°' if angle is not None else '—')

    @pyqtSlot(str)
    def on_log(self, text: str):
        self.log_box.append(text)
        doc = self.log_box.document()
        while doc.blockCount() > 8:
            cursor = self.log_box.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()
        sb = self.log_box.verticalScrollBar()
        sb.setValue(sb.maximum())

    @pyqtSlot(float, float, float)
    def on_pose(self, x: float, y: float, yaw: float):
        self.map_widget.update_pose(x, y, yaw)
        self.pose_label.setText(f'x: {x:.3f}   y: {y:.3f}   yaw: {math.degrees(yaw):.1f}°')

    @pyqtSlot(dict)
    def on_detection(self, result: dict):
        wp = result.get('waypoint', '')
        if wp in self.cards:
            self.cards[wp].update_detection(result)

    @pyqtSlot(str)
    def on_obstacle(self, state: str):
        colors = {'CLEAR': ('#27AE60', '● CLEAR'), 'STATIC': ('#E74C3C', '● STATIC'), 'DYNAMIC': ('#F39C12', '● DYNAMIC'), 'RECOVERING': ('#3498DB', '● RECOVERING')}
        color, text = colors.get(state, ('#888', f'● {state}'))
        self.obs_dot.setStyleSheet(f'color:{color};font-size:13px;font-weight:bold;')
        self.obs_dot.setText(text)

    def on_start(self):
        self.armed = True
        self.ros.send_command('start')
        self.log_box.append('[GUI] System armed — select AUTO or MANUAL')
    def on_stop(self):
        self.manual_mode = False
        self.armed = False
        self.ros.send_command('stop')
        self.log_box.append('[GUI] Mission stopped')
    def on_auto(self):
        self.manual_mode = False
        self.ros.send_command('auto')
        if self.armed:
            self.log_box.append('[GUI] AUTO mode activated')
    def on_manual(self):
        if self.armed:
            self.manual_mode = True
            self.ros.send_command('manual')
            self.log_box.append('[GUI] MANUAL mode activated')
        else:
            self.log_box.append('[GUI] Press START first!')
    def on_estop(self): self.ros.send_command('estop')
    def on_reset(self): self.ros.send_command('reset')

    def closeEvent(self, event):
        self.ros.send_command('estop')
        if hasattr(self, 'video_thread'):
            self.video_thread.stop()
            self.video_thread.wait()
        self.ros.stop()
        event.accept()


def main():
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    ros_worker = RosWorker()
    ros_thread = QThread()
    ros_worker.moveToThread(ros_thread)
    ros_thread.started.connect(ros_worker.start_ros)
    ros_worker.finished.connect(ros_thread.quit)
    ros_worker.finished.connect(ros_worker.deleteLater)
    ros_thread.finished.connect(ros_thread.deleteLater)
    ros_thread.start()
    window = MainWindow(ros_worker)
    window.show()
    ret = app.exec_()
    ros_worker.stop()
    ros_thread.quit()
    ros_thread.wait()
    sys.exit(ret)


if __name__ == '__main__':
    main()
