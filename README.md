# 🤖 Robot Photographer — Autonomous Object Detection & Photography Robot

> MCT334 Robotics Project | Ain Shams University — Mechatronics & Automation Engineering

A semi autonomous robot that navigates a predefined 3×3 meter square path, stops at three waypoints, scans shelves using a servo-mounted camera, detects target objects using YOLOv8, captures annotated photos, and streams everything live to a PyQt5 mission control GUI — all coordinated over ROS2 Jazzy across three computing nodes connected via WiFi hotspot.

---

## 📐 System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        LAPTOP (GUI)                             │
│  gui_node.py — PyQt5 Mission Control Dashboard                  │
│  • Live YOLOv8 camera stream                                    │
│  • 2D telemetry map with robot pose tracking                    │
│  • Waypoint detection capture cards                             │
│  • Manual override with directional control                     │
│  • Mission state indicators                                     │
└────────────────────────┬────────────────────────────────────────┘
                         │ ROS2 / CycloneDDS over WiFi
┌────────────────────────▼────────────────────────────────────────┐
│                   RASPBERRY PI 5 (High Level)                   │
│                                                                 │
│  robot_navigator.py — Mission State Machine                     │
│  • Waypoint sequencing (A→B→C→D→A)                             │
│  • Vision orchestration & handshake                             │
│  • GPIO LED status indicators                                   │
│  • Obstacle state forwarding to GUI                             │
│                                                                 │
│  vision_server.py — YOLOv8 Detection + Flask Stream            │
│  • Triggered YOLO inference per shelf angle                     │
│  • 30-second search window per angle for target object          │
│  • Annotated image saving and HTTP serving                      │
│  • Live MJPEG camera stream at port 5000                        │
└────────────────────────┬────────────────────────────────────────┘
                         │ MicroROS Serial (USB)
┌────────────────────────▼────────────────────────────────────────┐
│                    ESP32 (Low Level)                            │
│                                                                 │
│  7 FreeRTOS Tasks across Dual Cores:                           │
│  Core 1: IMU Integration | PID Motor Control & Odometry        │
│  Core 0: MicroROS Comms | Ultrasonic Safety | LED | Servo      │
│                                                                 │
│  • Differential drive with encoder odometry                     │
│  • MPU6050 IMU yaw integration                                  │
│  • Manhattan path navigation (X-leg → Y-leg → theta)           │
│  • Ultrasonic obstacle detection + 8-step triangle detour       │
│  • Servo-controlled camera tilt (3 shelf angles)                │
│  • MicroROS publisher/subscriber over serial                    │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🛠 Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | ESP32 DevKit (Dual Core, 240MHz) |
| Single Board Computer | Raspberry Pi 5 (Ubuntu 24.04) |
| Camera | Pi Camera Module v2 (IMX219, 8MP) |
| IMU | Adafruit MPU6050 (I2C, SDA=21 SCL=22) |
| Motors | DC Motors with Quadrature Encoders |
| Motor Driver | L298N / TB6612 compatible |
| Obstacle Sensors | 2× HC-SR04 Ultrasonic (Left + Right) |
| Camera Mechanism | SG90 Servo (PWM, pin 17) |
| Status LEDs | 3× LEDs on Raspberry Pi GPIO (17=Green, 27=Yellow, 22=Red) |
| Communication | USB Serial (ESP32↔Pi), WiFi Hotspot (Pi↔Laptop) |

### Pin Assignments (ESP32)

| Function | Pin |
|----------|-----|
| Left Motor IN1/IN2/ENA | 25, 26, 27 |
| Right Motor IN1/IN2/ENA | 32, 33, 14 |
| Left Encoder A/B | 34, 35 |
| Right Encoder A/B | 18, 19 |
| Ultrasonic Left TRIG/ECHO | 4, 13 |
| Ultrasonic Right TRIG/ECHO | 2, 15 |
| Servo (Camera Tilt) | 17 |
| LED Green/Yellow/Red | 23, 12, 5 |
| MPU6050 SDA/SCL | 21, 22 |

---

## 🗺 Mission Path

```
        C ─────────── B
        │             │
        │             │
        D ─────────── A (home/origin 0,0)
```

| Waypoint | Coordinates | Target Object | Shelf Angles |
|----------|-------------|---------------|--------------|
| A | (0, 0) | Home — no scan | — |
| B | (0, 3) | plastic_bottle | 60°, 53°, 45° |
| C | (-3, 3) | clock | 60°, 53°, 45° |
| D | (-3, 0) | perfume | 60°, 53°, 45° |

**Navigation sequence:** A → B (north) → C (west) → D (south) → A (east, return home)

**Heading behavior:** Robot keeps its travel heading while scanning at each waypoint. Once the target is found (or all angles exhausted), it rotates to face the next leg's direction before departing.

---

## 📡 ROS2 Topic Interface

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_position` | Pose2D | Pi → ESP32 | Target waypoint (x, y, theta) |
| `/robot_mode` | String | Pi → ESP32 | AUTO / MANUAL / STOP / RESET |
| `/cmd_direction` | String | Pi → ESP32 | F / B / L / R / S (manual) |
| `/cam_next` | Bool | Pi → ESP32 | Advance servo to next shelf angle |
| `/odom` | Odometry | ESP32 → Pi | Robot pose from encoders + IMU |
| `/waypoint_reached` | Bool | ESP32 → Pi | Robot arrived at waypoint |
| `/cam_ready` | Int32 | ESP32 → Pi | Servo settled at angle (degrees) |
| `/cam_done` | Bool | ESP32 → Pi | All shelf angles scanned |
| `/obstacle_state` | String | ESP32 → Pi | CLEAR / STATIC / RECOVERING |
| `/cam_ready_trigger` | Int32 | Pi → Vision | Trigger YOLO at this angle |
| `/vision_done` | Bool | Vision → Pi | Detection complete (found=True/False) |
| `/detection_result` | String | Vision → GUI | JSON detection result with image URL |
| `/mission_state` | String | Pi → GUI | Full JSON mission state at 2Hz |
| `/robot_pose` | Pose2D | Pi → GUI | Robot x, y, yaw for map display |
| `/mission_log` | String | Pi → GUI | Timestamped log messages |
| `/gui_command` | String | GUI → Pi | start / stop / auto / manual / d_f etc. |

---

## 🧠 Vision System

- **Model:** YOLOv8 custom trained — 4 classes: `plastic_bottle`, `backpack`, `perfume`, `clock`
- **Training:** mAP50 = 0.967, trained on RTX 3050, merged Roboflow datasets
- **Inference:** PyTorch CPU on Raspberry Pi 5, ~1.8s per inference
- **Detection logic:** At each waypoint, servo moves to each shelf angle (60° → 53° → 45°). At each angle, YOLO runs continuously for up to **30 seconds** searching for the waypoint's target object. If found → saves annotated photo → publishes result to GUI → moves to next waypoint. If not found at any angle → logs "not found" → moves on.
- **Streaming:** Flask MJPEG server at `http://<PI_IP>:5000/video`, annotated images served at `http://<PI_IP>:5000/images/<filename>`

---

## 🚗 Obstacle Avoidance

The ESP32 runs an **8-step isosceles triangle detour** when an obstacle is detected:

1. Detect obstacle (ultrasonic < 25cm) → stop → save position
2. Wait 5 seconds for clearance
3. If still blocked → execute detour:
   - Turn 90° left → drive sideways (D_SIDE + W_OBSTACLE/2)
   - Turn back to original heading → drive forward (D_FRONT)  
   - Turn 90° right → drive sideways back to original line
   - Turn back to original heading → resume original path
4. 2-second stabilization delay before resuming after obstacle clears

---

## 📦 Software Stack

| Component | Technology |
|-----------|-----------|
| Robot OS | ROS2 Jazzy (CycloneDDS) |
| ESP32 Framework | Arduino + FreeRTOS + MicroROS (v5.0.2) |
| Computer Vision | YOLOv8 (Ultralytics) |
| Camera Interface | libcamera v0.7.0 (built from source with PISP patches) |
| Vision Server | Flask + OpenCV |
| GUI Framework | PyQt5 |
| GPIO Control | gpiozero (RPi.GPIO incompatible with Pi 5) |
| Pose Estimation | tf_transformations (quaternion → euler) |

---

## 🚀 Running the System

### Prerequisites

**Raspberry Pi 5:**
```bash
# ROS2 Jazzy
# MicroROS agent built at ~/uros_ws
# gpiozero: pip3 install gpiozero lgpio --break-system-packages
# libcamera built at /usr/local/lib/aarch64-linux-gnu/libcamera
```

**Laptop:**
```bash
# ROS2 Jazzy
# PyQt5: pip install pyqt5
# tf_transformations: pip install transforms3d
```

### Network Setup

Connect both Raspberry Pi and Laptop to the **same WiFi hotspot**. Find Pi IP:
```bash
nmap -sn 172.20.10.0/24 --host-timeout 5s
```

Update IP in `vision_server.py` and `gui_node.py` to match Pi's actual IP.

### Launch Sequence

**Pi Terminal 1 — MicroROS Agent:**
```bash
python3 -c "import serial,time; s=serial.Serial('/dev/ttyUSB0',115200); s.setDTR(False); time.sleep(0.1); s.setDTR(True); time.sleep(0.1); s.close()"
sudo chmod 666 /dev/ttyUSB0
cd ~/uros_ws && source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Pi Terminal 2 — Navigator:**
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
python3 robot_navigator.py
```

**Pi Terminal 3 — Vision Server:**
```bash
export LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa
export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
python3 vision_server.py
```

**Laptop — GUI:**
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
python3 gui/gui_node.py
```

Then in the GUI: **START → AUTO**

---

## 📁 Repository Structure

```
robot-photographer/
├── README.md
├── real_robot/
│   ├── esp32/
│   │   └── esp32_firmware.ino       # Low-level firmware (FreeRTOS + MicroROS)
│   ├── raspberry_pi/
│   │   ├── robot_navigator.py       # Mission state machine + GPIO LEDs
│   │   └── vision_server.py         # YOLOv8 detection + Flask stream
│   └── gui/
│       └── gui_node.py              # PyQt5 mission control dashboard
├── simulation/
│   ├── robot_navigator.py           # ROS2 Gazebo simulation navigator
│   ├── launch/gz_sim.launch.py      # Gazebo Harmonic launch file
│   ├── urdf/                        # Robot URDF model
│   └── world/my_world.sdf           # Simulation world
└── docs/
    └── diagrams/                    # Architecture and dataflow diagrams
```

---

## 👥 Team

- **[Shahd Ahmed , Ahmed Behiry]** — Gazebo Simulation
- **[Ahmed Behiry,Rewan Mohamed ]** — System integration, ROS2 architecture, vision pipeline, GUI
- **[Ahmed Hisham]** — ESP32 firmware, motor control, FreeRTOS tasks
- **[Ahmed Behiry]** — YOLOv8 training, dataset preparation
- **[Ahmed Hisham ,Adham Hamada]** — Mechanical design, wiring, hardware integration
- **[Shahd Ahmed , Rewan Mohamed]** — Report & Documentation 

**Supervisor:** Dr. Shady Maged  — MCT334 Design of Mechatronics systems 2 course

**Institution:** Faculty of Engineering, Ain Shams University — Mechatronics & Automation Engineering, Class of 2027

---

## 📄 License

MIT License — see LICENSE file for details.
