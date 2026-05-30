# Robot Photographer — ROS2 Gazebo Simulation

A differential drive mobile robot that autonomously navigates a rectangular path (A→B→C→D→A), detects and classifies obstacles (static/dynamic), and operates a camera gear system at each waypoint.

Built with **ROS2 Jazzy** + **Gazebo Harmonic** on Ubuntu 24.04.

---

## Repository Structure

```
robot_photographer_sim/
├── simulation/                  # ROS2 Gazebo simulation
│   ├── robot_navigator.py       # Main navigation node
│   ├── robot_controller.py      # Low-level controller
│   ├── launch/
│   │   └── gz_sim.launch.py     # Launch file
│   ├── urdf/
│   │   └── Robot_Photographer_URDF.urdf
│   └── world/
│       └── my_world.sdf         # Gazebo world with obstacles + furniture
│
├── real_robot/                  # Real hardware implementation
│   ├── esp32/
│   │   ├── robot_test_only.ino  # ESP32 test code (no RPi comms)
│   │   └── robot_with_rpi.ino   # ESP32 full code with RPi UART
│   └── raspberry_pi/
│       └── serial_node.py       # ROS2 UART bridge node
│
└── docs/
    └── diagrams/
        ├── architecture.svg     # Full system architecture
        ├── dataflow.svg         # Mission data flow
        ├── software_flowchart.svg
        └── block_diagram.svg    # MCU/RPi block diagram
```

---

## Robot Specifications

| Parameter | Value |
|-----------|-------|
| Type | Differential drive |
| Wheel separation | 0.372 m (calibrated) |
| Wheel diameter | 0.065 m |
| Linear speed | 0.15 m/s |
| Angular speed | 0.5 rad/s |
| Sensor fusion | Odom (x,y) + IMU (yaw) |
| Obstacle threshold | 0.6 m |

---

## Waypoints

```
D(-0.5, 2.0) ◄─────────── C(5.0, 2.5)
     │                          ▲
     ▼                          │
A(0.0, 0.0)  ──────────► B(5.0, 0.0)
```

---

## Dependencies

```bash
# ROS2 Jazzy + Gazebo Harmonic
sudo apt install ros-jazzy-desktop
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-tf-transformations
pip3 install transforms3d
```

---

## Build and Run

```bash
# Build
cd ~/design_ws
colcon build --packages-select mobile_robot_sim
source install/setup.bash

# Launch simulation
pkill -f gz; pkill -f ruby; sleep 3
ros2 launch mobile_robot_sim gz_sim.launch.py

# Run navigator (new terminal)
source /opt/ros/jazzy/setup.bash
source ~/design_ws/install/setup.bash
python3 simulation/robot_navigator.py

# Start mission
ros2 topic pub /robot_command std_msgs/msg/String "{data: 'start'}" --once
```

---

## Navigation State Machine

```
rotate_to_target → move_forward → waypoint_stop (10s) → rotate_to_final → next waypoint

Obstacle detected:
  navigate → waiting (5s classify) → STATIC: avoid → recover → navigate
                                    → DYNAMIC: wait_dynamic → resume
```

## Obstacle Classification

- **5 second observation window** — robot fully stopped
- **Front-triggered**: uses front sensor variation only (`var_F > 0.1m` → DYNAMIC)
- **Side-only obstacle**: uses whichever side stayed consistently close
- **Front trigger margin**: 0.15m above threshold to catch corner approaches

---

## Key Features

- Cascaded PID steering with leg-heading reference (no diagonal drift)
- Sensor smoothing (5-reading moving average deque)
- Dynamic obstacle controller — spawns red box at waypoint B, moves `x=3.5↔7.5` at `y=1.25`
- Camera gear mover — cycles 4 joint positions during waypoint stops
- ZUPT-based gyro drift reduction (real hardware)
- Full reset between loops (`_reset_for_new_loop`)

---

## Real Hardware Architecture

See `docs/diagrams/architecture.svg` for the complete system diagram.

```
Laptop (PyQt5 GUI) ──WiFi──► Raspberry Pi ──UART──► ESP32
                              robot_navigator          Position PID
                              vision_node (YOLO)       Obstacle avoidance
                              serial_node              Camera motor
                              gui_node                 IMU + Encoders
```

UART Protocol: `P x y yaw` / `D F/B/L/R/S` / `CAM_NEXT` / `M MODE`

---

## Authors

Mechatronics and Automation Engineering — Ain Shams University
