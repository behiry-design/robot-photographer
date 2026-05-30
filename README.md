# Robot Photographer — ROS2 Gazebo Simulation

A differential drive mobile robot that autonomously navigates a rectangular path, detects and classifies obstacles (static/dynamic), and operates a camera gear system at each waypoint to simulate object photography.

Built with **ROS2 Jazzy** + **Gazebo Harmonic** on Ubuntu 24.04.

---

## Repository Structure

```
robot_photographer_sim/
├── simulation/                  # ROS2 Gazebo simulation
│   ├── robot_navigator.py       # Main navigation node
│   ├── launch/
│   │   └── gz_sim.launch.py     # Launch file
│   ├── urdf/
│   │   └── Robot_Photographer_URDF.urdf
│   └── world/
│       └── my_world.sdf         # Gazebo world with obstacles + furniture
│
├── real_robot/                  # Real hardware implementation
│   ├── esp32/
│   │   ├── robot_test_only.ino  # ESP32 standalone test (no RPi)
│   │   └── robot_with_rpi.ino   # ESP32 full code with RPi UART
│   └── raspberry_pi/
│       └── serial_node.py       # ROS2 UART bridge node
│
└── docs/
    └── diagrams/
        ├── architecture.svg       # Full system architecture
        ├── dataflow.svg           # Mission data flow
        ├── software_flowchart.svg # High + low level flowchart
        └── block_diagram.svg      # MCU/RPi block diagram
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
| Sensor fusion | Odometry (x,y) + IMU (yaw only) |
| Obstacle detection threshold | 0.6 m |
| Waypoint stop distance | 0.35 m |

---

## Mission Path

```
D(-0.25, 2.0) ◄─────────── C(5.0, 2.5)
      │                          ▲
      │      Rectangle path      │
      ▼                          │
A(0.0, 0.0)  ──────────► B(5.0, 0.0)
```

The robot navigates the rectangle **A → B → C → D → A** continuously in loops.
Each leg follows a fixed axis (Y=0 for A→B, X=5 for B→C, Y=2.5 for C→D, X=0 for D→A).

---

## What Happens at Each Waypoint

### Waypoint A — (0.0, 0.0) — Starting Point
- Robot departs from rest toward B
- On **loop completion** (returning from D):
  - Robot aligns heading to face B (0°)
  - Stops for **10 seconds** — camera gears rotate through 4 positions
  - Departs on next loop

### Waypoint B — (5.0, 0.0) — Dynamic Obstacle Zone
- When robot comes within 0.5m of B, a **red moving box** spawns in the world
  - Box moves horizontally: x = 3.5 ↔ 7.5 at y = 1.25, speed = 0.37 m/s
  - Robot must detect, classify, and wait for it to pass on leg B→C
- Robot stops at B for **10 seconds**
- Camera gears rotate through 4 positions simulating object photography
- Robot aligns to 90° heading then departs toward C

### Waypoint C — (5.0, 2.5) — Photography Stop
- Robot stops for **10 seconds**
- Camera gears rotate through 4 positions
- Robot aligns to 180° heading then departs toward D

### Waypoint D — (-0.25, 2.0) — Photography Stop
- Robot stops for **10 seconds**
- Camera gears rotate through 4 positions
- Robot aligns to -90° heading then departs toward A
- On arrival at A: dynamic obstacle is removed, state resets for next loop

---

## Camera Gear System

At each waypoint stop the `GearMover` class publishes `JointTrajectory` messages
to `/camera_gear_controller/joint_trajectory` cycling through 4 position pairs:

| Step | Large Gear | Camera Joint |
|------|-----------|--------------|
| 1 | +0.5 rad | -0.5 rad |
| 2 | +1.0 rad | -1.0 rad |
| 3 |  0.0 rad |  0.0 rad |
| 4 | -0.5 rad | +0.5 rad |

---

## Obstacle Handling

### Detection
Any sensor reading below **0.6 m** while navigating triggers the waiting state.
Sensor smoothing: 5-reading moving average deque per region (L/F/R).

### Classification (5 second observation window)
Robot stops completely. Raw sensor readings recorded every tick.

```
front_triggered = min(readings_F) < 0.75m  (threshold + 0.15m margin)

If front triggered:
    classification_var = max(F) - min(F)    ← front variation only
    side sensors ignored (dynamic box passing corrupts sides)

If side-only obstacle:
    use variation of whichever side stayed consistently close

classification_var > 0.1m  →  DYNAMIC
classification_var ≤ 0.1m  →  STATIC
```

### Static Obstacle Response — 8-case reactive avoidance

| Case | Sensors triggered | Action |
|------|------------------|--------|
| 1 | None | Move forward |
| 2 | Front only | Rotate left |
| 3 | Right only | Rotate left |
| 4 | Left only | Rotate right |
| 5 | Front + Right | Rotate left |
| 6 | Front + Left | Rotate right |
| 7 | All three | Rotate left (escape after 3s timeout) |
| 8 | Both sides only | Rotate left (do NOT move forward) |

After avoidance clears → **recovery mode**: navigate 0.5m ahead on leg axis
then resume normal navigation to waypoint.

### Dynamic Obstacle Response
Robot stays stopped until all sensors clear for **5 seconds** continuously.
After clearing: flush sensor buffers, reset PIDs, resume `move_forward`.

---

## Navigation State Machine

```
rotate_to_target
      │ aligned to waypoint direction
      ▼
move_forward   (uses leg_heading as steering reference)
      │ distance ≤ 0.35m
      ▼
waypoint_stop_at_arrival   (10s fully stopped, gears active)
      │ timer done
      ▼
rotate_to_final   (align to next leg heading)
      │ aligned
      ▼
next waypoint → rotate_to_target

Obstacle detected during move_forward:
      ▼
waiting (5s classify)
      ├── STATIC → avoid → recover → rotate_to_target
      └── DYNAMIC → wait_dynamic (all clear 5s) → move_forward
```

---

## Dependencies

```bash
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

# Terminal 1 — Launch Gazebo
pkill -f gz; pkill -f ruby; sleep 3
ros2 launch mobile_robot_sim gz_sim.launch.py

# Terminal 2 — Run navigator
source /opt/ros/jazzy/setup.bash
source ~/design_ws/install/setup.bash
python3 simulation/robot_navigator.py

# Terminal 3 — Start mission
ros2 topic pub /robot_command std_msgs/msg/String "{data: 'start'}" --once

# Other commands
ros2 topic pub /robot_command std_msgs/msg/String "{data: 'stop'}"   --once
ros2 topic pub /robot_command std_msgs/msg/String "{data: 'resume'}" --once
ros2 topic pub /robot_command std_msgs/msg/String "{data: 'status'}" --once
```

---

## World Contents

- Ground plane + 4 walls (north/south/east/west)
- `unit_box` — static obstacle at (1.51, -0.18) — on leg A→B
- `unit_sphere` — static obstacle at (-1.89, 2.36)
- Furniture: WoodenChair, LampAndStand, Table (1.5×0.8m), Suitcase, TrashBin, SmallTrolley

Download furniture models:
```bash
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/WoodenChair"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/LampAndStand"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/table"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Suitcase1"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/TrashBin"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/SmallTrolley"
```

---

## Real Hardware Architecture

See `docs/diagrams/architecture.svg` for the complete system diagram.

```
Laptop (PyQt5 GUI) ──WiFi──► Raspberry Pi ──UART──► ESP32
                              robot_navigator          Position PID (3-loop)
                              vision_node (YOLOv8)     Obstacle avoidance
                              serial_node              Camera motor control
                              gui_node                 IMU yaw + encoders
```

**UART Protocol:**

| Direction | Message | Meaning |
|-----------|---------|---------|
| RPi → ESP32 | `P x y yaw` | Go to position |
| RPi → ESP32 | `D F/B/L/R/S` | Manual direction |
| RPi → ESP32 | `CAM_NEXT` | Capture done, next angle |
| RPi → ESP32 | `M MANUAL/AUTO/S/R` | Mode / stop / reset |
| ESP32 → RPi | `O x y yaw` | Pose at 25Hz |
| ESP32 → RPi | `REACHED` | Waypoint reached |
| ESP32 → RPi | `CAM_READY angle` | Camera settled |
| ESP32 → RPi | `CAM_DONE` | All angles complete |
| ESP32 → RPi | `OBS CLEAR/STATIC/DYNAMIC/RECOVERING` | Obstacle state |

---

## Authors

Mechatronics and Automation Engineering — Ain Shams University
