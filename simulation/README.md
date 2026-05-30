# Simulation — robot_navigator.py

## Overview
Main navigation node for the ROS2 Gazebo simulation.
Runs at 10Hz and manages the full mission autonomously.

## ROS2 Topics

### Subscribes
| Topic | Type | Source |
|-------|------|--------|
| `/scan` | `LaserScan` | Left ultrasonic (Gazebo) |
| `/scan_right` | `LaserScan` | Right ultrasonic (Gazebo) |
| `/imu` | `Imu` | IMU plugin (heading only) |
| `/diff_drive_controller/odom` | `Odometry` | Wheel odometry (x,y only) |
| `/robot_command` | `String` | Mission control |

### Publishes
| Topic | Type | Purpose |
|-------|------|---------|
| `/diff_drive_controller/cmd_vel` | `TwistStamped` | Wheel velocity commands |
| `/camera_gear_controller/joint_trajectory` | `JointTrajectory` | Gear motion at waypoints |

## Mission Commands
```bash
ros2 topic pub /robot_command std_msgs/msg/String "{data: 'start'}"  --once
ros2 topic pub /robot_command std_msgs/msg/String "{data: 'stop'}"   --once
ros2 topic pub /robot_command std_msgs/msg/String "{data: 'resume'}" --once
ros2 topic pub /robot_command std_msgs/msg/String "{data: 'status'}" --once
```

## PID Parameters
| Controller | KP | KI | KD | I_MAX | OUT_MAX |
|------------|----|----|-----|-------|---------|
| steer_pid | 3.0 | 1.5 | 0.05 | 1.0 | 0.5 |
| yaw_pid | 0.5 | 0.05 | 0.05 | 0.15 | 0.5 |

## Tuning Notes
- `DIST_TOLERANCE = 0.35m` — stop this far from waypoint to prevent overshoot
- `CLEAR_CONFIRM_TIME = 2.0s` — sensors must be clear this long before resuming
- `SMOOTH_WINDOW = 5` — moving average window for sensor smoothing
- `ANGULAR_SPD = 0.18 rad/s` — avoidance turning speed (low = less wheel slip)
