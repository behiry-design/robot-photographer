# ESP32 Low-Level Controller

## Files

### robot_test_only.ino
Standalone test code — no Raspberry Pi required.
Control via Arduino Serial Monitor at 115200 baud.

**Commands:**
| Command | Action |
|---------|--------|
| `fw <sec>` | Move forward N seconds |
| `bw <sec>` | Move backward N seconds |
| `lt <deg>` | Turn left N degrees |
| `rt <deg>` | Turn right N degrees |
| `yh` | Enable yaw hold at current heading |
| `yf` | Free yaw (disable hold) |
| `yr` | Reset yaw to 0° |
| `st` | Stop immediately |
| `cal` | Re-calibrate gyro |
| `info` | Print current state |

**Tuning order:**
1. `WHL_DEADZONE` — increase until robot moves at minimum speed without stalling
2. `WHL_KP_VEL` — increase until wheel tracks speed without oscillating
3. `WHL_KI_VEL` — increase until steady-state speed error = 0
4. `YAW_KP` — increase until heading corrections are fast without overshoot

### robot_with_rpi.ino
Full production code with Raspberry Pi UART communication.

**Wiring:**
| Component | GPIO |
|-----------|------|
| MPU6050 SDA | 21 |
| MPU6050 SCL | 22 |
| Motor L IN1/IN2/ENA | 25/26/27 |
| Motor R IN3/IN4/ENB | 32/33/14 |
| Encoder L A/B | 34/35 |
| Encoder R A/B | 36/39 |
| RPi TX (Serial2) | 17 |
| RPi RX (Serial2) | 16 |

**UART Protocol — RPi → ESP32:**
```
P x y yaw      Go to position (AUTO mode)
D F/B/L/R/S    Direction command (MANUAL mode)
CAM_NEXT       Frame captured, move to next camera angle
M MANUAL       Switch to manual velocity mode
M AUTO         Switch to position PID mode
M S            Emergency stop
M R            Reset odometry and yaw
```

**UART Protocol — ESP32 → RPi:**
```
O x y yaw          Current pose at 25Hz
REACHED            Waypoint reached
CAM_READY angle    Camera settled at angle
CAM_DONE           All camera angles complete
OBS CLEAR          Path clear
OBS STATIC         Static obstacle — avoiding
OBS DYNAMIC        Dynamic obstacle — waiting
OBS RECOVERING     Recovering to path
```
