# Raspberry Pi High-Level Nodes

## serial_node.py
UART bridge between ROS2 and ESP32.

**Install dependencies:**
```bash
pip3 install pyserial
```

**Run:**
```bash
python3 serial_node.py --port /dev/ttyAMA0 --baud 115200
```

**Ports:**
- `/dev/ttyAMA0` — hardware UART on RPi GPIO 14(TX)/15(RX)
- `/dev/ttyUSB0` — USB-serial adapter (for testing)

## Planned nodes (in development)
- `robot_navigator.py` — simplified mission state machine
- `vision_node.py` — YOLOv8 object detection at waypoints
- `gui_node.py` — PyQt5 interface (AUTO/MANUAL control + detection display)
