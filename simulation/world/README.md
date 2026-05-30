# World File — my_world.sdf

## Contents
- Ground plane
- 4 walls (north/south/east/west)
- `unit_box` — static obstacle at (1.51, -0.18) on leg A→B
- `unit_sphere` — static obstacle at (-1.89, 2.36)
- Furniture models (from Gazebo Fuel):
  - WoodenChair at (6, -1)
  - LampAndStand at (-1, -1)
  - Table (1.5×0.8m) at (1.86, -0.01) — replaces TVStand
  - Suitcase1 at (6, 3)
  - TrashBin at (-1, 3)
  - SmallTrolley at (1.49, -3.36)

## Fuel Models Required
```bash
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/WoodenChair"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/LampAndStand"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/table"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Suitcase1"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/TrashBin"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/SmallTrolley"
```

## Dynamic Obstacle
Spawned programmatically by `robot_navigator.py` when robot reaches waypoint B.
Red 0.5m box moves horizontally: x = 3.5 ↔ 7.5 at y = 1.25, speed = 0.27 m/s.
