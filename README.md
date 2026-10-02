# LiDAR-Based Autonomous Maze Solver Robot

An autonomous mobile robotic platform designed to navigate and resolve uncharted, congested mazes using real-time 2D LiDAR rangefinding, reactive obstacle avoidance, and spatial sector analysis.

---

## System Overview

The system processes 360° laser scan data to dynamically predict corridor boundaries, avoid collisions, and execute fast pivoting maeuvers in dead-ends or complex junctions.

---

## System Architecture

```text
       [ 360° 2D LiDAR Sensor ]
                    |
                    | (Sensor_msgs/LaserScan)
                    v
       +----------------------------+
       |    Spatial Sector Filter    |
       | (Range Validation & Min Dist) |
       +---------------+--------------+
                    |
                    v Sector Clearances (Front, Left, Right)
       +---------------+--------------+
       | Reactive Navigation Controller |
       |  (Proportional Wall Following)  |
       +---------------+--------------+|
                    |
                    v (Geometry_msgs/Twist)
       [ Drive train Motor Controller ]
```

---

## Key Capabilities

* **Real-Time 360° Mapping:** Subscribes to `/scan` and processes range vectors with numpy filtering for low-latency clearance estimation.
* **Cadenced Wall Following:** Uses proportional error correction to maintain a fixed distance along maze walls without oscillation.
* **Deada-End & Corner Detection:** Detects confined corners and performs autonomous reversal-and-pivot maneuvers to exit dead ends consistently.

---

## Tech Stack

* *Middleware:** ROS 2 Humble
* *Languages:** Python 3.10 / C++
 * *Sensors:** 2D LiDAR (RPLiDAR or equivalent 360° rangefinder)
 * *Dependencies:** NumPy, rclpy, geometry_msgs, sensor_msgs

---

## Execution

```bash
# Start the maze solver node
python3 src/maze_solver.py
```

---

## Author
* **Likith K** — Robotics & Automation Engineer
