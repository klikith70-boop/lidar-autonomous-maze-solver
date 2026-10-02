#!/usr/bin/env python3
"""
Autonomous Maze Solver Node
Subscribes: /scan (sensor_msgs/LaserScan)
Publishes:  /cmd_vel (geometry_msgs/Twist)
Implements: Reactive LiDAR wall-following and obstacle clearance control.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class LidarMazeSolver(Node):
    def __init__(self):
        super().__init__('lidar_maze_solver')
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Operational Thresholds (meters)
        self.safe_front_distance = 0.40
        self.target_wall_distance = 0.35
        self.critical_stop_distance = 0.18
        
        self.get_logger().info("LiDAR Autonomous Maze Solver node initialized.")

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        # Filter inf and NaN values with max range
        ranges = np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max)

        num_readings = len(ranges)
        if num_readings == 0:
            return

        # Sector segmentation (Front, Left, Right)
        front_window = int(num_readings * (30 / 360))
        left_center = int(num_readings * (90 / 360))
        right_center = int(num_readings * (270 / 360))
        side_window = int(num_readings * (20 / 360))

        dist_front = min(min(ranges[:front_window]), min(ranges[-front_window:]))
        dist_left = min(ranges[left_center - side_window : left_center + side_window])
        dist_right = min(ranges[right_center - side_window : right_center + side_window])

        twist = Twist()

        # State Machine: Obstacle Avoidance & Wall Following
        if dist_front < self.critical_stop_distance:
            # Immediate recovery: reverse slightly and turn
            twist.linear.x = -0.05
            twist.angular.z = 0.5 if dist_left > dist_right else -0.5
        elif dist_front < self.safe_front_distance:
            # Corner / Dead-end detected: pivot towards open path
            twist.linear.x = 0.0
            twist.angular.z = 0.6 if dist_left > dist_right else -0.6
        else:
            # Corridor following: Maintain nominal forward speed with proportional steering
            twist.linear.x = 0.22
            error = self.target_wall_distance - dist_left
            twist.angular.z = -1.2 * error  # Proportional corrective bias

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LidarMazeSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())  # Send stop command on shutdown
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
