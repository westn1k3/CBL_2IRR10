# improved_navigation_node.py
# Improved navigation node with wall-avoidance fixes

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class ImprovedNavigationNode(Node):
    def __init__(self):
        super().__init__('improved_navigation_node')

        # Publisher and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Navigation parameters - key improvements
        self.min_obstacle_distance = 0.8  # Increased from 0.5m to avoid collisions
        self.side_obstacle_distance = 0.6  # Side detection threshold
        self.max_linear_speed = 0.15       # Slower max speed for better control
        self.max_angular_speed = 0.3       # Slower turn rate

        # Obstacle detection in multiple directions
        self.front_obstacle = False
        self.left_obstacle = False
        self.right_obstacle = False
        self.front_left_obstacle = False
        self.front_right_obstacle = False

        # Turning and stuck state tracking
        self.turning_duration = 0
        self.max_turning_duration = 10  # Max duration to avoid endless turning
        self.stuck_counter = 0
        self.max_stuck_counter = 20     # Stuck detection threshold

        # Position tracking for stuck detection
        self.current_x = 0.0
        self.current_y = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.position_change_threshold = 0.05  # Minimal position change to consider movement

        # Laser scan data storage
        self.laser_data = None

        # Navigation timer - increased to 1s for more reaction time
        self.timer = self.create_timer(1.0, self.navigate)

        self.get_logger().info("Improved navigation node started - Wall avoidance mode")

    def odom_callback(self, msg):
        """Track current position for stuck detection"""
        self.last_x = self.current_x
        self.last_y = self.current_y
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def scan_callback(self, msg):
        """Enhanced laser scan processing - detect obstacles in multiple directions"""
        if not msg.ranges:
            return

        self.laser_data = msg
        ranges = np.array(msg.ranges)

        # Clean invalid data
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        ranges = np.where(np.isnan(ranges), msg.range_max, ranges)

        total_points = len(ranges)

        # Front (±30 degrees)
        front_start = int(total_points * 0.83)  # -30 deg
        front_end = int(total_points * 0.17)    # +30 deg
        front_ranges = np.concatenate([ranges[front_start:], ranges[:front_end]])

        # Left (60-120 degrees)
        left_start = int(total_points * 0.17)
        left_end = int(total_points * 0.33)
        left_ranges = ranges[left_start:left_end] if left_end > left_start else []

        # Right (240-300 degrees)
        right_start = int(total_points * 0.67)
        right_end = int(total_points * 0.83)
        right_ranges = ranges[right_start:right_end] if right_end > right_start else []

        # Front-left (30-60 degrees)
        front_left_start = int(total_points * 0.08)
        front_left_end = int(total_points * 0.17)
        front_left_ranges = ranges[front_left_start:front_left_end] if front_left_end > front_left_start else []

        # Front-right (300-330 degrees)
        front_right_start = int(total_points * 0.83)
        front_right_end = int(total_points * 0.92)
        front_right_ranges = ranges[front_right_start:front_right_end] if front_right_end > front_right_start else []

        # Obstacle detection flags
        self.front_obstacle = len(front_ranges) > 0 and np.min(front_ranges) < self.min_obstacle_distance
        self.left_obstacle = len(left_ranges) > 0 and np.min(left_ranges) < self.side_obstacle_distance
        self.right_obstacle = len(right_ranges) > 0 and np.min(right_ranges) < self.side_obstacle_distance
        self.front_left_obstacle = len(front_left_ranges) > 0 and np.min(front_left_ranges) < self.side_obstacle_distance
        self.front_right_obstacle = len(front_right_ranges) > 0 and np.min(front_right_ranges) < self.side_obstacle_distance

        # Debugging obstacle distance
        if len(front_ranges) > 0:
            min_front_dist = np.min(front_ranges)
            if min_front_dist < self.min_obstacle_distance:
                self.get_logger().debug(f"Front obstacle detected at: {min_front_dist:.2f}m")

    def is_stuck(self):
        """Determine if the robot is stuck based on movement threshold"""
        position_change = math.sqrt((self.current_x - self.last_x)**2 + (self.current_y - self.last_y)**2)
        return position_change < self.position_change_threshold

    def navigate(self):
        """Improved navigation logic"""
        twist = Twist()

        # Stuck detection
        if self.is_stuck():
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        # Recovery behavior if stuck
        if self.stuck_counter > self.max_stuck_counter:
            self.get_logger().warn("Robot is stuck! Executing recovery")
            twist.linear.x = -0.1
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
            self.stuck_counter = 0
            return

        # Main navigation logic
        if self.front_obstacle:
            self.turning_duration += 1

            if self.turning_duration > self.max_turning_duration:
                self.get_logger().warn("Turning too long, backing up")
                twist.linear.x = -0.1
                twist.angular.z = 0.0
                self.turning_duration = 0
            else:
                twist.linear.x = 0.0

                if not self.right_obstacle and not self.front_right_obstacle:
                    twist.angular.z = -self.max_angular_speed
                    self.get_logger().info('Obstacle ahead! Turning right')
                elif not self.left_obstacle and not self.front_left_obstacle:
                    twist.angular.z = self.max_angular_speed
                    self.get_logger().info('Obstacle ahead! Turning left')
                else:
                    twist.linear.x = -0.1
                    twist.angular.z = self.max_angular_speed
                    self.get_logger().warn('Obstacles on all sides! Reversing and turning')

        elif self.front_left_obstacle or self.front_right_obstacle:
            twist.linear.x = self.max_linear_speed * 0.5

            if self.front_left_obstacle and not self.front_right_obstacle:
                twist.angular.z = -0.2
                self.get_logger().info('Front-left obstacle, adjusting right')
            elif self.front_right_obstacle and not self.front_left_obstacle:
                twist.angular.z = 0.2
                self.get_logger().info('Front-right obstacle, adjusting left')
            else:
                twist.angular.z = 0.0

            self.turning_duration = 0

        else:
            twist.linear.x = self.max_linear_speed
            twist.angular.z = 0.0
            self.turning_duration = 0
            self.get_logger().info('Path is clear, moving forward')

        # Final safety check for close-range obstacle
        if self.laser_data and len(self.laser_data.ranges) > 0:
            center_idx = len(self.laser_data.ranges) // 2
            check_range = 10
            start_idx = max(0, center_idx - check_range)
            end_idx = min(len(self.laser_data.ranges), center_idx + check_range)

            front_center_ranges = self.laser_data.ranges[start_idx:end_idx]
            if front_center_ranges and min(front_center_ranges) < 0.3:
                self.get_logger().error(f"Emergency stop! Obstacle too close: {min(front_center_ranges):.2f}m")
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ImprovedNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
