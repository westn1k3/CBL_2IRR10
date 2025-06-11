# improved_navigation_node2.py
# Enhanced ROS 2 navigation node for obstacle avoidance and recovery

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class ImprovedNavigationNode2(Node):
    def __init__(self):
        super().__init__('improved_navigation_node')

        # Set up ROS 2 publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # For sending velocity commands
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)  # For position updates
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)  # For obstacle detection

        # Declare and fetch parameters for tuning
        self.declare_parameter('min_obstacle_distance', 0.8)
        self.declare_parameter('side_obstacle_distance', 0.6)
        self.declare_parameter('max_linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.3)

        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.side_obstacle_distance = self.get_parameter('side_obstacle_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        # Initialize obstacle detection flags
        self.front_obstacle = False
        self.left_obstacle = False
        self.right_obstacle = False
        self.front_left_obstacle = False
        self.front_right_obstacle = False

        # State tracking for turns and recovery
        self.turning_duration = 0
        self.max_turning_duration = 10  # Maximum turns before recovery
        self.stuck_counter = 0
        self.max_stuck_counter = 20     # Frames with no movement = stuck
        self.recovery_cooldown = 0
        self.recovery_cooldown_duration = 5  # Wait time after recovery

        # Position tracking for stuck detection
        self.current_x = 0.0
        self.current_y = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.position_change_threshold = 0.05  # Minimum movement considered "progress"

        # Last command sent (used to decide if we expected motion)
        self.last_cmd = Twist()

        # Store last laser scan message
        self.laser_data = None

        # For throttling clear-path logging
        self.log_counter = 0

        # Run the navigation logic periodically
        self.timer = self.create_timer(1.0, self.navigate)

        self.get_logger().info("Improved navigation node started")

    def odom_callback(self, msg):
        # Update current and previous position for motion analysis
        self.last_x = self.current_x
        self.last_y = self.current_y
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def scan_callback(self, msg):
        # Handle incoming laser scan data and classify obstacles in different directions
        if not msg.ranges:
            return

        # Clean the data by replacing inf/nan with max range
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        ranges = np.where(np.isnan(ranges), msg.range_max, ranges)

        # Calculate angles for each laser reading
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        angles = np.arange(angle_min, angle_max, angle_increment)
        if len(angles) > len(ranges):
            angles = angles[:len(ranges)]

        # Utility: extract laser values within an angle range
        def extract_ranges(start_angle, end_angle):
            indices = np.where((angles >= start_angle) & (angles <= end_angle))[0]
            return ranges[indices] if len(indices) > 0 else []

        # Check for obstacles in various sectors
        self.front_obstacle = np.min(extract_ranges(-math.radians(30), math.radians(30)), initial=msg.range_max) < self.min_obstacle_distance
        self.left_obstacle = np.min(extract_ranges(math.radians(60), math.radians(120)), initial=msg.range_max) < self.side_obstacle_distance
        self.right_obstacle = np.min(extract_ranges(-math.radians(120), -math.radians(60)), initial=msg.range_max) < self.side_obstacle_distance
        self.front_left_obstacle = np.min(extract_ranges(math.radians(30), math.radians(60)), initial=msg.range_max) < self.side_obstacle_distance
        self.front_right_obstacle = np.min(extract_ranges(-math.radians(60), -math.radians(30)), initial=msg.range_max) < self.side_obstacle_distance

        self.laser_data = msg  # Store latest scan

    def is_stuck(self):
        # Determine if the robot is stuck: it tried to move but didn't
        position_change = math.hypot(self.current_x - self.last_x, self.current_y - self.last_y)
        intended_motion = abs(self.last_cmd.linear.x) > 0.05 or abs(self.last_cmd.angular.z) > 0.1
        return intended_motion and position_change < self.position_change_threshold

    def navigate(self):
        # Main navigation loop
        twist = Twist()

        # Stop if no scan data available
        if self.laser_data is None:
            self.get_logger().error("No laser scan data! Stopping for safety.")
            self.cmd_vel_pub.publish(twist)
            return

        # Wait after a recovery before navigating again
        if self.recovery_cooldown > 0:
            self.recovery_cooldown -= 1
            return

        # Check if the robot is stuck
        if self.is_stuck():
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        # Recovery behavior if stuck
        if self.stuck_counter > self.max_stuck_counter:
            self.get_logger().warn("Robot is stuck! Executing recovery.")
            twist.linear.x = -0.1  # Back up
            twist.angular.z = 0.5  # Turn
            self.cmd_vel_pub.publish(twist)
            self.recovery_cooldown = self.recovery_cooldown_duration
            self.stuck_counter = 0
            return

        # Navigation logic based on obstacles
        if self.front_obstacle:
            self.turning_duration += 1
            if self.turning_duration > self.max_turning_duration:
                # Recovery if turning doesn't help
                self.get_logger().warn("Turning too long, backing up")
                twist.linear.x = -0.1
                twist.angular.z = 0.0
                self.turning_duration = 0
            else:
                # Try turning to avoid obstacle
                twist.linear.x = 0.0
                if not self.right_obstacle and not self.front_right_obstacle:
                    twist.angular.z = -self.max_angular_speed
                    self.get_logger().info("Obstacle ahead! Turning right")
                elif not self.left_obstacle and not self.front_left_obstacle:
                    twist.angular.z = self.max_angular_speed
                    self.get_logger().info("Obstacle ahead! Turning left")
                else:
                    twist.linear.x = -0.1
                    twist.angular.z = self.max_angular_speed
                    self.get_logger().warn("Obstacles on all sides! Reversing and turning")

        elif self.front_left_obstacle or self.front_right_obstacle:
            # Slight turn to avoid side-front obstacles
            twist.linear.x = self.max_linear_speed * 0.5
            if self.front_left_obstacle and not self.front_right_obstacle:
                twist.angular.z = -0.2
                self.get_logger().info("Front-left obstacle, adjusting right")
            elif self.front_right_obstacle and not self.front_left_obstacle:
                twist.angular.z = 0.2
                self.get_logger().info("Front-right obstacle, adjusting left")
            else:
                twist.angular.z = 0.0
            self.turning_duration = 0

        else:
            # No obstacles, move forward
            twist.linear.x = self.max_linear_speed
            twist.angular.z = 0.0
            self.turning_duration = 0
            self.log_counter += 1
            if self.log_counter >= 5:
                self.get_logger().info("Path is clear, moving forward")
                self.log_counter = 0

        # Emergency stop if something is too close
        center_idx = len(self.laser_data.ranges) // 2
        check_range = 10
        start_idx = max(0, center_idx - check_range)
        end_idx = min(len(self.laser_data.ranges), center_idx + check_range)
        front_center_ranges = self.laser_data.ranges[start_idx:end_idx]
        if front_center_ranges and min(front_center_ranges) < 0.3:
            self.get_logger().error(f"Emergency stop! Obstacle too close: {min(front_center_ranges):.2f}m")
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Save and send the command
        self.last_cmd = twist
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ImprovedNavigationNode2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
