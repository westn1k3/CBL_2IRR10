# navigation_node.py
# FR1: Autonomous Navigation using LiDAR and SLAM

import rclpy  # ROS2 client library for Python
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import random

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Publisher to control robot's movement (linear and angular velocity)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to robot's odometry info (position and orientation)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Subscriber to LaserScan data from LiDAR
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Timer to control navigation loop (every 0.5s)
        self.timer = self.create_timer(0.5, self.navigate)

        self.obstacle_detected = False

    def odom_callback(self, msg):
        # This function gets called every time a new Odometry message is received
        # Odometry gives the robot's current position and orientation
        pass  # In this simple example, we are not using this directly

    def scan_callback(self, msg):
        # This function is called with new LaserScan data
        # We check if there's an obstacle within 0.5 meters in front of the robot
        min_distance = min(msg.ranges)
        if min_distance < 0.5:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def navigate(self):
        # This function decides how the robot should move
        twist = Twist()

        if self.obstacle_detected:
            # If obstacle is detected, rotate in place
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Turn right
            self.get_logger().info('Obstacle detected! Turning...')
        else:
            # Move forward
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.get_logger().info('Path clear. Moving forward...')

        # Send movement command to robot
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
