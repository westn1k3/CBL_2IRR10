
"""
SeedingBot Node:
- Subscribes to a topic with coordinates selected for planting.
- When a goal is received, performs 3 simulated rotations.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class SeedingBotNode(Node):
    def __init__(self):
        super().__init__('seedingbot_node')
        self.subscription = self.create_subscription(String, 'seeding_command', self.task_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def task_callback(self, msg):
        self.get_logger().info(f'Received seeding command at {msg.data}')
        self.spin(3)

    def spin(self, rotations):
        twist = Twist()
        twist.angular.z = 1.0
        duration = rotations * 2.0
        start = self.get_clock().now().seconds_nanoseconds()[0]
        while self.get_clock().now().seconds_nanoseconds()[0] - start < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info('Finished planting')

def main(args=None):
    rclpy.init(args=args)
    node = SeedingBotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
