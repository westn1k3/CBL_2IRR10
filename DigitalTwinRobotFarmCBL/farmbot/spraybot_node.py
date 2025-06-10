
"""
SprayBot Node:
- Subscribes to sensor data.
- If pH is outside normal range, performs shaking motion to simulate spraying.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import time

class SprayBotNode(Node):
    def __init__(self):
        super().__init__('spraybot_node')
        self.subscription = self.create_subscription(Float32MultiArray, 'simulated_sensor_data', self.sensor_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def sensor_callback(self, msg):
        ph = msg.data[0]
        self.get_logger().info(f'pH: {ph:.2f}')
        if ph < 5.5 or ph > 7.5:
            self.get_logger().info('Abnormal pH detected. Spraying...')
            self.shake()

    def shake(self):
        twist = Twist()
        for _ in range(5):
            twist.angular.z = 1.0
            self.cmd_pub.publish(twist)
            time.sleep(0.2)
            twist.angular.z = -1.0
            self.cmd_pub.publish(twist)
            time.sleep(0.2)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SprayBotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
