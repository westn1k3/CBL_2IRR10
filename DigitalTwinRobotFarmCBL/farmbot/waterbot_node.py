
"""
WaterBot Node:
- Subscribes to sensor data.
- If moisture < 30, it performs 2 rotations to simulate watering.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import time

class WaterBotNode(Node):
    def __init__(self):
        super().__init__('waterbot_node')
        self.subscription = self.create_subscription(Float32MultiArray, 'simulated_sensor_data', self.sensor_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def sensor_callback(self, msg):
        moisture = msg.data[1]
        self.get_logger().info(f'Moisture: {moisture:.2f}')
        if moisture < 30.0:
            self.get_logger().info('Low moisture detected. Watering...')
            self.spin(2)

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

def main(args=None):
    rclpy.init(args=args)
    node = WaterBotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
