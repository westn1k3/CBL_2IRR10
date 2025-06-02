# obstacle_detection_node.py
# FR3: Obstacle Detection using LiDAR

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')

        # Subscribe to LiDAR scan data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher to alert about obstacles
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_alert', 10)

    def scan_callback(self, msg):
        # Check for any object within 0.4 meters
        close_objects = [r for r in msg.ranges if 0.0 < r < 0.4]
        obstacle_present = len(close_objects) > 0

        alert_msg = Bool()
        alert_msg.data = obstacle_present
        self.obstacle_pub.publish(alert_msg)

        if obstacle_present:
            self.get_logger().info('Obstacle detected nearby!')
        else:
            self.get_logger().info('No obstacles in close range.')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
