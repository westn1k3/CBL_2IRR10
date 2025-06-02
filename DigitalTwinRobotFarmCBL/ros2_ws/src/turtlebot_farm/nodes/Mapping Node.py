# mapping_node.py
# FR2: Field Mapping using LiDAR data every hour

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import time

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')

        # Subscribe to LiDAR scan data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher for sending mapping report to the digital twin or central system
        self.map_pub = self.create_publisher(String, '/field_map', 10)

        # Store the last mapping time
        self.last_mapping_time = self.get_clock().now().seconds_nanoseconds()[0]

    def scan_callback(self, msg):
        # Called on every new LiDAR scan. Checks if it's time to generate a new map.
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        if current_time - self.last_mapping_time >= 3600:  # 3600 seconds = 1 hour
            self.generate_map(msg)
            self.last_mapping_time = current_time

    def generate_map(self, scan_data):
        # Simulate map generation from LiDAR data
        min_range = min(scan_data.ranges)
        max_range = max(scan_data.ranges)
        anomaly_detected = any(r < 0.3 for r in scan_data.ranges)

        report = f"Map report @ {time.ctime()}\n"
        report += f"Min Distance: {min_range:.2f} m, Max Distance: {max_range:.2f} m\n"
        report += "Anomaly Detected: YES\n" if anomaly_detected else "Anomaly Detected: NO\n"

        # Publish the mapping report
        map_msg = String()
        map_msg.data = report
        self.map_pub.publish(map_msg)

        self.get_logger().info("Map generated and sent:")
        self.get_logger().info(report)

def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
