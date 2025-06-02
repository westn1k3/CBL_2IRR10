# sensor_simulation_node.py
# FR4: Simulated Measurement Routine (pH and Moisture at Anomalies)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import random
import time

class SensorSimulationNode(Node):
    def __init__(self):
        super().__init__('sensor_simulation_node')

        # Publishers for simulated sensor readings
        self.ph_pub = self.create_publisher(Float32, '/sensor/ph', 10)
        self.moisture_pub = self.create_publisher(Float32, '/sensor/moisture', 10)

        # Publisher to notify when measurement is taken
        self.measurement_pub = self.create_publisher(String, '/sensor/log', 10)

        # Timer to simulate measurement every 30 seconds (representing anomaly stops)
        self.timer = self.create_timer(30.0, self.take_measurement)

    def take_measurement(self):
        # Simulate pH and moisture sensor values using a random number generator
        simulated_ph = random.uniform(4.5, 8.5)  # Simulated pH scale (acidic to basic)
        simulated_moisture = random.uniform(10.0, 90.0)  # Simulated % moisture

        # Publish the simulated readings
        ph_msg = Float32()
        ph_msg.data = simulated_ph
        self.ph_pub.publish(ph_msg)

        moisture_msg = Float32()
        moisture_msg.data = simulated_moisture
        self.moisture_pub.publish(moisture_msg)

        # Log the measurement
        log = String()
        log.data = f"Measurement taken at {time.ctime()}: pH={simulated_ph:.2f}, Moisture={simulated_moisture:.2f}%"
        self.measurement_pub.publish(log)

        self.get_logger().info(log.data)

def main(args=None):
    rclpy.init(args=args)
    node = SensorSimulationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
