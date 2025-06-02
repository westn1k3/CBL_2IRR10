# digital_twin_node.py
# FR5 + NFR4: Digital Twin interprets sensor values and sends instructions to robots

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class DigitalTwinNode(Node):
    def __init__(self):
        super().__init__('digital_twin_node')

        # Acceptable ranges for pH and moisture
        self.ph_range = (5.5, 7.5)         # Ideal pH range
        self.moisture_range = (30.0, 70.0) # Ideal soil moisture range (percent)

        # Last received sensor values
        self.last_ph = None
        self.last_moisture = None

        # Subscribers to sensor topics
        self.ph_sub = self.create_subscription(Float32, '/sensor/ph', self.ph_callback, 10)
        self.moisture_sub = self.create_subscription(Float32, '/sensor/moisture', self.moisture_callback, 10)

        # Publisher to issue task instructions to simulated robots
        self.instruction_pub = self.create_publisher(String, '/robot_instruction', 10)

    def ph_callback(self, msg):
        self.last_ph = msg.data
        self.evaluate_conditions()

    def moisture_callback(self, msg):
        self.last_moisture = msg.data
        self.evaluate_conditions()

    def evaluate_conditions(self):
        # Only evaluate if we have both values
        if self.last_ph is None or self.last_moisture is None:
            return

        # Log received data
        self.get_logger().info(f"Evaluating: pH={self.last_ph:.2f}, Moisture={self.last_moisture:.2f}")

        instruction = None

        # Decide based on values
        if not self.ph_range[0] <= self.last_ph <= self.ph_range[1]:
            instruction = 'Activate Pesticide Bot (pH out of range)'
        elif self.last_moisture < self.moisture_range[0]:
            instruction = 'Activate Water Bot (Soil too dry)'
        elif self.last_moisture > self.moisture_range[1]:
            instruction = 'Activate Seed Bot (Soil too wet)'
        else:
            instruction = 'Values normal. No action required.'

        # Publish instruction
        msg = String()
        msg.data = instruction
        self.instruction_pub.publish(msg)
        self.get_logger().info(f"Instruction sent: {instruction}")


def main(args=None):
    rclpy.init(args=args)
    node = DigitalTwinNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
