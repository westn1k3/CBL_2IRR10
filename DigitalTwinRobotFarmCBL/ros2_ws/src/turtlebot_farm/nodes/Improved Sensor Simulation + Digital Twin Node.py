# anomaly_detector_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class AnomalyDetector(Node):

    def __init__(self):
        super().__init__('anomaly_detector_node')
        self.publisher_ = self.create_publisher(Bool, 'anomaly_detected', 10)
        self.timer = self.create_timer(5.0, self.publish_anomaly)

    def publish_anomaly(self):
        msg = Bool()
        msg.data = True
        self.get_logger().info('Publishing anomaly_detected = True')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# sensor_simulator_node.py
import rclpy
from rclpy.node import Node
import random
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

class SensorSimulator(Node):

    def __init__(self):
        super().__init__('sensor_simulator_node')
        self.subscription = self.create_subscription(
            Bool,
            'anomaly_detected',
            self.anomaly_callback,
            10
        )
        self.publisher_ = self.create_publisher(Float32MultiArray, 'simulated_sensor_data', 10)

    def anomaly_callback(self, msg):
        if msg.data:
            simulated_ph = random.uniform(0, 14)
            simulated_moisture = random.uniform(0, 100)

            self.get_logger().info(f'Generated pH: {simulated_ph:.2f}, Moisture: {simulated_moisture:.2f}')

            data_msg = Float32MultiArray()
            data_msg.data = [simulated_ph, simulated_moisture]

            self.publisher_.publish(data_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# digital_twin_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

class DigitalTwin(Node):

    def __init__(self):
        super().__init__('digital_twin_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'simulated_sensor_data',
            self.sensor_callback,
            10
        )
        self.user_command_subscription = self.create_subscription(
            String,
            'user_command',
            self.user_command_callback,
            10
        )
        self.task_publisher = self.create_publisher(String, 'robot_task_command', 10)

    def sensor_callback(self, msg):
        ph = msg.data[0]
        moisture = msg.data[1]

        self.get_logger().info(f'Received pH: {ph:.2f}, Moisture: {moisture:.2f}')

        if moisture > 85.0:
            action = 'High Moisture Alert'
        elif moisture < 30.0:
            action = '2 rotations (spin twice)'
        elif ph < 5.5 or ph > 7.5:
            action = 'Shake'
        else:
            action = 'No action'

        self.publish_action(action)

    def user_command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'User command received: {command}')

        if command == 'Seeding':
            action = '3 rotations and beep noise'
            self.publish_action(action)

    def publish_action(self, action):
        self.get_logger().info(f'Action triggered: {action}')
        task_msg = String()
        task_msg.data = action
        self.task_publisher.publish(task_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DigitalTwin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# shake_controller_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class ShakeController(Node):

    def __init__(self):
        super().__init__('shake_controller_node')
        self.subscription = self.create_subscription(
            String,
            'robot_task_command',
            self.task_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def task_callback(self, msg):
        task = msg.data
        self.get_logger().info(f'Received task: {task}')

        if task == 'Shake':
            self.shake()
        elif '2 rotations' in task:
            self.spin(2)
        elif '3 rotations' in task:
            self.spin(3)
            self.beep()

    def shake(self):
        self.get_logger().info('Executing shake...')
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

    def spin(self, rotations):
        self.get_logger().info(f'Executing {rotations} rotations...')
        twist = Twist()
        twist.angular.z = 1.0
        duration = rotations * 2.0  # Approximation
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def beep(self):
        self.get_logger().info('Beep sound simulated! (play sound here if desired)')

def main(args=None):
    rclpy.init(args=args)
    node = ShakeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# user_command_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class UserCommandPublisher(Node):

    def __init__(self):
        super().__init__('user_command_publisher')
        self.publisher_ = self.create_publisher(String, 'user_command', 10)
        self.timer = self.create_timer(10.0, self.publish_command)

    def publish_command(self):
        msg = String()
        msg.data = 'Seeding'
        self.get_logger().info(f'Publishing user command: {msg.data}')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UserCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# farming_poc.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='farming_poc',
            executable='anomaly_detector_node',
            name='anomaly_detector_node'
        ),
        Node(
            package='farming_poc',
            executable='sensor_simulator_node',
            name='sensor_simulator_node'
        ),
        Node(
            package='farming_poc',
            executable='digital_twin_node',
            name='digital_twin_node'
        ),
        Node(
            package='farming_poc',
            executable='shake_controller_node',
            name='shake_controller_node'
        ),
        Node(
            package='farming_poc',
            executable='user_command_publisher',
            name='user_command_publisher'
        )
    ])


How to launch
# Build your workspace
colcon build –packages-select farming_poc

# Source
source install/setup.bash

# Launch EVERYTHING
ros2 launch farming_poc farming_poc.launch.py


