
"""
Navigator Node:
- Sends a list of navigation goals to the Nav2 action server.
- At each goal, it publishes simulated sensor data (pH and moisture).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import random

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')
        self.goals = [{'x': 1.0, 'y': 0.0}, {'x': 0.0, 'y': 1.0}, {'x': -1.0, 'y': 0.0}]
        self.current_goal_index = 0
        self.goal_active = False

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.sensor_pub = self.create_publisher(Float32MultiArray, 'simulated_sensor_data', 10)
        self.timer = self.create_timer(2.0, self.send_next_goal)

    def send_next_goal(self):
        if self.goal_active or not self.nav_client.wait_for_server(timeout_sec=1.0):
            return
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info('All goals reached.')
            return

        goal = self.goals[self.current_goal_index]
        self.get_logger().info(f"Navigating to: {goal['x']}, {goal['y']}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal['x']
        goal_msg.pose.pose.position.y = goal['y']
        goal_msg.pose.pose.orientation.w = 1.0

        self.goal_active = True
        self.nav_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.goal_active = False
            return
        goal_handle.get_result_async().add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        self.get_logger().info('Goal reached. Publishing fake sensor data...')
        msg = Float32MultiArray()
        msg.data = [random.uniform(4.0, 9.0), random.uniform(10.0, 100.0)]
        self.sensor_pub.publish(msg)
        self.current_goal_index += 1
        self.goal_active = False

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
