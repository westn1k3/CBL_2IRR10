# farm_launch_two.py 
# Launch file to start all TurtleBot simulation nodes

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot_farm',
            executable='improved_navigation_node',  # use the improved navigation node
            name='navigation_node'
        ),
        Node(
            package='turtlebot_farm',
            executable='mapping_node',
            name='mapping_node'
        ),
        Node(
            package='turtlebot_farm',
            executable='obstacle_detection_node',
            name='obstacle_detection_node'
        ),
        Node(
            package='turtlebot_farm',
            executable='sensor_simulation_node',
            name='sensor_simulation_node'
        ),
        Node(
            package='turtlebot_farm',
            executable='digital_twin_node',
            name='digital_twin_node'
        ),
    ])
