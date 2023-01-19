import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class RLPlanner(Node):
    def __init__(self):
        super().__init__('rl_planner')

        self.path_topic = self.declare_parameter('path_topic', 'offboard/execute_path')
        
        self.path_ = self.create_publisher(Path, path_topic, 10)
