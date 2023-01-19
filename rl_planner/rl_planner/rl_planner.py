import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from px4_autonomous_interfaces.action import ExecutePath

class RLPlanner(Node):
    def __init__(self):
        super().__init__('rl_planner')

        self.execute_path_action = self.declare_parameter('execute_path_action', 'execute_path')
        self.execute_path_action_client = ActionClient(self, ExecutePath, 'execute_path')

    def send_goal(self, path):
        goal_msg = ExecutePath.Goal()
        goal_msg.path = path

        self.execute_path_action_client.wait_for_server()

        return self.execute_path_action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    action_client = RLPlanner()

    pose1 = PoseStamped()
    pose1.pose.position.x = 1.0
    pose1.pose.position.y = 1.0
    pose1.pose.position.z = 1.0
    pose2 = PoseStamped()
    pose2.pose.position.x = 2.0
    pose2.pose.position.y = 2.0
    pose2.pose.position.z = 2.0
    pose3 = PoseStamped()
    pose3.pose.position.x = 3.0
    pose3.pose.position.y = 3.0
    pose3.pose.position.z = 3.0
    poses = [pose1, pose2, pose3]
    path = Path(poses=poses)
    future = action_client.send_goal(path)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
