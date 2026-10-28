import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_client')
        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.path_pub = self.create_publisher(Path, '/vyoma/global_path', 10)
        self.goal = None
        # Planning is requested by the mission manager after takeoff, when the
        # horizontal costmap represents flight-level obstacles rather than floor.
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Bool, '/vyoma/navigation/blocked', self.blocked_callback, 10)

    def goal_callback(self, msg):
        self.goal = msg
        self.request_path()

    def blocked_callback(self, msg):
        if msg.data and self.goal is not None:
            self.request_path()

    def request_path(self):
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 planner is not available')
            return
        request = ComputePathToPose.Goal()
        request.goal = self.goal
        request.use_start = False
        future = self.client.send_goal_async(request)
        future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Global plan request rejected')
            return
        result = handle.get_result_async()
        result.add_done_callback(lambda completed: self.path_pub.publish(completed.result().result.path))


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
