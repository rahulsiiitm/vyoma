import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan


class LocalTargetNode(Node):
    def __init__(self):
        super().__init__('local_target_node')
        self.declare_parameter('lookahead_distance', 2.0)
        self.lookahead = float(self.get_parameter('lookahead_distance').value)
        self.path = None
        self.target_pub = self.create_publisher(PoseStamped, '/vyoma/local_target', 10)
        self.blocked_pub = self.create_publisher(Bool, '/vyoma/navigation/blocked', 10)
        self.create_subscription(Path, '/vyoma/global_path', self.path_callback, 10)
        self.create_subscription(Odometry, '/vyoma/odometry', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/vyoma/depth/sectors', self.scan_callback, 10)

    def path_callback(self, msg):
        self.path = msg

    def odom_callback(self, msg):
        if self.path is None or not self.path.poses:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        chosen = self.path.poses[-1]
        for pose in self.path.poses:
            if math.hypot(pose.pose.position.x - x, pose.pose.position.y - y) >= self.lookahead:
                chosen = pose
                break
        chosen.header.stamp = self.get_clock().now().to_msg()
        self.target_pub.publish(chosen)

    def scan_callback(self, msg):
        center = len(msg.ranges) // 2
        half_width = max(1, len(msg.ranges) // 12)
        front = msg.ranges[center - half_width:center + half_width + 1]
        blocked = all(math.isfinite(value) and value < 1.0 for value in front)
        output = Bool()
        output.data = blocked
        self.blocked_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = LocalTargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
