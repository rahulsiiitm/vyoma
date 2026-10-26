import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class DepthSectorNode(Node):
    def __init__(self):
        super().__init__('depth_sector_node')
        self.declare_parameter('sector_count', 64)
        self.declare_parameter('maximum_range', 10.0)
        self.declare_parameter('minimum_height', -1.0)
        self.declare_parameter('maximum_height', 1.0)
        self.count = int(self.get_parameter('sector_count').value)
        self.maximum_range = float(self.get_parameter('maximum_range').value)
        self.minimum_height = float(self.get_parameter('minimum_height').value)
        self.maximum_height = float(self.get_parameter('maximum_height').value)
        self.publisher = self.create_publisher(LaserScan, '/vyoma/depth/sectors', 10)
        self.create_subscription(PointCloud2, '/depth_camera/points', self.callback, 10)

    def callback(self, cloud: PointCloud2) -> None:
        ranges = np.full(self.count, np.inf, dtype=np.float32)
        for x, y, z in point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = float(x), float(y), float(z)
            if not self.minimum_height <= z <= self.maximum_height:
                continue
            distance = math.hypot(x, y)
            if distance <= 0.0 or distance > self.maximum_range:
                continue
            angle = math.atan2(y, x)
            index = min(self.count - 1, int(((angle + math.pi) / (2.0 * math.pi)) * self.count))
            ranges[index] = min(ranges[index], distance)
        scan = LaserScan()
        scan.header = cloud.header
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / self.count
        scan.range_min = 0.1
        scan.range_max = self.maximum_range
        scan.ranges = ranges.tolist()
        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = DepthSectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
