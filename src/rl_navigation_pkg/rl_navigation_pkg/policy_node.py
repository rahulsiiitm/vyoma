import math
import os
import json

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger

from .observation import (OBSERVATION_SIZE, NavigationFeatures, ObservationHistory,
                          SCHEMA_VERSION, encode)


class PolicyNode(Node):
    """Runs a validated PPO policy, with a deterministic commissioning baseline."""

    def __init__(self):
        super().__init__('rl_policy_node')
        self.declare_parameter('model_path', '')
        self.declare_parameter('allow_fallback', True)
        self.declare_parameter('maximum_range', 10.0)
        self.model = None
        model_path = str(self.get_parameter('model_path').value)
        if model_path:
            from stable_baselines3 import PPO
            if not os.path.exists(model_path):
                raise FileNotFoundError(model_path)
            metadata_path = (model_path[:-4] if model_path.endswith('.zip') else model_path) + '.metadata.json'
            if not os.path.exists(metadata_path):
                raise RuntimeError(f'Model metadata is required: {metadata_path}')
            with open(metadata_path, encoding='utf-8') as metadata_file:
                metadata = json.load(metadata_file)
            if metadata.get('schema_version') != SCHEMA_VERSION:
                raise RuntimeError('Model observation schema is incompatible')
            if metadata.get('observation_size') != OBSERVATION_SIZE or metadata.get('action_size') != 3:
                raise RuntimeError('Model tensor shape metadata is incompatible')
            self.model = PPO.load(model_path)
            self.get_logger().info(f'Loaded PPO policy: {model_path}, schema={SCHEMA_VERSION}')
        elif not bool(self.get_parameter('allow_fallback').value):
            raise RuntimeError('A validated PPO model is required')
        else:
            self.get_logger().warn('No PPO model configured; using commissioning baseline')
        self.maximum_range = float(self.get_parameter('maximum_range').value)
        self.history = ObservationHistory()
        self.scan = self.odom = self.target = self.goal = self.path = None
        self.previous_action = np.zeros(3, dtype=np.float32)
        self.observation_pub = self.create_publisher(Float32MultiArray, '/vyoma/rl/observation', 10)
        self.action_pub = self.create_publisher(TwistStamped, '/vyoma/rl/raw_action', 10)
        self.create_subscription(LaserScan, '/vyoma/depth/sectors', self.scan_callback, 10)
        self.create_subscription(Odometry, '/vyoma/odometry', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/vyoma/local_target', self.target_callback, 10)
        self.create_subscription(PoseStamped, '/vyoma/mission/command', self.goal_callback, 10)
        self.create_subscription(Path, '/vyoma/global_path', self.path_callback, 10)
        self.create_service(Trigger, '/vyoma/policy/reset', self.reset_callback)
        self.create_timer(0.1, self.control_loop)

    def reset_callback(self, request, response):
        del request
        self.history.clear()
        self.scan = self.odom = self.target = self.goal = self.path = None
        self.previous_action.fill(0.0)
        response.success = True
        response.message = 'policy observation history reset'
        return response

    def scan_callback(self, msg): self.scan = msg
    def odom_callback(self, msg): self.odom = msg
    def target_callback(self, msg): self.target = msg
    def goal_callback(self, msg): self.goal = msg
    def path_callback(self, msg): self.path = msg

    @staticmethod
    def relative_goal(pose, goal):
        dx = goal.position.x - pose.position.x
        dy = goal.position.y - pose.position.y
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        bearing = (math.atan2(dy, dx) - yaw + math.pi) % (2.0 * math.pi) - math.pi
        return math.hypot(dx, dy), bearing

    def control_loop(self):
        if any(value is None for value in (self.scan, self.odom, self.target, self.goal)):
            return
        ranges = np.asarray(self.scan.ranges, dtype=np.float32)
        valid = np.isfinite(ranges)
        clean = np.where(valid, ranges, self.maximum_range)
        local_distance, local_bearing = self.relative_goal(self.odom.pose.pose, self.target.pose)
        goal_distance, goal_bearing = self.relative_goal(self.odom.pose.pose, self.goal.pose)
        cross_track = 0.0
        if self.path is not None and self.path.poses:
            x = self.odom.pose.pose.position.x
            y = self.odom.pose.pose.position.y
            cross_track = min(math.hypot(item.pose.position.x - x, item.pose.position.y - y)
                              for item in self.path.poses)
        features = NavigationFeatures(
            ranges=clean, valid=valid, local_distance=local_distance, local_bearing=local_bearing,
            goal_distance=goal_distance, goal_bearing=goal_bearing, cross_track_error=cross_track,
            heading_error=local_bearing, velocity_forward=self.odom.twist.twist.linear.x,
            velocity_lateral=self.odom.twist.twist.linear.y,
            yaw_rate=self.odom.twist.twist.angular.z, previous_action=self.previous_action,
            path_blocked=bool(np.min(clean) < 0.75), maximum_range=self.maximum_range)
        frame = encode(features)
        observation = self.history.append(frame)
        observation_msg = Float32MultiArray()
        dimension = MultiArrayDimension()
        dimension.label = f'schema={SCHEMA_VERSION};stamp_ns={self.get_clock().now().nanoseconds}'
        dimension.size = len(observation)
        dimension.stride = len(observation)
        observation_msg.layout.dim = [dimension]
        observation_msg.data = observation.tolist()
        self.observation_pub.publish(observation_msg)
        if self.model is not None:
            normalized, _ = self.model.predict(observation, deterministic=True)
            normalized = np.clip(np.asarray(normalized, dtype=np.float32), -1.0, 1.0)
        else:
            normalized = self.baseline_action(clean, local_bearing)
        action = np.asarray([2.0 * normalized[0], 1.5 * normalized[1], normalized[2]], dtype=np.float32)
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.twist.linear.x = float(action[0])
        command.twist.linear.y = float(action[1])
        command.twist.angular.z = float(action[2])
        self.action_pub.publish(command)
        self.previous_action = normalized

    @staticmethod
    def baseline_action(ranges, target_bearing):
        count = len(ranges)
        front_width = max(1, count // 12)
        center = count // 2
        front_indices = list(range(center - front_width, center + front_width + 1))
        front = float(np.min(ranges[front_indices]))
        if front < 2.0:
            left = float(np.mean(ranges[:count // 2]))
            right = float(np.mean(ranges[count // 2:]))
            lateral = 1.0 if left > right else -1.0
            return np.asarray([max(0.0, (front - 0.75) / 1.25) * 0.35,
                               lateral, np.clip(target_bearing / math.pi, -1.0, 1.0)], dtype=np.float32)
        return np.asarray([min(1.0, max(0.0, math.cos(target_bearing))),
                           np.clip(math.sin(target_bearing), -0.5, 0.5),
                           np.clip(target_bearing / math.pi, -1.0, 1.0)], dtype=np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
