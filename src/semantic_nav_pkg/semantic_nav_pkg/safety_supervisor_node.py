import json
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .safety import SafetyLimits, filter_action


class SafetySupervisorNode(Node):
    def __init__(self):
        super().__init__('safety_supervisor')
        for name, default in (
            ('max_forward', 2.0), ('max_lateral', 1.5), ('max_yaw_rate', 1.0),
            ('max_acceleration', 2.0), ('hard_clearance', 0.75),
            ('warning_clearance', 1.5), ('input_timeout', 0.5),
        ):
            self.declare_parameter(name, default)
        self.limits = SafetyLimits(**{
            key: float(self.get_parameter(key).value) for key in
            ('max_forward', 'max_lateral', 'max_yaw_rate', 'max_acceleration',
             'hard_clearance', 'warning_clearance')
        })
        self.timeout = float(self.get_parameter('input_timeout').value)
        self.last_scan = None
        self.last_scan_time = None
        self.previous = (0.0, 0.0, 0.0)
        self.last_time = self.get_clock().now()
        self.safe_pub = self.create_publisher(TwistStamped, '/vyoma/control/safe_command', 10)
        self.status_pub = self.create_publisher(String, '/vyoma/safety/status', 10)
        self.create_subscription(LaserScan, '/vyoma/depth/sectors', self.scan_callback, 10)
        self.create_subscription(TwistStamped, '/vyoma/rl/raw_action', self.action_callback, 10)
        self.create_service(Trigger, '/vyoma/safety/reset', self.reset_callback)
        self.last_action_time = None
        self.create_timer(0.1, self.watchdog)

    def reset_callback(self, request, response):
        del request
        self.last_scan = None
        self.last_scan_time = None
        self.last_action_time = None
        self.previous = (0.0, 0.0, 0.0)
        self.last_time = self.get_clock().now()
        stop = TwistStamped()
        stop.header.stamp = self.last_time.to_msg()
        stop.header.frame_id = 'base_link'
        self.safe_pub.publish(stop)
        response.success = True
        response.message = 'safety state reset'
        return response

    def scan_callback(self, msg):
        finite = [value for value in msg.ranges if math.isfinite(value) and value >= msg.range_min]
        self.last_scan = min(finite) if finite else msg.range_max
        self.last_scan_time = self.get_clock().now()

    def action_callback(self, msg):
        now = self.get_clock().now()
        self.last_action_time = now
        dt = max(0.001, (now - self.last_time).nanoseconds / 1e9)
        fresh = self.last_scan_time is not None and (now - self.last_scan_time).nanoseconds / 1e9 <= self.timeout
        result = filter_action(
            (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z), self.previous,
            self.last_scan if self.last_scan is not None else 0.0, dt, fresh, self.limits)
        output = TwistStamped()
        output.header.stamp = now.to_msg()
        output.header.frame_id = 'base_link'
        output.twist.linear.x, output.twist.linear.y, output.twist.angular.z = result.command
        self.safe_pub.publish(output)
        self.previous = result.command
        self.last_time = now
        status = String()
        status.data = json.dumps({'stamp_ns': now.nanoseconds,
                                  'override': result.overridden, 'reason': result.reason,
                                  'minimum_clearance': self.last_scan})
        self.status_pub.publish(status)

    def watchdog(self):
        now = self.get_clock().now()
        if self.last_action_time is None or (now - self.last_action_time).nanoseconds / 1e9 > self.timeout:
            self.previous = (0.0, 0.0, 0.0)
            self.last_time = now
            output = TwistStamped()
            output.header.stamp = now.to_msg()
            output.header.frame_id = 'base_link'
            self.safe_pub.publish(output)
            status = String()
            status.data = json.dumps({'stamp_ns': now.nanoseconds,
                                      'override': True, 'reason': 'stale_policy',
                                      'minimum_clearance': self.last_scan})
            self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
