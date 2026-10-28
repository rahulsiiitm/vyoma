import json
import math

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleOdometry, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .frames import enu_to_ned_vector, ned_to_enu_vector, ned_yaw_to_enu


class Px4AdapterNode(Node):
    """Exclusive owner of PX4 offboard input topics and coordinate conversion."""

    def __init__(self):
        super().__init__('px4_adapter')
        self.declare_parameter('takeoff_altitude', 5.0)
        # Commander publishes vehicle_status at 2 Hz when unchanged. Its
        # freshness budget must allow more than one 500ms publication interval.
        self.declare_parameter('vehicle_status_timeout', 1.5)
        self.status_timeout = float(self.get_parameter('vehicle_status_timeout').value)
        self.declare_parameter(
            'vehicle_status_topic', '/fmu/out/vehicle_status_v4')
        self.altitude = float(self.get_parameter('takeoff_altitude').value)
        status_topic = str(
            self.get_parameter('vehicle_status_topic').value)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.odom_pub = self.create_publisher(Odometry, '/vyoma/odometry', 10)
        self.status_pub = self.create_publisher(String, '/vyoma/px4/status', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos)
        self.create_subscription(
            VehicleStatus, status_topic, self.vehicle_status_callback, qos)
        self.create_subscription(TwistStamped, '/vyoma/control/safe_command', self.control_callback, 10)
        self.create_subscription(String, '/vyoma/mission/state', self.state_callback, 10)
        self.create_service(Trigger, '/vyoma/px4/reset', self.reset_callback)
        self.state = 'DISARMED'
        self.safe_command = (0.0, 0.0, 0.0)
        self.position_enu = (0.0, 0.0, 0.0)
        self.hold_xy = None
        self.yaw_enu = 0.0
        self.counter = 0
        self.arm_sent = False
        self.last_odom_time = None
        self.last_vehicle_status_time = None
        self.last_control_time = None
        self.vehicle_status = None
        self.create_timer(0.1, self.timer_callback)

    def reset_callback(self, request, response):
        del request
        # This service is used only by the simulation episode manager. PX4
        # remains the authority for whether disarming is actually permitted.
        self.vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.state = 'DISARMED'
        self.safe_command = (0.0, 0.0, 0.0)
        self.hold_xy = None
        self.counter = 0
        self.arm_sent = False
        self.last_odom_time = None
        self.last_vehicle_status_time = None
        self.last_control_time = None
        self.vehicle_status = None
        response.success = True
        response.message = 'PX4 adapter state reset and disarm requested'
        return response

    def state_callback(self, msg):
        try:
            state = json.loads(msg.data)['state']
            if state != self.state:
                self.hold_xy = self.position_enu[:2]
            self.state = state
        except (ValueError, KeyError, TypeError):
            self.state = 'EMERGENCY_HOLD'

    def control_callback(self, msg):
        command = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        now = self.get_clock().now()
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (now.nanoseconds - stamp_ns) / 1e9
        if (msg.header.frame_id != 'base_link' or not 0.0 <= age < 0.5
                or not all(math.isfinite(value) for value in command)):
            self.safe_command = (0.0, 0.0, 0.0)
            self.last_control_time = None
            return
        self.safe_command = command
        self.last_control_time = now

    def odom_callback(self, msg):
        self.last_odom_time = self.get_clock().now()
        self.position_enu = ned_to_enu_vector(*msg.position)
        q = msg.q
        yaw_ned = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                             1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2))
        self.yaw_enu = ned_yaw_to_enu(yaw_ned)
        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x = float(self.position_enu[0])
        out.pose.pose.position.y = float(self.position_enu[1])
        out.pose.pose.position.z = float(self.position_enu[2])
        out.pose.pose.orientation.w = math.cos(self.yaw_enu / 2.0)
        out.pose.pose.orientation.z = math.sin(self.yaw_enu / 2.0)
        vx, vy, vz = ned_to_enu_vector(*msg.velocity)
        # Odometry twist is expressed in child_frame_id (base_link).
        out.twist.twist.linear.x = float(math.cos(self.yaw_enu) * vx + math.sin(self.yaw_enu) * vy)
        out.twist.twist.linear.y = float(-math.sin(self.yaw_enu) * vx + math.cos(self.yaw_enu) * vy)
        out.twist.twist.linear.z = float(vz)
        self.odom_pub.publish(out)
        transform = TransformStamped()
        transform.header = out.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = out.pose.pose.position.x
        transform.transform.translation.y = out.pose.pose.position.y
        transform.transform.translation.z = out.pose.pose.position.z
        transform.transform.rotation = out.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        self.last_vehicle_status_time = self.get_clock().now()

    def vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command, msg.param1, msg.param2 = command, float(param1), float(param2)
        msg.target_system = msg.source_system = 1
        msg.target_component = msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.command_pub.publish(msg)

    def timer_callback(self):
        now = self.get_clock().now()
        odom_fresh = self.last_odom_time is not None and (now - self.last_odom_time).nanoseconds / 1e9 < 0.5
        status_fresh = (self.last_vehicle_status_time is not None and
                        0.0 <= (now - self.last_vehicle_status_time).nanoseconds / 1e9 < self.status_timeout)
        status = String()
        status.data = json.dumps({
            'stamp_ns': now.nanoseconds,
            'connected': odom_fresh and status_fresh,
            'arming_state': getattr(self.vehicle_status, 'arming_state', -1),
            'nav_state': getattr(self.vehicle_status, 'nav_state', -1),
        })
        self.status_pub.publish(status)
        mode = OffboardControlMode()
        position_hold = self.hold_xy is not None and self.state in (
            'ARMING', 'TAKEOFF', 'GLOBAL_PLAN', 'GOAL_HOLD',
            'EMERGENCY_HOLD', 'LAND', 'FAILSAFE_LAND')
        mode.position = position_hold
        mode.velocity = not position_hold
        mode.timestamp = self.get_clock().now().nanoseconds // 1000
        self.mode_pub.publish(mode)

        if self.state in ('ARMING', 'TAKEOFF') and self.counter >= 20 and not self.arm_sent:
            self.vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.arm_sent = True

        vx = vy = yaw_rate = 0.0
        altitude_error = self.altitude - self.position_enu[2]
        vz_enu = 0.0
        if self.state == 'TAKEOFF':
            vz_enu = max(-1.0, min(1.0, altitude_error))
        elif self.state in ('RL_NAVIGATE', 'OBSTACLE_DEFLECTION', 'PATH_RECOVERY'):
            control_fresh = (self.last_control_time is not None and
                             0.0 <= (now - self.last_control_time).nanoseconds / 1e9 < 0.5)
            body_x, body_y, yaw_rate = self.safe_command if control_fresh else (0.0, 0.0, 0.0)
            vx = math.cos(self.yaw_enu) * body_x - math.sin(self.yaw_enu) * body_y
            vy = math.sin(self.yaw_enu) * body_x + math.cos(self.yaw_enu) * body_y
            vz_enu = max(-0.75, min(0.75, altitude_error))
        elif self.state in ('LAND', 'FAILSAFE_LAND'):
            vz_enu = -0.75
            if self.position_enu[2] <= 0.2:
                self.vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                self.arm_sent = False
        ned_velocity = enu_to_ned_vector(vx, vy, vz_enu)
        setpoint = TrajectorySetpoint()
        setpoint.position = ([float(value) for value in enu_to_ned_vector(*self.hold_xy, math.nan)]
                             if position_hold else [math.nan, math.nan, math.nan])
        setpoint.velocity = list(ned_velocity)
        setpoint.acceleration = [math.nan, math.nan, math.nan]
        setpoint.jerk = [math.nan, math.nan, math.nan]
        setpoint.yaw = math.nan
        setpoint.yawspeed = float(-yaw_rate)
        setpoint.timestamp = self.get_clock().now().nanoseconds // 1000
        self.setpoint_pub.publish(setpoint)
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = Px4AdapterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
