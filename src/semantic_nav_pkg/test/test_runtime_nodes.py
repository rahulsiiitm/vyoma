"""Node-level regression checks; run in the sourced ROS container."""

import json
import math
import numpy as np

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('px4_msgs')
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.duration import Duration

from semantic_nav_pkg.mission import MissionState
from semantic_nav_pkg.mission_manager_node import MissionManagerNode
from semantic_nav_pkg.px4_adapter_node import Px4AdapterNode


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_missing_policy_does_not_block_preflight_but_stops_navigation():
    node = MissionManagerNode()
    try:
        node.safety_callback(String(data=json.dumps({
            'reason': 'stale_policy', 'minimum_clearance': None})))
        node.machine.transition(MissionState.PREFLIGHT, 'test')
        node.update()
        assert node.machine.state == MissionState.PREFLIGHT
        node.px4_connected = True
        node.machine.transition(MissionState.RL_NAVIGATE, 'test')
        node.update()
        assert node.machine.state == MissionState.EMERGENCY_HOLD
    finally:
        node.destroy_node()


@pytest.mark.parametrize('frame,age,value,accepted', [
    ('base_link', 0.0, 1.0, True),
    ('map', 0.0, 1.0, False),
    ('base_link', 1.0, 1.0, False),
    ('base_link', -1.0, 1.0, False),
    ('base_link', 0.0, math.nan, False),
])
def test_px4_adapter_rejects_invalid_commands(frame, age, value, accepted):
    node = Px4AdapterNode()
    try:
        message = TwistStamped()
        message.header.frame_id = frame
        stamp_ns = node.get_clock().now().nanoseconds - int(age * 1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        message.twist.linear.x = value
        node.control_callback(message)
        assert (node.last_control_time is not None) == accepted
        assert node.safe_command == ((1.0, 0.0, 0.0) if accepted else (0.0, 0.0, 0.0))
    finally:
        node.destroy_node()


def test_px4_adapter_does_not_replay_command_after_supervisor_stops():
    class Capture:
        def publish(self, message):
            self.message = message

    node = Px4AdapterNode()
    try:
        node.state = 'RL_NAVIGATE'
        node.safe_command = (2.0, 1.0, 0.5)
        node.last_control_time = node.get_clock().now() - Duration(seconds=1.0)
        capture = Capture()
        node.setpoint_pub = capture
        node.timer_callback()
        assert list(capture.message.velocity[:2]) == [0.0, 0.0]
        assert capture.message.yawspeed == 0.0
    finally:
        node.destroy_node()


@pytest.mark.parametrize('age,connected', [(0.7, True), (2.0, False)])
def test_status_freshness_allows_px4_two_hz_publication(age, connected):
    class Capture:
        def publish(self, message):
            self.message = message

    node = Px4AdapterNode()
    try:
        now = node.get_clock().now()
        node.last_odom_time = now
        node.last_vehicle_status_time = now - Duration(seconds=age)
        capture = Capture()
        node.status_pub = capture
        node.timer_callback()
        assert json.loads(capture.message.data)['connected'] == connected
    finally:
        node.destroy_node()


def test_takeoff_waits_for_confirmed_arming_and_offboard():
    node = MissionManagerNode()
    try:
        node.machine.transition(MissionState.ARMING, 'test')
        node.update()
        assert node.machine.state == MissionState.ARMING
        node.px4_status_callback(String(data=json.dumps({
            'connected': True, 'arming_state': 2, 'nav_state': 14})))
        node.update()
        assert node.machine.state == MissionState.TAKEOFF
    finally:
        node.destroy_node()


def test_landing_holds_captured_horizontal_position():
    class Capture:
        def publish(self, message):
            self.message = message

    node = Px4AdapterNode()
    try:
        # PX4 arrays yield NumPy scalars; ROS setpoints require Python floats.
        node.position_enu = tuple(np.float32(value) for value in (1.5, -0.5, 5.0))
        node.state_callback(String(data=json.dumps({'state': 'LAND'})))
        node.position_enu = (2.0, -0.8, 4.0)
        node.state_callback(String(data=json.dumps({'state': 'LAND'})))
        capture = Capture()
        node.setpoint_pub = capture
        node.timer_callback()
        assert list(capture.message.position[:2]) == [-0.5, 1.5]
        assert math.isnan(capture.message.position[2])
        assert capture.message.velocity[2] > 0
    finally:
        node.destroy_node()


def test_near_ground_does_not_mean_landed_until_px4_disarms():
    node = MissionManagerNode()
    try:
        node.machine.transition(MissionState.LAND, 'test')
        node.odom = Odometry()
        node.odom.pose.pose.position.z = 0.1
        node.px4_connected = True
        node.px4_armed = True
        node.update()
        assert node.machine.state == MissionState.LAND
        node.px4_armed = False
        node.update()
        assert node.machine.state == MissionState.DISARMED
    finally:
        node.destroy_node()
