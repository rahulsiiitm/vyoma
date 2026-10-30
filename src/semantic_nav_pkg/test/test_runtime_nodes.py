"""Node-level regression checks; run in the sourced ROS container."""

import json
import math
import numpy as np

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('px4_msgs')
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.duration import Duration
from std_srvs.srv import Trigger

from semantic_nav_pkg.mission import MissionState
from semantic_nav_pkg.mission_manager_node import MissionManagerNode
from semantic_nav_pkg.px4_adapter_node import Px4AdapterNode
from semantic_nav_pkg.safety_supervisor_node import SafetySupervisorNode


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


def test_empty_or_invalid_depth_scan_does_not_indicate_clear_space():
    node = SafetySupervisorNode()
    try:
        scan = LaserScan()
        scan.range_min, scan.range_max = 0.1, 10.0
        scan.ranges = [float('nan'), float('inf'), -1.0]
        node.scan_callback(scan)
        assert node.last_scan is None
        assert node.last_scan_time is None
        scan.ranges = [float('inf'), 2.0, 3.0]
        node.scan_callback(scan)
        assert node.last_scan == 2.0
        assert node.last_scan_time is not None
    finally:
        node.destroy_node()


def test_emergency_recovery_requires_sustained_fresh_clearance(monkeypatch):
    node = MissionManagerNode()
    class FakeClock:
        seconds = 0.0
        def now(self):
            return rclpy.time.Time(seconds=self.seconds)
    clock = FakeClock()
    monkeypatch.setattr(node, 'get_clock', lambda: clock)
    def status(reason, clearance):
        node.safety_callback(String(data=json.dumps({'reason': reason, 'minimum_clearance': clearance,
                                                     'stamp_ns': clock.now().nanoseconds})))
    try:
        node.px4_connected = True
        node.machine.transition(MissionState.RL_NAVIGATE, 'test')
        status('hard_clearance', 0.74)
        node.update()
        assert node.machine.state == MissionState.EMERGENCY_HOLD
        clock.seconds = 0.1
        status('warning_clearance', 0.8)
        node.update()
        assert node.machine.state == MissionState.EMERGENCY_HOLD
        clock.seconds = 0.2
        status('ok', 1.2)
        clock.seconds = 0.8  # A gap in status messages cancels recovery.
        node.update()
        assert node.safety_clear_since is None
        for seconds in (1.0, 1.4, 1.8, 2.1):
            clock.seconds = seconds
            status('ok', 1.2)
            node.update()
        assert node.machine.state == MissionState.GLOBAL_PLAN
    finally:
        node.destroy_node()


def test_emergency_hold_times_out_when_clearance_never_recovers(monkeypatch):
    node = MissionManagerNode()
    class FakeClock:
        seconds = 0.0
        def now(self):
            return rclpy.time.Time(seconds=self.seconds)
    clock = FakeClock()
    monkeypatch.setattr(node, 'get_clock', lambda: clock)
    try:
        node.px4_connected = True
        node.machine.transition(MissionState.RL_NAVIGATE, 'test')
        node.safety_callback(String(data=json.dumps({'reason': 'hard_clearance', 'minimum_clearance': 0.7})))
        node.update()
        clock.seconds = 5.1
        node.update()
        assert node.machine.state == MissionState.FAILSAFE_LAND
    finally:
        node.destroy_node()


def test_path_recovery_waits_until_odometry_returns_near_global_path():
    node = MissionManagerNode()
    try:
        node.machine.transition(MissionState.OBSTACLE_DEFLECTION, 'test')
        path = Path()
        for x, y in ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)
        node.path_callback(path)
        node.odom = Odometry()
        node.odom.pose.pose.position.x = 1.0
        node.odom.pose.pose.position.y = 2.0
        node.machine.update_navigation(False, node.path_error(), node.recovery_error)
        assert node.machine.state == MissionState.PATH_RECOVERY
        node.odom.pose.pose.position.y = 0.5
        node.machine.update_navigation(False, node.path_error(), node.recovery_error)
        assert node.machine.state == MissionState.RL_NAVIGATE
    finally:
        node.destroy_node()


def test_stale_healthy_status_does_not_clear_emergency(monkeypatch):
    node = MissionManagerNode()
    class FakeClock:
        def now(self):
            return rclpy.time.Time(seconds=2.0)
    monkeypatch.setattr(node, 'get_clock', lambda: FakeClock())
    try:
        node.px4_connected = True
        node.machine.transition(MissionState.EMERGENCY_HOLD, 'test')
        node.emergency_since = rclpy.time.Time(seconds=1.5)
        node.safety_callback(String(data=json.dumps({
            'reason': 'ok', 'minimum_clearance': 2.0, 'stamp_ns': 1_000_000_000})))
        node.update()
        assert node.safety_override
        assert node.safety_clear_since is None
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


def test_mission_reset_rejects_armed_vehicle_and_requests_landing():
    node = MissionManagerNode()
    try:
        node.authorized = True
        node.px4_connected = True
        node.px4_armed = True
        node.odom = Odometry()
        node.machine.transition(MissionState.RL_NAVIGATE, 'test')
        result = node.reset_callback(Trigger.Request(), Trigger.Response())
        assert not result.success
        assert node.machine.state == MissionState.RL_NAVIGATE
        result = node.land_for_reset_callback(Trigger.Request(), Trigger.Response())
        assert result.success
        assert node.machine.state == MissionState.FAILSAFE_LAND
    finally:
        node.destroy_node()


def test_px4_adapter_reset_requires_confirmed_disarm():
    node = Px4AdapterNode()
    try:
        result = node.reset_callback(Trigger.Request(), Trigger.Response())
        assert not result.success
    finally:
        node.destroy_node()


def test_px4_adapter_publishes_flu_yaw_rate_in_odometry():
    # The policy observes yaw rate from this field; it was previously left at 0.
    from px4_msgs.msg import VehicleOdometry

    class Capture:
        def publish(self, message):
            self.message = message

    node = Px4AdapterNode()
    try:
        capture = Capture()
        node.odom_pub = capture
        msg = VehicleOdometry()
        msg.position = [0.0, 0.0, -5.0]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.q = [1.0, 0.0, 0.0, 0.0]
        msg.angular_velocity = [0.0, 0.0, 0.4]  # FRD: clockwise seen from above
        node.odom_callback(msg)
        assert capture.message.twist.twist.angular.z == pytest.approx(-0.4)
    finally:
        node.destroy_node()
