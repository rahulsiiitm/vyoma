import json
import math
import os
from pathlib import Path as FilePath
import re
import subprocess
import time
from typing import Optional

import gymnasium as gym
from gymnasium import spaces
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from ros_gz_interfaces.srv import ControlWorld

from .observation import OBSERVATION_SIZE, calculate_reward
from .reset import ResetValidator, VehicleMotion
from .scenario import EASY_BOUNDS, ScenarioBounds, ScenarioSampler, TRAINING_WORLD_EXCLUSIONS

NAVIGATION_STATES = ('RL_NAVIGATE', 'OBSTACLE_DEFLECTION', 'PATH_RECOVERY')


class DroneEnv(gym.Env, Node):
    """ROS/Gazebo environment for the Stage-1 three-axis local policy."""

    metadata = {'render_modes': []}

    def __init__(self, maximum_steps: int = 1200, reset_timeout: float = 45.0,
                 step_timeout: float = 3.0, mission_start_timeout: float = 180.0,
                 scenario_profile: str = 'standard'):
        gym.Env.__init__(self)
        Node.__init__(self, 'drone_rl_environment', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(-1.0, 1.0, shape=(OBSERVATION_SIZE,), dtype=np.float32)
        self.maximum_steps = maximum_steps
        self.reset_timeout = reset_timeout
        # Each step lasts exactly one policy_node control tick: the step returns
        # on the next observation, so step length follows the node's sim-time timer.
        self.step_timeout = step_timeout
        self.mission_start_timeout = mission_start_timeout
        self.observation_sequence = 0
        self.navigation_observation_sequence = 0
        self.navigation_started_ns = None
        self.observation_stamp_ns = None
        self.safety_action_stamp_ns = None
        self.safety_action_result = None
        self.path: Optional[Path] = None
        self.observation = np.zeros(OBSERVATION_SIZE, dtype=np.float32)
        self.odom: Optional[Odometry] = None
        self.raw_odometry = None
        self.raw_odometry_sequence = 0
        self.mission_state = None
        self.px4_status = None
        self.px4_status_received = None
        self.px4_status_sequence = 0
        self.goal = np.zeros(2, dtype=np.float32)
        self.previous_distance = None
        self.previous_action = np.zeros(3, dtype=np.float32)
        self.safety_override = False
        self.safety_reason = 'unknown'
        self.clearance = 10.0
        self.step_count = 0
        self.odom_sequence = 0
        if scenario_profile not in ('standard', 'easy'):
            raise ValueError('unknown scenario profile')
        bounds = EASY_BOUNDS if scenario_profile == 'easy' else ScenarioBounds()
        self.scenario_sampler = ScenarioSampler(bounds=bounds, exclusions=TRAINING_WORLD_EXCLUSIONS)
        self.action_pub = self.create_publisher(TwistStamped, '/vyoma/rl/raw_action', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/vyoma/mission/command', 10)
        self.reset_client = self.create_client(ControlWorld, '/world/training_world/control')
        self.component_reset_clients = {
            'mission': self.create_client(Trigger, '/vyoma/mission/reset'),
            'safety': self.create_client(Trigger, '/vyoma/safety/reset'),
            'px4': self.create_client(Trigger, '/vyoma/px4/reset'),
        }
        self.policy_reset_client = self.create_client(Trigger, '/vyoma/policy/reset')
        self.land_client = self.create_client(Trigger, '/vyoma/mission/land_for_reset')
        self.create_subscription(
            Float32MultiArray, '/vyoma/rl/observation', self.observation_callback, 10
        )
        self.create_subscription(Odometry, '/vyoma/odometry', self.odom_callback, 10)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                                 self.raw_odom_callback, qos_profile_sensor_data)
        self.create_subscription(Path, '/vyoma/global_path', self.path_callback, 10)
        self.create_subscription(String, '/vyoma/safety/status', self.safety_callback, 10)
        self.create_subscription(String, '/vyoma/mission/state', self.mission_callback, 10)
        self.create_subscription(String, '/vyoma/px4/status', self.px4_status_callback, 10)

    def px4_status_callback(self, msg):
        try:
            self.px4_status = json.loads(msg.data)
            self.px4_status_received = time.monotonic()
            self.px4_status_sequence += 1
        except (ValueError, TypeError):
            self.px4_status = None

    def mission_callback(self, msg):
        try:
            status = json.loads(msg.data)
            state = status['state']
            if state in NAVIGATION_STATES and self.mission_state not in NAVIGATION_STATES:
                self.navigation_started_ns = int(status['stamp_ns'])
            self.mission_state = state
        except (ValueError, TypeError, KeyError, OverflowError):
            self.mission_state = None

    def observation_callback(self, msg):
        values = np.asarray(msg.data, dtype=np.float32)
        if (values.shape == (OBSERVATION_SIZE,) and np.all(np.isfinite(values))
                and np.all(np.abs(values) <= 1.0)):
            self.observation = values
            self.observation_sequence += 1
            try:
                label = msg.layout.dim[0].label
                stamp_ns = int(label.split('stamp_ns=', 1)[1])
                self.observation_stamp_ns = stamp_ns
                if (self.mission_state in NAVIGATION_STATES
                        and self.navigation_started_ns is not None
                        and stamp_ns >= self.navigation_started_ns):
                    self.navigation_observation_sequence = self.observation_sequence
            except (IndexError, ValueError, TypeError):
                pass

    def odom_callback(self, msg):
        self.odom = msg
        self.odom_sequence += 1

    def raw_odom_callback(self, msg):
        self.raw_odometry = msg
        self.raw_odometry_sequence += 1

    def path_callback(self, msg):
        self.path = msg

    def cross_track_error(self):
        if self.odom is None or self.path is None or not self.path.poses:
            return 0.0
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        return min(math.hypot(item.pose.position.x - x, item.pose.position.y - y)
                   for item in self.path.poses)

    def wait_for_observation(self, after_sequence, timeout=None, keepalive=False,
                             require_navigation=False):
        deadline = time.monotonic() + (self.step_timeout if timeout is None else timeout)
        next_keepalive = 0.0
        while (self.observation_sequence <= after_sequence or
               (require_navigation and (self.mission_state not in NAVIGATION_STATES or
                                        self.navigation_observation_sequence <= after_sequence))):
            now = time.monotonic()
            if now >= deadline:
                return False
            if (keepalive and self.mission_state in
                    ('GLOBAL_PLAN', 'RL_NAVIGATE', 'OBSTACLE_DEFLECTION',
                     'PATH_RECOVERY', 'EMERGENCY_HOLD') and now >= next_keepalive):
                self.publish_action(np.zeros(3, dtype=np.float32))
                next_keepalive = now + 0.1
            rclpy.spin_once(self, timeout_sec=0.01)
        return True

    def safety_callback(self, msg):
        try:
            status = json.loads(msg.data)
            self.safety_override = bool(status.get('override', False))
            self.safety_reason = str(status.get('reason', 'unknown'))
            clearance = status.get('minimum_clearance')
            self.clearance = 0.0 if clearance is None else float(clearance)
            if not math.isfinite(self.clearance) or self.clearance < 0.0:
                raise ValueError('invalid clearance')
            if status.get('action_stamp_ns') is not None:
                self.safety_action_stamp_ns = status['action_stamp_ns']
                self.safety_action_result = (self.safety_override, self.clearance, self.safety_reason)
        except (ValueError, TypeError):
            self.safety_override = True
            self.clearance = 0.0
            self.safety_reason = 'invalid_status'

    def distance_to_goal(self):
        if self.odom is None:
            return math.inf
        return math.hypot(self.goal[0] - self.odom.pose.pose.position.x,
                          self.goal[1] - self.odom.pose.pose.position.y)

    def publish_action(self, action):
        msg = TwistStamped()
        stamp = self.get_clock().now()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(action[0] * 2.0)
        msg.twist.linear.y = float(action[1] * 1.5)
        msg.twist.angular.z = float(action[2])
        self.action_pub.publish(msg)
        return stamp.nanoseconds

    def reset_component(self, name, client, required=True):
        if not client.wait_for_service(timeout_sec=2.0):
            if required:
                raise RuntimeError(f'{name} reset service is unavailable')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.exception() is not None:
            raise RuntimeError(f'{name} reset request timed out or failed')
        response = future.result()
        if response is None or not response.success:
            message = getattr(response, 'message', 'no response')
            raise RuntimeError(f'{name} reset was rejected: {message}')
        return True

    def prepare_vehicle_for_reset(self):
        deadline = time.monotonic() + self.mission_start_timeout
        landing_requested = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            status = self.px4_status or {}
            fresh = (self.px4_status_received is not None and
                     time.monotonic() - self.px4_status_received < 2.0)
            if not fresh or not status.get('connected'):
                continue
            if status.get('arming_state') == 1 and self.mission_state == 'DISARMED':
                return
            if status.get('arming_state') == 2 and not landing_requested:
                self.reset_component('landing', self.land_client)
                landing_requested = True
        raise RuntimeError('vehicle did not land and disarm before episode reset')

    def reset_model_pose(self):
        # The pinned Gazebo bridge does not expose SetEntityPose as a ROS
        # service. Reset the spawned model through Gazebo's native service.
        request = ('name: "x500_depth_0" position: {x: 0, y: 0, z: 0.3} '
                   'orientation: {w: 1}')
        try:
            result = subprocess.run(
                ['gz', 'service', '-s', '/world/training_world/set_pose',
                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '5000', '--req', request],
                capture_output=True, text=True, timeout=7.0, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError('Gazebo model pose reset failed') from exc
        if result.returncode != 0 or not re.search(r'\bdata:\s*true\b', result.stdout):
            raise RuntimeError(f'Gazebo rejected model pose reset: {result.stderr or result.stdout}')

    def restart_simulator_estimator(self):
        # Teleporting the simulated vehicle does not reset PX4's EKF. Restart
        # it only after confirmed disarm, then require fresh preflight readiness.
        workspace = FilePath(__file__).resolve().parents[3]
        source = FilePath(os.environ.get('VYOMA_PX4_SOURCE', workspace / 'simulation/PX4-Autopilot'))
        executable = source / 'build/px4_sitl_default/bin/px4-ekf2'
        for operation in ('stop', 'start'):
            try:
                result = subprocess.run([str(executable), operation], capture_output=True,
                                        text=True, timeout=15.0, check=False)
            except (OSError, subprocess.TimeoutExpired) as exc:
                raise RuntimeError(f'PX4 simulator estimator {operation} failed') from exc
            if result.returncode != 0:
                raise RuntimeError(f'PX4 simulator estimator {operation} failed: {result.stderr or result.stdout}')

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        if not self.action_space.contains(action):
            raise ValueError('action must contain three finite values in [-1, 1]')
        sequence = self.observation_sequence
        action_stamp_ns = self.publish_action(action)
        deadline = time.monotonic() + self.step_timeout
        while (self.observation_sequence <= sequence or self.observation_stamp_ns is None or
               self.observation_stamp_ns <= action_stamp_ns or
               self.safety_action_stamp_ns != action_stamp_ns):
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    'no observation and safety result for action within the step timeout '
                    f'(mission={self.mission_state}, action={action_stamp_ns}, '
                    f'observation={self.observation_stamp_ns}, safety_action={self.safety_action_stamp_ns})')
            rclpy.spin_once(self, timeout_sec=0.01)
        if self.safety_action_result is not None:
            self.safety_override, self.clearance, self.safety_reason = self.safety_action_result
        distance = self.distance_to_goal()
        progress = 0.0 if self.previous_distance is None else self.previous_distance - distance
        reached = distance < 0.5
        collision = self.clearance < 0.3
        mission_failure = (not reached and self.mission_state in
                           ('EMERGENCY_HOLD', 'FAILSAFE_LAND', 'LAND', 'DISARMED'))
        reward = calculate_reward(progress, self.cross_track_error(), self.clearance, action,
                                  self.previous_action, self.safety_override, reached,
                                  collision or mission_failure, self.safety_reason)
        self.previous_distance = distance
        self.previous_action = action.copy()
        self.step_count += 1
        terminated = reached or collision or mission_failure
        truncated = self.step_count >= self.maximum_steps or not np.isfinite(distance)
        return self.observation.copy(), reward, terminated, truncated, {
            'distance': distance, 'clearance': self.clearance,
            'safety_override': self.safety_override, 'safety_reason': self.safety_reason,
            'mission_failure': mission_failure, 'collision': collision, 'reached': reached,
        }

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.prepare_vehicle_for_reset()
        for name, client in self.component_reset_clients.items():
            self.reset_component(name, client)
        self.reset_component('policy', self.policy_reset_client)
        self.path = None
        self.odom = None
        self.raw_odometry = None
        self.mission_state = None
        self.navigation_started_ns = None
        self.navigation_observation_sequence = 0
        self.observation_stamp_ns = None
        self.safety_action_stamp_ns = None
        self.safety_action_result = None
        self.observation.fill(0.0)
        self.previous_distance = None
        self.previous_action.fill(0.0)
        self.step_count = 0
        self.safety_override = True
        self.clearance = 0.0
        initial_sequence = self.odom_sequence
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('Gazebo reset service is unavailable')

        request = ControlWorld.Request()
        # Resetting simulation time breaks PX4 lockstep and clears ROS TF caches.
        # Reset model state while keeping the clock monotonic.
        request.world_control.reset.model_only = True
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.exception() is not None:
            raise RuntimeError('Gazebo reset request timed out or failed')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('Gazebo rejected the episode reset request')
        self.reset_model_pose()
        self.restart_simulator_estimator()
        initial_sequence = self.odom_sequence
        last_raw_sequence = self.raw_odometry_sequence
        initial_status_sequence = self.px4_status_sequence

        validator = ResetValidator(initial_sequence=initial_sequence, minimum_stable_duration=2.0)
        deadline = time.monotonic() + self.reset_timeout
        settled = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.odom is None or self.raw_odometry is None or
                    self.raw_odometry_sequence <= last_raw_sequence):
                continue
            last_raw_sequence = self.raw_odometry_sequence
            velocity = self.odom.twist.twist.linear
            position = self.odom.pose.pose.position
            q = self.raw_odometry.q
            if not all(math.isfinite(float(value)) for value in q):
                validator.stable_samples = 0
                validator.stable_since_us = None
                continue
            w, x, y, z = (float(value) for value in q)
            if abs(math.sqrt(w*w + x*x + y*y + z*z) - 1.0) > 0.05:
                validator.stable_samples = 0
                validator.stable_since_us = None
                continue
            roll = math.atan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y))
            pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w*y - z*x))))
            settled = validator.observe(VehicleMotion(
                sequence=self.odom_sequence,
                velocity_x=float(velocity.x),
                velocity_y=float(velocity.y),
                velocity_z=float(velocity.z),
                position_x=float(position.x), position_y=float(position.y),
                position_z=float(position.z),
                roll=roll, pitch=pitch,
                angular_speed=math.hypot(*(float(value) for value in self.raw_odometry.angular_velocity)),
                timestamp_us=int(self.raw_odometry.timestamp),
            ))
            status = self.px4_status or {}
            if (settled and self.px4_status_sequence > initial_status_sequence and
                    status.get('connected') and status.get('arming_state') == 1 and
                    status.get('preflight_ready') and not status.get('failsafe', True)):
                break
            settled = False
        if not settled:
            raise RuntimeError('vehicle did not return to a fresh, settled start pose after reset')

        episode_seed = int(seed) if seed is not None else int(
            self.np_random.integers(0, np.iinfo(np.int32).max)
        )
        scenario = self.scenario_sampler.sample(episode_seed)
        self.goal = np.asarray(scenario.goal, dtype=np.float32)
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = float(self.goal[0])
        goal.pose.position.y = float(self.goal[1])
        goal.pose.orientation.w = 1.0
        # Publish more than once so a resetting mission manager cannot miss the
        # command while its subscriptions are being re-established.
        for _ in range(3):
            self.goal_pub.publish(goal)
            rclpy.spin_once(self, timeout_sec=0.1)
        # Observations require a local target, which becomes available only
        # after arming, takeoff and global planning have completed.
        if not self.wait_for_observation(self.observation_sequence,
                                         timeout=self.mission_start_timeout,
                                         keepalive=True, require_navigation=True):
            raise RuntimeError('no navigation observation before mission startup timeout')
        self.previous_distance = self.distance_to_goal()
        return self.observation.copy(), {
            'seed': episode_seed,
            'goal': self.goal.tolist(),
            'reset_odometry_samples': validator.stable_samples,
        }
