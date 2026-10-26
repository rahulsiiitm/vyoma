import json
import math
import time
from typing import Optional

import gymnasium as gym
from gymnasium import spaces
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from ros_gz_interfaces.srv import ControlWorld

from .observation import OBSERVATION_SIZE, calculate_reward
from .reset import ResetValidator, VehicleMotion
from .scenario import ScenarioSampler, TRAINING_WORLD_EXCLUSIONS


class DroneEnv(gym.Env, Node):
    """ROS/Gazebo environment for the Stage-1 three-axis local policy."""

    metadata = {'render_modes': []}

    def __init__(self, maximum_steps: int = 1200, reset_timeout: float = 10.0,
                 step_timeout: float = 1.0):
        gym.Env.__init__(self)
        Node.__init__(self, 'drone_rl_environment')
        self.action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(-1.0, 1.0, shape=(OBSERVATION_SIZE,), dtype=np.float32)
        self.maximum_steps = maximum_steps
        self.reset_timeout = reset_timeout
        # Each step lasts exactly one policy_node control tick: the step returns
        # on the next observation, so step length follows the node's sim-time timer.
        self.step_timeout = step_timeout
        self.observation_sequence = 0
        self.path: Optional[Path] = None
        self.observation = np.zeros(OBSERVATION_SIZE, dtype=np.float32)
        self.odom: Optional[Odometry] = None
        self.goal = np.zeros(2, dtype=np.float32)
        self.previous_distance = None
        self.previous_action = np.zeros(3, dtype=np.float32)
        self.safety_override = False
        self.clearance = 10.0
        self.step_count = 0
        self.odom_sequence = 0
        self.scenario_sampler = ScenarioSampler(exclusions=TRAINING_WORLD_EXCLUSIONS)
        self.action_pub = self.create_publisher(TwistStamped, '/vyoma/rl/raw_action', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/vyoma/mission/command', 10)
        self.reset_client = self.create_client(ControlWorld, '/world/training_world/control')
        self.component_reset_clients = {
            'mission': self.create_client(Trigger, '/vyoma/mission/reset'),
            'safety': self.create_client(Trigger, '/vyoma/safety/reset'),
            'px4': self.create_client(Trigger, '/vyoma/px4/reset'),
        }
        self.policy_reset_client = self.create_client(Trigger, '/vyoma/policy/reset')
        self.create_subscription(
            Float32MultiArray, '/vyoma/rl/observation', self.observation_callback, 10
        )
        self.create_subscription(Odometry, '/vyoma/odometry', self.odom_callback, 10)
        self.create_subscription(Path, '/vyoma/global_path', self.path_callback, 10)
        self.create_subscription(String, '/vyoma/safety/status', self.safety_callback, 10)

    def observation_callback(self, msg):
        values = np.asarray(msg.data, dtype=np.float32)
        if values.shape == (OBSERVATION_SIZE,):
            self.observation = values
            self.observation_sequence += 1

    def odom_callback(self, msg):
        self.odom = msg
        self.odom_sequence += 1

    def path_callback(self, msg):
        self.path = msg

    def cross_track_error(self):
        if self.odom is None or self.path is None or not self.path.poses:
            return 0.0
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        return min(math.hypot(item.pose.position.x - x, item.pose.position.y - y)
                   for item in self.path.poses)

    def wait_for_observation(self, after_sequence):
        deadline = time.monotonic() + self.step_timeout
        while self.observation_sequence <= after_sequence:
            if time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.01)
        return True

    def safety_callback(self, msg):
        try:
            status = json.loads(msg.data)
            self.safety_override = bool(status.get('override', False))
            self.clearance = float(status.get('minimum_clearance') or 10.0)
        except (ValueError, TypeError):
            self.safety_override = True

    def distance_to_goal(self):
        if self.odom is None:
            return math.inf
        return math.hypot(self.goal[0] - self.odom.pose.pose.position.x,
                          self.goal[1] - self.odom.pose.pose.position.y)

    def publish_action(self, action):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(action[0] * 2.0)
        msg.twist.linear.y = float(action[1] * 1.5)
        msg.twist.angular.z = float(action[2])
        self.action_pub.publish(msg)

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

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        sequence = self.observation_sequence
        self.publish_action(action)
        if not self.wait_for_observation(sequence):
            raise RuntimeError('no fresh observation within the step timeout')
        distance = self.distance_to_goal()
        progress = 0.0 if self.previous_distance is None else self.previous_distance - distance
        reached = distance < 0.5
        collision = self.clearance < 0.3
        reward = calculate_reward(progress, self.cross_track_error(), self.clearance, action,
                                  self.previous_action, self.safety_override, reached, collision)
        self.previous_distance = distance
        self.previous_action = action.copy()
        self.step_count += 1
        terminated = reached or collision
        truncated = self.step_count >= self.maximum_steps or not np.isfinite(distance)
        return self.observation.copy(), reward, terminated, truncated, {
            'distance': distance, 'clearance': self.clearance,
            'safety_override': self.safety_override,
        }

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.publish_action(np.zeros(3, dtype=np.float32))
        for name, client in self.component_reset_clients.items():
            self.reset_component(name, client)
        self.reset_component('policy', self.policy_reset_client, required=False)
        initial_sequence = self.odom_sequence
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('Gazebo reset service is unavailable')

        request = ControlWorld.Request()
        request.world_control.reset.all = True
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.exception() is not None:
            raise RuntimeError('Gazebo reset request timed out or failed')
        response = future.result()
        if hasattr(response, 'success') and not response.success:
            raise RuntimeError('Gazebo rejected the episode reset request')

        validator = ResetValidator(initial_sequence=initial_sequence)
        deadline = time.monotonic() + self.reset_timeout
        settled = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom is None:
                continue
            velocity = self.odom.twist.twist.linear
            settled = validator.observe(VehicleMotion(
                sequence=self.odom_sequence,
                velocity_x=float(velocity.x),
                velocity_y=float(velocity.y),
                velocity_z=float(velocity.z),
            ))
            if settled:
                break
        if not settled:
            raise RuntimeError('vehicle did not publish fresh, settled odometry after reset')

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
        self.path = None
        if not self.wait_for_observation(self.observation_sequence):
            raise RuntimeError('no observation published after reset')
        self.previous_distance = None
        self.previous_action.fill(0.0)
        self.step_count = 0
        self.safety_override = False
        self.clearance = 10.0
        return self.observation.copy(), {
            'seed': episode_seed,
            'goal': self.goal.tolist(),
            'reset_odometry_samples': validator.stable_samples,
        }
