import json
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('gymnasium')
import rclpy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String

from rl_navigation_pkg.drone_env import DroneEnv
from rl_navigation_pkg.observation import OBSERVATION_SIZE


@pytest.fixture
def environment():
    rclpy.init()
    node = DroneEnv()
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize('clearance', [0.0, None, float('nan'), -1.0])
def test_missing_or_unsafe_clearance_never_becomes_clear_space(environment, clearance):
    environment.safety_callback(String(data=json.dumps({
        'override': True, 'minimum_clearance': clearance})))
    assert environment.clearance == 0.0
    assert environment.safety_override


def test_invalid_observation_does_not_count_as_fresh(environment):
    message = Float32MultiArray()
    message.data = [0.0] * OBSERVATION_SIZE
    message.data[0] = float('nan')
    environment.observation_callback(message)
    assert environment.observation_sequence == 0
    message.data = [0.0] * OBSERVATION_SIZE
    environment.observation_callback(message)
    assert environment.observation_sequence == 1


def test_startup_rejects_observations_older_than_navigation(environment):
    environment.mission_callback(String(data=json.dumps({'state': 'RL_NAVIGATE', 'stamp_ns': 100})))
    message = Float32MultiArray(data=[0.0] * OBSERVATION_SIZE)
    message.layout.dim = [MultiArrayDimension(label='schema=1;stamp_ns=99')]
    environment.observation_callback(message)
    assert environment.navigation_observation_sequence == 0
    message.layout.dim[0].label = 'schema=1;stamp_ns=100'
    environment.observation_callback(message)
    assert environment.navigation_observation_sequence == 2


def test_startup_wait_publishes_neutral_action_until_observation(environment, monkeypatch):
    actions = []
    environment.mission_state = 'GLOBAL_PLAN'
    monkeypatch.setattr(environment, 'publish_action', lambda action: actions.append(action.copy()))
    monkeypatch.setattr(rclpy, 'spin_once',
                        lambda node, timeout_sec: setattr(node, 'observation_sequence', 1))

    assert environment.wait_for_observation(0, timeout=1.0, keepalive=True)
    assert len(actions) == 1
    np.testing.assert_array_equal(actions[0], np.zeros(3, dtype=np.float32))


def test_startup_wait_does_not_publish_neutral_action_during_preflight(environment, monkeypatch):
    actions = []
    environment.mission_state = 'PREFLIGHT'
    monkeypatch.setattr(environment, 'publish_action', lambda action: actions.append(action.copy()))
    monkeypatch.setattr(rclpy, 'spin_once',
                        lambda node, timeout_sec: setattr(node, 'observation_sequence', 1))

    assert environment.wait_for_observation(0, timeout=1.0, keepalive=True)
    assert not actions


def test_step_waits_for_new_observation_and_matching_safety_result(environment, monkeypatch):
    monkeypatch.setattr(environment, 'publish_action', lambda action: 100)
    monkeypatch.setattr(environment, 'distance_to_goal', lambda: 1.0)
    calls = []

    def deliver(node, timeout_sec):
        calls.append(None)
        node.observation_sequence += 1
        node.observation_stamp_ns = 99 if len(calls) == 1 else 101
        node.safety_action_stamp_ns = 99 if len(calls) == 1 else 100

    monkeypatch.setattr(rclpy, 'spin_once', deliver)
    environment.step(np.zeros(3, dtype=np.float32))
    assert len(calls) == 2


def test_mission_failure_ends_episode(environment, monkeypatch):
    monkeypatch.setattr(environment, 'publish_action', lambda action: 100)
    monkeypatch.setattr(environment, 'distance_to_goal', lambda: 2.0)

    def deliver(node, timeout_sec):
        node.observation_sequence += 1
        node.observation_stamp_ns = 101
        node.safety_action_stamp_ns = 100
        node.mission_state = 'FAILSAFE_LAND'

    monkeypatch.setattr(rclpy, 'spin_once', deliver)
    _, reward, terminated, truncated, info = environment.step(np.zeros(3, dtype=np.float32))
    assert terminated and not truncated and info['mission_failure']
    assert reward <= -100.0


@pytest.mark.parametrize('returncode,stdout,accepted', [(0, 'data: true', True),
                                                        (0, 'data: false', False),
                                                        (1, 'data: true', False)])
def test_pose_reset_requires_gazebo_success(environment, monkeypatch, returncode, stdout, accepted):
    monkeypatch.setattr('rl_navigation_pkg.drone_env.subprocess.run',
                        lambda *args, **kwargs: SimpleNamespace(
                            returncode=returncode, stdout=stdout, stderr=''))
    if accepted:
        environment.reset_model_pose()
    else:
        with pytest.raises(RuntimeError, match='pose reset'):
            environment.reset_model_pose()


def test_simulator_estimator_is_stopped_then_started(environment, monkeypatch):
    calls = []
    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0, stdout='', stderr='')
    monkeypatch.setattr('rl_navigation_pkg.drone_env.subprocess.run', run)
    environment.restart_simulator_estimator()
    assert [command[-1] for command in calls] == ['stop', 'start']
    assert all(command[0].endswith('/px4-ekf2') for command in calls)


@pytest.mark.parametrize('action', [[float('nan'), 0.0, 0.0], [2.0, 0.0, 0.0], [0.0]])
def test_invalid_actions_rejected_before_publishing(environment, action):
    with pytest.raises(ValueError):
        environment.step(np.asarray(action, dtype=np.float32))
