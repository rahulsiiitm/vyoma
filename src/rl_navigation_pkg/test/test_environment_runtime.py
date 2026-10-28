import json

import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('gymnasium')
import rclpy
from std_msgs.msg import Float32MultiArray, String

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


@pytest.mark.parametrize('action', [[float('nan'), 0.0, 0.0], [2.0, 0.0, 0.0], [0.0]])
def test_invalid_actions_rejected_before_publishing(environment, action):
    with pytest.raises(ValueError):
        environment.step(np.asarray(action, dtype=np.float32))
