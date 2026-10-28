import numpy as np
import pytest

pytest.importorskip('rclpy')
from rl_navigation_pkg.policy_node import PolicyNode


def test_baseline_deflects_toward_clear_left_side():
    ranges = np.full(64, 1.0, dtype=np.float32)
    ranges[38:] = 10.0
    action = PolicyNode.baseline_action(ranges, 0.0)
    assert action[1] > 0.0


def test_baseline_deflects_toward_clear_right_side():
    ranges = np.full(64, 1.0, dtype=np.float32)
    ranges[:26] = 10.0
    action = PolicyNode.baseline_action(ranges, 0.0)
    assert action[1] < 0.0
