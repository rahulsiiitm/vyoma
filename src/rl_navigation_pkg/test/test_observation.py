import numpy as np

from rl_navigation_pkg.observation import (
    BASE_FEATURE_COUNT, HISTORY_LENGTH, OBSERVATION_SIZE, NavigationFeatures,
    ObservationHistory, calculate_reward, encode,
)


def features():
    return NavigationFeatures(
        ranges=[10.0] * 64, valid=[True] * 64, local_distance=2.0, local_bearing=0.0,
        goal_distance=5.0, goal_bearing=0.0, cross_track_error=0.0, heading_error=0.0,
        velocity_forward=0.0, velocity_lateral=0.0, yaw_rate=0.0,
        previous_action=[0.0, 0.0, 0.0], path_blocked=False,
    )


def test_observation_shape_and_history():
    frame = encode(features())
    assert frame.shape == (BASE_FEATURE_COUNT,)
    history = ObservationHistory(HISTORY_LENGTH)
    assert history.reset(frame).shape == (OBSERVATION_SIZE,)


def test_observation_history_can_be_cleared_between_episodes():
    history = ObservationHistory(HISTORY_LENGTH)
    history.reset(encode(features()))
    history.clear()
    try:
        history.value()
    except RuntimeError as error:
        assert 'empty' in str(error)
    else:
        raise AssertionError('cleared history should not expose stale observations')


def test_invalid_ranges_are_masked_as_far():
    value = features()
    value.ranges[0] = float('nan')
    value.valid[0] = False
    frame = encode(value)
    assert frame[0] == 1.0
    assert frame[64] == 0.0


def test_reward_prefers_progress_and_goal():
    neutral = calculate_reward(0.0, 0.0, 10.0, np.zeros(3), np.zeros(3), False, False, False)
    success = calculate_reward(1.0, 0.0, 10.0, np.zeros(3), np.zeros(3), False, True, False)
    collision = calculate_reward(0.0, 0.0, 0.0, np.zeros(3), np.zeros(3), True, False, True)
    assert success > neutral > collision


def test_reward_distinguishes_acceleration_limits_from_safety_intervention():
    args = (0.0, 0.0, 10.0, np.zeros(3), np.zeros(3), True, False, False)
    rate_limited = calculate_reward(*args, safety_reason='limited')
    emergency = calculate_reward(*args, safety_reason='stale_input')
    assert rate_limited == 0.0
    assert emergency == -3.0
