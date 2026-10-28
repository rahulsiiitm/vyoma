from semantic_nav_pkg.safety import SafetyLimits, filter_action
import pytest


def test_invalid_action_stops():
    result = filter_action((float('nan'), 0.0, 0.0), (0.0, 0.0, 0.0), 10.0, 0.1, True, SafetyLimits())
    assert result.command == (0.0, 0.0, 0.0)
    assert result.reason == 'invalid_action'


def test_stale_input_stops():
    result = filter_action((1.0, 0.0, 0.0), (0.0, 0.0, 0.0), 10.0, 0.1, False, SafetyLimits())
    assert result.reason == 'stale_input'


def test_hard_clearance_stops():
    result = filter_action((1.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.5, 0.1, True, SafetyLimits())
    assert result.command == (0.0, 0.0, 0.0)


def test_acceleration_is_limited():
    result = filter_action((2.0, 1.5, 1.0), (0.0, 0.0, 0.0), 10.0, 0.1, True, SafetyLimits())
    assert result.command == (0.2, 0.2, 0.2)


@pytest.mark.parametrize('clearance', [float('nan'), float('inf'), -1.0])
def test_invalid_clearance_stops(clearance):
    result = filter_action((1.0, 0.0, 0.0), (0.0, 0.0, 0.0), clearance,
                           0.1, True, SafetyLimits())
    assert result.command == (0.0, 0.0, 0.0)
    assert result.reason == 'invalid_clearance'


@pytest.mark.parametrize('previous,dt', [((float('nan'), 0.0, 0.0), 0.1),
                                       ((0.0,), 0.1), ((0.0, 0.0, 0.0), float('nan')),
                                       ((0.0, 0.0, 0.0), -1.0)])
def test_invalid_filter_state_stops(previous, dt):
    result = filter_action((1.0, 0.0, 0.0), previous, 10.0, dt, True, SafetyLimits())
    assert result.command == (0.0, 0.0, 0.0)
    assert result.reason == 'invalid_state'
