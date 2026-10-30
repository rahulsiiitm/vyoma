import math

from semantic_nav_pkg.safety import (SafetyLimits, directional_clearance, evasive_action,
                                     filter_action)
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


def test_clear_oblique_escape_is_not_stopped_by_nearest_side_obstacle():
    ranges = [10.0] * 64
    ranges[32] = 0.5
    side_clearance = directional_clearance(ranges, math.pi / 2, -math.pi, 2 * math.pi / 64)
    result = filter_action((0.0, 0.5, 0.0), (0.0, 0.0, 0.0), 0.5, 0.1, True,
                           SafetyLimits(), directional_clearance=side_clearance)
    assert side_clearance == 10.0
    assert result.command[1] > 0.0
    assert result.reason == 'limited'


def test_evasive_action_selects_a_clear_heading_around_blocked_front():
    ranges = [10.0] * 64
    for index in range(29, 36):
        ranges[index] = 0.5
    escape = evasive_action(ranges, 0.0, -math.pi, 2 * math.pi / 64, SafetyLimits())
    assert escape is not None
    command, clearance = escape
    assert abs(command[1]) > 0.1
    assert clearance >= SafetyLimits().warning_clearance


def test_evasive_action_does_not_steer_into_unobserved_sectors():
    # Forward camera: only sectors 22-41 observed, the centre is blocked and
    # everything outside the field of view is unknown (inf).
    ranges = [math.inf] * 64
    for index in range(22, 42):
        ranges[index] = 10.0
    for index in range(29, 36):
        ranges[index] = 0.5
    escape = evasive_action(ranges, 0.0, -math.pi, 2 * math.pi / 64, SafetyLimits(),
                            minimum_clearance=2.25, half_width=6)
    assert escape is None
    ranges[22:29] = [10.0] * 7
    ranges[15:22] = [10.0] * 7
    escape = evasive_action(ranges, 0.0, -math.pi, 2 * math.pi / 64, SafetyLimits(),
                            minimum_clearance=2.25, half_width=6)
    assert escape is not None
    assert escape[0][1] < 0.0


def test_evasive_action_stops_when_no_forward_sector_is_clear():
    escape = evasive_action([0.5] * 64, 0.0, -math.pi,
                            2 * math.pi / 64, SafetyLimits())
    assert escape is None


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
