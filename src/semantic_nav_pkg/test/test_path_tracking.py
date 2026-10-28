import pytest

from semantic_nav_pkg.path_tracking import target_index


def test_target_stays_ahead_after_vehicle_passes_path_start():
    path = [(float(x), 0.0) for x in range(11)]
    assert target_index(path, 6.0, 0.0, 2.0) == 8


def test_lookahead_follows_route_around_corner():
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    assert target_index(path, 0.0, 0.0, 2.0) == 2


def test_target_clamps_to_goal():
    assert target_index([(0.0, 0.0), (1.0, 0.0)], 1.0, 0.0, 2.0) == 1


def test_empty_path_is_rejected():
    with pytest.raises(ValueError):
        target_index([], 0.0, 0.0, 2.0)
