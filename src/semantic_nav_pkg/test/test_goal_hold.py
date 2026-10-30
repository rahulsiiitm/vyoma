from semantic_nav_pkg.mission import goal_hold_progress


def test_goal_hold_requires_a_full_stable_dwell():
    started, ready, drifted = goal_hold_progress(
        0.4, 0.1, None, 10.0, 0.5, 1.0, 0.25, 1.5)
    assert started == 10.0
    assert not ready
    assert not drifted
    started, ready, drifted = goal_hold_progress(
        0.4, 0.1, started, 11.49, 0.5, 1.0, 0.25, 1.5)
    assert not ready
    assert not drifted
    _, ready, drifted = goal_hold_progress(
        0.4, 0.1, started, 11.5, 0.5, 1.0, 0.25, 1.5)
    assert ready
    assert not drifted


def test_motion_resets_the_goal_hold_dwell():
    started, ready, drifted = goal_hold_progress(
        0.4, 0.5, 10.0, 11.0, 0.5, 1.0, 0.25, 1.5)
    assert started is None
    assert not ready
    assert not drifted


def test_goal_hold_replans_after_large_position_drift():
    started, ready, drifted = goal_hold_progress(
        1.1, 0.1, 10.0, 11.0, 0.5, 1.0, 0.25, 1.5)
    assert started is None
    assert not ready
    assert drifted


def test_goal_hold_rejects_nonfinite_telemetry():
    started, ready, drifted = goal_hold_progress(
        float('nan'), 0.0, None, 10.0, 0.5, 1.0, 0.25, 1.5)
    assert started is None
    assert not ready
    assert not drifted
