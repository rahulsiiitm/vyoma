from rl_navigation_pkg.reset import ResetValidator, VehicleMotion
import pytest


def motion(sequence, speed):
    return VehicleMotion(sequence, speed, 0.0, 0.0)


def test_stale_odometry_cannot_complete_reset():
    validator = ResetValidator(initial_sequence=10, required_stable_samples=2)
    assert not validator.observe(motion(10, 0.0))
    assert not validator.observe(motion(9, 0.0))


def test_reset_requires_consecutive_settled_samples():
    validator = ResetValidator(initial_sequence=10, maximum_speed=0.1, required_stable_samples=2)
    assert not validator.observe(motion(11, 0.0))
    assert not validator.observe(motion(12, 0.2))
    assert not validator.observe(motion(13, 0.0))
    assert validator.observe(motion(14, 0.05))


def test_non_finite_motion_resets_progress():
    validator = ResetValidator(initial_sequence=0, required_stable_samples=2)
    assert not validator.observe(motion(1, 0.0))
    assert not validator.observe(motion(2, float("nan")))
    assert not validator.observe(motion(3, 0.0))
    assert validator.observe(motion(4, 0.0))


def test_duplicate_and_out_of_order_samples_do_not_settle_vehicle():
    validator = ResetValidator(initial_sequence=10, required_stable_samples=2)
    assert not validator.observe(motion(12, 0.0))
    for _ in range(10):
        assert not validator.observe(motion(12, 0.0))
    assert not validator.observe(motion(11, 0.0))
    assert validator.stable_samples == 1
    assert validator.observe(motion(13, 0.0))


@pytest.mark.parametrize('speed', [-1.0, float('nan'), float('inf')])
def test_invalid_speed_threshold_rejected(speed):
    with pytest.raises(ValueError):
        ResetValidator(initial_sequence=0, maximum_speed=speed)


def test_extreme_motion_is_rejected_without_numeric_overflow():
    validator = ResetValidator(initial_sequence=0)
    assert not validator.observe(motion(1, 1e308))
