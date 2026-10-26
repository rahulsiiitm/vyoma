from rl_navigation_pkg.reset import ResetValidator, VehicleMotion


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
