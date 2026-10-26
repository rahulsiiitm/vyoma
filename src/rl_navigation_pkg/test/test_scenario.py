import math

import pytest

from rl_navigation_pkg.scenario import (
    ScenarioBounds,
    ScenarioSampler,
    TRAINING_WORLD_EXCLUSIONS,
)


def test_same_seed_generates_same_goal():
    sampler = ScenarioSampler(exclusions=TRAINING_WORLD_EXCLUSIONS)
    assert sampler.sample(42) == sampler.sample(42)


def test_generated_goal_obeys_bounds_and_clearance():
    sampler = ScenarioSampler(exclusions=TRAINING_WORLD_EXCLUSIONS)
    scenario = sampler.sample(7)
    bounds = sampler.bounds
    assert bounds.minimum_x <= scenario.goal_x <= bounds.maximum_x
    assert bounds.minimum_y <= scenario.goal_y <= bounds.maximum_y
    assert math.hypot(scenario.goal_x, scenario.goal_y) >= bounds.minimum_start_distance
    for obstacle in TRAINING_WORLD_EXCLUSIONS:
        assert math.hypot(scenario.goal_x - obstacle.x, scenario.goal_y - obstacle.y) >= (
            obstacle.radius + bounds.goal_clearance
        )


def test_impossible_scenario_fails_explicitly():
    sampler = ScenarioSampler(
        bounds=ScenarioBounds(0.0, 0.1, 0.0, 0.1, minimum_start_distance=10.0),
        maximum_attempts=2,
    )
    with pytest.raises(RuntimeError, match="unable to generate"):
        sampler.sample(1)
