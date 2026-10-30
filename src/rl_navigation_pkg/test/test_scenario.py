import math

import pytest

from rl_navigation_pkg.scenario import (
    ScenarioBounds,
    ScenarioSampler,
    TRAINING_WORLD_EXCLUSIONS,
    EASY_BOUNDS,
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


def test_easy_curriculum_stays_short_and_clear_over_seeded_goals():
    sampler = ScenarioSampler(bounds=EASY_BOUNDS, exclusions=TRAINING_WORLD_EXCLUSIONS)
    bearings = []
    for seed in range(100):
        goal = sampler.sample(seed)
        assert 1.2 <= math.hypot(*goal.goal) < 2.9
        bearings.append(math.degrees(math.atan2(goal.goal_y, goal.goal_x)))
        for obstacle in TRAINING_WORLD_EXCLUSIONS:
            assert math.hypot(goal.goal_x - obstacle.x, goal.goal_y - obstacle.y) >= (
                obstacle.radius + EASY_BOUNDS.goal_clearance)
    # Bearing-diverse goals: a constant forward command must not solve the curriculum.
    assert min(bearings) < -60.0 and max(bearings) > 60.0
