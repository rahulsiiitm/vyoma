"""Deterministic Stage-1 scenario generation.

The simulator integration owns entity placement.  This module deliberately
contains no ROS dependencies so scenario generation can be reproduced and
tested outside Gazebo.
"""

from dataclasses import dataclass
import math
from typing import Iterable, Tuple

import numpy as np


@dataclass(frozen=True)
class CircularExclusion:
    """A 2-D region in which a mission goal must not be generated."""

    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class ScenarioBounds:
    minimum_x: float = 3.0
    maximum_x: float = 8.0
    minimum_y: float = -4.0
    maximum_y: float = 4.0
    minimum_start_distance: float = 3.0
    goal_clearance: float = 0.75


@dataclass(frozen=True)
class EpisodeScenario:
    seed: int
    goal_x: float
    goal_y: float

    @property
    def goal(self) -> Tuple[float, float]:
        return self.goal_x, self.goal_y


class ScenarioSampler:
    """Generate collision-free goals solely from an explicit seed."""

    def __init__(
        self,
        bounds: ScenarioBounds = ScenarioBounds(),
        exclusions: Iterable[CircularExclusion] = (),
        maximum_attempts: int = 256,
    ) -> None:
        self.bounds = bounds
        self.exclusions = tuple(exclusions)
        self.maximum_attempts = maximum_attempts

    def sample(self, seed: int, start: Tuple[float, float] = (0.0, 0.0)) -> EpisodeScenario:
        rng = np.random.default_rng(seed)
        for _ in range(self.maximum_attempts):
            x = float(rng.uniform(self.bounds.minimum_x, self.bounds.maximum_x))
            y = float(rng.uniform(self.bounds.minimum_y, self.bounds.maximum_y))
            if math.hypot(x - start[0], y - start[1]) < self.bounds.minimum_start_distance:
                continue
            if any(
                math.hypot(x - obstacle.x, y - obstacle.y)
                < obstacle.radius + self.bounds.goal_clearance
                for obstacle in self.exclusions
            ):
                continue
            return EpisodeScenario(seed=seed, goal_x=x, goal_y=y)
        raise RuntimeError(
            f"unable to generate a valid scenario for seed {seed} after "
            f"{self.maximum_attempts} attempts"
        )


TRAINING_WORLD_EXCLUSIONS = (
    CircularExclusion(3.0, 0.0, 0.71),
    CircularExclusion(5.0, 2.0, 0.50),
    CircularExclusion(7.0, -1.0, 1.12),
)
