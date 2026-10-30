"""Stable Stage-1 PPO observation and reward contracts."""

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

import numpy as np


SCHEMA_VERSION = 1
SECTOR_COUNT = 64
HISTORY_LENGTH = 3
BASE_FEATURE_COUNT = (SECTOR_COUNT * 2) + 15
OBSERVATION_SIZE = BASE_FEATURE_COUNT * HISTORY_LENGTH


@dataclass(frozen=True)
class NavigationFeatures:
    ranges: Sequence[float]
    valid: Sequence[bool]
    local_distance: float
    local_bearing: float
    goal_distance: float
    goal_bearing: float
    cross_track_error: float
    heading_error: float
    velocity_forward: float
    velocity_lateral: float
    yaw_rate: float
    previous_action: Sequence[float]
    path_blocked: bool
    maximum_range: float = 10.0


def encode(features: NavigationFeatures) -> np.ndarray:
    if len(features.ranges) != SECTOR_COUNT or len(features.valid) != SECTOR_COUNT:
        raise ValueError(f"expected {SECTOR_COUNT} depth sectors")
    ranges = np.asarray(features.ranges, dtype=np.float32)
    valid = np.asarray(features.valid, dtype=np.float32)
    ranges = np.nan_to_num(ranges, nan=features.maximum_range,
                           posinf=features.maximum_range, neginf=0.0)
    ranges = np.clip(ranges / features.maximum_range, 0.0, 1.0)
    valid_ranges = np.where(valid > 0.5, ranges, 1.0)
    minimum_index = int(np.argmin(valid_ranges))
    minimum_range = float(valid_ranges[minimum_index])
    minimum_direction = ((minimum_index + 0.5) / SECTOR_COUNT) * 2.0 - 1.0
    scalars = np.asarray([
        min(features.local_distance / 10.0, 1.0), features.local_bearing / math.pi,
        min(features.goal_distance / 50.0, 1.0), features.goal_bearing / math.pi,
        np.clip(features.cross_track_error / 5.0, -1.0, 1.0), features.heading_error / math.pi,
        np.clip(features.velocity_forward / 2.0, -1.0, 1.0),
        np.clip(features.velocity_lateral / 1.5, -1.0, 1.0),
        np.clip(features.yaw_rate, -1.0, 1.0),
        np.clip(features.previous_action[0], -1.0, 1.0),
        np.clip(features.previous_action[1], -1.0, 1.0),
        np.clip(features.previous_action[2], -1.0, 1.0),
        minimum_range, minimum_direction, float(features.path_blocked),
    ], dtype=np.float32)
    return np.concatenate((valid_ranges, valid, scalars)).astype(np.float32)


class ObservationHistory:
    def __init__(self, length: int = HISTORY_LENGTH) -> None:
        self.length = length
        self._frames: list[np.ndarray] = []

    def reset(self, frame: np.ndarray) -> np.ndarray:
        self._frames = [frame.copy() for _ in range(self.length)]
        return self.value()

    def clear(self) -> None:
        self._frames = []

    def append(self, frame: np.ndarray) -> np.ndarray:
        if not self._frames:
            return self.reset(frame)
        self._frames = (self._frames + [frame.copy()])[-self.length:]
        return self.value()

    def value(self) -> np.ndarray:
        if not self._frames:
            raise RuntimeError("observation history is empty")
        return np.concatenate(self._frames).astype(np.float32)


def calculate_reward(progress: float, cross_track: float, clearance: float,
                     action: Iterable[float], previous_action: Iterable[float],
                     safety_override: bool, reached_goal: bool, collision: bool,
                     safety_reason: str = '') -> float:
    action_array = np.asarray(tuple(action), dtype=np.float32)
    previous_array = np.asarray(tuple(previous_action), dtype=np.float32)
    reward = (8.0 * progress) - (0.15 * abs(cross_track))
    reward -= 0.05 * float(np.linalg.norm(action_array - previous_array))
    if clearance < 1.5:
        reward -= 2.0 * (1.5 - max(0.0, clearance))
    # Normal acceleration limiting is expected for exploratory PPO actions;
    # penalizing it like an emergency can swamp the progress signal.
    if safety_override and safety_reason != 'limited':
        reward -= 3.0
    if reached_goal:
        reward += 100.0
    if collision:
        reward -= 100.0
    return float(reward)
