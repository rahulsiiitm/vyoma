"""Deterministic post-policy safety filter."""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple


@dataclass(frozen=True)
class SafetyLimits:
    max_forward: float = 2.0
    max_lateral: float = 1.5
    max_yaw_rate: float = 1.0
    max_acceleration: float = 2.0
    hard_clearance: float = 0.75
    warning_clearance: float = 1.5


@dataclass(frozen=True)
class SafetyResult:
    command: Tuple[float, float, float]
    overridden: bool
    reason: str


def filter_action(
    action: Sequence[float],
    previous: Sequence[float],
    minimum_clearance: float,
    dt: float,
    inputs_fresh: bool,
    limits: SafetyLimits,
) -> SafetyResult:
    if len(action) != 3 or not all(math.isfinite(float(value)) for value in action):
        return SafetyResult((0.0, 0.0, 0.0), True, "invalid_action")
    if not inputs_fresh:
        return SafetyResult((0.0, 0.0, 0.0), True, "stale_input")
    if not math.isfinite(minimum_clearance) or minimum_clearance < 0.0:
        return SafetyResult((0.0, 0.0, 0.0), True, "invalid_clearance")
    if (len(previous) != 3 or not all(math.isfinite(float(value)) for value in previous)
            or not math.isfinite(dt) or dt <= 0.0):
        return SafetyResult((0.0, 0.0, 0.0), True, "invalid_state")
    if minimum_clearance < limits.hard_clearance:
        return SafetyResult((0.0, 0.0, 0.0), True, "hard_clearance")

    target = (
        max(-limits.max_forward, min(limits.max_forward, float(action[0]))),
        max(-limits.max_lateral, min(limits.max_lateral, float(action[1]))),
        max(-limits.max_yaw_rate, min(limits.max_yaw_rate, float(action[2]))),
    )
    max_delta = limits.max_acceleration * max(0.0, dt)
    limited = tuple(
        max(float(old) - max_delta, min(float(old) + max_delta, new))
        for new, old in zip(target, previous)
    )

    if minimum_clearance < limits.warning_clearance:
        scale = max(0.0, (minimum_clearance - limits.hard_clearance) /
                    (limits.warning_clearance - limits.hard_clearance))
        limited = (min(limited[0], max(0.0, limited[0]) * scale), limited[1], limited[2])
        return SafetyResult(limited, True, "warning_clearance")

    overridden = limited != tuple(float(value) for value in action)
    return SafetyResult(limited, overridden, "limited" if overridden else "ok")
