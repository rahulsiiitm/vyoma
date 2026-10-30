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
    directional_clearance: float = None,
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
    clearance = minimum_clearance if directional_clearance is None else directional_clearance
    if not math.isfinite(clearance) or clearance < 0.0:
        return SafetyResult((0.0, 0.0, 0.0), True, "invalid_clearance")
    if clearance < limits.hard_clearance:
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

    if clearance < limits.warning_clearance:
        scale = max(0.0, (clearance - limits.hard_clearance) /
                    (limits.warning_clearance - limits.hard_clearance))
        limited = (min(limited[0], max(0.0, limited[0]) * scale), limited[1], limited[2])
        return SafetyResult(limited, True, "warning_clearance")

    overridden = limited != tuple(float(value) for value in action)
    return SafetyResult(limited, overridden, "limited" if overridden else "ok")


def directional_clearance(ranges, bearing, angle_min, angle_increment, half_width=2,
                          maximum_range=10.0):
    """Return the nearest range in a cone centered on a body-frame travel bearing."""
    if not ranges or not all(math.isfinite(float(value)) or math.isinf(float(value)) for value in ranges):
        return 0.0
    if (not math.isfinite(bearing) or not math.isfinite(angle_min) or
            not math.isfinite(angle_increment) or angle_increment <= 0.0 or half_width < 0):
        return 0.0
    nearest = math.inf
    for index, value in enumerate(ranges):
        angle = angle_min + index * angle_increment
        difference = (angle - bearing + math.pi) % (2.0 * math.pi) - math.pi
        if abs(difference) <= (half_width + 0.5) * angle_increment:
            value = float(value)
            if math.isnan(value) or value < 0.0:
                return 0.0
            nearest = min(nearest, maximum_range if math.isinf(value) else value)
    return 0.0 if math.isinf(nearest) else nearest


def evasive_action(ranges, preferred_bearing, angle_min, angle_increment, limits,
                   minimum_clearance=None, half_width=2, maximum_range=10.0):
    """Find the nearest forward heading with clearance above the warning margin."""
    count = len(ranges)
    if count == 0 or not all(math.isfinite(float(value)) or math.isinf(float(value)) for value in ranges):
        return None
    minimum_clearance = limits.warning_clearance if minimum_clearance is None else minimum_clearance
    candidates = []
    for index in range(count):
        bearing = angle_min + index * angle_increment
        bearing = (bearing + math.pi) % (2.0 * math.pi) - math.pi
        if abs(bearing) > math.pi / 2.0:
            continue
        # Never steer into a cone containing unobserved (inf) sectors: outside
        # the camera field of view is unknown space, not free space.
        if any(math.isinf(float(ranges[(index + offset) % count]))
               for offset in range(-half_width, half_width + 1)):
            continue
        clearance = directional_clearance(ranges, bearing, angle_min, angle_increment,
                                          half_width, maximum_range)
        if clearance >= minimum_clearance:
            delta = (bearing - preferred_bearing + math.pi) % (2.0 * math.pi) - math.pi
            candidates.append((abs(delta), -clearance, bearing, clearance))
    if not candidates:
        return None
    _, _, bearing, clearance = min(candidates)
    speed = min(0.8, max(0.4, limits.max_forward * 0.4))
    return (speed * math.cos(bearing), speed * math.sin(bearing), 0.0), clearance
