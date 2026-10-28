"""Deterministic rolling targets along an ordered global route."""

import math
from typing import Sequence, Tuple


def target_index(points: Sequence[Tuple[float, float]], x: float, y: float,
                 lookahead: float) -> int:
    if not points:
        raise ValueError('path must not be empty')
    if not math.isfinite(lookahead) or lookahead < 0.0:
        raise ValueError('lookahead must be finite and nonnegative')
    nearest = min(range(len(points)),
                  key=lambda i: math.hypot(points[i][0] - x, points[i][1] - y))
    distance = 0.0
    for index in range(nearest, len(points) - 1):
        if distance >= lookahead:
            return index
        distance += math.hypot(points[index + 1][0] - points[index][0],
                               points[index + 1][1] - points[index][1])
    return len(points) - 1
