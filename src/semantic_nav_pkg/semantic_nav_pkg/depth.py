"""Vectorized camera-frame polar projection for the Stage-1 depth contract."""

import math

import numpy as np


def sector_ranges(points, count=64, maximum_range=10.0,
                  minimum_height=-1.0, maximum_height=1.0):
    if count < 1 or not math.isfinite(maximum_range) or maximum_range <= 0:
        raise ValueError('sector count and maximum range must be positive')
    xyz = np.asarray(points).reshape(-1, 3)
    ranges = np.full(count, np.inf, dtype=np.float32)
    distances = np.hypot(xyz[:, 0], xyz[:, 1])
    valid = (np.all(np.isfinite(xyz), axis=1)
             & (xyz[:, 2] >= minimum_height) & (xyz[:, 2] <= maximum_height)
             & (distances > 0) & (distances <= maximum_range))
    selected = xyz[valid]
    indices = (((np.arctan2(selected[:, 1], selected[:, 0]) + math.pi)
                / (2 * math.pi)) * count).astype(np.int64)
    np.minimum.at(ranges, np.clip(indices, 0, count - 1), distances[valid])
    return ranges
