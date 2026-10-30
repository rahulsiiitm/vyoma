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
    observed = np.all(np.isfinite(xyz), axis=1) & (distances > 0)
    valid = (observed
             & (xyz[:, 2] >= minimum_height) & (xyz[:, 2] <= maximum_height)
             & (distances <= maximum_range))
    indices = np.clip((((np.nan_to_num(np.arctan2(xyz[:, 1], xyz[:, 0])) + math.pi)
                        / (2 * math.pi)) * count).astype(np.int64), 0, count - 1)
    # A sector the camera saw without a nearby obstacle is clear to the sensor
    # range; inf is reserved for sectors outside the field of view (unknown).
    ranges[np.unique(indices[observed])] = maximum_range
    np.minimum.at(ranges, indices[valid], distances[valid])
    return ranges
