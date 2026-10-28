import math

import numpy as np

from semantic_nav_pkg.depth import sector_ranges


def test_sectors_keep_nearest_point_and_scan_direction():
    ranges = sector_ranges([[3, 0, 0], [2, 0, 0], [0, 4, 0], [0, -5, 0]])
    assert ranges[32] == 2
    assert ranges[48] == 4
    assert ranges[16] == 5
    assert np.count_nonzero(np.isfinite(ranges)) == 3


def test_projection_excludes_out_of_band_and_invalid_points():
    ranges = sector_ranges([[0, 0, 0], [20, 0, 0], [1, 0, 2], [1, 0, -2],
                            [math.nan, 1, 0], [math.inf, 0, 0]])
    assert np.all(np.isinf(ranges))


def test_empty_cloud_and_backward_boundary_are_supported():
    assert np.all(np.isinf(sector_ranges(np.empty((0, 3)))))
    ranges = sector_ranges([[-1, 0.0, 0], [-2, -0.0, 0]])
    assert ranges[63] == 1
    assert ranges[0] == 2
