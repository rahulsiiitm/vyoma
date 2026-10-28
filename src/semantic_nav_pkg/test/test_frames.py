import math

from semantic_nav_pkg.frames import enu_to_ned_vector, enu_yaw_to_ned, ned_to_enu_vector


def test_vector_round_trip():
    enu = (2.0, -3.0, 5.0)
    assert ned_to_enu_vector(*enu_to_ned_vector(*enu)) == enu


def test_yaw_axes():
    assert math.isclose(enu_yaw_to_ned(0.0), math.pi / 2.0)
    assert math.isclose(enu_yaw_to_ned(math.pi / 2.0), 0.0)
