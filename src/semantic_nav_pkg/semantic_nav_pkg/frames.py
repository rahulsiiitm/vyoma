"""Coordinate helpers for the ROS ENU <-> PX4 NED boundary."""

import math
from typing import Tuple


def enu_to_ned_vector(x: float, y: float, z: float) -> Tuple[float, float, float]:
    return y, x, -z


def ned_to_enu_vector(x: float, y: float, z: float) -> Tuple[float, float, float]:
    return y, x, -z


def enu_yaw_to_ned(yaw: float) -> float:
    return wrap_angle((math.pi / 2.0) - yaw)


def ned_yaw_to_enu(yaw: float) -> float:
    return wrap_angle((math.pi / 2.0) - yaw)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
