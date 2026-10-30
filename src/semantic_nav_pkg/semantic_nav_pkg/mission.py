"""Deterministic mission state machine used by the ROS mission manager."""

from enum import Enum
import math


class MissionState(str, Enum):
    DISARMED = "DISARMED"
    PREFLIGHT = "PREFLIGHT"
    ARMING = "ARMING"
    TAKEOFF = "TAKEOFF"
    GLOBAL_PLAN = "GLOBAL_PLAN"
    RL_NAVIGATE = "RL_NAVIGATE"
    OBSTACLE_DEFLECTION = "OBSTACLE_DEFLECTION"
    PATH_RECOVERY = "PATH_RECOVERY"
    GOAL_HOLD = "GOAL_HOLD"
    LAND = "LAND"
    EMERGENCY_HOLD = "EMERGENCY_HOLD"
    FAILSAFE_LAND = "FAILSAFE_LAND"


AIRBORNE_STATES = {
    MissionState.TAKEOFF,
    MissionState.GLOBAL_PLAN,
    MissionState.RL_NAVIGATE,
    MissionState.OBSTACLE_DEFLECTION,
    MissionState.PATH_RECOVERY,
    MissionState.GOAL_HOLD,
    MissionState.LAND,
    MissionState.EMERGENCY_HOLD,
    MissionState.FAILSAFE_LAND,
}


def goal_hold_progress(distance, speed, hold_started, now, distance_tolerance,
                       drift_tolerance, speed_tolerance, dwell_duration):
    """Track a settled goal hold; return (start time, landed-ready, drifted)."""
    values = (distance, speed, now, distance_tolerance, drift_tolerance,
              speed_tolerance, dwell_duration)
    if not all(math.isfinite(float(value)) for value in values):
        return None, False, False
    if (distance < 0.0 or speed < 0.0 or distance_tolerance <= 0.0 or
            drift_tolerance < distance_tolerance or speed_tolerance < 0.0 or dwell_duration <= 0.0):
        return None, False, False
    if distance > drift_tolerance:
        return None, False, True
    if distance > distance_tolerance or speed > speed_tolerance:
        return None, False, False
    started = now if hold_started is None or now < hold_started else hold_started
    return started, now - started >= dwell_duration, False


class MissionMachine:
    def __init__(self) -> None:
        self.state = MissionState.DISARMED
        self.reason = "startup"

    def transition(self, state: MissionState, reason: str) -> MissionState:
        self.state = state
        self.reason = reason
        return self.state

    def reset(self, reason: str = "episode_reset") -> MissionState:
        return self.transition(MissionState.DISARMED, reason)

    def emergency(self, reason: str) -> MissionState:
        if self.state in AIRBORNE_STATES:
            return self.transition(MissionState.EMERGENCY_HOLD, reason)
        return self.transition(MissionState.DISARMED, reason)

    def update_navigation(self, obstacle: bool, path_error: float, recovery_error: float) -> MissionState:
        if obstacle:
            return self.transition(MissionState.OBSTACLE_DEFLECTION, "obstacle_detected")
        if self.state == MissionState.OBSTACLE_DEFLECTION:
            return self.transition(MissionState.PATH_RECOVERY, "obstacle_cleared")
        if self.state == MissionState.PATH_RECOVERY and path_error <= recovery_error:
            return self.transition(MissionState.RL_NAVIGATE, "path_recovered")
        return self.state
