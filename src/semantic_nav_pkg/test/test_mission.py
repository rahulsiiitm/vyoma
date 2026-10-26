from semantic_nav_pkg.mission import MissionMachine, MissionState


def test_obstacle_deflection_and_recovery():
    mission = MissionMachine()
    mission.transition(MissionState.RL_NAVIGATE, 'test')
    assert mission.update_navigation(True, 2.0, 0.75) == MissionState.OBSTACLE_DEFLECTION
    assert mission.update_navigation(False, 2.0, 0.75) == MissionState.PATH_RECOVERY
    assert mission.update_navigation(False, 0.5, 0.75) == MissionState.RL_NAVIGATE


def test_airborne_emergency_holds():
    mission = MissionMachine()
    mission.transition(MissionState.RL_NAVIGATE, 'test')
    assert mission.emergency('sensor_failure') == MissionState.EMERGENCY_HOLD


def test_episode_reset_clears_mission_state():
    mission = MissionMachine()
    mission.transition(MissionState.OBSTACLE_DEFLECTION, 'test')
    assert mission.reset() == MissionState.DISARMED
    assert mission.reason == 'episode_reset'
