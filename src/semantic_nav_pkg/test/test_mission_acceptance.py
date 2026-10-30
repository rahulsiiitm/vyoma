import pytest

pytest.importorskip('rclpy')
from scripts.mission_smoke import classify_outcome
from scripts.evaluate_policy_batch import capture_provenance, summarize


def test_mission_requires_valid_safe_clearance():
    phases = ['ARMING', 'TAKEOFF', 'RL_NAVIGATE', 'GOAL_HOLD', 'LAND', 'DISARMED']
    assert classify_outcome(True, phases, 0.4, 0.75, 1.0, 0.75) == (True, True, 'completed')
    assert classify_outcome(True, phases, 0.4, 0.75, 0.744, 0.75) == (
        True, False, 'clearance_below_threshold')
    assert classify_outcome(True, phases, 0.4, 0.75, None, 0.75) == (
        True, False, 'insufficient_clearance_data')


def test_mission_rejects_failsafe_and_invalid_final_position():
    assert classify_outcome(True, ['GOAL_HOLD', 'FAILSAFE_LAND', 'DISARMED'], 0.1, 0.75, 1, 0.75) == (
        False, False, 'mission_aborted')
    assert classify_outcome(True, ['GOAL_HOLD', 'LAND', 'DISARMED'], float('nan'), 0.75, 1, 0.75) == (
        True, False, 'landing_outside_goal_tolerance')


def test_batch_summary_distinguishes_landing_from_passing_safety():
    result = summarize([
        {'success': False, 'lifecycle_completed': True},
        {'success': True, 'lifecycle_completed': True},
        {'success': False, 'lifecycle_completed': False},
    ], planned=5, model_path=None)
    assert result['planned_cases'] == 5
    assert result['completed_cases'] == 3
    assert result['accepted_cases'] == 1
    assert result['goal_and_landing_cases'] == 2


def test_batch_provenance_records_config_and_checkpoint_metadata(tmp_path):
    names = ('src/semantic_nav_pkg/semantic_nav_pkg/mission_manager_node.py',
             'src/semantic_nav_pkg/semantic_nav_pkg/safety_supervisor_node.py',
             'src/semantic_nav_pkg/semantic_nav_pkg/depth_sector_node.py',
             'src/rl_navigation_pkg/rl_navigation_pkg/policy_node.py',
             'src/semantic_nav_pkg/config/autopilot_params.yaml',
             'simulation/worlds/training_world.world')
    for name in names:
        file = tmp_path / name
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text(name)
    model = tmp_path / 'model.v1.zip'
    model.write_bytes(b'model')
    (tmp_path / 'model.v1.metadata.json').write_text('{"algorithm": "PPO"}')
    provenance = capture_provenance(tmp_path, model)
    assert len(provenance['configuration_sha256']) == len(names)
    assert provenance['model_metadata']['algorithm'] == 'PPO'
