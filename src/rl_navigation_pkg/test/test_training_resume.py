import json

import gymnasium as gym
import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('stable_baselines3')

from rl_navigation_pkg import train_node
from rl_navigation_pkg.observation import OBSERVATION_SIZE


class TinyDroneEnv(gym.Env):
    def __init__(self, maximum_steps, scenario_profile='standard'):
        self.observation_space = gym.spaces.Box(-1, 1, (OBSERVATION_SIZE,), dtype=np.float32)
        self.action_space = gym.spaces.Box(-1, 1, (3,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        return np.zeros(OBSERVATION_SIZE, dtype=np.float32), {}

    def step(self, action):
        return np.zeros(OBSERVATION_SIZE, dtype=np.float32), 0.0, True, False, {
            'reached': False, 'collision': False, 'mission_failure': False, 'safety_reason': 'test',
            'distance': 1.0, 'clearance': 2.0}

    def destroy_node(self):
        pass


def test_training_resumes_checkpoint_and_preserves_source(tmp_path, monkeypatch):
    monkeypatch.setattr(train_node, 'DroneEnv', TinyDroneEnv)
    monkeypatch.setattr(train_node.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(train_node.rclpy, 'try_shutdown', lambda: None)
    source = tmp_path / 'first'
    resumed = tmp_path / 'second'
    train_node.main(['train_node', '--timesteps', '4', '--rollout-steps', '4', '--model-path', str(source)])
    train_node.main(['train_node', '--timesteps', '4', '--rollout-steps', '4', '--resume-from',
                     str(source) + '.zip', '--model-path', str(resumed)])
    first = json.loads((tmp_path / 'first.metadata.json').read_text())
    second = json.loads((tmp_path / 'second.metadata.json').read_text())
    assert first['timesteps'] == 4
    assert second['timesteps'] == 8
    assert second['completed'] and second['resumed_from'] == str(source) + '.zip'
    assert (tmp_path / 'first.zip').exists() and (tmp_path / 'second.zip').exists()


def test_sac_saves_and_restores_experience(tmp_path, monkeypatch):
    monkeypatch.setattr(train_node, 'DroneEnv', TinyDroneEnv)
    monkeypatch.setattr(train_node.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(train_node.rclpy, 'try_shutdown', lambda: None)
    first, second = tmp_path / 'sac-first', tmp_path / 'sac-second'
    common = ['--algorithm', 'sac', '--timesteps', '4', '--rollout-steps', '4', '--learning-starts', '2']
    train_node.main(['train_node', *common, '--model-path', str(first)])
    train_node.main(['train_node', *common, '--resume-from', str(first) + '.zip', '--model-path', str(second)])
    metadata = json.loads((tmp_path / 'sac-second.metadata.json').read_text())
    assert metadata['algorithm'] == 'SAC' and metadata['timesteps'] == 8
    assert (tmp_path / 'sac-first.replay.pkl').exists()
    model = train_node.SAC.load(str(second) + '.zip')
    model.load_replay_buffer(str(second) + '.replay.pkl')
    assert model.replay_buffer.size() == 8


def test_resume_rejects_algorithm_mismatch_and_missing_sac_replay(tmp_path):
    source = tmp_path / 'source'
    (tmp_path / 'source.metadata.json').write_text(json.dumps({
        'algorithm': 'SAC', 'schema_version': train_node.SCHEMA_VERSION,
        'observation_size': OBSERVATION_SIZE, 'action_size': 3}))
    with pytest.raises(SystemExit):
        train_node.main(['train_node', '--resume-from', str(source) + '.zip', '--model-path', str(tmp_path / 'other')])
    with pytest.raises(SystemExit):
        train_node.main(['train_node', '--algorithm', 'sac', '--resume-from', str(source) + '.zip',
                         '--model-path', str(tmp_path / 'other')])


def test_periodic_checkpoints_are_saved_with_metadata(tmp_path, monkeypatch):
    monkeypatch.setattr(train_node, 'DroneEnv', TinyDroneEnv)
    monkeypatch.setattr(train_node.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(train_node.rclpy, 'try_shutdown', lambda: None)
    path = tmp_path / 'run'
    train_node.main(['train_node', '--timesteps', '8', '--rollout-steps', '4',
                     '--checkpoint-every', '4', '--model-path', str(path)])
    for steps in (4, 8):
        metadata = json.loads((tmp_path / f'run-{steps}.metadata.json').read_text())
        assert (tmp_path / f'run-{steps}.zip').is_file()
        assert metadata['timesteps'] == steps and metadata['completed'] is False
    assert json.loads((tmp_path / 'run.metadata.json').read_text())['completed'] is True
