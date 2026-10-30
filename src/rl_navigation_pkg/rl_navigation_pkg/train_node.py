import argparse
import rclpy
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from rl_navigation_pkg.drone_env import DroneEnv
import os
import json
import platform
import subprocess

from rl_navigation_pkg.observation import OBSERVATION_SIZE, SCHEMA_VERSION

def save_checkpoint(model, model_path, completed, seed=42, resumed_from=None, curriculum='standard'):
    model.save(model_path)
    algorithm = 'SAC' if isinstance(model, SAC) else 'PPO'
    if algorithm == 'SAC':
        model.save_replay_buffer(model_path + '.replay.pkl')
    git = ['git', '-c', f'safe.directory={os.getcwd()}']
    revision = subprocess.run(git + ['rev-parse', 'HEAD'], capture_output=True,
                              text=True, check=False)
    worktree = subprocess.run(git + ['status', '--porcelain'], capture_output=True,
                              text=True, check=False)
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "reward_version": 2,
        "observation_size": OBSERVATION_SIZE,
        "action_size": 3,
        "algorithm": algorithm,
        "seed": seed,
        "resumed_from": resumed_from,
        "curriculum": curriculum,
        "rollout_steps": int(model.n_steps) if algorithm == 'PPO' else None,
        "replay_buffer": model_path + '.replay.pkl' if algorithm == 'SAC' else None,
        "timesteps": int(model.num_timesteps),
        "completed": completed,
        "python": platform.python_version(),
        "git_commit": revision.stdout.strip() or "unknown",
        "git_dirty": bool(worktree.stdout.strip()) if worktree.returncode == 0 else None,
    }
    with open(model_path + ".metadata.json", "w", encoding="utf-8") as metadata_file:
        json.dump(metadata, metadata_file, indent=2)
    print(f"Model saved to {model_path}.zip")


class PeriodicCheckpoint(BaseCallback):
    """Save schema-tagged intermediate checkpoints so long runs can be probed mid-training."""

    def __init__(self, model_path, every, **metadata):
        super().__init__()
        self.model_path, self.every, self.metadata = model_path, every, metadata
        self.last_saved = 0

    def _on_step(self):
        if self.num_timesteps - self.last_saved >= self.every:
            self.last_saved = self.num_timesteps
            save_checkpoint(self.model, f'{self.model_path}-{self.num_timesteps}', completed=False,
                            seed=self.model.seed, **self.metadata)
        return True


def main(args=None):
    parser = argparse.ArgumentParser(description='Train a Stage 1 policy in simulation')
    parser.add_argument('--algorithm', choices=('ppo', 'sac'), default='ppo')
    parser.add_argument('--timesteps', type=int, default=100000)
    parser.add_argument('--rollout-steps', type=int, default=2048)
    parser.add_argument('--episode-steps', type=int, default=1200)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--resume-from', help='Continue training from an existing checkpoint')
    parser.add_argument('--device', choices=('auto', 'cpu', 'cuda'), default='auto')
    parser.add_argument('--curriculum', choices=('standard', 'easy'), default='standard')
    parser.add_argument('--learning-starts', type=int, default=64,
                        help='Number of initial SAC exploration steps')
    parser.add_argument('--checkpoint-every', type=int, default=0,
                        help='Also save <model-path>-<steps> every N timesteps (0 disables)')
    parser.add_argument('--model-path', help='Output checkpoint path; defaults to artifacts/<algorithm>_drone_nav')
    options = parser.parse_args(rclpy.utilities.remove_ros_args(args=args)[1:])
    if options.timesteps < 1 or options.rollout_steps < 2 or options.episode_steps < 1 or options.learning_starts < 0 or options.checkpoint_every < 0:
        parser.error('timesteps and episode steps must be positive; rollout steps must be at least 2')
    if options.device == 'cuda':
        import torch
        if not torch.cuda.is_available():
            parser.error('CUDA was requested but is not available to PyTorch in this container')
    model_path = os.path.abspath(options.model_path or os.path.join(
        os.getcwd(), 'artifacts', options.algorithm + '_drone_nav'))
    if model_path.endswith('.zip'):
        model_path = model_path[:-4]
    if options.resume_from and os.path.abspath(options.resume_from.removesuffix('.zip')) == model_path:
        parser.error('use a different --model-path when resuming to preserve the source checkpoint and monitor log')
    if options.resume_from:
        source_base = os.path.abspath(options.resume_from.removesuffix('.zip'))
        try:
            with open(source_base + '.metadata.json', encoding='utf-8') as stream:
                source_metadata = json.load(stream)
        except (OSError, ValueError) as exc:
            parser.error(f'cannot read checkpoint metadata: {exc}')
        if source_metadata.get('algorithm') != options.algorithm.upper():
            parser.error('checkpoint algorithm does not match --algorithm')
        if (source_metadata.get('schema_version') != SCHEMA_VERSION or
                source_metadata.get('observation_size') != OBSERVATION_SIZE or
                source_metadata.get('action_size') != 3):
            parser.error('checkpoint observation or action schema is incompatible')
        if options.algorithm == 'sac' and not os.path.isfile(source_base + '.replay.pkl'):
            parser.error('SAC resume requires the matching replay buffer')
    os.makedirs(os.path.dirname(model_path), exist_ok=True)
    rclpy.init(args=args)
    raw_env = DroneEnv(maximum_steps=options.episode_steps, scenario_profile=options.curriculum)
    env = Monitor(raw_env, filename=model_path + '.monitor.csv',
                  info_keywords=('reached', 'collision', 'mission_failure', 'safety_reason',
                                 'distance', 'clearance'))
    
    print(f"Starting {options.algorithm.upper()} Training for Drone Navigation...")
    model = None
    try:
        cls = PPO if options.algorithm == 'ppo' else SAC
        if options.resume_from:
            model = cls.load(options.resume_from, env=env, device=options.device)
            if options.algorithm == 'sac':
                model.load_replay_buffer(source_base + '.replay.pkl')
        else:
            common = dict(verbose=1, learning_rate=3e-4, gamma=0.99,
                          tensorboard_log='./artifacts/tensorboard/', seed=options.seed,
                          device=options.device)
            if options.algorithm == 'ppo':
                model = PPO('MlpPolicy', env, n_steps=options.rollout_steps,
                            batch_size=min(64, options.rollout_steps), n_epochs=10,
                            gae_lambda=0.95, **common)
            else:
                model = SAC('MlpPolicy', env, buffer_size=10_000, batch_size=64,
                            learning_starts=options.learning_starts, **common)
        # Train the model
        callback = (PeriodicCheckpoint(model_path, options.checkpoint_every, resumed_from=options.resume_from,
                                       curriculum=options.curriculum)
                    if options.checkpoint_every else None)
        model.learn(total_timesteps=options.timesteps, reset_num_timesteps=not bool(options.resume_from),
                    callback=callback)
        save_checkpoint(model, model_path, completed=True, seed=model.seed, resumed_from=options.resume_from,
                        curriculum=options.curriculum)
    except KeyboardInterrupt:
        print("Training interrupted! Saving current progress...")
        if model is not None:
            save_checkpoint(model, model_path, completed=False, seed=model.seed, resumed_from=options.resume_from,
                            curriculum=options.curriculum)
    except Exception:
        if model is not None:
            save_checkpoint(model, model_path, completed=False, seed=model.seed, resumed_from=options.resume_from,
                            curriculum=options.curriculum)
        raise
    finally:
        env.close()
        raw_env.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
