import rclpy
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from rl_navigation_pkg.drone_env import DroneEnv
import os
import json
import platform
import subprocess

from rl_navigation_pkg.observation import OBSERVATION_SIZE, SCHEMA_VERSION

def save_checkpoint(model, model_path, completed):
    model.save(model_path)
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "observation_size": OBSERVATION_SIZE,
        "action_size": 3,
        "algorithm": "PPO",
        "seed": 42,
        "timesteps": int(model.num_timesteps),
        "completed": completed,
        "python": platform.python_version(),
        "git_commit": subprocess.run(
            ["git", "rev-parse", "HEAD"], capture_output=True, text=True, check=False
        ).stdout.strip() or "unknown",
    }
    with open(model_path + ".metadata.json", "w", encoding="utf-8") as metadata_file:
        json.dump(metadata, metadata_file, indent=2)
    print(f"Model saved to {model_path}.zip")


def main(args=None):
    rclpy.init(args=args)
    
    env = DroneEnv()
    
    # Optional: Check if environment follows gym API
    # check_env(env)
    
    model_path = os.path.join(os.getcwd(), "artifacts", "ppo_drone_nav")
    os.makedirs(os.path.dirname(model_path), exist_ok=True)
    
    print("Starting PPO Training for Drone Navigation...")
    # Initialize PPO agent
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        tensorboard_log="./artifacts/tensorboard/",
        seed=42,
    )
    
    try:
        # Train the model
        model.learn(total_timesteps=100000)
        save_checkpoint(model, model_path, completed=True)
    except KeyboardInterrupt:
        print("Training interrupted! Saving current progress...")
        save_checkpoint(model, model_path, completed=False)
    finally:
        env.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
