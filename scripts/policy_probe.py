"""Offline probe: deterministic PPO action vs. target bearing in an obstacle-free observation."""
import math, sys
from pathlib import Path
import numpy as np
from stable_baselines3 import PPO
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/rl_navigation_pkg'))
from rl_navigation_pkg.observation import NavigationFeatures, ObservationHistory, encode, SECTOR_COUNT

model = PPO.load(sys.argv[1], device='cpu')
ranges = np.full(SECTOR_COUNT, 10.0, dtype=np.float32)
fov = np.zeros(SECTOR_COUNT, dtype=bool); fov[25:39] = True  # ~ front camera sectors
for valid_name, valid in (('all_valid', np.ones(SECTOR_COUNT, bool)), ('front_fov_only', fov)):
    for degrees in (-90, -45, -20, 0, 20, 45, 90):
        b = math.radians(degrees)
        f = NavigationFeatures(ranges=ranges, valid=valid, local_distance=2.0, local_bearing=b,
                               goal_distance=5.0, goal_bearing=b, cross_track_error=0.0,
                               heading_error=b, velocity_forward=0.0, velocity_lateral=0.0,
                               yaw_rate=0.0, previous_action=np.zeros(3), path_blocked=False)
        h = ObservationHistory(); obs = h.append(encode(f))
        a, _ = model.predict(obs, deterministic=True)
        print(f'{valid_name:15s} bearing {degrees:+4d}  action x={a[0]:+.2f} y={a[1]:+.2f} yaw={a[2]:+.2f}')
