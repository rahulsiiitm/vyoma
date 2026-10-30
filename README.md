# Vyoma: Semantic Drone Autopilot

![Vyoma Workbench](assets/banner.jpg)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Simulation](https://img.shields.io/badge/Simulation-PX4%20SITL-orange.svg)](https://px4.io/)
[![Hardware](https://img.shields.io/badge/Hardware-Jetson_Orin_Nano_Super-green.svg)](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit)
[![License](https://img.shields.io/badge/License-Proprietary-red.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Active_Development-yellow.svg)]()

Vyoma is a simulation-first autonomous drone research platform built on **ROS 2 Humble**, **PX4 SITL**, **Gazebo**, **Nav2**, and **Stable-Baselines3 PPO**.

Given an authorized destination, the Stage 1 system is designed to take off, request a global route, use a reinforcement-learning local controller to deflect around obstacles, recover its route, reach the goal, and land without manual steering. PX4 remains responsible for stabilization and motor control, while every learned command passes through an independent deterministic safety supervisor.

---

## Why RL Autonomy

Classical navigation is valuable for global route generation, but manually designed local rules become brittle in cluttered and dynamic environments. Vyoma uses a hybrid architecture: Nav2 provides global guidance, PPO learns local movement and obstacle-deflection behavior, and a deterministic safety layer enforces non-negotiable flight limits.

The policy can slow down, stop, move laterally, turn around an obstacle, pass through a safe opening, and return to the global route. It cannot arm the aircraft, change PX4 modes, command motors directly, or bypass safety constraints.

Semantic perception remains part of the project direction. Geometry-based autonomy is being established first so that semantic class channels can later be introduced and evaluated against a reliable baseline.

---

## System Architecture

### 1. Perception & Sensing Layer

- **Depth processing:** Converts the Gazebo depth-camera `PointCloud2` stream into 64 polar range sectors with validity information.
- **Temporal observations:** Stacks three consecutive observations so PPO can infer changing geometry and future moving-obstacle behavior.
- **Semantic vision:** The YOLOv8 vision package publishes semantic and debug imagery. Metric semantic-depth fusion is planned for Stage 2.

### 2. Planning & RL Navigation Layer

- **Global planning:** Nav2 NavFn generates a global path from the current vehicle pose to the mission goal.
- **Rolling local target:** A path-tracking node selects a look-ahead target along the global route.
- **PPO local control:** The policy observes depth, local and global goal geometry, path error, velocity, yaw rate, obstacle direction, and previous actions.
- **Obstacle deflection:** PPO outputs bounded forward velocity, lateral velocity, and yaw rate, allowing the drone to steer around obstacles and recover the route.
- **Commissioning baseline:** A deterministic deflection controller is available when no trained model is supplied. It exists for integration testing and comparison, not as a replacement for PPO training.

### 3. Mission, Safety & PX4 Layer

- **Mission manager:** Owns the autonomous lifecycle from preflight through takeoff, planning, RL navigation, obstacle deflection, path recovery, landing, and disarming.
- **Safety supervisor:** Rejects invalid or stale policy output, enforces speed and acceleration limits, applies warning-zone slowing, and commands a stop inside the hard-clearance boundary.
- **PX4 adapter:** Is the only component allowed to publish PX4 offboard topics. It owns ROS ENU to PX4 NED coordinate conversion and deterministic altitude control during Stage 1.
- **Failure handling:** Unsafe conditions trigger emergency hold, replanning when conditions recover, or failsafe landing after a persistent emergency.

```text
Mission Goal
    │
    ▼
Mission Manager ──► Nav2 Global Planner ──► Rolling Local Target
                                                    │
Depth PointCloud ──► 64 Sectors ──► PPO Policy ◄────┘
                                      │
                                      ▼
                              Safety Supervisor
                                      │
                                      ▼
                         PX4 ENU/NED Adapter ──► PX4 SITL
```

---

## Project Structure

```text
vyoma/
├── Dockerfile                         # ROS 2 Humble development image
├── requirements.txt                   # Pinned Python and RL dependencies
├── run_docker.sh                      # Build and enter the development container
├── run_simulation.sh                  # Build and launch the complete Stage 1 stack
├── setup_bridge.sh                    # Pinned XRCE-DDS and px4_msgs setup
├── setup_simulation.sh                # Pinned PX4 SITL setup
├── simulation/
│   └── worlds/training_world.world    # Canonical Gazebo training world
└── src/
    ├── semantic_nav_pkg/              # Mission, planning, safety, depth and PX4 nodes
    ├── semantic_vision_pkg/           # YOLOv8 camera perception
    └── rl_navigation_pkg/             # PPO observation, training and inference
```

---

## Current Capability

| Capability | Status |
| :--- | :--- |
| Containerized ROS 2/PX4 environment | Implemented; working simulation container used for training and evaluation |
| Pinned PX4, `px4_msgs`, XRCE-DDS and Python dependencies | Implemented |
| ENU/NED coordinate conversion | Implemented and unit tested |
| Autonomous mission state machine | Implemented and exercised in PX4 SITL |
| Nav2 global-planner integration | Implemented; short missions exercised in SITL |
| 64-sector depth observation | Implemented with stale/invalid scan handling; observed-but-empty sectors read as maximum range, `inf` means outside the camera view (unknown) |
| Three-frame PPO observation history | Implemented |
| Three-axis local RL action contract | Implemented |
| Deterministic safety supervisor | Direction-aware bounded escape that never steers into unobserved sectors, and a hard stop when no observed safe forward sector exists; unit tested, but the obstacle margin still fails in 5 m SITL routes |
| PPO/SAC training and checkpoint resumption | Implemented; PPO is the stronger short-goal candidate in the current matched evaluation |
| Reproducible five-case short-goal evaluation | Implemented; PPO accepted 2/5, SAC 0/5 |
| Trained and accepted obstacle-avoidance policy | Not yet available; no current checkpoint steers toward its target (see below) |
| Per-step mission telemetry | `mission_smoke.py` records target, raw policy action, safety-filtered action, safety reason, pose and Nav2 paths |
| Actual PX4 odometry replay | Blender scene, still image and MP4 available for a measured short mission; 5 m route also has an odometry plot |
| Semantic depth fusion | Planned for Stage 2 |
| Learned vertical avoidance | Planned for Stage 3 |
| Real-aircraft deployment | Out of scope until simulation acceptance passes; hardware integration remains |

The project includes training checkpoints, five-case matched evaluations,
plots, and a Blender replay built from actual PX4 SITL odometry. PPO passed 2/5
short-goal cases; SAC passed 0/5. A 5 m PPO mission completed goal reach, landing
and disarm, but its minimum reported navigation range was 0.43 m against the
0.75 m acceptance threshold. It is therefore a failed obstacle-course result,
not a reliable avoidance policy. The outstanding software milestones are safe
obstacle-course margins and repeatable multi-seed evaluation; physical hardware
integration is separate.

### Latest findings (2026-10-30)

- **Why PPO turned away from its goal:** the `ppo-easy-reward2` checkpoint
  (512 steps) outputs a near-constant command (forward +0.14, lateral -0.13,
  yaw +0.10) whatever the target bearing is. Its constant left yaw makes the
  body-frame command arc away from the goal. Telemetry from a 5 m route to
  (5, -4) confirmed this in SITL: the raw action stayed at about
  [0.36, -0.23, +0.13] while the target was 45-60° to the right. This is
  untrained learned behavior; the frames, path and local-target pipeline were
  checked and are consistent.
- **Pipeline fixes:** PX4 yaw rate is now published in `/vyoma/odometry`
  (previously always 0 in the observation). Depth sectors now distinguish
  "observed, clear" from "not observed", so the safety supervisor no longer
  steers into space the forward camera cannot see.
- **Curriculum fix:** the `easy` profile previously placed goals within ±14° of
  straight ahead, which a constant forward command solves. Goals now span about
  ±115° of bearing at 1.2-2.9 m, so steering has to be learned.
- **Post-fix SITL routes to (5, -4)** both failed the acceptance gate. PPO
  timed out at 4.52 m from the goal with 0.739 m minimum clearance. The
  deterministic baseline timed out at 2.29 m with 0.592 m clearance.
- **Retraining status:** two PPO runs on the fixed pipeline stopped during
  episode resets (one with no navigation observation after a Nav2 planning
  failure, one with the vehicle not disarming after a likely crash). No
  checkpoint passes the offline bearing probe, and none is qualified.

The full record, including commands, failures and evidence paths, is in
`artifacts/implementation-log-2026-10-30.md`.

See [IMPLEMENTATION_CHECKLIST.md](IMPLEMENTATION_CHECKLIST.md) for the measured
results, [RESEARCH.md](RESEARCH.md) for the algorithm rationale, and
`artifacts/eval-ppo-512-safe/summary.json` and
`artifacts/eval-sac-512-safe/summary.json` for case-level outcomes. The rendered
replay is `artifacts/ppo-512-telemetry-replay-v2.mp4` and its still image is
`artifacts/ppo-512-telemetry-replay-v2.png`. The 5 m obstacle route and its
clearance failure are plotted in
`artifacts/mission-ppo-obstacle-5m-default-rate.png` and recorded in the
matching JSON file.

---

## Getting Started (Simulation)

### Prerequisites

- Docker with the daemon running
- X11 server for Gazebo and RViz
- 8 GB RAM minimum; more is recommended for training

### One-Time Setup

1. Start the development container:

   ```bash
   chmod +x run_docker.sh
   ./run_docker.sh
   ```

2. Inside the container, install the pinned bridge and simulation dependencies:

   ```bash
   ./setup_bridge.sh
   ./setup_simulation.sh
   colcon build --symlink-install
   source install/setup.bash
   ```

### Launch the Autonomous Stack

From the host, while the `vyoma-dev` container is running:

```bash
./run_simulation.sh
```

On Windows with Docker Desktop, an existing development container, and WSLg:

```powershell
.\run_simulation.ps1 -Gui
```

This opens Gazebo's actual 3D world. The current simulated vehicle is PX4's
`x500_depth`, a commissioning stand-in for the planned S500 hardware.
Without `-Gui`, the PowerShell launcher runs headless. `-Training` keeps the
policy node in observation-only mode. `-Duration 60` stops a bounded session
60 seconds after PX4 telemetry becomes ready; omit it for an interactive run.
For an experimental faster simulation, `-DepthRate 10` requests a 10 Hz depth
camera. This rate has passed one short mission but has not passed reliability
or obstacle-avoidance acceptance; omit it to keep Gazebo's original rate.

The managed session waits for PX4 odometry and status, writes component logs
under `artifacts/simulation/`, and stops its owned processes on exit or component
failure. It uses a fresh PX4 working directory for each launch and supplies a
heartbeat-only ground station on loopback for headless SITL. It does not disable
PX4 arming checks. Keep the real-aircraft operator link independent of this
simulation helper.

Launcher builds are kept inside the container under `/tmp/vyoma-runtime/` to
avoid reusing CMake caches from a different host mount. Source
`/tmp/vyoma-runtime/install/setup.bash` in additional terminals. PowerShell
accepts `-Workspace`, `-InstallPath`, `-BuildBase`, and `-Px4Source` overrides;
the shell launcher accepts `VYOMA_WORKSPACE`, `VYOMA_INSTALL`, and
`VYOMA_BUILD_BASE`. Use `-SkipBuild` or `VYOMA_SKIP_BUILD=1` only for an already
built runtime.

Submit an autonomous mission goal from another sourced container terminal:

```bash
ros2 topic pub --once /vyoma/mission/command geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 8.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

Without a model path, the stack uses the commissioning controller. To run an accepted PPO checkpoint:

```bash
ros2 launch semantic_nav_pkg autonomous.launch.py \
  model_path:=/absolute/path/ppo_drone_nav.zip
```

The model must have a matching `.metadata.json` file containing the observation schema, tensor sizes, algorithm, seed, Python version, and source revision.

---

## PPO Training

Start the autonomy stack with the policy node publishing observations only,
then start training in another sourced terminal:

```bash
ros2 launch semantic_nav_pkg autonomous.launch.py \
  use_rviz:=false training:=true

ros2 run rl_navigation_pkg train_node
```

For a short pipeline check, specify a smaller rollout and a separate checkpoint:

```bash
ros2 run rl_navigation_pkg train_node --timesteps 128 --rollout-steps 64 \
  --model-path ./artifacts/ppo-smoke
```

A smoke run only checks training integration. It does not demonstrate that the
policy learned reliable navigation. For simulation evaluation on Windows, pass
the checkpoint path as seen inside the container:

```powershell
.\run_simulation.ps1 -ModelPath /mnt/vyoma/artifacts/ppo-smoke.zip
```

To continue a checkpoint after restarting the training simulator, use a new
output name so the earlier checkpoint and episode log are preserved:

```bash
ros2 run rl_navigation_pkg train_node --resume-from ./artifacts/ppo-smoke.zip \
  --timesteps 512 --model-path ./artifacts/ppo-continued
```

The requested timesteps are additional; PPO reuses the checkpoint's rollout
configuration and optimizer. An interrupted partial rollout is discarded, so
the saved timestep counter includes collected steps that may not have produced
an optimizer update. `--device cuda` explicitly requires a CUDA-enabled
PyTorch runtime and an exposed GPU; `--device cpu` selects the CPU.
Reward version 2 distinguishes routine acceleration limiting from emergency
interventions. Compare reward curves only within the same reward version; older
checkpoints used the previous reward function.
For initial goal-reaching practice, `--curriculum easy` samples short goals
(1.2-2.9 m, about ±115° of bearing) before the first obstacle. The default `standard` profile samples longer goals
among obstacles. Report and evaluate the profiles separately: performance on
the easy profile is not evidence of obstacle avoidance.

Long runs can save intermediate checkpoints, each with its own metadata, for
mid-training evaluation:

```bash
ros2 run rl_navigation_pkg train_node --curriculum easy --timesteps 8192   --rollout-steps 512 --checkpoint-every 2048 --model-path ./artifacts/ppo-easy-wide
```

This writes `ppo-easy-wide-2048.zip`, `ppo-easy-wide-4096.zip`, and so on.
Training runs at about 1 step/s in the headless real-time simulator. When the
package is installed without `--symlink-install`, set `VYOMA_PX4_SOURCE` to the
PX4-Autopilot checkout so episode resets can restart the PX4 estimator.

Before evaluating any checkpoint in simulation, check that its action depends
on the target bearing. A policy that outputs the same command for every bearing
cannot navigate:

```bash
python3 scripts/policy_probe.py ./artifacts/ppo-easy-wide-2048.zip
```

SAC is available as an experimental comparison using the same environment:

```bash
ros2 run rl_navigation_pkg train_node --algorithm sac --curriculum easy \
  --timesteps 512 --episode-steps 200 --model-path ./artifacts/sac-easy
```

SAC checkpoints include a replay buffer (`.replay.pkl`), which must be kept
beside the model when resuming. Evaluate PPO and SAC separately with matching
scenarios and seeds; training reward alone does not establish which is better.

After stopping any existing simulation session, run the five short-goal
evaluation cases against a saved policy:

```bash
python3 scripts/evaluate_policy_batch.py --px4-source /path/to/PX4-Autopilot \
  --model-path ./artifacts/ppo-easy-reward2.zip --output-dir ./artifacts/eval-ppo-easy
```

Omit `--model-path` to evaluate the commissioning baseline. Each case writes
its mission result and sampled odometry path. The summary distinguishes goal
and landing completion from passing the clearance gate. These short goals do
not establish obstacle-course performance.

Use the actual container workspace path if it differs from `/mnt/vyoma`.
Training resets require the simulated vehicle to land, disarm and settle near
the start. Resetting from an arbitrary landing location is not yet established.

The Stage 1 observation contains:

- 64 normalized depth sectors and 64 validity values
- Local-target distance and bearing
- Final-goal distance and bearing
- Cross-track and heading error
- Forward/lateral velocity and yaw rate
- Previous three-axis action
- Closest-obstacle distance and direction
- Path-blocked indicator
- Three-frame temporal history

The action is `[forward velocity, lateral velocity, yaw rate]`. Rewards favor route progress, safe obstacle clearance, smooth control, path recovery, and mission completion. Collisions, unsafe proximity, oscillation, control jerk, and safety overrides are penalized.

Training artifacts are written below `artifacts/`. A completed training run is not automatically approved for autonomous use; it must pass held-out seeded missions and failure-injection tests.

---

## Public ROS Interfaces

| Topic | Message | Purpose |
| :--- | :--- | :--- |
| `/vyoma/mission/command` | `geometry_msgs/PoseStamped` | Authorized simulation goal |
| `/vyoma/mission/state` | `std_msgs/String` JSON | Timestamped mission state and reason |
| `/vyoma/px4/status` | `std_msgs/String` JSON | PX4 connection and flight status |
| `/vyoma/depth/sectors` | `sensor_msgs/LaserScan` | Timestamped 64-sector depth observation |
| `/vyoma/global_path` | `nav_msgs/Path` | Nav2-generated route |
| `/vyoma/local_target` | `geometry_msgs/PoseStamped` | Rolling PPO target |
| `/vyoma/rl/observation` | `std_msgs/Float32MultiArray` | Versioned stacked policy observation |
| `/vyoma/rl/raw_action` | `geometry_msgs/TwistStamped` | Policy command before safety filtering |
| `/vyoma/control/safe_command` | `geometry_msgs/TwistStamped` | Supervisor-approved body-frame command |
| `/vyoma/safety/status` | `std_msgs/String` JSON | Clearance, override state, and reason |
| `/vyoma/navigation/blocked` | `std_msgs/Bool` | Global replanning request |

---

## Tests

From the repository root, run `python -m pytest -q`. The root `pytest.ini`
configures both Python package paths. Outside ROS, node-level tests are skipped;
the deterministic mission, safety, frame, path, observation, scenario, and reset
tests still run with only NumPy and pytest installed.

For node-level regression checks, source `/opt/ros/humble/setup.bash` and your
built workspace's `install/setup.bash`, then run the same command. These checks
exercise the ROS callbacks and message contracts; they do not replace live
Gazebo reset cycles, flight missions, or trained-policy acceptance.

ROS tests run in DDS domain 196 to keep test publishers away from live
simulation. Set `VYOMA_TEST_DOMAIN_ID` if that domain is already in use.

With the simulation running, measure a short commissioning mission from another
sourced container terminal at the repository root:

```bash
python3 scripts/mission_smoke.py --x 1.5 --y 0.0 --timeout 180
```

The JSON result records mission phases, actual arming, altitude, path length,
navigation clearance, final goal distance, and elapsed wall time. A successful
lifecycle alone is insufficient: the smoke check also requires landing within
0.75m of the goal and a minimum reported navigation clearance of at least
0.75m. During navigation it also records 5 Hz `telemetry` rows (state, pose,
local target, raw and safe actions, safety reason, clearance) and every Nav2
`paths` update. These rows separate learned behavior from safety intervention. This short mission is an integration check, not an obstacle
avoidance or policy acceptance result.

---

## Roadmap & Current Status

### Phase 1: Geometry-Based RL Autopilot

- [x] Separate mission, safety, planning, RL and PX4 responsibilities
- [x] Implement fixed-shape depth observation and temporal history
- [x] Implement forward/lateral/yaw PPO action interface
- [x] Implement deterministic safety boundary
- [x] Add global replanning and route-recovery states
- [ ] Complete clean Docker/PX4/Gazebo integration validation
- [ ] Build reliable Gazebo episode reset and randomization
- [ ] Train PPO curriculum for static obstacle deflection
- [ ] Evaluate at least 100 unseen seeded missions
- [ ] Reach the Stage 1 success and safety thresholds

### Phase 2: Semantic Autonomy

- [ ] Project YOLO detections through metric depth
- [ ] Add semantic class channels to the PPO observation
- [ ] Train class-aware clearance behavior
- [ ] Evaluate humans, vehicles, vegetation, structures and unknown obstacles

### Phase 3: Dynamic & 3D Autonomy

- [ ] Train crossing and head-on moving-obstacle scenarios
- [ ] Add upward and downward clearance observations
- [ ] Add bounded vertical velocity to the PPO action
- [ ] Train climb-over, descend-under and lateral-around decisions

### Phase 4: Hardware Readiness

- [ ] Reproduce accepted policies on Jetson hardware-in-the-loop
- [ ] Add physical kill switch and operator arming controls
- [ ] Complete a separate flight-safety review
- [ ] Begin controlled Pixhawk/Jetson flight testing

---

## Safety Notice

Vyoma is research software and is not approved for operation on a real aircraft. Simulation success alone is insufficient for hardware deployment. Real flight requires hardware-in-the-loop validation, explicit operator authorization, independent emergency controls, geofencing, and a dedicated safety review.

---

## Contributing

Contributions should preserve the authority boundary: PPO may control local navigation, but PX4 stabilization and the deterministic safety supervisor remain authoritative. Particularly useful contributions include simulation curriculum design, reset reliability, policy evaluation, semantic-depth projection, and failure-injection testing.

## License

This project is licensed under a Proprietary License — see the [LICENSE](LICENSE) file for details. All rights reserved.

---

## Author

**Rahul**
B.Tech, Computer Science & Engineering (AI & Data Science) — IIIT Manipur
Building **Sutra**, a production RAG knowledge assistant, at IIT Roorkee's Academic Affairs Department
Author of **VidChain** (multimodal video RAG library, published on PyPI)

> *"The next frontier in robotics isn't just navigating the world—it's understanding it."*
