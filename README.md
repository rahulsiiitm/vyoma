# Vyoma: Semantic Drone Autopilot

![Vyoma Workbench](assets/banner.jpg)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Simulation](https://img.shields.io/badge/Simulation-PX4%20SITL-orange.svg)](https://px4.io/)
[![Hardware](https://img.shields.io/badge/Hardware-Jetson_Nano-green.svg)](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)
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
| Containerized ROS 2/PX4 environment | Implemented; clean Docker build validation pending |
| Pinned PX4, `px4_msgs`, XRCE-DDS and Python dependencies | Implemented |
| ENU/NED coordinate conversion | Implemented and unit tested |
| Autonomous mission state machine | Implemented |
| Nav2 global-planner integration | Implemented; SITL validation pending |
| 64-sector depth observation | Implemented |
| Three-frame PPO observation history | Implemented |
| Three-axis local RL action contract | Implemented |
| Deterministic safety supervisor | Implemented and unit tested |
| Commissioning obstacle-deflection baseline | Implemented |
| Trained and accepted PPO checkpoint | Not yet available |
| Semantic depth fusion | Planned for Stage 2 |
| Learned vertical avoidance | Planned for Stage 3 |
| Real-aircraft deployment | Blocked until simulation acceptance passes |

The repository currently provides the architecture and training/runtime contracts for Stage 1. It does not include a pre-trained policy, and the complete Gazebo mission acceptance suite has not yet been run.

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

Start the autonomy stack without the runtime policy node, then start training:

```bash
ros2 launch semantic_nav_pkg autonomous.launch.py \
  use_rviz:=false run_policy:=false

ros2 run rl_navigation_pkg train_node
```

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
