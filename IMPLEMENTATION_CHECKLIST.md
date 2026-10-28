# Vyoma Hybrid Autonomy Implementation Checklist

This checklist is ordered by dependency. A phase is complete only when its
acceptance checks pass; code presence alone does not count as completion.

### Verification on 2026-10-26

- Fixed duplicate odometry reset acceptance, invalid observation/action handling,
  zero-clearance reward handling, and the observation-only training command.
- Fixed rolling-target selection behind the vehicle, reversed commissioning
  deflection, and the PX4 adapter's odometry twist frame.
- Added PX4 command freshness/frame/finiteness checks and handling of policy
  loss during navigation without blocking preflight on an absent policy.
- Windows: 35 deterministic tests; ROS-only test modules require the container.
- Existing ROS Humble container: 52 tests passed, including node callbacks and
  stopping expired PX4 commands after supervisor loss.
- A clean colcon build of px4_msgs and all three application packages succeeded
  using separate build/install directories; all 52 tests also passed against it.
  This used the existing container image, not a fresh Docker image build.
- PX4 SITL and Gazebo reached startup in the existing container; this does not
  establish sensor rendering, autonomous mission success, or reset reliability.
- A Gazebo depth point cloud was received. Nav2 reached its active lifecycle
  state, but PX4 reported heading/magnetic preflight failures and bounded ROS
  CLI checks received no PX4 odometry/status. End-to-end readiness is unproven.
- No trained policy or simulation acceptance thresholds have been established.

### Continued integration verification on 2026-10-26

- Added managed simulation sessions, finite-telemetry startup checks, component
  logs, fresh PX4 working directories, process-group cleanup, and a Windows
  launcher. A headless session's shutdown left no owned PX4/Gazebo/DDS/bridge
  processes. A separately opened Gazebo viewer is intentionally independent.
- Opened Gazebo's actual 3D viewer on Windows through WSLg. Added `-Gui` to the
  PowerShell launcher; GUI framing is pending while the desktop is locked.
- Matched the world physics, magnetic field, and geographic origin to the pinned
  PX4 reference world; obstacles and walls now intersect the 5m flight plane.
- Fixed planning before takeoff, camera mounting transform, world-frame costmap
  height limits, 2Hz PX4 status freshness, and confirmed arming/disarming gates.
- Added horizontal position hold for takeoff, planning, emergency hold, and
  landing. Added a regression for PX4 NumPy scalar conversion into ROS setpoints.
- Vectorized depth projection; a synthetic 307,200-point cloud took about 31ms in
  one container measurement. This is not an end-to-end perception latency result.
- Added a measured mission smoke harness. One run completed arming, takeoff,
  goal arrival, landing, and disarming, but drifted 2.58m from the goal during
  descent. The acceptance check now also requires landing within 0.75m; lifecycle
  completion alone is not accepted. Subsequent landing validation is pending.
- An intermittent native PX4 mutex assertion occurred in earlier runs. Native
  backtrace support was added; root cause and repeated stability remain unproven.
- ROS regression tests are isolated in DDS domain 196. An earlier live mission
  contaminated by same-domain test publishers was discarded.
- Latest ROS regression run: 65 passed. Updated application packages built.
- RViz now displays the real route, local target, odometry and depth topics;
  its goal tool routes through the mission manager. Configuration parses, but
  the updated RViz rendering has not yet been visually checked.
- The final commissioning run at goal (1.5m, 0m) passed acceptance version 2:
  actual arming, takeoff, goal arrival, landing, and confirmed disarming, with
  about 0.31m final goal error. Evidence:
  `artifacts/mission-20261026-131054.json`. This was one short mission under gdb;
  it does not resolve the intermittent native assertion or establish obstacle
  avoidance, repeated-reset, training, or hardware acceptance.
- Process cleanup now includes gdb inferiors in separate process groups within
  the owned session. A regression test covers this; the orphaned debug processes
  identified during integration were stopped before the final run.

## 0. Repository stabilization

- [ ] Review and commit the existing cleanup as a coherent baseline.
- [ ] Validate a clean Docker image build from an empty cache.
- [ ] Start and stop the complete simulation with one command.
- [ ] Confirm that shutdown leaves no PX4, Gazebo, DDS, or ROS processes.

## 1. Deterministic episode reset

- [x] Add deterministic, seed-based scenario generation.
- [x] Add reset lifecycle and vehicle-settling validation primitives.
- [x] Integrate reset validation with the ROS/Gazebo training environment.
- [x] Add coordinated reset services for PX4 adapter, mission, safety, and policy state.
- [ ] Verify PX4, mission, observations, actions, goals, and obstacles in live simulation.
- [ ] Reject incomplete or timed-out resets instead of training on them.
- [ ] Pass 1,000 consecutive seeded reset cycles without restarting Gazebo.

## 2. Runtime contracts

- [ ] Freeze versioned vehicle-state, perception, planning, and control topics.
- [ ] Enforce timestamps, frames, validity, and schema versions.
- [ ] Verify that only the PX4 adapter publishes to `/fmu/in/*`.
- [ ] Add ROS graph contract tests.

## 3. Classical navigation baseline

- [ ] Retain Nav2 global planning for Stage 1 constant-altitude flight.
- [ ] Implement a deterministic rolling-target selector.
- [ ] Implement a constrained local trajectory/MPC baseline.
- [ ] Add safe stop and global replanning for blocked routes.
- [ ] Record success, clearance, path length, time, jerk, and latency.

## 4. Independent safety boundary

- [ ] Validate freshness, finiteness, PX4 state, and obstacle clearance.
- [ ] Enforce velocity, acceleration, jerk, yaw-rate, and geofence limits.
- [ ] Add braking, hold, replan, landing, and manual-override responses.
- [ ] Pass automated sensor, policy, DDS, and estimator failure injection.

## 5. RL environment

- [ ] Change the learned action from final control to bounded local intent.
- [ ] Finalize observation, reward, termination, and normalization contracts.
- [ ] Add progressive curriculum scenarios and domain randomization.
- [ ] Store schema, configuration, seed, revision, and metrics per checkpoint.

## 6. Training and comparison

- [ ] Train PPO.
- [ ] Train SAC using the same scenarios and evaluation seeds.
- [ ] Evaluate both against the classical baseline on unseen missions.
- [ ] Promote only versioned checkpoints that pass acceptance thresholds.

## 7. Hybrid operation

- [ ] Implement baseline, shadow, and hybrid operating modes.
- [ ] Log RL proposals and hypothetical interventions in shadow mode.
- [ ] Fall back to the classical planner on invalid or late RL output.
- [ ] Validate every selected trajectory before it reaches the PX4 adapter.

## 8. Simulation release gate

- [ ] Run at least 1,000 unseen seeded missions.
- [ ] Achieve at least 95% success in the supported operational domain.
- [ ] Produce zero post-safety hard-clearance violations.
- [ ] Handle 100% of defined injected failures safely.
- [ ] Demonstrate a measurable benefit over the classical baseline.

## 9. Hardware readiness (after hardware details are available)

- Hardware inventory supplied on 2026-10-26:
  - Pixhawk 2.4.8; installed PX4 version and board/flash identification pending.
  - Jetson Orin Nano Super 8GB developer kit; user reports JetPack 7.2.
  - Arducam Sony IMX477 12.3MP RGB camera with swappable adjustable M12 lens.
  - Arducam Sony IMX219 8MP RGB camera with autofocus lens.
  - NEO-7M GPS module.
  - S500 drone frame kit.
  - T-Motor Air Gear 450 propulsion system with 10-inch propellers.
  - 5200mAh 4S 14.8V LiPo, 35C continuous / 70C burst, 76.96Wh nominal.
- Start with PX4 to match the existing adapter. ArduPilot would require a
  separately implemented and verified control integration. Confirm that the
  actual firmware includes uXRCE-DDS before assuming `/fmu/*` connectivity;
  board flash constraints may prevent the current transport from being used.
- NVIDIA currently documents Orin support for JetPack 7.2/7.2.1. Confirm the
  installed L4T release and exact Arducam SKUs/driver support before selecting
  the ARM64 runtime; the existing Humble x86 CPU image is a simulation image.
- Two different RGB cameras are not a validated stereo depth system. Camera
  mounting, synchronization, calibration, autofocus behavior, and software
  metric-depth support have not been established.
- Ultrasonic models/count/orientation/range/FOV/connection, camera roles, and
  indoor/outdoor operating domain remain undecided. The latest inventory does
  not include ultrasonic hardware. Current simulation depth sectors do not
  implement this hardware sensor stack.
- Jetson regulator/power wiring, RC/manual override, storage, final vehicle mass,
  measured thrust/current, and payload endurance remain to be verified; the
  supplied battery label alone does not establish payload capacity or flight time.
- RGB detections do not provide the metric 64-sector depth observation used by
  the current policy. Implement and validate range fusion and unknown-space
  handling, then match training observations to measured sensor coverage before
  promoting a simulation policy for hardware evaluation.

- [ ] Build the Jetson ARM64 runtime image for the installed JetPack version.
- [ ] Match PX4, `px4_msgs`, ROS 2, and XRCE-DDS versions.
- [ ] Configure and validate the Pixhawk-Jetson communication link.
- [ ] Calibrate sensors and measure real latency, noise, and dropout rates.
- [ ] Complete HIL, shadow-flight, restrained-flight, and bounded-flight gates.

Hardware compatibility references checked on 2026-10-26:

- [NVIDIA JetPack archive](https://developer.nvidia.com/embedded/jetpack-archive)
- [NVIDIA Orin Nano setup](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/quick_start.html)
- [Arducam camera and JetPack support](https://docs.arducam.com/camera-for-jetson-nano/)
- [PX4 uXRCE-DDS firmware and version requirements](https://docs.px4.io/main/en/middleware/uxrce_dds)
