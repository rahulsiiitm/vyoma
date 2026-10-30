# Vyoma Hybrid Autonomy Implementation Checklist

This checklist is ordered by dependency. A phase is complete only when its
acceptance checks pass; code presence alone does not count as completion.

### Verification on 2026-10-30

- Added resumable PPO and SAC training, bounded replay-buffer persistence for
  SAC, algorithm-aware inference loading, reward version 2, and an easy
  short-goal curriculum. Added an independent five-case evaluation runner
  that stores PX4 odometry, acceptance outcomes and selected source/config
  fingerprints for each model.
- Fixed emergency-hold recovery to require continuous fresh clear status and
  made empty or invalid depth scans unknown rather than clear. Smoke acceptance
  now rejects a completed lifecycle if clearance is missing or below 0.75m.
- Matched evaluation of the 512-step easy PPO and SAC checkpoints used the same
  five short goals, safety thresholds and mission timeout. PPO reached, landed,
  disarmed and passed the clearance gate in 2/5 cases. SAC passed 0/5. Neither
  model is accepted for obstacle avoidance; these goals do not establish
  obstacle-course performance. Case-level results and provenance are in
  `artifacts/eval-ppo-512-safe/summary.json` and
  `artifacts/eval-sac-512-safe/summary.json`.
- Blender replay uses the passing PPO case's recorded PX4 positions and
  orientations. It is explicitly labeled as a schematic telemetry replay, not
  Gazebo footage. Preview image: `artifacts/ppo-512-telemetry-replay-v2.png`;
  editable scene: `artifacts/ppo-512-telemetry-replay-v2.blend`.
- Replaced the global-minimum-only hard stop with direction-aware sector checks
  and a bounded escape heading when the intended direction is blocked. Unit
  tests verify side clearance, front escape selection, and stopping when every
  forward heading is blocked. The escape margin is a tunable simulation
  parameter; it has not yet produced an accepted obstacle route.
- A 5m PPO mission at PX4's default camera rate completed goal reach, landing
  and disarm, with 0.373m final goal error. Its minimum reported navigation
  range was 0.425m, below the 0.75m gate, so the mission failed acceptance. The
  path plot and full odometry are `artifacts/mission-ppo-obstacle-5m-default-rate.png`
  and `artifacts/mission-ppo-obstacle-5m-default-rate.json`. A 3.0m escape-margin
  trial also completed but measured only 0.408m minimum range. These runs show
  route completion, not safe obstacle avoidance; a 10Hz trial additionally
  reproduced the PX4 mutex assertion during landing.
- The clean ROS build finished all four packages and the application regression
  suite passed 95 tests. This verifies builds and software behavior, not flight
  reliability. Robust obstacle-course acceptance and multi-seed reliability
  remain software work; hardware integration, semantic perception and vertical
  avoidance remain later phases.

### Verification on 2026-10-29

- Added PPO checkpoint resumption and explicit device selection. A small test
  trained a checkpoint, resumed it and verified that the step count increased
  from four to eight without overwriting the original.
- Resumed a 512-step checkpoint for another 512 steps. The run completed and
  saved `artifacts/ppo-resumed-1024.zip`; none of its six completed episodes
  reached a goal, and three ended with mission failures. The episode chart is
  `artifacts/ppo-resumed-1024-rewards.png`. This is not evidence of a successful
  navigation policy.
- Added reward version 2, which does not penalize routine acceleration limiting
  as an emergency intervention. Added an easy short-goal curriculum separate
  from the obstacle scenarios. A fresh 512-step easy run reached its goal in
  four of seven completed training episodes, with two mission failures and one
  episode reaching its step limit. Evidence: `artifacts/ppo-easy-reward2.zip`,
  `.metadata.json`, `.monitor.csv`, and `artifacts/ppo-easy-reward2.png`.
  A separate deterministic 1.5m-goal mission loaded this checkpoint but ended
  in emergency hold and failsafe landing, 0.757m from the goal, with a minimum
  reported navigation clearance of 0.749m. Evidence:
  `artifacts/mission-ppo-easy-eval-1.json`. Training results are not evidence
  of reliable navigation or obstacle avoidance.
- The ROS application regression suite passed 82 tests in the existing
  container. The PX4 scheduler patch was revised and the SITL binary rebuilt;
  its long-running stability is under test.
- On the revised PX4 build, a resumed PPO run completed 512 additional steps
  and saved `artifacts/ppo-lockstep-v2-1536.zip`. Ten episodes completed;
  nine ended on hard-clearance intervention and none reached a goal. This
  session did not reproduce the earlier clock stall, but one session is not a
  reliability proof. It used the previous reward and standard curriculum.
- A further 512-step easy-curriculum continuation completed and saved
  `artifacts/ppo-easy-reward2-1024.zip`; its three completed episodes ended at
  hard clearance. A separate 1.5m mission reached its goal, landed and
  disarmed in 50.9 seconds, 0.424m from the goal, but reported a minimum
  navigation clearance of 0.744m. It therefore does not satisfy the 0.75m
  clearance gate. Evidence: `artifacts/mission-ppo-easy-1024-eval-1.json` and
  the recorded path plot `artifacts/mission-ppo-easy-1024-eval-1.png`. The
  recorded JSON used acceptance version 2, which did not enforce this gate.
- Mission acceptance version 3 separately requires measured clearance at or
  above the threshold. Added SAC as a separate experimental algorithm, including
  saving and restoring its replay buffer. A 512-step SAC easy-curriculum run
  completed four episodes: one reached a goal during initial random exploration,
  two had mission failures and one reached its step limit. Evidence:
  `artifacts/sac-easy-512.metadata.json`, `.monitor.csv`, and `.png`. This does
  not establish SAC policy performance. The application tests passed 86 cases.
- A five-case independent evaluation of the 1,024-step PPO checkpoint under the
  previous recovery rule accepted 3 of 5 short-goal missions. One reached the
  goal but failed the clearance gate; one ended in failsafe. This is not an
  obstacle-course evaluation. Evidence: `artifacts/eval-ppo-easy-1024/summary.json`.
- After observing rapid emergency-hold/replan cycling in SAC evaluation, the
  mission manager was changed to require one second of fresh, sufficiently
  clear safety status before recovery. Persistent danger still triggers failsafe
  landing. Unit tests cover recovery gaps, stale statuses and timeout. A two-case
  PPO recheck under the new rule accepted one and failed one; broader comparisons
  remain necessary. Application tests passed 90 cases.

### Verification on 2026-10-28

- Fixed Windows launcher commands containing CRLF when passed to container Bash.
- Built the four ROS packages in a fresh workspace build directory; 84 ROS
  regression tests passed in the existing container image.
- A commissioning-controller mission to (1.5m, 0m) armed, took off, reached
  the goal, landed 0.277m from it and disarmed. Evidence:
  `artifacts/mission-20261028-063739.json`. One run does not establish reliability.
- PPO reset now preserves simulation time, checks the settled start pose, waits
  for navigation observations, and requests landing and confirmed disarm
  between episodes. Gazebo's native pose service returns the disarmed model to
  the origin; a 1.5m displaced start and two short episodes were exercised.
  The latest reset additionally requires fresh PX4 preflight readiness, level
  attitude, low angular speed and two seconds of stable simulation time; two
  short episodes passed these checks.
  Repeated-reset reliability and arbitrary landing scenarios remain unproven.
- A bounded PPO smoke run completed 128 timesteps and saved
  `artifacts/ppo-smoke-v2.zip` with metadata. This is an experimental checkpoint,
  not an accepted policy or evidence of learning improvement.
- A mission with that checkpoint reached the goal and began landing but timed
  out at 180 seconds before confirmed disarm. Evidence:
  `artifacts/mission-ppo-smoke-v2.json`. No success was recorded for that run.
- A second mission using the same checkpoint lost PX4 status during takeoff and
  timed out. PX4 internal odometry timestamps stopped while Gazebo time kept
  advancing. Evidence: `artifacts/mission-ppo-smoke-v2-retry.json`.
- A third mission under gdb with the same checkpoint and a 270-second limit
  armed, reached the 1.5m goal, landed 0.426m from it and disarmed in 234.5
  seconds. Evidence: `artifacts/mission-ppo-smoke-debug.json`. This one short
  success does not establish generalization, learning benefit or stability.
- A 512-step PPO run was stopped at 220 timesteps after a later reset cycle
  entered an unstable simulated attitude. The partial checkpoint is marked
  incomplete. Longer training and reset stability are still blocked.
- A later 256-step run was stopped at 131 collected steps after a short episode
  and unstable takeoff. `artifacts/ppo-256.monitor.csv` records a 128-step
  episode with reward -402.84 and a 4-step episode with reward -113.78. These
  are training rewards, not success rates or evidence of improvement.
- After adding the preflight and stability gates, a 192-step PPO run completed,
  including a 100-step episode and a reset into a second episode. Its first
  episode had reward -303.68, reached no goal and ended at the step limit.
  Evidence: `artifacts/ppo-192-gated.monitor.csv` and
  `artifacts/ppo-192-gated.metadata.json`. It remains an experimental checkpoint.
- A 300-second 1.5m-goal evaluation of that checkpoint timed out without
  reaching the goal, with 1.90m final distance. Evidence:
  `artifacts/mission-ppo-192-gated.json`.
- Exposing the laptop RTX 3050 to a separate copy of the container did not
  improve a short idle simulation clock measurement (0.347x with GPU access,
  0.355x without). These single samples do not establish performance limits;
  the installed PyTorch build is CPU-only and NVIDIA GPU memory use was zero.
- Reducing the simulated depth camera from 30Hz to 10Hz raised one idle clock
  measurement from 0.352x to 0.974x real time. One 10Hz commissioning mission
  under gdb passed in 23.0 seconds with 0.278m final goal error
  (`artifacts/mission-depth10-debug.json`). A separate session hit the PX4
  mutex assertion; 10Hz remains an optional, experimental setting.
- After restarting the simulator's PX4 estimator during reset, a 10Hz PPO run
  passed multiple episode resets and collected 420 timesteps before PX4 time
  stalled. The checkpoint is incomplete; five completed episodes did not reach
  their goals and two ended on hard-clearance intervention. Evidence:
  `artifacts/ppo-512-ekf.monitor.csv` and `.metadata.json`.
- A subsequent 10Hz run under gdb completed 512 timesteps and five episodes
  in about 275 seconds. No completed episode reached its goal; one ended after
  a hard-clearance intervention. Evidence: `artifacts/ppo-512-debug.log`,
  `artifacts/ppo-512-debug.monitor.csv` and `.metadata.json`. A single completed
  run does not resolve the intermittent PX4 stall or prove policy quality.
- After an experimental PX4 lockstep scheduler patch, a 10Hz run collected
  856 of 1,024 requested timesteps and completed nine episodes, with no goals
  reached. The simulator's 600-second session limit stopped the run during a
  reset; its checkpoint is marked incomplete. No mutex assertion was observed
  in this session. This is not sufficient to establish that the patch fixes
  the intermittent crash. Evidence: `artifacts/ppo-1024-lockstep.metadata.json`
  and `.monitor.csv`.
- A second 10Hz training attempt stopped after 199 steps when reset did not
  meet the settled-pose gate. Its checkpoint is incomplete; two completed
  episodes reached no goals (`artifacts/ppo-1024-lockstep-retry.metadata.json`
  and `.monitor.csv`).
- Another patched PX4 session stopped after 229 steps with no fresh navigation
  observation. PX4 telemetry stopped while the Gazebo clock continued. A
  manually interrupted debugger captured the clock callback waiting for a
  mutex in `LockstepScheduler::set_absolute_time`; this shows the experimental
  patch has not resolved all scheduler stalls. Three completed episodes did
  not reach their goals (`artifacts/ppo-headless-1024.metadata.json`,
  `.monitor.csv`, and `artifacts/simulation/20261028-173940-102567/px4.log`).
- A live 5m obstacle-course mission was visually observed taking off. The
  controller reported obstacle deflection and recovery, then entered emergency
  hold and failsafe landing before reaching the goal. It landed 2.94m from the
  goal and is a failed mission (`artifacts/mission-obstacle-live.json`). The
  software-rendered Gazebo view was dark and slow with the GUI open.
- One PPO startup failed after loss of PX4 telemetry, and a separate session hit
  the existing native PX4 mutex assertion. The causes remain unconfirmed.

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

- [x] Review and commit the existing cleanup as a coherent baseline (PR #1).
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

- Hardware inventory supplied on 2026-10-26 and updated on 2026-10-28:
  - Pixhawk 2.4.8; installed PX4 version and board/flash identification pending.
  - Jetson Orin Nano Super 8GB developer kit; user reports JetPack 7.2.
  - Waveshare IMX477-160 12.3MP RGB camera with M12 lens.
  - Arducam IMX219 8MP RGB camera with motorized focus.
  - HC-SR04 ultrasonic range sensors; quantity and mounting remain to be chosen.
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
- The cameras will be used as monocular cameras. Camera mounting, calibration,
  autofocus behavior, and software metric-depth support remain unverified.
- HC-SR04 sensor count, orientation, coverage, interface and operating domain
  remain undecided. The simulation depth sectors do not yet model this stack.
- The intended Jetson supply is a 19V, 4A DC-DC regulator from the 4S battery;
  confirm step-up operation, wiring and performance under load. Manual override
  is planned through Mission Planner and MAVLink; compatibility with the actual
  firmware, telemetry link and loss-of-link response remain unverified.
- Storage, final vehicle mass, measured thrust/current, and payload endurance
  remain to be verified; the
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
