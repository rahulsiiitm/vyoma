# Vyoma Hybrid Autonomy Implementation Checklist

This checklist is ordered by dependency. A phase is complete only when its
acceptance checks pass; code presence alone does not count as completion.

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

- [ ] Build the Jetson ARM64 runtime image for the installed JetPack version.
- [ ] Match PX4, `px4_msgs`, ROS 2, and XRCE-DDS versions.
- [ ] Configure and validate the Pixhawk-Jetson communication link.
- [ ] Calibrate sensors and measure real latency, noise, and dropout rates.
- [ ] Complete HIL, shadow-flight, restrained-flight, and bounded-flight gates.
