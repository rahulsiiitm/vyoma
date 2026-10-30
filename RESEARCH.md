# Research notes: simulation navigation

## Findings applied to Vyoma

1. Stable-Baselines3 recommends normalized observations and actions, shaped
   rewards, a simpler starting task, and separate evaluation episodes. Vyoma's
   429-value observation already scales the input features, and actions use
   `[-1, 1]`. The `easy` curriculum now samples short goals; `standard` retains
   obstacle scenarios. Reward version 2 distinguishes routine acceleration
   limiting from emergency intervention. These changes are hypotheses to test,
   not proof of improved performance.
2. PPO is on-policy. SAC is an off-policy continuous-control algorithm that
   reuses transitions through a replay buffer. The published SAC work and SB3
   guidance motivate testing SAC when simulator samples are expensive. This
   does not guarantee that SAC will outperform PPO in this project. SAC models
   and their replay buffers are saved separately, and the training entry point
   can resume either algorithm.
3. Research on vision-based agile flight reports that combining imitation and
   RL can improve sample efficiency on its evaluated racing task. A
   commissioning-controller imitation approach is a possible later experiment;
   it is not currently implemented or evaluated in Vyoma.

## Evaluation protocol

- Compare algorithms with the same world, reward version, simulator settings,
  training step budget and evaluation goal list. Run multiple training seeds;
  one seed cannot establish superiority.
- Use deterministic actions in a fresh evaluation session. The batch evaluator's
  easy goals test basic navigation only, not obstacle avoidance. Test obstacle
  goals separately and report every attempted run, including failures.
- Report completed missions, accepted missions, minimum reported clearance,
  final goal distance, time and interventions. Acceptance requires takeoff,
  goal hold, landing and disarm, and recorded minimum navigation clearance at
  or above the configured hard threshold. Missing clearance does not pass.
- The mission manager now requires a sustained sequence of fresh clearance
  readings before leaving an emergency hold. Persistent danger still leads to
  failsafe landing. This change responds to observed hold/replan cycling and
  requires new evaluations; results recorded before it use a different
  controller configuration and must not be compared as if it were unchanged.
- A scan without finite in-range measurements is treated as unknown and causes
  a safety stop. This is deliberately conservative: a depth camera returning
  only no-returns cannot by itself prove an unobstructed path.
- The simulated depth sensor is a simulation input. The intended physical
  monocular cameras are not equivalent depth sensors; transferring the policy
  would need a separately validated depth-estimation or perception interface.
- Safety filtering now measures the action cone and can select a bounded,
  sensor-cleared forward escape direction. Unit tests cover an oblique escape,
  front-obstacle steering and the all-directions-blocked stop. In SITL, the
  5m route completes but the best measured reported clearance among the recent
  default-rate trials was 0.425m, below the 0.75m acceptance threshold. The
  action-cone method is therefore an experimental improvement, not an accepted
  avoidance algorithm.

## Results so far (2026-10-30)

- Matched five-case easy-curriculum evaluation: PPO accepted 2/5 and SAC 0/5.
  This favors PPO only as the current short-goal candidate; it is not a
  statistically meaningful comparison and does not test obstacle avoidance.
- The 512-step easy PPO checkpoint completed a 5m mission, reached the goal,
  landed and disarmed. It did not pass the clearance gate: 0.425m measured
  versus 0.75m required. Keep this result separate from a safety pass.
- The next useful learning experiment is continued PPO training on the standard
  obstacle curriculum, followed by an unchanged held-out 5m test and multiple
  seeded obstacle missions. Do not infer improvement from training return
  alone.

## Sources

- [Stable-Baselines3 2.4.1: RL tips and evaluation](https://stable-baselines3.readthedocs.io/en/v2.4.1/guide/rl_tips.html)
- [Haarnoja et al., Soft Actor-Critic](https://proceedings.mlr.press/v80/haarnoja18b.html)
- [Bootstrapping Reinforcement Learning with Imitation for Vision-Based Agile Flight](https://arxiv.org/abs/2403.12203)
- [Curriculum Reinforcement Learning for Quadrotor Racing with Random Obstacles](https://arxiv.org/abs/2602.24030)
