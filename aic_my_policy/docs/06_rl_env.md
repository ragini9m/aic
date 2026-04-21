# Artifact #6 — Gym-Style Gazebo Env Wrapper

Prepares the RL training ground for residual SAC (artifact #7). The env wraps the live Gazebo/ROS stack, runs at 20 Hz, exposes an asymmetric observation (actor sees only eval-legal info; critic sees privileged GT), and applies residual actions on top of the artifact #2 base policy's commands.

This artifact delivers the env only — no learning algorithm yet. Artifact #7 adds SAC against this env.

---

## Files

```
aic_my_policy/rl/
└── env.py    # ResidualInsertionEnv (Node-based gym-like wrapper)
```

---

## Action space (12-dim, bounded [-1, 1])

| Indices | Meaning | Scale |
|---|---|---|
| 0–2 | Δx, Δy, Δz translation residual | ±5 mm |
| 3–5 | Δrx, Δry, Δrz axis-angle residual | ±0.04 rad (~2.3°) |
| 6–11 | Δstiffness log-factor per axis | 2^[-1,1] = ×0.5 to ×2 |

The final command is produced by `_apply_residual(base_cmd, a)`:

- position = base + clipped Δpose
- orientation = base (residual currently zero for rotations; TODO: axis-angle composition)
- stiffness_diag = base_diag × 2^action[6:12]
- damping_diag = base_damping × √(stiffness factor) (critical-damping ratio preserved)

## Observation space

**Actor (33-dim):**
| Slice | Content |
|---|---|
| 0–6 | estimated port pose (xyz + quat) |
| 7–13 | estimated plug tip pose (xyz + quat) |
| 14–20 | TCP pose from controller_state |
| 21–26 | wrist wrench (fx, fy, fz, tx, ty, tz) |
| 27–32 | tcp_error |

**Critic (47-dim = actor obs + 14):** the extra 14 slots are reserved for the training loop to fill with **ground-truth** port + plug poses from `/tf`. Kept outside the env so the env itself is source-of-truth-agnostic.

## Reward

Shaped, aligned with the scoring tiers:

```
lateral  = ||plug_xy - port_xy||
axial    = port_z - plug_z           # +ve when plug is below port entrance (inserted)
reward   = -5.0 * lateral - 1.0 * max(0, -axial)
if force_mag > 20: reward -= 0.5
if lateral < 2mm and axial > 12mm: reward += 50; terminated = True
```

This mirrors: proximity matters (linear in lateral xy), insertion depth matters (linear in axial), sustained force is penalized, and success gets a large sparse bonus. Tune coefficients once we see rollouts.

## Reset strategy

`reset(task)`:

1. `estimator.initialize(task)` (GT for bring-up; vision for sim-to-real parity).
2. Home the robot via a canonical joint-space command.
3. Wait 2 s for the robot to settle.
4. Return (actor_obs, critic_obs).

Cleanly re-randomizing the task board between episodes without restarting the launch is non-trivial. A production reset should call gazebo create/delete entity services (available during training because the Zenoh ACL is off). That hardening is a TODO; for now we relaunch Gazebo between episodes if we want pose randomization.

## Step loop

```python
env = ResidualInsertionEnv(estimator)
actor_obs, critic_obs = env.reset(task)

for _ in range(env.MAX_STEPS):
    base_cmd = base_policy.compute_base_command(...)   # from artifact #2
    a = policy(actor_obs)                              # SAC actor, 12-dim in [-1,1]
    result = env.step(a, base_cmd)
    if result.terminated or result.truncated:
        break
    actor_obs, critic_obs = result.obs_actor, result.obs_critic
```

In artifact #7 we'll refactor `InsertCablePolicy` to expose `compute_base_command(state, observation)` as a pure function so it can be called from both the live ROS lifecycle and this env.

## Known limitations

- **No scene randomization on reset.** First version homes the robot but does not respawn the task board. Use the collect-dataset orchestrator for randomization variety during training.
- **Orientation residual is position-only.** Rotation Δ is accepted in the action but not yet applied (needs proper axis-angle-to-quat composition). Easy follow-up.
- **Single env.** Gazebo can't run thousands of parallel envs; SAC will be sample-limited. We'll warm-start from the base policy so the residual only has to learn millimeters.
- **Reward coefficients are guesses.** Tune after first successful training run, when we can see if lateral-vs-axial weighting is reasonable.

## Running a smoke test (before SAC exists)

```python
import rclpy
from aic_my_policy.rl.env import ResidualInsertionEnv
from aic_my_policy.estimators.ground_truth import GroundTruthPortPoseEstimator
# (This assumes you've set up a stub Node with _tf_buffer, or call inside aic_model.)
```

A proper test harness is part of artifact #7.

## Next artifact

**Artifact #7** — SAC (off-policy) with asymmetric critic trained against this env, warm-started from the base policy. Actor → actor_obs → 12-dim residual; critic → critic_obs → Q(s, a). Replay buffer, target network, entropy regularization — standard SAC stack.
