# AIC Competition Work Log

This file is the living project log for our Intrinsic AI for Industry Challenge work. It is meant to preserve context for future agents and for us, so we do not have to re-read the full repository every time.

## Current Status

- We completed a read-only repo reconnaissance pass before implementing anything.
- We identified the safest first implementation step: a compliant policy scaffold that proves lifecycle, task parsing, and observation plumbing.
- We added `aic_model/aic_model/TaskReporter.py`.
- We changed `docker/aic_model/Dockerfile` so the model image defaults to:

```text
policy:=aic_model.TaskReporter
```

- The user ran the scaffold locally with:

```bash
pixi run ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=aic_model.TaskReporter
```

- The log confirmed:
  - `TaskReporter` imports and loads.
  - Lifecycle transitions work: configure, activate, deactivate, cleanup, shutdown.
  - The configured-state goal rejection check works. The early `aic_model lifecycle is not in the active state` error is expected.
  - All three tasks were received and parsed.
  - Observations include camera image, joint state, TCP pose, and wrench data.
  - The scaffold returns `True` cleanly for each task.
- The user added the eval terminal log at `/home/optimus/ws_aic/src/terminal_1.log`.
- Review of that eval log confirmed:
  - All 3 trials processed.
  - Successful trials: 3.
  - Failed trials: 0.
  - Total score: 3.000.
  - Tier 1 passed for all trials.
  - Tier 2 was 0 for all trials.
  - Tier 3 was 0 for all trials.
  - No off-limit contacts were detected.
  - No excessive force penalty was applied.
  - No insertion was detected, as expected for a no-motion scaffold.

The current scaffold therefore proves lifecycle/action/observation plumbing but does not attempt the task.

## Important Competition Rules

Highest-priority rule: submitted policy must use only official interfaces.

Forbidden during evaluation:

- Directly manipulating robot/task-board/simulation state.
- Communicating with simulator/scoring/backend internals.
- Using `/scoring`, `/gazebo`, `/gz_server`, `/aic_engine`, `/ros_gz_bridge`, or similar backend namespaces.
- Spawning/deleting entities.
- Reading simulator world/model state, `/clock`, `/model`, `/world_stats`, physics pause/reset, or internal Gazebo state.
- Hardcoding trial-specific simulator state to exploit the public sample config.
- Depending on ground-truth object poses during evaluation.

Allowed/expected interfaces:

- Sensor inputs through official ROS topics and the `Observation` message.
- `/insert_cable` action.
- `/aic_controller/pose_commands`.
- `/aic_controller/joint_commands`.
- `/aic_controller/change_target_mode`.
- Standard lifecycle behavior for node `aic_model`.

Ground truth may be used for training/debugging, but not as an evaluation dependency.

## Repository Overview

Repo root:

```text
/home/optimus/ws_aic/src/aic
```

Main packages:

- `aic_model/`
  - Participant-facing Python lifecycle node wrapper.
  - Entry point: `aic_model/aic_model/aic_model.py`.
  - Base policy API: `aic_model/aic_model/policy.py`.
  - Our scaffold: `aic_model/aic_model/TaskReporter.py`.
  - Recommended place for simple policy work if we keep everything inside existing package.

- `aic_example_policies/`
  - Reference policies.
  - `WaveArm`: minimal dummy movement.
  - `CheatCode`: ground-truth policy for debugging only; not compliant for final eval.
  - `RunACT`: example learned policy integration.

- `aic_engine/`
  - Trial orchestrator.
  - Validates lifecycle behavior.
  - Spawns task board and cable.
  - Sends `/insert_cable` goals.
  - Starts/stops scoring recordings.
  - Writes scoring output to `$AIC_RESULTS_DIR/scoring.yaml` or `~/aic_results/scoring.yaml`.

- `aic_adapter/`
  - Synchronizes sensor streams into `aic_model_interfaces/msg/Observation`.
  - Observation includes:
    - left/center/right RGB camera image and camera info
    - wrist wrench
    - joint states
    - controller state

- `aic_controller/`
  - Low-level impedance controller.
  - Accepts Cartesian commands via `/aic_controller/pose_commands`.
  - Accepts joint commands via `/aic_controller/joint_commands`.
  - Mode is selected through `/aic_controller/change_target_mode`.

- `aic_scoring/`
  - Implements scoring.
  - Tier 1: model validity.
  - Tier 2: jerk, duration, path efficiency, force penalty, off-limit contact penalty.
  - Tier 3: insertion success, partial insertion, proximity.

- `aic_interfaces/`
  - ROS message/action/service definitions.
  - Important files:
    - `aic_task_interfaces/action/InsertCable.action`
    - `aic_task_interfaces/msg/Task.msg`
    - `aic_model_interfaces/msg/Observation.msg`
    - `aic_control_interfaces/msg/MotionUpdate.msg`
    - `aic_control_interfaces/msg/JointMotionUpdate.msg`

- `aic_bringup/`
  - Launch files and sim config.
  - Main launch: `aic_bringup/launch/aic_gz_bringup.launch.py`.

- `aic_assets/`
  - Gazebo models/assets for cables, SFP, SC, NIC cards, task board, cameras, gripper, etc.

- `docker/`
  - `docker/aic_model/Dockerfile`: participant model image.
  - `docker/aic_eval/Dockerfile`: evaluation image.
  - `docker/docker-compose.yaml`: local two-container eval/model setup.

## Task Understanding

Qualification phase:

- Single cable insertion per trial.
- Same submitted policy is used for all trials.
- Evaluation occurs in Gazebo.
- Robot starts holding one plug.
- Plug starts close to the target.
- Board pose and components are randomized.
- Exact final trial sequence may change, but tasks are combinations of SFP and SC insertion.

Trial types:

- SFP trials:
  - Robot holds `SFP_MODULE`.
  - Insert into `SFP_PORT_0` or `SFP_PORT_1` on a target NIC card.
  - There may be multiple NIC cards and therefore multiple SFP ports.
  - Target is specified by `Task.target_module_name` and `Task.port_name`.

- SC trials:
  - Robot holds `SC_PLUG`.
  - Insert into target SC port.
  - One or both SC ports may be present.
  - Target is specified by `Task.target_module_name` and `Task.port_name`.

Observed local sample tasks from scaffold run:

```text
trial 1: cable_0, plug=sfp_tip, target=nic_card_mount_0/sfp_port_0
trial 2: cable_0, plug=sfp_tip, target=nic_card_mount_1/sfp_port_0
trial 3: cable_1, plug=sc_tip, target=sc_port_1/sc_port_base
```

## Scoring Notes

Current scoring source and `docs/scoring.md` agree on:

- Max per trial: 100.
- Max qualification total: 300 over 3 trials.
- Tier 1:
  - 1 point for valid lifecycle/action behavior.
- Tier 2:
  - smoothness: 0 to 6
  - duration: 0 to 12
  - efficiency: 0 to 6
  - force penalty: 0 to -12
  - off-limit contact penalty: 0 to -24
- Tier 3:
  - correct insertion: 75
  - wrong-port insertion: -12
  - partial insertion: 38 to 50
  - proximity: 0 to 25

Important scoring behavior:

- Tier 2 positive bonuses are only awarded if Tier 3 score is greater than 0.
- Task duration ends when the action result returns.
- Returning `True` without moving is fine for scaffold validation, but will not score insertion.
- Full insertion dominates scoring; optimize reliability before speed.

Known documentation discrepancy:

- `docs/scoring_tests.md` appears to mention older score values, such as Tier 3 max 60.
- Trust `docs/scoring.md` plus `aic_scoring/src/ScoringTier2.cc` unless later official docs say otherwise.

## Decisions Made

### Decision: Use existing `aic_model` wrapper

Why:

- It already satisfies most lifecycle/action boilerplate.
- It rejects goals when inactive/configured.
- It handles command publishing through lifecycle publishers.
- It reduces compliance risk compared with writing a custom lifecycle node.

### Decision: First implementation is read-only `TaskReporter`

Why:

- Tier 1/lifecycle compliance is a gate.
- We need to prove the task message and observations are available before perception/control.
- It avoids premature movement, force penalties, and simulator contact risk.

### Decision: Avoid `CheatCode` for default Docker policy

Why:

- `CheatCode` depends on ground-truth TF and is not a compliant evaluation policy.
- It is useful only for training/debugging.

### Decision: Next technical focus should be perception/localization

Why:

- Multiple SFP/SC ports may be present.
- Hidden evaluation randomizes board and component placement.
- We need target localization from allowed observations before insertion control.

## Commands To Run

Use these from repo root:

```bash
cd /home/optimus/ws_aic/src/aic
```

### Refresh local Pixi package after source edits

Pixi does not automatically see changes in local packages.

```bash
pixi reinstall ros-kilted-aic-model
```

If this fails due cache permissions, fix Pixi/cache permissions or run with appropriate local user setup. Avoid changing code just to work around package installation unless necessary.

### Run evaluation environment

Terminal 1:

```bash
export DBX_CONTAINER_MANAGER=docker
distrobox enter -r aic_eval -- /entrypoint.sh ground_truth:=false start_aic_engine:=true
```

Wait for engine logs saying it is looking for `aic_model`.

### Run scaffold policy

Terminal 2:

```bash
pixi run ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=aic_model.TaskReporter
```

Expected model logs:

- `Loading policy module: aic_model.TaskReporter`
- `Using policy: TaskReporter`
- `on_configure`
- `TaskReporter.__init__()`
- early inactive-state rejection error during engine test
- `on_activate`
- `Goal accepted`
- `TaskReporter.insert_cable() enter`
- `Task received: ...`
- `Observation received: center_image=1152x1024 ...`
- `TaskReporter observed task and sensor inputs successfully.`
- `insert_cable() returned True`
- repeats for all trials
- `on_cleanup`
- `on_shutdown`

Expected scoring:

- Tier 1 should pass.
- Tier 3 should be 0 or near 0 because no insertion is attempted.
- Total score should be low. That is expected at this stage.

Observed scaffold scoring from `/home/optimus/ws_aic/src/terminal_1.log`:

```text
total: 3
trial_1: tier_1=1, tier_2=0, tier_3=0, final plug-port distance=0.13m
trial_2: tier_1=1, tier_2=0, tier_3=0, final plug-port distance=0.14m
trial_3: tier_1=1, tier_2=0, tier_3=0, final plug-port distance=0.32m
```

This is the expected result for `TaskReporter`.

## Current Issues And Watch Items

### Pixi reinstall needed after source changes

Symptom:

```text
ModuleNotFoundError: No module named 'aic_model.TaskReporter'
```

Cause:

- The source file exists, but Pixi is using the installed copy under `.pixi/envs/default`.

Fix:

```bash
pixi reinstall ros-kilted-aic-model
```

### Wrench z value around 20N in scaffold logs

Observation:

- Scaffold showed wrist force z around 20.5N while no motion commands were sent.

Interpretation:

- Likely attached cable/tool/load/tare behavior, not policy-induced insertion force.

Follow-up:

- Watch force carefully once we begin motion.
- Use compliant, low-stiffness/slow final insertion to avoid sustained force penalty.

### Need terminal 1 scoring confirmation

Resolved. Eval terminal log was added at `/home/optimus/ws_aic/src/terminal_1.log` and reviewed.

- Model validation/Tier 1 passed in all 3 trials.
- Trial processing completed cleanly.
- No insertion was detected, which is expected.
- Total score was 3.000.

### Shutdown noise after Ctrl-C

After the engine completed cleanly, the user interrupted the eval environment with Ctrl-C. The log then showed Gazebo/container shutdown errors such as:

```text
double free or corruption (!prev)
process failed to terminate ... escalating to SIGTERM/SIGKILL
```

Interpretation:

- These happened after `All Trials Processed` and after `aic_engine` finished cleanly.
- Treat them as Gazebo/eval shutdown noise, not a policy failure.

### ACL watch item

The eval log contains Zenoh access-control messages like:

```text
did not match any configured ACL subject. Default permission `Allow` will be applied
```

Interpretation:

- The scaffold itself did not touch forbidden interfaces, so this is not currently a policy concern.
- Before final confidence, run a stricter ACL-enabled local check so we know the policy works under portal-like model access controls.

## Next Implementation Plan

1. Keep `TaskReporter` as a diagnostic scaffold.
2. Add a perception debug policy or extend scaffold to save/log non-forbidden visual observations carefully.
3. Determine target localization strategy:
   - likely start with classical vision on wrist camera images
   - use task message to select expected target type
   - avoid ground-truth object TF in eval path
4. Add guarded motion in stages:
   - move to a visually estimated pre-insertion pose
   - align yaw/roll/pitch
   - slow compliant approach
   - monitor wrench and controller state
   - return action result after insertion/stabilization
5. Test SFP and SC separately.
6. Only after reliable insertion, optimize speed/path/smoothness.

## Files Changed So Far

- `aic_model/aic_model/TaskReporter.py`
  - New read-only scaffold policy.

- `docker/aic_model/Dockerfile`
  - Changed default policy from `aic_example_policies.ros.CheatCode` to `aic_model.TaskReporter`.
