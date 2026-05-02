---
name: AIC My Policy Package - Full Technical Map
description: Complete breakdown of aic_my_policy: architecture, pipeline stages, file roles, config knobs, and run commands. Covers base policy, data collection, vision training, and RL env.
type: project
---

The `aic_my_policy` package (at `~/ws_aic/src/aic/aic_my_policy/`) is the team's submission policy for the AIC cable insertion challenge. It implements a 5-state impedance controller with a pluggable pose estimator and an optional residual RL layer.

**Why:** The policy must score above the CheatCode baseline (~60 Tier 3 pts/trial) while avoiding the 20 N force penalty. Impedance compliance achieves this without requiring RL.

**How to apply:** When suggesting code changes, always route pose estimation through the `PortPoseEstimator` ABC — never call TF directly from the state machine. All tuning numbers live in `control/impedance.py` and `ros/InsertCablePolicy.py` (timing budgets).

## Artifact completion status
- #1 Package skeleton — done
- #2 Impedance state machine (GT TF) — done
- #3 Gazebo data collection — done
- #4 Keypoint CNN + PnP estimator — done
- #5 Vision estimator wired into policy — done
- #6 Gym RL env wrapper — done
- #7 SAC residual RL — NEXT (not started)
- #8 Wire residual on top of base policy — pending
- #9 Dockerfile + ECR push — pending

## Key file roles
- `ros/InsertCablePolicy.py` — 20 Hz state machine, ROS param `estimator` selects GT vs vision
- `control/impedance.py` — per-state stiffness/damping/feedforward profiles (APPROACH, ALIGN, SEARCH, SEAT)
- `control/geometry.py` — quaternion alignment, spiral sweep, gripper back-computation
- `estimators/base.py` — `PortPoseEstimator` ABC
- `estimators/ground_truth.py` — /tf-backed, needs `ground_truth:=true`
- `estimators/vision.py` — keypoint+PnP, needs trained .pt weights
- `perception/keypoints.py` — hard-coded 3D port geometry (SFP: 13.7x8.5mm, SC: 2.5mm bore) — MUST verify against USD/SDF assets
- `perception/model.py` — ResNet-18 + deconv heatmap head, 4 keypoints, INPUT 256x256
- `perception/dataset.py` — .npz loader, projects 3D->2D, Gaussian heatmap targets
- `perception/train.py` — supervised CLI, MSE on heatmaps, saves best-val checkpoint
- `perception/infer.py` — PortKeypointInference class, calls solvePnP
- `data_collection/collect_dataset.sh` — outer loop: randomize -> launch -> settle -> capture
- `data_collection/randomize.py` — SceneConfig dataclass, challenge-range randomization
- `data_collection/capture_scene.py` — one-shot ROS node, writes .npz
- `rl/env.py` — ResidualInsertionEnv, 12-dim action, 33-dim actor obs, 47-dim critic obs

## Critical warnings
- `ground_truth:=true` REQUIRED for GT estimator; eval Zenoh ACL BLOCKS these TF frames
- keypoints.py has "TODO verify against asset" markers — must be fixed before trusting vision estimator
- Data collection must NOT run while aic_model/aic_engine are running
- `pixi reinstall ros-kilted-aic-my-policy` required after every Python edit
- Vision estimator still reads plug TF from /tf (development shortcut); full eval needs calibration step
- Start Terminal 0 (Zenoh) BEFORE Terminals 1 and 2; Terminal 2 must appear within 30s of Terminal 1
