# Artifact #5 — Wire Vision Estimator into the Policy

Drop-in replacement of the `GroundTruthPortPoseEstimator` with a `VisionPortPoseEstimator`, selected at runtime via a ROS parameter. Zero changes to the state-machine logic.

---

## What changed

- `aic_my_policy/estimators/vision.py` — new `VisionPortPoseEstimator`:
  - On `initialize()`, picks the right keypoint model (SFP vs SC) based on the incoming `Task.port_type`, waits for the plug TF frame.
  - On `get_port_pose()`, reads the center-camera image + `CameraInfo` from the current `Observation`, runs keypoint+PnP, multiplies by the camera-to-base static TF.
  - On `get_plug_tip_pose()`, still uses TF — the grasped plug is rigidly attached at a known offset. This is a development-mode shortcut; at real eval the plug offset must come from a calibration step at task start (tracked as a TODO).

- `aic_my_policy/ros/InsertCablePolicy.py` — constructor now reads ROS params:

```
estimator            : 'ground_truth' (default) | 'vision'
sfp_keypoint_weights : path to SFP keypoint .pt checkpoint
sc_keypoint_weights  : path to SC  keypoint .pt checkpoint
```

The estimator class is chosen at `__init__`; no behavioral change while `estimator=ground_truth`.

Additional vision guard/debug parameters:

```
vision_min_keypoint_confidence : minimum per-keypoint heatmap confidence, default 0.05
vision_max_reprojection_error_px : maximum mean PnP reprojection error, default 25.0
vision_max_stale_age_s : maximum age of reused port estimate, default 2.0
vision_debug_gt : log vision-vs-GT error when GT TF is available, default false
vision_gt_error_gate : reject vision estimates whose GT error is too high, default false
vision_max_gt_error_m : GT gate threshold, default 0.02
vision_use_configured_plug_offsets : use configured TCP-frame plug offsets if TF is unavailable, default false
vision_allow_tcp_plug_fallback : use TCP as plug tip when no offset/TF is available, default true
vision_latch_port_pose : freeze the initial accepted port pose for the whole trial, default true
vision_latch_min_samples : accepted samples before latching, default 5
vision_use_fixed_port_z : replace estimated port z with per-port-type base-frame constants, default true
sfp_fixed_port_z_base : base-frame SFP port entrance z, default 0.1335
sc_fixed_port_z_base : base-frame SC port entrance z, default 0.0145
sfp_plug_tip_offset_tcp_xyz : TCP-frame SFP plug-tip offset, default [0, 0, 0]
sc_plug_tip_offset_tcp_xyz : TCP-frame SC plug-tip offset, default [0, 0, 0]
```

`vision_debug_gt` and `vision_gt_error_gate` are development-only. Keep them off for submission.

---

## Running with vision

```bash
# Terminal 0 — Zenoh router
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd

# Terminal 1 — Dev eval env with GT available for measuring vision error.
distrobox enter -r aic_eval -- /entrypoint.sh ground_truth:=true start_aic_engine:=true

# Terminal 2 — Policy with vision estimator
pixi reinstall ros-kilted-aic-my-policy
pixi run ros2 run aic_model aic_model --ros-args \
    -p use_sim_time:=true \
    -p policy:=aic_my_policy.ros.InsertCablePolicy \
    -p estimator:=vision \
    -p vision_debug_gt:=true \
    -p vision_gt_error_gate:=true \
    -p sfp_keypoint_weights:=/home/$USER/aic_data/models/sfp_keypoints.pt \
    -p sc_keypoint_weights:=/home/$USER/aic_data/models/sc_keypoints.pt
```

> The plug pose can still be calibrated from `/tf` during development. With `ground_truth:=false`, the estimator uses configured TCP-frame plug-tip offsets if provided; otherwise it falls back to treating the TCP as the plug tip so the policy can still execute.

---

## Expected first-run behavior

- `aic_model` log: `InsertCablePolicy ready (VisionPortPoseEstimator).`
- `[vision estimator] port_type=sfp plug_frame=sfp_sc_cable/sfp_module_link`
- The state machine executes as before; if Tier 3 regresses vs the GT baseline, the keypoint model is the likely culprit — run the validation protocol in `docs/04_vision_estimator.md`.

---

## Regression checks

Keep two submissions on the leaderboard for comparison:

| Version | estimator | Expected |
|---|---|---|
| v1 (GT) | ground_truth | ~60–75 Tier 3 (ceiling) |
| v2 (vision) | vision | Within 5–10 Tier 3 pts of v1 when training data quality is good |

If v2 is much worse, the bottleneck is almost always: (a) wrong 3D keypoint coords in `keypoints.py`, or (b) dataset too small.

---

## Next artifact

**Artifact #6** (`docs/06_rl_env.md`) adds the Gym-style Gazebo env wrapper for residual RL — same estimator, same base policy, plus a learnable Δ-action on top.
