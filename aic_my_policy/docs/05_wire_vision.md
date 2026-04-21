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

---

## Running with vision

```bash
# Terminal 0 — Zenoh router
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd

# Terminal 1 — Eval env. IMPORTANT: ground_truth:=false simulates the real eval.
distrobox enter -r aic_eval -- /entrypoint.sh ground_truth:=false start_aic_engine:=true

# Terminal 2 — Policy with vision estimator
pixi reinstall ros-kilted-aic-my-policy
pixi run ros2 run aic_model aic_model --ros-args \
    -p use_sim_time:=true \
    -p policy:=aic_my_policy.ros.InsertCablePolicy \
    -p estimator:=vision \
    -p sfp_keypoint_weights:=/home/$USER/aic_data/models/sfp_keypoints.pt \
    -p sc_keypoint_weights:=/home/$USER/aic_data/models/sc_keypoints.pt
```

> The plug pose is still read from `/tf`. You'll want `ground_truth:=true` for that in this intermediate milestone. A fully eval-ready version (no TF at all) requires an added calibration step at task start; see the TODO in `estimators/vision.py`.

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
