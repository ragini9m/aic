# aic_my_policy

Our team's policy package for the AI for Industry Challenge (AIC).

This package will hold:

1. A **vision-based impedance state-machine** base policy — approach, align, search, seat, verify — driven by a learned port-pose estimator, using the existing `aic_controller` impedance primitives (stiffness/damping matrices + feedforward wrench).
2. A **residual RL policy** (SAC) trained in Gazebo with an asymmetric critic (actor sees estimator output; critic sees ground-truth `/tf` during training).

The package currently contains a **stub policy** so we can verify the build + load path end-to-end before writing the real logic.

---

## Status

| Artifact | State | Doc |
|---|---|---|
| 1. Package skeleton | ✅ done | (this README) |
| 2. Impedance state-machine base policy (GT-TF backed) | ✅ done | [02](./docs/02_impedance_state_machine.md) |
| 3. Gazebo synthetic-data collection script | ✅ done | [03](./docs/03_data_collection.md) |
| 4. Keypoint CNN + PnP port-pose estimator | ✅ done | [04](./docs/04_vision_estimator.md) |
| 5. Wire vision estimator into policy (ROS-param selectable) | ✅ done | [05](./docs/05_wire_vision.md) |
| 6. Gym-style Gazebo env wrapper | ✅ done | [06](./docs/06_rl_env.md) |
| 7. SAC residual RL (asymmetric critic) | ⏳ next | — |
| 8. Wire residual on top of base policy | ⏳ | — |
| 9. Dockerfile + ECR push | ⏳ | — |

---

> **Deep dive on artifact #2:** [`docs/02_impedance_state_machine.md`](./docs/02_impedance_state_machine.md)

## What it does (artifact #2)

`InsertCablePolicy` (`aic_my_policy/ros/InsertCablePolicy.py`) runs a 20 Hz state machine driving `aic_controller` via per-state impedance profiles:

| State | Goal | Stiffness (x,y,z,rx,ry,rz) | Feedforward force (N) |
|---|---|---|---|
| **APPROACH** | Park plug tip 8 cm above port, slerp into port orientation | 90,90,90, 50,50,50 | 0 |
| **ALIGN** | Descend to 2 cm above, fully oriented | 90,90,90, 50,50,50 | 0 |
| **SEARCH** | Descend while spiraling (up to 3 mm radius), lateral compliance | **30,30,90**, 50,50,50 | (0, 0, **-3**) |
| **SEAT** | Push down through the hole | **20,20,90**, 50,50,50 | (0, 0, **-8**) |
| **VERIFY** | Hold, return success | — | — |

The policy uses a pluggable `PortPoseEstimator`; the current implementation is `GroundTruthPortPoseEstimator` which looks up `/tf` frames, so the launch must use `ground_truth:=true`. The vision-based estimator (artifacts #3–4) will drop in behind the same interface without policy changes.

**Expected score vs CheatCode baseline** (which scores ~60 Tier 3 per trial per `docs/scoring_tests.md`):

- Tier 1: **pass** on all 3 trials.
- Tier 3: close to CheatCode; possibly higher on smoothness thanks to impedance compliance, possibly lower if stiffness tuning is off.
- Tier 2 force penalty: should stay clear of −12 (feedforward force capped at 8 N, well under 20 N threshold).

If numbers come back bad, the first place to tune is `aic_my_policy/control/impedance.py` — profiles are isolated there.

---

## Prerequisites

Host machine must be **Ubuntu 24.04 with an NVIDIA GPU** and the AIC toolkit set up per [docs/getting_started.md](../docs/getting_started.md):

- Docker
- Distrobox (`export DBX_CONTAINER_MANAGER=docker`)
- Pixi
- NVIDIA Container Toolkit
- `aic_eval` container pulled (`ghcr.io/intrinsic-dev/aic/aic_eval:latest`)
- Repo cloned at `~/ws_aic/src/aic` with `pixi install` completed

---

## Run it

Open three terminals. All commands are executed on the GPU host.

### Terminal 0 — Zenoh router

```bash
cd ~/ws_aic/src/aic
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd
```

### Terminal 1 — Evaluation environment

```bash
cd ~/ws_aic/src/aic
pixi install                      # picks up aic_my_policy on first run
distrobox enter -r aic_eval -- /entrypoint.sh ground_truth:=true start_aic_engine:=true
```

> `ground_truth:=true` is only for bring-up — it makes later debugging easier. Final submissions must work with `ground_truth:=false`.

### Terminal 2 — Run the stub policy

```bash
cd ~/ws_aic/src/aic
pixi reinstall ros-kilted-aic-my-policy
pixi run ros2 run aic_model aic_model --ros-args \
    -p use_sim_time:=true \
    -p policy:=aic_my_policy.ros.InsertCablePolicy
```

---

## Expected output

In **Terminal 2** (`aic_model`):

```
Loading policy module: aic_my_policy.ros.InsertCablePolicy
Loaded policy module aic_my_policy.ros.InsertCablePolicy
Using policy: InsertCablePolicy
InsertCablePolicy.__init__() (skeleton stub)
...
InsertCablePolicy.insert_cable() task=...
Holding pose p=(x, y, z)
InsertCablePolicy.insert_cable() returning True
```

In **Terminal 1** (eval env / engine):

- Three trials run sequentially (SFP rail 0 → SFP rail 1 → SC).
- Tier 1 should **pass** on every trial.
- Tier 2 and Tier 3 should be **0** (the arm just holds position).
- Results written to `~/aic_results/` (or `$AIC_RESULTS_DIR` if set).

---

## Develop loop

Any time you edit a `.py` file under `aic_my_policy/`:

```bash
cd ~/ws_aic/src/aic
pixi reinstall ros-kilted-aic-my-policy
```

Then restart **Terminal 2** only (Terminals 0 and 1 can stay running).

---

## Layout

```
aic_my_policy/
├── README.md                        # this file
├── package.xml
├── setup.py / setup.cfg
├── pixi.toml
├── resource/aic_my_policy
├── docs/                            # per-artifact deep dives
└── aic_my_policy/
    ├── estimators/
    │   ├── base.py                  # PortPoseEstimator ABC
    │   ├── ground_truth.py          # /tf-backed impl (dev)
    │   └── vision.py                # keypoint+PnP impl (submission)
    ├── control/
    │   ├── impedance.py             # per-state stiffness/damping/ff
    │   └── geometry.py              # port/plug alignment + spiral
    ├── perception/
    │   ├── keypoints.py             # 3D port keypoints (local frame)
    │   ├── model.py                 # ResNet-18 heatmap net
    │   ├── dataset.py               # HDF5-free .npz dataset loader
    │   ├── train.py                 # supervised training CLI
    │   └── infer.py                 # image -> port pose (cam frame)
    ├── data_collection/
    │   ├── randomize.py             # random scene-config generator
    │   ├── capture_scene.py         # ROS node, one-shot capture
    │   └── collect_dataset.sh       # outer launch/capture loop
    ├── rl/
    │   └── env.py                   # Gym-style residual-RL wrapper
    └── ros/
        └── InsertCablePolicy.py     # state machine; estimator selected via param
```

The policy is loaded by `aic_model` via Python import — the ROS param `policy:=aic_my_policy.ros.InsertCablePolicy` means "import module `aic_my_policy.ros.InsertCablePolicy`, instantiate the class named `InsertCablePolicy`."

---

## Troubleshooting

**`Unable to load policy aic_my_policy.ros.InsertCablePolicy`**
You likely haven't installed the new package yet, or haven't reinstalled after editing. Run:
```bash
cd ~/ws_aic/src/aic
pixi install
pixi reinstall ros-kilted-aic-my-policy
```

**`Class InsertCablePolicy not in module aic_my_policy.ros.InsertCablePolicy`**
The loader expects the **class name to match the last dotted segment** of the module path. If you rename the file, rename the class to match (and vice versa), then `pixi reinstall`.

**`No observation received within 10s`**
The `aic_adapter` isn't publishing on `/observations`. Confirm the eval env came up cleanly in Terminal 1 and that both Gazebo and RViz opened. Check `ros2 topic hz /observations`.

**`Zenoh router not reachable`**
Terminal 0 must be running *before* Terminal 1 and Terminal 2. Restart in order.

**Node lifecycle warnings / `aic_engine` times out**
`aic_engine` waits 30 s for `aic_model` to appear. Start Terminal 2 within 30 s of Terminal 1.

For more general issues, see [docs/troubleshooting.md](../docs/troubleshooting.md).

---

## Tuning cheat sheet

All profile numbers live in `aic_my_policy/control/impedance.py`. Common knobs:

| Symptom | Fix |
|---|---|
| Plug slams into port entrance, force penalty | Reduce `SEARCH.feedforward_force.z` magnitude; lower `SEAT.stiffness_diag` z |
| Plug bounces off lip, never enters | Increase `SEARCH.feedforward_force.z` magnitude, increase `spiral_xy_offset.radius_mm` |
| Plug tilts during insertion | Raise angular stiffness (last 3 entries of `SEARCH`/`SEAT` stiffness) |
| Motion is jerky | Lower `LOOP_HZ` in `InsertCablePolicy.py` (set `1/LOOP_DT` lower), increase damping proportionally |
| Enters SEARCH before fully aligned | Raise `ALIGN_BUDGET_S` in `InsertCablePolicy.py` |

State timing knobs (`InsertCablePolicy.py`): `APPROACH_BUDGET_S`, `ALIGN_BUDGET_S`, `SEARCH_BUDGET_S`, `SEAT_BUDGET_S`, `VERIFY_HOLD_S` — all sum well under the 180 s trial limit.

## Next artifact

Artifact #3: **Gazebo synthetic-data collection script** — spawn randomized task board configurations with `ground_truth:=true`, subscribe to the three wrist cameras + `/tf`, dump `(image, CameraInfo, port_pose_in_base_link)` tuples. This dataset feeds artifact #4 (keypoint CNN + PnP) which replaces `GroundTruthPortPoseEstimator`.
