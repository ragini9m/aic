# Artifact #3 — Gazebo Synthetic Data Collection

Generates a labeled dataset of `(image, camera_info, camera_extrinsics, port_pose)` tuples for training the vision-based port pose estimator (artifact #4). Uses Gazebo because the evaluation runs on Gazebo — no sim-to-sim gap.

---

## Files

```
aic_my_policy/data_collection/
├── randomize.py          # generates launch args for one random scene
├── capture_scene.py      # ROS node: one-shot capture, writes .npz, exits
└── collect_dataset.sh    # outer loop: launch -> settle -> capture -> repeat
```

---

## What gets stored per sample

A single `sample_XXXXXX.npz` file per scene, containing:

| Key | Shape | Notes |
|---|---|---|
| `image_left`, `image_center`, `image_right` | (H, W, 3) uint8 | RGB |
| `K_left`, `K_center`, `K_right` | (3, 3) | camera intrinsics |
| `width_{cam}`, `height_{cam}` | scalar | image dimensions |
| `cam_{cam}_to_base` | (7,) | static extrinsics `(tx, ty, tz, qx, qy, qz, qw)` |
| `port_poses` | (P, 7) | all ports present in TF this scene |
| `port_types` | (P,) object | `'sfp'` or `'sc'` |
| `port_frames` | (P,) object | TF frame names (for debugging) |

Each scene spawns multiple ports; the training loader indexes by (file, port_index).

---

## Randomization

`randomize.py` produces launch args within the challenge's stated limits (see `docs/task_board_description.md`):

| Parameter | Range |
|---|---|
| `task_board_x` | [0.10, 0.22] m |
| `task_board_y` | [-0.25, 0.05] m |
| `task_board_z` | fixed 1.14 m |
| `task_board_yaw` | [2.8, π+0.3] rad |
| `nic_card_mount_*_translation` | [0.0, 0.062] m |
| `nic_card_mount_*_yaw` | [-10°, +10°] |
| `sc_port_*_translation` | [0.0, 0.115] m |

Half of scenes are SFP-focused (NIC card on rail 0 or 1), half SC-focused (port on rail 0 or 1). Cable type and `attach_cable_to_gripper` are set to match so a plug is always grasped.

---

## Running

```bash
# Inside the aic_eval distrobox, with ROS 2 env sourced.
cd ~/ws_aic/src/aic
pixi reinstall ros-kilted-aic-my-policy

mkdir -p ~/aic_data/raw
bash aic_my_policy/aic_my_policy/data_collection/collect_dataset.sh \
    ~/aic_data/raw 500
```

Knobs (environment variables):

| Env var | Default | Meaning |
|---|---|---|
| `LAUNCH_READY_SEC` | 20 | seconds to wait for Gazebo to finish spawning |
| `SETTLE_SEC` | 3 | seconds of physics settling before capture |

Resume after interruption: `bash collect_dataset.sh ~/aic_data/raw 500 <last_idx + 1>`.

### Speed

Expect ~30 seconds per sample (20 s launch + 3 s settle + capture + teardown). 500 samples ≈ 4 hours wall-clock. Kick it off before bed.

For faster iteration, collect 50 samples first, train a dev model, validate the pipeline end-to-end, then scale up.

---

## Validation

After `collect_dataset.sh` finishes:

```bash
python - <<'EOF'
import numpy as np, pathlib, collections
ctr = collections.Counter()
for f in sorted(pathlib.Path("~/aic_data/raw").expanduser().glob("*.npz")):
    z = np.load(f, allow_pickle=True)
    for t in z["port_types"]:
        ctr[str(t)] += 1
print(f"Files: {len(list(pathlib.Path('~/aic_data/raw').expanduser().glob('*.npz')))}")
print(f"Ports by type: {dict(ctr)}")
EOF
```

Expect a rough 50/50 split between `sfp` and `sc`, with ~1–2 ports per file.

---

## Gotchas

- **Don't run while `aic_model` or `aic_engine` is also running.** The orchestrator owns the launch. Kill other sessions first.
- **Distrobox required.** `ros2 launch aic_bringup ...` needs the `aic_eval` container.
- **`ground_truth:=true` is baked in.** Needed for GT port poses; do **not** change it in this script.
- If many samples show `"No known port frames appeared in TF"`, the TF frame naming for the asset version you have may differ from `task_board/nic_card_mount_{n}/sfp_port_{p}_link`. Inspect with `ros2 run tf2_tools view_frames` during one launch and update `PORT_FRAMES_OF_INTEREST` in `capture_scene.py`.

---

## Next artifact

**Artifact #4** trains a ResNet-18-based keypoint network on this dataset to predict port pose directly from a single center-camera image (see `docs/04_vision_estimator.md`).
