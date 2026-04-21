# Artifact #4 — Keypoint CNN + PnP Port-Pose Estimator

Replaces the ground-truth TF estimator with a learned vision pipeline. A small ResNet-18-based heatmap network predicts 4 keypoints on the port entrance; OpenCV's `solvePnP` then recovers the port's 6-DoF pose in the camera frame. The policy multiplies this by the camera-to-base static extrinsic to get the pose in `base_link` — the same quantity the GT estimator returns.

---

## Why keypoint + PnP (not direct 6-DoF regression)?

| | keypoint + PnP | direct 6-DoF regression |
|---|---|---|
| Failure mode | Localized: one bad keypoint → bounded error | Opaque: black-box regressor fails silently |
| Debuggable | Yes — visualize predicted pixels | No — just see the wrong pose |
| Sample efficiency | Higher (supervision is dense) | Lower |
| Needs calibrated camera | **Yes** (we have `CameraInfo`) | No |

We have calibrated cameras and care about debuggability, so keypoint + PnP wins.

---

## Files

```
aic_my_policy/perception/
├── keypoints.py        # hard-coded 3D port keypoint coordinates (local frame)
├── model.py            # ResNet-18 + deconv heatmap head
├── dataset.py          # loader + 3D->2D target generation
├── train.py            # training entry point (one port type per invocation)
└── infer.py            # image + K -> 6-DoF port pose in camera frame
```

---

## The 3D keypoints

Defined in `keypoints.py` **per port type**. Current placeholders:

| Port | Keypoints | Source |
|---|---|---|
| SFP | 4 corners of the rectangular opening (~13.7 × 8.5 mm) | SFP datasheet approximation |
| SC | 4 points on the ferrule bore circumference (~2.5 mm radius) | SC datasheet approximation |

**These values must be verified against the actual USD/SDF assets in `aic_assets/models/` before trusting the estimator.** Mark in the file with `TODO verify against asset` — easy to grep.

Convention: port local frame has +z into the port, keypoints sit on the z=0 entrance plane.

---

## Training

Train once per port type:

```bash
cd ~/ws_aic/src/aic
pixi shell

# SFP
python -m aic_my_policy.perception.train \
    --data_dir ~/aic_data/raw \
    --port_type sfp \
    --out ~/aic_data/models/sfp_keypoints.pt \
    --epochs 30 --batch_size 16 --lr 1e-3

# SC
python -m aic_my_policy.perception.train \
    --data_dir ~/aic_data/raw \
    --port_type sc \
    --out ~/aic_data/models/sc_keypoints.pt \
    --epochs 30 --batch_size 16 --lr 1e-3
```

The trainer:

1. Loads all samples containing that port type, using the center camera.
2. Projects the known 3D keypoints via `K · T_port_cam · kpts_local` → pixel targets.
3. Rasterizes each keypoint as a 2D Gaussian on a H/4 × W/4 output grid.
4. MSE-regresses `sigmoid(heatmap_logits)` against those Gaussians.
5. Cosine LR schedule over `--epochs`; saves best-val-loss checkpoint.

Training takes ~15–30 min per port on an L4 GPU with 500 samples.

---

## Inference

`infer.py` exposes `PortKeypointInference`:

```python
from aic_my_policy.perception.infer import PortKeypointInference

infer = PortKeypointInference("~/aic_data/models/sfp_keypoints.pt", port_type="sfp")
result = infer(image_rgb, K=camera_info_K)   # dict or None
if result is not None:
    R = result["R_port_cam"]      # 3x3
    t = result["t_port_cam"]      # 3
    pixels = result["keypoints_uv"]  # (4, 2) for visualization
```

`infer()` returns `None` if PnP fails (degenerate keypoint geometry or all-zero heatmaps). The caller is expected to reuse the last good pose estimate.

---

## Model sizing & runtime

| Item | Value |
|---|---|
| Input resolution | 256 × 256 |
| Output resolution | 64 × 64 (stride 4) |
| Params | ~11 M |
| Inference on L4 | ~3 ms/frame |
| At 20 Hz budget | comfortable (~50 ms per tick) |

---

## Validation protocol

On the held-out 10% split from training:

1. **Mean pixel error** per keypoint: target < 2 px at 256 × 256.
2. **PnP pose error**: reproject the predicted pose's keypoints and measure mean pixel residual; should be similar to item 1.
3. **3D translation error**: compare the predicted port pose to GT `port_poses` from the dataset; target < 5 mm.

Quick eval snippet for (3):

```python
import numpy as np, torch
from aic_my_policy.perception.infer import PortKeypointInference
from aic_my_policy.perception.dataset import PortKeypointDataset

ds = PortKeypointDataset("~/aic_data/raw", port_type="sfp")
infer = PortKeypointInference("~/aic_data/models/sfp_keypoints.pt", "sfp")

errors_m = []
for img_t, _, meta in ds:
    if not meta["valid"]:
        continue
    # De-normalize image for infer(): reverse the dataset's preprocessing.
    img = (img_t.numpy().transpose(1, 2, 0) * PortKeypointDataset.IMG_STD + PortKeypointDataset.IMG_MEAN) * 255
    img = img.clip(0, 255).astype("uint8")
    res = infer(img, K=meta["K"])
    if res is None:
        continue
    # Transform predicted pose into base_link for comparison against GT.
    # (Omitted for brevity; see estimators/vision.py:_lookup_transform_mat)
    t_pred_cam = res["t_port_cam"]
    # ...
    # errors_m.append(distance_in_base_link)
print("median 3D error (m):", np.median(errors_m))
```

---

## Known limitations

- **Single-camera only.** Using all three wrist cameras (via PnP merging or late-fusion heatmaps) should cut error 2–3× but adds complexity. Defer until we see how far single-camera goes.
- **Keypoint values are approximate.** The SFP/SC dimensions hardcoded in `keypoints.py` are from generic datasheets, not measured from the challenge assets. Verify before submitting.
- **No domain randomization in capture.** Lighting and textures come from whatever Gazebo configures by default. Adding randomization is a quick win that can be done *inside the data loader* via albumentations augmentation — color jitter, Gaussian noise, slight blur. Add if sim-to-eval gap shows up.
- **Weights only cover SFP and SC.** If the evaluation introduces a new connector type in a future phase, we need a new port-type class and a new training run.

---

## Next artifact

**Artifact #5** (`docs/05_wire_vision.md`) wires this estimator into `InsertCablePolicy` behind the existing `PortPoseEstimator` interface, selected via a ROS parameter. No changes to the state-machine logic.
