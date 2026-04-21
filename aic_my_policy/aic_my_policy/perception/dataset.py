"""Dataset loader + target generator for keypoint training.

Reads `.npz` samples written by `capture_scene.py`, projects the 3D
port keypoints into image space using the camera intrinsics and the
ground-truth port pose, and produces Gaussian heatmap targets.

For simplicity we use only the center camera. Extending to all three
cameras is a later optimization.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Literal

import numpy as np
import torch
from torch.utils.data import Dataset

from aic_my_policy.perception.keypoints import PORT_KEYPOINTS, NUM_KEYPOINTS
from aic_my_policy.perception.model import INPUT_SIZE, OUTPUT_SIZE, OUTPUT_STRIDE


def _quat_to_R(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy)],
            [    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx)],
            [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _pose7_to_T(pose7: np.ndarray) -> np.ndarray:
    """[tx, ty, tz, qx, qy, qz, qw] -> 4x4 homogeneous transform."""
    tx, ty, tz, qx, qy, qz, qw = pose7
    T = np.eye(4)
    T[:3, :3] = _quat_to_R(qx, qy, qz, qw)
    T[:3, 3] = (tx, ty, tz)
    return T


def project_port_keypoints(
    port_pose_base: np.ndarray,   # 7-vector
    cam_to_base: np.ndarray,      # 7-vector (base_link frame of camera optical)
    K: np.ndarray,                # 3x3 intrinsics
    port_type: str,
) -> np.ndarray | None:
    """Project a port's 3D keypoints into the camera image.

    Returns (K_num, 2) pixel coords, or None if any keypoint is behind
    the camera or outside the image area.
    """
    kpts_local = PORT_KEYPOINTS[port_type]                # (K, 3)
    T_port_base = _pose7_to_T(port_pose_base)
    T_cam_base = _pose7_to_T(cam_to_base)
    T_base_cam = np.linalg.inv(T_cam_base)
    T_port_cam = T_base_cam @ T_port_base                  # port in camera

    ones = np.ones((kpts_local.shape[0], 1))
    kpts_h = np.concatenate([kpts_local, ones], axis=1)    # (K, 4)
    kpts_cam = (T_port_cam @ kpts_h.T).T[:, :3]            # (K, 3)

    if (kpts_cam[:, 2] <= 1e-3).any():
        return None
    uv = (K @ kpts_cam.T).T
    uv = uv[:, :2] / uv[:, 2:3]
    return uv


def _gaussian_heatmap(H: int, W: int, cx: float, cy: float, sigma: float) -> np.ndarray:
    ys = np.arange(H).reshape(H, 1)
    xs = np.arange(W).reshape(1, W)
    d2 = (xs - cx) ** 2 + (ys - cy) ** 2
    return np.exp(-d2 / (2.0 * sigma * sigma)).astype(np.float32)


class PortKeypointDataset(Dataset):
    """Samples for a single port type ('sfp' or 'sc').

    Each __getitem__ returns:
      image: (3, H, W) float tensor, ImageNet-normalized
      heatmaps: (K, H/stride, W/stride) float tensor, Gaussian peaks
      meta: dict with numpy arrays (pose, intrinsics, etc.) for debugging
    """

    IMG_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    IMG_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    HEATMAP_SIGMA_PX = 2.0

    def __init__(
        self,
        root: str | Path,
        port_type: Literal["sfp", "sc"],
        camera: str = "center",
    ):
        self.root = Path(root)
        self.port_type = port_type
        self.camera = camera
        self._index: list[tuple[Path, int]] = []
        for path in sorted(self.root.glob("*.npz")):
            try:
                with np.load(path, allow_pickle=True) as z:
                    types = z["port_types"]
                for i, t in enumerate(types):
                    if str(t) == port_type:
                        self._index.append((path, i))
            except Exception:
                continue
        if not self._index:
            raise RuntimeError(f"No samples of type '{port_type}' under {root}")

    def __len__(self) -> int:
        return len(self._index)

    def __getitem__(self, i: int):
        path, port_idx = self._index[i]
        with np.load(path, allow_pickle=True) as z:
            img = z[f"image_{self.camera}"]
            K = z[f"K_{self.camera}"]
            cam_to_base = z[f"cam_{self.camera}_to_base"]
            port_pose = z["port_poses"][port_idx]

        # Project keypoints at original resolution first.
        uv_full = project_port_keypoints(port_pose, cam_to_base, K, self.port_type)
        if uv_full is None:
            # Degenerate sample (port behind camera). Return a zero mask so
            # the trainer can skip.
            img_t = self._preprocess_image(img)
            heat = np.zeros((NUM_KEYPOINTS, *OUTPUT_SIZE), dtype=np.float32)
            return img_t, torch.from_numpy(heat), {"valid": False}

        H_orig, W_orig = img.shape[:2]
        img_resized = self._resize(img, INPUT_SIZE)
        sx = INPUT_SIZE[1] / W_orig
        sy = INPUT_SIZE[0] / H_orig
        uv_input = uv_full.copy()
        uv_input[:, 0] *= sx
        uv_input[:, 1] *= sy

        hm_h, hm_w = OUTPUT_SIZE
        heat = np.zeros((NUM_KEYPOINTS, hm_h, hm_w), dtype=np.float32)
        for k in range(NUM_KEYPOINTS):
            cx = uv_input[k, 0] / OUTPUT_STRIDE
            cy = uv_input[k, 1] / OUTPUT_STRIDE
            if 0 <= cx < hm_w and 0 <= cy < hm_h:
                heat[k] = _gaussian_heatmap(hm_h, hm_w, cx, cy, self.HEATMAP_SIGMA_PX)

        img_t = self._preprocess_image(img_resized)
        meta = {
            "valid": True,
            "port_pose_base": port_pose,
            "K": K,
            "cam_to_base": cam_to_base,
            "uv_full": uv_full,
        }
        return img_t, torch.from_numpy(heat), meta

    @staticmethod
    def _resize(img: np.ndarray, size_hw) -> np.ndarray:
        import cv2
        return cv2.resize(img, (size_hw[1], size_hw[0]), interpolation=cv2.INTER_AREA)

    def _preprocess_image(self, img: np.ndarray) -> torch.Tensor:
        img_f = img.astype(np.float32) / 255.0
        img_f = (img_f - self.IMG_MEAN) / self.IMG_STD
        img_f = img_f.transpose(2, 0, 1)   # HWC -> CHW
        return torch.from_numpy(img_f.copy())
