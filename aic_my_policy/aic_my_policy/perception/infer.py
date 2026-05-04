"""Inference: image + intrinsics -> 6-DoF port pose in camera frame.

Combines the trained heatmap net with cv2.solvePnP.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import torch
from PIL import Image

from aic_my_policy.perception.dataset import PortKeypointDataset
from aic_my_policy.perception.keypoints import PORT_KEYPOINTS
from aic_my_policy.perception.model import (
    INPUT_SIZE,
    OUTPUT_STRIDE,
    KeypointHeatmapNet,
)


class PortKeypointInference:
    def __init__(
        self,
        weights_path: str | Path,
        port_type: str,
        device: str = "cpu",
    ):
        ckpt = torch.load(weights_path, map_location=device)
        self.port_type = ckpt.get("port_type", port_type)
        self.device = device
        self.model = KeypointHeatmapNet(
            num_keypoints=PORT_KEYPOINTS[self.port_type].shape[0],
            pretrained=False,
        )
        self.model.load_state_dict(ckpt["model"])
        self.model.to(device).eval()
        self.kpts_3d = PORT_KEYPOINTS[self.port_type].astype(np.float64)

    def __call__(
        self,
        image_rgb: np.ndarray,          # HxWx3 uint8 or float
        K: np.ndarray,                  # 3x3 intrinsics in original image coords
    ) -> Optional[dict]:
        H_orig, W_orig = image_rgb.shape[:2]
        img_resized = np.array(Image.fromarray(image_rgb).resize((INPUT_SIZE[1], INPUT_SIZE[0]), Image.LANCZOS))

        img_f = img_resized.astype(np.float32) / 255.0
        img_f = (img_f - PortKeypointDataset.IMG_MEAN) / PortKeypointDataset.IMG_STD
        img_t = torch.from_numpy(img_f.transpose(2, 0, 1)).unsqueeze(0).to(self.device)

        with torch.no_grad():
            heatmaps = self.model(img_t)
            heatmap_probs = torch.sigmoid(heatmaps)
            peaks_input, peak_scores = _heatmap_argmax_to_input_pixels(heatmap_probs)
            peaks_input = peaks_input[0].cpu().numpy()
            peak_scores = peak_scores[0].cpu().numpy()

        sx = W_orig / INPUT_SIZE[1]
        sy = H_orig / INPUT_SIZE[0]
        uv_full = np.empty_like(peaks_input)
        uv_full[:, 0] = peaks_input[:, 0] * sx
        uv_full[:, 1] = peaks_input[:, 1] * sy

        candidates = _solve_pnp_candidates(
            object_points=self.kpts_3d,
            image_points=uv_full.astype(np.float64),
            K=K,
        )
        if not candidates:
            return None
        best = min(candidates, key=lambda c: c["reprojection_error_px"])
        return {
            "R_port_cam": best["R_port_cam"],          # port-in-camera rotation
            "t_port_cam": best["t_port_cam"],          # port-in-camera translation
            "keypoints_uv": uv_full,
            "keypoint_scores": peak_scores,
            "reprojection_error_px": best["reprojection_error_px"],
            "pnp_candidates": candidates,
        }


def _heatmap_argmax_to_input_pixels(
    heatmaps: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Return hard heatmap peaks in input-image pixel coordinates."""
    b, k, h, w = heatmaps.shape
    flat = heatmaps.flatten(2)
    scores, idx = flat.max(dim=-1)
    y = torch.div(idx, w, rounding_mode="floor").to(dtype=heatmaps.dtype)
    x = (idx % w).to(dtype=heatmaps.dtype)
    peaks = torch.stack([x, y], dim=-1) * float(OUTPUT_STRIDE)
    return peaks, scores


def _solve_pnp_candidates(
    object_points: np.ndarray,
    image_points: np.ndarray,
    K: np.ndarray,
) -> list[dict]:
    """Return plausible planar PnP candidates sorted later by the caller."""
    candidates: list[dict] = []

    for flag, name in (
        (cv2.SOLVEPNP_IPPE, "IPPE"),
        (cv2.SOLVEPNP_ITERATIVE, "ITERATIVE"),
    ):
        try:
            result = cv2.solvePnPGeneric(
                objectPoints=object_points,
                imagePoints=image_points,
                cameraMatrix=K,
                distCoeffs=None,
                flags=flag,
            )
        except cv2.error:
            continue
        ok = bool(result[0])
        if not ok:
            continue
        rvecs = result[1]
        tvecs = result[2]
        for rvec, tvec in zip(rvecs, tvecs):
            if tvec.reshape(3)[2] <= 0.0:
                continue
            err = _mean_reprojection_error(object_points, image_points, K, rvec, tvec)
            R, _ = cv2.Rodrigues(rvec)
            candidates.append(
                {
                    "solver": name,
                    "R_port_cam": R,
                    "t_port_cam": tvec.reshape(3),
                    "reprojection_error_px": err,
                }
            )

    return candidates


def _mean_reprojection_error(
    object_points: np.ndarray,
    image_points: np.ndarray,
    K: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
) -> float:
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, None)
    projected = projected.reshape(-1, 2)
    return float(np.linalg.norm(projected - image_points, axis=1).mean())
