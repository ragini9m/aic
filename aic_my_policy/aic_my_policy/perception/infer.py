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
    KeypointHeatmapNet,
    heatmap_peaks_to_pixels,
    soft_argmax_2d,
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
            peak_scores = torch.sigmoid(heatmaps).flatten(2).amax(dim=-1)
            peaks_hm = soft_argmax_2d(heatmaps)                 # (1, K, 2) in heatmap coords
            peaks_input = heatmap_peaks_to_pixels(peaks_hm)      # in 256x256 coords
            peaks_input = peaks_input[0].cpu().numpy()
            peak_scores = peak_scores[0].cpu().numpy()

        sx = W_orig / INPUT_SIZE[1]
        sy = H_orig / INPUT_SIZE[0]
        uv_full = np.empty_like(peaks_input)
        uv_full[:, 0] = peaks_input[:, 0] * sx
        uv_full[:, 1] = peaks_input[:, 1] * sy

        pnp = _solve_pnp_best(
            object_points=self.kpts_3d,
            image_points=uv_full.astype(np.float64),
            K=K,
        )
        if pnp is None:
            return None
        rvec, tvec, reprojection_error = pnp
        R, _ = cv2.Rodrigues(rvec)
        return {
            "R_port_cam": R,                # port-in-camera rotation
            "t_port_cam": tvec.reshape(3),  # port-in-camera translation
            "keypoints_uv": uv_full,
            "keypoint_scores": peak_scores,
            "reprojection_error_px": reprojection_error,
        }


def _solve_pnp_best(
    object_points: np.ndarray,
    image_points: np.ndarray,
    K: np.ndarray,
) -> Optional[tuple[np.ndarray, np.ndarray, float]]:
    """Solve planar PnP and choose the candidate with lowest reprojection error."""
    candidates: list[tuple[np.ndarray, np.ndarray, float]] = []

    # The port keypoints are coplanar, so IPPE gives better initial solutions
    # than the generic iterative method when heatmaps are noisy.
    for flag in (cv2.SOLVEPNP_IPPE, cv2.SOLVEPNP_ITERATIVE):
        try:
            ok, rvec, tvec = cv2.solvePnP(
                objectPoints=object_points,
                imagePoints=image_points,
                cameraMatrix=K,
                distCoeffs=None,
                flags=flag,
            )
        except cv2.error:
            continue
        if not ok or tvec.reshape(3)[2] <= 0.0:
            continue
        err = _mean_reprojection_error(object_points, image_points, K, rvec, tvec)
        candidates.append((rvec, tvec, err))

    if not candidates:
        return None
    return min(candidates, key=lambda c: c[2])


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
