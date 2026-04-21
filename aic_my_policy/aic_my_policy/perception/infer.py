"""Inference: image + intrinsics -> 6-DoF port pose in camera frame.

Combines the trained heatmap net with cv2.solvePnP.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import torch

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
        device: str = "cuda" if torch.cuda.is_available() else "cpu",
    ):
        ckpt = torch.load(weights_path, map_location=device)
        self.port_type = ckpt.get("port_type", port_type)
        self.device = device
        self.model = KeypointHeatmapNet(num_keypoints=PORT_KEYPOINTS[self.port_type].shape[0])
        self.model.load_state_dict(ckpt["model"])
        self.model.to(device).eval()
        self.kpts_3d = PORT_KEYPOINTS[self.port_type].astype(np.float64)

    def __call__(
        self,
        image_rgb: np.ndarray,          # HxWx3 uint8 or float
        K: np.ndarray,                  # 3x3 intrinsics in original image coords
    ) -> Optional[dict]:
        H_orig, W_orig = image_rgb.shape[:2]
        img_resized = cv2.resize(image_rgb, (INPUT_SIZE[1], INPUT_SIZE[0]), interpolation=cv2.INTER_AREA)

        img_f = img_resized.astype(np.float32) / 255.0
        img_f = (img_f - PortKeypointDataset.IMG_MEAN) / PortKeypointDataset.IMG_STD
        img_t = torch.from_numpy(img_f.transpose(2, 0, 1)).unsqueeze(0).to(self.device)

        with torch.no_grad():
            heatmaps = self.model(img_t)
            peaks_hm = soft_argmax_2d(heatmaps)                 # (1, K, 2) in heatmap coords
            peaks_input = heatmap_peaks_to_pixels(peaks_hm)      # in 256x256 coords
            peaks_input = peaks_input[0].cpu().numpy()

        sx = W_orig / INPUT_SIZE[1]
        sy = H_orig / INPUT_SIZE[0]
        uv_full = np.empty_like(peaks_input)
        uv_full[:, 0] = peaks_input[:, 0] * sx
        uv_full[:, 1] = peaks_input[:, 1] * sy

        ok, rvec, tvec = cv2.solvePnP(
            objectPoints=self.kpts_3d,
            imagePoints=uv_full.astype(np.float64),
            cameraMatrix=K,
            distCoeffs=None,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        return {
            "R_port_cam": R,                # port-in-camera rotation
            "t_port_cam": tvec.reshape(3),  # port-in-camera translation
            "keypoints_uv": uv_full,
        }
