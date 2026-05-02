"""3D keypoint definitions for port types, in the port's local frame.

These are design-time constants that depend on the asset geometry in
`aic_assets/models/`. The values below are reasonable approximations
from the connector datasheets (SFP: 13.7 × 8.5 × 56.5 mm opening; SC:
~2.5 mm ferrule bore). **Measure the exact values from the USD/SDF
assets before trusting the pose estimate.**

Convention: port local frame has +z pointing INTO the port (down the
insertion axis). The keypoints define the rectangle at the port
entrance (z=0).
"""

from __future__ import annotations

import numpy as np

# --- SFP (small form-factor pluggable) -------------------------------------
# 4 keypoints at the corners of the rectangular port entrance.
SFP_HALF_WIDTH_M = 0.00642   # 13.7 mm wide  -> TODO verify against asset
SFP_HALF_HEIGHT_M = 0.004139  # 8.5 mm tall   -> TODO verify against asset

SFP_KEYPOINTS_LOCAL = np.array(
    [
        [+SFP_HALF_WIDTH_M, +SFP_HALF_HEIGHT_M, 0.0],  # TR
        [-SFP_HALF_WIDTH_M, +SFP_HALF_HEIGHT_M, 0.0],  # TL
        [-SFP_HALF_WIDTH_M, -SFP_HALF_HEIGHT_M, 0.0],  # BL
        [+SFP_HALF_WIDTH_M, -SFP_HALF_HEIGHT_M, 0.0],  # BR
    ],
    dtype=np.float64,
)

# --- SC (subscriber connector) ---------------------------------------------
# SC is circular; pick 4 points on the bore rim for PnP stability.
SC_RADIUS_M = 0.0019  # 2.5 mm bore -> TODO verify against asset

SC_KEYPOINTS_LOCAL = np.array(
    [
        [+SC_RADIUS_M, 0.0, 0.0],
        [0.0, +SC_RADIUS_M, 0.0],
        [-SC_RADIUS_M, 0.0, 0.0],
        [0.0, -SC_RADIUS_M, 0.0],
    ],
    dtype=np.float64,
)


PORT_KEYPOINTS: dict[str, np.ndarray] = {
    "sfp": SFP_KEYPOINTS_LOCAL,
    "sc": SC_KEYPOINTS_LOCAL,
}

NUM_KEYPOINTS = 4  # all port types share the same head size
