"""Keypoint detection network.

ResNet-18 backbone + small deconv head that outputs one heatmap per
port keypoint. Input: RGB image at arbitrary resolution (resized to
INPUT_SIZE). Output: heatmap of shape (K, H/4, W/4) with peaks at the
projected keypoint locations. A soft-argmax at inference time yields
subpixel coordinates.

We train a single network per port type (SFP vs SC); both share this
architecture.
"""

from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import resnet18, ResNet18_Weights

INPUT_SIZE = (256, 256)     # H, W input
OUTPUT_STRIDE = 4
OUTPUT_SIZE = (INPUT_SIZE[0] // OUTPUT_STRIDE, INPUT_SIZE[1] // OUTPUT_STRIDE)


class KeypointHeatmapNet(nn.Module):
    def __init__(self, num_keypoints: int = 4, pretrained: bool = True):
        super().__init__()
        weights = ResNet18_Weights.IMAGENET1K_V1 if pretrained else None
        backbone = resnet18(weights=weights)
        # Keep everything up to layer3 -> spatial stride 16 at this point.
        self.stem = nn.Sequential(
            backbone.conv1, backbone.bn1, backbone.relu, backbone.maxpool,
            backbone.layer1, backbone.layer2, backbone.layer3,
        )
        # Two upsamples to reach stride 4.
        self.up1 = nn.Sequential(
            nn.ConvTranspose2d(256, 128, 4, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(128), nn.ReLU(inplace=True),
        )
        self.up2 = nn.Sequential(
            nn.ConvTranspose2d(128, 64, 4, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(64), nn.ReLU(inplace=True),
        )
        self.head = nn.Conv2d(64, num_keypoints, kernel_size=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        h = self.stem(x)
        h = self.up1(h)
        h = self.up2(h)
        return self.head(h)  # logits; apply sigmoid at train time for loss


def soft_argmax_2d(heatmaps: torch.Tensor) -> torch.Tensor:
    """Soft-argmax over HxW. Returns (B, K, 2) as (x, y) in heatmap coords."""
    b, k, h, w = heatmaps.shape
    flat = heatmaps.view(b, k, -1)
    prob = F.softmax(flat, dim=-1).view(b, k, h, w)
    ys = torch.arange(h, dtype=heatmaps.dtype, device=heatmaps.device).view(1, 1, h, 1)
    xs = torch.arange(w, dtype=heatmaps.dtype, device=heatmaps.device).view(1, 1, 1, w)
    x = (prob * xs).sum(dim=(2, 3))
    y = (prob * ys).sum(dim=(2, 3))
    return torch.stack([x, y], dim=-1)   # (B, K, 2)


def heatmap_peaks_to_pixels(
    keypoints_hm: torch.Tensor,
    input_size=INPUT_SIZE,
    stride: int = OUTPUT_STRIDE,
) -> torch.Tensor:
    """Scale (B, K, 2) heatmap coords up to input-image pixel coords."""
    scale = stride
    return keypoints_hm * scale
