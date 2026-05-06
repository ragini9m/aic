"""Export captured `.npz` port labels to a YOLO detection dataset.

The raw capture format remains the source of truth. This exporter writes
camera images and normalized bbox labels for a detector that can find the
target port ROI before the keypoint/PnP estimator runs.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image

CLASS_IDS = {"sfp": 0, "sc": 1}
CAMERAS = ("left", "center", "right")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", required=True, help="directory containing capture .npz files")
    parser.add_argument("--out_dir", required=True, help="YOLO dataset output directory")
    parser.add_argument(
        "--cameras",
        nargs="+",
        default=["center"],
        choices=CAMERAS,
        help="camera views to export",
    )
    args = parser.parse_args()

    data_dir = Path(args.data_dir)
    out_dir = Path(args.out_dir)
    image_dir = out_dir / "images"
    label_dir = out_dir / "labels"
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)

    count_images = 0
    count_boxes = 0
    for sample_path in sorted(data_dir.glob("*.npz")):
        with np.load(sample_path, allow_pickle=True) as z:
            port_types = [str(t) for t in z["port_types"]]
            for camera in args.cameras:
                img_key = f"image_{camera}"
                bbox_key = f"port_bboxes_{camera}"
                visible_key = f"port_visible_{camera}"
                if img_key not in z or bbox_key not in z or visible_key not in z:
                    continue
                img = z[img_key]
                h, w = img.shape[:2]
                stem = f"{sample_path.stem}_{camera}"
                img_out = image_dir / f"{stem}.png"
                label_out = label_dir / f"{stem}.txt"

                rows = []
                for port_type, bbox, visible in zip(port_types, z[bbox_key], z[visible_key]):
                    if not bool(visible) or port_type not in CLASS_IDS:
                        continue
                    x1, y1, x2, y2 = [float(v) for v in bbox]
                    if x2 <= x1 or y2 <= y1:
                        continue
                    cx = ((x1 + x2) * 0.5) / w
                    cy = ((y1 + y2) * 0.5) / h
                    bw = (x2 - x1) / w
                    bh = (y2 - y1) / h
                    rows.append(f"{CLASS_IDS[port_type]} {cx:.8f} {cy:.8f} {bw:.8f} {bh:.8f}")

                if not rows:
                    continue
                Image.fromarray(img).save(img_out)
                label_out.write_text("\n".join(rows) + "\n")
                count_images += 1
                count_boxes += len(rows)

    (out_dir / "classes.txt").write_text("sfp\nsc\n")
    (out_dir / "dataset.yaml").write_text(
        f"path: {out_dir.resolve()}\n"
        "train: images\n"
        "val: images\n"
        "names:\n"
        "  0: sfp\n"
        "  1: sc\n"
    )
    print(f"exported {count_images} images with {count_boxes} boxes to {out_dir}")


if __name__ == "__main__":
    main()

