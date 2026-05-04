#!/usr/bin/env python3
"""Export debug-only ground-truth SFP keypoint labels for detector development."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


CAMERAS = ("left", "center", "right")
KEYPOINT_SUFFIXES = (
    "sfp_port_0_link",
    "sfp_port_0_link_entrance",
    "sfp_port_1_link",
    "sfp_port_1_link_entrance",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export SFP keypoint labels from ground-truth projection output."
    )
    parser.add_argument(
        "capture_dir",
        type=Path,
        help="Directory produced by capture_ground_truth_debug.py.",
    )
    parser.add_argument(
        "--projection-summary",
        type=Path,
        default=None,
        help="projection_summary.json. Defaults to artifacts/ground_truth_debug_<capture>/gt_projection/projection_summary.json.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output directory. Defaults to artifacts/sfp_keypoint_labels_<capture>.",
    )
    parser.add_argument(
        "--copy-images",
        action="store_true",
        help="Copy source PPM images into the output images/ folder.",
    )
    return parser.parse_args()


def default_projection_summary(capture_dir: Path) -> Path:
    return (
        Path("artifacts")
        / f"ground_truth_debug_{capture_dir.name}"
        / "gt_projection"
        / "projection_summary.json"
    )


def default_output_dir(capture_dir: Path) -> Path:
    return Path("artifacts") / f"sfp_keypoint_labels_{capture_dir.name}"


def load_font(size: int = 16) -> ImageFont.ImageFont:
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size=size)
    except OSError:
        return ImageFont.load_default()


def suffix(frame_name: str) -> str:
    return frame_name.rsplit("/", 1)[-1]


def color_for_keypoint(name: str) -> str:
    if name.endswith("_entrance"):
        return "magenta"
    if "port_0" in name:
        return "cyan"
    return "lime"


def draw_cross(draw: ImageDraw.ImageDraw, u: float, v: float, color: str) -> None:
    draw.line([(u - 12, v), (u + 12, v)], fill=color, width=3)
    draw.line([(u, v - 12), (u, v + 12)], fill=color, width=3)


def keypoints_from_projection(camera_projection: list[dict[str, object]]) -> dict[str, dict[str, object]]:
    keypoints = {}
    for item in camera_projection:
        name = suffix(str(item["target_frame"]))
        if name not in KEYPOINT_SUFFIXES:
            continue
        keypoints[name] = {
            "u": float(item["u"]),
            "v": float(item["v"]),
            "visible": bool(item["visible"]),
            "z_camera_m": float(item["z_camera_m"]),
            "target_frame": item["target_frame"],
        }
    return keypoints


def annotate_image(image_path: Path, output_path: Path, keypoints: dict[str, dict[str, object]]) -> None:
    image = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(image)
    font = load_font()

    for name, point in keypoints.items():
        if not point["visible"]:
            continue
        color = color_for_keypoint(name)
        u, v = point["u"], point["v"]
        draw_cross(draw, u, v, color)
        draw.ellipse((u - 7, v - 7, u + 7, v + 7), outline=color, width=3)
        draw.text((int(u) + 10, int(v) + 8), name, fill=color, font=font)

    draw.rectangle((0, 0, image.width, 32), fill=(0, 0, 0))
    draw.text((8, 7), image_path.name, fill="white", font=font)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)


def build_contact_sheet(paths: list[Path], output_path: Path) -> None:
    if not paths:
        return
    thumb_width = 384
    thumb_height = 341
    label_height = 26
    margin = 14
    columns = 3
    rows = (len(paths) + columns - 1) // columns
    sheet = Image.new(
        "RGB",
        (columns * thumb_width, rows * (thumb_height + label_height + margin)),
        "white",
    )
    draw = ImageDraw.Draw(sheet)
    font = load_font(size=13)
    for index, path in enumerate(paths):
        image = Image.open(path).convert("RGB")
        image.thumbnail((thumb_width, thumb_height))
        x = (index % columns) * thumb_width
        y = (index // columns) * (thumb_height + label_height + margin)
        draw.text((x + 4, y + 5), path.stem, fill="black", font=font)
        sheet.paste(image, (x, y + label_height))
    sheet.save(output_path)


def main() -> int:
    args = parse_args()
    projection_summary_path = args.projection_summary or default_projection_summary(args.capture_dir)
    output_dir = args.output or default_output_dir(args.capture_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    annotations_dir = output_dir / "annotated"
    images_dir = output_dir / "images"

    summary = json.loads(projection_summary_path.read_text(encoding="utf-8"))
    labels = []
    annotated_paths = []

    for frame_summary in summary["frames"]:
        frame = frame_summary["frame"]
        for camera in CAMERAS:
            image_path = args.capture_dir / f"{frame}_{camera}.ppm"
            if not image_path.exists():
                continue
            keypoints = keypoints_from_projection(frame_summary["projections"][camera])
            complete = all(
                name in keypoints and keypoints[name]["visible"]
                for name in KEYPOINT_SUFFIXES
            )
            sample_id = f"{args.capture_dir.name}_{frame}_{camera}"
            image_ref = str(image_path)
            if args.copy_images:
                images_dir.mkdir(parents=True, exist_ok=True)
                copied = images_dir / f"{sample_id}.ppm"
                shutil.copy2(image_path, copied)
                image_ref = str(copied)

            label = {
                "sample_id": sample_id,
                "capture_dir": str(args.capture_dir),
                "frame": frame,
                "camera": camera,
                "image_path": image_ref,
                "image_width": 1152,
                "image_height": 1024,
                "complete": complete,
                "keypoints": keypoints,
            }
            labels.append(label)

            annotated_path = annotations_dir / f"{sample_id}_labels.png"
            annotate_image(image_path, annotated_path, keypoints)
            annotated_paths.append(annotated_path)

    labels_path = output_dir / "sfp_keypoint_labels.json"
    labels_path.write_text(json.dumps({"samples": labels}, indent=2), encoding="utf-8")
    jsonl_path = output_dir / "sfp_keypoint_labels.jsonl"
    with jsonl_path.open("w", encoding="utf-8") as file:
        for label in labels:
            file.write(json.dumps(label) + "\n")

    build_contact_sheet(annotated_paths, output_dir / "sfp_keypoint_label_contact_sheet.png")
    print(output_dir)
    print(f"samples={len(labels)} complete={sum(1 for item in labels if item['complete'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
