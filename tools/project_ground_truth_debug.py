#!/usr/bin/env python3
"""Project debug-only ground-truth TF points into captured camera images."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


CAMERA_TO_FRAME = {
    "left": "left_camera/optical",
    "center": "center_camera/optical",
    "right": "right_camera/optical",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Overlay ground-truth target frame projections on debug captures."
    )
    parser.add_argument(
        "capture_dir",
        type=Path,
        help="Directory created by tools/capture_ground_truth_debug.py.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output directory. Defaults to <capture_dir>/gt_projection.",
    )
    return parser.parse_args()


def load_font(size: int = 16) -> ImageFont.ImageFont:
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size=size)
    except OSError:
        return ImageFont.load_default()


def quat_to_matrix(q: dict[str, float]) -> list[list[float]]:
    x, y, z, w = q["x"], q["y"], q["z"], q["w"]
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [[matrix[j][i] for j in range(3)] for i in range(3)]


def mat_vec(matrix: list[list[float]], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3))


def point_from_transform(transform: dict[str, object]) -> tuple[float, float, float]:
    translation = transform["translation"]
    return (
        float(translation["x"]),
        float(translation["y"]),
        float(translation["z"]),
    )


def project_point(
    point_base: tuple[float, float, float],
    camera_transform_base: dict[str, object],
    camera_k: list[float],
) -> tuple[float, float, float] | None:
    cam_origin_base = point_from_transform(camera_transform_base)
    rotation_base_camera = quat_to_matrix(camera_transform_base["rotation"])
    rotation_camera_base = transpose(rotation_base_camera)
    point_relative = (
        point_base[0] - cam_origin_base[0],
        point_base[1] - cam_origin_base[1],
        point_base[2] - cam_origin_base[2],
    )
    x, y, z = mat_vec(rotation_camera_base, point_relative)
    if z <= 0.0:
        return None
    fx, fy = float(camera_k[0]), float(camera_k[4])
    cx, cy = float(camera_k[2]), float(camera_k[5])
    u = fx * x / z + cx
    v = fy * y / z + cy
    return u, v, z


def draw_cross(draw: ImageDraw.ImageDraw, u: float, v: float, color: str) -> None:
    x = int(round(u))
    y = int(round(v))
    draw.line([(x - 14, y), (x + 14, y)], fill=color, width=3)
    draw.line([(x, y - 14), (x, y + 14)], fill=color, width=3)


def label_for_frame(frame: str) -> str:
    parts = frame.split("/")
    if len(parts) >= 3:
        return "/".join(parts[-2:])
    return frame


def color_for_frame(frame: str) -> str:
    if frame.endswith("_entrance"):
        return "lime"
    if "sfp_port" in frame:
        return "cyan"
    if "sc_port" in frame:
        return "magenta"
    return "yellow"


def annotate_frame(metadata_path: Path, output_dir: Path) -> dict[str, object]:
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    frame_prefix = metadata_path.name.split("_", 1)[0]
    target_transforms = metadata["ground_truth_transforms_base_link"]
    camera_transforms = metadata.get("camera_transforms_base_link", {})
    projections: dict[str, list[dict[str, object]]] = {}
    font = load_font()

    for camera_name, camera_frame in CAMERA_TO_FRAME.items():
        if camera_frame not in camera_transforms:
            projections[camera_name] = []
            continue
        image_path = metadata_path.parent / f"{frame_prefix}_{camera_name}.ppm"
        image = Image.open(image_path).convert("RGB")
        draw = ImageDraw.Draw(image)
        camera_k = metadata["cameras"][camera_name]["k"]
        camera_results = []

        for target_frame, target_transform in target_transforms.items():
            projected = project_point(
                point_from_transform(target_transform),
                camera_transforms[camera_frame],
                camera_k,
            )
            if projected is None:
                camera_results.append(
                    {
                        "target_frame": target_frame,
                        "visible": False,
                        "reason": "behind_camera",
                    }
                )
                continue
            u, v, z = projected
            in_image = 0 <= u < image.width and 0 <= v < image.height
            color = color_for_frame(target_frame)
            if in_image:
                draw_cross(draw, u, v, color)
                draw.text(
                    (int(u) + 8, int(v) + 8),
                    f"{label_for_frame(target_frame)} z={z:.3f}m",
                    fill=color,
                    font=font,
                )
            camera_results.append(
                {
                    "target_frame": target_frame,
                    "visible": in_image,
                    "u": round(u, 2),
                    "v": round(v, 2),
                    "z_camera_m": round(z, 4),
                }
            )

        draw.rectangle((0, 0, image.width, 30), fill=(0, 0, 0))
        draw.text(
            (8, 6),
            f"{metadata_path.parent.name} / {frame_prefix}_{camera_name}",
            fill="white",
            font=font,
        )
        output_path = output_dir / f"{frame_prefix}_{camera_name}_gt_projection.png"
        image.save(output_path)
        projections[camera_name] = camera_results

    return {"frame": frame_prefix, "projections": projections}


def build_contact_sheet(output_dir: Path) -> None:
    paths = sorted(output_dir.glob("*_gt_projection.png"))
    if not paths:
        return

    thumb_width = 384
    thumb_height = 341
    label_height = 26
    margin = 14
    columns = 3
    rows = math.ceil(len(paths) / columns)
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
        col = index % columns
        row = index // columns
        x = col * thumb_width
        y = row * (thumb_height + label_height + margin)
        draw.text((x + 4, y + 5), path.stem, fill="black", font=font)
        sheet.paste(image, (x, y + label_height))

    sheet.save(output_dir / "gt_projection_contact_sheet.png")


def main() -> int:
    args = parse_args()
    output_dir = args.output or (args.capture_dir / "gt_projection")
    output_dir.mkdir(parents=True, exist_ok=True)

    metadata_paths = sorted(args.capture_dir.glob("*_metadata.json"))
    if not metadata_paths:
        raise SystemExit(f"No metadata files found in {args.capture_dir}")

    results = [annotate_frame(path, output_dir) for path in metadata_paths]
    build_contact_sheet(output_dir)
    summary_path = output_dir / "projection_summary.json"
    summary_path.write_text(json.dumps({"frames": results}, indent=2), encoding="utf-8")
    print(output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
