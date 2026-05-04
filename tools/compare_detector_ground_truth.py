#!/usr/bin/env python3
"""Compare detector output against debug-only ground-truth projections."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from itertools import combinations

from PIL import Image, ImageDraw, ImageFont

from analyze_snapshots import (
    CAMERAS,
    component_to_dict,
    detect_ports,
    draw_cross,
    select_sfp_from_pair,
)
from project_ground_truth_debug import mat_vec, point_from_transform, quat_to_matrix


CAMERA_TO_FRAME = {
    "left": "left_camera/optical",
    "center": "center_camera/optical",
    "right": "right_camera/optical",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Overlay detector output and ground-truth projections."
    )
    parser.add_argument(
        "capture_dir",
        type=Path,
        help="Directory produced by tools/capture_ground_truth_debug.py.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output directory. Defaults to artifacts/gt_detector_compare_<capture-dir-name>.",
    )
    parser.add_argument("--port-type", default="sfp", choices=("sfp", "sc"))
    parser.add_argument("--port-name", default="sfp_port_0")
    parser.add_argument("--min-area", type=int, default=12)
    return parser.parse_args()


def load_font(size: int = 15) -> ImageFont.ImageFont:
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size=size)
    except OSError:
        return ImageFont.load_default()


def default_output_dir(capture_dir: Path) -> Path:
    return Path("artifacts") / f"gt_detector_compare_{capture_dir.name}"


def select_camera_component(components, port_type: str, port_name: str):
    if not components:
        return None
    if port_type == "sfp":
        return select_sfp_from_pair(components, port_name) or components[0]
    return components[0]


def transform_point(
    transform_base_child: dict[str, object],
    point_child: tuple[float, float, float],
) -> tuple[float, float, float]:
    rotation = quat_to_matrix(transform_base_child["rotation"])
    translation = point_from_transform(transform_base_child)
    rotated = mat_vec(rotation, point_child)
    return (
        translation[0] + rotated[0],
        translation[1] + rotated[1],
        translation[2] + rotated[2],
    )


def ray_from_pixel(
    u: float,
    v: float,
    camera_k: list[float],
    camera_transform_base: dict[str, object],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    fx, fy = float(camera_k[0]), float(camera_k[4])
    cx, cy = float(camera_k[2]), float(camera_k[5])
    x = (u - cx) / fx
    y = (v - cy) / fy
    direction_camera = (x, y, 1.0)
    norm = math.sqrt(sum(value * value for value in direction_camera))
    direction_camera = tuple(value / norm for value in direction_camera)
    direction_base = mat_vec(quat_to_matrix(camera_transform_base["rotation"]), direction_camera)
    direction_norm = math.sqrt(sum(value * value for value in direction_base))
    direction_base = tuple(value / direction_norm for value in direction_base)
    origin_base = point_from_transform(camera_transform_base)
    return origin_base, direction_base


def solve_3x3(matrix: list[list[float]], vector: list[float]) -> tuple[float, float, float] | None:
    a = [row[:] + [rhs] for row, rhs in zip(matrix, vector)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(a[row][col]))
        if abs(a[pivot][col]) < 1e-9:
            return None
        a[col], a[pivot] = a[pivot], a[col]
        scale = a[col][col]
        for item in range(col, 4):
            a[col][item] /= scale
        for row in range(3):
            if row == col:
                continue
            factor = a[row][col]
            for item in range(col, 4):
                a[row][item] -= factor * a[col][item]
    return a[0][3], a[1][3], a[2][3]


def triangulate_rays(
    rays: list[tuple[tuple[float, float, float], tuple[float, float, float]]],
) -> tuple[float, float, float] | None:
    if len(rays) < 2:
        return None

    matrix = [[0.0, 0.0, 0.0] for _ in range(3)]
    vector = [0.0, 0.0, 0.0]
    for origin, direction in rays:
        projection = [
            [float(i == j) - direction[i] * direction[j] for j in range(3)]
            for i in range(3)
        ]
        for i in range(3):
            vector[i] += sum(projection[i][j] * origin[j] for j in range(3))
            for j in range(3):
                matrix[i][j] += projection[i][j]
    return solve_3x3(matrix, vector)


def distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def find_target_projection(
    projection_results: list[dict[str, object]],
    camera: str,
    suffix: str,
) -> dict[str, object] | None:
    for item in projection_results:
        if item["target_frame"].endswith(suffix) and item.get("visible"):
            return item
    return None


def nearest_projection_errors(
    centroid: tuple[float, float],
    camera_projection_results: list[dict[str, object]],
) -> dict[str, object]:
    errors = {}
    for item in camera_projection_results:
        if not item.get("visible"):
            continue
        du = centroid[0] - float(item["u"])
        dv = centroid[1] - float(item["v"])
        errors[item["target_frame"]] = {
            "du_px": round(du, 2),
            "dv_px": round(dv, 2),
            "error_px": round(math.hypot(du, dv), 2),
        }
    return errors


def annotate_image(
    image_path: Path,
    output_path: Path,
    components,
    selected,
    gt_projection_results: list[dict[str, object]],
    title: str,
) -> None:
    image = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(image)
    font = load_font()

    for index, component in enumerate(components[:6]):
        color = "lime" if component == selected else "yellow"
        width = 4 if component == selected else 2
        draw.rectangle(component.bbox, outline=color, width=width)
        draw_cross(draw, component.centroid[0], component.centroid[1], color)
        draw.text(
            (component.bbox[0], max(0, component.bbox[1] - 20)),
            f"det {index}: area={component.area} score={component.score:.0f}",
            fill=color,
            font=font,
        )

    for item in gt_projection_results:
        if not item.get("visible"):
            continue
        color = "cyan"
        if str(item["target_frame"]).endswith("_entrance"):
            color = "magenta"
        u, v = float(item["u"]), float(item["v"])
        draw.ellipse((u - 9, v - 9, u + 9, v + 9), outline=color, width=4)
        draw.text(
            (int(u) + 10, int(v) + 10),
            "GT entrance" if color == "magenta" else "GT port",
            fill=color,
            font=font,
        )

    draw.rectangle((0, 0, image.width, 34), fill=(0, 0, 0))
    draw.text((8, 7), title, fill="white", font=font)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)


def build_contact_sheet(output_dir: Path) -> None:
    paths = sorted(output_dir.glob("*_compare.png"))
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
    sheet.save(output_dir / "detector_gt_compare_contact_sheet.png")


def compare_frame(
    metadata_path: Path,
    projection_summary: dict[str, object],
    output_dir: Path,
    port_type: str,
    port_name: str,
    min_area: int,
) -> dict[str, object]:
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    frame = metadata_path.name.split("_", 1)[0]
    projection_frame = next(
        item for item in projection_summary["frames"] if item["frame"] == frame
    )
    camera_transforms = metadata["camera_transforms_base_link"]
    target_transforms = metadata["ground_truth_transforms_base_link"]
    camera_results = {}
    rays_by_camera = {}

    for camera in CAMERAS:
        image_path = metadata_path.parent / f"{frame}_{camera}.ppm"
        components = detect_ports(image_path, port_type, camera, min_area)
        selected = select_camera_component(components, port_type, port_name)
        gt_results = projection_frame["projections"][camera]
        title = f"{metadata_path.parent.name} / {frame}_{camera}"
        annotate_image(
            image_path,
            output_dir / f"{frame}_{camera}_compare.png",
            components,
            selected,
            gt_results,
            title,
        )

        selected_dict = component_to_dict(selected) if selected else None
        pixel_errors = {}
        if selected:
            pixel_errors = nearest_projection_errors(selected.centroid, gt_results)
            camera_k = metadata["cameras"][camera]["k"]
            camera_tf = camera_transforms.get(CAMERA_TO_FRAME[camera])
            if camera_tf:
                rays_by_camera[camera] = ray_from_pixel(
                    selected.centroid[0],
                    selected.centroid[1],
                    camera_k,
                    camera_tf,
                )

        camera_results[camera] = {
            "selected_detection": selected_dict,
            "pixel_errors_to_ground_truth": pixel_errors,
        }

    target_points = {
        target_frame: point_from_transform(transform)
        for target_frame, transform in target_transforms.items()
    }

    pairwise_results = {}
    for camera_a, camera_b in combinations(rays_by_camera.keys(), 2):
        triangulated_pair = triangulate_rays([rays_by_camera[camera_a], rays_by_camera[camera_b]])
        if not triangulated_pair:
            continue
        pairwise_results[f"{camera_a}_{camera_b}"] = {
            "point_base_link": [round(value, 6) for value in triangulated_pair],
            "errors_to_ground_truth": {
                target_frame: {
                    "error_m": round(distance(triangulated_pair, target_point), 5),
                    "error_mm": round(distance(triangulated_pair, target_point) * 1000.0, 2),
                }
                for target_frame, target_point in target_points.items()
            },
        }

    triangulated = triangulate_rays(list(rays_by_camera.values()))
    triangulation_result = None
    if triangulated:
        errors = {}
        for target_frame, target_point in target_points.items():
            errors[target_frame] = {
                "error_m": round(distance(triangulated, target_point), 5),
                "error_mm": round(distance(triangulated, target_point) * 1000.0, 2),
            }
        triangulation_result = {
            "point_base_link": [round(value, 6) for value in triangulated],
            "ray_count": len(rays_by_camera),
            "errors_to_ground_truth": errors,
            "pairwise": pairwise_results,
        }

    return {
        "frame": frame,
        "camera_results": camera_results,
        "triangulation": triangulation_result,
    }


def ensure_projection(capture_dir: Path, output_dir: Path) -> dict[str, object]:
    projection_path = output_dir / "gt_projection" / "projection_summary.json"
    if not projection_path.exists():
        raise SystemExit(
            "Run project_ground_truth_debug.py first, or pass an output directory "
            "that contains gt_projection/projection_summary.json"
        )
    return json.loads(projection_path.read_text(encoding="utf-8"))


def main() -> int:
    args = parse_args()
    output_dir = args.output or default_output_dir(args.capture_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    projection_summary = ensure_projection(args.capture_dir, output_dir)
    metadata_paths = sorted(args.capture_dir.glob("*_metadata.json"))
    results = [
        compare_frame(
            metadata_path,
            projection_summary,
            output_dir,
            args.port_type,
            args.port_name,
            args.min_area,
        )
        for metadata_path in metadata_paths
    ]
    build_contact_sheet(output_dir)
    summary = {
        "capture_dir": str(args.capture_dir),
        "port_type": args.port_type,
        "port_name": args.port_name,
        "frames": results,
    }
    (output_dir / "detector_gt_compare_summary.json").write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8",
    )
    print(output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
