#!/usr/bin/env python3
"""Offline first-pass port detector for saved AIC perception snapshots."""

from __future__ import annotations

import argparse
import json
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from PIL import Image, ImageDraw, ImageFont


DEFAULT_INPUT = Path("artifacts/perception_snapshot_20260503_005954")
CAMERAS = ("left", "center", "right")


@dataclass(frozen=True)
class Component:
    bbox: tuple[int, int, int, int]
    area: int
    centroid: tuple[float, float]
    mean_rgb: tuple[float, float, float]
    score: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze saved AIC camera snapshots and annotate likely target ports."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_INPUT,
        help=f"Snapshot directory, relative to cwd by default: {DEFAULT_INPUT}",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help=(
            "Output directory. Defaults to artifacts/perception_analysis_<input-dir-name> "
            "so new snapshot runs get separate analysis folders."
        ),
    )
    parser.add_argument(
        "--min-area",
        type=int,
        default=12,
        help="Minimum connected-component area in pixels.",
    )
    return parser.parse_args()


def default_output_dir(input_dir: Path) -> Path:
    run_name = input_dir.name
    if run_name.startswith("perception_snapshot_"):
        run_name = run_name.removeprefix("perception_snapshot_")
    return Path("artifacts") / f"perception_analysis_{run_name}"


def load_font(size: int = 16) -> ImageFont.ImageFont:
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size=size)
    except OSError:
        return ImageFont.load_default()


def is_sc_pixel(r: int, g: int, b: int) -> bool:
    return b > 120 and g > 95 and r < 95 and b > r + 55 and g > r + 45


def is_sfp_pixel(r: int, g: int, b: int) -> bool:
    return g > 45 and r < 95 and b < 120 and g > r + 12 and g >= b - 12


def collect_components(
    image: Image.Image,
    predicate: Callable[[int, int, int], bool],
    min_area: int,
    max_components: int = 20,
) -> list[Component]:
    rgb = image.convert("RGB")
    width, height = rgb.size
    pixels = list(rgb.getdata())
    mask = bytearray(1 if predicate(*pixel) else 0 for pixel in pixels)
    seen = bytearray(width * height)
    components: list[Component] = []

    for start in range(width * height):
        if not mask[start] or seen[start]:
            continue

        queue: deque[int] = deque([start])
        seen[start] = 1
        area = 0
        sum_x = 0
        sum_y = 0
        sum_r = 0
        sum_g = 0
        sum_b = 0
        min_x = width
        min_y = height
        max_x = 0
        max_y = 0

        while queue:
            index = queue.popleft()
            y, x = divmod(index, width)
            r, g, b = pixels[index]
            area += 1
            sum_x += x
            sum_y += y
            sum_r += r
            sum_g += g
            sum_b += b
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            max_x = max(max_x, x)
            max_y = max(max_y, y)

            if x > 0:
                neighbor = index - 1
                if mask[neighbor] and not seen[neighbor]:
                    seen[neighbor] = 1
                    queue.append(neighbor)
            if x + 1 < width:
                neighbor = index + 1
                if mask[neighbor] and not seen[neighbor]:
                    seen[neighbor] = 1
                    queue.append(neighbor)
            if y > 0:
                neighbor = index - width
                if mask[neighbor] and not seen[neighbor]:
                    seen[neighbor] = 1
                    queue.append(neighbor)
            if y + 1 < height:
                neighbor = index + width
                if mask[neighbor] and not seen[neighbor]:
                    seen[neighbor] = 1
                    queue.append(neighbor)

        if area < min_area:
            continue

        box_width = max_x - min_x + 1
        box_height = max_y - min_y + 1
        if box_width < 2 or box_height < 2:
            continue

        mean_rgb = (sum_r / area, sum_g / area, sum_b / area)
        centroid = (sum_x / area, sum_y / area)
        components.append(
            Component(
                bbox=(min_x, min_y, max_x, max_y),
                area=area,
                centroid=centroid,
                mean_rgb=mean_rgb,
                score=0.0,
            )
        )

    components.sort(key=lambda component: component.area, reverse=True)
    return components[:max_components]


def score_sc(component: Component, image_size: tuple[int, int], camera: str) -> float:
    width, height = image_size
    x, y = component.centroid
    min_x, min_y, max_x, max_y = component.bbox
    box_width = max_x - min_x + 1
    box_height = max_y - min_y + 1
    aspect = box_width / max(box_height, 1)
    aspect_score = 1.0 / (1.0 + abs(aspect - 0.55))
    saturation = component.mean_rgb[2] + component.mean_rgb[1] - component.mean_rgb[0]
    camera_bonus = 1.15 if camera == "center" else 1.0
    board_region_bonus = 1.0 if y < height * 0.75 else 0.65
    horizontal_bonus = 1.0 - min(abs(x - width * 0.5) / (width * 0.75), 0.45)
    return component.area * aspect_score * saturation * camera_bonus * board_region_bonus * horizontal_bonus


def score_sfp(component: Component, image_size: tuple[int, int], camera: str) -> float:
    width, height = image_size
    x, y = component.centroid
    min_x, min_y, max_x, max_y = component.bbox
    box_width = max_x - min_x + 1
    box_height = max_y - min_y + 1
    aspect = box_width / max(box_height, 1)
    aspect_score = 1.0 / (1.0 + abs(aspect - 1.8))
    green_contrast = component.mean_rgb[1] - 0.5 * (component.mean_rgb[0] + component.mean_rgb[2])
    center_camera_bonus = 1.5 if camera == "center" else 0.75
    lower_half_bonus = 1.25 if y > height * 0.35 else 0.65
    central_bonus = 1.0 - min(abs(x - width * 0.5) / (width * 0.65), 0.55)
    size_penalty = 0.55 if component.area > 4000 else 1.0
    return (
        component.area
        * max(green_contrast, 1.0)
        * aspect_score
        * center_camera_bonus
        * lower_half_bonus
        * central_bonus
        * size_penalty
    )


def with_scores(
    components: list[Component],
    scorer: Callable[[Component, tuple[int, int], str], float],
    image_size: tuple[int, int],
    camera: str,
) -> list[Component]:
    scored = [
        Component(
            bbox=component.bbox,
            area=component.area,
            centroid=component.centroid,
            mean_rgb=component.mean_rgb,
            score=scorer(component, image_size, camera),
        )
        for component in components
    ]
    return sorted(scored, key=lambda component: component.score, reverse=True)


def detect_ports(
    image_path: Path,
    port_type: str,
    camera: str,
    min_area: int,
) -> list[Component]:
    image = Image.open(image_path).convert("RGB")
    if port_type == "sc":
        components = collect_components(image, is_sc_pixel, min_area=min_area)
        return with_scores(components, score_sc, image.size, camera)
    if port_type == "sfp":
        components = collect_components(image, is_sfp_pixel, min_area=min_area)
        return with_scores(components, score_sfp, image.size, camera)
    return []


def select_detection(
    detections: dict[str, list[Component]],
    port_type: str,
    port_name: str,
) -> tuple[str | None, Component | None, str]:
    if port_type == "sfp" and detections.get("center"):
        paired = select_sfp_from_pair(detections["center"], port_name)
        if paired:
            return "center", paired, "selected from matched SFP port pair in center camera"
        return "center", detections["center"][0], "fallback to highest-scoring center-camera SFP component"

    best_camera = None
    best_component = None
    for camera, components in detections.items():
        if not components:
            continue
        if best_component is None or components[0].score > best_component.score:
            best_camera = camera
            best_component = components[0]
    return best_camera, best_component, "selected highest-scoring component across cameras"


def parse_port_index(port_name: str) -> int:
    try:
        return int(port_name.rsplit("_", 1)[1])
    except (IndexError, ValueError):
        return 0


def is_sfp_port_opening_shape(component: Component) -> bool:
    min_x, min_y, max_x, max_y = component.bbox
    box_width = max_x - min_x + 1
    box_height = max_y - min_y + 1
    aspect = box_width / max(box_height, 1)
    return box_width >= 18 and box_height <= 14 and aspect >= 3.0


def select_sfp_from_pair(components: list[Component], port_name: str) -> Component | None:
    candidates = [component for component in components if is_sfp_port_opening_shape(component)]
    best_pair: tuple[Component, Component] | None = None
    best_score = 0.0

    for left_index, first in enumerate(candidates):
        for second in candidates[left_index + 1 :]:
            first_y = first.centroid[1]
            second_y = second.centroid[1]
            first_width = first.bbox[2] - first.bbox[0] + 1
            second_width = second.bbox[2] - second.bbox[0] + 1
            x_gap = abs(first.centroid[0] - second.centroid[0])
            y_gap = abs(first_y - second_y)
            width_ratio = max(first_width, second_width) / max(min(first_width, second_width), 1)

            if y_gap > 18 or x_gap < 30 or x_gap > 130 or width_ratio > 1.8:
                continue

            pair_score = (
                first.score
                + second.score
                + 6000.0 / (1.0 + y_gap)
                + 1500.0 / (1.0 + abs(x_gap - 75.0))
            )
            if pair_score > best_score:
                best_score = pair_score
                best_pair = (first, second)

    if not best_pair:
        return None

    ordered_left_to_right = sorted(best_pair, key=lambda component: component.centroid[0])
    port_index = parse_port_index(port_name)
    # In the current NIC mount visual, sfp_port_0 appears as the right-hand
    # opening from the center wrist-camera view; sfp_port_1 appears left-hand.
    ordered_by_port = list(reversed(ordered_left_to_right))
    return ordered_by_port[min(port_index, len(ordered_by_port) - 1)]


def component_to_dict(component: Component) -> dict[str, object]:
    return {
        "bbox": list(component.bbox),
        "area": component.area,
        "centroid": [round(component.centroid[0], 2), round(component.centroid[1], 2)],
        "mean_rgb": [round(value, 2) for value in component.mean_rgb],
        "score": round(component.score, 2),
    }


def draw_cross(draw: ImageDraw.ImageDraw, x: float, y: float, color: str) -> None:
    xi = int(round(x))
    yi = int(round(y))
    draw.line([(xi - 14, yi), (xi + 14, yi)], fill=color, width=3)
    draw.line([(xi, yi - 14), (xi, yi + 14)], fill=color, width=3)


def annotate_image(
    image_path: Path,
    output_path: Path,
    components: list[Component],
    selected: Component | None,
    title: str,
) -> None:
    image = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(image)
    font = load_font()

    for index, component in enumerate(components[:6]):
        color = "lime" if component == selected else "yellow"
        width = 4 if component == selected else 2
        draw.rectangle(component.bbox, outline=color, width=width)
        cx, cy = component.centroid
        draw_cross(draw, cx, cy, color)
        label = f"{index}: area={component.area} score={component.score:.0f}"
        draw.text((component.bbox[0], max(0, component.bbox[1] - 20)), label, fill=color, font=font)

    draw.rectangle((0, 0, image.width, 30), fill=(0, 0, 0))
    draw.text((8, 6), title, fill="white", font=font)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)


def available_frames(task_dir: Path) -> list[str]:
    frames = []
    for metadata_path in sorted(task_dir.glob("*_metadata.json")):
        frame = metadata_path.name.split("_", 1)[0]
        if frame.isdigit():
            frames.append(frame)
    return frames


def analyze_frame(
    task_dir: Path,
    output_dir: Path,
    task: dict[str, object],
    frame: str,
    min_area: int,
) -> dict[str, object]:
    port_type = str(task["port_type"])
    port_name = str(task["port_name"])
    detections: dict[str, list[Component]] = {}

    for camera in CAMERAS:
        image_path = task_dir / f"{frame}_{camera}.ppm"
        if not image_path.exists():
            detections[camera] = []
            continue
        detections[camera] = detect_ports(
            image_path=image_path,
            port_type=port_type,
            camera=camera,
            min_area=min_area,
        )

    selected_camera, selected_component, selection_reason = select_detection(
        detections,
        port_type,
        port_name,
    )
    task_output_dir = output_dir / task_dir.name

    for camera in CAMERAS:
        image_path = task_dir / f"{frame}_{camera}.ppm"
        if not image_path.exists():
            continue
        selected = selected_component if camera == selected_camera else None
        title = (
            f"{task_dir.name} | frame={frame} | {camera} | {port_type} | "
            f"selected={selected_camera if selected else '-'}"
        )
        annotate_image(
            image_path=image_path,
            output_path=task_output_dir / f"{frame}_{camera}_annotated.png",
            components=detections.get(camera, []),
            selected=selected,
            title=title,
        )

    return {
        "frame": frame,
        "selected_camera": selected_camera,
        "selected_detection": component_to_dict(selected_component) if selected_component else None,
        "selection_reason": selection_reason,
        "detections": {
            camera: [component_to_dict(component) for component in components[:8]]
            for camera, components in detections.items()
        },
    }


def compute_stability(frames: list[dict[str, object]]) -> dict[str, object]:
    points: list[tuple[str, str, tuple[float, float]]] = []
    for frame in frames:
        selected = frame.get("selected_detection")
        camera = frame.get("selected_camera")
        if not isinstance(selected, dict) or not isinstance(camera, str):
            continue
        centroid = selected.get("centroid")
        if not isinstance(centroid, list) or len(centroid) != 2:
            continue
        points.append((str(frame["frame"]), camera, (float(centroid[0]), float(centroid[1]))))

    if len(points) < 2:
        return {
            "frame_count": len(points),
            "max_centroid_drift_px": None,
            "mean_centroid_drift_px": None,
            "same_selected_camera": len({point[1] for point in points}) <= 1,
            "status": "insufficient_frames",
        }

    base_frame, base_camera, base_point = points[0]
    drifts = []
    for frame_name, camera, point in points[1:]:
        drift = math.dist(base_point, point) if camera == base_camera else None
        drifts.append(
            {
                "from_frame": base_frame,
                "to_frame": frame_name,
                "same_camera": camera == base_camera,
                "drift_px": round(drift, 3) if drift is not None else None,
            }
        )

    numeric_drifts = [item["drift_px"] for item in drifts if item["drift_px"] is not None]
    max_drift = max(numeric_drifts) if numeric_drifts else None
    mean_drift = sum(numeric_drifts) / len(numeric_drifts) if numeric_drifts else None
    same_camera = all(item["same_camera"] for item in drifts)
    stable = same_camera and max_drift is not None and max_drift <= 5.0

    return {
        "frame_count": len(points),
        "base_frame": base_frame,
        "base_camera": base_camera,
        "max_centroid_drift_px": round(max_drift, 3) if max_drift is not None else None,
        "mean_centroid_drift_px": round(mean_drift, 3) if mean_drift is not None else None,
        "same_selected_camera": same_camera,
        "drifts": drifts,
        "status": "stable" if stable else "needs_review",
    }


def process_snapshot_dir(task_dir: Path, output_dir: Path, min_area: int) -> dict[str, object]:
    metadata_path = task_dir / "00_metadata.json"
    with metadata_path.open("r", encoding="utf-8") as file:
        metadata = json.load(file)

    task = metadata["task"]
    task_output_dir = output_dir / task_dir.name
    frames = [
        analyze_frame(task_dir, output_dir, task, frame, min_area)
        for frame in available_frames(task_dir)
    ]
    primary_frame = frames[0] if frames else {}

    result = {
        "task_dir": task_dir.name,
        "task": task,
        "selected_camera": primary_frame.get("selected_camera"),
        "selected_detection": primary_frame.get("selected_detection"),
        "selection_reason": primary_frame.get("selection_reason"),
        "stability": compute_stability(frames),
        "frames": frames,
        "notes": [
            "First-pass offline heuristic detector.",
            "Selected detection fields at task level mirror the first saved frame.",
            "Use annotations and stability metrics for visual review before runtime integration.",
        ],
    }
    task_output_dir.mkdir(parents=True, exist_ok=True)
    with (task_output_dir / "detections.json").open("w", encoding="utf-8") as file:
        json.dump(result, file, indent=2)
        file.write("\n")
    return result


def build_contact_sheet(output_dir: Path, results: list[dict[str, object]]) -> None:
    annotated_paths: list[Path] = []
    for result in results:
        task_dir = output_dir / str(result["task_dir"])
        frames = result.get("frames", [])
        frame_names = [str(frame["frame"]) for frame in frames if isinstance(frame, dict)]
        annotated_paths.extend(
            task_dir / f"{frame}_{camera}_annotated.png"
            for frame in frame_names
            for camera in CAMERAS
        )

    images = [Image.open(path).convert("RGB") for path in annotated_paths if path.exists()]
    if not images:
        return

    thumb_width = 384
    thumb_height = 341
    margin = 18
    label_height = 26
    columns = 3
    rows = (len(images) + columns - 1) // columns
    sheet = Image.new(
        "RGB",
        (
            columns * thumb_width,
            rows * (thumb_height + label_height + margin),
        ),
        "white",
    )
    draw = ImageDraw.Draw(sheet)
    font = load_font(size=13)

    for index, path in enumerate(path for path in annotated_paths if path.exists()):
        image = Image.open(path).convert("RGB")
        image.thumbnail((thumb_width, thumb_height))
        col = index % columns
        row = index // columns
        x = col * thumb_width
        y = row * (thumb_height + label_height + margin)
        label = path.parent.name.replace("task_1_", "") + " / " + path.stem.replace("_annotated", "")
        draw.text((x + 4, y + 4), label[:62], fill="black", font=font)
        sheet.paste(image, (x, y + label_height))

    sheet.save(output_dir / "detection_contact_sheet.png")


def main() -> int:
    args = parse_args()
    input_dir = args.input
    output_dir = args.output if args.output is not None else default_output_dir(input_dir)

    if not input_dir.exists():
        raise SystemExit(f"Input directory not found: {input_dir}")

    task_dirs = sorted(path for path in input_dir.iterdir() if path.is_dir() and path.name.startswith("task_"))
    if not task_dirs:
        raise SystemExit(f"No task snapshot directories found under: {input_dir}")

    results = [process_snapshot_dir(task_dir, output_dir, args.min_area) for task_dir in task_dirs]
    build_contact_sheet(output_dir, results)

    summary = {
        "input_dir": str(input_dir),
        "output_dir": str(output_dir),
        "task_count": len(results),
        "results": results,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "summary.json").open("w", encoding="utf-8") as file:
        json.dump(summary, file, indent=2)
        file.write("\n")

    print(f"Wrote analysis to {output_dir}")
    for result in results:
        selected = result["selected_detection"]
        if selected:
            print(
                f"{result['task_dir']}: selected {result['selected_camera']} "
                f"centroid={selected['centroid']} bbox={selected['bbox']} "
                f"score={selected['score']}"
            )
        else:
            print(f"{result['task_dir']}: no detection")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
