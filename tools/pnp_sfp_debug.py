#!/usr/bin/env python3
"""Debug PnP/model-fitting for an SFP port from image keypoints.

This is an offline/debug tool. It can use ground-truth-projected keypoints to
validate the PnP math, or a user-supplied keypoint JSON to test detector output.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from project_ground_truth_debug import CAMERA_TO_FRAME, point_from_transform, quat_to_matrix


SFP_PORT_SPACING_M = 0.01295 - (-0.01025)
SFP_ENTRANCE_OFFSET_M = 0.0458


OBJECT_POINTS_PORT0 = {
    # Object coordinates are expressed in sfp_port_0_link frame.
    # SFP port 1 has the same orientation and is 23.2 mm left of port 0
    # in the NIC-card model.
    "sfp_port_0_link": (0.0, 0.0, 0.0),
    "sfp_port_0_link_entrance": (0.0, 0.0, -SFP_ENTRANCE_OFFSET_M),
    "sfp_port_1_link": (-SFP_PORT_SPACING_M, 0.0, 0.0),
    "sfp_port_1_link_entrance": (-SFP_PORT_SPACING_M, 0.0, -SFP_ENTRANCE_OFFSET_M),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Solve SFP PnP from debug keypoints.")
    parser.add_argument(
        "capture_dir",
        type=Path,
        help="Directory produced by capture_ground_truth_debug.py.",
    )
    parser.add_argument("--frame", default="00")
    parser.add_argument("--camera", default="center", choices=tuple(CAMERA_TO_FRAME.keys()))
    parser.add_argument(
        "--keypoints-json",
        type=Path,
        default=None,
        help=(
            "Optional keypoint JSON mapping object point names to [u, v]. "
            "If omitted, ground-truth projections are used as a math sanity check."
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output directory. Defaults to artifacts/pnp_sfp_debug_<capture-dir-name>.",
    )
    parser.add_argument(
        "--projection-summary",
        type=Path,
        default=None,
        help=(
            "projection_summary.json from project_ground_truth_debug.py. "
            "Required when --keypoints-json is omitted unless it can be inferred."
        ),
    )
    return parser.parse_args()


def load_font(size: int = 16) -> ImageFont.ImageFont:
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size=size)
    except OSError:
        return ImageFont.load_default()


def default_output_dir(capture_dir: Path) -> Path:
    return Path("artifacts") / f"pnp_sfp_debug_{capture_dir.name}"


def target_suffix(frame_name: str) -> str:
    return frame_name.rsplit("/", 1)[-1]


def load_metadata(capture_dir: Path, frame: str) -> dict[str, object]:
    return json.loads((capture_dir / f"{frame}_metadata.json").read_text(encoding="utf-8"))


def inferred_projection_summary(capture_dir: Path) -> Path:
    return (
        Path("artifacts")
        / f"ground_truth_debug_{capture_dir.name}"
        / "gt_projection"
        / "projection_summary.json"
    )


def load_gt_projection(projection_path: Path, frame: str, camera: str) -> dict[str, list[dict[str, object]]]:
    if not projection_path.exists():
        raise SystemExit(f"Projection summary not found: {projection_path}")
    summary = json.loads(projection_path.read_text(encoding="utf-8"))
    for frame_summary in summary["frames"]:
        if frame_summary["frame"] == frame:
            return frame_summary["projections"][camera]
    raise SystemExit(f"Frame {frame!r} not found in {projection_path}")


def keypoints_from_gt_projection(
    projection_results: list[dict[str, object]],
) -> dict[str, tuple[float, float]]:
    keypoints = {}
    for item in projection_results:
        if not item.get("visible"):
            continue
        suffix = target_suffix(str(item["target_frame"]))
        if suffix in OBJECT_POINTS_PORT0:
            keypoints[suffix] = (float(item["u"]), float(item["v"]))
    return keypoints


def load_keypoints(path: Path) -> dict[str, tuple[float, float]]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    return {name: (float(value[0]), float(value[1])) for name, value in raw.items()}


def camera_matrix(metadata: dict[str, object], camera: str) -> np.ndarray:
    k = metadata["cameras"][camera]["k"]
    return np.array(
        [[k[0], 0.0, k[2]], [0.0, k[4], k[5]], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def solve_pnp(
    keypoints: dict[str, tuple[float, float]],
    camera_k: np.ndarray,
) -> tuple[bool, np.ndarray, np.ndarray, list[str], np.ndarray]:
    names = [name for name in OBJECT_POINTS_PORT0 if name in keypoints]
    if len(names) < 4:
        raise SystemExit(
            f"Need at least 4 keypoints for this PnP prototype, got {len(names)}: {names}"
        )
    object_points = np.array([OBJECT_POINTS_PORT0[name] for name in names], dtype=np.float64)
    image_points = np.array([keypoints[name] for name in names], dtype=np.float64)
    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_k,
        None,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    return success, rvec, tvec, names, object_points


def pose_matrix_from_gt_port0(metadata: dict[str, object], camera: str) -> np.ndarray | None:
    target_name = "task_board/nic_card_mount_0/sfp_port_0_link"
    camera_frame = CAMERA_TO_FRAME[camera]
    target_tf = metadata["ground_truth_transforms_base_link"].get(target_name)
    camera_tf = metadata["camera_transforms_base_link"].get(camera_frame)
    if target_tf is None or camera_tf is None:
        return None

    rotation_base_target = np.array(quat_to_matrix(target_tf["rotation"]), dtype=np.float64)
    translation_base_target = np.array(point_from_transform(target_tf), dtype=np.float64).reshape(3, 1)
    rotation_base_camera = np.array(quat_to_matrix(camera_tf["rotation"]), dtype=np.float64)
    translation_base_camera = np.array(point_from_transform(camera_tf), dtype=np.float64).reshape(3, 1)
    rotation_camera_base = rotation_base_camera.T
    rotation_camera_target = rotation_camera_base @ rotation_base_target
    translation_camera_target = rotation_camera_base @ (
        translation_base_target - translation_base_camera
    )
    matrix = np.eye(4)
    matrix[:3, :3] = rotation_camera_target
    matrix[:3, 3] = translation_camera_target.reshape(3)
    return matrix


def pose_matrix_from_pnp(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    rotation, _ = cv2.Rodrigues(rvec)
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = tvec.reshape(3)
    return matrix


def rotation_error_deg(a: np.ndarray, b: np.ndarray) -> float:
    delta = a[:3, :3] @ b[:3, :3].T
    value = (np.trace(delta) - 1.0) * 0.5
    value = float(np.clip(value, -1.0, 1.0))
    return float(np.degrees(np.arccos(value)))


def reprojection_errors(
    object_points: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_k: np.ndarray,
    names: list[str],
) -> dict[str, object]:
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_k, None)
    projected = projected.reshape(-1, 2)
    errors = {}
    for name, measured, predicted in zip(names, image_points, projected):
        diff = measured - predicted
        errors[name] = {
            "measured_uv": [round(float(measured[0]), 2), round(float(measured[1]), 2)],
            "projected_uv": [round(float(predicted[0]), 2), round(float(predicted[1]), 2)],
            "error_px": round(float(np.linalg.norm(diff)), 3),
        }
    return errors


def draw_overlay(
    image_path: Path,
    output_path: Path,
    keypoints: dict[str, tuple[float, float]],
    object_points: np.ndarray,
    names: list[str],
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_k: np.ndarray,
) -> None:
    image = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(image)
    font = load_font()
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_k, None)
    projected = projected.reshape(-1, 2)

    for name, predicted in zip(names, projected):
        measured = keypoints[name]
        draw.ellipse(
            (measured[0] - 8, measured[1] - 8, measured[0] + 8, measured[1] + 8),
            outline="lime",
            width=4,
        )
        draw.line(
            [(predicted[0] - 10, predicted[1]), (predicted[0] + 10, predicted[1])],
            fill="magenta",
            width=3,
        )
        draw.line(
            [(predicted[0], predicted[1] - 10), (predicted[0], predicted[1] + 10)],
            fill="magenta",
            width=3,
        )
        draw.text(
            (int(measured[0]) + 10, int(measured[1]) + 10),
            name,
            fill="white",
            font=font,
        )

    draw.rectangle((0, 0, image.width, 34), fill=(0, 0, 0))
    draw.text(
        (8, 7),
        "PnP SFP debug: green=input keypoint, magenta=reprojection",
        fill="white",
        font=font,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)


def main() -> int:
    args = parse_args()
    output_dir = args.output or default_output_dir(args.capture_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    metadata = load_metadata(args.capture_dir, args.frame)
    camera_k = camera_matrix(metadata, args.camera)

    if args.keypoints_json:
        keypoints = load_keypoints(args.keypoints_json)
        keypoint_source = str(args.keypoints_json)
    else:
        projection_summary = args.projection_summary or inferred_projection_summary(args.capture_dir)
        keypoints = keypoints_from_gt_projection(
            load_gt_projection(projection_summary, args.frame, args.camera)
        )
        keypoint_source = "ground_truth_projection"

    success, rvec, tvec, names, object_points = solve_pnp(keypoints, camera_k)
    image_points = np.array([keypoints[name] for name in names], dtype=np.float64)
    pnp_pose = pose_matrix_from_pnp(rvec, tvec)
    gt_pose = pose_matrix_from_gt_port0(metadata, args.camera)

    comparison = None
    if gt_pose is not None:
        translation_error_m = float(np.linalg.norm(pnp_pose[:3, 3] - gt_pose[:3, 3]))
        comparison = {
            "translation_error_m": round(translation_error_m, 6),
            "translation_error_mm": round(translation_error_m * 1000.0, 3),
            "rotation_error_deg": round(rotation_error_deg(pnp_pose, gt_pose), 3),
            "pnp_translation_camera_m": [round(float(v), 6) for v in pnp_pose[:3, 3]],
            "gt_translation_camera_m": [round(float(v), 6) for v in gt_pose[:3, 3]],
        }

    errors = reprojection_errors(object_points, image_points, rvec, tvec, camera_k, names)
    image_path = args.capture_dir / f"{args.frame}_{args.camera}.ppm"
    overlay_path = output_dir / f"{args.frame}_{args.camera}_pnp_overlay.png"
    draw_overlay(image_path, overlay_path, keypoints, object_points, names, rvec, tvec, camera_k)

    summary = {
        "capture_dir": str(args.capture_dir),
        "frame": args.frame,
        "camera": args.camera,
        "keypoint_source": keypoint_source,
        "success": bool(success),
        "used_keypoints": names,
        "object_points_port0_frame_m": {
            name: list(OBJECT_POINTS_PORT0[name]) for name in names
        },
        "image_keypoints_px": {
            name: [round(keypoints[name][0], 2), round(keypoints[name][1], 2)]
            for name in names
        },
        "reprojection_errors": errors,
        "comparison_to_ground_truth_pose": comparison,
        "overlay": str(overlay_path),
    }
    summary_path = output_dir / f"{args.frame}_{args.camera}_pnp_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(summary_path)
    if comparison:
        print(comparison)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
