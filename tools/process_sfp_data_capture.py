#!/usr/bin/env python3
"""Process one or more SFP ground-truth debug captures into training artifacts.

This is an offline helper for debug/training data preparation. It consumes
folders produced by `capture_ground_truth_debug.py`, projects ground-truth
keypoints into the saved legal camera images, exports SFP keypoint labels, and
runs the PnP sanity check against those labels.

Do not use ground-truth projections or outputs from this script inside the
submitted evaluation policy.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


CAMERAS = ("left", "center", "right")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create projection, label, and PnP artifacts from SFP debug captures."
    )
    parser.add_argument(
        "capture_dir",
        nargs="+",
        type=Path,
        help="One or more directories produced by tools/capture_ground_truth_debug.py.",
    )
    parser.add_argument(
        "--artifacts-root",
        type=Path,
        default=Path("artifacts"),
        help="Repo-local artifact root. Defaults to artifacts/.",
    )
    parser.add_argument(
        "--copy-images",
        action="store_true",
        help="Copy PPM images into each label export directory.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Re-run steps even if expected output files already exist.",
    )
    parser.add_argument(
        "--skip-pnp",
        action="store_true",
        help="Only project and export labels; skip PnP sanity checks.",
    )
    return parser.parse_args()


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def script_path(name: str) -> Path:
    return repo_root() / "tools" / name


def run_command(command: list[str]) -> subprocess.CompletedProcess[str]:
    print("+ " + " ".join(command))
    return subprocess.run(
        command,
        cwd=repo_root(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def load_font(size: int = 13) -> ImageFont.ImageFont:
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size=size)
    except OSError:
        return ImageFont.load_default()


def build_contact_sheet(paths: list[Path], output_path: Path) -> None:
    if not paths:
        return
    thumb_width = 384
    thumb_height = 341
    label_height = 28
    margin = 14
    columns = 3
    rows = (len(paths) + columns - 1) // columns
    sheet = Image.new(
        "RGB",
        (columns * thumb_width, rows * (thumb_height + label_height + margin)),
        "white",
    )
    draw = ImageDraw.Draw(sheet)
    font = load_font()
    for index, path in enumerate(paths):
        image = Image.open(path).convert("RGB")
        image.thumbnail((thumb_width, thumb_height))
        col = index % columns
        row = index // columns
        x = col * thumb_width
        y = row * (thumb_height + label_height + margin)
        draw.text((x + 4, y + 5), path.stem[:68], fill="black", font=font)
        sheet.paste(image, (x, y + label_height))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(output_path)


def frame_ids(capture_dir: Path) -> list[str]:
    return sorted(path.name.split("_", 1)[0] for path in capture_dir.glob("*_metadata.json"))


def count_labels(labels_path: Path) -> dict[str, int]:
    if not labels_path.exists():
        return {"samples": 0, "complete": 0}
    labels = json.loads(labels_path.read_text(encoding="utf-8"))["samples"]
    return {
        "samples": len(labels),
        "complete": sum(1 for label in labels if label.get("complete")),
    }


def process_capture(
    capture_dir: Path,
    artifacts_root: Path,
    copy_images: bool,
    force: bool,
    skip_pnp: bool,
) -> dict[str, object]:
    capture_dir = capture_dir.resolve()
    capture_name = capture_dir.name
    projection_dir = artifacts_root / f"ground_truth_debug_{capture_name}" / "gt_projection"
    labels_dir = artifacts_root / f"sfp_keypoint_labels_{capture_name}"
    pnp_dir = artifacts_root / f"pnp_sfp_debug_{capture_name}"
    projection_summary = projection_dir / "projection_summary.json"

    result: dict[str, object] = {
        "capture_dir": str(capture_dir),
        "projection_dir": str(projection_dir),
        "labels_dir": str(labels_dir),
        "pnp_dir": str(pnp_dir),
        "steps": [],
        "pnp": [],
    }

    if force or not projection_summary.exists():
        command = [
            sys.executable,
            str(script_path("project_ground_truth_debug.py")),
            str(capture_dir),
            "--output",
            str(projection_dir),
        ]
        completed = run_command(command)
        result["steps"].append(
            {
                "name": "project_ground_truth",
                "returncode": completed.returncode,
                "output": completed.stdout.strip(),
            }
        )
        if completed.returncode != 0:
            result["status"] = "failed_projection"
            return result
    else:
        result["steps"].append({"name": "project_ground_truth", "status": "skipped"})

    labels_path = labels_dir / "sfp_keypoint_labels.json"
    if force or not labels_path.exists():
        command = [
            sys.executable,
            str(script_path("export_sfp_keypoint_labels.py")),
            str(capture_dir),
            "--projection-summary",
            str(projection_summary),
            "--output",
            str(labels_dir),
        ]
        if copy_images:
            command.append("--copy-images")
        completed = run_command(command)
        result["steps"].append(
            {
                "name": "export_sfp_keypoint_labels",
                "returncode": completed.returncode,
                "output": completed.stdout.strip(),
            }
        )
        if completed.returncode != 0:
            result["status"] = "failed_label_export"
            return result
    else:
        result["steps"].append({"name": "export_sfp_keypoint_labels", "status": "skipped"})

    result["label_counts"] = count_labels(labels_path)

    if not skip_pnp:
        overlays: list[Path] = []
        for frame in frame_ids(capture_dir):
            for camera in CAMERAS:
                summary_path = pnp_dir / f"{frame}_{camera}_pnp_summary.json"
                if summary_path.exists() and not force:
                    result["pnp"].append(
                        {
                            "frame": frame,
                            "camera": camera,
                            "status": "skipped",
                            "summary": str(summary_path),
                        }
                    )
                    overlays.append(pnp_dir / f"{frame}_{camera}_pnp_overlay.png")
                    continue
                command = [
                    sys.executable,
                    str(script_path("pnp_sfp_debug.py")),
                    str(capture_dir),
                    "--frame",
                    frame,
                    "--camera",
                    camera,
                    "--projection-summary",
                    str(projection_summary),
                    "--output",
                    str(pnp_dir),
                ]
                completed = run_command(command)
                item = {
                    "frame": frame,
                    "camera": camera,
                    "returncode": completed.returncode,
                    "output": completed.stdout.strip(),
                }
                if summary_path.exists():
                    item["summary"] = str(summary_path)
                    overlays.append(pnp_dir / f"{frame}_{camera}_pnp_overlay.png")
                result["pnp"].append(item)

        overlays = [path for path in overlays if path.exists()]
        if overlays:
            contact_sheet = pnp_dir / "pnp_overlay_contact_sheet.png"
            build_contact_sheet(overlays, contact_sheet)
            result["pnp_contact_sheet"] = str(contact_sheet)

    result["status"] = "ok"
    return result


def main() -> int:
    args = parse_args()
    artifacts_root = args.artifacts_root
    if not artifacts_root.is_absolute():
        artifacts_root = repo_root() / artifacts_root
    artifacts_root.mkdir(parents=True, exist_ok=True)

    results = [
        process_capture(
            capture_dir,
            artifacts_root,
            copy_images=args.copy_images,
            force=args.force,
            skip_pnp=args.skip_pnp,
        )
        for capture_dir in args.capture_dir
    ]
    collection_dir = artifacts_root / "sfp_data_collection"
    collection_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    summary = {
        "created_at": datetime.now().isoformat(),
        "warning": "Debug/training artifact summary. Do not use ground-truth labels in evaluation.",
        "results": results,
    }
    summary_path = collection_dir / f"{timestamp}_process_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (collection_dir / "latest_process_summary.json").write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8",
    )
    print(summary_path)
    failed = [item for item in results if item.get("status") != "ok"]
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
