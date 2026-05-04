#!/usr/bin/env python3
"""Build a contact sheet for manual AIC camera capture folders."""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


CAMERAS = ("left", "center", "right")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a PNG contact sheet from manual camera capture folders."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("artifacts/manual_camera_captures"),
        help="Directory containing capture subdirectories.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/manual_camera_captures/contact_sheet.png"),
        help="Output PNG path.",
    )
    return parser.parse_args()


def load_font(size: int = 13) -> ImageFont.ImageFont:
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size=size)
    except OSError:
        return ImageFont.load_default()


def image_paths(root: Path) -> list[Path]:
    paths: list[Path] = []
    for capture_dir in sorted(path for path in root.iterdir() if path.is_dir()):
        frames = sorted(
            {
                path.name.split("_", 1)[0]
                for path in capture_dir.glob("*_center.ppm")
                if path.name[:2].isdigit()
            }
        )
        for frame in frames:
            paths.extend(
                capture_dir / f"{frame}_{camera}.ppm"
                for camera in CAMERAS
                if (capture_dir / f"{frame}_{camera}.ppm").exists()
            )
    return paths


def main() -> int:
    args = parse_args()
    paths = image_paths(args.input)
    if not paths:
        raise SystemExit(f"No camera images found under {args.input}")

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
        label = f"{path.parent.name} / {path.stem}"
        draw.text((x + 4, y + 5), label[:68], fill="black", font=font)
        sheet.paste(image, (x, y + label_height))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(args.output)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
