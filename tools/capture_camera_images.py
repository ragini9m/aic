#!/usr/bin/env python3
"""Save current AIC wrist-camera images without commanding the robot."""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


CAMERAS = ("left", "center", "right")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture left/center/right AIC camera images to PPM files."
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/home/optimus/ws_aic/camera_captures"),
        help="Directory where a timestamped capture folder will be created.",
    )
    parser.add_argument(
        "--label",
        default="manual",
        help="Label to include in the capture folder name.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=3,
        help="Number of synchronized-ish frame sets to save.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.25,
        help="Seconds between saved frame sets.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for all camera images and camera info.",
    )
    return parser.parse_args()


def image_bytes_rgb(image: Image) -> tuple[bytes, str]:
    raw = bytes(image.data)
    encoding = image.encoding.lower()
    pixel_count = int(image.width) * int(image.height)

    if encoding == "rgb8":
        return raw, "ppm"

    if encoding == "bgr8":
        converted = bytearray(len(raw))
        for i in range(0, len(raw), 3):
            converted[i] = raw[i + 2]
            converted[i + 1] = raw[i + 1]
            converted[i + 2] = raw[i]
        return bytes(converted), "ppm"

    if encoding == "rgba8":
        converted = bytearray(pixel_count * 3)
        for src, dst in zip(range(0, len(raw), 4), range(0, len(converted), 3)):
            converted[dst : dst + 3] = raw[src : src + 3]
        return bytes(converted), "ppm"

    if encoding == "bgra8":
        converted = bytearray(pixel_count * 3)
        for src, dst in zip(range(0, len(raw), 4), range(0, len(converted), 3)):
            converted[dst] = raw[src + 2]
            converted[dst + 1] = raw[src + 1]
            converted[dst + 2] = raw[src]
        return bytes(converted), "ppm"

    if encoding == "mono8":
        return raw, "pgm"

    raise ValueError(f"Unsupported image encoding: {image.encoding}")


def save_image(image: Image, path_without_suffix: Path) -> Path:
    payload, suffix = image_bytes_rgb(image)
    path = path_without_suffix.with_suffix(f".{suffix}")
    magic = b"P6" if suffix == "ppm" else b"P5"
    header = magic + f"\n{image.width} {image.height}\n255\n".encode("ascii")
    path.write_bytes(header + payload)
    return path


def camera_info_metadata(camera_info: CameraInfo | None) -> dict[str, object] | None:
    if camera_info is None:
        return None
    return {
        "width": int(camera_info.width),
        "height": int(camera_info.height),
        "distortion_model": camera_info.distortion_model,
        "d": list(camera_info.d),
        "k": list(camera_info.k),
        "r": list(camera_info.r),
        "p": list(camera_info.p),
    }


def image_metadata(image: Image) -> dict[str, object]:
    return {
        "topic_stamp": {
            "sec": int(image.header.stamp.sec),
            "nanosec": int(image.header.stamp.nanosec),
        },
        "frame_id": image.header.frame_id,
        "width": int(image.width),
        "height": int(image.height),
        "encoding": image.encoding,
        "step": int(image.step),
    }


class CameraCapture(Node):
    def __init__(self, output_root: Path, label: str, count: int, interval: float):
        super().__init__("aic_camera_capture")
        run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_label = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in label)
        self.output_dir = output_root / f"{run_id}_{safe_label}"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.count = count
        self.interval = interval
        self.images: dict[str, Image] = {}
        self.camera_infos: dict[str, CameraInfo] = {}

        for camera in CAMERAS:
            self.create_subscription(
                Image,
                f"/{camera}_camera/image",
                lambda msg, camera=camera: self._image_callback(camera, msg),
                10,
            )
            self.create_subscription(
                CameraInfo,
                f"/{camera}_camera/camera_info",
                lambda msg, camera=camera: self._camera_info_callback(camera, msg),
                10,
            )

        self.get_logger().info(f"Saving camera captures to {self.output_dir}")

    def _image_callback(self, camera: str, msg: Image) -> None:
        self.images[camera] = msg

    def _camera_info_callback(self, camera: str, msg: CameraInfo) -> None:
        self.camera_infos[camera] = msg

    def ready(self) -> bool:
        return all(camera in self.images for camera in CAMERAS) and all(
            camera in self.camera_infos for camera in CAMERAS
        )

    def save_frame_set(self, index: int) -> None:
        prefix = f"{index:02d}"
        image_paths = {}
        metadata = {
            "capture_index": index,
            "capture_time": datetime.now().isoformat(),
            "images": {},
            "cameras": {},
        }

        for camera in CAMERAS:
            image = self.images[camera]
            image_path = save_image(image, self.output_dir / f"{prefix}_{camera}")
            image_paths[camera] = str(image_path)
            metadata["images"][camera] = {
                **image_metadata(image),
                "path": str(image_path),
            }
            metadata["cameras"][camera] = camera_info_metadata(
                self.camera_infos.get(camera)
            )

        metadata["image_paths"] = image_paths
        metadata_path = self.output_dir / f"{prefix}_metadata.json"
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        self.get_logger().info(f"Saved frame set {index + 1}/{self.count}")


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = CameraCapture(args.output, args.label, args.count, args.interval)

    try:
        deadline = node.get_clock().now().nanoseconds / 1e9 + args.timeout
        while rclpy.ok() and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds / 1e9 > deadline:
                missing_images = [camera for camera in CAMERAS if camera not in node.images]
                missing_infos = [
                    camera for camera in CAMERAS if camera not in node.camera_infos
                ]
                raise TimeoutError(
                    f"Timed out waiting for cameras. "
                    f"missing_images={missing_images}, missing_camera_info={missing_infos}"
                )

        for index in range(args.count):
            until = node.get_clock().now().nanoseconds / 1e9 + args.interval
            while rclpy.ok() and node.get_clock().now().nanoseconds / 1e9 < until:
                rclpy.spin_once(node, timeout_sec=0.05)
            node.save_frame_set(index)

        print(node.output_dir)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
