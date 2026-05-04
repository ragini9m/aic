#!/usr/bin/env python3
"""Debug-only capture of camera images plus ground-truth target TF frames.

This is for offline verification with `ground_truth:=true`. Do not use this
script, its topics, or its frame outputs in the submitted policy.
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from capture_camera_images import CAMERAS, camera_info_metadata, image_metadata, save_image


DEFAULT_TARGETS = (
    "task_board/nic_card_mount_0/sfp_port_0_link",
    "task_board/nic_card_mount_0/sfp_port_0_link_entrance",
    "task_board/nic_card_mount_0/sfp_port_1_link",
    "task_board/nic_card_mount_0/sfp_port_1_link_entrance",
    "task_board/nic_card_mount_1/sfp_port_0_link",
    "task_board/nic_card_mount_1/sfp_port_0_link_entrance",
    "task_board/nic_card_mount_1/sfp_port_1_link",
    "task_board/nic_card_mount_1/sfp_port_1_link_entrance",
    "task_board/sc_port_1/sc_port_base_link",
    "task_board/sc_port_1/sc_port_base_link_entrance",
)

DEFAULT_CAMERA_FRAMES = (
    "left_camera/optical",
    "center_camera/optical",
    "right_camera/optical",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture legal camera images plus debug-only ground-truth TF."
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/home/optimus/ws_aic/ground_truth_debug"),
        help="Directory where a timestamped debug capture folder will be created.",
    )
    parser.add_argument(
        "--label",
        default="gt_debug",
        help="Label to include in the capture folder name.",
    )
    parser.add_argument(
        "--target-frame",
        action="append",
        default=[],
        help=(
            "Target frame to capture relative to base_link. Can be repeated. "
            "Defaults cover sample SFP and SC target frames."
        ),
    )
    parser.add_argument(
        "--camera-frame",
        action="append",
        default=[],
        help=(
            "Camera optical frame to capture relative to base_link. Can be repeated. "
            "Defaults to left/center/right camera optical frames."
        ),
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="Number of frame sets to save.",
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
        help="Seconds to wait for cameras and TF frames.",
    )
    return parser.parse_args()


def transform_metadata(transform: TransformStamped) -> dict[str, object]:
    t = transform.transform.translation
    q = transform.transform.rotation
    return {
        "header": {
            "frame_id": transform.header.frame_id,
            "stamp": {
                "sec": int(transform.header.stamp.sec),
                "nanosec": int(transform.header.stamp.nanosec),
            },
        },
        "child_frame_id": transform.child_frame_id,
        "translation": {"x": t.x, "y": t.y, "z": t.z},
        "rotation": {"x": q.x, "y": q.y, "z": q.z, "w": q.w},
    }


class GroundTruthDebugCapture(Node):
    def __init__(
        self,
        output_root: Path,
        label: str,
        target_frames: list[str],
        camera_frames: list[str],
    ):
        super().__init__("aic_ground_truth_debug_capture")
        run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_label = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in label)
        self.output_dir = output_root / f"{run_id}_{safe_label}"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.target_frames = target_frames
        self.camera_frames = camera_frames
        self.images: dict[str, Image] = {}
        self.camera_infos: dict[str, CameraInfo] = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        self.get_logger().info(f"Saving debug capture to {self.output_dir}")

    def _image_callback(self, camera: str, msg: Image) -> None:
        self.images[camera] = msg

    def _camera_info_callback(self, camera: str, msg: CameraInfo) -> None:
        self.camera_infos[camera] = msg

    def cameras_ready(self) -> bool:
        return all(camera in self.images for camera in CAMERAS) and all(
            camera in self.camera_infos for camera in CAMERAS
        )

    def lookup_frames(self, frames: list[str]) -> tuple[dict[str, object], list[str]]:
        transforms = {}
        missing = []
        for frame in frames:
            try:
                transform = self.tf_buffer.lookup_transform("base_link", frame, Time())
            except TransformException:
                missing.append(frame)
                continue
            transforms[frame] = transform_metadata(transform)
        return transforms, missing

    def save_frame_set(self, index: int) -> None:
        prefix = f"{index:02d}"
        target_transforms, missing_target_tf = self.lookup_frames(self.target_frames)
        camera_transforms, missing_camera_tf = self.lookup_frames(self.camera_frames)
        metadata = {
            "capture_index": index,
            "capture_time": datetime.now().isoformat(),
            "warning": "Debug-only ground-truth capture. Do not use in evaluation policy.",
            "images": {},
            "cameras": {},
            "ground_truth_transforms_base_link": target_transforms,
            "camera_transforms_base_link": camera_transforms,
            "missing_ground_truth_frames": missing_target_tf,
            "missing_camera_frames": missing_camera_tf,
        }

        for camera in CAMERAS:
            image = self.images[camera]
            image_path = save_image(image, self.output_dir / f"{prefix}_{camera}")
            metadata["images"][camera] = {
                **image_metadata(image),
                "path": str(image_path),
            }
            metadata["cameras"][camera] = camera_info_metadata(
                self.camera_infos.get(camera)
            )

        metadata_path = self.output_dir / f"{prefix}_metadata.json"
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        self.get_logger().info(
            f"Saved frame set {index + 1}; "
            f"target_tf_found={len(target_transforms)} "
            f"target_tf_missing={len(missing_target_tf)} "
            f"camera_tf_found={len(camera_transforms)} "
            f"camera_tf_missing={len(missing_camera_tf)}"
        )


def main() -> int:
    args = parse_args()
    target_frames = args.target_frame or list(DEFAULT_TARGETS)
    camera_frames = args.camera_frame or list(DEFAULT_CAMERA_FRAMES)

    rclpy.init()
    node = GroundTruthDebugCapture(args.output, args.label, target_frames, camera_frames)

    try:
        deadline = node.get_clock().now().nanoseconds / 1e9 + args.timeout
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            target_transforms, missing_targets = node.lookup_frames(target_frames)
            camera_transforms, missing_cameras = node.lookup_frames(camera_frames)
            if node.cameras_ready() and target_transforms and camera_transforms:
                break
            if node.get_clock().now().nanoseconds / 1e9 > deadline:
                missing_images = [camera for camera in CAMERAS if camera not in node.images]
                missing_infos = [
                    camera for camera in CAMERAS if camera not in node.camera_infos
                ]
                raise TimeoutError(
                    f"Timed out waiting for debug data. "
                    f"missing_images={missing_images}, "
                    f"missing_camera_info={missing_infos}, "
                    f"missing_target_tf={missing_targets}, "
                    f"missing_camera_tf={missing_cameras}"
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
