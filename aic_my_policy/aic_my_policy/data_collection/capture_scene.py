"""One-shot capture ROS node.

Runs inside a live `aic_bringup` session (with `ground_truth:=true` and
`start_aic_engine:=false`). Waits for cameras and TF to arrive, then
writes a single `.npz` containing the three wrist-camera images, their
intrinsics, per-camera static extrinsics to base_link, and the
ground-truth 6-DoF poses of every port currently present in TF. Exits.

Invoke this after each randomized launch; an outer shell loop cycles
randomization + capture to build a dataset.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

CAMERA_NAMES = ("left", "center", "right")
PORT_FRAMES_OF_INTEREST = [
    # (port_type, tf_frame_suffix template)
    ("sfp", "task_board/nic_card_mount_{rail}/sfp_port_{port}_link"),
    ("sc",  "task_board/sc_port_{rail}/sc_port_base_link"),
]


class CaptureNode(Node):
    def __init__(self, out_path: Path, settle_s: float):
        super().__init__("aic_capture_scene")
        self._bridge = CvBridge()
        self._out_path = out_path
        self._settle_s = settle_s

        self._images: dict[str, np.ndarray | None] = {n: None for n in CAMERA_NAMES}
        self._camera_infos: dict[str, CameraInfo | None] = {n: None for n in CAMERA_NAMES}

        for name in CAMERA_NAMES:
            self.create_subscription(
                Image,
                f"/{name}_camera/image",
                lambda msg, n=name: self._on_image(n, msg),
                10,
            )
            self.create_subscription(
                CameraInfo,
                f"/{name}_camera/camera_info",
                lambda msg, n=name: self._on_camera_info(n, msg),
                10,
            )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _on_image(self, name: str, msg: Image) -> None:
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as ex:
            self.get_logger().warn(f"cv_bridge failed for {name}: {ex}")
            return
        self._images[name] = img

    def _on_camera_info(self, name: str, msg: CameraInfo) -> None:
        self._camera_infos[name] = msg

    def _wait_ready(self, timeout_s: float = 30.0) -> bool:
        start = time.time()
        while time.time() - start < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            have_all_images = all(v is not None for v in self._images.values())
            have_all_infos = all(v is not None for v in self._camera_infos.values())
            if have_all_images and have_all_infos:
                return True
        return False

    def _settle(self) -> None:
        end = time.time() + self._settle_s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _lookup_tf(self, target_frame: str, source_frame: str) -> np.ndarray | None:
        try:
            t = self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except TransformException:
            return None
        tr = t.transform.translation
        r = t.transform.rotation
        return np.array([tr.x, tr.y, tr.z, r.x, r.y, r.z, r.w], dtype=np.float64)

    def _discover_port_frames(self) -> list[tuple[str, str]]:
        """Return (port_type, tf_frame_name) for every port currently in TF."""
        found = []
        for port_type, tmpl in PORT_FRAMES_OF_INTEREST:
            if port_type == "sfp":
                for rail in range(5):
                    for port in range(2):
                        frame = tmpl.format(rail=rail, port=port)
                        if self._lookup_tf("base_link", frame) is not None:
                            found.append((port_type, frame))
            elif port_type == "sc":
                for rail in range(2):
                    frame = tmpl.format(rail=rail, port=0)
                    if self._lookup_tf("base_link", frame) is not None:
                        found.append((port_type, frame))
        return found

    def capture_and_save(self) -> bool:
        self.get_logger().info("Waiting for cameras + camera_info ...")
        if not self._wait_ready():
            self.get_logger().error("Timed out waiting for image/camera_info topics.")
            return False
        self.get_logger().info(f"Settling {self._settle_s}s for TF to populate.")
        self._settle()

        port_entries = self._discover_port_frames()
        if not port_entries:
            self.get_logger().error("No known port frames appeared in TF.")
            return False
        self.get_logger().info(f"Found {len(port_entries)} port frame(s).")

        cam_extrinsics: dict[str, np.ndarray] = {}
        for name in CAMERA_NAMES:
            optical = f"{name}_camera/optical"
            extr = self._lookup_tf("base_link", optical)
            if extr is None:
                self.get_logger().warn(f"No static TF for {optical}; skipping.")
                continue
            cam_extrinsics[name] = extr

        if len(cam_extrinsics) == 0:
            self.get_logger().error("No camera extrinsics resolved.")
            return False

        port_poses = np.stack(
            [self._lookup_tf("base_link", f) for _, f in port_entries]
        )
        port_types = np.array([pt for pt, _ in port_entries], dtype=object)
        port_frames = np.array([f for _, f in port_entries], dtype=object)

        payload = {"port_poses": port_poses, "port_types": port_types, "port_frames": port_frames}
        for name in CAMERA_NAMES:
            if self._images[name] is None or self._camera_infos[name] is None:
                continue
            info = self._camera_infos[name]
            payload[f"image_{name}"] = self._images[name]
            payload[f"K_{name}"] = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
            payload[f"width_{name}"] = np.int32(info.width)
            payload[f"height_{name}"] = np.int32(info.height)
            if name in cam_extrinsics:
                payload[f"cam_{name}_to_base"] = cam_extrinsics[name]

        self._out_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(self._out_path, **payload)
        self.get_logger().info(f"Wrote {self._out_path}")
        return True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True, help="output .npz path")
    parser.add_argument("--settle", type=float, default=2.0, help="seconds to settle before capture")
    args = parser.parse_args(argv)

    rclpy.init()
    try:
        node = CaptureNode(Path(args.out), settle_s=args.settle)
        ok = node.capture_and_save()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
