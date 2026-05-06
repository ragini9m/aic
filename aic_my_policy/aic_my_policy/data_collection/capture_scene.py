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
from aic_control_interfaces.msg import ControllerState
from aic_my_policy.perception.keypoints import PORT_KEYPOINTS
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
PLUG_FRAMES_OF_INTEREST = [
    ("sfp", "cable_0/sfp_tip_link"),
    ("sc", "cable_1/sc_tip_link"),
]


class CaptureNode(Node):
    def __init__(self, out_path: Path, settle_s: float):
        super().__init__("aic_capture_scene")
        self._bridge = CvBridge()
        self._out_path = out_path
        self._settle_s = settle_s

        self._images: dict[str, np.ndarray | None] = {n: None for n in CAMERA_NAMES}
        self._camera_infos: dict[str, CameraInfo | None] = {n: None for n in CAMERA_NAMES}
        self._controller_state: ControllerState | None = None

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
        self.create_subscription(
            ControllerState,
            "/aic_controller/controller_state",
            self._on_controller_state,
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

    def _on_controller_state(self, msg: ControllerState) -> None:
        self._controller_state = msg

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

    def _controller_tcp_pose(self) -> np.ndarray | None:
        if self._controller_state is None:
            return self._lookup_tf("base_link", "gripper/tcp")
        p = self._controller_state.tcp_pose.position
        q = self._controller_state.tcp_pose.orientation
        return np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w], dtype=np.float64)

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

    def _discover_plug_frames(self) -> list[tuple[str, str]]:
        found = []
        for plug_type, frame in PLUG_FRAMES_OF_INTEREST:
            if self._lookup_tf("base_link", frame) is not None:
                found.append((plug_type, frame))
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
        plug_entries = self._discover_plug_frames()
        self.get_logger().info(f"Found {len(plug_entries)} plug-tip frame(s).")

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
        plug_poses = np.stack(
            [self._lookup_tf("base_link", f) for _, f in plug_entries]
        ) if plug_entries else np.zeros((0, 7), dtype=np.float64)
        plug_types = np.array([pt for pt, _ in plug_entries], dtype=object)
        plug_frames = np.array([f for _, f in plug_entries], dtype=object)
        tcp_pose = self._controller_tcp_pose()
        plug_tip_offsets_tcp = self._plug_offsets_in_tcp(tcp_pose, plug_poses)

        payload = {
            "port_poses": port_poses,
            "port_types": port_types,
            "port_frames": port_frames,
            "plug_poses": plug_poses,
            "plug_types": plug_types,
            "plug_frames": plug_frames,
            "tcp_pose": tcp_pose if tcp_pose is not None else np.full(7, np.nan),
            "plug_tip_offsets_tcp": plug_tip_offsets_tcp,
        }
        for name in CAMERA_NAMES:
            if self._images[name] is None or self._camera_infos[name] is None:
                continue
            info = self._camera_infos[name]
            K = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
            payload[f"image_{name}"] = self._images[name]
            payload[f"K_{name}"] = K
            payload[f"width_{name}"] = np.int32(info.width)
            payload[f"height_{name}"] = np.int32(info.height)
            if name in cam_extrinsics:
                cam_to_base = cam_extrinsics[name]
                payload[f"cam_{name}_to_base"] = cam_to_base
                labels = self._project_port_labels(
                    port_poses=port_poses,
                    port_types=port_types,
                    cam_to_base=cam_to_base,
                    K=K,
                    width=int(info.width),
                    height=int(info.height),
                )
                payload[f"port_keypoints_{name}"] = labels["keypoints"]
                payload[f"port_bboxes_{name}"] = labels["bboxes"]
                payload[f"port_visible_{name}"] = labels["visible"]

        self._out_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(self._out_path, **payload)
        self.get_logger().info(f"Wrote {self._out_path}")
        return True

    def _project_port_labels(
        self,
        port_poses: np.ndarray,
        port_types: np.ndarray,
        cam_to_base: np.ndarray,
        K: np.ndarray,
        width: int,
        height: int,
    ) -> dict[str, np.ndarray]:
        keypoints = np.full((len(port_poses), 4, 2), np.nan, dtype=np.float32)
        bboxes = np.full((len(port_poses), 4), np.nan, dtype=np.float32)
        visible = np.zeros((len(port_poses),), dtype=bool)

        for i, (pose, port_type) in enumerate(zip(port_poses, port_types)):
            uv = project_port_keypoints(pose, cam_to_base, K, str(port_type))
            if uv is None:
                continue
            finite = np.isfinite(uv).all(axis=1)
            if not finite.all():
                continue
            x1, y1 = uv.min(axis=0)
            x2, y2 = uv.max(axis=0)
            pad = max(6.0, 0.25 * max(x2 - x1, y2 - y1))
            x1 = float(np.clip(x1 - pad, 0, width - 1))
            y1 = float(np.clip(y1 - pad, 0, height - 1))
            x2 = float(np.clip(x2 + pad, 0, width - 1))
            y2 = float(np.clip(y2 + pad, 0, height - 1))
            if x2 <= x1 or y2 <= y1:
                continue
            keypoints[i] = uv.astype(np.float32)
            bboxes[i] = np.array([x1, y1, x2, y2], dtype=np.float32)
            visible[i] = True
        return {"keypoints": keypoints, "bboxes": bboxes, "visible": visible}

    def _plug_offsets_in_tcp(
        self,
        tcp_pose: np.ndarray | None,
        plug_poses: np.ndarray,
    ) -> np.ndarray:
        if tcp_pose is None or len(plug_poses) == 0:
            return np.zeros((0, 3), dtype=np.float64)
        T_base_tcp = pose7_to_T(tcp_pose)
        T_tcp_base = np.linalg.inv(T_base_tcp)
        offsets = []
        for plug_pose in plug_poses:
            p_base = np.array([plug_pose[0], plug_pose[1], plug_pose[2], 1.0])
            offsets.append((T_tcp_base @ p_base)[:3])
        return np.asarray(offsets, dtype=np.float64)


def quat_to_R(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy)],
            [    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx)],
            [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def pose7_to_T(pose7: np.ndarray) -> np.ndarray:
    tx, ty, tz, qx, qy, qz, qw = pose7
    T = np.eye(4)
    T[:3, :3] = quat_to_R(qx, qy, qz, qw)
    T[:3, 3] = (tx, ty, tz)
    return T


def project_port_keypoints(
    port_pose_base: np.ndarray,
    cam_to_base: np.ndarray,
    K: np.ndarray,
    port_type: str,
) -> np.ndarray | None:
    kpts_local = PORT_KEYPOINTS.get(port_type)
    if kpts_local is None:
        return None
    T_port_base = pose7_to_T(port_pose_base)
    T_cam_base = pose7_to_T(cam_to_base)
    T_port_cam = np.linalg.inv(T_cam_base) @ T_port_base
    kpts_h = np.concatenate([kpts_local, np.ones((kpts_local.shape[0], 1))], axis=1)
    kpts_cam = (T_port_cam @ kpts_h.T).T[:, :3]
    if (kpts_cam[:, 2] <= 1e-3).any():
        return None
    uv = (K @ kpts_cam.T).T
    return uv[:, :2] / uv[:, 2:3]


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
