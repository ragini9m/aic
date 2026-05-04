"""Vision-based PortPoseEstimator.

Subscribes to the center wrist camera, runs the trained keypoint+PnP
pipeline, and reports the target port pose in base_link. The plug-tip
pose is still obtained from TF — the grasped plug is rigidly attached
to the gripper at a static offset determined at task start (via TF
while `ground_truth:=true`). At true eval time the plug offset must
come from a calibration step instead.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException

from aic_my_policy.estimators.base import PortPoseEstimator


class VisionPortPoseEstimator(PortPoseEstimator):
    BASE_FRAME = "base_link"
    CAMERA = "center"

    def __init__(
        self,
        parent_node,
        sfp_weights: str | Path,
        sc_weights: str | Path,
    ):
        # Defer heavy imports so ROS lifecycle bring-up stays fast even if
        # torch / cv2 aren't actually exercised (e.g. when running tests).
        from aic_my_policy.perception.infer import PortKeypointInference

        self._parent_node = parent_node
        self._logger = parent_node.get_logger()
        self._tf_buffer = parent_node._tf_buffer
        self._clock = parent_node.get_clock()
        self._bridge = CvBridge()

        def _param(name: str, default):
            if not parent_node.has_parameter(name):
                parent_node.declare_parameter(name, default)
            return parent_node.get_parameter(name).value

        self._inference = {
            "sfp": PortKeypointInference(sfp_weights, port_type="sfp"),
            "sc":  PortKeypointInference(sc_weights,  port_type="sc"),
        }
        self._min_keypoint_confidence = float(_param("vision_min_keypoint_confidence", 0.05))
        self._max_reprojection_error_px = float(_param("vision_max_reprojection_error_px", 25.0))
        self._max_stale_age_s = float(_param("vision_max_stale_age_s", 2.0))
        self._debug_gt = bool(_param("vision_debug_gt", False))
        self._gt_error_gate = bool(_param("vision_gt_error_gate", False))
        self._max_gt_error_m = float(_param("vision_max_gt_error_m", 0.02))
        self._use_configured_plug_offsets = bool(
            _param("vision_use_configured_plug_offsets", False)
        )
        self._allow_tcp_plug_fallback = bool(
            _param("vision_allow_tcp_plug_fallback", True)
        )
        self._configured_plug_offsets_tcp = {
            "sfp": tuple(float(v) for v in _param("sfp_plug_tip_offset_tcp_xyz", [0.0, 0.0, 0.0])),
            "sc": tuple(float(v) for v in _param("sc_plug_tip_offset_tcp_xyz", [0.0, 0.0, 0.0])),
        }
        self._port_type: Optional[str] = None
        self._plug_frame: Optional[str] = None
        self._last_port_pose = None
        self._last_port_stamp_ns: Optional[int] = None
        self._plug_tip_offset_tcp: Optional[tuple[float, float, float]] = None

    # --- lifecycle --------------------------------------------------------

    def initialize(self, task: Task) -> bool:
        self._last_port_pose = None
        self._smoothed_port_xyz = None   # EMA-smoothed position
        self._last_port_stamp_ns = None
        self._plug_tip_offset_tcp = None
        self._port_type = task.port_type
        self._gt_port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        if self._port_type not in self._inference:
            self._logger.error(f"No weights loaded for port_type={self._port_type!r}")
            return False
        self._plug_frame = f"{task.cable_name}/{task.plug_name}_link"
        if self._use_configured_plug_offsets:
            self._plug_tip_offset_tcp = self._configured_plug_offsets_tcp.get(self._port_type)
        self._logger.info(
            f"[vision estimator] port_type={self._port_type} "
            f"plug_frame={self._plug_frame}"
        )
        return True

    # --- queries ----------------------------------------------------------

    def get_port_pose(self, observation: Optional[Observation]) -> Optional[Pose]:
        if observation is None or self._port_type is None:
            self._logger.warn("[vision] observation or port_type is None", throttle_duration_sec=2.0)
            return None
        img_msg = getattr(observation, f"{self.CAMERA}_image", None)
        info_msg = getattr(observation, f"{self.CAMERA}_camera_info", None)
        if img_msg is None or info_msg is None:
            self._logger.warn(f"[vision] missing image/camera_info for camera='{self.CAMERA}'", throttle_duration_sec=2.0)
            return None
        try:
            img = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
        except Exception as ex:
            self._logger.warn(f"[vision] cv_bridge failed: {ex}", throttle_duration_sec=2.0)
            return None

        K = np.asarray(info_msg.k, dtype=np.float64).reshape(3, 3)
        cam_frame = info_msg.header.frame_id
        result = self._inference[self._port_type](img, K)

        if result is None:
            self._logger.warn("[vision] PnP failed — bad keypoints", throttle_duration_sec=2.0)
        else:
            min_score = float(np.min(result.get("keypoint_scores", [0.0])))
            reproj = float(result.get("reprojection_error_px", float("inf")))
            self._logger.info(
                f"[vision] scores={np.array2string(np.array(result.get('keypoint_scores', [])), precision=2)} "
                f"reproj={reproj:.1f}px",
                throttle_duration_sec=1.0,
            )
            if min_score < self._min_keypoint_confidence:
                self._logger.warn(
                    f"[vision] estimate rejected: keypoint confidence {min_score:.3f} "
                    f"< {self._min_keypoint_confidence:.3f}",
                    throttle_duration_sec=2.0,
                )
                return self._fresh_last_port_pose()
            if reproj > self._max_reprojection_error_px:
                self._logger.warn(
                    f"[vision] estimate rejected: reprojection error {reproj:.1f}px "
                    f"> {self._max_reprojection_error_px:.1f}px",
                    throttle_duration_sec=2.0,
                )
                return self._fresh_last_port_pose()
            T_cam_base = self._lookup_transform_mat(self.BASE_FRAME, cam_frame)
            if T_cam_base is None:
                self._logger.warn(f"[vision] TF lookup failed: {self.BASE_FRAME} <- {cam_frame}", throttle_duration_sec=2.0)
            else:
                selected = self._select_pnp_candidate(result, T_cam_base)
                if selected is not None:
                    T_port_base, candidate = selected
                    alpha = 0.3  # EMA weight for new measurement (lower = smoother)
                    px, py, pz = T_port_base[0, 3], T_port_base[1, 3], T_port_base[2, 3]
                    if self._smoothed_port_xyz is None:
                        self._smoothed_port_xyz = np.array([px, py, pz])
                    else:
                        self._smoothed_port_xyz = (
                            alpha * np.array([px, py, pz]) +
                            (1 - alpha) * self._smoothed_port_xyz
                        )
                    smoothed = _mat_to_pose(T_port_base)
                    smoothed.position.x = float(self._smoothed_port_xyz[0])
                    smoothed.position.y = float(self._smoothed_port_xyz[1])
                    smoothed.position.z = float(self._smoothed_port_xyz[2])
                    self._last_port_pose = smoothed
                    self._last_port_stamp_ns = self._clock.now().nanoseconds
                    self._logger.info(
                        f"[vision] accepted {candidate.get('solver', 'PNP')} "
                        f"pose=({px:.3f},{py:.3f},{pz:.3f}) "
                        f"reproj={candidate['reprojection_error_px']:.1f}px",
                        throttle_duration_sec=1.0,
                    )
                else:
                    return self._fresh_last_port_pose()

        if self._last_port_pose is None:
            self._logger.warn("[vision] no valid port pose yet", throttle_duration_sec=2.0)

        return self._fresh_last_port_pose()

    def get_plug_tip_pose(self, observation: Optional[Observation]) -> Optional[Pose]:
        tcp = observation.controller_state.tcp_pose if observation else None

        # Try ground-truth TF first (only available with ground_truth:=true).
        if self._plug_frame is not None:
            T = self._lookup_transform_mat(self.BASE_FRAME, self._plug_frame)
            if T is not None:
                plug_pose = _mat_to_pose(T)
                if tcp is not None:
                    T_tcp_base = _pose_to_mat(tcp)
                    plug_in_tcp = np.linalg.inv(T_tcp_base) @ np.array(
                        [T[0, 3], T[1, 3], T[2, 3], 1.0],
                        dtype=np.float64,
                    )
                    self._plug_tip_offset_tcp = tuple(float(v) for v in plug_in_tcp[:3])
                return plug_pose

        # TF blocked (ground_truth:=false): derive from TCP + calibrated delta.
        if tcp is None:
            return None
        if self._plug_tip_offset_tcp is None:
            if not self._allow_tcp_plug_fallback:
                self._logger.warn(
                    "[vision] no plug-tip calibration available; cannot estimate plug pose without TF",
                    throttle_duration_sec=2.0,
                )
                return None
            self._logger.warn(
                "[vision] no plug-tip calibration available; using TCP as plug-tip fallback",
                throttle_duration_sec=2.0,
            )
            return tcp
        dx, dy, dz = self._plug_tip_offset_tcp
        tip_in_base = _pose_to_mat(tcp) @ np.array([dx, dy, dz, 1.0], dtype=np.float64)
        plug = Pose()
        plug.position.x = float(tip_in_base[0])
        plug.position.y = float(tip_in_base[1])
        plug.position.z = float(tip_in_base[2])
        plug.orientation = tcp.orientation
        return plug

    # --- helpers ----------------------------------------------------------

    def _fresh_last_port_pose(self) -> Optional[Pose]:
        if self._last_port_pose is None or self._last_port_stamp_ns is None:
            return None
        age_s = (self._clock.now().nanoseconds - self._last_port_stamp_ns) / 1e9
        if age_s > self._max_stale_age_s:
            self._logger.warn(
                f"[vision] last port pose is stale ({age_s:.2f}s old); withholding command",
                throttle_duration_sec=2.0,
            )
            return None
        return self._last_port_pose

    def _select_pnp_candidate(
        self,
        result: dict,
        T_cam_base: np.ndarray,
    ) -> Optional[tuple[np.ndarray, dict]]:
        candidates = result.get("pnp_candidates", [])
        if not candidates:
            candidates = [
                {
                    "solver": "PNP",
                    "R_port_cam": result["R_port_cam"],
                    "t_port_cam": result["t_port_cam"],
                    "reprojection_error_px": result["reprojection_error_px"],
                }
            ]

        viable: list[tuple[np.ndarray, dict]] = []
        rejected = []
        for candidate in candidates:
            if float(candidate["reprojection_error_px"]) > self._max_reprojection_error_px:
                rejected.append((candidate, "reprojection"))
                continue
            T_port_cam = np.eye(4)
            T_port_cam[:3, :3] = candidate["R_port_cam"]
            T_port_cam[:3, 3] = candidate["t_port_cam"]
            T_port_base = T_cam_base @ T_port_cam
            px, py, pz = T_port_base[0, 3], T_port_base[1, 3], T_port_base[2, 3]
            if not (-0.70 <= px <= 0.20 and -0.20 <= py <= 0.50 and -0.15 <= pz <= 0.45):
                rejected.append((candidate, f"workspace ({px:.3f},{py:.3f},{pz:.3f})"))
                continue
            if not self._passes_gt_debug_gate(T_port_base):
                rejected.append((candidate, "gt_gate"))
                continue
            viable.append((T_port_base, candidate))

        if viable:
            if self._smoothed_port_xyz is not None:
                return min(
                    viable,
                    key=lambda item: (
                        float(item[1]["reprojection_error_px"])
                        + 100.0 * float(np.linalg.norm(item[0][:3, 3] - self._smoothed_port_xyz))
                    ),
                )
            return min(viable, key=lambda item: float(item[1]["reprojection_error_px"]))

        summaries = []
        for candidate, reason in rejected[:4]:
            summaries.append(
                f"{candidate.get('solver', 'PNP')}:{reason}:"
                f"{float(candidate['reprojection_error_px']):.1f}px"
            )
        self._logger.warn(
            "[vision] all PnP candidates rejected "
            + ("; ".join(summaries) if summaries else "(none)"),
            throttle_duration_sec=2.0,
        )
        return None

    def _passes_gt_debug_gate(self, T_port_base: np.ndarray) -> bool:
        if not (self._debug_gt or self._gt_error_gate):
            return True
        gt_T = self._lookup_transform_mat(self.BASE_FRAME, self._gt_port_frame)
        if gt_T is None:
            return True

        vx, vy, vz = T_port_base[0, 3], T_port_base[1, 3], T_port_base[2, 3]
        gx, gy, gz = gt_T[0, 3], gt_T[1, 3], gt_T[2, 3]
        err = np.array([vx - gx, vy - gy, vz - gz], dtype=np.float64)
        err_norm = float(np.linalg.norm(err))
        self._logger.info(
            f"[vision vs GT] "
            f"vision=({vx:.4f},{vy:.4f},{vz:.4f})  "
            f"gt=({gx:.4f},{gy:.4f},{gz:.4f})  "
            f"err=({err[0]:.4f},{err[1]:.4f},{err[2]:.4f}) norm={err_norm:.4f}",
            throttle_duration_sec=2.0,
        )
        if self._gt_error_gate and err_norm > self._max_gt_error_m:
            self._logger.warn(
                f"[vision] estimate rejected by GT gate: {err_norm:.3f}m "
                f"> {self._max_gt_error_m:.3f}m",
                throttle_duration_sec=2.0,
            )
            return False
        return True

    def _lookup_transform_mat(self, target: str, source: str) -> Optional[np.ndarray]:
        try:
            t = self._tf_buffer.lookup_transform(target, source, Time())
        except TransformException:
            return None
        return _tf_to_mat(t.transform)

    def _wait_for_tf(self, frame: str, timeout_sec: float) -> bool:
        start = self._clock.now()
        timeout = Duration(seconds=timeout_sec)
        while (self._clock.now() - start) < timeout:
            try:
                self._tf_buffer.lookup_transform(self.BASE_FRAME, frame, Time())
                return True
            except TransformException:
                self._clock.sleep_for(Duration(seconds=0.1))
        return False


def _quat_to_R(x: float, y: float, z: float, w: float) -> np.ndarray:
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


def _R_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """Return (x, y, z, w)."""
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        S = (tr + 1.0) ** 0.5 * 2
        qw = 0.25 * S
        qx = (m[2, 1] - m[1, 2]) / S
        qy = (m[0, 2] - m[2, 0]) / S
        qz = (m[1, 0] - m[0, 1]) / S
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        S = (1.0 + m[0, 0] - m[1, 1] - m[2, 2]) ** 0.5 * 2
        qw = (m[2, 1] - m[1, 2]) / S
        qx = 0.25 * S
        qy = (m[0, 1] + m[1, 0]) / S
        qz = (m[0, 2] + m[2, 0]) / S
    elif m[1, 1] > m[2, 2]:
        S = (1.0 + m[1, 1] - m[0, 0] - m[2, 2]) ** 0.5 * 2
        qw = (m[0, 2] - m[2, 0]) / S
        qx = (m[0, 1] + m[1, 0]) / S
        qy = 0.25 * S
        qz = (m[1, 2] + m[2, 1]) / S
    else:
        S = (1.0 + m[2, 2] - m[0, 0] - m[1, 1]) ** 0.5 * 2
        qw = (m[1, 0] - m[0, 1]) / S
        qx = (m[0, 2] + m[2, 0]) / S
        qy = (m[1, 2] + m[2, 1]) / S
        qz = 0.25 * S
    return qx, qy, qz, qw


def _tf_to_mat(tf) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _quat_to_R(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)
    T[:3, 3] = (tf.translation.x, tf.translation.y, tf.translation.z)
    return T


def _pose_to_mat(pose: Pose) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _quat_to_R(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    T[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
    return T


def _mat_to_pose(T: np.ndarray) -> Pose:
    qx, qy, qz, qw = _R_to_quat(T[:3, :3])
    p = Pose()
    p.position.x = float(T[0, 3])
    p.position.y = float(T[1, 3])
    p.position.z = float(T[2, 3])
    p.orientation.x = qx
    p.orientation.y = qy
    p.orientation.z = qz
    p.orientation.w = qw
    return p
