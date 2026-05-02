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

        self._inference = {
            "sfp": PortKeypointInference(sfp_weights, port_type="sfp"),
            "sc":  PortKeypointInference(sc_weights,  port_type="sc"),
        }
        self._port_type: Optional[str] = None
        self._plug_frame: Optional[str] = None
        self._last_port_pose = None

    # --- lifecycle --------------------------------------------------------

    def initialize(self, task: Task) -> bool:
        self._last_port_pose = None
        self._port_type = task.port_type
        if self._port_type not in self._inference:
            self._logger.error(f"No weights loaded for port_type={self._port_type!r}")
            return False
        self._plug_frame = f"{task.cable_name}/{task.plug_name}_link"
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
            T_cam_base = self._lookup_transform_mat(self.BASE_FRAME, cam_frame)
            if T_cam_base is None:
                self._logger.warn(f"[vision] TF lookup failed: {self.BASE_FRAME} <- {cam_frame}", throttle_duration_sec=2.0)
            else:
                T_port_cam = np.eye(4)
                T_port_cam[:3, :3] = result["R_port_cam"]
                T_port_cam[:3, 3] = result["t_port_cam"]
                T_port_base = T_cam_base @ T_port_cam
                self._last_port_pose = _mat_to_pose(T_port_base)

        if self._last_port_pose is None:
            self._logger.warn("[vision] no valid port pose yet", throttle_duration_sec=2.0)
        return self._last_port_pose

    def get_plug_tip_pose(self, observation: Optional[Observation]) -> Optional[Pose]:
        if self._plug_frame is None:
            return None
        T = self._lookup_transform_mat(self.BASE_FRAME, self._plug_frame)
        if T is None:
            return None
        return _mat_to_pose(T)

    # --- helpers ----------------------------------------------------------

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
