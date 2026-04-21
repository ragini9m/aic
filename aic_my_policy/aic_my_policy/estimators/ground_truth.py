from typing import Optional

from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException

from aic_my_policy.estimators.base import PortPoseEstimator


class GroundTruthPortPoseEstimator(PortPoseEstimator):
    """Looks up port and plug-tip poses from /tf.

    Only valid during development (ground_truth:=true at launch). The
    final evaluation blocks these frames via Zenoh ACL, so this
    estimator will time out at eval and must be swapped for the
    vision-based estimator before submitting.
    """

    BASE_FRAME = "base_link"

    def __init__(self, parent_node):
        self._parent_node = parent_node
        self._logger = parent_node.get_logger()
        self._clock = parent_node.get_clock()
        self._tf_buffer = parent_node._tf_buffer
        self._port_frame: Optional[str] = None
        self._plug_frame: Optional[str] = None

    def initialize(self, task: Task) -> bool:
        self._port_frame = f"task_board/{task.target_module_name}/{task.port_name}_link"
        self._plug_frame = f"{task.cable_name}/{task.plug_name}_link"
        self._logger.info(
            f"[GT estimator] port_frame={self._port_frame} plug_frame={self._plug_frame}"
        )
        for frame in (self._port_frame, self._plug_frame):
            if not self._wait_for_tf(frame, timeout_sec=10.0):
                self._logger.error(f"[GT estimator] TF '{frame}' unavailable.")
                return False
        return True

    def get_port_pose(self, observation: Optional[Observation]) -> Optional[Pose]:
        return self._lookup_as_pose(self._port_frame)

    def get_plug_tip_pose(self, observation: Optional[Observation]) -> Optional[Pose]:
        return self._lookup_as_pose(self._plug_frame)

    def _lookup_as_pose(self, frame: Optional[str]) -> Optional[Pose]:
        if frame is None:
            return None
        try:
            tfs = self._tf_buffer.lookup_transform(self.BASE_FRAME, frame, Time())
        except TransformException:
            return None
        pose = Pose()
        pose.position.x = tfs.transform.translation.x
        pose.position.y = tfs.transform.translation.y
        pose.position.z = tfs.transform.translation.z
        pose.orientation = tfs.transform.rotation
        return pose

    def _wait_for_tf(self, frame: str, timeout_sec: float) -> bool:
        start = self._clock.now()
        timeout = Duration(seconds=timeout_sec)
        attempt = 0
        while (self._clock.now() - start) < timeout:
            try:
                self._tf_buffer.lookup_transform(self.BASE_FRAME, frame, Time())
                return True
            except TransformException:
                if attempt % 20 == 0:
                    self._logger.info(
                        f"Waiting for TF '{frame}' (run eval with ground_truth:=true)"
                    )
                attempt += 1
                self._clock.sleep_for(Duration(seconds=0.1))
        return False
