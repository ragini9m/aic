from abc import ABC, abstractmethod
from typing import Optional

from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Pose


class PortPoseEstimator(ABC):
    """Pluggable interface for estimating port and plug-tip pose in base_link.

    The impedance state-machine policy depends on this interface; the
    concrete implementation (ground-truth TF, learned keypoint+PnP,
    calibrated stereo, ...) is swappable without touching the policy.
    """

    @abstractmethod
    def initialize(self, task: Task) -> bool:
        """Called once per trial. Return True on success, False to abort."""

    @abstractmethod
    def get_port_pose(self, observation: Optional[Observation]) -> Optional[Pose]:
        """Target port pose in base_link, or None if unavailable."""

    @abstractmethod
    def get_plug_tip_pose(self, observation: Optional[Observation]) -> Optional[Pose]:
        """Grasped plug-tip pose in base_link, or None if unavailable."""
