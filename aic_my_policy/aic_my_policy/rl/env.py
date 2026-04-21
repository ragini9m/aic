"""Gym-style residual-RL environment wrapping the live Gazebo/ROS stack.

Design notes
------------

The residual action is applied *on top of* the existing base policy
(artifact #2): at each step the base policy proposes a command from the
state machine, and the residual policy learns a small correction

    a = clip(a_base + pi_res(s))

The critic, during training only, receives privileged GT information
from `/tf` (ground_truth:=true) — asymmetric actor-critic. The actor
consumes only observations that will also be available at evaluation
time.

Reset strategy (first pass)
---------------------------

Cleanly re-randomizing a Gazebo world without restarting the launch is
non-trivial. This env targets a lightweight strategy:

1. Home the robot via `JointMotionUpdate` to a canonical joint config.
2. Despawn + respawn the task board through Gazebo create/delete
   services with randomized parameters. During training the Zenoh ACL
   is disabled, so these services are available.

If reset fails (or the Gazebo services are unavailable in the current
runtime) the env raises; the caller can restart the outer launch.
Future work: add a more robust reset via a custom ROS service in
aic_engine that we can call.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from aic_control_interfaces.msg import MotionUpdate, JointMotionUpdate, TrajectoryGenerationMode
from aic_control_interfaces.srv import ChangeTargetMode
from aic_model_interfaces.msg import Observation
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.node import Node
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from aic_my_policy.control.impedance import (
    ALIGN,
    APPROACH,
    SEARCH,
    SEAT,
    make_motion_update,
)

ACTION_DIM = 12          # [dx, dy, dz, drx, dry, drz, ds_x, ds_y, ds_z, ds_rx, ds_ry, ds_rz]
OBS_DIM_ACTOR = 33       # see _build_actor_obs
OBS_DIM_CRITIC_EXTRA = 14  # GT port pose + plug pose
OBS_DIM_CRITIC = OBS_DIM_ACTOR + OBS_DIM_CRITIC_EXTRA

DPOSE_LIM = np.array([0.005, 0.005, 0.005, 0.04, 0.04, 0.04])     # meters / radians
DSTIFF_LIM = np.array([1.0] * 6)                                   # log-scale factor in [-1,1] -> x0.5..x2


@dataclass
class StepResult:
    obs_actor: np.ndarray
    obs_critic: np.ndarray
    reward: float
    terminated: bool
    truncated: bool
    info: dict


class ResidualInsertionEnv(Node):
    """Single-env Gym-like wrapper. Call `reset()` then loop `step(action)`."""

    MAX_STEPS = 200  # ~10s at 20 Hz
    DT = 0.05

    def __init__(
        self,
        estimator,
        node_name: str = "aic_residual_env",
    ):
        super().__init__(node_name)
        self._estimator = estimator
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._obs_msg: Optional[Observation] = None
        self.create_subscription(Observation, "observations", self._on_obs, 10)

        self._motion_pub = self.create_publisher(MotionUpdate, "/aic_controller/pose_commands", 2)
        self._joint_pub = self.create_publisher(JointMotionUpdate, "/aic_controller/joint_commands", 2)
        self._change_mode = self.create_client(ChangeTargetMode, "/aic_controller/change_target_mode")

        self._step_count = 0
        self._last_port_xyz: Optional[np.ndarray] = None
        self._last_plug_xyz: Optional[np.ndarray] = None
        self._initial_distance: float = 0.0

    # ---- Gym-like API ----------------------------------------------------

    def reset(self, task) -> tuple[np.ndarray, np.ndarray]:
        if not self._estimator.initialize(task):
            raise RuntimeError("estimator.initialize() failed during reset")
        self._home_robot()
        self._step_count = 0
        self._last_port_xyz = None
        self._last_plug_xyz = None
        obs_actor, obs_critic, _, _ = self._read_state()
        if self._last_port_xyz is not None and self._last_plug_xyz is not None:
            self._initial_distance = float(np.linalg.norm(self._last_port_xyz - self._last_plug_xyz))
        return obs_actor, obs_critic

    def step(self, action: np.ndarray, base_command: MotionUpdate) -> StepResult:
        action = np.clip(action, -1.0, 1.0)
        d_pose = action[:6] * DPOSE_LIM
        d_stiff_log = action[6:12] * DSTIFF_LIM

        cmd = _apply_residual(base_command, d_pose, d_stiff_log)
        self._motion_pub.publish(cmd)
        self._spin_for(self.DT)

        obs_actor, obs_critic, port_xyz, plug_xyz = self._read_state()
        self._step_count += 1

        reward, terminated, info = self._compute_reward(port_xyz, plug_xyz)
        truncated = self._step_count >= self.MAX_STEPS
        return StepResult(obs_actor, obs_critic, reward, terminated, truncated, info)

    # ---- internals -------------------------------------------------------

    def _on_obs(self, msg: Observation) -> None:
        self._obs_msg = msg

    def _spin_for(self, seconds: float) -> None:
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def _read_state(self):
        port = self._estimator.get_port_pose(self._obs_msg)
        plug = self._estimator.get_plug_tip_pose(self._obs_msg)

        port_xyz = np.array([port.position.x, port.position.y, port.position.z]) if port else None
        plug_xyz = np.array([plug.position.x, plug.position.y, plug.position.z]) if plug else None
        if port_xyz is not None:
            self._last_port_xyz = port_xyz
        if plug_xyz is not None:
            self._last_plug_xyz = plug_xyz

        obs_actor = self._build_actor_obs(port, plug)
        obs_critic = self._build_critic_obs(obs_actor)
        return obs_actor, obs_critic, self._last_port_xyz, self._last_plug_xyz

    def _build_actor_obs(self, port: Optional[Pose], plug: Optional[Pose]) -> np.ndarray:
        obs = np.zeros(OBS_DIM_ACTOR, dtype=np.float32)
        idx = 0
        if port is not None:
            obs[idx:idx + 7] = [port.position.x, port.position.y, port.position.z,
                                port.orientation.x, port.orientation.y, port.orientation.z, port.orientation.w]
        idx += 7
        if plug is not None:
            obs[idx:idx + 7] = [plug.position.x, plug.position.y, plug.position.z,
                                plug.orientation.x, plug.orientation.y, plug.orientation.z, plug.orientation.w]
        idx += 7
        if self._obs_msg is not None:
            tcp = self._obs_msg.controller_state.tcp_pose
            obs[idx:idx + 7] = [tcp.position.x, tcp.position.y, tcp.position.z,
                                tcp.orientation.x, tcp.orientation.y, tcp.orientation.z, tcp.orientation.w]
            idx += 7
            w = self._obs_msg.wrist_wrench.wrench
            obs[idx:idx + 6] = [w.force.x, w.force.y, w.force.z,
                                w.torque.x, w.torque.y, w.torque.z]
            idx += 6
            err = np.asarray(self._obs_msg.controller_state.tcp_error)[:6] if len(self._obs_msg.controller_state.tcp_error) >= 6 else np.zeros(6)
            obs[idx:idx + 6] = err
        return obs

    def _build_critic_obs(self, actor_obs: np.ndarray) -> np.ndarray:
        """Asymmetric critic obs: actor obs + privileged GT (filled outside if needed)."""
        critic = np.zeros(OBS_DIM_CRITIC, dtype=np.float32)
        critic[:OBS_DIM_ACTOR] = actor_obs
        # Extra slots reserved for GT port+plug looked up directly from TF by the
        # training loop (not filled here to keep env source-of-truth-agnostic).
        return critic

    def _compute_reward(self, port_xyz, plug_xyz) -> tuple[float, bool, dict]:
        if port_xyz is None or plug_xyz is None:
            return 0.0, False, {"reason": "missing_pose"}
        delta = plug_xyz - port_xyz
        lateral = float(np.linalg.norm(delta[:2]))
        axial = float(-delta[2])  # plug below port entrance -> positive progress

        reward = -5.0 * lateral - 1.0 * max(0.0, -axial)
        terminated = False
        if self._obs_msg is not None:
            w = self._obs_msg.wrist_wrench.wrench
            fmag = float(np.linalg.norm([w.force.x, w.force.y, w.force.z]))
            if fmag > 20.0:
                reward -= 0.5
        if lateral < 0.002 and axial > 0.012:
            reward += 50.0
            terminated = True
        return reward, terminated, {"lateral": lateral, "axial": axial}

    def _home_robot(self) -> None:
        """Send a joint-space home command and wait briefly for it to settle."""
        home = JointTrajectoryPoint(positions=[0.0, -1.57, -1.57, -1.57, 1.57, 0.0])
        msg = JointMotionUpdate(
            target_state=home,
            target_stiffness=[85.0] * 6,
            target_damping=[75.0] * 6,
            trajectory_generation_mode=TrajectoryGenerationMode(
                mode=TrajectoryGenerationMode.MODE_POSITION
            ),
        )
        # Ensure we're in joint mode first.
        try:
            req = ChangeTargetMode.Request()
            req.target_mode.mode = 2  # joint
            self._change_mode.call_async(req)
        except Exception:
            pass
        self._joint_pub.publish(msg)
        self._spin_for(2.0)


def _apply_residual(
    base: MotionUpdate,
    d_pose: np.ndarray,
    d_stiff_log: np.ndarray,
) -> MotionUpdate:
    """Add residual to a base MotionUpdate without mutating the original."""
    out = MotionUpdate()
    out.header = base.header
    out.pose = Pose(
        position=Point(
            x=base.pose.position.x + float(d_pose[0]),
            y=base.pose.position.y + float(d_pose[1]),
            z=base.pose.position.z + float(d_pose[2]),
        ),
        orientation=Quaternion(
            x=base.pose.orientation.x,
            y=base.pose.orientation.y,
            z=base.pose.orientation.z,
            w=base.pose.orientation.w,
        ),
    )
    # Log-scale stiffness modulation in [-1,1] -> factor in [0.5, 2.0].
    stiff = np.array(base.target_stiffness, dtype=np.float64).reshape(6, 6)
    damp = np.array(base.target_damping, dtype=np.float64).reshape(6, 6)
    factors = np.power(2.0, d_stiff_log)          # shape (6,)
    stiff_diag = np.diag(stiff) * factors
    damp_diag = np.diag(damp) * np.sqrt(factors)  # damping scales ~ sqrt(stiffness)
    out.target_stiffness = np.diag(stiff_diag).flatten().tolist()
    out.target_damping = np.diag(damp_diag).flatten().tolist()
    out.feedforward_wrench_at_tip = base.feedforward_wrench_at_tip
    out.wrench_feedback_gains_at_tip = list(base.wrench_feedback_gains_at_tip)
    out.trajectory_generation_mode = base.trajectory_generation_mode
    return out


# Convenience re-exports so training scripts can reach the base profiles.
BASE_PROFILES = {"APPROACH": APPROACH, "ALIGN": ALIGN, "SEARCH": SEARCH, "SEAT": SEAT}
