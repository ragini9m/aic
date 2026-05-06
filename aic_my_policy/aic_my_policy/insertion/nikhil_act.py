"""Nikhil's ACT insertion policy adapted as a post-alignment controller."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from aic_model.policy import GetObservationCallback, MoveRobotCallback, SendFeedbackCallback
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Pose, Twist, Vector3, Wrench
from rclpy.node import Node

from aic_my_policy.estimators.base import PortPoseEstimator


MAX_LIN_VEL = 0.15
MAX_ANG_VEL = 1.5
DEFAULT_TIMEOUT_S = 60.0
STALL_GRACE_S = 5.0
STALL_WINDOW_S = 4.0
STALL_RATE_MM_S = 0.5
INSERT_DONE_M = 0.002
CLOSE_THRESH_M = 0.010
LATERAL_KP = 10.0


class NikhilACTInsertion:
    """Run the trained ACT insertion phase after our policy has aligned the plug.

    This deliberately skips Nikhil's GT approach/recovery state machine. Our
    vision alignment supplies the starting pose, and this controller only runs
    the learned velocity policy plus lightweight progress/lateral checks.
    """

    def __init__(self, parent_node: Node):
        self._node = parent_node
        self._logger = parent_node.get_logger()
        self._enabled_for_sc = bool(self._param("nikhil_act_enable_sc", False))
        self._timeout_s = float(self._param("nikhil_act_timeout_s", DEFAULT_TIMEOUT_S))
        self._lateral_correct = bool(self._param("nikhil_act_lateral_correct", True))
        self._action_scale = float(self._param("nikhil_act_action_scale", 1.0))
        path_param = str(self._param("nikhil_act_policy_path", ""))
        self._policy_path = Path(path_param).expanduser() if path_param else self._default_policy_path()

        self._torch = None
        self._policy = None
        self._device = None
        self._img_stats = {}
        self._image_scale = 0.25
        self._state_mean = None
        self._state_std = None
        self._wrist_mean = None
        self._wrist_std = None
        self._action_mean = None
        self._action_std = None
        self._loaded = False

    def _param(self, name: str, default):
        if not self._node.has_parameter(name):
            self._node.declare_parameter(name, default)
        return self._node.get_parameter(name).value

    @staticmethod
    def _default_policy_path() -> Path:
        source_tree_path = (
            Path(__file__).resolve().parents[2]
            / "policy"
            / "sfp_insertion_demos_act_20260504_184125_steplast"
            / "pretrained_model"
        )
        if source_tree_path.is_dir():
            return source_tree_path
        try:
            from ament_index_python.packages import get_package_share_directory

            share_path = (
                Path(get_package_share_directory("aic_my_policy"))
                / "policy"
                / "sfp_insertion_demos_act_20260504_184125_steplast"
                / "pretrained_model"
            )
            if share_path.is_dir():
                return share_path
        except Exception:
            pass
        return source_tree_path

    def _ensure_loaded(self) -> None:
        if self._loaded:
            return

        model_file = self._policy_path / "model.safetensors"
        stats_file = self._policy_path / "policy_preprocessor_step_3_normalizer_processor.safetensors"
        for path in (self._policy_path / "config.json", model_file, stats_file):
            if not path.is_file():
                raise RuntimeError(f"Nikhil ACT artifact missing: {path}")
        self._reject_lfs_pointer(model_file)
        self._reject_lfs_pointer(stats_file)

        try:
            import draccus
            import torch
            from lerobot.policies.act.configuration_act import ACTConfig
            from lerobot.policies.act.modeling_act import ACTPolicy
            from safetensors.torch import load_file
        except Exception as ex:
            raise RuntimeError(
                "Nikhil ACT requires draccus, torch, lerobot, and safetensors. "
                "Run `pixi install` after the updated pixi.toml is in place."
            ) from ex

        with open(self._policy_path / "config.json") as f:
            config_dict = json.load(f)
        config_dict.pop("type", None)

        self._torch = torch
        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        config = draccus.decode(ACTConfig, config_dict)
        self._policy = ACTPolicy(config)
        self._policy.load_state_dict(load_file(model_file))
        self._policy.eval()
        self._policy.to(self._device)

        stats = load_file(stats_file)

        def stat(key: str, shape):
            return stats[key].to(self._device).view(*shape)

        self._img_stats = {
            "left": {
                "mean": stat("observation.images.left_camera.mean", (1, 3, 1, 1)),
                "std": stat("observation.images.left_camera.std", (1, 3, 1, 1)),
            },
            "center": {
                "mean": stat("observation.images.center_camera.mean", (1, 3, 1, 1)),
                "std": stat("observation.images.center_camera.std", (1, 3, 1, 1)),
            },
            "right": {
                "mean": stat("observation.images.right_camera.mean", (1, 3, 1, 1)),
                "std": stat("observation.images.right_camera.std", (1, 3, 1, 1)),
            },
        }
        self._state_mean = stat("observation.state.mean", (1, -1))
        self._state_std = stat("observation.state.std", (1, -1))
        self._wrist_mean = stat("observation.wrist_force.mean", (1, -1))
        self._wrist_std = stat("observation.wrist_force.std", (1, -1))
        self._action_mean = stat("action.mean", (1, -1))
        self._action_std = stat("action.std", (1, -1))
        self._loaded = True
        self._logger.info(f"Nikhil ACT insertion loaded from {self._policy_path}")

    @staticmethod
    def _reject_lfs_pointer(path: Path) -> None:
        with open(path, "rb") as f:
            prefix = f.read(64)
        if prefix.startswith(b"version https://git-lfs.github.com/spec"):
            raise RuntimeError(
                f"{path} is a Git LFS pointer, not the real model artifact. "
                "Run `git lfs pull` in the teammate repo or replace this file "
                "with the real safetensors artifact before running ACT insertion."
            )

    def run(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        estimator: PortPoseEstimator,
        port_pose: Pose,
        plug_pose: Pose,
    ) -> bool:
        if task.port_type == "sc" and not self._enabled_for_sc:
            self._logger.warn(
                "[nikhil_act] SC requested, but only the SFP checkpoint is enabled; "
                "holding aligned pose instead."
            )
            return True

        self._ensure_loaded()
        self._policy.reset()
        axis = self._insertion_axis(task)
        port_xyz = _pose_xyz(port_pose)
        start_plug_xyz = _pose_xyz(plug_pose)
        lateral_ref = start_plug_xyz.copy()
        last_prog = 0.0
        last_t = time.monotonic()
        stall_start = None
        start = time.time()
        step = 0

        self._logger.info(
            f"[nikhil_act] starting from plug={start_plug_xyz.round(4)} "
            f"port={port_xyz.round(4)} axis={axis.round(4)}"
        )
        send_feedback("NIKHIL_ACT_INSERT")

        while time.time() - start < self._timeout_s:
            loop_start = time.time()
            obs = get_observation()
            if obs is None:
                self._sleep_remaining(loop_start)
                continue

            current_plug = estimator.get_plug_tip_pose(obs)
            plug_xyz = _pose_xyz(current_plug) if current_plug is not None else None
            if plug_xyz is not None:
                progress = float(np.dot(plug_xyz - start_plug_xyz, axis))
                remaining = float(np.dot(port_xyz - plug_xyz, axis))
                now = time.monotonic()
                dt = max(now - last_t, 0.01)
                rate_mm_s = (progress - last_prog) / dt * 1000.0
                last_prog = progress
                last_t = now

                if remaining <= INSERT_DONE_M:
                    self._stop(move_robot)
                    self._logger.info(
                        f"[nikhil_act] insertion complete remaining={remaining*1000:.1f}mm step={step}"
                    )
                    return True
                if remaining > CLOSE_THRESH_M and time.time() - start > STALL_GRACE_S:
                    if rate_mm_s < STALL_RATE_MM_S:
                        stall_start = stall_start or now
                        if now - stall_start >= STALL_WINDOW_S:
                            self._stop(move_robot)
                            self._logger.warn(
                                f"[nikhil_act] stalled progress={progress*1000:.1f}mm "
                                f"rate={rate_mm_s:.2f}mm/s"
                            )
                            return False
                    else:
                        stall_start = None

            with self._torch.inference_mode():
                normalized_action = self._policy.select_action(self._prepare_observations(obs))
            raw_action = normalized_action * self._action_std + self._action_mean
            action = raw_action[0].cpu().numpy()
            lin = np.array(action[:3], dtype=float) * self._action_scale
            ang = np.array(action[3:6], dtype=float) * self._action_scale

            if self._lateral_correct and plug_xyz is not None:
                disp = plug_xyz - lateral_ref
                axial_component = float(np.dot(disp, axis))
                lat_drift = disp - axial_component * axis
                lin += LATERAL_KP * (-lat_drift)

            self._send_velocity(move_robot, lin, ang)
            if step % 10 == 0:
                self._logger.info(
                    f"[nikhil_act] step={step} lin=({lin[0]:.4f},{lin[1]:.4f},{lin[2]:.4f})",
                    throttle_duration_sec=1.0,
                )
            step += 1
            self._sleep_remaining(loop_start)

        self._stop(move_robot)
        self._logger.info(f"[nikhil_act] timeout after {step} steps")
        return True

    @staticmethod
    def _insertion_axis(task: Task) -> np.ndarray:
        # Current challenge port entrances are aligned along base -Z for our
        # vision-staged pose. Keep this explicit until we replace it with a
        # calibrated per-port axis.
        return np.array([0.0, 0.0, -1.0 if task.port_type in ("sfp", "sc") else -1.0])

    def _prepare_observations(self, obs: Observation) -> dict:
        torch = self._torch
        tcp = obs.controller_state.tcp_pose
        vel = obs.controller_state.tcp_velocity
        state_np = np.array(
            [
                tcp.position.x, tcp.position.y, tcp.position.z,
                tcp.orientation.x, tcp.orientation.y, tcp.orientation.z, tcp.orientation.w,
                vel.linear.x, vel.linear.y, vel.linear.z,
                vel.angular.x, vel.angular.y, vel.angular.z,
                *obs.controller_state.tcp_error,
                *obs.joint_states.position[:7],
            ],
            dtype=np.float32,
        )
        raw_state = torch.from_numpy(state_np).unsqueeze(0).to(self._device)

        ft = obs.wrist_wrench.wrench.force
        wrist_np = np.array([ft.x, ft.y, ft.z], dtype=np.float32)
        raw_wrist = torch.from_numpy(wrist_np).unsqueeze(0).to(self._device)

        return {
            "observation.images.left_camera": self._img_to_tensor(obs.left_image, "left"),
            "observation.images.center_camera": self._img_to_tensor(obs.center_image, "center"),
            "observation.images.right_camera": self._img_to_tensor(obs.right_image, "right"),
            "observation.state": (raw_state - self._state_mean) / self._state_std,
            "observation.wrist_force": (raw_wrist - self._wrist_mean) / self._wrist_std,
        }

    def _img_to_tensor(self, raw_img, camera: str):
        img_np = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(
            raw_img.height, raw_img.width, 3
        )
        if self._image_scale != 1.0:
            img_np = cv2.resize(
                img_np, None, fx=self._image_scale, fy=self._image_scale,
                interpolation=cv2.INTER_AREA,
            )
        tensor = (
            self._torch.from_numpy(img_np.copy())
            .permute(2, 0, 1)
            .float()
            .div(255.0)
            .unsqueeze(0)
            .to(self._device)
        )
        return (tensor - self._img_stats[camera]["mean"]) / self._img_stats[camera]["std"]

    def _send_velocity(
        self,
        move_robot: MoveRobotCallback,
        lin: np.ndarray,
        ang: np.ndarray,
    ) -> None:
        lin = _clip_vec(lin, MAX_LIN_VEL)
        ang = _clip_vec(ang, MAX_ANG_VEL)
        twist = Twist(
            linear=Vector3(x=float(lin[0]), y=float(lin[1]), z=float(lin[2])),
            angular=Vector3(x=float(ang[0]), y=float(ang[1]), z=float(ang[2])),
        )
        msg = MotionUpdate()
        msg.velocity = twist
        msg.header.frame_id = "base_link"
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.target_stiffness = np.diag([100.0, 100.0, 100.0, 50.0, 50.0, 50.0]).flatten().tolist()
        msg.target_damping = np.diag([40.0, 40.0, 40.0, 15.0, 15.0, 15.0]).flatten().tolist()
        msg.feedforward_wrench_at_tip = Wrench(
            force=Vector3(x=0.0, y=0.0, z=0.0),
            torque=Vector3(x=0.0, y=0.0, z=0.0),
        )
        msg.wrench_feedback_gains_at_tip = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
        msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_VELOCITY
        move_robot(motion_update=msg)

    def _stop(self, move_robot: MoveRobotCallback) -> None:
        self._send_velocity(move_robot, np.zeros(3), np.zeros(3))

    @staticmethod
    def _sleep_remaining(loop_start: float) -> None:
        time.sleep(max(0.0, 0.1 - (time.time() - loop_start)))


def _pose_xyz(pose: Optional[Pose]) -> np.ndarray:
    if pose is None:
        return np.array([np.nan, np.nan, np.nan], dtype=float)
    return np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)


def _clip_vec(vec: np.ndarray, max_norm: float) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm > max_norm:
        return vec / norm * max_norm
    return vec
