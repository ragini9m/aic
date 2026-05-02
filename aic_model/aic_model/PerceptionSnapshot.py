import json
import os
from datetime import datetime
from pathlib import Path

from rclpy.duration import Duration

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from sensor_msgs.msg import CameraInfo, Image, JointState


class PerceptionSnapshot(Policy):
    """Save read-only perception snapshots for target-localization work."""

    def __init__(self, parent_node):
        super().__init__(parent_node)
        run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        snapshot_root = os.environ.get(
            "AIC_SNAPSHOT_DIR", "/tmp/aic_perception_snapshots"
        )
        self._run_dir = Path(snapshot_root) / run_id
        self._run_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Saving perception snapshots to {self._run_dir}")

    def _task_dir(self, task: Task) -> Path:
        safe_target = f"{task.target_module_name}_{task.port_name}".replace("/", "_")
        task_dir = self._run_dir / f"{task.id}_{task.plug_type}_to_{safe_target}"
        task_dir.mkdir(parents=True, exist_ok=True)
        return task_dir

    def _task_metadata(self, task: Task) -> dict:
        return {
            "id": task.id,
            "cable_type": task.cable_type,
            "cable_name": task.cable_name,
            "plug_type": task.plug_type,
            "plug_name": task.plug_name,
            "port_type": task.port_type,
            "port_name": task.port_name,
            "target_module_name": task.target_module_name,
            "time_limit": int(task.time_limit),
        }

    def _camera_info_metadata(self, camera_info: CameraInfo) -> dict:
        return {
            "width": int(camera_info.width),
            "height": int(camera_info.height),
            "distortion_model": camera_info.distortion_model,
            "d": list(camera_info.d),
            "k": list(camera_info.k),
            "r": list(camera_info.r),
            "p": list(camera_info.p),
        }

    def _joint_metadata(self, joint_state: JointState) -> dict:
        return {
            "name": list(joint_state.name),
            "position": list(joint_state.position),
            "velocity": list(joint_state.velocity),
            "effort": list(joint_state.effort),
        }

    def _observation_metadata(self, observation: Observation) -> dict:
        controller = observation.controller_state
        wrench = observation.wrist_wrench.wrench
        return {
            "stamp": {
                "sec": int(observation.center_image.header.stamp.sec),
                "nanosec": int(observation.center_image.header.stamp.nanosec),
            },
            "cameras": {
                "left": self._camera_info_metadata(observation.left_camera_info),
                "center": self._camera_info_metadata(observation.center_camera_info),
                "right": self._camera_info_metadata(observation.right_camera_info),
            },
            "joints": self._joint_metadata(observation.joint_states),
            "tcp_pose": {
                "position": {
                    "x": controller.tcp_pose.position.x,
                    "y": controller.tcp_pose.position.y,
                    "z": controller.tcp_pose.position.z,
                },
                "orientation": {
                    "x": controller.tcp_pose.orientation.x,
                    "y": controller.tcp_pose.orientation.y,
                    "z": controller.tcp_pose.orientation.z,
                    "w": controller.tcp_pose.orientation.w,
                },
            },
            "tcp_velocity": {
                "linear": {
                    "x": controller.tcp_velocity.linear.x,
                    "y": controller.tcp_velocity.linear.y,
                    "z": controller.tcp_velocity.linear.z,
                },
                "angular": {
                    "x": controller.tcp_velocity.angular.x,
                    "y": controller.tcp_velocity.angular.y,
                    "z": controller.tcp_velocity.angular.z,
                },
            },
            "wrist_wrench": {
                "force": {
                    "x": wrench.force.x,
                    "y": wrench.force.y,
                    "z": wrench.force.z,
                },
                "torque": {
                    "x": wrench.torque.x,
                    "y": wrench.torque.y,
                    "z": wrench.torque.z,
                },
            },
        }

    def _image_bytes_rgb(self, image: Image) -> tuple[bytes, str]:
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

    def _save_image(self, image: Image, path_without_suffix: Path) -> Path:
        image_bytes, suffix = self._image_bytes_rgb(image)
        path = path_without_suffix.with_suffix(f".{suffix}")
        magic = b"P6" if suffix == "ppm" else b"P5"
        header = magic + f"\n{image.width} {image.height}\n255\n".encode("ascii")
        path.write_bytes(header + image_bytes)
        return path

    def _save_snapshot(
        self, task: Task, observation: Observation, task_dir: Path, index: int
    ) -> None:
        prefix = f"{index:02d}"
        image_paths = {}
        for name, image in (
            ("left", observation.left_image),
            ("center", observation.center_image),
            ("right", observation.right_image),
        ):
            image_path = self._save_image(image, task_dir / f"{prefix}_{name}")
            image_paths[name] = str(image_path)

        metadata = {
            "task": self._task_metadata(task),
            "observation": self._observation_metadata(observation),
            "images": image_paths,
        }
        metadata_path = task_dir / f"{prefix}_metadata.json"
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        del move_robot

        task_dir = self._task_dir(task)
        self.get_logger().info(
            "PerceptionSnapshot task: "
            f"id={task.id!r}, plug={task.plug_type!r}/{task.plug_name!r}, "
            f"target={task.target_module_name!r}/{task.port_name!r}, "
            f"saving to {task_dir}"
        )
        send_feedback("saving perception snapshots")

        timeout_sec = min(10.0, max(1.0, float(task.time_limit) * 0.25))
        deadline = self.time_now() + Duration(seconds=timeout_sec)
        saved = 0
        last_log_time = self.time_now()

        while self.time_now() < deadline:
            observation = get_observation()
            if observation is None:
                if self.time_now() - last_log_time >= Duration(seconds=1.0):
                    self.get_logger().info("No observation received yet.")
                    last_log_time = self.time_now()
                self.sleep_for(0.1)
                continue

            try:
                self._save_snapshot(task, observation, task_dir, saved)
            except Exception as ex:
                self.get_logger().error(f"Failed to save snapshot: {ex}")
                return False

            saved += 1
            self.get_logger().info(f"Saved perception snapshot {saved} to {task_dir}")
            send_feedback(f"saved perception snapshot {saved}")

            if saved >= 3:
                self.get_logger().info("PerceptionSnapshot complete.")
                return True

            self.sleep_for(0.25)

        self.get_logger().error(
            f"Timed out after {timeout_sec:.1f}s waiting for observations."
        )
        return False
