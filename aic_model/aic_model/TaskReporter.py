from rclpy.duration import Duration

from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task


class TaskReporter(Policy):
    """Read-only policy scaffold for validating task and observation plumbing."""

    def __init__(self, parent_node):
        super().__init__(parent_node)
        self.get_logger().info("TaskReporter.__init__()")

    def _log_task(self, task: Task) -> None:
        self.get_logger().info(
            "Task received: "
            f"id={task.id!r}, cable_type={task.cable_type!r}, "
            f"cable_name={task.cable_name!r}, plug_type={task.plug_type!r}, "
            f"plug_name={task.plug_name!r}, port_type={task.port_type!r}, "
            f"port_name={task.port_name!r}, "
            f"target_module_name={task.target_module_name!r}, "
            f"time_limit={task.time_limit}s"
        )

    def _log_observation_summary(self, observation: Observation) -> None:
        center = observation.center_image
        joints = observation.joint_states
        controller = observation.controller_state
        wrench = observation.wrist_wrench.wrench.force

        joint_pairs = []
        for name, position in zip(joints.name, joints.position):
            joint_pairs.append(f"{name}={position:.4f}")

        self.get_logger().info(
            "Observation received: "
            f"center_image={center.width}x{center.height} "
            f"encoding={center.encoding!r}, "
            f"stamp={center.header.stamp.sec}."
            f"{center.header.stamp.nanosec:09d}, "
            f"joints=[{', '.join(joint_pairs)}], "
            f"tcp_position=("
            f"{controller.tcp_pose.position.x:.4f}, "
            f"{controller.tcp_pose.position.y:.4f}, "
            f"{controller.tcp_pose.position.z:.4f}), "
            f"wrist_force=({wrench.x:.3f}, {wrench.y:.3f}, {wrench.z:.3f})"
        )

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        del move_robot

        self.get_logger().info("TaskReporter.insert_cable() enter")
        self._log_task(task)
        send_feedback("waiting for observations")

        timeout_sec = min(10.0, max(1.0, float(task.time_limit) * 0.25))
        deadline = self.time_now() + Duration(seconds=timeout_sec)
        observations_seen = 0
        last_log_time = self.time_now()

        while self.time_now() < deadline:
            observation = get_observation()
            if observation is None:
                if self.time_now() - last_log_time >= Duration(seconds=1.0):
                    self.get_logger().info("No observation received yet.")
                    last_log_time = self.time_now()
                self.sleep_for(0.1)
                continue

            observations_seen += 1
            self._log_observation_summary(observation)
            send_feedback(f"observed task inputs ({observations_seen})")

            if observations_seen >= 3:
                self.get_logger().info(
                    "TaskReporter observed task and sensor inputs successfully."
                )
                return True

            self.sleep_for(0.2)

        self.get_logger().error(
            f"Timed out after {timeout_sec:.1f}s waiting for observations."
        )
        return False
