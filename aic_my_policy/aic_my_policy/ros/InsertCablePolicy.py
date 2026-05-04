"""Impedance state-machine base policy: APPROACH -> ALIGN -> SEARCH -> SEAT -> VERIFY.

Pluggable PortPoseEstimator; defaults to ground-truth TF (requires
`ground_truth:=true` at launch). Swap the estimator for the vision-based
one before submission.
"""

from enum import Enum
from typing import Optional

from aic_control_interfaces.msg import MotionUpdate
from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_model_interfaces.msg import Observation
from aic_task_interfaces.msg import Task
from geometry_msgs.msg import Pose

from aic_my_policy.control.geometry import (
    gripper_orientation_for_alignment,
    spiral_xy_offset,
    target_gripper_pose_for_plug_tip,
)
from aic_my_policy.control.impedance import (
    ALIGN,
    APPROACH,
    ImpedanceProfile,
    SEARCH,
    SEAT,
    make_motion_update,
)
from aic_my_policy.estimators.base import PortPoseEstimator
from aic_my_policy.estimators.ground_truth import GroundTruthPortPoseEstimator


# --- Parameters ------------------------------------------------------------
# Offsets are applied to the port position in base_link (+z is up).

LOOP_HZ = 20.0
LOOP_DT = 1.0 / LOOP_HZ

APPROACH_Z_OFFSET = 0.08      # plug tip parks 8 cm above the port
ALIGN_Z_OFFSET = 0.02         # 2 cm above the port, fully aligned
SEARCH_Z_START = 0.005        # begin search 5 mm above the port entrance
SEAT_Z_DEPTH = -0.015         # 15 mm below entrance = fully seated

LIFT_BUDGET_S = 3.0
XY_ALIGN_BUDGET_S = 4.0
APPROACH_BUDGET_S = 5.0
ALIGN_BUDGET_S = 3.0
SEARCH_BUDGET_S = 15.0
SEAT_BUDGET_S = 8.0
VERIFY_HOLD_S = 2.0
MISSING_POSE_ABORT_S = 5.0

SAFE_Z = 0.35           # lift to this height before moving laterally (m in base_link)
XY_ALIGN_TOLERANCE_M = 0.015

# Port position sanity bounds (base_link frame). Reject vision estimates outside these.
PORT_X_BOUNDS = (-1.00, 1.00)
PORT_Y_BOUNDS = (-0.50, 1.00)
PORT_Z_BOUNDS = (-0.30, 0.80)

SEARCH_DESCENT_M_PER_S = 0.004   # slow sink while spiraling (4 mm/s)
SEAT_DESCENT_M_PER_S = 0.010     # faster once seated (10 mm/s)


class State(Enum):
    INIT = "INIT"
    LIFT = "LIFT"
    XY_ALIGN = "XY_ALIGN"
    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    SEARCH = "SEARCH"
    SEAT = "SEAT"
    VERIFY = "VERIFY"
    DONE = "DONE"
    ABORT = "ABORT"


class InsertCablePolicy(Policy):
    def __init__(self, parent_node):
        super().__init__(parent_node)
        self._estimator = self._build_estimator(parent_node)
        self.get_logger().info(
            f"InsertCablePolicy ready ({type(self._estimator).__name__})."
        )

    def _build_estimator(self, parent_node) -> PortPoseEstimator:
        """Choose estimator via ROS params; default to GT for bring-up.

        Params (declared on the aic_model parent node):
          estimator              : 'ground_truth' | 'vision' (default 'ground_truth')
          sfp_keypoint_weights   : path to trained SFP keypoint checkpoint
          sc_keypoint_weights    : path to trained SC  keypoint checkpoint
        """
        def _param(name: str, default):
            if not parent_node.has_parameter(name):
                parent_node.declare_parameter(name, default)
            return parent_node.get_parameter(name).value

        kind = _param("estimator", "ground_truth")
        if kind == "ground_truth":
            return GroundTruthPortPoseEstimator(parent_node)
        if kind == "vision":
            from aic_my_policy.estimators.vision import VisionPortPoseEstimator
            sfp = _param("sfp_keypoint_weights", "")
            sc = _param("sc_keypoint_weights", "")
            if not sfp or not sc:
                raise RuntimeError(
                    "estimator=vision requires sfp_keypoint_weights and sc_keypoint_weights"
                )
            return VisionPortPoseEstimator(parent_node, sfp_weights=sfp, sc_weights=sc)
        raise RuntimeError(f"Unknown estimator={kind!r}")

    # ---- lifecycle -------------------------------------------------------

    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
    ) -> bool:
        self.get_logger().info(f"insert_cable() task={task}")
        send_feedback(f"starting {task.plug_name} -> {task.port_name}")

        if not self._estimator.initialize(task):
            self.get_logger().error("Estimator init failed.")
            return False

        task_started = self.time_now()
        state = State.LIFT
        state_entered = self.time_now()
        missing_pose_since = None
        send_feedback("LIFT")
        success = False

        while state not in (State.DONE, State.ABORT):
            elapsed_task_s = (self.time_now() - task_started).nanoseconds / 1e9
            if task.time_limit > 0 and elapsed_task_s >= max(0.0, task.time_limit - 1.0):
                self.get_logger().error(
                    f"[policy] aborting before task time limit ({elapsed_task_s:.1f}s/{task.time_limit}s)"
                )
                send_feedback("ABORT time_limit")
                state = State.ABORT
                break

            obs = get_observation()
            port = self._estimator.get_port_pose(obs)
            plug = self._estimator.get_plug_tip_pose(obs)
            gripper = self._gripper_pose(obs)

            if port is None or plug is None or gripper is None:
                if missing_pose_since is None:
                    missing_pose_since = self.time_now()
                missing_s = (self.time_now() - missing_pose_since).nanoseconds / 1e9
                self.get_logger().warn(
                    f"[policy] waiting: port={'ok' if port else 'NONE'} "
                    f"plug={'ok' if plug else 'NONE'} "
                    f"gripper={'ok' if gripper else 'NONE'} "
                    f"missing_s={missing_s:.1f}",
                    throttle_duration_sec=2.0,
                )
                if missing_s >= MISSING_POSE_ABORT_S:
                    self.get_logger().error("[policy] aborting after missing pose timeout")
                    send_feedback("ABORT missing_pose")
                    state = State.ABORT
                    break
                self.sleep_for(LOOP_DT)
                continue

            # Sanity-check port pose — reject vision estimates outside the workspace.
            px, py, pz = port.position.x, port.position.y, port.position.z
            if not (PORT_X_BOUNDS[0] <= px <= PORT_X_BOUNDS[1] and
                    PORT_Y_BOUNDS[0] <= py <= PORT_Y_BOUNDS[1] and
                    PORT_Z_BOUNDS[0] <= pz <= PORT_Z_BOUNDS[1]):
                if missing_pose_since is None:
                    missing_pose_since = self.time_now()
                missing_s = (self.time_now() - missing_pose_since).nanoseconds / 1e9
                self.get_logger().warn(
                    f"[policy] port pose out of bounds ({px:.3f},{py:.3f},{pz:.3f}), "
                    f"skipping tick missing_s={missing_s:.1f}",
                    throttle_duration_sec=2.0,
                )
                if missing_s >= MISSING_POSE_ABORT_S:
                    self.get_logger().error("[policy] aborting after invalid port timeout")
                    send_feedback("ABORT invalid_port")
                    state = State.ABORT
                    break
                self.sleep_for(LOOP_DT)
                continue
            missing_pose_since = None

            elapsed_in_state = (self.time_now() - state_entered).nanoseconds / 1e9

            if state == State.LIFT:
                # Lift straight up to SAFE_Z before moving laterally.
                lift_target = Pose()
                lift_target.position.x = gripper.position.x
                lift_target.position.y = gripper.position.y
                lift_target.position.z = SAFE_Z
                lift_target.orientation = gripper.orientation
                cmd = make_motion_update(
                    pose=lift_target,
                    profile=APPROACH,
                    stamp=self.time_now().to_msg(),
                )
                try:
                    move_robot(motion_update=cmd)
                except Exception:
                    pass
                at_safe_z = gripper.position.z >= (SAFE_Z - 0.03)
                if at_safe_z or elapsed_in_state > LIFT_BUDGET_S:
                    state, state_entered = State.XY_ALIGN, self.time_now()
                    send_feedback("XY_ALIGN")

            elif state == State.XY_ALIGN:
                # First align laterally at the current safe height. This avoids
                # coupling noisy z estimates with lateral motion near the board.
                keep_current_plug_z = plug.position.z - port.position.z
                lateral_error = (
                    (plug.position.x - port.position.x) ** 2
                    + (plug.position.y - port.position.y) ** 2
                ) ** 0.5
                self.get_logger().info(
                    f"[xy_align] current_plug_xy=({plug.position.x:.4f},{plug.position.y:.4f}) "
                    f"target_port_xy=({port.position.x:.4f},{port.position.y:.4f}) "
                    f"err={lateral_error:.4f}m "
                    f"tcp_xy=({gripper.position.x:.4f},{gripper.position.y:.4f}) "
                    f"fixed_port_z={port.position.z:.4f}",
                    throttle_duration_sec=0.5,
                )
                self._command(
                    move_robot, APPROACH, port, plug, gripper,
                    z_offset=keep_current_plug_z,
                    slerp_fraction=min(1.0, elapsed_in_state / max(0.1, XY_ALIGN_BUDGET_S)),
                )
                if lateral_error <= XY_ALIGN_TOLERANCE_M or elapsed_in_state > XY_ALIGN_BUDGET_S:
                    state, state_entered = State.APPROACH, self.time_now()
                    send_feedback("APPROACH")

            elif state == State.APPROACH:
                self._command(
                    move_robot, APPROACH, port, plug, gripper,
                    z_offset=APPROACH_Z_OFFSET,
                    slerp_fraction=min(1.0, elapsed_in_state / (APPROACH_BUDGET_S * 0.75)),
                )
                if elapsed_in_state > APPROACH_BUDGET_S:
                    state, state_entered = State.ALIGN, self.time_now()
                    send_feedback("ALIGN")

            elif state == State.ALIGN:
                self._command(
                    move_robot, ALIGN, port, plug, gripper,
                    z_offset=ALIGN_Z_OFFSET,
                )
                if elapsed_in_state > ALIGN_BUDGET_S:
                    state, state_entered = State.SEARCH, self.time_now()
                    send_feedback("SEARCH")

            elif state == State.SEARCH:
                z = SEARCH_Z_START - SEARCH_DESCENT_M_PER_S * elapsed_in_state
                dx, dy = spiral_xy_offset(elapsed_in_state)
                self._command(
                    move_robot, SEARCH, port, plug, gripper,
                    z_offset=z,
                    xy_offset=(dx, dy),
                )
                timed_out = elapsed_in_state > SEARCH_BUDGET_S
                passed_entrance = plug.position.z < (port.position.z - 0.002)
                if timed_out or passed_entrance:
                    state, state_entered = State.SEAT, self.time_now()
                    send_feedback("SEAT")

            elif state == State.SEAT:
                z = -0.002 - SEAT_DESCENT_M_PER_S * elapsed_in_state
                z = max(z, SEAT_Z_DEPTH)
                self._command(
                    move_robot, SEAT, port, plug, gripper,
                    z_offset=z,
                )
                if z <= SEAT_Z_DEPTH + 1e-4 or elapsed_in_state > SEAT_BUDGET_S:
                    state, state_entered = State.VERIFY, self.time_now()
                    send_feedback("VERIFY")

            elif state == State.VERIFY:
                if elapsed_in_state >= VERIFY_HOLD_S:
                    success = True
                    state = State.DONE

            self.sleep_for(LOOP_DT)

        self._hold_current_pose(get_observation(), move_robot)

        self.get_logger().info(f"insert_cable() exit state={state.value} success={success}")
        return success

    # ---- helpers ---------------------------------------------------------

    def _gripper_pose(self, observation: Optional[Observation]) -> Optional[Pose]:
        if observation is None:
            return None
        return observation.controller_state.tcp_pose

    def _hold_current_pose(
        self,
        observation: Optional[Observation],
        move_robot: MoveRobotCallback,
    ) -> None:
        gripper_pose = self._gripper_pose(observation)
        if gripper_pose is None:
            return
        cmd = make_motion_update(
            pose=gripper_pose,
            profile=APPROACH,
            stamp=self.time_now().to_msg(),
        )
        try:
            move_robot(motion_update=cmd)
        except Exception as ex:
            self.get_logger().warn(f"hold-current command failed during abort: {ex}")

    def _command(
        self,
        move_robot: MoveRobotCallback,
        profile: ImpedanceProfile,
        port_pose: Pose,
        plug_pose: Pose,
        gripper_pose: Pose,
        z_offset: float,
        xy_offset: tuple[float, float] = (0.0, 0.0),
        slerp_fraction: float = 1.0,
    ) -> None:
        orientation = gripper_orientation_for_alignment(
            port_pose=port_pose,
            plug_pose=plug_pose,
            current_gripper_pose=gripper_pose,
            slerp_fraction=slerp_fraction,
        )
        plug_tip_target = (
            port_pose.position.x + xy_offset[0],
            port_pose.position.y + xy_offset[1],
            port_pose.position.z + z_offset,
        )
        gripper_target = target_gripper_pose_for_plug_tip(
            plug_tip_target_xyz=plug_tip_target,
            gripper_pose=gripper_pose,
            plug_pose=plug_pose,
            orientation=orientation,
        )
        self.get_logger().info(
            f"[command:{profile.name}] plug_xy=({plug_pose.position.x:.4f},{plug_pose.position.y:.4f}) "
            f"target_plug_xy=({plug_tip_target[0]:.4f},{plug_tip_target[1]:.4f}) "
            f"target_gripper_xy=({gripper_target.position.x:.4f},{gripper_target.position.y:.4f}) "
            f"target_plug_z={plug_tip_target[2]:.4f}",
            throttle_duration_sec=1.0,
        )
        cmd: MotionUpdate = make_motion_update(
            pose=gripper_target,
            profile=profile,
            stamp=self.time_now().to_msg(),
        )
        try:
            move_robot(motion_update=cmd)
        except Exception as ex:
            self.get_logger().warn(f"move_robot failed in {profile.name}: {ex}")
