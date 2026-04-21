"""Geometry helpers: port/plug alignment + lateral search patterns."""

import math

from geometry_msgs.msg import Pose, Point, Quaternion
from transforms3d._gohlketransforms import quaternion_multiply, quaternion_slerp

QuatWxyz = tuple[float, float, float, float]


def _pose_q(pose: Pose) -> QuatWxyz:
    return (pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)


def _q_inv(q: QuatWxyz) -> QuatWxyz:
    # Unit-quaternion inverse = conjugate.
    return (q[0], -q[1], -q[2], -q[3])


def gripper_orientation_for_alignment(
    port_pose: Pose,
    plug_pose: Pose,
    current_gripper_pose: Pose,
    slerp_fraction: float = 1.0,
) -> Quaternion:
    """Gripper orientation that aligns the grasped plug with the port.

    Computes the rotation that maps the current plug orientation to the
    port orientation, applies it to the current gripper orientation, and
    optionally slerps from current toward that target.
    """
    q_port = _pose_q(port_pose)
    q_plug = _pose_q(plug_pose)
    q_gripper = _pose_q(current_gripper_pose)

    q_diff = quaternion_multiply(q_port, _q_inv(q_plug))
    q_target = quaternion_multiply(q_diff, q_gripper)
    q_slerp = quaternion_slerp(q_gripper, q_target, slerp_fraction)
    return Quaternion(w=q_slerp[0], x=q_slerp[1], y=q_slerp[2], z=q_slerp[3])


def plug_to_gripper_offset(
    gripper_pose: Pose,
    plug_pose: Pose,
) -> tuple[float, float, float]:
    """Vector from the plug tip to the gripper TCP, in base_link.

    Used to back-compute where the gripper must be so the plug tip lands
    at a desired base_link point (since the policy commands TCP, not plug).
    """
    return (
        gripper_pose.position.x - plug_pose.position.x,
        gripper_pose.position.y - plug_pose.position.y,
        gripper_pose.position.z - plug_pose.position.z,
    )


def target_gripper_pose_for_plug_tip(
    plug_tip_target_xyz: tuple[float, float, float],
    gripper_pose: Pose,
    plug_pose: Pose,
    orientation: Quaternion,
) -> Pose:
    """Gripper pose such that the plug tip reaches the requested xyz."""
    dx, dy, dz = plug_to_gripper_offset(gripper_pose, plug_pose)
    return Pose(
        position=Point(
            x=plug_tip_target_xyz[0] + dx,
            y=plug_tip_target_xyz[1] + dy,
            z=plug_tip_target_xyz[2] + dz,
        ),
        orientation=orientation,
    )


def spiral_xy_offset(
    elapsed_s: float,
    period_s: float = 2.0,
    radius_mm: float = 3.0,
    ramp_s: float = 5.0,
) -> tuple[float, float]:
    """Archimedean-style spiral sweep in meters.

    Radius ramps linearly from 0 to `radius_mm` over `ramp_s`, then holds.
    Superimpose on the desired port xy during SEARCH to probe the hole.
    """
    r_max = radius_mm * 1e-3
    r = r_max * min(1.0, elapsed_s / ramp_s)
    omega = 2.0 * math.pi / period_s
    return (r * math.cos(omega * elapsed_s), r * math.sin(omega * elapsed_s))
