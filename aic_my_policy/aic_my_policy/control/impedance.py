"""MotionUpdate builders for per-state impedance profiles.

`aic_controller` accepts a full 6x6 stiffness + damping matrix and a
feedforward wrench on every command. The state machine switches
between profiles to implement compliant insertion:

  APPROACH / ALIGN  - stiff in all 6 DoF, no feedforward force
  SEARCH            - soft laterally, stiff axially, small push down
  SEAT              - softer laterally, stiff axially, larger push down
"""

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
from geometry_msgs.msg import Pose, Vector3, Wrench
from std_msgs.msg import Header


@dataclass(frozen=True)
class ImpedanceProfile:
    """Diagonal stiffness/damping + feedforward wrench for one state."""

    name: str
    stiffness_diag: Sequence[float]          # len 6: x y z rx ry rz
    damping_diag: Sequence[float]            # len 6
    feedforward_force: Sequence[float]       # len 3, base_link frame
    feedforward_torque: Sequence[float] = (0.0, 0.0, 0.0)
    wrench_feedback_gains: Sequence[float] = (0.5, 0.5, 0.5, 0.0, 0.0, 0.0)


# --- Default profiles ------------------------------------------------------
#
# Tuning notes:
# - Positional stiffness is N/m, angular is Nm/rad.
# - Keep angular stiffness high throughout so the plug doesn't tilt.
# - Lowering lateral (x,y) stiffness during SEARCH lets contact with the
#   port lip passively guide the plug into the hole.
# - Feedforward force.z is negative to push the TCP downward in base_link.
#   (base_link +z is up.) Keep magnitude under the 20 N penalty threshold.

APPROACH = ImpedanceProfile(
    name="APPROACH",
    stiffness_diag=(90.0, 90.0, 90.0, 50.0, 50.0, 50.0),
    damping_diag=(50.0, 50.0, 50.0, 20.0, 20.0, 20.0),
    feedforward_force=(0.0, 0.0, 0.0),
)

ALIGN = ImpedanceProfile(
    name="ALIGN",
    stiffness_diag=(90.0, 90.0, 90.0, 50.0, 50.0, 50.0),
    damping_diag=(50.0, 50.0, 50.0, 20.0, 20.0, 20.0),
    feedforward_force=(0.0, 0.0, 0.0),
)

SEARCH = ImpedanceProfile(
    name="SEARCH",
    stiffness_diag=(30.0, 30.0, 90.0, 50.0, 50.0, 50.0),
    damping_diag=(30.0, 30.0, 50.0, 20.0, 20.0, 20.0),
    feedforward_force=(0.0, 0.0, -3.0),
)

SEAT = ImpedanceProfile(
    name="SEAT",
    stiffness_diag=(20.0, 20.0, 90.0, 50.0, 50.0, 50.0),
    damping_diag=(25.0, 25.0, 50.0, 20.0, 20.0, 20.0),
    feedforward_force=(0.0, 0.0, -8.0),
)


def make_motion_update(
    pose: Pose,
    profile: ImpedanceProfile,
    stamp,
    frame_id: str = "base_link",
) -> MotionUpdate:
    return MotionUpdate(
        header=Header(frame_id=frame_id, stamp=stamp),
        pose=pose,
        target_stiffness=np.diag(profile.stiffness_diag).flatten(),
        target_damping=np.diag(profile.damping_diag).flatten(),
        feedforward_wrench_at_tip=Wrench(
            force=Vector3(
                x=profile.feedforward_force[0],
                y=profile.feedforward_force[1],
                z=profile.feedforward_force[2],
            ),
            torque=Vector3(
                x=profile.feedforward_torque[0],
                y=profile.feedforward_torque[1],
                z=profile.feedforward_torque[2],
            ),
        ),
        wrench_feedback_gains_at_tip=list(profile.wrench_feedback_gains),
        trajectory_generation_mode=TrajectoryGenerationMode(
            mode=TrajectoryGenerationMode.MODE_POSITION,
        ),
    )
