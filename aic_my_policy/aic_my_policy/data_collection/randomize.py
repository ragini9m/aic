"""Random scene-configuration generator for data collection.

Each call to `sample_scene_config()` returns a dict of launch args that
can be forwarded to `ros2 launch aic_bringup aic_gz_bringup.launch.py`.
Randomization stays within the limits documented in
`docs/task_board_description.md` and `docs/qualification_phase.md`.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, asdict
from typing import Literal

# Randomization limits per challenge_rules / task_board_description.
NIC_TRANSLATION_RANGE = (0.0, 0.062)
NIC_YAW_RANGE_DEG = (-10.0, 10.0)
SC_PORT_TRANSLATION_RANGE = (0.0, 0.115)

# Task board pose — roughly the union of the two qualification trial poses.
TASK_BOARD_X_RANGE = (0.10, 0.22)
TASK_BOARD_Y_RANGE = (-0.25, 0.05)
TASK_BOARD_Z = 1.14                  # fixed per qualification trials
TASK_BOARD_YAW_RANGE = (2.8, math.pi + 0.3)

CableKind = Literal["sfp_sc_cable", "sfp_sc_cable_reversed"]


@dataclass
class SceneConfig:
    task_board_x: float
    task_board_y: float
    task_board_z: float
    task_board_yaw: float
    nic_card_mount_0_present: bool
    nic_card_mount_0_translation: float
    nic_card_mount_0_yaw: float
    nic_card_mount_1_present: bool
    nic_card_mount_1_translation: float
    nic_card_mount_1_yaw: float
    sc_port_0_present: bool
    sc_port_0_translation: float
    sc_port_1_present: bool
    sc_port_1_translation: float
    cable_type: CableKind

    def as_launch_args(self) -> list[str]:
        args = []
        for key, value in asdict(self).items():
            if isinstance(value, bool):
                args.append(f"{key}:={'true' if value else 'false'}")
            else:
                args.append(f"{key}:={value}")
        args.append("spawn_task_board:=true")
        args.append("spawn_cable:=true")
        args.append("attach_cable_to_gripper:=true")
        args.append("ground_truth:=true")
        args.append("start_aic_engine:=false")
        return args


def _u(lo: float, hi: float) -> float:
    return random.uniform(lo, hi)


def sample_scene_config(seed: int | None = None, trial_kind: str = "random") -> SceneConfig:
    """Sample a randomized scene.

    trial_kind:
      'sfp'    - NIC card present, SFP end grasped (Trial 1/2 analogue)
      'sc'     - SC port present, SC end grasped (Trial 3 analogue)
      'random' - 50/50 split
    """
    if seed is not None:
        random.seed(seed)
    if trial_kind == "random":
        trial_kind = random.choice(["sfp", "sc"])

    if trial_kind == "sfp":
        rail = random.choice([0, 1])
        nic0 = rail == 0
        nic1 = rail == 1
        return SceneConfig(
            task_board_x=_u(*TASK_BOARD_X_RANGE),
            task_board_y=_u(*TASK_BOARD_Y_RANGE),
            task_board_z=TASK_BOARD_Z,
            task_board_yaw=_u(*TASK_BOARD_YAW_RANGE),
            nic_card_mount_0_present=nic0,
            nic_card_mount_0_translation=_u(*NIC_TRANSLATION_RANGE) if nic0 else 0.0,
            nic_card_mount_0_yaw=math.radians(_u(*NIC_YAW_RANGE_DEG)) if nic0 else 0.0,
            nic_card_mount_1_present=nic1,
            nic_card_mount_1_translation=_u(*NIC_TRANSLATION_RANGE) if nic1 else 0.0,
            nic_card_mount_1_yaw=math.radians(_u(*NIC_YAW_RANGE_DEG)) if nic1 else 0.0,
            sc_port_0_present=False,
            sc_port_0_translation=0.0,
            sc_port_1_present=False,
            sc_port_1_translation=0.0,
            cable_type="sfp_sc_cable",
        )

    # SC trial
    rail = random.choice([0, 1])
    return SceneConfig(
        task_board_x=_u(*TASK_BOARD_X_RANGE),
        task_board_y=_u(*TASK_BOARD_Y_RANGE),
        task_board_z=TASK_BOARD_Z,
        task_board_yaw=_u(*TASK_BOARD_YAW_RANGE),
        nic_card_mount_0_present=False,
        nic_card_mount_0_translation=0.0,
        nic_card_mount_0_yaw=0.0,
        nic_card_mount_1_present=False,
        nic_card_mount_1_translation=0.0,
        nic_card_mount_1_yaw=0.0,
        sc_port_0_present=rail == 0,
        sc_port_0_translation=_u(*SC_PORT_TRANSLATION_RANGE) if rail == 0 else 0.0,
        sc_port_1_present=rail == 1,
        sc_port_1_translation=_u(*SC_PORT_TRANSLATION_RANGE) if rail == 1 else 0.0,
        cable_type="sfp_sc_cable_reversed",
    )
