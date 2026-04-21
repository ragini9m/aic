# Artifact #2 — Impedance State-Machine Base Policy

This doc describes the second artifact of the `aic_my_policy` package: an impedance-controlled state-machine that performs the cable insertion task. It currently uses **ground-truth TF** as its pose source (via a pluggable estimator interface) so we can validate the control loop in isolation before the learned vision estimator is ready.

> See the package [README](../README.md) for overall project status and artifact order.

---

## Goal

Produce a **submittable, non-learning baseline** that:

1. Passes Tier 1 (valid `aic_model` with valid robot commands) on all 3 qualification trials.
2. Scores **at or above CheatCode's Tier 3 (~60 pts/trial)** on all 3 trials, with lower force/jerk penalties thanks to impedance compliance.
3. Is structured so the pose source can be swapped from GT TF → learned vision estimator (artifacts #3–#4) **without touching the policy code**.

This is the first "real" policy. Subsequent artifacts (vision estimator, residual RL) add learning on top; nothing below the estimator abstraction is expected to change.

---

## Design rationale

### Why a state machine?

Contact-rich insertion is fundamentally multi-phase: free-space motion, fine alignment, touching, search, and seating each want **different control gains and feedforward force**. A single global PD gain is either too stiff (slams the port, force penalty) or too soft (never lands, proximity penalty only). A state machine lets each phase use the right compliance profile.

### Why impedance (not pure position or pure PID)?

The competition-provided `aic_controller` already implements cartesian impedance control with full 6×6 stiffness/damping matrices and a feedforward wrench. We leverage it rather than rolling our own PID. Per-state stiffness profiles turn this into "hybrid force/position control with compliance selection" — the textbook approach for peg-in-hole tasks.

### Why an abstract `PortPoseEstimator`?

The control loop doesn't care where port and plug poses come from — only that they arrive in `base_link` at ~20 Hz. Making this an abstract interface means:

- Bring-up today: back it with `/tf` (GT at `ground_truth:=true` launch).
- Submission tomorrow: swap in a CNN keypoint + PnP estimator that consumes `observation.*_image`.
- Residual RL later: feed the same estimator output into the actor, and optionally wrap the estimator with a Kalman filter or keep a learned prior.

No policy code changes across any of those swaps.

---

## Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                     aic_my_policy/ros/InsertCablePolicy            │
│                                                                    │
│   ┌─────────────────┐     ┌──────────────────┐                     │
│   │ State machine   │────▶│ Per-state        │   make_motion_      │
│   │ APPROACH ─▶     │     │ ImpedanceProfile │   update(...)       │
│   │ ALIGN    ─▶     │     │ (Kp, Kd, ff_F)   │──────────────┐      │
│   │ SEARCH   ─▶     │     └──────────────────┘              │      │
│   │ SEAT     ─▶     │                                       ▼      │
│   │ VERIFY          │     ┌──────────────────┐      ┌──────────┐   │
│   └────────┬────────┘     │ geometry:        │      │ Motion   │   │
│            │              │ - gripper orient │      │ Update   │   │
│            │              │   alignment      │      └────┬─────┘   │
│            ▼              │ - spiral sweep   │           │         │
│   ┌─────────────────┐     └──────────────────┘           │         │
│   │ get_observation │                                    │         │
│   └────────┬────────┘                                    │         │
│            ▼                                             │         │
│   ┌─────────────────────────────────────┐                │         │
│   │ PortPoseEstimator (abstract)        │                │         │
│   │   .get_port_pose(obs) -> Pose       │                │         │
│   │   .get_plug_tip_pose(obs) -> Pose   │                │         │
│   └─────────────────┬───────────────────┘                │         │
│                     │                                    │         │
│          ┌──────────┴──────────┐                         │         │
│          ▼                     ▼                         │         │
│    GroundTruth…      (future) Vision…                    │         │
│    (/tf backed)      (CNN + PnP)                         │         │
└──────────────────────────────────────────────────────────┼─────────┘
                                                           │
                                                           ▼
                                             /aic_controller/pose_commands
```

---

## State machine

All z-offsets are applied to the **port position** in `base_link` (+z is up). The gripper target is back-computed from the plug-tip target so that the **plug tip** (not the TCP) lands where we ask.

| State | Exit condition | Plug-tip z-target (relative to port.z) | Stiffness (x,y,z,rx,ry,rz) | Feedforward force |
|---|---|---|---|---|
| **APPROACH** | 4 s elapsed | +8 cm | 90,90,90, 50,50,50 | 0 |
| **ALIGN** | 3 s elapsed | +2 cm | 90,90,90, 50,50,50 | 0 |
| **SEARCH** | z < −2 mm OR 15 s elapsed | ramps: +5 mm → below entrance at 4 mm/s, with up-to-3 mm spiral on xy | **30,30,90**, 50,50,50 | (0, 0, **−3 N**) |
| **SEAT** | z ≤ −15 mm OR 8 s elapsed | ramps from −2 mm toward −15 mm at 10 mm/s | **20,20,90**, 50,50,50 | (0, 0, **−8 N**) |
| **VERIFY** | 2 s hold | — | (last command held) | 0 |

Total worst-case budget: 4 + 3 + 15 + 8 + 2 = **32 s**, well under the 180 s per-trial limit.

### Orientation handling

At every tick, we compute the gripper orientation that aligns the **grasped plug** with the **port** (rotation that maps plug → port, applied to the current gripper quaternion). During APPROACH, we **slerp** from the robot's current orientation to this target over ~3 s to avoid jerky wrist motion. From ALIGN onward the slerp fraction is 1.0 (full alignment).

### Spiral sweep

During SEARCH we superimpose an Archimedean-style sweep on the plug-tip xy target:

```
r(t) = r_max * min(1, t / 5 s)          # ramps up over 5 s
θ(t) = 2π * t / 2 s                     # 2 s per full loop
dx = r(t) cos θ(t),  dy = r(t) sin θ(t)
```

Default `r_max = 3 mm` (tunable in `control/geometry.py:spiral_xy_offset`).

### Lateral compliance

Key intuition for SEARCH/SEAT: lowering lateral (x, y) stiffness from 90 → 30 → 20 N/m lets the passive mechanical geometry of the port chamfer guide the plug inward once we're close enough, rather than the robot fighting against the port walls.

---

## File layout

```
aic_my_policy/aic_my_policy/
├── estimators/
│   ├── base.py                # PortPoseEstimator (ABC)
│   └── ground_truth.py        # /tf-backed implementation
├── control/
│   ├── impedance.py           # ImpedanceProfile + make_motion_update()
│   └── geometry.py            # orientation alignment + spiral sweep
└── ros/
    └── InsertCablePolicy.py   # state machine + 20 Hz loop
```

### `estimators/base.py`

```python
class PortPoseEstimator(ABC):
    def initialize(self, task: Task) -> bool: ...
    def get_port_pose(self, observation) -> Optional[Pose]: ...
    def get_plug_tip_pose(self, observation) -> Optional[Pose]: ...
```

`initialize()` runs once per trial (resolve frames, warm up the network, wait for tf). The `get_*` methods run at 20 Hz.

### `estimators/ground_truth.py`

`GroundTruthPortPoseEstimator` resolves TF frame names from the incoming `Task` message:

- Port: `task_board/{target_module_name}/{port_name}_link`
- Plug tip: `{cable_name}/{plug_name}_link`

and looks them up in the node's TF buffer. Waits up to 10 s at trial start for frames to appear; aborts the trial otherwise.

### `control/impedance.py`

`ImpedanceProfile` is a frozen dataclass holding the per-state numbers:

```python
@dataclass(frozen=True)
class ImpedanceProfile:
    stiffness_diag: Sequence[float]    # 6
    damping_diag:   Sequence[float]    # 6
    feedforward_force:  Sequence[float]   # 3, base_link
    feedforward_torque: Sequence[float] = (0,0,0)
    wrench_feedback_gains: Sequence[float] = (0.5,0.5,0.5,0,0,0)
```

Constants `APPROACH`, `ALIGN`, `SEARCH`, `SEAT` are exported. `make_motion_update(pose, profile, stamp)` assembles a full `MotionUpdate` message.

### `control/geometry.py`

Pure math:

- `gripper_orientation_for_alignment(port, plug, current_gripper, slerp_fraction)` — quaternion alignment with slerp.
- `target_gripper_pose_for_plug_tip(plug_tip_target_xyz, gripper, plug, orientation)` — back-compute TCP target so plug tip lands where requested.
- `spiral_xy_offset(elapsed_s, period_s, radius_mm, ramp_s)` — Archimedean sweep helper.

### `ros/InsertCablePolicy.py`

Flat while-loop state machine at 20 Hz. No async, no threads (the harness already runs `insert_cable()` in its own thread). All state transitions are time- or position-based, so the loop is fully deterministic given observations.

---

## Configuration knobs

All tuning is one edit away, grouped by file:

### `control/impedance.py` — per-state gains & forces

| Symptom | Knob | Direction |
|---|---|---|
| Force penalty (>20 N sustained) | `SEARCH.feedforward_force.z`, `SEAT.feedforward_force.z` | smaller magnitude |
| Plug bounces off lip | `SEARCH.feedforward_force.z` | larger magnitude |
| Plug tilts during descent | angular stiffness (last 3 of `stiffness_diag`) | larger |
| Chatter / oscillation | `*_damping_diag` | larger |

### `control/geometry.py` — spiral sweep

| Symptom | Knob | Direction |
|---|---|---|
| Plug misses hole laterally | `spiral_xy_offset(radius_mm=...)` | larger (try 5) |
| Spiral too fast, hits walls | `spiral_xy_offset(period_s=...)` | larger (try 3) |

### `ros/InsertCablePolicy.py` — state budgets and geometry

| Knob | Default | When to tune |
|---|---|---|
| `APPROACH_Z_OFFSET` | 0.08 | Robot moves up too far before descending → reduce |
| `APPROACH_BUDGET_S` | 4.0 | Orientation not converged → increase |
| `ALIGN_BUDGET_S` | 3.0 | Final alignment shaky → increase |
| `SEARCH_BUDGET_S` | 15.0 | Plug always times out without seating → increase |
| `SEAT_Z_DEPTH` | −0.015 | Score shows partial insertion → push deeper (more negative) |
| `SEARCH_DESCENT_M_PER_S` | 0.004 | Too slow / too aggressive | match to port depth |
| `LOOP_HZ` | 20.0 | Observations arrive at 20 Hz; match that |

---

## Running

Same 3-terminal recipe as the package README. **Must launch with `ground_truth:=true`** while we're on the GT estimator.

```bash
# Terminal 0 — Zenoh router
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd

# Terminal 1 — Evaluation environment (GT required for this estimator)
distrobox enter -r aic_eval -- /entrypoint.sh ground_truth:=true start_aic_engine:=true

# Terminal 2 — Policy
cd ~/ws_aic/src/aic
pixi reinstall ros-kilted-aic-my-policy
pixi run ros2 run aic_model aic_model --ros-args \
    -p use_sim_time:=true \
    -p policy:=aic_my_policy.ros.InsertCablePolicy
```

---

## Expected output

### aic_model log

```
Loading policy module: aic_my_policy.ros.InsertCablePolicy
Using policy: InsertCablePolicy
InsertCablePolicy ready (GT estimator).
insert_cable() task=Task(..., port_name='sfp_port_0', ...)
[GT estimator] port_frame=task_board/nic_card_mount_0/sfp_port_0_link plug_frame=sfp_sc_cable/sfp_module_link
(feedback) APPROACH
(feedback) ALIGN
(feedback) SEARCH
(feedback) SEAT
(feedback) VERIFY
insert_cable() exit state=DONE success=True
```

### Scoring (`~/aic_results/.../scoring.yaml`)

Target, per trial:

| Tier | Metric | Expected |
|---|---|---|
| 1 | Model validity | **1** |
| 2 | Trajectory smoothness | 3–6 |
| 2 | Task duration | 6–10 (30 s nominal) |
| 2 | Trajectory efficiency | 3–6 |
| 2 | Insertion force penalty | **0** (no penalty) |
| 2 | Off-limit contact penalty | **0** (no penalty) |
| 3 | Cable insertion | 50–75 (partial or full) |

Full insertion target: **75 + 3 + 10 + 3 = ~91 / 100 per trial** if the stiffness tuning is right. CheatCode's reported baseline is ~60 Tier 3; we should match or exceed with the bonus of no force penalty.

---

## Known limitations / TODOs

- **GT dependency.** `GroundTruthPortPoseEstimator` requires `ground_truth:=true` and will **not work at evaluation** (Zenoh ACL blocks the relevant `/tf` frames). Swap it before submitting.
- **No recovery behavior.** If SEARCH times out without seating, we still transition to SEAT and push, which may cause a wasted trial. A future addition: jump back to ALIGN, refine estimate, retry SEARCH.
- **Fixed insertion axis (world −z).** We assume port roll/pitch are zero, so insertion is always along world −z. The competition guide confirms this for qualification, but Phase 1 / Phase 2 may need a general-axis formulation.
- **No force-based state transitions yet.** SEARCH → SEAT is purely geometric. A richer version would read `observation.wrist_wrench` and transition when a compressive force plateau appears (the plug has found the lip).
- **`wrench_feedback_gains` left at `[0.5, 0.5, 0.5, 0, 0, 0]` everywhere.** This is a reasonable default but may need per-state tuning once we see real F/T data.

---

## How to swap in the vision estimator (artifact #4 preview)

Once the learned estimator exists:

```python
# in InsertCablePolicy.__init__
from aic_my_policy.estimators.vision import VisionPortPoseEstimator
self._estimator = VisionPortPoseEstimator(parent_node, weights_path=...)
```

Everything else stays identical. The only other thing that changes is the launch flag: `ground_truth:=false` becomes the default, and eventually we drop `ground_truth=true` entirely.

---

## Next artifact

**Artifact #3** — `aic_my_policy/data_collection/`, a Gazebo-driven script that:

- Launches `aic_bringup` repeatedly with randomized task board / NIC rail / SC port parameters (`ground_truth:=true`).
- Subscribes to `/left_camera/image`, `/center_camera/image`, `/right_camera/image`, their `CameraInfo`, and `/tf`.
- After each scene settles, captures one multi-camera tuple and writes an HDF5 row: `(image_l, image_c, image_r, K_l, K_c, K_r, port_pose_base_link, port_id)`.

The resulting dataset is the supervised training signal for artifact #4 (keypoint CNN + PnP).
