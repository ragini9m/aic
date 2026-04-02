# AI for Industry Challenge (AIC) — Complete Competition Guide

> **Hosted by:** Intrinsic (a Google company)  
> **Prize Pool:** $180,000 (Top 5 teams)  
> **Duration:** March 2, 2026 – September 8, 2026  
> **Website:** aiforindustrychallenge.ai

---

## Table of Contents

1. [What Is This Competition?](#1-what-is-this-competition)
2. [Timeline & Phases](#2-timeline--phases)
3. [What You Need To Do](#3-what-you-need-to-do)
4. [Scoring System](#4-scoring-system)
5. [Rules & Prohibited Actions](#5-rules--prohibited-actions)
6. [Technical Requirements](#6-technical-requirements)
7. [ROS 2 Interfaces (Inputs & Outputs)](#7-ros-2-interfaces-inputs--outputs)
8. [Trial Configurations](#8-trial-configurations)
9. [Submission Process](#9-submission-process)
10. [Tools & Frameworks](#10-tools--frameworks)
11. [Example Policies](#11-example-policies)
12. [Environment Setup Checklist](#12-environment-setup-checklist)
13. [Critical Warnings & Gotchas](#13-critical-warnings--gotchas)
14. [Repository Structure](#14-repository-structure)
15. [Key Files to Know](#15-key-files-to-know)

---

## 1. What Is This Competition?

The AI for Industry Challenge targets a critical unsolved bottleneck in modern manufacturing: **dexterous cable management in electronics assembly**.

**Your task:** Build an AI-powered robotic policy that can autonomously **insert fiber optic cables** (SFP modules, SC plugs) **into networking hardware** (NIC cards, optical patch panels) using a simulated **Universal Robots UR5e** arm.

**Why it's hard:**
- Complex physics of flexible cables
- Millimeter-level precision required for connector alignment
- Sim-to-real transfer challenges
- Need for robust force feedback control during insertion

The competition asks you to bridge the "sim-to-real gap" — train policies in simulation that eventually work on a real workcell.

---

## 2. Timeline & Phases

| Phase | Dates | Goal | Who Advances |
|---|---|---|---|
| **Qualification** | Mar 2 – May 15, 2026 | Train and submit a simulation policy | Top 30 teams |
| Qualification Evaluation | May 18 – 27, 2026 | Cloud scoring of submissions | — |
| Top 30 Announced | May 28, 2026 | — | — |
| **Phase 1** | May 28 – Jul 14, 2026 | Develop in Intrinsic Flowstate with Vision Model | Top 10 teams |
| Phase 1 Evaluation | Jul 14 – 21, 2026 | — | — |
| Top 10 Announced | Jul 22, 2026 | — | — |
| **Phase 2** | Jul 27 – Aug 25, 2026 | Deploy on real robot at Intrinsic HQ | Prize winners |
| Phase 2 Evaluation | Aug 26 – Sep 4, 2026 | — | — |
| **Winner Announced** | Sep 8, 2026 | — | — |

> Phase 1 and Phase 2 details are marked "Coming Soon" in current docs.

---

## 3. What You Need To Do

### Qualification Phase (Current Active Phase)

#### 3.1 Implement a ROS 2 Lifecycle Node

Create a node named **`aic_model`** that:

- Starts in `unconfigured` state
- Transitions through: `configured` -> `active` -> `deactivate` -> `cleanup` -> `shutdown`
- Each lifecycle transition must complete within **60 seconds**
- In `active` state, accepts goals on the `/insert_cable` action server

**State behavior requirements:**

| State | Required Behavior |
|---|---|
| `unconfigured` | No robot commands published; no topics published |
| `configured` | No robot commands; MUST reject any `/insert_cable` action goals |
| `active` | MUST accept `/insert_cable` goals; goals must be cancellable |
| `deactivated` | Returns to `configured` behavior within 60s |
| `shutdown` | No publishers in the ROS graph; no topics published |

#### 3.2 Implement the `insert_cable()` Policy Method

Your Python class inherits from `aic_model.Policy` and implements:

```python
def insert_cable(self, task, get_observation, move_robot, send_feedback):
    # task: target port info and time limit
    # get_observation(): returns latest Observation (call up to 20 Hz)
    # move_robot(update): sends MotionUpdate or JointMotionUpdate
    # send_feedback(string): publishes debug feedback
    ...
```

- Reads sensor data: camera images, joint states, F/T sensor, TCP pose/velocity
- Sends motion commands to the arm
- Returns when the task is complete or the `time_limit` expires (default: 180 seconds)

#### 3.3 Pass Three Trials

| Trial | Connector | Target | What It Tests |
|---|---|---|---|
| **Trial 1** | SFP Module | `SFP_PORT_0` on NIC card (rail 0) | Basic insertion |
| **Trial 2** | SFP Module | `SFP_PORT_0` on NIC card (rail 1) | Generalization across rails |
| **Trial 3** | SC Plug | SC port | Generalization across connector types |

#### 3.4 Package and Submit

Containerize your policy with Docker, push to AWS ECR, and register via the submission portal.

---

## 4. Scoring System

**Maximum score per trial: 100 points.** Final ranking is by cumulative total across all trials.

### Tier 1: Model Validity (0 or 1 point)

This is a prerequisite gate — if this fails, no other scoring applies.

| Outcome | Score |
|---|---|
| Node activates, responds to `/insert_cable`, sends valid robot commands | **1** |
| Any failure | **0** |

### Tier 2: Performance & Convergence (+24 to -36 points)

> **IMPORTANT:** All Tier 2 **positive** scores are only awarded if Tier 3 score > 0 (plug must be near or in the port).

**Positive Metrics:**

| Metric | Points | How It's Calculated |
|---|---|---|
| Trajectory Smoothness | 0–6 | Inversely proportional to jerk (m/s^3). 0 jerk = 6 pts, >=50 m/s^3 = 0 pts |
| Task Duration | 0–12 | Inversely proportional to time. <=5s = 12 pts, >=60s = 0 pts |
| Trajectory Efficiency | 0–6 | Inversely proportional to path length. Shortest path = 6 pts, >=1m extra = 0 pts |

**Penalties:**

| Penalty | Points | Trigger |
|---|---|---|
| Insertion Force | 0 to **-12** | Force > 20N sustained for > 1 second |
| Off-Limit Contact | 0 to **-24** | Any robot link contacts enclosure, enclosure walls, or task board |

**Off-limit models** (robot must NOT touch):
- `enclosure` — floor, corner posts, ceiling structural frame
- `enclosure walls` — transparent acrylic panels
- `task_board` — the board and all mounted components

> Note: The cable itself does NOT trigger the off-limit penalty.

### Tier 3: Task Success (-12 to 75 points)

| Outcome | Score |
|---|---|
| Correct port insertion (verified by contact sensors) | **75 points** |
| Wrong port insertion | **-12 points** |
| Partial insertion (plug in bounding box, within 5mm x-y tolerance) | **38–50 points** (proportional to depth) |
| Proximity (plug near port but not inserted) | **0–25 points** (inversely proportional to distance) |
| Outside max acceptable distance | **0 points** |

Max acceptable distance = half the initial plug-to-port distance.

### Score Range Summary

```
Best case per trial:  1 + 6 + 12 + 6 + 75           = 100 points
Worst case per trial: 0 + 0 + 0 + 0 - 12 - 24 - 12  = -48 points
```

---

## 5. Rules & Prohibited Actions

### What's Prohibited

- Direct state manipulation: teleporting components, forcing insertions programmatically
- Accessing `/scoring`, `/gazebo`, `/gz_server` namespaces
- Calling entity management services (spawn, despawn, delete models)
- Accessing `/clock`, `/model`, `/world_stats`, `/pause_physics`, simulation reset services
- Reverse-engineering or tampering with cloud evaluation infrastructure
- Submitting containers with malicious code or backdoors
- Using poses/states from the evaluation environment not accessible through official interfaces

### What's Allowed

- Any training approach: RL, imitation learning, classical control, hybrid
- Any simulator: MuJoCo, Isaac Lab, Gazebo, O3DE
- Teleoperation for data collection
- Pre-trained models (e.g., ACT from HuggingFace)
- During **training**: ALL internal state information including ground truth from `/tf`

### Enforcement

- Automated access control via **Zenoh ACL** blocks prohibited namespaces
- Container audits of top-performing teams
- Behavioral verification during live evaluation
- Metric anomaly detection

### Training vs. Evaluation Differences

| Feature | Training | Evaluation |
|---|---|---|
| Ground truth `/tf` data | Available | **Blocked** |
| F/T tare service | Available | **Disabled** (auto-tared before each trial) |
| Simulation internals | Accessible | **Blocked by Zenoh ACL** |

---

## 6. Technical Requirements

### Minimum Local Hardware

| Spec | Minimum |
|---|---|
| OS | Ubuntu 24.04 |
| CPU | 4–8 cores |
| RAM | 32 GB+ |
| GPU | NVIDIA RTX 2070+ or equivalent |
| VRAM | 8 GB+ |

### Cloud Evaluation Hardware (what your submission runs on)

| Spec | Value |
|---|---|
| vCPU | 64 cores |
| RAM | 256 GiB |
| GPU | 1x NVIDIA L4 Tensor Core |
| VRAM | 24 GiB |

### Robot & Sensor Specs

| Component | Detail |
|---|---|
| Robot | Universal Robots UR5e manipulator |
| Gripper | Robotiq Hand-E |
| F/T Sensor | Axia80 M20 (3D force + 3D torque) |
| Cameras | 3 wrist-mounted RGB cameras (left, center, right) |
| Observation rate | Up to 20 Hz |
| Controller command rate | 10–30 Hz (bridged to hardware at ~500 Hz) |

### Software Stack

| Tool | Role |
|---|---|
| ROS 2 Kilted Kaiju | Robot middleware |
| `rmw_zenoh_cpp` | Mandatory middleware (no DDS allowed) |
| Gazebo | Official evaluation simulator |
| Docker / Podman | Containerization |
| Distrobox | Integrate eval container with host |
| Pixi | Package & dependency management |
| AWS CLI | Upload to ECR |

### Required Environment Variables

```bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true;transport/shared_memory/transport_optimization/pool_size=536870912'
```

---

## 7. ROS 2 Interfaces (Inputs & Outputs)

### Inputs (Subscribe To)

| Topic | Message Type | Description |
|---|---|---|
| `/left_camera/image` | `sensor_msgs/Image` | Left wrist camera |
| `/center_camera/image` | `sensor_msgs/Image` | Center wrist camera |
| `/right_camera/image` | `sensor_msgs/Image` | Right wrist camera |
| `/left_camera/camera_info` | `sensor_msgs/CameraInfo` | Left camera calibration |
| `/center_camera/camera_info` | `sensor_msgs/CameraInfo` | Center camera calibration |
| `/right_camera/camera_info` | `sensor_msgs/CameraInfo` | Right camera calibration |
| `/fts_broadcaster/wrench` | `geometry_msgs/WrenchStamped` | 6-DOF force/torque |
| `/joint_states` | `sensor_msgs/JointState` | Robot arm joint states |
| `/gripper_state` | `sensor_msgs/JointState` | Gripper state |
| `/tf` | `tf2_msgs/TFMessage` | Dynamic transforms (ground truth in training only) |
| `/tf_static` | `tf2_msgs/TFMessage` | Static transforms |
| `/aic_controller/controller_state` | `aic_control_interfaces/ControllerState` | TCP pose, velocity, error, target torques |
| `/insert_cable` (action) | `aic_task_interfaces/action/InsertCable` | Task trigger from engine |

### Outputs (Publish To)

| Topic | Message Type | Description |
|---|---|---|
| `/aic_controller/pose_commands` | `aic_control_interfaces/MotionUpdate` | Cartesian-space targets |
| `/aic_controller/joint_commands` | `aic_control_interfaces/JointMotionUpdate` | Joint-space targets |

### Services (Call These)

| Service | Type | Description |
|---|---|---|
| `/aic_controller/change_target_mode` | `aic_control_interfaces/srv/ChangeTargetMode` | Switch Cartesian (mode 1) / joint (mode 2) |
| `/aic_controller/tare_ft_sensor` | `std_srvs/srv/Trigger` | Zero F/T sensor (DISABLED during evaluation) |

### Fused Observation (via `aic_adapter`)

All sensors are fused into one `Observation` message at 20 Hz:
- `left_image`, `center_image`, `right_image` (+ camera_info)
- `joint_states`
- `wrist_wrench`
- `controller_state` (TCP pose, velocity, error)

---

## 8. Trial Configurations

### Trial 1 — SFP on NIC Rail 0

- Task board: x=0.15, y=-0.2, z=1.14, yaw=pi
- NIC card on rail 0, translation=0.036m, yaw=0 deg
- Insert `sfp_tip` into `sfp_port_0` on `nic_card_mount_0`
- Time limit: 180 seconds
- Cable: `sfp_sc_cable` (robot holds SFP end)

### Trial 2 — SFP on NIC Rail 1

- Task board: same pose as Trial 1
- NIC card on rail 1 instead (no card on rail 0)
- Insert `sfp_tip` into `sfp_port_0` on `nic_card_mount_1`
- Time limit: 180 seconds
- Tests generalization across different rail positions

### Trial 3 — SC Plug Insertion

- Task board: x=0.17, y=0.0, z=1.14, yaw=3.0 rad
- No NIC cards; SC port on rail 1, translation=-0.055m
- Insert `sc_tip` into `sc_port_base` on `sc_port_1`
- Time limit: 180 seconds
- Cable: `sfp_sc_cable_reversed` (robot holds SC end)

### Randomization Limits

| Component | Translation Range | Rotation Range |
|---|---|---|
| NIC Card on rail | [0, 0.062] m | [-10 deg, +10 deg] yaw |
| SC Port on rail | [0, 0.115] m | Fixed yaw=0 at evaluation |
| Fixture mounts (Zones 3&4) | [0, 0.188] m | [-60 deg, +60 deg] |

> Task board roll/pitch are always 0. SC port yaw is always 0. Only task board position (x, y) and yaw are randomized.

---

## 9. Submission Process

### Step-by-Step

**Step 1: Build your Docker image**
```bash
docker compose -f docker/docker-compose.yaml build model
```

**Step 2: Verify locally (DO NOT SKIP)**
```bash
docker compose -f docker/docker-compose.yaml up
```
> A failed local run wastes your daily submission slot if pushed.

**Step 3: Authenticate with AWS ECR**
```bash
aws configure --profile <team_name>   # Use credentials from onboarding email
export AWS_PROFILE=<team_name>
aws ecr get-login-password --region us-east-1 | \
  docker login --username AWS --password-stdin \
  973918476471.dkr.ecr.us-east-1.amazonaws.com
```

**Step 4: Tag and push**
```bash
docker tag localhost/my-solution:v1 \
  973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/<team_name>:v1

docker push \
  973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/<team_name>:v1
```

**Step 5: Register on portal**
- Go to `aiforindustrychallenge.ai`
- Submit > Qualification phase > Paste OCI Image URI

**Step 6: Monitor**
- Check "My Submissions" on the portal
- Evaluation takes 5–15 minutes (Queued -> Running -> Finished)
- Results appear on the Leaderboard

### Submission Limits

- **1 submission per day** (team-wide, not per member)
- No total cap on submissions across the competition
- ECR image tags are **immutable** — use new version tags each time
- AWS login tokens expire every **12 hours** — reauthenticate if push fails

---

## 10. Tools & Frameworks

### Required

| Tool | Purpose |
|---|---|
| Docker / Podman | Containerize and submit your policy |
| Distrobox | Integrate `aic_eval` container with host system |
| Pixi | Package and dependency management |
| AWS CLI | Upload to ECR registry |
| ROS 2 Kilted | Robot middleware |
| `rmw_zenoh_cpp` | Mandatory ROS 2 middleware |
| Gazebo | Official evaluation simulator |

### Optional (Provided)

| Tool | Provider | Purpose |
|---|---|---|
| Isaac Lab | NVIDIA | Alternative training simulator |
| MuJoCo | Google DeepMind | Alternative training simulator |
| LeRobot | HuggingFace | Data collection & ACT policy training |
| PlotJuggler | Open source | ROS topic time-series visualization |
| RViz | ROS | Robot state visualization |

### Controller Modes

- **Cartesian mode (default, mode 1):** Send target pose or velocity in `base_link` or `gripper/tcp` frame
- **Joint mode (mode 2):** Send target joint positions or velocities
- Switch modes via `/aic_controller/change_target_mode` before sending commands of that type

---

## 11. Example Policies

| Policy | Purpose | Expected Score |
|---|---|---|
| `WaveArm` | Minimal API demo; arm waves, ignores task | Tier 3: 0 |
| `CheatCode` | Uses ground truth TF for perfect targeting (debug only, invalid for eval) | Tier 3: ~60 |
| `RunACT` | ACT transformer policy trained via LeRobot on collected demos | Partial |

---

## 12. Environment Setup Checklist

### Initial Setup
- [ ] Install Docker Engine + Linux post-installation steps
- [ ] Install Distrobox: `sudo apt install distrobox`
- [ ] Install Pixi: `curl -fsSL https://pixi.sh/install.sh | sh`
- [ ] Install NVIDIA Container Toolkit (if NVIDIA GPU)
- [ ] Set: `export DBX_CONTAINER_MANAGER=docker`

### Clone & Install
```bash
mkdir -p ~/ws_aic/src && cd ~/ws_aic/src
git clone https://github.com/intrinsic-dev/aic
cd ~/ws_aic/src/aic && pixi install
```

### Running the Evaluation Environment
```bash
# Pull eval container
docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest

# Create distrobox
distrobox create -r --nvidia -i ghcr.io/intrinsic-dev/aic/aic_eval:latest aic_eval

# Enter and start
distrobox enter -r aic_eval
/entrypoint.sh ground_truth:=false start_aic_engine:=true
```

### Policy Development
- [ ] Create Python class inheriting from `aic_model.Policy`
- [ ] Implement `insert_cable()` method
- [ ] Test with WaveArm and CheatCode baselines to understand scoring
- [ ] Use `ground_truth:=true` for development debugging
- [ ] Tare F/T sensor before each training episode:
  ```bash
  ros2 service call /aic_controller/tare_ft_sensor std_srvs/srv/Trigger
  ```
- [ ] Test generalization across all three trial configurations

---

## 13. Critical Warnings & Gotchas

1. **Tier 2 scores are conditional.** Smoothness, duration, and efficiency points are ONLY awarded if Tier 3 > 0 (plug must be near the port). You cannot score Tier 2 without making progress on insertion.

2. **Ground truth TF is blocked during evaluation.** The `ground_truth:=true` flag and `/tf` ground truth data are only for local development. Your submission cannot rely on these.

3. **F/T tare service is disabled during evaluation.** The system auto-tares before each trial. Do not call `/aic_controller/tare_ft_sensor` in your submission.

4. **Always verify locally before submitting.** A failed container wastes your daily submission slot.

5. **ECR tags are immutable.** Every push needs a new tag. Use version numbers or commit SHAs (e.g., `:v1`, `:v2`, `:abc123`).

6. **Zenoh router must start before your model node.** The engine times out after 30 seconds waiting for `aic_model`.

7. **Controller tracking error auto-reset.** If the robot collides while commands are being sent, the controller accumulates tracking error and resets when free — this causes sudden jerky motion. Be aware during teleoperation.

8. **NVIDIA RTX 50xx cards need a PyTorch override.** Add `torch = ">=2.7.1"` to `pixi.toml` for RTX 5090 cards.

9. **Trial count/sequence may change at final evaluation**, but will always involve SFP and SC insertion types.

10. **`scoring.md` is the authoritative scoring reference.** `scoring_tests.md` may show slightly different (older) values.

---

## 14. Repository Structure

```
aic/
├── docs/                        # All competition & technical documentation
│   ├── overview.md              # High-level challenge description
│   ├── challenge_rules.md       # Technical specs and prohibited actions
│   ├── phases.md                # Per-phase breakdown
│   ├── qualification_phase.md   # Qualification trial descriptions
│   ├── scoring.md               # Full scoring tier definitions (AUTHORITATIVE)
│   ├── scoring_tests.md         # Reproducible scoring examples
│   ├── submission.md            # Containerization and upload guide
│   ├── getting_started.md       # Environment setup quickstart
│   ├── policy.md                # Policy implementation tutorial
│   ├── aic_interfaces.md        # All ROS topics, services, actions
│   ├── aic_controller.md        # Controller architecture and parameters
│   ├── scene_description.md     # Simulation environment details
│   ├── task_board_description.md# Task board zones, components, BOM
│   ├── access_control.md        # Zenoh ACL anti-cheat mechanism
│   ├── glossary.md              # Terminology dictionary
│   ├── participant_utilities.md # Teleoperation, visualization tools
│   ├── troubleshooting.md       # Known issues and fixes
│   ├── build_eval.md            # Building evaluation from source
│   └── custom_dockerfile.md     # Custom Docker for non-standard setups
├── aic_model/                   # YOUR POLICY GOES HERE
├── aic_example_policies/        # 3 reference policies
├── aic_engine/                  # Trial orchestrator (do not modify)
├── aic_bringup/                 # Launch files for simulation
├── aic_controller/              # Impedance controller for UR5e
├── aic_adapter/                 # Sensor fusion -> Observation
├── aic_interfaces/              # ROS 2 message/service/action definitions
├── aic_scoring/                 # Scoring system source code
├── aic_assets/                  # 3D models (SFP, SC, NIC, cables, enclosure)
├── aic_description/             # Robot and world URDF/SDF descriptions
├── aic_gazebo/                  # Gazebo plugins (scoring, cable, contacts)
├── aic_utils/
│   ├── aic_isaac/               # NVIDIA Isaac Lab integration
│   ├── aic_mujoco/              # Google DeepMind MuJoCo integration
│   ├── aic_teleoperation/       # Keyboard teleoperation tools
│   └── lerobot_robot_aic/       # HuggingFace LeRobot data collection
├── docker/
│   ├── aic_eval/                # Evaluation environment Docker image
│   ├── aic_model/               # Template Dockerfile for your submission
│   └── docker-compose.yaml      # Local end-to-end testing
└── pixi.toml                    # Workspace dependency management
```

---

## 15. Key Files to Know

| File | Why It Matters |
|---|---|
| `aic_model/aic_model/policy.py` | Base class you inherit from |
| `aic_engine/config/sample_config.yaml` | All 3 trial definitions with scene specs |
| `docker/aic_model/Dockerfile` | Starting Dockerfile for your submission |
| `docker/docker-compose.yaml` | Local end-to-end test harness |
| `docs/scoring.md` | Authoritative scoring rules |
| `docs/aic_interfaces.md` | Complete ROS interface reference |
| `docs/aic_controller.md` | Controller modes and parameters |
| `docs/submission.md` | Step-by-step submission guide |

---

*Good luck! May your insertions be precise and your penalties be zero.*
