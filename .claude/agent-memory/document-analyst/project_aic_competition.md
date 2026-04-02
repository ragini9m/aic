---
name: AIC Competition - Core Project Knowledge
description: Full analysis of the AI for Industry Challenge repository, including competition structure, scoring, rules, and submission requirements
type: project
---

The project is the **AI for Industry Challenge (AIC)** by Intrinsic (a Google company). It is a robotics competition where participants build AI policies to autonomously insert fiber optic cables into networking hardware using a simulated UR5e robot arm.

**Why:** The challenge targets electronics assembly automation (cable management in server/data center hardware), one of the hardest unsolved problems in manufacturing robotics.

**How to apply:** All future work in this repo should be understood in the context of this competition. Policy development, scoring, and submission are the core activities.

**Prize pool:** $180,000 shared among top 5 teams.

**Competition dates:** March 2 – September 8, 2026.

**Three phases:**
1. Qualification (Mar 2 – May 15) — simulation only, Gazebo evaluation; top 30 advance (eval May 18–27, results May 28)
2. Phase 1 (May 28 – Jul 14) — access to Intrinsic Flowstate; top 10 advance (eval Jul 14–21, results Jul 22)
3. Phase 2 (Jul 27 – Aug 25) — real robot at Intrinsic HQ; winner announced Sep 8

**Key submission facts:**
- 1 submission per day limit (per team, not per individual)
- Final submission per phase is used for scoring
- Must containerize with Docker/Podman and push to AWS ECR
- Registry: `973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/<team_name>`
- Portal: aiforindustrychallenge.ai

**Scoring tiers (max 100 pts per trial):**
- Tier 1 (0-1): Model validity pass/fail
- Tier 2 (up to 24, penalties up to -36): smoothness (0-6), duration (0-12), efficiency (0-6), force penalty (0 to -12), off-limit contacts (0 to -24)
- Tier 3 (-12 to 75): correct insertion = 75, wrong port = -12, partial/proximity = 0-50

**Qualification trials (3 total):**
- Trials 1 & 2: SFP module insertion into NIC card SFP ports (randomized NIC pose)
- Trial 3: SC plug insertion into SC port (generalization test)

**Technology stack:** ROS 2 (Kilted), Gazebo (primary sim), MuJoCo (Google DeepMind mirror), Isaac Lab (NVIDIA mirror), rmw_zenoh_cpp middleware, Docker, Pixi (package manager), Distrobox

**Robot:** Universal Robots UR5e + Robotiq Hand-E gripper + Axia80 F/T sensor + 3 wrist cameras

**What participants submit:** A Docker container with a ROS 2 Lifecycle node named `aic_model` that responds to the `/insert_cable` action and outputs commands to `/aic_controller/pose_commands` or `/aic_controller/joint_commands`.
