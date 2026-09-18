# UR10e rail-and-arm trajectory planning

This ROS 2 Humble workspace plans a continuous motion for a UR10e on a linear
rail (seven joints) to reproduce a section of a SISIFOS target's triaxial
tumble. It searches an unseen recording for a planner-certified section of at
least 60 seconds. The fast search aims to return a first certificate within a
five-minute wall-clock budget; it does **not** guarantee finding one, proving
infeasibility, or finding the longest possible section.

## Current scope and safety boundary

- Input motion is `q_I_G` only: the attitude of target body frame G in the
  inertial frame I. Required CSV columns are `timestamp`, `q_I_G_x`,
  `q_I_G_y`, `q_I_G_z`, and `q_I_G_w`. SISIFOS translation is ignored. The
  target is placed at a fixed, legacy arena position; placement optimization
  and camera-relative motion are not part of this branch.
- The mount transform `T_EG` (target body G in robot tool frame E) is a
  **provisional identity**. Every exported plan records it. It must be
  established for the real mock-up before hardware use.
- A passing plan includes a home-to-start warmup, spin-up from rest, a lifted
  seven-joint task path, refinement, and continuous command validation for
  tracking, joint limits, velocity, collision and self-clearance. The rail
  velocity planning cap is **0.5 m/s**.
- Arm acceleration is provisional; arm jerk and rail acceleration and jerk
  are assumed. The validator gates acceleration and jerk against those
  provisional values, which are planning policy, **not validated hardware
  capability**. A passing plan is **planner-certified, not hardware-certified**.
  The RViz preview is visualization, not a hardware command.

## Run a section search

Clone with the pinned UR description submodule:

```bash
git clone --recurse-submodules git@github.com:HarrisonJenkins45/UR10e-Trajectory-Validation.git
```

For an existing checkout, run `git submodule update --init --recursive`.
Build the container from `ros2_ws/Dockerfile`. The planner commands below run
**inside** a container (or a host) where the ROS workspace has been built and
sourced; building the image alone does not install the package on the host.
Provide the recording and a pinned `home_choice.json`:

The home file is robot setup, reused across recordings—not a trajectory-specific
search result. Its required shape is
`{"chosen": {"configuration": [rail_m, six_arm_joint_radians]}}`.
The planner rechecks this pose and will not substitute another one. No home
pose is certified for physical hardware by this repository.

```bash
docker build -t ur10e_sim ros2_ws
python3 -m ur10e_trajectory_pkg.fast_section \
    --csv camera_traj.csv --home-json home_choice.json \
    --work-dir runs/sections --workers 12
```

`--via-poses screened_poses.json` optionally supplies vetted intermediate
warmup poses. `fast_section` publishes `best_plan.json` immediately when it
certifies at least 60 seconds, and writes `fast_section.json` and a probe
journal. If it returns `inconclusive_under_this_budget` or
`none_found_under_this_search`, it has **not** shown that the recording lacks a
feasible section: its on-demand beam search is incomplete.

Use the same work directory to spend a separate budget extending a found
section. The journal allows the longer search to replay the existing
certificate rather than rediscover it:

```bash
python3 -m ur10e_trajectory_pkg.section_planner \
    --csv camera_traj.csv --home-json home_choice.json \
    --work-dir runs/sections --workers 12 --budget-probes 8
```

That command writes `sections.json` and `best_plan.json`. Its reported length
is the longest **certified by that finite search**, not a global maximum. The
exhaustive candidate-graph mode (`--engine pipeline`) can bound a particular
start when its graph disconnects, but is much slower and still does not prove
that every section of a recording is infeasible.

## Inspect a certified plan in RViz

In a built and sourced ROS workspace:

```bash
ros2 launch ur10e_trajectory_pkg PreviewPlan_Rviz.py \
    plan:=/path/to/best_plan.json csv:=/path/to/the/same/camera_traj.csv
```

The preview verifies the recording digest and shows the arm, rail and target
body frame. It can loop home, warmup and task; the reverse return is for
display only and is **not certified**. Older plans may have their displayed
warmup slowed to the current 0.5 m/s rail cap; that retimed warmup needs new
validation before it is used as a command.

## Test and reproduce

```bash
cd ros2_ws
./run_tests.sh
./run_tests.sh -k slice_targets
```

`run_tests.sh` builds out of tree and runs tests inside the pinned container.
The Dockerfile pins its base image and Python dependencies. The reproducibility
unit is the **built image**, because the apt repository state is not frozen by
the base-image digest. Reports include the recording, model and code
provenance so results from different inputs or revisions are not conflated.

The prior frame conventions and translation experiments remain in Git history
and on their own branches; they are not implemented by the current
orientation-only planner.
