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
  target is placed at `[1.0, 0.5, 0.5]` metres in `rail_base_link`. Placement
  optimization and camera-relative motion are not part of this branch.
- `trajectory_input.py` is the single CSV reader and input validator. Search,
  target construction and RViz use its timestamps, orientations,
  slices and source digest. Separate planning processes may reopen the file
  through that module; they do not maintain separate CSV parsers.
- `target_builder.py` converts validated orientation samples into fixed-position,
  time-stamped tool targets, including placement, mount and optional spin-up.
  Its core interface accepts the loaded trajectory object, with a path-based
  adapter for separate planning stages. It has no ROS client dependency.
- `plan_artifact.py` owns the exported joint-plan format, structural checks and
  source-matched target reconstruction. The ROS client and RViz consume that
  same plan contract; neither parses trajectory inputs or invents targets.
- `joint_motion.py` owns the rest-to-rest quintic warmup and PCHIP task curve.
  The validator, service playback and RViz sample those same joint motions;
  ROS publishes the resulting coordinates rather than defining the motion.
  The section planners and optional service check task dynamics at 200 Hz;
  RViz displays at 30 Hz. Collision, clearance and tracking checks also use
  finite sampling. Certification is relative to these numerical checks,
  not an analytic proof at every instant.
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

## Exact runbook

The supported workflow is:

```text
trajectory CSV + pinned home configuration
                 |
                 v
       search and certify a section
                 |
                 v
             best_plan.json
                 |
                 v
              RViz preview
```

Planning does not require ROS nodes to be running. ROS is used to display or
play the plan only after the search has produced `best_plan.json`.

Prerequisites: a Linux host with Docker, Git, and an X11/XWayland display
(`DISPLAY` must be set and `xhost` available) for the interactive RViz window.
Planning and tests do not need a display. The commands below use 12 workers;
reduce `--workers` on smaller machines. A shorter compute budget or fewer
workers may finish without a certificate.

### 1. Clone and prepare the inputs

Clone the repository with the pinned Universal Robots description package:

```bash
git clone --recurse-submodules \
    git@github.com:HarrisonJenkins45/UR10e-Trajectory-Validation.git
cd UR10e-Trajectory-Validation
git switch revision/ik_foundations
git submodule update --init --recursive
```

For an existing checkout:

```bash
git submodule update --init --recursive
```

For a self-contained simulation, use the bundled 5,000-sample recording
(0.1 s spacing, 499.9 s total) and example home pose. Run on the **host** from
the repository root:

```bash
mkdir -p ros2_ws/input ros2_ws/runs/sections
cp ros2_ws/src/ur10e_trajectory_pkg/ur10e_trajectory_pkg/camera_traj.csv \
    ros2_ws/input/camera_traj.csv
cp ros2_ws/examples/home_choice.json ros2_ws/input/home_choice.json
```

The example home is rail = 1.5 m and arm angles =
`[0, -75, 100, -115, -80, 0]` degrees, stored in radians in the JSON.
It is a simulation input, re-screened by the planner on each solve.
For another dataset or robot setup, replace the two input files with your own
recording and configured home; no home-selection tool is required for this demo.

The CSV must contain these columns:

```text
timestamp,q_I_G_x,q_I_G_y,q_I_G_z,q_I_G_w
```

Other columns are allowed but ignored. Translation columns are not used. The
quaternion is the attitude of target body frame G in inertial frame I.
Timestamps are seconds and must be finite, strictly increasing and uniformly
spaced (absolute step tolerance `1e-9` s). Quaternion components must be finite
and unit length (norm tolerance `1e-6`). At least two samples are required;
a 60-second search needs at least 60 seconds of recorded motion.

`home_choice.json` is a robot-setup artifact reused across recordings, not a
trajectory-specific result. At minimum it contains:

```json
{
  "chosen": {
    "configuration": ["rail_m", "shoulder_pan_rad", "shoulder_lift_rad",
                      "elbow_rad", "wrist_1_rad", "wrist_2_rad", "wrist_3_rad"]
  }
}
```

The strings above document the seven-value ordering; an actual file must use
finite JSON numbers. The planner rechecks the supplied pose and refuses one
that fails its current collision, clearance or conditioning criteria. This
repository deliberately does not claim a hardware-certified home pose.

### 2. Build the image

Run this from the repository root:

```bash
docker build -t ur10e_sim:latest ros2_ws
```

### 3. Start the development container

On the host, permit the root user in the container to connect to the local X
server, then start a named container with the workspace mounted. Check
`docker ps` first: if `ur10e_sim` already exists, choose another name and use
that name in later `docker exec` and `docker stop` commands. `ROS_DOMAIN_ID=25`
keeps this preview separate from default-domain ROS nodes; use another unused
domain if 25 is already in use:

```bash
xhost +si:localuser:root

docker run --rm -it \
    --name ur10e_sim \
    --network host \
    -e DISPLAY="$DISPLAY" \
    -e ROS_DOMAIN_ID=25 \
    -e UR10E_GIT_REVISION="$(git rev-parse HEAD)" \
    -e UR10E_IMAGE_ID="$(docker image inspect ur10e_sim:latest --format '{{.Id}}')" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PWD/ros2_ws:/root/ros2_ws" \
    ur10e_sim:latest bash
```

All remaining commands in this runbook are run **inside this container** unless
they are explicitly labelled as host commands.

### 4. Build and source the ROS workspace

Inside the container:

```bash
source /opt/ros/humble/setup.bash

colcon --log-base /root/ur10e_log build \
    --base-paths /root/ros2_ws/src \
    --build-base /root/ur10e_build \
    --install-base /root/ur10e_install \
    --symlink-install

source /root/ur10e_install/setup.bash
```

This builds outside the mounted repository so generated build files do not
accumulate in the checkout.

### 5. Find and certify a section of at least 60 seconds

Inside the built and sourced container. Use a fresh `--work-dir` for a new
first-certificate run: `fast_section` removes any existing `best_plan.json`
in that directory before searching. Use step 7 to extend a saved result.

```bash
python3 -m ur10e_trajectory_pkg.fast_section \
    --csv /root/ros2_ws/input/camera_traj.csv \
    --home-json /root/ros2_ws/input/home_choice.json \
    --work-dir /root/ros2_ws/runs/sections \
    --target-s 60 \
    --time-budget-s 300 \
    --workers 12
```

This is the IK solve: it searches rail/arm configurations, connects and
refines a joint path, then validates the warmup and task before exporting it.
Exit code 0 and `status: certified_section_found` mean success; only then
continue to RViz. Codes 1, 2 and 3 mean input refused, no certificate in the
scheduled search, and budget exhausted, respectively. Invalid input files can
also produce an exception; inspect the terminal output.

On success, the reusable plan is:

```text
ros2_ws/runs/sections/best_plan.json
```

The directory also contains `fast_section.json` and a probe journal. Inspect
the final status with:

```bash
python3 -m json.tool \
    /root/ros2_ws/runs/sections/fast_section.json
```

`inconclusive_under_this_budget` and `none_found_under_this_search` do **not**
mean that no feasible section exists. The fast beam search is deliberately
time-bounded and incomplete.

To supply vetted intermediate poses for a collision-free home-to-task warmup,
add this option to the search command:

```text
--via-poses /root/ros2_ws/input/screened_poses.json
```

### 6. Play the certified plan in RViz

In the same built and sourced container:

```bash
ros2 launch ur10e_trajectory_pkg VisualizeTraj_RvizPlayback.py \
    plan:=/root/ros2_ws/runs/sections/best_plan.json \
    csv:=/root/ros2_ws/input/camera_traj.csv
```

The player verifies the source-recording digest before publishing anything. It
shows the rail, arm, floor, wall and Target frame and loops this display
sequence:

```text
home -> certified warmup -> spin-up -> certified task -> display-only return
```

The return from the end of the task is a visualization convenience and is not
planner-certified. Close RViz with `Ctrl-C` in the launch terminal.

Do not start another joint-state source while this command is running. The
direct plan player publishes `/joint_states` itself.

### 7. Optionally search for a longer certified section

Use the same input files (including `--via-poses` if supplied), image, source
revision and work directory so this search can reuse the first
certificate instead of discovering it again:

```bash
python3 -m ur10e_trajectory_pkg.section_planner \
    --csv /root/ros2_ws/input/camera_traj.csv \
    --home-json /root/ros2_ws/input/home_choice.json \
    --work-dir /root/ros2_ws/runs/sections \
    --target-s 60 \
    --workers 12 \
    --budget-probes 8
```

This writes `sections.json` and may replace `best_plan.json` with a longer
certified plan. The reported section is the longest found by that finite
search, not a guaranteed global maximum. `--engine pipeline` invokes the much
slower candidate-graph engine and is intended for diagnosis rather than the
normal five-minute workflow.

### 8. Optional ROS service playback

The direct RViz command above is the simplest way to inspect a plan. The ROS
client/server path revalidates and publishes simulated `/joint_states`; it
has no robot driver or encoder feedback and does not command physical hardware.
The client assumes the simulated arm begins at the plan's home. This mode
requires three sourced terminals in the running container.

From the host, open each additional terminal with:

```bash
docker exec -it ur10e_sim bash
```

Then source the workspace inside that new container shell:

```bash
source /opt/ros/humble/setup.bash
source /root/ur10e_install/setup.bash
```

Terminal 1, validation and playback services:

```bash
ros2 run ur10e_trajectory_pkg validation_server
```

Terminal 2, passive RViz display:

```bash
ros2 launch ur10e_trajectory_pkg VisualizeTraj_RvizPlayback.py
```

Terminal 3, play the certified warmup and task plan:

```bash
ros2 run ur10e_trajectory_pkg trajectory_client \
    /root/ros2_ws/runs/sections/best_plan.json
```

Keep the recording at the `recording.csv_path` stored in the plan: the service
client has no CSV-path override. The direct player in step 6 accepts a relocated
CSV when its digest matches. The service display shows the robot and obstacles;
the animated Target frame is supplied only by the direct plan player.

Do not give `plan` and `csv` arguments to the RViz launch in service mode. The
service publishes `/joint_states`; enabling the direct player at the same time
would create two competing publishers.

### 9. Run the authoritative tests

Tests are launched from the **host**, not from the interactive container:

```bash
cd ros2_ws
./run_tests.sh
```

To run a selected group:

```bash
./run_tests.sh -k slice_targets
```

`run_tests.sh` builds out of tree and runs the functional tests inside the
pinned image. It excludes the template copyright, flake8 and pep257 checks;
a passing run is not a lint result.
The Dockerfile pins its base image and Python dependencies. The reproducibility
unit is the built image because the apt repository state is not frozen by the
base-image digest.

### 10. Stop the container

Exit the container shell or stop it from the host:

```bash
docker stop ur10e_sim
xhost -si:localuser:root
```

The container is disposable. Inputs, reports and plans persist because
`ros2_ws` is mounted from the host.

A completed bundled-dataset run, review findings and validation evidence are
recorded in [the branch review](docs/ik_foundations_review.md).

## Search components

The two search commands share `section_contract.py` for input identity,
spin-up selection, probe evidence, certificate checks and journal identity.
`fast_section.py` chooses a first-certificate schedule; `section_planner.py`
chooses starts and lengths for the resumable search. Neither search command
imports the other. `section_search.py` contains the ROS-free length-search
policy; `fast_beam.py` contains the oracle-based beam policy; and
`fast_certify.py` supplies robot-specific candidates, edges and certification.
The old corpus benchmark, failure census, placement sweep and MAT-file player
are not part of this package's execution path.

The direct player checks the plan structure, warmup endpoints and rail-speed
cap, mount, recording digest and slice identity. It does not rerun the full
motion validator or authenticate a certificate: use unmodified planner output.
The service path additionally revalidates the motions before playback. A stored
warmup that exceeds the current 0.5 m/s rail cap must be replanned and recertified, not silently retimed for display.

The prior frame conventions and translation experiments remain in Git history
and on their own branches; they are not implemented by the current
orientation-only planner.



## Commanding the arm from a ros node (wrist package)
ros2 run wrist_motion_pkg wrist server
ros2 service call /trigger_wrist_motion std_srvs/srv/Trigger "{}"

## Commanding the rail from python
python3 railTest_modified.py /home/hjenkins33/UR10e-Trajectory-Validation/ros2_ws/runs/review_20260925/best_plan.json


