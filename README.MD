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

## Executing a plan on hardware through ROS (demo)

`hardware_demo.launch.py` replaces the Simulink BaseFrameDetermination command
path. It uses the same ports and wire formats, but takes its reference from
`best_plan.json` instead of a hand-written sinusoid:

| Simulink | ROS node | Topic |
|---|---|---|
| `refVel(t)`, Manual Switch, 20 Hz hold | `plan_executor` | `/plan/joint_reference`, `/ur/joint_velocity_command`, `/rail/velocity_command` |
| `tcpUR10eBlock` + `UR10eSend` (`speedj`, :30003) | `ur_bridge` | `/ur/joint_states` |
| `udpPARKERBlock` + rail Subsystem (`JOG VEL`, :5002/:5003) | `rail_bridge` | `/rail/joint_state` |
| Gamma/dGamma mux | `joint_state_merger` | `/joint_states` |

RViz shows the measured robot solid and the plan reference translucent. The
reference is published under the `sim/` TF prefix; `gazebo:=true` also drives
the Gazebo model from it. The plan is still **planner-certified, not
hardware-certified**. `~/move_to_home` is an uncertified move with no
collision check.

### Rehearse with fake hardware

Inside the built, sourced container (steps 3 and 4):

```bash
ros2 launch ur10e_trajectory_pkg hardware_demo.launch.py \
    plan:=/root/ros2_ws/runs/sections/best_plan.json \
    csv:=/root/ros2_ws/input/camera_traj.csv \
    hardware:=fake mode:=warmup time_scale:=0.1
```

In a second shell:

```bash
ros2 service call /plan_executor/start std_srvs/srv/Trigger
ros2 service call /plan_executor/stop std_srvs/srv/Trigger       # any time
```

`~/start` is one call from wherever the robot is. It runs these stages:

1. **Straighten the arm** to the plan-home posture, with the rail held still.
   Only if every arm joint is within `auto_home_arm_limit` (0.2 rad).
   Otherwise it refuses and prints the pendant joint values to use.
2. **Home the rail**: `AXIS0 JOG VEL <rail_homing_speed>:AXIS0 JOG HOME -1`.
   With `home_rail:=if_needed` (default) this only happens when the
   home-found flag is clear; `always` homes every time, `never` requires the
   flag to be set already.
3. **Move to plan home**: the rail moves with the arm held at home. From the
   task start (after a warmup-only run), it retraces the certified warmup
   instead.
4. **Run the plan**: the warmup (`mode:=warmup`), or warmup plus task
   (`mode:=full`).

`~/move_to_home` runs steps 1 to 3 only.

Stage 3's rail move and the rail homing keep the arm at the screened home
posture. The modelled wall and floor are uniform along the rail, so the
clearance stays as screened. Obstacles in the lab that are not in the model
are not covered.

`time_scale` slows every stage uniformly: velocities scale by it and
accelerations by its square. The executor logs the resulting peak speeds, and
refuses if they exceed `arm_speed_limit` (0.5 rad/s) or `rail_speed_limit`
(0.05 m/s).

To rehearse homing, add `fake_rail_homed:=false` to the launch; the fake rail
homes to `rail_offset_m`.

### On the rig: full runbook

Plan used below: `runs/review_20260925/best_plan.json`. Its home is rail
1.5 m and arm `[0, -75, 100, -115, -80, 0]` degrees. For another plan, print
its home with:

```bash
python3 -c "import json,math;h=json.load(open('ros2_ws/runs/review_20260925/best_plan.json'))['home'];print('rail m',h[0],'arm deg',[round(math.degrees(a),3) for a in h[1:]])"
```

Controllers: UR10e at `192.168.7.8` (TCP 30003), Parker rail at `192.168.7.6`
(TCP 5002 commands, UDP 5003 feedback). One process at a time may own each:
close Simulink, PuTTY and any other rail or UR client first.

#### R1. Host network (host shell)

```bash
ip -4 addr                        # a 192.168.7.x/24 address must be present (was 192.168.7.50 on enp4s0f1)
ping -c 3 192.168.7.8             # UR10e
ping -c 3 192.168.7.6             # Parker rail
ss -ulpn | grep 5003              # must print nothing: UDP 5003 is free for rail_bridge
```

The arm is commanded over RTDE by default. Every connection goes from this PC
to the robot, so no firewall change or sudo is needed. Only the optional
`ur_stream:=program` needs the robot to connect back to this PC on TCP 50010.
To see whether a firewall is on, without sudo: `grep ENABLED
/etc/ufw/ufw.conf` and `systemctl is-active ufw firewalld`.

#### R2. Prepare the rail controller (host shell, Python)

This is the same interactive socket helper as `TERMINAL_SOCKET.md` on
`revision/hardware_movement`. From the host:

```bash
python3
```

Then paste this into Python:

```python
import socket
s = socket.create_connection(("192.168.7.6", 5002), timeout=2)
s.settimeout(0.5)

def command(text):
    s.sendall((text + "\r").encode("ascii"))
    try:
        while True:
            data = s.recv(4096)
            if not data:
                break
            print(data.decode("ascii", errors="replace"), end="", flush=True)
    except socket.timeout:
        pass

```

Then run these, one at a time:

```python
command("VER")                 # controller answers: connection works
command("PRINT P12349")        # jog acceleration: expect 10, not 0.1 (0.1 makes homing crawl)
command("PRINT P12350")        # jog deceleration: expect 10
command("FSTAT0(48,2)")        # UDP feedback word 0: position counts
command("FSTAT1(48,58)")       # UDP feedback word 8: jog velocity
command("FSTAT ON")
command("AXIS0 DRIVE ON")      # enables the drive; the rail can now move on command
command("PRINT BIT 16134")     # home-found flag: -1 homed, 0 not homed
s.close()
exit()
```

Note the exact text `PRINT BIT 16134` printed: `rail_bridge` parses that
reply. If its format is not the command echo, then `-1` or `0` on its own
line, then a prompt, the bridge will log "unreadable home-flag reply". In
that case, home here with `command("AXIS0 JOG HOME -1")` and launch below
with `home_rail:=never`.

#### R3. Prepare the UR10e (teach pendant)

1. Power on and release the brakes.
2. Enable remote control: Settings → System → Remote Control → Enable. Then
   switch the mode (top right) from Local to **Remote**. Scripts sent to
   30003 are not executed in Local mode.
3. Check that RTDE is enabled: Settings → Security → Services → RTDE. It is
   on by default. `ur_bridge` writes the arm velocities through it.
4. Move the arm to within 0.2 rad (about 11°) of the plan's home on every
   joint. Joints on the pendant are Base, Shoulder, Elbow, Wrist 1, Wrist 2,
   Wrist 3, i.e. `[0, -75, 100, -115, -80, 0]` degrees for this plan. `~/start`
   refuses if any joint is further off.
5. Set the **speed slider** (bottom of the Run screen) to **100%**, and check
   the robot is not in Reduced mode. The UR scales every commanded arm motion
   by the slider, but the rail is not scaled. Below 100% the arm lags the
   rail, and the tool drifts with the carriage instead of holding still. The
   bridge logs `UR speed scaling ...`, and `~/start` refuses below 100%.
   The scaling value's position in the UR packet comes from UR's documented
   layout but is unverified on this controller. If the log shows 100% on the
   pendant as some other value, relaunch with `speed_scaling_check:=false`.

#### R4. Rail calibration values

- `rail_offset_m`: the planner rail coordinate (URDF metres) of the
  controller's homed zero. The carriage centre is `AXIS0 JOG ABS 1550`, and
  the URDF rail runs 0–3 m. If 1550 controller units = 1550 mm = the
  planner's 1.5 m, then the offset is -0.05. **Confirm by measuring** before
  trusting it.
- `rail_sign`: +1 if JOG FWD (increasing counts) moves toward the URDF +x end.

A wrong offset is caught by the start gate, not by motion. It still decides
where "plan home" is on the real rail, so check it in R6.

#### R5. Plan, recording and container (host shell)

`ros2_ws/runs/` and `ros2_ws/input/` are not in Git. The plan and the
recording it was certified on must be copied onto this PC. The executor
refuses a recording whose SHA-256 differs from the one stored in the plan.
From the repository root:

```bash
mkdir -p ros2_ws/input ros2_ws/runs/review_20260925
cp ros2_ws/src/ur10e_trajectory_pkg/ur10e_trajectory_pkg/camera_traj.csv ros2_ws/input/
# copy best_plan.json into ros2_ws/runs/review_20260925/, then check the digests match:
python3 -c "import json;print(json.load(open('ros2_ws/runs/review_20260925/best_plan.json'))['recording']['csv_sha256'])"
sha256sum ros2_ws/input/camera_traj.csv
```

Start the container as in step 3 above, keeping `--network host`, which the
bridges need to reach 192.168.7.x. Then build and source as in step 4.
Everything below runs inside the container.

Without Docker (ROS 2 Humble installed natively):

1. Build the workspace with colcon.
2. Install the Python pins from `ros2_ws/requirements.txt`.
3. Add `urdf:=<clone>/ros2_ws/ur10e.urdf` to the launch commands.
4. Replace the `/root/ros2_ws/...` paths with your clone's `ros2_ws`.

The pinned container is the tested route.

#### R6. Shadow launch: read both devices, send no motion

```bash
ros2 launch ur10e_trajectory_pkg hardware_demo.launch.py \
    plan:=/root/ros2_ws/runs/review_20260925/best_plan.json \
    csv:=/root/ros2_ws/input/camera_traj.csv \
    hardware:=real enable_commands:=false \
    rail_offset_m:=-0.05 rail_sign:=1 \
    mode:=warmup time_scale:=0.1 kp:=1.0
```

In a second terminal (`docker exec -it ur10e_sim bash`, then source as in
step 8), check:

```bash
ros2 topic echo --once /joint_states     # 7 values: rail m, then arm rad; compare with the pendant
ros2 topic echo --once /rail/homed       # data: true/false (the launch log also prints it)
```

In RViz, the solid robot should match the real one.

If the rail is homed, it reads `rail_offset_m` at the switch. Otherwise, the
position is meaningless until `~/start` homes it.

The launch log's `[shadow]` lines show the exact `speedj` and `JOG` text that
would be sent. Calling `~/start` now runs the executor, but the robot does not
move, so the executor halts on tracking error. That is expected in shadow
mode. Ctrl-C when done.

#### R7. Enabled launch and the demo

Relaunch the same command with `enable_commands:=true`, with the pendant e-stop
and the rail's physical stop in reach.

**How the arm is commanded.** With `enable_commands:=true`, `ur_bridge`:

1. connects to the UR's RTDE interface (port 30004),
2. uploads one URScript program, `ros_speedj_rtde`, over port 30003.

ROS writes the six joint velocities and a sequence counter into RTDE input
registers (`input_double_register_24..29`, `input_int_register_24`). The
program reads them and runs `speedj` at 125 Hz on the robot. It zeroes the
arm by itself if no new command arrives for 0.25 s. All connections go from
this PC to the robot.

The launch log should show, in order:

```text
RTDE connected to 192.168.7.8:30004; writing arm velocities to input_double_register_24..29 ...
Uploading the streaming program ros_speedj_rtde to 192.168.7.8:30003; ...
Streaming program running on the robot; ready for commands
```

The pendant's Log tab shows the program's own start-up message
(`ros_speedj_rtde: running ...`). If something fails, the bridge log names the
cause and what to check:

| Log message | Check |
|---|---|
| "RTDE setup ... failed" | RTDE is disabled on the robot, or another RTDE client (`ur_robot_driver`, a second bridge) holds the registers |
| "did not start" | Remote Control mode; robot powered on with brakes released; the pendant Log tab |

If the program stops on the robot (stop pressed on the pendant, a protective
stop), the bridge latches a fault. `/ur_bridge/reset_fault` restarts the
program on the existing RTDE connection.

Other transports:

- `ur_stream:=program`: the same persistent program, but it connects back to
  this PC on TCP 50010. It needs an open firewall.
- `ur_stream:=lines`: Simulink's method, a new one-line `speedj` program per
  command.

These were validated on URSim (PolyScope 5.26). All three tracked a wrist
command at 100%.

**Reading the arm diagnostics.** While the arm is commanded, `ur_bridge` logs
one line per second:

```text
arm: 20 cmd/s | wrist_3_joint cmd +0.0500 target +0.0500 (100%) actual +0.0497 (99%) | scaling 1.00 | RUNNING (7) | NORMAL (1) | PLAYING (2) | stream running
```

| Field | Meaning |
|---|---|
| `cmd` | What ROS asked for, on the joint with the largest command |
| `target` | What the UR controller is aiming for (its own target velocity) |
| `actual` | What the joint measures |

Percentages are the share of the whole command vector delivered. Any
disagreement is followed by an `arm diagnosis:` warning that says where the
chain breaks:

- too few commands per second means the executor or ROS is slow,
- a low `target` means the controller is not running our program,
- a low `actual` with a good `target` means the arm is held back (safety mode,
  limits),
- scaling below 1.00 means the speed slider or Reduced mode.

Robot mode, safety mode, program state and speed scaling are also logged
whenever they change. Send these lines when reporting a problem.

Before the first plan run, check that the arm follows streamed velocities. With
the executor idle, stream a slow wrist 3 turn for 4 s: 80 messages at 20 Hz,
0.05 rad/s, 0.20 rad expected.

```bash
ros2 topic echo --once /ur/joint_states --field position        # note wrist_3 (last value)
ros2 topic pub -r 20 -t 80 /ur/joint_velocity_command std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 0, 0, 0.05]}"
ros2 topic echo --once /ur/joint_states --field position        # wrist_3 should be about 0.20 rad larger
```

Much less than 0.20 rad means the arm is not executing the stream at the
commanded speed. The `arm:` and `arm diagnosis:` lines logged during the check
say why. Undo the turn with the same command and `-0.05`.

Then in the second terminal:

```bash
ros2 topic echo /plan_executor/phase &                                 # live phase
ros2 service call /plan_executor/start std_srvs/srv/Trigger            # the whole sequence
ros2 service call /plan_executor/stop std_srvs/srv/Trigger             # stop, any time
```

While moving, the executor logs a line like `tracking: rail +0.4 mm, worst arm
+0.0031 rad (wrist_1_joint), arm speed 99% of reference` once a second. The
arm speed should stay close to 100%. During the task, the tool should hold
its position while the carriage slides, because the arm compensates for the
rail.

`start` straightens the arm if needed, homes the rail if its flag is clear,
moves the rail to 1.5 m, then runs the warmup (about 20 s at
`time_scale:=0.1`).

Calling `start` again returns along the warmup and repeats it. For the tumble,
relaunch with `mode:=full time_scale:=0.14` (the elbow limits the scale; about
7.5 minutes).

If a call is refused, its reply says why. After a bridge fault:

```bash
ros2 service call /ur_bridge/reset_fault std_srvs/srv/Trigger
ros2 service call /rail_bridge/reset_fault std_srvs/srv/Trigger
```

If the rail ignores jog commands, relaunch with `rail_terminator:=lfcr` (the
line ending Simulink used).

#### R8. Shut down

1. Call `~/stop`, then Ctrl-C the launch. The bridges send `stopj` and
   `JOG OFF` on exit.
2. Optionally disable the rail drive from the R2 helper with
   `command("AXIS0 DRIVE OFF")`.
3. Put the pendant back in Local mode.

#### What the system does on its own

Each motion stage refuses to begin unless the measured pose matches its start.
The plan itself needs 1 cm (rail) and 0.03 rad (arm).

The executor halts, commanding zero, on:

- stale feedback,
- tracking error above 1 cm (rail) or 0.02 rad (any arm joint), which keeps the tool
  within a few cm of the plan,
- UR speed scaling below 100%,
- a command above the speed limits,
- homing that does not complete in time.

The bridges also stop independently:

- a command stream silent for 0.25 s triggers `stopj` or `JOG OFF`,
- `speedj` expires after 0.2 s on the controller itself,
- the rail stops at its soft limits `rail_soft_min_m`/`rail_soft_max_m`
  (moving back inside is allowed),
- `~/stop` aborts rail homing with `JOG OFF`.

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

These two one-off tests predate the plan executor. To run a plan on the
hardware, use [Executing a plan on hardware through ROS](#executing-a-plan-on-hardware-through-ros-demo).

ros2 run wrist_motion_pkg wrist server
ros2 service call /trigger_wrist_motion std_srvs/srv/Trigger "{}"

## Commanding the rail from python
python3 railTest_modified.py /home/hjenkins33/UR10e-Trajectory-Validation/ros2_ws/runs/review_20260925/best_plan.json


