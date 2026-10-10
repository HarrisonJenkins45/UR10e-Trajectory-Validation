# IK foundations review and dataset run

Reviewed `revision/ik_foundations` at `131d111` on 2026-09-25. The review
focused on the branch's changes after `revision/cleanup_and_7DOF`: deterministic
seven-joint IK and pose acceptance, fixed-position target construction,
section search and certification, plan artifacts, and ROS/RViz playback.

## Findings addressed

- The runbook required external input paths and an unexplained home artifact.
  It now uses the existing bundled recording and a numeric simulation home in
  `ros2_ws/examples/home_choice.json`. The solve rechecked this home successfully.
- The optional service was described as hardware-facing. It publishes simulated
  joint states and assumes the home/start state; no hardware driver or encoder
  feedback is implemented. The README now states this and distinguishes the
  direct player's Target-frame display from the passive service display.
- The README implied that loading a plan verifies its certification. Direct
  playback checks structure, source identity, mount and warmup consistency; it
  does not repeat full motion validation. The README now requires unmodified
  planner output and explains finite sampling (200 Hz task checks, 30 Hz display).
- `fast_section` removes an existing `best_plan.json` before a new search.
  The README now calls for a fresh work directory and directs continuation to
  `section_planner`, including the same optional via poses and provenance.
- The test runner claimed to build outside the checkout but left colcon logs
  inside it. `run_tests.sh` now also sets an external log directory.
- The runbook now selects this branch, states input validation requirements,
  captures source/image provenance, uses a separate ROS domain, and explains
  container-name conflicts, exit codes and test exclusions. Local inputs and
  generated reports are ignored by Git.

## Verification

The documented Docker build reused image
`sha256:b0935ec83c4bc431b3d354ff6e85c68ac6d7940d4484777d6471ff0bc1578bf2`.
The UR description submodule was at
`18e6f603b3ebc2ec479fecb62d6be544b15755e9`.

`ros2_ws/run_tests.sh` passed all **488 functional tests** in 108.57 seconds,
with 29 warnings. The template lint/copyright tests are explicitly excluded
by that runner. `bash -n ros2_ws/run_tests.sh` and `git diff --check` also passed.
No planner or playback algorithm was changed during this review.

The dataset was copied from the tracked
`ros2_ws/src/ur10e_trajectory_pkg/ur10e_trajectory_pkg/camera_traj.csv`.
It contains 5,000 samples spanning 499.9 seconds; its SHA-256 is
`2dea1586d3bb6e193256d1f27643fd94aa154bb72bc9d17c901ab7434b1a1d5b`.

The README commands were run with container name `ur10e_review_20260925`,
`ROS_DOMAIN_ID=25`, and work directory `/root/ros2_ws/runs/review_20260925`.
The container was started detached with `sleep infinity`, then built and
operated through `docker exec`; this keeps the preview available after the
review. The solve used `--target-s 60 --time-budget-s 300 --workers 12`, the
example home, and no via poses or start hints.

Result: **certified_section_found**, after 5 attempts and 240.84 seconds.

| Measurement | Result |
| --- | --- |
| Recorded interval | 439.9–499.9 s (60.0 s) |
| Source indices | `[4399, 5000)` (601 samples) |
| Spin-up duration | 0.4 s |
| Task playback duration including time warp | 60.2 s |
| Direct warmup duration | 2.013 s |
| Home, warmup and task checks | All passed |
| Minimum sampled task self-clearance | 18.82 mm |
| Maximum task velocity budget used | 5.64% |

The direct RViz launch loaded the new plan and matching CSV. Its window
reported **Global Status: Ok**, displayed the robot, obstacles and Target
frame, and completed the forward task before entering the display-only return.
A ten-second subscriber check received 293 seven-joint messages at about
29.25 Hz, changing joint positions, 292 Target transforms and 292 marker
messages. There was exactly one `/joint_states` publisher in domain 25.
RViz printed GPU-driver warnings but successfully initialized OpenGL and
rendered the scene. The optional service playback and longer-section search
were inspected and covered by tests, but were not separately launched here.

## Local evidence and running preview

Generated evidence is retained locally under
`ros2_ws/runs/review_20260925/` (ignored by Git):

- `best_plan.json`, `fast_section.json`, `probes.jsonl`, and per-attempt reports;
- `solve.log`, `tests.log`, `rviz.log`, and `playback_check.json`;
- `rviz.png`, showing the live RViz scene.

Watch the preview log from the repository root:

```bash
tail -f ros2_ws/runs/review_20260925/rviz.log
```

Stop only this review's container from the host:

```bash
docker stop ur10e_review_20260925
xhost -si:localuser:root
```

The older containers were left untouched. The new preview uses a separate ROS
domain. The plan is planner-certified under the branch's provisional limits
and finite checks; it is not a hardware certification. The reverse return is
for display only. Solve timing and the first section found are observations
of this run, not guarantees for another machine or budget.
