# Plan: wall-mounted rig, 2026-09-29

`best_plan.json` is the plan run on the rig on 2026-09-29. `fast_section.json`
is the search report that produced it.

| | |
|---|---|
| Recording | `src/ur10e_trajectory_pkg/ur10e_trajectory_pkg/camera_traj.csv`, samples 0–601 (60.0 s plus a 0.4 s spin-up); SHA-256 `2dea1586…1d5b`, checked by the executor |
| Home | rail 1.5 m, arm `[0, -75, 100, -115, -80, 0]` deg (`examples/home_choice.json`) |
| Tumble point | `tool0` held at rail frame (1.0, 0.5, 0.5) m: 1.0 m along the rail, 0.5 m above it, 0.5 m out from the wall |
| Rail | 1.36–1.50 m during the task, 3 reversals |
| Speeds | task fits the executor's limits up to `task_time_scale` 3.2; warmup needs `time_scale` ≤ 0.14 |

## Provenance

- Planned with `fast_section` (`--target-s 60 --time-budget-s 300`) against the
  corrected rig model. In that model, the arm is turned 180° on the carriage,
  and the frame is the wall's: +x along the rail, +y up, +z out of the wall.
- **The certificate used a floor 3 m below the rail.** The rig's floor is 1 m
  below, and the model was corrected to that afterwards. The plan has not
  been re-checked against the 1 m floor. It ran clear of it on the rig, and
  the operator accepted it as safe on that basis.
- **Rig run:** a checked return from the previous pose, then the warmup at
  0.1×, then the task at 1.0× (60.2 s). Worst tracking error was 0.6 mm on
  the rail and 0.0044 rad on the arm, and the flange held its position.

## Run

See the README section "Running the wall-mount plan".
