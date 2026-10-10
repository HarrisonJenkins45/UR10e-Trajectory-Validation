# Plan: payload tumbling about its body's centre, 2026-10-10

`best_plan.json` is the plan for the rig with the payload mounted.
`fast_section.json` is the search report that produced it.

| | |
|---|---|
| Recording | `src/ur10e_trajectory_pkg/ur10e_trajectory_pkg/camera_traj.csv`, samples 1099–1700 (recorded 109.9–169.9 s, 60.0 s, plus a 0.4 s spin-up); SHA-256 `2dea1586…1d5b`, checked by the executor |
| Home | rail 1.5 m, arm `[0, -75, 100, -115, 100, 90]` deg (`examples/home_choice.json`) |
| Mount `T_EG` | centre of rotation at the geometric centre of the 6 × 6 × 18 in body, 0.5334 m (21 in) out along `tool0` +z, no rotation; provisional |
| Placement | centre of rotation held at rail frame (1.0, 0.5, 0.5) m; recording turned so the boom faces out of the mounting wall (`target_builder.lab_orientation`) |
| Payload | 1 ft, 1 in boom; 6 × 6 × 18 in body, bottom face centred on the boom tip; two flat 6 × 12 × 0.125 in wings out of its `tool0` ±x faces, flush with its top |
| Rail | 1.51–2.59 m during the task, 1 reversal |
| Warmup | direct, 2.6 s at planner speed |
| Speeds | the executor accepts the warmup up to `time_scale` 0.143 and the task up to `task_time_scale` 1.37 (the rail limits it) |

## Provenance

- Planned with `fast_section` (`--target-s 60 --time-budget-s 900 --workers
  12`) against the model with the payload and wings, the 1 m floor and both
  end walls. Certified on the seventh attempt, in 414 s. The default 300 s
  budget was not enough: with the flange orbiting the centre at 21 in, the
  arm sweeps further than at 12 in, and the first five starts disconnected
  between layers 325 and 579 of 603.
- Planned from an uncommitted working tree on `feature/rail-end-walls`
  (base `4b23968`), so `fast_section.json`'s code revision does not identify
  the code exactly. The model inputs are pinned by the URDF digest it
  records.
- **Independently re-checked** against the same model by sampling. There
  were no collisions at any 200 Hz sample of the warmup or the task. The
  centre of rotation stays within 0.004 mm of (1.0, 0.5, 0.5) through the
  task, by forward kinematics. Sampled at 20 Hz:

  | | Warmup | Task |
  |---|---|---|
  | Environment clearance (arm) | 47 mm | 47 mm |
  | Self-clearance | 19 mm | 16 mm |
  | Payload to arm links | 44 mm | **17 mm** |
  | Payload to mounting wall | 254 mm | 228 mm |
  | Payload to floor | 706 mm | 997 mm |
  | Payload to end walls | ≥ 0.95 m | ≥ 1.36 m |

  The closest approach is the payload to the arm's own links: 17 mm between
  padded boxes, about 37 mm for the real parts.
- **Not run on the rig.** `T_EG` and the payload dimensions are as
  described, not measured; check them against the hardware before running.

## Run

See the README section "Running the payload plan". The runbook's
`time_scale:=0.1` and `task_time_scale:=1.0` are both accepted; the task at
1.0 is at 73% of the rail's speed limit.
