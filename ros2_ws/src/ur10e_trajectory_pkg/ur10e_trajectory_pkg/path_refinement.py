#!/usr/bin/env python3
"""Refine a graph path into a smooth joint path, validated before it is kept.

The graph decides the branch, the winding and the rough rail motion. It does
not produce a smooth joint path: its cost counts step size only, so it can hop
between neighbouring solutions along the rail's redundancy at a single
waypoint. At coupled_14 such hops put 1,108 rad/s^3 of jerk on the elbow; 8 of
28 placements already had them before continuation seeding existed.

Refinement, in the per-trajectory pipeline for good:

    graph -> refinement -> start winding and warmup -> both validations

  rail   smoothed through the graph path's rail positions by penalised least
         squares on second differences, with the first three samples held at
         the start's rail, so the rail's entry velocity AND acceleration are
         zero under the PCHIP playback uses
  arm    solved at each smoothed rail value, starting from the previous arm
         solution. With the rail fixed the arm has only a few solutions, so it
         cannot drift to another branch
  level  REFINEMENT_RMS_LEVELS_M, fixed in advance: the smoothest level whose
         refined path passes full continuous validation is kept. If none
         passes, the graph path is kept and the result says
         refinement_failed. An unvalidated path is never substituted.

The start is pinned, so the start winding, the warmup and the home are
unchanged by refinement.

Usage:
    python3 -m ur10e_trajectory_pkg.path_refinement --graph graph.json \\
        --out refined.json
"""
import argparse
import json
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from ur10e_trajectory_pkg.pose_metrics import IK_ORIENTATION_TOL_RAD, IK_POSITION_TOL_M

SCHEMA_VERSION = 1
# RMS rail deviation from the graph path, smoothest first.
REFINEMENT_RMS_LEVELS_M = (0.005, 0.002, 0.001)
PINNED_START_SAMPLES = 3
REFINEMENT_RATE_HZ = 200.0

# Acceptance, the same at every placement: full continuous validation (limits
# including jerk, collision, self-clearance, tracking, condition <= 50,
# alpha*) and start motion within the declared at-rest tolerance. The 1e-3
# figures were stand-ins for kinks in the coupled_14 prototype; the jerk gate
# catches kinks directly, so they are reported diagnostics, not gates.
DIAGNOSTIC_SECOND_DIFFERENCE = 1e-3
DIAGNOSTIC_START_MOTION = 1e-3


def smooth_rail(rail, rms_target, pinned=PINNED_START_SAMPLES, bounds=None):
    """Rail positions smoothed to about rms_target, the start held fixed.

    Minimises |f - rail|^2 + lam |D f|^2 with D the second-difference
    operator and f[0:pinned] = rail[0]; lam is bisected (in log space) so the
    RMS deviation meets rms_target. If even the smoothest fit stays below the
    target, that fit is returned. Returns (f, lam, rms).

    bounds, when given as (lower, upper), keeps the smoothed rail inside the
    joint's travel. Smoothing a path that runs near an end of the rail cuts
    the corner past it: coupled_08 left the limit by 8 mm and every level
    failed validation on position limits, with nothing else wrong. The clip
    is applied inside the fit, so the bisection measures the deviation the
    caller actually gets rather than one the clip then changes.
    """
    rail = np.asarray(rail, dtype=float)
    n = len(rail)
    D = np.zeros((n - 2, n))
    for i in range(n - 2):
        D[i, i:i + 3] = (1.0, -2.0, 1.0)
    DtD = D.T @ D
    fixed = np.full(pinned, rail[0])
    free = slice(pinned, n)

    def fit(lam):
        A = np.eye(n - pinned) + lam * DtD[free, free]
        b = rail[free] - lam * DtD[free, :pinned] @ fixed
        f = np.concatenate((fixed, np.linalg.solve(A, b)))
        if bounds is not None:
            f = np.clip(f, bounds[0], bounds[1])
        return f, float(np.sqrt(np.mean((f - rail) ** 2)))

    low, high = -8.0, 14.0
    f_high, rms_high = fit(10.0 ** high)
    if rms_high <= rms_target:
        return f_high, 10.0 ** high, rms_high
    for _ in range(60):
        middle = 0.5 * (low + high)
        _, rms = fit(10.0 ** middle)
        if rms > rms_target:
            high = middle
        else:
            low = middle
    f, rms = fit(10.0 ** low)
    return f, 10.0 ** low, rms


def solve_arm_along(validator, path, rail, positions, quaternions):
    """Arm solved at each rail value from the previous arm solution.

    Returns (configurations, report) or (None, report) naming the first
    waypoint that could not be solved within the IK pose tolerances.
    """
    from ur10e_trajectory_pkg.ready_pose_sweep import arm_only_ik

    path = np.asarray(path, dtype=float)
    out = []
    previous = path[0].copy()
    worst_deviation = 0.0
    for index in range(len(path)):
        seed = previous.copy()
        seed[0] = rail[index]
        rotation = Rotation.from_quat(quaternions[index]).as_matrix()
        solution, _, _ = arm_only_ik(validator, seed, positions[index], rotation)
        pose = validator.robot.fkine(solution, end='tool0')
        position_error = float(np.linalg.norm(pose.t - np.asarray(positions[index])))
        orientation_error = float(np.linalg.norm(
            (Rotation.from_quat(quaternions[index]).inv()
             * Rotation.from_matrix(pose.R)).as_rotvec()))
        if position_error > IK_POSITION_TOL_M or orientation_error > IK_ORIENTATION_TOL_RAD:
            return None, {'failed_at': index, 'position_error_m': position_error,
                          'orientation_error_rad': orientation_error}
        worst_deviation = max(worst_deviation, float(np.max(np.abs(
            solution[1:] - path[index][1:]))))
        out.append(solution)
        previous = solution
    return np.stack(out), {'max_arm_deviation_from_graph_rad': worst_deviation}


def refine_path(validator, path, positions, quaternions, dt,
                levels=REFINEMENT_RMS_LEVELS_M, rate_hz=REFINEMENT_RATE_HZ):
    """Smoothest refinement level that passes full continuous validation."""
    from ur10e_trajectory_pkg import continuous_validator

    path = np.asarray(path, dtype=float)
    attempts = []
    for level in levels:
        lower, upper = validator.robot.qlim
        rail, lam, rms = smooth_rail(path[:, 0], level,
                                     bounds=(float(lower[0]), float(upper[0])))
        refined, solve = solve_arm_along(validator, path, rail, positions, quaternions)
        attempt = {'level_m': level, 'rail_rms_m': rms, 'smoothing_weight': lam, **solve}
        if refined is None:
            attempts.append(dict(attempt, passed=False, reason='arm could not be solved'))
            continue
        report = continuous_validator.validate_task_command(
            validator, refined, positions, quaternions, dt, rate_hz=rate_hz)
        attempt.update(
            passed=bool(report['passed']),
            max_second_difference=float(np.max(np.abs(np.diff(refined, n=2, axis=0)))),
            start_motion=max(report['entry_velocity_ratio'],
                             report['entry_acceleration_ratio']),
            max_condition_number=report['conditioning']['max_condition_number'],
            twist_status=report['conditioning']['twist_status'],
            limit_violations=report['limit_violations'],
            collision=report['collision']['collision_found'],
            min_self_clearance_m=report['self_clearance']['min_distance_m'],
            self_clearance_links=report['self_clearance']['links'],
            self_clearance_passed=report['self_clearance']['passed'],
            tracking=bool(report['tracking']['within_tolerance']),
            peak_jerk=report['peak_command_stream']['jerk'],
        )
        attempts.append(attempt)
        if attempt['passed']:
            return {'status': 'refined', 'level_m': level, 'path': refined,
                    'attempts': attempts, 'validation': report}
    return {'status': 'refinement_failed', 'level_m': None, 'path': path,
            'attempts': attempts, 'validation': None}


def meets_acceptance(result):
    """Full validation plus start motion within the at-rest tolerance."""
    from ur10e_trajectory_pkg.graph_planner import AT_REST_TOLERANCE_FRACTION

    if result['status'] != 'refined':
        return False
    chosen = next(a for a in result['attempts'] if a['level_m'] == result['level_m'])
    return bool(chosen['passed']
                and chosen['start_motion'] <= AT_REST_TOLERANCE_FRACTION)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--graph', required=True)
    parser.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    parser.add_argument('--out', required=True)
    args = parser.parse_args(argv)

    from ament_index_python.packages import get_package_share_directory

    from ur10e_trajectory_pkg.failure_census import (
        _validator,
        load_trajectory,
        placement_RG_for,
    )

    with open(args.graph, encoding='utf-8') as handle:
        graph = json.load(handle)
    if not graph.get('complete_path'):
        print('graph path incomplete; nothing to refine')
        return 1
    validator = _validator(args.urdf, get_package_share_directory('ur_description'))
    spin_up = graph.get('spin_up')
    placement = graph.get('placement', 'nominal')
    positions, quaternions, dt, _ = load_trajectory(
        None, graph['recorded_waypoints'], with_metadata=True,
        spin_up_s=None if spin_up is None else spin_up['requested_duration_s'],
        placement_RG=None if placement == 'nominal' else placement_RG_for(placement))
    result = refine_path(validator, graph['path'], positions, quaternions, dt)

    def plain(value):
        if isinstance(value, np.bool_):
            return bool(value)
        if isinstance(value, (np.integer, np.floating)):
            return value.item()
        if isinstance(value, np.ndarray):
            return value.tolist()
        raise TypeError(type(value).__name__)

    document = {'schema_version': SCHEMA_VERSION, 'graph': args.graph,
                'placement': placement, 'status': result['status'],
                'level_m': result['level_m'], 'attempts': result['attempts'],
                'meets_acceptance': meets_acceptance(result),
                'acceptance': 'full continuous validation and start motion '
                              'within AT_REST_TOLERANCE_FRACTION',
                'diagnostics': {'second_difference_reference': DIAGNOSTIC_SECOND_DIFFERENCE,
                                'start_motion_reference': DIAGNOSTIC_START_MOTION},
                'path': np.asarray(result['path']).tolist()}
    with open(args.out, 'w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=1, default=plain)
    for attempt in result['attempts']:
        print(f"level {attempt['level_m'] * 1000:.0f} mm: passed={attempt['passed']} "
              f"rms={attempt['rail_rms_m'] * 1000:.2f} mm "
              f"max_second_difference={attempt.get('max_second_difference')} "
              f"start_motion={attempt.get('start_motion')} "
              f"max_cond={attempt.get('max_condition_number')} "
              f"self_clearance={attempt.get('min_self_clearance_m')} "
              f"arm_deviation={attempt.get('max_arm_deviation_from_graph_rad')} "
              f"violations={list(attempt.get('limit_violations') or [])}")
    print(f"{placement}: {result['status']} at level {result['level_m']} | "
          f"meets acceptance {document['meets_acceptance']}")
    return 0 if result['status'] == 'refined' else 1


if __name__ == '__main__':
    sys.exit(main())
