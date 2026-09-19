#!/usr/bin/env python3
"""Refine a graph path into a smooth joint path, validated before it is kept.

The graph decides the branch, the winding and the rough rail motion. It does
not produce a smooth joint path: its cost counts step size only, so it can hop
between neighbouring solutions along the rail's redundancy at a single
waypoint. Refinement smooths that motion and validates the resulting command.

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
         refined path passes full continuous validation is kept. Only if
         every declared level fails is FALLBACK_RMS_LEVEL_M tried, on the
         same validation, and the result then carries fallback_note saying
         what the declared levels failed and whether those limits were
         assumed. If that fails too, the graph path is kept and the result
         says refinement_failed. An unvalidated path is never substituted.

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

# The jerk gate catches kinks directly; these figures are reported diagnostics,
# not separate acceptance gates.
DIAGNOSTIC_SECOND_DIFFERENCE = 1e-3
DIAGNOSTIC_START_MOTION = 1e-3

# Tried only when every declared level fails. More smoothing can move the
# command away from the graph's chosen branch, so this is never the first rung.
FALLBACK_RMS_LEVEL_M = 0.010


def second_difference_normal_bands(n, pinned):
    """The bands of D^T D for the free samples, and its coupling to the pinned.

    D is the (n-2) x n second-difference operator. Returns (main, first,
    second, coupling): the main, first and second diagonals of the free block
    D^T D[pinned:, pinned:], and the dense (n - pinned) x pinned block
    D^T D[pinned:, :pinned], which is non-zero only in its first rows.
    """
    from scipy import sparse

    count = max(n - pinned, 0)
    if n < 3:
        return (np.zeros(count), np.zeros(max(count - 1, 0)),
                np.zeros(max(count - 2, 0)), np.zeros((count, pinned)))
    D = sparse.diags([1.0, -2.0, 1.0], [0, 1, 2], shape=(n - 2, n), format='csr')
    normal = (D.T @ D).tocsr()
    block = normal[pinned:, pinned:]
    return (block.diagonal(0), block.diagonal(1), block.diagonal(2),
            normal[pinned:, :pinned].toarray())


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

    A = I + lam D^T D is pentadiagonal, so each fit is a banded solve, O(n) in
    time and memory. It was dense: at a full 5000-waypoint recording that is
    a 5000 x 5000 system per bisection step, sixty-odd per level, and three
    5000 x 5000 float arrays held at once. Banded LU pivots as the dense solve
    did, so the fit is the same one.
    """
    from scipy.linalg import solve_banded

    rail = np.asarray(rail, dtype=float)
    n = len(rail)
    fixed = np.full(pinned, rail[0])
    free = slice(pinned, n)
    main, first, second, coupling = second_difference_normal_bands(n, pinned)

    def fit(lam):
        count = n - pinned
        if count <= 0:
            f = fixed[:n].copy()
        else:
            # Rows of the (2, 2)-banded form: two upper, main, two lower; the
            # matrix is symmetric, so upper and lower bands are the same values.
            bands = np.zeros((5, count))
            bands[0, 2:] = lam * second
            bands[1, 1:] = lam * first
            bands[2] = 1.0 + lam * main
            bands[3, :-1] = lam * first
            bands[4, :-2] = lam * second
            b = rail[free] - lam * coupling @ fixed
            f = np.concatenate((fixed, solve_banded((2, 2), bands, b)))
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
    from ur10e_trajectory_pkg.robot_checks import arm_only_ik

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


def violation_provenance(validator, violations):
    """Each violated limit with its provenance and how far it was exceeded.

    A jerk limit with no vendor figure behind it and a certified velocity
    limit read the same in a violations dict. They should not: an artifact
    that records a refusal needs to say which kind of limit refused.
    """
    from ur10e_trajectory_pkg import motion_limits

    statuses = motion_limits.limit_statuses(validator)
    out = {}
    for joint, kinds in violations.items():
        index = list(validator.joint_names).index(joint)
        for kind, detail in kinds.items():
            status = statuses.get(kind, [None] * len(validator.joint_names))[index]
            out[f'{joint}.{kind}'] = {
                'peak': detail['peak'], 'limit': detail['limit'], 'status': status,
                'exceedance_fraction': float(detail['peak'] / detail['limit'] - 1.0)}
    return out


def refine_path(validator, path, positions, quaternions, dt,
                levels=REFINEMENT_RMS_LEVELS_M, rate_hz=REFINEMENT_RATE_HZ,
                fallback_level=FALLBACK_RMS_LEVEL_M):
    """Smoothest refinement level that passes full continuous validation.

    The declared levels are tried smoothest first. Only if every one fails is
    fallback_level tried, and it is accepted on the same full validation as
    any other level; the result then carries fallback_note saying what the
    declared levels failed and whether those limits are assumed.
    """
    from ur10e_trajectory_pkg import continuous_validator

    path = np.asarray(path, dtype=float)
    attempts = []
    ladder = list(levels) + ([fallback_level] if fallback_level is not None else [])
    for level in ladder:
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
            conditioning_ok=report['conditioning_ok'],
            twist_status=report['conditioning']['twist_status'],
            limit_violations=report['limit_violations'],
            # Every part of the verdict, so a failed attempt says why: a 171 s
            # section once failed every level on joint position alone, and
            # the record showed nothing that had failed.
            position_limit_violations=report['position_limit_violations'],
            position_limit_excursions=report.get('position_limit_excursions', {}),
            collision=report['collision']['collision_found'],
            min_self_clearance_m=report['self_clearance']['min_distance_m'],
            self_clearance_links=report['self_clearance']['links'],
            self_clearance_passed=report['self_clearance']['passed'],
            tracking=bool(report['tracking']['within_tolerance']),
            peak_jerk=report['peak_command_stream']['jerk'],
        )
        if attempt['limit_violations']:
            attempt['limit_violation_provenance'] = violation_provenance(
                validator, attempt['limit_violations'])
        attempts.append(attempt)
        if attempt['passed']:
            result = {'status': 'refined', 'level_m': level, 'path': refined,
                      'attempts': attempts, 'validation': report}
            if level == fallback_level and fallback_level not in levels:
                result['fallback_note'] = fallback_note(levels, attempts)
            return result
    return {'status': 'refinement_failed', 'level_m': None, 'path': path,
            'attempts': attempts, 'validation': None}


def fallback_note(levels, attempts):
    """What the declared levels failed, for an artifact to carry.

    Says whether the declared levels fell only to limits that are assumed
    rather than certified, and by how much. The fallback was accepted on full
    validation either way; this is the record that makes the precedent
    visible if a placement ever has no passing rung.
    """
    declared = []
    for attempt in attempts:
        if attempt['level_m'] not in levels:
            continue
        provenance = attempt.get('limit_violation_provenance', {})
        declared.append({
            'level_m': attempt['level_m'],
            'passed': attempt['passed'],
            'limit_violations': provenance,
            'all_violated_limits_assumed': bool(provenance) and all(
                entry['status'] == 'assumed' for entry in provenance.values()),
            'worst_exceedance_fraction': (max(entry['exceedance_fraction']
                                              for entry in provenance.values())
                                          if provenance else None),
            'other_failures': sorted(
                key for key, value in (
                    ('collision', attempt.get('collision')),
                    ('self_clearance', attempt.get('self_clearance_passed') is False),
                    ('tracking', attempt.get('tracking') is False),
                ) if value),
        })
    return {'declared_levels_m': list(levels), 'declared_attempts': declared,
            'accepted_on': 'full continuous validation at the fallback level'}


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

    from ur10e_trajectory_pkg.planning_runtime import (
        make_validator as _validator,
        load_trajectory,
    )

    with open(args.graph, encoding='utf-8') as handle:
        graph = json.load(handle)
    if not graph.get('complete_path'):
        print('graph path incomplete; nothing to refine')
        return 1
    validator = _validator(args.urdf, get_package_share_directory('ur_description'))
    spin_up = graph.get('spin_up')
    placement = graph.get('placement', 'nominal')
    if placement != 'nominal':
        raise ValueError('only the fixed nominal target placement is supported')
    positions, quaternions, dt, _ = load_trajectory(
        None, graph['recorded_waypoints'], with_metadata=True,
        spin_up_s=None if spin_up is None else spin_up['requested_duration_s'])
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
    # The reason the fallback level was reached is the point of having it;
    # dropping the note here would leave the artifact saying only that a
    # smoother level passed.
    if result.get('fallback_note') is not None:
        document['fallback_note'] = result['fallback_note']
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
