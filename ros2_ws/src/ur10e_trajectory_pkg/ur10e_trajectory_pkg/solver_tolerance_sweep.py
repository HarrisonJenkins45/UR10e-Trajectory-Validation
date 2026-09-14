#!/usr/bin/env python3
"""Does the solver's stopping tolerance, not the task, set our pose error?

The census showed accepted orientation error with a median of 0.27 deg and a
viability cliff between 0.25 and 0.5 deg. That is a description of the
stopping rule rather than of the task. ikine_LM minimises the quadratic error

    E = 0.5 * e.T @ We @ e

over the 6-vector angle-axis error e, and stops at E < tol. With tol = 1e-4
that admits |e| up to sqrt(2e-4) = 0.0141, about 0.81 deg when angular error
dominates, which is exactly where the observed distribution sits.

So the question is not what tolerance the current distribution justifies. It
is whether the solver can meet the accuracy the task wants, and at what cost.
This sweeps tol and reports pose error, convergence, effort and the resulting
segment, so acceptance limits can be set from requirements rather than from
whatever the solver happens to emit.

Usage:
    python3 -m ur10e_trajectory_pkg.solver_tolerance_sweep --out sweep.json
"""
import argparse
import json
import sys
import time

import numpy as np

from ur10e_trajectory_pkg import environment
from ur10e_trajectory_pkg.configurations import LEGACY_MATLAB_START_Q
from ur10e_trajectory_pkg.failure_census import (
    GATE_KEYS,
    load_trajectory,
    run_tracking,
    viable,
)
from ur10e_trajectory_pkg.pose_metrics import (
    IK_ORIENTATION_TOL_RAD,
    IK_POSITION_TOL_M,
)
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

SCHEMA_VERSION = 1

# Spanning four orders of magnitude. 5e-7 is the value implied by a 1 mm
# translation bound under equal weighting; it is also tighter than the roughly
# 1.5e-6 implied by 0.1 deg of orientation alone, so if one scalar has to
# serve both, this is the binding one.
TOLERANCES = (1e-4, 1e-5, 1e-6, 5e-7, 1e-7, 1e-8)


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(int(fraction * len(ordered)), len(ordered) - 1)]


def measure(urdf_path, mesh_path, targets, quaternions, dt, solver_tol):
    """One tracking run at a given stopping tolerance."""
    validator = TrajectoryValidator(urdf_path, mesh_base_path=mesh_path,
                                    solver_tol=solver_tol)
    started = time.perf_counter()
    records, segments = run_tracking(
        validator, targets, quaternions, dt, LEGACY_MATLAB_START_Q)
    elapsed = time.perf_counter() - started

    accepted = [r for r in records if r['accepted']]
    position = [r['position_error_m'] for r in accepted]
    orientation = [r['orientation_error_rad'] for r in accepted]
    iterations = [r['solver_iterations'] for r in records
                  if r['solver_iterations'] >= 0]
    longest = max((s['length'] for s in segments), default=0)

    return {
        'solver_tol': solver_tol,
        'seconds': round(elapsed, 2),
        'attempts': len(records),
        'accepted': len(accepted),
        'converged': sum(1 for r in records if not r['gate_solver_failed']),
        'longest_segment': int(longest),
        'viable_waypoints': len({r['waypoint_index'] for r in records if viable(r)}),
        'position_error_m': {
            'median': percentile(position, 0.5),
            'p90': percentile(position, 0.9),
            'max': max(position) if position else None,
        },
        'orientation_error_rad': {
            'median': percentile(orientation, 0.5),
            'p90': percentile(orientation, 0.9),
            'max': max(orientation) if orientation else None,
        },
        'within_tolerance': sum(
            1 for r in accepted
            if r['position_error_m'] <= IK_POSITION_TOL_M
            and r['orientation_error_rad'] <= IK_ORIENTATION_TOL_RAD),
        'iterations': {
            'median': percentile(iterations, 0.5),
            'max': max(iterations) if iterations else None,
        },
        'gate_counts': {
            key: sum(1 for r in records if r[key] is True) for key in GATE_KEYS
        },
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    parser.add_argument('--waypoints', type=int, default=500)
    parser.add_argument('--out', default='sweep.json')
    args = parser.parse_args(argv)

    from ament_index_python.packages import get_package_share_directory
    mesh_path = get_package_share_directory('ur_description')
    targets, quaternions, dt = load_trajectory(None, args.waypoints)

    rows = [measure(args.urdf, mesh_path, targets, quaternions, dt, tol)
            for tol in TOLERANCES]

    document = {
        'schema_version': SCHEMA_VERSION,
        'environment': environment.describe(),
        'num_waypoints': len(targets),
        'acceptance_tolerances': {
            'position_m': IK_POSITION_TOL_M,
            'orientation_rad': IK_ORIENTATION_TOL_RAD,
        },
        'rows': rows,
    }
    with open(args.out, 'w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=1, sort_keys=True)

    header = (f"{'tol':>8} {'seg':>5} {'viable':>7} {'ok@prov':>8} "
              f"{'ori med':>9} {'ori p90':>9} {'pos med':>9} "
              f"{'iters':>6} {'sec':>7}")
    print(header)
    for row in rows:
        print(f"{row['solver_tol']:8.0e} {row['longest_segment']:5d} "
              f"{row['viable_waypoints']:7d} {row['within_tolerance']:8d} "
              f"{np.rad2deg(row['orientation_error_rad']['median'] or 0):8.4f}d "
              f"{np.rad2deg(row['orientation_error_rad']['p90'] or 0):8.4f}d "
              f"{row['position_error_m']['median'] or 0:9.2e} "
              f"{row['iterations']['median']:6} {row['seconds']:7.1f}")
    print(f'\nwrote {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
