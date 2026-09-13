#!/usr/bin/env python3
"""Stage 1B.5: a read-only census of why waypoints fail.

Answers a question the summary numbers cannot. "417 of 500" is the length of
the largest contiguous segment, not a count of independently feasible
targets, and a single reason string per waypoint hides simultaneous failures
because _failure_reason applies precedence and describes only the last
attempt.

Four modes, each deterministic:

  tracking   production behaviour, seeded from the previous solution
  fresh      every waypoint independently seeded, no continuity
  wide_bank  every waypoint against a fixed bank of spread seeds
  ablation   tracking with one gate disabled, one run per gate

Reading the result:

  fresh passes where tracking fails          branch SELECTION, greedy
                                             tracking strands itself
  wide_bank passes where fresh fails         branch DISCOVERY, local
                                             perturbation cannot cross
                                             branches
  no mode finds a viable configuration       geometry or placement
  collision or velocity dominates            a posture sweep will not help
  large pose errors among accepted points    the stage 2b acceptance gap is
                                             contaminating the baseline

Discovery and selection are different problems and the distinction matters:
per-waypoint modes can only establish the first. Proving the second needs a
trajectory-level candidate graph, which this does not build.

Nothing here gates anything. Raw pose errors are stored for every attempt so
stage 2b can change tolerances without a re-run.

Usage:
    python3 -m ur10e_trajectory_pkg.failure_census --out census.json
"""
import argparse
import json
import sys

import numpy as np

from ur10e_trajectory_pkg import environment
from ur10e_trajectory_pkg.configurations import LEGACY_MATLAB_START_Q, NUM_JOINTS
from ur10e_trajectory_pkg.pose_metrics import (
    PROVISIONAL_ORIENTATION_TOL_RAD,
    PROVISIONAL_POSITION_TOL_M,
    pose_error,
    within_provisional_tolerance,
)
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

SCHEMA_VERSION = 1

# Fixed and explicit, so the bank is part of the record rather than a detail
# of whoever ran it. These are diagnostic probes for whether a viable branch
# exists at all; they are NOT candidate postures for adoption, which is what
# the stage 1C sweep produces under its own criteria.
WIDE_SEED_BANK_DEG = (
    (0.0, -135.0, 90.0, -90.0, 0.0, 0.0),      # the legacy posture
    (0.0, -135.0, 90.0, -90.0, 45.0, 0.0),     # legacy, off the wrist degeneracy
    (0.0, -90.0, 0.0, -90.0, 0.0, 0.0),        # upstream description default
    (90.0, -60.0, 60.0, -90.0, 90.0, 0.0),     # elbow up, wrist turned
    (-90.0, -120.0, -60.0, -60.0, -90.0, 45.0),  # elbow down, mirrored
    (45.0, -100.0, 110.0, -120.0, 60.0, -90.0),
    (-45.0, -160.0, 120.0, -40.0, -60.0, 90.0),
    (180.0, -90.0, 90.0, -90.0, 90.0, 180.0),  # shoulder reversed
)

GATE_KEYS = (
    'gate_solver_failed',
    'gate_singular',
    'gate_arm_velocity',
    'gate_rail_velocity',
    'gate_collision',
)


def _validator(urdf_path, mesh_path, skip_gate=None):
    """Build a validator, optionally with one gate disabled.

    Ablation runs are kept separate rather than combined, because disabling a
    gate changes which waypoints are accepted and therefore the seed handed to
    every later waypoint. The runs are not comparable point by point.
    """
    validator = TrajectoryValidator(urdf_path, mesh_base_path=mesh_path)
    if skip_gate == 'collision':
        validator.check_all_collisions = lambda q, verbose=False: False
    return validator


def _thresholds(skip_gate):
    """Gate thresholds, with one relaxed to effectively off."""
    values = dict(condition_number_threshold=50.0, max_joint_vel_threshold=2.0)
    if skip_gate == 'singular':
        values['condition_number_threshold'] = float('inf')
    if skip_gate == 'arm_velocity':
        values['max_joint_vel_threshold'] = float('inf')
    return values


def run_tracking(validator, targets, quaternions, dt, q_start, skip_gate=None):
    """Production behaviour, with every attempt recorded."""
    records = []
    segments = validator.find_feasible_segments(
        targets[:, 0], targets[:, 1], targets[:, 2], quaternions, q_start,
        min_length=10, dt_waypoint=dt, verbose=False,
        recorder=records.append, **_thresholds(skip_gate)
    )
    for record in records:
        record['mode'] = 'tracking' if skip_gate is None else f'ablation:{skip_gate}'
    return records, segments


def run_per_waypoint(validator, targets, quaternions, dt, seeds, mode):
    """Each waypoint solved independently, with no continuity between them.

    check_jump is off because there is no previous waypoint to jump from, so
    velocity gates do not apply. That makes this a measure of whether a pose
    is reachable at all, separate from whether it is reachable *next*.
    """
    records = []
    for index in range(len(targets)):
        for seed_number, seed_arm in enumerate(seeds):
            validator.reset_rng()
            validator._solve_waypoint_with_recovery(
                targets[index], quaternions[index], np.asarray(seed_arm),
                rail_pos=float(q_start_rail(seed_arm)),
                check_jump=False, verbose=False,
                recorder=records.append,
                record_context=dict(
                    mode=mode, waypoint_index=index,
                    entry_kind='independent', seed_number=seed_number,
                ),
                **_thresholds(None)
            )
    return records


def q_start_rail(_seed_arm):
    """Rail seed for independent solves.

    Mid-rail, so the solver can move either way without immediately clamping.
    A per-waypoint mode has no previous rail position to continue from.
    """
    return 1.5


def viable(record):
    """A record proves a usable configuration only if everything passes.

    Conditioning alone is not enough: a well-conditioned configuration that
    misses the target is not a solution. Gates that do not apply in this mode
    are None and are ignored rather than treated as failures.
    """
    if not record['configuration_finite'] or record['gate_solver_failed']:
        return False
    if not within_provisional_tolerance(record['position_error_m'],
                                        record['orientation_error_rad']):
        return False
    return not any(record[key] for key in GATE_KEYS if record[key] is not None)


def summarise(all_records, segments, num_waypoints):
    """Per-waypoint rollup across modes, plus segment membership."""
    best = max(segments, key=lambda s: s['length']) if segments else None
    # Indices and length only: a segment also carries its joint configurations,
    # which belong in the attempt records rather than duplicated here.
    longest = None if best is None else {
        'start_idx': int(best['start_idx']),
        'end_idx': int(best['end_idx']),
        'length': int(best['length']),
    }
    member = set()
    if longest:
        member = set(range(longest['start_idx'], longest['end_idx'] + 1))

    by_mode = {}
    for record in all_records:
        by_mode.setdefault(record['mode'], {}).setdefault(
            record['waypoint_index'], []).append(record)

    rows = []
    for index in range(num_waypoints):
        row = {
            'waypoint_index': index,
            'in_largest_segment': index in member,
        }
        for mode, per_waypoint in by_mode.items():
            attempts = per_waypoint.get(index, [])
            row[f'{mode}:attempts'] = len(attempts)
            row[f'{mode}:accepted'] = any(a['accepted'] for a in attempts)
            row[f'{mode}:viable'] = any(viable(a) for a in attempts)
            for key in GATE_KEYS:
                row[f'{mode}:{key}'] = sum(
                    1 for a in attempts if a[key] is True)
        rows.append(row)
    return {'longest_segment': longest, 'waypoints': rows}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    parser.add_argument('--csv', default=None,
                        help='trajectory CSV; defaults to the packaged one')
    parser.add_argument('--waypoints', type=int, default=500)
    parser.add_argument('--out', default='census.json')
    parser.add_argument('--modes', default='tracking,fresh,wide_bank,ablation')
    args = parser.parse_args(argv)

    from ament_index_python.packages import get_package_share_directory
    mesh_path = get_package_share_directory('ur_description')

    targets, quaternions, dt = load_trajectory(args.csv, args.waypoints)
    modes = [m.strip() for m in args.modes.split(',') if m.strip()]

    all_records = []
    segments = []

    if 'tracking' in modes:
        records, segments = run_tracking(
            _validator(args.urdf, mesh_path), targets, quaternions, dt,
            LEGACY_MATLAB_START_Q)
        all_records += records

    if 'fresh' in modes:
        all_records += run_per_waypoint(
            _validator(args.urdf, mesh_path), targets, quaternions, dt,
            [LEGACY_MATLAB_START_Q[1:]], 'fresh')

    if 'wide_bank' in modes:
        all_records += run_per_waypoint(
            _validator(args.urdf, mesh_path), targets, quaternions, dt,
            [np.deg2rad(s) for s in WIDE_SEED_BANK_DEG], 'wide_bank')

    if 'ablation' in modes:
        for gate in ('singular', 'collision', 'arm_velocity'):
            records, _ = run_tracking(
                _validator(args.urdf, mesh_path, skip_gate=gate),
                targets, quaternions, dt, LEGACY_MATLAB_START_Q, skip_gate=gate)
            all_records += records

    document = {
        'schema_version': SCHEMA_VERSION,
        'environment': environment.describe(),
        'provisional_tolerances': {
            'position_m': PROVISIONAL_POSITION_TOL_M,
            'orientation_rad': PROVISIONAL_ORIENTATION_TOL_RAD,
            'note': 'recorded only; nothing is gated on these',
        },
        'wide_seed_bank_deg': [list(s) for s in WIDE_SEED_BANK_DEG],
        'num_waypoints': len(targets),
        'summary': summarise(all_records, segments, len(targets)),
        'attempts': all_records,
    }
    def plain(value):
        """numpy scalars and arrays are not JSON types; convert rather than
        fail after a run that takes minutes."""
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, (np.integer,)):
            return int(value)
        if isinstance(value, (np.floating,)):
            return float(value)
        if isinstance(value, (np.bool_,)):
            return bool(value)
        raise TypeError(f'unserialisable {type(value).__name__}')

    with open(args.out, 'w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=1, sort_keys=True, default=plain)
    print(f'wrote {args.out}: {len(all_records)} attempt records')
    return 0


def load_trajectory(csv_path, num_waypoints):
    """Targets exactly as the service receives them.

    Delegates to the client's own builder rather than rebuilding the
    conversion. An earlier version of this function reimplemented it and
    omitted the hand-placement offset, putting every target in the floor so
    that collision fired on all 5500 tracking attempts and the census
    measured a trajectory nobody runs.
    """
    from ur10e_trajectory_pkg.ClientNode import (
        DEFAULT_CSV_PATH,
        build_trajectory_targets,
    )

    x, y, z, quaternions, times = build_trajectory_targets(
        csv_path or DEFAULT_CSV_PATH, num_waypoints)
    return np.column_stack((x, y, z)), quaternions, float(times[1] - times[0])


if __name__ == '__main__':
    sys.exit(main())
