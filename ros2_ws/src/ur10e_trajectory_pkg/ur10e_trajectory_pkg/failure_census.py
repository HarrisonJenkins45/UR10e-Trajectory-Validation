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

  fresh passes where tracking fails          continuation failure, consistent
                                             with branch SELECTION or with
                                             local DISCOVERY failing at that
                                             waypoint; these modes cannot
                                             separate the two
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
import hashlib
import json
import os
import subprocess
import sys

import numpy as np

from ur10e_trajectory_pkg import environment
from ur10e_trajectory_pkg.configurations import LEGACY_MATLAB_START_Q, NUM_JOINTS
from ur10e_trajectory_pkg.pose_metrics import (
    IK_ORIENTATION_TOL_RAD,
    IK_POSITION_TOL_M,
    pose_error,
    within_pose_tolerance,
)
from ur10e_trajectory_pkg.validation_core import (
    IK_SEARCH_LIMIT,
    IK_SOLVER_TOL,
    RAIL_VEL_SAFETY_CAP,
    TrajectoryValidator,
)

# 3: configurations are now LIFTED, q + 2*pi*k chosen per joint, so q_full is
#    continuous with its predecessor rather than wrapped into [-pi, pi]. Adds
#    q_arm_canonical, winding and arm_delta_canonical_rad alongside the lifted
#    delta. Version 2 configurations are canonical, so the two are not
#    directly comparable and the schema had to move.
#
#    `viable` is DEPRECATED and now duplicates `accepted`, because production
#    acceptance includes the pose check that viable was invented to supply.
#    Retained so version 2 readers keep working. The graph-facing replacement
#    is node_valid, the pose and configuration checks, against
#    transition_valid, the predecessor-dependent velocity and swept-collision
#    checks.
#
# 2: adds gate_pose_position and gate_pose_orientation, and `accepted` means
#    production acceptance INCLUDING the forward-kinematics pose check.
#    Version 1 artifacts stay interpretable because they retained raw pose
#    errors, so the new flags can be recomputed from them.
SCHEMA_VERSION = 3

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

# Shortest run of feasible waypoints the segment scan will report.
MIN_SEGMENT_LENGTH = 10

GATE_KEYS = (
    'gate_solver_failed',
    'gate_pose_position',
    'gate_pose_orientation',
    'gate_singular',
    'gate_arm_velocity',
    'gate_rail_velocity',
    'gate_collision',
)


def file_digest(path):
    """sha256 of an input file, so an artifact names the bytes it used."""
    try:
        with open(path, 'rb') as handle:
            return hashlib.sha256(handle.read()).hexdigest()
    except OSError:
        return None


def repository_revision(path):
    """Commit the workspace was at.

    Falls back to UR10E_GIT_REVISION because the container mounts ros2_ws
    alone, and the .git directory sits at the repository root above it, so git
    is not reachable from inside. The runner passes it instead of the artifact
    silently recording null.
    """
    from_env = os.environ.get('UR10E_GIT_REVISION')
    if from_env:
        return from_env.strip()
    try:
        result = subprocess.run(
            ['git', '-c', f'safe.directory={path}', '-C', str(path),
             'rev-parse', 'HEAD'],
            capture_output=True, text=True, timeout=10, check=False)
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() or None


def manifest(args, validator, trajectory_metadata, modes, dt):
    """Every load-bearing input, so the artifact is independently reproducible.

    Library versions alone are not enough: a verdict also depends on the input
    bytes, the gate thresholds, the solver settings, and where the trajectory
    was placed. Recording them here means a result can be reproduced, or shown
    to be irreproducible, without reading the code that produced it.

    The image id is not discoverable from inside the container, so it comes
    from UR10E_IMAGE_ID and is recorded as null when the runner does not set
    it, rather than being guessed at.
    """
    workspace = environment.workspace_root()
    return {
        'repository_revision': repository_revision(workspace),
        'docker_image_id': os.environ.get('UR10E_IMAGE_ID'),
        'inputs': {
            'urdf_path': args.urdf,
            'urdf_sha256': file_digest(args.urdf),
            'csv_path': trajectory_metadata['csv_path'],
            'csv_sha256': file_digest(trajectory_metadata['csv_path']),
        },
        'solver': {
            'tolerance': IK_SOLVER_TOL,
            'search_limit': IK_SEARCH_LIMIT,
            'rng_seed': validator._seed,
        },
        'gate_thresholds': {
            'condition_number': 50.0,
            'arm_velocity_rad_s': 2.0,
            'rail_velocity_m_s': validator._rail_vel_limit,
            'rail_velocity_safety_cap_m_s': RAIL_VEL_SAFETY_CAP,
            'rail_position_limits_m': list(validator._rail_limits),
            'pose_position_m': IK_POSITION_TOL_M,
            'pose_orientation_rad': IK_ORIENTATION_TOL_RAD,
        },
        'trajectory': dict(trajectory_metadata, dt_waypoint_s=dt),
        'run': {
            'q_start': LEGACY_MATLAB_START_Q.tolist(),
            'q_start_name': 'LEGACY_MATLAB_START_Q',
            'independent_solve_rail_seed_m': q_start_rail(None),
            'min_segment_length': MIN_SEGMENT_LENGTH,
            'modes': list(modes),
            'wide_seed_bank_deg': [list(s) for s in WIDE_SEED_BANK_DEG],
        },
    }


def _validator(urdf_path, mesh_path, skip_gate=None, solver_tol=None):
    """Build a validator, optionally with one gate disabled.

    Ablation runs are kept separate rather than combined, because disabling a
    gate changes which waypoints are accepted and therefore the seed handed to
    every later waypoint. The runs are not comparable point by point.
    """
    kwargs = {} if solver_tol is None else {'solver_tol': solver_tol}
    validator = TrajectoryValidator(urdf_path, mesh_base_path=mesh_path, **kwargs)
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
        min_length=MIN_SEGMENT_LENGTH, dt_waypoint=dt, verbose=False,
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
    if not within_pose_tolerance(record['position_error_m'],
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

    targets, quaternions, dt, trajectory_metadata = load_trajectory(
        args.csv, args.waypoints, with_metadata=True)
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
        'manifest': manifest(args, _validator(args.urdf, mesh_path),
                             trajectory_metadata, modes, dt),
        'acceptance_tolerances': {
            'position_m': IK_POSITION_TOL_M,
            'orientation_rad': IK_ORIENTATION_TOL_RAD,
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


def load_trajectory(csv_path, num_waypoints, with_metadata=False):
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

    result = build_trajectory_targets(
        csv_path or DEFAULT_CSV_PATH, num_waypoints,
        return_metadata=with_metadata)
    if with_metadata:
        (x, y, z, quaternions, times), metadata = result
        return (np.column_stack((x, y, z)), quaternions,
                float(times[1] - times[0]), metadata)
    x, y, z, quaternions, times = result
    return np.column_stack((x, y, z)), quaternions, float(times[1] - times[0])


if __name__ == '__main__':
    sys.exit(main())
