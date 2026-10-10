#!/usr/bin/env python3
"""Generate distinct rail-and-arm IK candidates for each target waypoint.

Seed coverage combines tracking-derived rail positions, independent arm
postures, a coarse rail grid, arm-by-rail combinations and continuation from
preceding candidates. Canonical joint values identify kinematic alternatives;
full-turn lifts are expanded later from each predecessor during graph search.

Velocity is deliberately not checked here. It is a property of a transition
between nodes, so it belongs to graph edges; enforcing it during node
generation would discard candidates that are perfectly good successors of
some other predecessor.

Usage:
    python3 -m ur10e_trajectory_pkg.candidate_generator --csv recording.csv \\
        --out candidates.json
"""
import argparse
import json
import sys

import numpy as np

from ur10e_trajectory_pkg import environment
from ur10e_trajectory_pkg.configurations import (
    ARM_SLICE,
    LEGACY_MATLAB_START_Q,
    RAIL_INDEX,
)
from ur10e_trajectory_pkg.planning_runtime import (
    WIDE_SEED_BANK_DEG,
    make_validator as _validator,
    load_trajectory,
    candidate_manifest as manifest,
    run_tracking,
)

SCHEMA_VERSION = 1

# Declared, not tuned: a coarse sweep of the rail's 0 to 3 m travel.
RAIL_GRID_M = (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0)

# Upstream UR description's initial posture, as the fixed arm seed for rail
# isolation. Chosen because it is external to this project, so it cannot be
# accused of having been picked to flatter the result.
UPSTREAM_ARM_SEED_DEG = (0.0, -90.0, 0.0, -90.0, 0.0, 0.0)

# Decimal places for deduplication. 1e-6 rad is far below any physical
# distinction and far above solver noise, so equality here means the solver
# returned the same configuration twice.
DEDUP_DECIMALS = 6


def tracking_reference_path(validator, targets, quaternions, dt, q_start):
    """The corrected greedy path, as the reference every arm is seeded from.

    Returns (rail_seed_per_waypoint, arm_config_per_waypoint). The rail seed
    for waypoint 0 is q_start's rail; afterwards it is the rail the tracker
    committed to at the preceding waypoint. Using a fixed 1.5 m instead, as
    an earlier experiment did, measures reachability from a rail position
    tracking never visits and confounds the arm effect with a rail offset.
    """
    records, _ = run_tracking(validator, targets, quaternions, dt, q_start)
    committed = {}
    for record in records:
        if record['accepted']:
            committed.setdefault(record['waypoint_index'], record)

    rail_seeds = np.empty(len(targets))
    arm_configs = np.empty((len(targets), 6))
    rail_seeds[0] = q_start[RAIL_INDEX]
    arm_configs[0] = q_start[ARM_SLICE]
    for index in range(len(targets)):
        record = committed.get(index)
        if record is not None:
            arm_configs[index] = np.asarray(record['q_full'])[ARM_SLICE]
        elif index:
            arm_configs[index] = arm_configs[index - 1]
        if index + 1 < len(targets):
            rail_seeds[index + 1] = (
                np.asarray(record['q_full'])[RAIL_INDEX] if record is not None
                else rail_seeds[index])
    return rail_seeds, arm_configs


def solve_one(validator, target, quaternion, arm_seed, rail_seed, context):
    """One independent solve, recording every attempt.

    The recovery generator is reset first, so a solve depends only on its own
    seeds and not on how many solves preceded it.
    """
    records = []
    validator.reset_rng()
    validator._solve_waypoint_with_recovery(
        target, quaternion, np.asarray(arm_seed, dtype=float),
        rail_pos=float(rail_seed),
        check_jump=False,          # velocity is an edge property
        verbose=False,
        recorder=records.append,
        record_context=dict(context, rail_seed=float(rail_seed)),
    )
    return records


def run_arm_isolation(validator, targets, quaternions, rail_seeds, arm_seeds):
    out = []
    for index in range(len(targets)):
        for seed_number, arm_seed in enumerate(arm_seeds):
            out += solve_one(
                validator, targets[index], quaternions[index], arm_seed,
                rail_seeds[index],
                dict(mode='arm_isolation', waypoint_index=index,
                     entry_kind='independent', arm_seed_number=seed_number,
                     rail_seed_number=None))
    return out


def run_rail_isolation(validator, targets, quaternions, arm_configs):
    """One arm seed, a grid of rail seeds, two choices of that arm seed."""
    upstream = np.deg2rad(UPSTREAM_ARM_SEED_DEG)
    out = []
    for index in range(len(targets)):
        previous_arm = arm_configs[index - 1] if index else arm_configs[0]
        for label, arm_seed in (('tracking_prev', previous_arm),
                                ('upstream_default', upstream)):
            for rail_number, rail_seed in enumerate(RAIL_GRID_M):
                out += solve_one(
                    validator, targets[index], quaternions[index], arm_seed,
                    rail_seed,
                    dict(mode=f'rail_isolation:{label}', waypoint_index=index,
                         entry_kind='independent', arm_seed_number=None,
                         rail_seed_number=rail_number))
    return out


def run_interaction(validator, targets, quaternions, arm_seeds):
    out = []
    for index in range(len(targets)):
        for seed_number, arm_seed in enumerate(arm_seeds):
            for rail_number, rail_seed in enumerate(RAIL_GRID_M):
                out += solve_one(
                    validator, targets[index], quaternions[index], arm_seed,
                    rail_seed,
                    dict(mode='interaction', waypoint_index=index,
                         entry_kind='independent',
                         arm_seed_number=seed_number,
                         rail_seed_number=rail_number))
    return out


EXTRA_SEED_MODE = 'extra_seed'
CONTINUATION_MODE = 'continuation'

# Declared before any run with merging: a continuation result within this of
# an existing candidate at the same waypoint -- wrapped radians on every arm
# joint, metres on the rail -- is the same solution, not a new one. Far below
# one step's velocity budget (0.1 m on the rail, at least 0.2 rad on the arm),
# so merging cannot remove a reachable successor except at the very edge of
# that budget.
CONTINUATION_MERGE_TOLERANCE = 0.01


def _vector(entry):
    return np.concatenate(([entry['rail_position']], entry['q_arm_canonical']))


def _within(a, b, tolerance):
    return (abs(a[RAIL_INDEX] - b[RAIL_INDEX]) <= tolerance
            and float(np.max(np.abs(np.angle(np.exp(1j * (a[ARM_SLICE] - b[ARM_SLICE]))))))
            <= tolerance)


def run_continuation(validator, targets, quaternions, base_records,
                     tolerance=CONTINUATION_MERGE_TOLERANCE):
    """Seed each waypoint from every distinct candidate at the previous one.

    Every other mode solves each waypoint independently, so a branch the
    graph was following could go unfound at one waypoint and disconnect it.
    Measured at coupled_14: the only reachable branch had candidates at 355
    and 357 but none at 356, and re-solving 356 from the branch returned valid
    solutions every time.

    Results are MERGED. The rail makes the arm redundant, so re-solving from a
    candidate lands a hair away from solutions already found, and exact
    deduplication cannot merge them: unmerged, the count grew by the whole
    independent count at every waypoint (49, 98, 159, ... 706 over 12 layers
    at coupled_14; a median 0.0016 from an existing candidate, 92% within
    0.01). A result within tolerance of an existing candidate is not a new
    candidate; it is returned separately so its provenance can be attached to
    the one it matches. Seeds come from the merged set, so growth is bounded
    by the number of genuinely distinct solutions.

    Returns (kept_records, merged_records).
    """
    by_waypoint = {}
    for record in base_records:
        by_waypoint.setdefault(record['waypoint_index'], []).append(record)
    kept, merged = [], []
    previous = collect_candidates(by_waypoint.get(0, [])).get(0, [])
    for index in range(1, len(targets)):
        existing = [_vector(e) for e in
                    collect_candidates(by_waypoint.get(index, [])).get(index, [])]
        kept_here = []
        for number, entry in enumerate(previous):
            for record in solve_one(
                    validator, targets[index], quaternions[index],
                    np.asarray(entry['q_arm_canonical'], dtype=float),
                    entry['rail_position'],
                    dict(mode=CONTINUATION_MODE, waypoint_index=index,
                         entry_kind='continuation', arm_seed_number=number,
                         rail_seed_number=None,
                         seed_origin=f'waypoint{index - 1}:candidate{number}')):
                if not record['accepted']:
                    kept_here.append(record)
                    continue
                vector = np.concatenate(([record['rail_position']],
                                         record['q_arm_canonical']))
                if any(_within(vector, other, tolerance) for other in existing):
                    merged.append(record)
                else:
                    existing.append(vector)
                    kept_here.append(record)
        kept += kept_here
        previous = collect_candidates(
            by_waypoint.get(index, []) + kept_here).get(index, [])
    return kept, merged


def attach_merged_provenance(candidates, merged_records,
                             tolerance=CONTINUATION_MERGE_TOLERANCE):
    """Record merged continuation results on the candidate each one matched."""
    for record in merged_records:
        entries = candidates.get(record['waypoint_index'], [])
        vector = np.concatenate(([record['rail_position']], record['q_arm_canonical']))
        matches = [e for e in entries if _within(vector, _vector(e), tolerance)]
        if not matches:
            continue
        nearest = min(matches, key=lambda e: float(np.max(np.abs(
            np.angle(np.exp(1j * (vector - _vector(e))))))))
        nearest['provenance'].append({
            'mode': CONTINUATION_MODE, 'merged_within': tolerance,
            'seed_origin': record.get('seed_origin'),
            'attempt': record['attempt'],
            'solver_iterations': record['solver_iterations'],
            'arm_seed_number': record.get('arm_seed_number'),
            'rail_seed_number': None, 'rail_seed': record.get('rail_seed'),
        })
    return candidates


def run_extra_seeds(validator, targets, quaternions, extra_seeds):
    """Additional seeds per waypoint, each tagged with where it came from.

    extra_seeds[index] is a list of (configuration, origin). Used to add
    another placement's candidates as seeds without letting them stand in for
    independent generation: their provenance stays on every candidate they
    produce, so what they alone found is countable.
    """
    out = []
    for index in range(len(targets)):
        for seed_number, (configuration, origin) in enumerate(
                extra_seeds[index] if index < len(extra_seeds) else []):
            configuration = np.asarray(configuration, dtype=float)
            out += solve_one(
                validator, targets[index], quaternions[index],
                configuration[ARM_SLICE], configuration[RAIL_INDEX],
                dict(mode=EXTRA_SEED_MODE, waypoint_index=index,
                     entry_kind='independent', arm_seed_number=seed_number,
                     rail_seed_number=None, seed_origin=origin))
    return out


def generate_layers(validator, targets, quaternions, dt, q_start,
                    num_layers=3, extra_seeds=None, continuation=True):
    """Candidates for the first num_layers waypoints of one placement.

    The same three modes as the full generator, restricted to a prefix. The
    tracker is causal, so the rail seeds it derives for these waypoints are
    the ones a full-length run would derive.
    """
    targets = np.asarray(targets)[:num_layers]
    quaternions = np.asarray(quaternions)[:num_layers]
    rail_seeds, arm_configs = tracking_reference_path(
        validator, targets, quaternions, dt, q_start)
    arm_seeds = [np.deg2rad(seed) for seed in WIDE_SEED_BANK_DEG]

    records = run_arm_isolation(validator, targets, quaternions, rail_seeds,
                                arm_seeds)
    records += run_rail_isolation(validator, targets, quaternions, arm_configs)
    records += run_interaction(validator, targets, quaternions, arm_seeds)
    if extra_seeds:
        records += run_extra_seeds(validator, targets, quaternions, extra_seeds)
    merged = []
    if continuation:
        kept, merged = run_continuation(validator, targets, quaternions, records)
        records += kept
    candidates = attach_merged_provenance(collect_candidates(records), merged)
    return candidates, records + merged


def only_from_extra_seeds(entries):
    """Candidates no independent mode found, per the recorded provenance."""
    return [e for e in entries
            if all(p['mode'] == EXTRA_SEED_MODE for p in e['provenance'])]


def candidate_key(record):
    """Identity of a configuration for deduplication.

    Uses the CANONICAL arm values plus the rail, so two lifts of one solution
    collapse to a single candidate. The lifted forms are collected against
    that key rather than counted as separate candidates.
    """
    canonical = np.round(np.asarray(record['q_arm_canonical']), DEDUP_DECIMALS)
    rail = round(float(record['rail_position']), DEDUP_DECIMALS)
    return (rail, *canonical.tolist())


def collect_candidates(records):
    """Accepted solves, deduplicated per waypoint, with provenance retained."""
    per_waypoint = {}
    for record in records:
        if not record['accepted']:
            continue
        waypoint = per_waypoint.setdefault(record['waypoint_index'], {})
        key = candidate_key(record)
        entry = waypoint.get(key)
        if entry is None:
            entry = waypoint[key] = {
                'q_arm_canonical': record['q_arm_canonical'],
                'rail_position': record['rail_position'],
                'lifted_forms': [],
                'windings': [],
                'provenance': [],
                'position_error_m': record['position_error_m'],
                'orientation_error_rad': record['orientation_error_rad'],
                'arm_condition_number': record['arm_condition_number'],
            }
        lifted = record['q_full']
        if lifted not in entry['lifted_forms']:
            entry['lifted_forms'].append(lifted)
            entry['windings'].append(record.get('winding'))
        entry['provenance'].append({
            'mode': record['mode'],
            'arm_seed_number': record.get('arm_seed_number'),
            'rail_seed_number': record.get('rail_seed_number'),
            'rail_seed': record.get('rail_seed'),
            'attempt': record['attempt'],
            'solver_iterations': record['solver_iterations'],
            'seed_origin': record.get('seed_origin'),
        })
    return {index: list(entries.values())
            for index, entries in per_waypoint.items()}


def summarise(candidates, records, num_waypoints):
    modes = sorted({r['mode'] for r in records})
    rows = []
    for index in range(num_waypoints):
        entries = candidates.get(index, [])
        row = {
            'waypoint_index': index,
            'distinct_candidates': len(entries),
            'total_lifted_forms': sum(len(e['lifted_forms']) for e in entries),
        }
        for mode in modes:
            contributing = {
                candidate_key({'q_arm_canonical': e['q_arm_canonical'],
                               'rail_position': e['rail_position']})
                for e in entries
                if any(p['mode'] == mode for p in e['provenance'])
            }
            row[f'{mode}:candidates'] = len(contributing)
        rows.append(row)
    return {'modes': modes, 'waypoints': rows}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    parser.add_argument('--csv', default=None,
                        help='recording to generate candidates for '
                             '(default: the packaged camera_traj.csv)')
    parser.add_argument('--waypoints', type=int, default=500,
                        help='RECORDED waypoints; a spin-up adds samples')
    parser.add_argument('--start-index', type=int, default=0,
                        help='first recorded sample of the slice')
    parser.add_argument('--spin-up-s', type=float, default=None,
                        help='prepend a spin-up so the task starts from rest')
    parser.add_argument('--out', default='candidates.json')
    args = parser.parse_args(argv)

    from ament_index_python.packages import get_package_share_directory
    mesh_path = get_package_share_directory('ur_description')
    targets, quaternions, dt, trajectory_metadata = load_trajectory(
        args.csv, args.waypoints, with_metadata=True, spin_up_s=args.spin_up_s,
        start_index=args.start_index)
    trajectory_metadata['placement'] = 'nominal'
    modes = ['arm_isolation', 'rail_isolation', 'interaction', CONTINUATION_MODE]

    validator = _validator(args.urdf, mesh_path)
    rail_seeds, arm_configs = tracking_reference_path(
        validator, targets, quaternions, dt, LEGACY_MATLAB_START_Q)
    arm_seeds = [np.deg2rad(seed) for seed in WIDE_SEED_BANK_DEG]

    records = run_arm_isolation(validator, targets, quaternions,
                                rail_seeds, arm_seeds)
    records += run_rail_isolation(validator, targets, quaternions, arm_configs)
    records += run_interaction(validator, targets, quaternions, arm_seeds)
    kept, merged = run_continuation(validator, targets, quaternions, records)
    records += kept
    candidates = attach_merged_provenance(collect_candidates(records), merged)
    document = {
        'schema_version': SCHEMA_VERSION,
        'environment': environment.describe(),
        'manifest': dict(
            manifest(args, validator, trajectory_metadata, modes, dt),
            rail_grid_m=list(RAIL_GRID_M),
            upstream_arm_seed_deg=list(UPSTREAM_ARM_SEED_DEG),
            dedup_decimals=DEDUP_DECIMALS,
            continuation_merge_tolerance=CONTINUATION_MERGE_TOLERANCE,
            continuation_merged_results=len(merged),
            rail_seed_source='corrected tracking path, per waypoint',
        ),
        'num_waypoints': len(targets),
        'summary': summarise(candidates, records, len(targets)),
        'candidates': {str(k): v for k, v in candidates.items()},
    }

    def plain(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.integer):
            return int(value)
        if isinstance(value, np.floating):
            return float(value)
        if isinstance(value, np.bool_):
            return bool(value)
        raise TypeError(f'unserialisable {type(value).__name__}')

    with open(args.out, 'w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=1, sort_keys=True, default=plain)

    counts = [row['distinct_candidates'] for row in document['summary']['waypoints']]
    print(f'wrote {args.out}: {len(records)} attempts, '
          f'{sum(counts)} distinct candidates')
    print(f'distinct candidates per waypoint: min={min(counts)} '
          f'median={int(np.median(counts))} max={max(counts)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
