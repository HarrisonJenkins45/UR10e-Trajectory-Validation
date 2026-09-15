#!/usr/bin/env python3
"""Candidate configurations per waypoint, and a controlled seed experiment.

Corrected tracking already reaches every waypoint, so this is not a
reachability test. It measures candidate DIVERSITY and the robustness of
discovery, and produces the node set a layered graph will consume.

Three arms, run as a factorial rather than as another fresh-versus-wide
headline. Every arm uses the rail seed tracking actually used at that
waypoint, so the rail is held fixed where it is not the variable:

  arm_isolation    every arm seed, at the tracking-derived rail seed
  rail_isolation   a declared coarse rail grid, at one fixed arm seed, run
                   once from the preceding tracking arm configuration and
                   once from the upstream default posture
  interaction      the Cartesian product, which is the real generator

Reading a difference:

  same rail seed, different arm result        arm-seed discovery effect
  same arm seed, different rail result        rail-initialisation effect
  found only by one combination               interaction effect
  same canonical values, different winding    coordinate representation,
                                              NOT a new kinematic branch
  different canonical values                  genuine configuration diversity

That last distinction is why both forms are recorded. Two lifts of one
solution are one branch wearing two coordinates, and counting them as two
would inflate every diversity measure.

Velocity is deliberately not checked here. It is a property of a transition
between nodes, so it belongs to graph edges; enforcing it during node
generation would discard candidates that are perfectly good successors of
some other predecessor.

Usage:
    python3 -m ur10e_trajectory_pkg.candidate_generator --out candidates.json
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
from ur10e_trajectory_pkg.failure_census import (
    WIDE_SEED_BANK_DEG,
    _validator,
    file_digest,
    load_trajectory,
    manifest,
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


def run_continuation(validator, targets, quaternions, base_records):
    """Seed each waypoint from every candidate found at the previous one.

    Every other mode solves each waypoint independently, so a branch the
    graph is following can go unfound at a single waypoint and disconnect the
    graph. Measured at coupled_14: the only reachable branch had candidates at
    355 and 357 but none at 356, and re-solving 356 from the branch returned
    valid solutions every time. Seeding from the previous waypoint's
    candidates re-solves every branch found there, so none can vanish for one
    step.

    Waypoints run in order and the seeds include what continuation itself
    found, so a branch is carried forward as far as it stays solvable.
    """
    by_waypoint = {}
    for record in base_records:
        by_waypoint.setdefault(record['waypoint_index'], []).append(record)
    out = []
    previous = collect_candidates(by_waypoint.get(0, [])).get(0, [])
    for index in range(1, len(targets)):
        records = []
        for number, entry in enumerate(previous):
            records += solve_one(
                validator, targets[index], quaternions[index],
                np.asarray(entry['q_arm_canonical'], dtype=float),
                entry['rail_position'],
                dict(mode=CONTINUATION_MODE, waypoint_index=index,
                     entry_kind='continuation', arm_seed_number=number,
                     rail_seed_number=None,
                     seed_origin=f'waypoint{index - 1}:candidate{number}'))
        out += records
        previous = collect_candidates(
            by_waypoint.get(index, []) + records).get(index, [])
    return out


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
    if continuation:
        records += run_continuation(validator, targets, quaternions, records)
    return collect_candidates(records), records


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
    parser.add_argument('--waypoints', type=int, default=500,
                        help='RECORDED waypoints; a spin-up adds samples')
    parser.add_argument('--spin-up-s', type=float, default=None,
                        help='prepend a spin-up so the task starts from rest')
    parser.add_argument('--placement', default='nominal',
                        help='envelope placement to generate candidates at')
    parser.add_argument('--no-continuation', action='store_true',
                        help='skip seeding each waypoint from the previous '
                             "waypoint's candidates (for comparison only)")
    parser.add_argument('--out', default='candidates.json')
    parser.add_argument('--modes',
                        default='arm_isolation,rail_isolation,interaction')
    args = parser.parse_args(argv)

    from ament_index_python.packages import get_package_share_directory
    mesh_path = get_package_share_directory('ur_description')
    from ur10e_trajectory_pkg.failure_census import placement_RG_for
    placement_RG = (None if args.placement == 'nominal'
                    else placement_RG_for(args.placement))
    targets, quaternions, dt, trajectory_metadata = load_trajectory(
        None, args.waypoints, with_metadata=True, spin_up_s=args.spin_up_s,
        placement_RG=placement_RG)
    trajectory_metadata['placement'] = args.placement
    modes = [m.strip() for m in args.modes.split(',') if m.strip()]

    validator = _validator(args.urdf, mesh_path)
    rail_seeds, arm_configs = tracking_reference_path(
        validator, targets, quaternions, dt, LEGACY_MATLAB_START_Q)
    arm_seeds = [np.deg2rad(seed) for seed in WIDE_SEED_BANK_DEG]

    records = []
    if 'arm_isolation' in modes:
        records += run_arm_isolation(validator, targets, quaternions,
                                     rail_seeds, arm_seeds)
    if 'rail_isolation' in modes:
        records += run_rail_isolation(validator, targets, quaternions,
                                      arm_configs)
    if 'interaction' in modes:
        records += run_interaction(validator, targets, quaternions, arm_seeds)

    if not args.no_continuation:
        records += run_continuation(validator, targets, quaternions, records)
        modes = modes + [CONTINUATION_MODE]
    candidates = collect_candidates(records)
    document = {
        'schema_version': SCHEMA_VERSION,
        'environment': environment.describe(),
        'manifest': dict(
            manifest(args, validator, trajectory_metadata, modes, dt),
            rail_grid_m=list(RAIL_GRID_M),
            upstream_arm_seed_deg=list(UPSTREAM_ARM_SEED_DEG),
            dedup_decimals=DEDUP_DECIMALS,
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
