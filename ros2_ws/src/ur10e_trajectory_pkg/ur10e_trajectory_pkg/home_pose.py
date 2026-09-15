#!/usr/bin/env python3
"""Choose the home pose once, under criteria fixed before any candidate is seen.

The task now starts from rest at a start the free-start planner chooses, which
differs per placement, and the arm reaches it by a rest-to-rest warmup. The
home is the one fixed configuration every warmup begins from.

HOME_CRITERIA:

  must pass, all of them
    static gates     clearance of ARM links to the environment >= 0.02 m,
                     joint-limit fraction >= 0.05, posture margin >= 0.02,
                     not in collision
    singularity      arm condition <= 50, the task gate, across a +/-5 deg
                     neighbourhood: each arm joint alone at +/-5 deg, plus 64
                     random +/-5 deg samples from a fixed seed. A random sample's
                     worst is a lower bound, so a chosen home above about 40
                     is confirmed with a dense +/-5 deg sample first
    coverage         across all 29 envelope placements of the spin-up
                     trajectory, a direct collision-free warmup to every IK
                     family: connectivity 1.0 and family fraction 1.0. If no
                     candidate reaches it, the best coverage is taken and the
                     shortfall reported

  ranked, among poses that pass
    1  worst condition number in the +/-5 deg neighbourhood, lowest first
    2  worst warmup duration, as a tie-break only

Continuous measures are floors, not ranking keys. Ranked one after another,
the first would almost never tie and the rest would never count.

Coverage against all 29 placements rather than the one nominal start: the
exact placement is unknown until setup, and the planner picks a different
start for each, so a home tuned to one start could have no safe warmup after
a small placement change. Covering every family means a direct warmup exists
whichever start the planner picks.

Usage, in order:
    python3 -m ur10e_trajectory_pkg.home_pose screen --pool sweep_v2.json \\
        --refinement refinement.json --out home_screen.json \\
        --poses-out home_screened_poses.json
    python3 -m ur10e_trajectory_pkg.ready_pose_runner --placements all \\
        --poses-json home_screened_poses.json --spin-up-s 2.0 --entry-at-rest \\
        --repeats 1 --no-exhaustive-check --out home_coverage.json
    python3 -m ur10e_trajectory_pkg.home_pose choose --screen home_screen.json \\
        --coverage home_coverage.json --out home_choice.json
    python3 -m ur10e_trajectory_pkg.home_pose commands --choice home_choice.json \\
        --graph graph_spinup_free_generator.json --out home_commands.json
"""
import argparse
import itertools
import json
import sys

import numpy as np

from ur10e_trajectory_pkg import ready_pose_sweep as sweep
from ur10e_trajectory_pkg.configurations import ARM_SLICE, PERIODIC_JOINTS, RAIL_INDEX
from ur10e_trajectory_pkg.joint_coordinates import TWO_PI, feasible_lifts

SCHEMA_VERSION = 1
REFINED_CANDIDATES = (70, 24, 67, 68)
HOME_CRITERIA = (
    'Fixed before any candidate was evaluated. Must pass: static gates with '
    'clearance on arm links only (>= 0.02 m), joint-limit fraction >= 0.05 and '
    'posture margin >= 0.02; '
    'arm condition <= 50 across +/-5 deg (each arm joint alone plus 64 random '
    'samples, fixed seed); coverage of all 29 spin-up placements with '
    'connectivity 1.0 and family fraction 1.0 (else best coverage, shortfall '
    'reported). Rank passing poses by worst neighbourhood condition number, '
    'then worst warmup duration as tie-break.')


def _key(configuration):
    return tuple(np.round(np.asarray(configuration, dtype=float), 9).tolist())


def candidate_homes(pool_results, refinement_outcomes,
                    refined_from=REFINED_CANDIDATES):
    """The V2 pool plus the named refined poses, each with its provenance."""
    out = [{'configuration': list(r['configuration']), 'provenance': r['provenance'],
            'anchor_name': r.get('anchor_name'), 'source': 'v2_pool',
            'pool_index': r['ready_index']} for r in pool_results]
    by_index = {o['ready_index']: o for o in refinement_outcomes}
    for index in refined_from:
        outcome = by_index[index]
        out.append({'configuration': outcome['history']['final_configuration'],
                    'provenance': f'refined_from_{index}', 'anchor_name': None,
                    'source': 'refinement', 'pool_index': index})
    return out


def screen(validator, candidates):
    """Static gates and singularity robustness for every candidate."""
    records = []
    for candidate in candidates:
        q = np.asarray(candidate['configuration'], dtype=float)
        gates = sweep.static_gates(validator, q)
        robustness = sweep.singularity_robustness(validator, q)
        records.append(dict(candidate, static_gates=gates,
                            singularity=robustness,
                            screen_passed=bool(gates['passed']
                                               and robustness['passed'])))
    return records


def combine(screen_records, coverage_results):
    """Join screening with coverage, by configuration."""
    coverage = {_key(r['configuration']): r['summary'] for r in coverage_results}
    out = []
    for record in screen_records:
        summary = coverage.get(_key(record['configuration']))
        full = bool(summary is not None
                    and summary.get('connectivity') == 1.0
                    and summary.get('worst_family_fraction') == 1.0)
        out.append(dict(record, coverage=summary, full_coverage=full,
                        passed=bool(record['screen_passed'] and full)))
    return out


def choose_home(records):
    """Apply HOME_CRITERIA to combined records."""
    def condition(record):
        return record['singularity']['worst_condition']

    def duration(record):
        summary = record['coverage'] or {}
        value = summary.get('worst_duration_s')
        return np.inf if value is None else value

    passing = sorted((r for r in records if r['passed']),
                     key=lambda r: (condition(r), duration(r)))
    if passing:
        return {'criteria': HOME_CRITERIA, 'full_coverage_found': True,
                'chosen': passing[0], 'ranking': passing, 'shortfall': None}

    covered = [r for r in records if r['screen_passed'] and r['coverage']]
    if not covered:
        return {'criteria': HOME_CRITERIA, 'full_coverage_found': False,
                'chosen': None, 'ranking': [],
                'shortfall': 'no candidate passed the static gates and the '
                             'singularity requirement'}

    def coverage_key(record):
        summary = record['coverage']
        return (-(summary.get('connectivity') or 0.0),
                -(summary.get('worst_family_fraction') or 0.0),
                condition(record), duration(record))

    ranked = sorted(covered, key=coverage_key)
    best = ranked[0]['coverage']
    return {'criteria': HOME_CRITERIA, 'full_coverage_found': False,
            'chosen': ranked[0], 'ranking': ranked,
            'shortfall': {'connectivity': best.get('connectivity'),
                          'worst_family_fraction': best.get('worst_family_fraction')}}


def start_windings(validator, path):
    """Every legal winding of the path's start, with the whole path shifted along.

    The planner chose a canonical start; any 2*pi*k shift of the periodic arm
    joints is the same task, provided the whole path stays inside the limits.
    """
    path = np.asarray(path, dtype=float)
    lower, upper = validator.robot.qlim
    start = path[0]
    options = [feasible_lifts(value, lower[ARM_SLICE][i], upper[ARM_SLICE][i],
                              PERIODIC_JOINTS[ARM_SLICE][i])
               for i, value in enumerate(start[ARM_SLICE])]
    out = []
    for arm in itertools.product(*options):
        shift = np.zeros(path.shape[1])
        shift[ARM_SLICE] = np.asarray(arm) - start[ARM_SLICE]
        shifted = path + shift
        if np.all(shifted >= lower - 1e-9) and np.all(shifted <= upper + 1e-9):
            out.append({'winding': np.rint(shift[ARM_SLICE] / TWO_PI).astype(int).tolist(),
                        'path': shifted})
    return out


def choose_start_winding(validator, home, path):
    """The winding keeping the path in limits with a valid warmup, shortest first."""
    from ur10e_trajectory_pkg import warmup

    options = []
    for option in start_windings(validator, path):
        result = warmup.plan_warmup(validator, home, option['path'][0])
        options.append(dict(option, warmup=result))
    valid = [o for o in options if o['warmup']['status'] == warmup.OK]
    summary = [{'winding': o['winding'], 'status': o['warmup']['status'],
                'duration_s': o['warmup'].get('duration_s')} for o in options]
    if not valid:
        return None, summary
    best = min(valid, key=lambda o: (o['warmup']['duration_s'],
                                     sum(abs(w) for w in o['winding']),
                                     tuple(o['winding'])))
    return best, summary


def _load(path):
    with open(path, encoding='utf-8') as handle:
        return json.load(handle)


def _dump(document, path):
    def plain(value):
        if isinstance(value, np.bool_):
            return bool(value)
        if isinstance(value, np.integer):
            return int(value)
        if isinstance(value, np.floating):
            return float(value)
        if isinstance(value, np.ndarray):
            return value.tolist()
        raise TypeError(type(value).__name__)
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=1, default=plain)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    s = commands.add_parser('screen')
    s.add_argument('--pool', required=True)
    s.add_argument('--refinement', required=True)
    s.add_argument('--out', required=True)
    s.add_argument('--poses-out', required=True)
    c = commands.add_parser('choose')
    c.add_argument('--screen', required=True)
    c.add_argument('--coverage', required=True)
    c.add_argument('--out', required=True)
    m = commands.add_parser('commands')
    m.add_argument('--choice', required=True)
    m.add_argument('--graph', required=True)
    m.add_argument('--rate', type=float, default=200.0)
    m.add_argument('--out', required=True)
    for p in (s, c, m):
        p.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    args = parser.parse_args(argv)

    if args.command == 'choose':
        combined = combine(_load(args.screen)['records'],
                           _load(args.coverage)['results'])
        decision = choose_home(combined)
        _dump(dict(decision, schema_version=SCHEMA_VERSION), args.out)
        chosen = decision['chosen']
        print('full coverage found:', decision['full_coverage_found'],
              '| passing:', len(decision['ranking']) if decision['full_coverage_found'] else 0,
              '| shortfall:', decision['shortfall'])
        if chosen:
            print('chosen:', chosen['provenance'], chosen.get('anchor_name'),
                  np.round(chosen['configuration'], 4).tolist(),
                  '| worst +/-5 deg condition',
                  round(chosen['singularity']['worst_condition'], 2),
                  '| worst warmup', chosen['coverage'].get('worst_duration_s'))
        return 0

    from ament_index_python.packages import get_package_share_directory

    from ur10e_trajectory_pkg.failure_census import _validator

    validator = _validator(args.urdf, get_package_share_directory('ur_description'))

    if args.command == 'screen':
        candidates = candidate_homes(_load(args.pool)['results'],
                                     _load(args.refinement)['outcomes'])
        records = screen(validator, candidates)
        passing = [r for r in records if r['screen_passed']]
        _dump({'schema_version': SCHEMA_VERSION, 'criteria': HOME_CRITERIA,
               'candidates': len(records), 'screen_passed': len(passing),
               'records': records}, args.out)
        _dump([{'configuration': r['configuration'], 'provenance': r['provenance'],
                'anchor_name': r['anchor_name']} for r in passing], args.poses_out)
        print(f'{len(passing)} of {len(records)} candidates pass the static gates '
              f'and the singularity requirement')
        return 0 if passing else 1

    # commands: winding, warmup, and both validations for the chosen home
    from ur10e_trajectory_pkg import continuous_validator
    from ur10e_trajectory_pkg.failure_census import load_trajectory

    choice = _load(args.choice)
    graph = _load(args.graph)
    home = np.asarray(choice['chosen']['configuration'], dtype=float)
    best, summary = choose_start_winding(validator, home, graph['path'])
    document = {'schema_version': SCHEMA_VERSION, 'home': home.tolist(),
                'graph': args.graph, 'winding_options': summary}
    if best is None:
        document['status'] = 'no valid warmup to any legal winding of the start'
        _dump(document, args.out)
        print(document['status'])
        return 1
    spin_up = graph.get('spin_up')
    targets, quaternions, dt, _ = load_trajectory(
        None, graph['recorded_waypoints'], with_metadata=True,
        spin_up_s=None if spin_up is None else spin_up['requested_duration_s'])
    warmup_report = continuous_validator.validate_warmup(validator, best['warmup'])
    task_report = continuous_validator.validate_task_command(
        validator, best['path'], targets, quaternions, dt, rate_hz=args.rate)
    document.update(
        status='ok', start_winding=best['winding'],
        command_2_start=best['path'][0].tolist(),
        warmup={k: v for k, v in best['warmup'].items()
                if k not in ('times', 'positions')},
        warmup_validation=warmup_report, task_validation=task_report,
        both_commands_pass=bool(warmup_report['passed'] and task_report['passed']))
    _dump(document, args.out)
    print('start winding', best['winding'], '| warmup', round(best['warmup']['duration_s'], 3),
          's bound by', best['warmup']['binding'], '| warmup validation passed',
          warmup_report['passed'], '| task validation passed', task_report['passed'])
    return 0 if document['both_commands_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
