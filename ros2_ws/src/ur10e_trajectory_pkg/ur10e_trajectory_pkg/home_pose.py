#!/usr/bin/env python3
"""Validate a configured home, connect it to the task, and export joint commands.

The home is a robot setup input, not a per-recording pose search. Every run
rechecks its static gates and confirms the graph starts from that home.
"""

import argparse
import inspect
import itertools
import json
import os
import sys

import numpy as np

from ur10e_trajectory_pkg import motion_limits, plan_artifact, robot_checks as checks
from ur10e_trajectory_pkg.configurations import ARM_SLICE, PERIODIC_JOINTS
from ur10e_trajectory_pkg.joint_coordinates import TWO_PI, feasible_lifts

SCHEMA_VERSION = 1
HOME_CRITERIA = (
    'Configured home must pass environment clearance >= 0.02 m, '
    'self-clearance >= 0.01 m, joint-limit fraction >= 0.05, '
    'posture margin >= 0.02, no collision, and arm condition <= 50 '
    'through a +/-5 deg neighbourhood. Every chosen task start must have '
    'a validated warmup from this home.'
)


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


def choose_start_winding(validator, home, path, via_poses=None):
    """The winding keeping the path in limits with a VALIDATED warmup, shortest first.

    The warmup is a route: straight when the direct move plans, through a
    resting via pose when it does not and via_poses are offered. Windings are
    ranked on the route's total duration, so a winding reachable only by a
    detour loses to one reachable straight, which is what it should do.

    Planned routes are validated in that order and the first that passes is
    kept. Ranking planned routes and validating only the winner let one route
    that planned but failed validation sink the run while windings further
    down went untried. The winner carries its report as warmup_validation.
    """
    from ur10e_trajectory_pkg import continuous_validator, warmup

    options = []
    for option in start_windings(validator, path):
        result = warmup.plan_route(validator, home, option['path'][0],
                                   via_poses=via_poses)
        options.append(dict(option, warmup=result, validation=None))
    planned = sorted((o for o in options if o['warmup']['status'] == warmup.OK),
                     key=lambda o: (o['warmup']['total_duration_s'],
                                    sum(abs(w) for w in o['winding']),
                                    tuple(o['winding'])))
    best = None
    for option in planned:
        option['validation'] = continuous_validator.validate_route(validator,
                                                                   option['warmup'])
        if option['validation']['passed']:
            best = dict(option, warmup_validation=option['validation'])
            break
    summary = [{'winding': o['winding'], 'status': o['warmup']['status'],
                'duration_s': o['warmup'].get('total_duration_s'),
                'route_kind': o['warmup'].get('route_kind'),
                'segments': len(o['warmup'].get('segments', [])),
                'validated': (None if o['validation'] is None
                              else bool(o['validation']['passed'])),
                'validation_failures': (None if o['validation'] is None
                                        else o['validation']['failures'])}
               for o in options]
    return best, summary


# --------------------------------------------------------------------------
# What the command checks before it plans
# --------------------------------------------------------------------------
#
# Start-reachability is structural: a graph built with --home-json cannot
# produce a start the home cannot reach, because the filter removed every
# layer-0 candidate it could not warm up to. Re-deriving that here would
# prove nothing a single-trajectory run depends on. Two
# things the filter does not cover are worth the second they cost, before the
# expensive refinement runs.
#
#   the home itself   home_choice.json is a file, and a file can be stale,
#                     hand-edited, or written against a different URDF. The
#                     criteria are properties of the pose alone -- no
#                     placements, no planning -- so they are cheap to redo.
#   this placement    the filter's own result, read back from the graph: how
#                     many layer-0 candidates it kept and by which route, and
#                     a failure that distinguishes "no start the home can
#                     reach here" from "no candidates here". That distinction
#                     is what says whether to re-home or to look at generation.
#
# Neither replaces the end-to-end gate. choose_start_winding and the two
# validations below prove the route for the path actually being run, which no
# pre-check can do.

HOME_RECHECK_FAILED = 'the chosen home no longer passes its own criteria'
HOME_MISMATCH = 'the graph was filtered against a different home'
NO_REACHABLE_START = 'no start the home can reach at this placement'
NO_CANDIDATES_HERE = 'no candidates at this placement'

# Measured field -> the static_gates parameter that gates it, and how to say
# it. The threshold is READ from that signature rather than restated here, so
# a changed gate cannot leave the explanation quoting a stale number.
GATE_THRESHOLDS = {
    'collision_distance_m': ('min_clearance_m', 'environment clearance', ' m'),
    'joint_limit_clearance': ('min_limit_fraction', 'joint-limit fraction', ''),
    'posture_margin': ('min_posture_margin', 'posture margin', ''),
    'self_clearance_m': ('min_self_clearance_m', 'self-clearance', ' m'),
}

# Recorded measurements compared against recomputed ones. Agreement is what
# makes the artifact evidence: a home whose recorded clearance no longer
# reproduces is not the pose that was screened, whatever its stored 'passed'
# says. A field the record does not carry is skipped rather than assumed --
# the self-clearance floor was added after this home was chosen, so the
# record in hand predates the field.
RECHECKED_GATES = ('collision_distance_m', 'joint_limit_clearance',
                   'posture_margin', 'self_clearance_m')
RECHECKED_SINGULARITY = ('nominal_condition', 'worst_condition')

# These are geometric measurements of one configuration, recomputed from the
# same URDF and environment, so they agree to the solver's repeatability. A
# looser tolerance would let a real change -- a different URDF, an edited
# configuration -- pass as rounding.
RECHECK_TOLERANCE = 1e-6


def _gate_thresholds():
    """The numbers static_gates actually applies, read from its signature."""
    parameters = inspect.signature(checks.static_gates).parameters
    values = {field: parameters[name].default
              for field, (name, _, _) in GATE_THRESHOLDS.items()}
    if values['self_clearance_m'] is None:
        values['self_clearance_m'] = motion_limits.SELF_CLEARANCE_FLOOR_M
    return values


def _screen_failures(gates, robustness):
    """Which criterion the pose fails, and by how much."""
    failures = []
    if gates['in_collision']:
        failures.append('the pose is in collision')
    for field, threshold in _gate_thresholds().items():
        measured = gates.get(field)
        if measured is not None and measured < threshold:
            _, label, unit = GATE_THRESHOLDS[field]
            failures.append(f'{label} {measured:.4f}{unit} below the '
                            f'{threshold:.4f}{unit} floor')
    if not robustness['passed']:
        failures.append(f"arm condition {robustness['worst_condition']:.2f} "
                        f"over the {robustness['gate']:.1f} gate across "
                        f"+/-{robustness['neighbourhood_deg']:.0f} deg")
    return failures


def _recorded_drift(record, gates, robustness):
    """Recorded measurements that no longer reproduce."""
    drift = []
    for source, fields, recomputed in (
            ('static_gates', RECHECKED_GATES, gates),
            ('singularity', RECHECKED_SINGULARITY, robustness)):
        stored = record.get(source) or {}
        for field in fields:
            if field not in stored:
                continue
            was, now = float(stored[field]), float(recomputed[field])
            if abs(was - now) > RECHECK_TOLERANCE:
                drift.append(f'recorded {source}.{field} {was:.6f} but '
                             f'recomputed {now:.6f}')
    return drift


def recheck_home(validator, choice, workspace=None):
    """Re-screen the chosen home against its own criteria, the pose alone.

    Static gates (the self-clearance floor included) and the +/-5 deg
    condition test, recomputed, plus a comparison with what the artifact
    recorded. No placements and no planning: this is what catches a stale or
    hand-edited home_choice.json, and it costs a second.
    """
    from ur10e_trajectory_pkg.planning_runtime import repository_revision

    record = choice.get('chosen') or {}
    configuration = np.asarray(record['configuration'], dtype=float)
    gates = checks.static_gates(validator, configuration)
    robustness = checks.singularity_robustness(validator, configuration)
    failures = _screen_failures(gates, robustness)
    drift = _recorded_drift(record, gates, robustness)
    if workspace is None:
        workspace = os.path.dirname(os.path.abspath(__file__))
    return {'configuration': configuration.tolist(), 'criteria': HOME_CRITERIA,
            'static_gates': gates, 'singularity': robustness,
            'failures': failures, 'recorded_drift': drift,
            'passed': not failures and not drift,
            'repository_revision': repository_revision(workspace)}


def placement_reachability(graph, home):
    """What the graph's own home filter says about the placement being planned.

    The filter already did this work while the graph was built, so reading its
    result says exactly what a re-derivation would and costs nothing. kept == 0
    and layer_0_candidates == 0 are different failures: the first says re-home
    or widen the via pool, the second says look at candidate generation.
    """
    record = (graph.get('candidate_filters') or {}).get('home_reachable')
    if record is None:
        return {'status': 'unfiltered', 'passed': True, 'filter': None,
                'note': 'the graph was not built with --home-json, so the '
                        'start was never restricted to what the home reaches'}
    out = {'status': 'ok', 'passed': True, 'filter': record,
           'candidates': record.get('layer_0_candidates'),
           'kept': record.get('kept'), 'routes': record.get('routes'),
           'winding_aware': record.get('winding_aware'),
           'via_poses_offered': record.get('via_poses_offered')}
    filtered = record.get('home')
    if filtered is not None and not np.allclose(
            np.asarray(filtered, dtype=float), home, atol=1e-9):
        return dict(out, status=HOME_MISMATCH, passed=False)
    if not out['candidates']:
        return dict(out, status=NO_CANDIDATES_HERE, passed=False)
    if not out['kept']:
        return dict(out, status=NO_REACHABLE_START, passed=False,
                    blocked_reasons=record.get('reasons'))
    return out


def conditioning_cost(graph):
    """This path's worst condition against the best any path could manage.

    The edge cost counts step size only, so the cheapest path drifts up to
    whatever the candidate cap allows, and the home constraint can remove a
    better-conditioned branch on top of that. Neither is visible from a pass,
    and the cost can be large and silent. The bound is a
    lower bound over UNFILTERED candidates and does not prove a path achieving
    it is connected, so this is a figure to read, not a gate.
    """
    bound = ((graph.get('candidate_filters') or {})
             .get('best_condition_lower_bound') or {})
    achieved, value = graph.get('path_max_condition'), bound.get('value')
    return {'achieved_worst_condition': achieved,
            'achievable_bottleneck': value,
            'bottleneck_layer': bound.get('layer'),
            'excess': (None if achieved is None or value is None
                       else float(achieved) - float(value)),
            'ratio': (None if not value or achieved is None
                      else float(achieved) / float(value))}


def home_gate(validator, choice, graph, workspace=None):
    """Recheck the configured home and the graph's reachable starts."""
    home = np.asarray(choice['chosen']['configuration'], dtype=float)
    recheck = recheck_home(validator, choice, workspace=workspace)
    reachability = placement_reachability(graph, home)
    record = {'home_recheck': recheck, 'placement_reachability': reachability,
              'conditioning': conditioning_cost(graph)}
    if not recheck['passed']:
        record['status'] = HOME_RECHECK_FAILED
    elif not reachability['passed']:
        record['status'] = reachability['status']
    else:
        record['status'] = 'ok'
    record['passed'] = record['status'] == 'ok'
    return record


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
    m = commands.add_parser('commands')
    m.add_argument('--choice', required=True)
    m.add_argument('--graph', required=True)
    m.add_argument('--rate', type=float, default=200.0)
    m.add_argument('--out', required=True)
    m.add_argument('--plan-out', default=None,
                   help='write the joint plan only if warmup and task pass')
    m.add_argument('--via-poses', default=None,
                   help='vetted resting poses for a blocked direct warmup')
    m.add_argument('--csv', default=None,
                   help='recording the graph was planned for')
    m.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    args = parser.parse_args(argv)

    from ament_index_python.packages import get_package_share_directory
    from ur10e_trajectory_pkg.planning_runtime import make_validator as _validator

    validator = _validator(args.urdf, get_package_share_directory('ur_description'))

    # commands: winding, warmup, and both validations for the chosen home
    from ur10e_trajectory_pkg import continuous_validator
    from ur10e_trajectory_pkg.planning_runtime import load_trajectory

    choice = _load(args.choice)
    graph = _load(args.graph)
    home = np.asarray(choice['chosen']['configuration'], dtype=float)

    # An incomplete graph has no path to refine. Refused by name here too, so
    # a caller other than the pipeline cannot reach refinement with None.
    if not graph.get('complete_path') or graph.get('path') is None:
        status = ('the graph has no complete path: first disconnected layer '
                  f"{graph.get('first_disconnected_layer')}")
        _dump({'schema_version': SCHEMA_VERSION, 'graph': args.graph,
               'status': status}, args.out)
        print(status)
        return 1

    from ur10e_trajectory_pkg.planning_runtime import (
        recording_mismatch,
        recording_record,
    )
    recording = recording_record(args.csv)
    mismatch = recording_mismatch(graph.get('recording'), recording)
    if mismatch:
        status = f'the graph was {mismatch}'
        _dump({'schema_version': SCHEMA_VERSION, 'graph': args.graph,
               'status': status, 'recording': recording}, args.out)
        print(status)
        return 1

    # Before the expensive refinement: the home against its own criteria, and
    # the filter's own verdict on this placement. Report and stop, never
    # substitute -- re-homing stays a deliberate act.
    gate = home_gate(validator, choice, graph)
    if not gate['passed']:
        _dump({'schema_version': SCHEMA_VERSION, 'home': home.tolist(),
               'graph': args.graph,
               'placement': graph.get('placement', 'nominal'),
               'status': gate['status'], 'home_gate': gate}, args.out)
        print(gate['status'])
        for line in (gate['home_recheck']['failures']
                     + gate['home_recheck']['recorded_drift']):
            print('  ', line)
        blocked = gate['placement_reachability'].get('blocked_reasons')
        if blocked:
            print('   blocked:', blocked)
        return 1

    from ur10e_trajectory_pkg import path_refinement

    # graph -> refinement -> start winding and warmup -> both validations.
    # Targets first, since refinement solves the arm against them.
    spin_up = graph.get('spin_up')
    placement = graph.get('placement', 'nominal')
    if placement != 'nominal':
        raise ValueError('only the fixed nominal target placement is supported')
    start_index = int(graph.get('start_index', 0))
    targets, quaternions, dt, metadata = load_trajectory(
        args.csv, graph['recorded_waypoints'], with_metadata=True,
        spin_up_s=None if spin_up is None else spin_up['requested_duration_s'],
        start_index=start_index)
    refinement = path_refinement.refine_path(validator, graph['path'], targets,
                                             quaternions, dt, rate_hz=args.rate)
    via_poses = None
    if args.via_poses is not None:
        via_poses = [entry['configuration'] if isinstance(entry, dict) else entry
                     for entry in _load(args.via_poses)]
    best, summary = choose_start_winding(validator, home, refinement['path'],
                                         via_poses=via_poses)
    document = {'schema_version': SCHEMA_VERSION, 'home': home.tolist(),
                'graph': args.graph, 'placement': placement,
                'home_gate': gate, 'recording': recording,
                'refinement': {k: refinement[k] for k in ('status', 'level_m', 'attempts')},
                'refinement_meets_acceptance': path_refinement.meets_acceptance(refinement),
                'winding_options': summary}
    if best is None:
        document['status'] = 'no valid warmup to any legal winding of the start'
        _dump(document, args.out)
        print(document['status'])
        return 1
    # Reject a graph whose saved placement transform differs from the targets.
    if 'placement_RG' in graph and not np.allclose(
            graph['placement_RG'], metadata['placement_RG'], atol=1e-9):
        document['status'] = f'targets do not match the graph placement {placement!r}'
        _dump(document, args.out)
        print(document['status'])
        return 1
    # Already validated when the winding was chosen; the same report, not a
    # second pass over the same route.
    warmup_report = (best.get('warmup_validation')
                     or continuous_validator.validate_route(validator, best['warmup']))
    task_report = continuous_validator.validate_task_command(
        validator, best['path'], targets, quaternions, dt, rate_hz=args.rate)
    document.update(
        status='ok', start_winding=best['winding'],
        command_2_start=best['path'][0].tolist(),
        warmup=plan_artifact.route_record(best['warmup']),
        warmup_validation=warmup_report, task_validation=task_report,
        both_commands_pass=bool(warmup_report['passed'] and task_report['passed']))
    _dump(document, args.out)
    if args.plan_out and document['both_commands_pass']:
        _dump(plan_artifact.task_plan(home, best['path'], graph, args.graph, best['winding'],
                        route=best['warmup'], recording=recording),
              args.plan_out)
    print('start winding', best['winding'], '| warmup',
          round(best['warmup']['total_duration_s'], 3), 's over',
          len(best['warmup']['segments']), 'segment(s)',
          f"({best['warmup'].get('route_kind')})", '| warmup validation passed',
          warmup_report['passed'], '| task validation passed', task_report['passed'])
    conditioning = gate['conditioning']
    if conditioning['achieved_worst_condition'] is not None:
        bottleneck = conditioning['achievable_bottleneck']
        print('condition: achieved',
              round(conditioning['achieved_worst_condition'], 2),
              '| achievable bottleneck',
              'unknown' if bottleneck is None else round(bottleneck, 2),
              'at layer', conditioning['bottleneck_layer'])
    return 0 if document['both_commands_pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
