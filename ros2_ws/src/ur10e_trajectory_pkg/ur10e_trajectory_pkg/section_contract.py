"""Shared input, evidence, and certificate contracts for section searches.

The fast first-certificate search and the resumable length search are separate
strategies. Neither should import the other's command module to interpret a
recording, probe, or certified plan. This module owns those common operations;
it does not schedule starts, lengths, or beam expansions.
"""
import hashlib
import json
import os
import subprocess

import numpy as np

from ur10e_trajectory_pkg import section_search as search
from ur10e_trajectory_pkg.section_search import (
    LOCATED_FAILURE_MARGIN_LAYERS,
    samples_through_layer,
)

LIMIT_MARGIN_BOUNDARY_RAD = 0.05
JOURNAL_SCHEMA_VERSION = 1
DEFAULT_MAX_PROBE_MEMORY_GB = 12.0
JOINTS = ('rail', 'shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3')
LIMITATIONS = (
    'Orientation only: q_I_G drives the motion and the end-effector position is held at '
    'the first sample; no translation, scaling or camera-relative motion.',
    'Nominal placement only; no placement or mount-alignment optimisation.',
    'T_EG, the mount transform, is a provisional identity until the hardware interface '
    'fixes it; every plan carries it.',
    'Arm acceleration is provisional and arm jerk and every rail limit are assumed '
    '(motion_limits); a certified section is planner-certified, not hardware-certified.',
    'The longest section is the longest this finite search certified under its budget; '
    'unprobed starts and lengths are listed, and nothing here shows that an uncertified '
    'section is infeasible.',
)


def load_json(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as handle:
        return json.load(handle)


def sha256_of(path):
    if not path:
        return None
    with open(path, 'rb') as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def git_revision():
    revision = os.environ.get('UR10E_GIT_REVISION')
    if revision:
        return revision
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        return subprocess.run(['git', '-C', here, 'rev-parse', 'HEAD'], capture_output=True,
                              text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def config_key(args, inputs):
    """Identity of a probe result for replay across fast and length modes."""
    material = {'csv_sha256': inputs['recording']['sha256'],
                'home_sha256': inputs['home']['sha256'],
                'via_sha256': (inputs['via_poses'] or {}).get('sha256'),
                'urdf': args.urdf, 'urdf_sha256': inputs['urdf_sha256'],
                'placement': 'nominal', 'spin_up_policy': args.spin_up_policy,
                'engine': getattr(args, 'engine', 'pipeline'),
                'code_revision': git_revision(),
                'max_probe_memory_gb': args.max_probe_memory_gb}
    return hashlib.sha256(json.dumps(material, sort_keys=True).encode()).hexdigest()[:16]


def read_inputs(args):
    """Validate the recording and identify every input before any probe."""
    from ur10e_trajectory_pkg import trajectory_input

    check, trajectory = trajectory_input.read_with_report(args.csv)
    recording = {'csv_path': args.csv, 'sha256': check.get('source_digest'),
                 'samples': check.get('samples'), 'duration_s': check.get('duration_s'),
                 'step_range_s': check.get('step_range_s'),
                 'columns_used': ['timestamp', 'q_I_G_x', 'q_I_G_y', 'q_I_G_z', 'q_I_G_w'],
                 'problems': check.get('problems', []), 'passed': bool(check.get('passed'))}
    timeline = None
    if recording['passed']:
        timeline = search.Timeline(trajectory.times_s)
        recording.update(first_timestamp_s=float(timeline.times[0]),
                         last_timestamp_s=float(timeline.times[-1]))
    home = load_json(args.home_json)
    via = load_json(args.via_poses) if args.via_poses else None
    inputs = {
        'recording': recording,
        'home': {'path': args.home_json, 'sha256': sha256_of(args.home_json),
                 'configuration': ((home or {}).get('chosen') or {}).get('configuration')},
        'via_poses': None if args.via_poses is None else {
            'path': args.via_poses, 'sha256': sha256_of(args.via_poses),
            'count': None if via is None else len(via)},
        'urdf': args.urdf,
        'urdf_sha256': sha256_of(args.urdf) if os.path.exists(args.urdf) else None,
        'placement': 'nominal',
    }
    return inputs, timeline


def load_hints(path):
    if not path:
        return None
    return {int(entry['start']): float(entry['score']) for entry in load_json(path)}


def first_rung_for(csv_path, count, start):
    """The derived spin-up ladder's first rung for this exact slice."""
    from ur10e_trajectory_pkg.target_builder import recorded_start_rate
    from ur10e_trajectory_pkg.pipeline import spin_up_ladder

    measured = recorded_start_rate(csv_path, count, start_index=start)
    return spin_up_ladder(measured['rate_rad_s'], measured['step_s'])[0]


def probe_name(start, samples):
    return f'start_{int(start):06d}_samples_{int(samples):06d}'


def limits_record(urdf):
    """Joint limits and their certification status, or why they are unavailable."""
    try:
        from ament_index_python.packages import get_package_share_directory

        from ur10e_trajectory_pkg import motion_limits
        from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

        validator = TrajectoryValidator(
            urdf, mesh_base_path=get_package_share_directory('ur_description'))
        lower, upper = (np.asarray(v, dtype=float) for v in validator.robot.qlim)
        record = {'statuses': motion_limits.limit_statuses(validator),
                  'may_certify_for_hardware': motion_limits.may_certify_for_hardware(
                      validator)}
        return record, lower, upper
    except Exception as exc:  # noqa: BLE001  the report says why instead
        return {'unavailable': f'{type(exc).__name__}: {exc}',
                'may_certify_for_hardware': False}, None, None


def joint_margins(path, lower, upper):
    """Each joint's closest approach to either limit along a path."""
    path = np.atleast_2d(np.asarray(path, dtype=float))
    to_lower = path.min(axis=0) - np.asarray(lower, dtype=float)
    to_upper = np.asarray(upper, dtype=float) - path.max(axis=0)
    return {name: {'to_lower': float(a), 'to_upper': float(b)}
            for name, a, b in zip(JOINTS, to_lower, to_upper)}


def limit_boundary(state, lower, upper, tolerance=LIMIT_MARGIN_BOUNDARY_RAD):
    """Joints of a path's last state within tolerance of a limit."""
    state = np.asarray(state, dtype=float)
    out = []
    for name, value, low, high in zip(JOINTS, state, lower, upper):
        if value - low <= tolerance:
            out.append(f'{name} at its lower limit ({value:.3f})')
        elif high - value <= tolerance:
            out.append(f'{name} at its upper limit ({value:.3f})')
    return out


def located_failure_time(commands):
    """Earliest located failure time in refinement or task validation."""
    commands = commands or {}
    times = []
    for attempt in (commands.get('refinement') or {}).get('attempts', []):
        if attempt.get('passed'):
            continue
        times += [entry['first_time_s'] for entry in
                  (attempt.get('position_limit_excursions') or {}).values()]
    task = commands.get('task_validation') or {}
    times += [entry['first_time_s'] for entry in
              (task.get('position_limit_excursions') or {}).values()]
    for key in ('self_clearance', 'tracking'):
        check = task.get(key) or {}
        failed = (check.get('passed') is False if key == 'self_clearance'
                  else check.get('within_tolerance') is False)
        if failed and check.get('at_time_s') is not None:
            times.append(check['at_time_s'])
    return min(times) if times else None


def graph_bound_samples(graph, via_offered):
    """Recorded samples a disconnected exhaustive graph covered, if sound."""
    if not graph or graph.get('complete_path'):
        return None
    if (graph.get('search') or {}).get('exhaustive') is False:
        return None
    home = (graph.get('candidate_filters') or {}).get('home_reachable')
    layer = graph.get('first_disconnected_layer')
    if home is None or layer is None or home.get('winding_aware') is not True:
        return None
    seeds = [attempt.get('seeds') for attempt in home.get('attempts') or []]
    if via_offered and 'direct+via' not in seeds:
        return None
    if int(layer) == 0:
        return 0
    added = int((graph.get('spin_up') or {}).get('samples_added', 0))
    return samples_through_layer(int(layer) - 1, added)


def rung_graphs(work_dir, summary):
    """The last graph of every spin-up rung the pipeline tried."""
    ladder = summary.get('spin_up_ladder') or {}
    if len(ladder.get('rungs_s') or []) <= 1:
        return [load_json(os.path.join(work_dir, 'graph.json'))]
    return [load_json(os.path.join(work_dir, f"spin_up_{entry['spin_up_s']:g}s", 'graph.json'))
            for entry in ladder.get('tried') or []]


def probe_evidence(work_dir, samples, step_s, via_offered, run):
    """What a finished probe's artifacts establish, in the policy's terms."""
    from ur10e_trajectory_pkg import probe_verdict

    summary = load_json(os.path.join(work_dir, 'pipeline.json')) or None
    if (summary or {}).get('failed_stage') == 'deadline':
        run = dict(run, resource_limited=True, killed_reason=summary.get('status'))
    if run.get('resource_limited'):
        return {'passed': False, 'classification': {
            'class': 'resource_limit', 'detail': run.get('killed_reason')},
            'graph_bound_samples': None, 'shrink_hint_samples': None}, summary, None, None, None
    paths = (summary or {}).get('artifacts') or {}
    graph, commands = load_json(paths.get('graph')), load_json(paths.get('commands'))
    plan_path = paths.get('task_plan')
    verdict = probe_verdict.classify({'passed': True}, summary, graph, commands)
    passed = verdict['class'] == 'pass' and bool(plan_path and os.path.exists(plan_path))
    if verdict['class'] == 'pass' and not passed:
        verdict = {'class': 'pipeline_error', 'detail': 'passed without writing a plan'}

    bound = None
    if not passed and summary and summary.get('failed_stage') == 'graph':
        bounds = [graph_bound_samples(g, via_offered) for g in rung_graphs(work_dir, summary)]
        if bounds and all(b is not None for b in bounds):
            bound = max(bounds)

    hints = []
    added = int(((graph or {}).get('spin_up') or {}).get('samples_added', 0))
    located = located_failure_time(commands)
    if not passed and located is not None:
        layer = int(np.floor(located / float(step_s) + 1e-9)) - LOCATED_FAILURE_MARGIN_LAYERS
        hints.append(samples_through_layer(max(layer, 0), added))
    if (not passed and bound is None and graph is not None and not graph.get('complete_path')
            and graph.get('last_connected_layer') is not None):
        hints.append(samples_through_layer(int(graph['last_connected_layer']), added))
    hints = [h for h in hints if h < samples]
    evidence = {'passed': passed, 'classification': verdict,
                'graph_bound_samples': bound,
                'shrink_hint_samples': min(hints) if hints else None,
                'located_failure_time_s': located}
    return evidence, summary, graph, commands, plan_path if passed else None


def route_record(graph):
    home = ((graph or {}).get('candidate_filters') or {}).get('home_reachable') or {}
    return {'routes': home.get('routes'), 'attempts': home.get('attempts'),
            'chosen_start': home.get('chosen_start')}


def section_record(timeline, probe, boundary, lower, upper):
    """Certified section interval, plan, margins, and evidence locations."""
    plan = load_json(probe.get('plan_path'))
    work_dir = probe['work_dir']
    commands = load_json(os.path.join(work_dir, 'commands.json')) or {}
    graph = load_json(os.path.join(work_dir, 'graph.json')) or {}
    task = commands.get('task_validation') or {}
    conditioning = task.get('conditioning') or {}
    path = (plan or {}).get('q_path')
    record = dict(timeline.interval(probe['start'], probe['samples']))
    record.update({
        'boundary': boundary,
        'classification': probe['classification'],
        'spin_up_s': probe.get('spin_up_s'),
        'spin_up': graph.get('spin_up'),
        'mount': (plan or {}).get('mount'),
        'placement': (plan or {}).get('placement'),
        'start_winding': commands.get('start_winding'),
        'start_route': probe.get('start_routes'),
        'warmup': commands.get('warmup'),
        'refinement': {key: (commands.get('refinement') or {}).get(key)
                       for key in ('status', 'level_m')},
        'margins': {
            'worst_condition': conditioning.get('max_condition_number'),
            'min_alpha_star': conditioning.get('min_alpha_star'),
            'min_self_clearance_m': (task.get('self_clearance') or {}).get('min_distance_m'),
            'velocity_budget_used': (task.get('velocity_headroom') or {}).get(
                'max_budget_used'),
            'joint_limits': (None if path is None or lower is None
                             else joint_margins(path, lower, upper)),
            'joints_near_a_limit_at_section_end': (
                None if path is None or lower is None
                else limit_boundary(path[-1], lower, upper)),
        },
        'evidence': {'probe': search.probe_key(probe), 'work_dir': work_dir,
                     'artifacts': {name: os.path.join(work_dir, f'{name}.json') for name in
                                   ('pipeline', 'candidates', 'graph', 'commands',
                                    'task_plan')},
                     'seconds': probe.get('seconds'),
                     'peak_memory_gb': probe.get('peak_memory_gb')},
    })
    return record, plan


def plan_problems(plan, probe, inputs):
    """Why an exported plan does not describe the reported section."""
    from ur10e_trajectory_pkg.target_builder import mount_record

    if plan is None:
        return ['no plan']
    problems = []
    if int(plan.get('start_index', -1)) != probe['start']:
        problems.append(f"plan start_index {plan.get('start_index')} is not {probe['start']}")
    if int(plan.get('recorded_waypoints', -1)) != probe['samples']:
        problems.append(f"plan covers {plan.get('recorded_waypoints')} recorded samples, "
                        f"not {probe['samples']}")
    if (plan.get('recording') or {}).get('csv_sha256') != inputs['recording']['sha256']:
        problems.append('plan was validated against other recording bytes')
    if plan.get('mount') != mount_record():
        problems.append('plan mount transform is not the declared T_EG')
    return problems
