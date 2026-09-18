#!/usr/bin/env python3
"""Multi-trajectory benchmark: every recording through the same whole pipeline.

An all-or-nothing generalisation study. Each recording runs candidates ->
graph -> commands under one fixed protocol (PROTOCOL), and every outcome is
classified BEFORE anyone considers dividing trajectories into sections: a
warmup failure, a candidate-generation defect or a poor placement should be
fixed directly, and slicing the trajectory would hide it.

Three identities per recording
------------------------------
  source_digest   sha256 of the CSV bytes. Provenance only.
  motion_digest   timestamps relative to the first sample, and rotations
                  R_IG(0)^T R_IG(t). The same physical tumble, whatever the
                  camera, the translation, the quaternion sign or the initial
                  attitude.
  task_digest     timestamps, target positions and target rotations as
                  build_trajectory_targets produces them, after placement and
                  spin-up. The IK case itself: trials are de-duplicated on it.

The targets include the spin-up, which the pipeline derives per run, so
task_digest is taken at the FIRST rung of the derived ladder -- the same
definition for every recording -- and the rung each run actually tried is
recorded beside it. Two recordings sharing a motion but not a task are
labelled same_motion_different_task; that is expected when the initial
attitude or the placement differs.

Rotations are hashed as matrices, so q and -q are the same by construction.
Values are rounded to DIGEST_*_DECIMALS first, and -0.0 is folded into 0.0.

Failure classes
---------------
  input                schema, timestamps or quaternion data
  candidate_discovery  a layer with no candidate left after the filters
  graph_connectivity   candidates exist, but no legal transition reaches them
  warmup_home          a path exists but the selected home cannot approach it
  refinement           a discrete path exists but could not be smoothed
  continuous_limit     limits, collision, clearance, tracking, conditioning or
                       twist fail between waypoints
  pipeline_error       a stage refused or crashed for another reason

placement is a class too, but one run at the nominal placement cannot assign
it: that needs the same recording re-run elsewhere. Every record therefore
says placement_sensitivity 'untested' rather than guessing.

Usage:
    python3 -m ur10e_trajectory_pkg.benchmark inspect --entries corpus.json \\
        --waypoints 500 --out identities.json
    python3 -m ur10e_trajectory_pkg.benchmark case --case-id c1-anchor-s1 \\
        --csv camera_traj.csv --waypoints 500 --home-json home_choice.json \\
        --via-poses poses.json --work-dir runs/c1-anchor-s1
    python3 -m ur10e_trajectory_pkg.benchmark report --identities identities.json \\
        --cases runs --out report.json --markdown report.md
"""
import argparse
import glob
import hashlib
import json
import os
import resource
import sys
import time

import numpy as np

SCHEMA_VERSION = 1

PROTOCOL = {
    'motion': 'q_I_G only; end-effector position fixed; no translation or scaling',
    'placement': 'nominal',
    'home': 'the selected home_choice.json, unchanged',
    'timing': 'original sample timing',
    'spin_up': 'derived from the recording (pipeline ladder, shortest first)',
    'stages': 'candidates -> graph -> commands: refinement, start winding, '
              'warmup, continuous validation of both commands',
}

REQUIRED_COLUMNS = ('timestamp', 'q_I_G_x', 'q_I_G_y', 'q_I_G_z', 'q_I_G_w')

# Input criteria, declared before any recording is inspected. The builder
# itself refuses a non-uniform step at 1e-9 s, so the check agrees with it.
STEP_TOLERANCE_S = 1e-9
QUATERNION_NORM_TOLERANCE = 1e-6

DIGEST_TIME_DECIMALS = 9
DIGEST_ROTATION_DECIMALS = 12
DIGEST_POSITION_DECIMALS = 9

FAILURE_CLASSES = ('input', 'candidate_discovery', 'graph_connectivity',
                   'placement', 'warmup_home', 'refinement',
                   'continuous_limit', 'pipeline_error')


# --------------------------------------------------------------------------
# Input
# --------------------------------------------------------------------------

def inspect_recording(csv_path, num_waypoints=None):
    """Schema, timestamps and quaternions, checked before any stage runs."""
    import pandas as pd
    from scipy.spatial.transform import Rotation

    from ur10e_trajectory_pkg.failure_census import file_digest

    record = {'csv_path': str(csv_path), 'source_digest': file_digest(csv_path),
              'problems': []}
    problems = record['problems']
    try:
        frame = pd.read_csv(csv_path)
    except (OSError, ValueError) as exc:
        problems.append(f'unreadable: {exc}')
        record['passed'] = False
        return record
    missing = [column for column in REQUIRED_COLUMNS if column not in frame.columns]
    if missing:
        problems.append(f'missing columns {missing}')
        record['passed'] = False
        return record

    times = frame['timestamp'].to_numpy(dtype=float)
    quaternions = frame[list(REQUIRED_COLUMNS[1:])].to_numpy(dtype=float)
    record['samples'] = int(len(frame))
    if num_waypoints is not None and len(frame) < int(num_waypoints):
        problems.append(f'{len(frame)} samples, fewer than the {num_waypoints} '
                        'waypoints asked for')
    if len(frame) < 2:
        problems.append('fewer than two samples')
    if not (np.all(np.isfinite(times)) and np.all(np.isfinite(quaternions))):
        problems.append('non-finite timestamps or quaternions')
    if problems:
        record['passed'] = False
        return record

    steps = np.diff(times)
    record['duration_s'] = float(times[-1] - times[0])
    record['step_s'] = float(steps[0])
    record['step_range_s'] = [float(steps.min()), float(steps.max())]
    if np.any(steps <= 0.0):
        problems.append('timestamps do not increase')
    elif not np.allclose(steps, steps[0], atol=STEP_TOLERANCE_S, rtol=0.0):
        problems.append(f'not uniformly sampled: steps span {steps.min():.12g} '
                        f'to {steps.max():.12g} s')
    norm_error = float(np.max(np.abs(np.linalg.norm(quaternions, axis=1) - 1.0)))
    record['max_quaternion_norm_error'] = norm_error
    if norm_error > QUATERNION_NORM_TOLERANCE:
        problems.append(f'quaternion norm off unity by {norm_error:.3g}')

    if not problems:
        rotations = Rotation.from_quat(quaternions)
        rates = np.degrees((rotations[1:] * rotations[:-1].inv()).magnitude() / steps)
        record['angular_rate_deg_s'] = {'min': float(rates.min()),
                                        'median': float(np.median(rates)),
                                        'max': float(rates.max())}
    record['passed'] = not problems
    return record


# --------------------------------------------------------------------------
# Identities
# --------------------------------------------------------------------------

def _update(hasher, label, values, decimals):
    values = np.asarray(values, dtype=float)
    hasher.update(f'{label}{list(values.shape)}r{decimals};'.encode())
    rounded = np.round(values, decimals) + 0.0      # folds -0.0 into 0.0
    hasher.update(np.ascontiguousarray(rounded, dtype='<f8').tobytes())


def _rotation_matrices(quaternions):
    from scipy.spatial.transform import Rotation
    return Rotation.from_quat(np.asarray(quaternions, dtype=float)).as_matrix()


def motion_digest(times, quaternions):
    """Relative timestamps and R_IG(0)^T R_IG(t): the tumble itself."""
    times = np.asarray(times, dtype=float)
    matrices = _rotation_matrices(quaternions)
    relative = np.einsum('ji,tjk->tik', matrices[0], matrices)
    hasher = hashlib.sha256(b'motion_digest/v1;')
    _update(hasher, 'time', times - times[0], DIGEST_TIME_DECIMALS)
    _update(hasher, 'rotation', relative, DIGEST_ROTATION_DECIMALS)
    return hasher.hexdigest()


def task_digest(x, y, z, quaternions, times):
    """The targets as the planner receives them: the IK case."""
    times = np.asarray(times, dtype=float)
    hasher = hashlib.sha256(b'task_digest/v1;')
    _update(hasher, 'time', times - times[0], DIGEST_TIME_DECIMALS)
    _update(hasher, 'position', np.column_stack([x, y, z]), DIGEST_POSITION_DECIMALS)
    _update(hasher, 'rotation', _rotation_matrices(quaternions),
            DIGEST_ROTATION_DECIMALS)
    return hasher.hexdigest()


def first_rung(csv_path, num_waypoints):
    """The first spin-up the pipeline's derived ladder would try."""
    from ur10e_trajectory_pkg.ClientNode import recorded_start_rate
    from ur10e_trajectory_pkg.pipeline import spin_up_ladder

    measured = recorded_start_rate(csv_path, num_waypoints)
    return spin_up_ladder(measured['rate_rad_s'], measured['step_s'])[0], measured


def task_digest_at(csv_path, num_waypoints, spin_up_s):
    from ur10e_trajectory_pkg.ClientNode import build_trajectory_targets
    return task_digest(*build_trajectory_targets(csv_path, num_waypoints,
                                                 spin_up_s=spin_up_s))


def recording_identities(csv_path, num_waypoints):
    """All three identities, over the samples the run plans."""
    import pandas as pd

    from ur10e_trajectory_pkg.failure_census import file_digest

    frame = pd.read_csv(csv_path).iloc[:int(num_waypoints)]
    rung, measured = first_rung(csv_path, num_waypoints)
    return {'source_digest': file_digest(csv_path),
            'motion_digest': motion_digest(frame['timestamp'],
                                           frame[list(REQUIRED_COLUMNS[1:])]),
            'task_digest': task_digest_at(csv_path, num_waypoints, rung),
            'task_digest_spin_up_s': rung,
            'recorded_start_rate_rad_s': measured['rate_rad_s'],
            'num_waypoints': int(num_waypoints)}


def group_identities(records):
    """De-duplicate on task_digest; flag motions shared across tasks.

    records: dicts with case_id, motion_digest and task_digest, in corpus
    order. The first case of each task is the one that runs.
    """
    first_of_task, motion_tasks = {}, {}
    for record in records:
        first_of_task.setdefault(record['task_digest'], record['case_id'])
        motion_tasks.setdefault(record['motion_digest'], []).append(record)
    out = []
    for record in records:
        owner = first_of_task[record['task_digest']]
        others = sorted({other['case_id'] for other in motion_tasks[record['motion_digest']]
                         if other['task_digest'] != record['task_digest']})
        out.append({'case_id': record['case_id'],
                    'duplicate_of': None if owner == record['case_id'] else owner,
                    'same_motion_different_task': others})
    return out


# --------------------------------------------------------------------------
# Classification and metrics
# --------------------------------------------------------------------------

def candidates_after_filters(graph):
    """Candidates left per layer after the condition, clearance and home filters.

    None when the graph predates the per-layer count. Layer 0 is what the home
    filter kept, since that is the last filter it sees.
    """
    filters = (graph or {}).get('candidate_filters') or {}
    condition = filters.get('condition') or {}
    generated = condition.get('candidates_per_layer')
    if generated is None:
        return None
    kept = np.asarray(generated, dtype=int) - np.asarray(
        condition.get('removed_per_layer', 0), dtype=int)
    clearance = (filters.get('self_clearance') or {}).get('removed_per_layer')
    if clearance is not None:
        kept = kept - np.asarray(clearance, dtype=int)
    home = filters.get('home_reachable')
    if home is not None and len(kept):
        kept[0] = int(home['kept'])
    return kept.tolist()


def _verdict(name, detail=None):
    return {'class': name, 'detail': detail}


def _continuous_reasons(task):
    reasons = []
    violations = task.get('limit_violations') or {}
    if violations:
        reasons.append('limits ' + ', '.join(
            f'{joint}.{kind}' for joint, kinds in sorted(violations.items())
            for kind in sorted(kinds)))
    if task.get('position_limit_violations'):
        reasons.append('position limits')
    if (task.get('collision') or {}).get('collision_found'):
        reasons.append('collision')
    clearance = task.get('self_clearance') or {}
    if clearance and not clearance.get('passed', True):
        reasons.append(f"self-clearance {1000.0 * clearance['min_distance_m']:.1f} mm")
    tracking = task.get('tracking') or {}
    if tracking and not tracking.get('within_tolerance', True):
        reasons.append('tracking')
    conditioning = task.get('conditioning') or {}
    if task.get('conditioning_ok') is False:
        reasons.append(f"conditioning {conditioning.get('max_condition_number', float('nan')):.1f}")
    if conditioning.get('twist_status') not in (None, 'pass'):
        reasons.append(f"twist {conditioning['twist_status']}")
    return reasons


def classify(input_check, summary, graph, commands):
    """One failure class per run, from the artifacts the run left."""
    from ur10e_trajectory_pkg import home_pose, pipeline

    if not (input_check or {}).get('passed'):
        return _verdict('input', '; '.join((input_check or {}).get('problems', [])))
    if summary is None:
        return _verdict('pipeline_error', 'no pipeline summary was written')
    # A pass needs no failed stage as well as status 'ok': pipelines before
    # the fix summarised a completed-but-failed commands stage as 'ok'.
    if summary.get('status') == 'ok' and not summary.get('failed_stage'):
        return _verdict('pass')
    stage, status = summary.get('failed_stage'), summary.get('status')

    if stage == pipeline.STAGE_CANDIDATES:
        return _verdict('candidate_discovery', 'candidate generation exited non-zero')

    if stage == pipeline.STAGE_GRAPH:
        if graph is None:
            return _verdict('pipeline_error', 'the graph stage refused before '
                                              'writing a graph')
        if graph.get('complete_path'):
            return _verdict('pipeline_error', 'graph stage failed with a complete path')
        layer = graph.get('first_disconnected_layer')
        kept = candidates_after_filters(graph)
        home = (graph.get('candidate_filters') or {}).get('home_reachable')
        if layer == 0 and home and home.get('layer_0_candidates') and not home.get('kept'):
            return _verdict('warmup_home', f"none of {home['layer_0_candidates']} "
                                           'layer-0 candidates reachable from the home')
        if kept is not None and layer is not None and layer < len(kept):
            if kept[layer] <= 0:
                return _verdict('candidate_discovery',
                                f'layer {layer} has no candidate left after the filters')
            return _verdict('graph_connectivity',
                            f'layer {layer} has {kept[layer]} candidates but no legal '
                            f'transition reaches them')
        return _verdict('graph_connectivity', f'first disconnected layer {layer} '
                                              '(per-layer counts unavailable)')

    if stage == pipeline.STAGE_COMMANDS:
        if status in (pipeline.NO_VALID_WINDING, home_pose.NO_REACHABLE_START,
                      home_pose.HOME_RECHECK_FAILED, home_pose.HOME_MISMATCH):
            return _verdict('warmup_home', status)
        if status == home_pose.NO_CANDIDATES_HERE:
            return _verdict('candidate_discovery', status)
        if commands is None:
            return _verdict('pipeline_error', status or 'no commands artifact')
        refinement = commands.get('refinement') or {}
        if (refinement.get('status') == 'refinement_failed'
                or commands.get('refinement_meets_acceptance') is False):
            return _verdict('refinement', refinement.get('status'))
        warmup = commands.get('warmup_validation')
        if warmup is not None and not warmup.get('passed'):
            return _verdict('warmup_home', 'warmup validation failed')
        task = commands.get('task_validation')
        if task is not None and not task.get('passed'):
            return _verdict('continuous_limit', '; '.join(_continuous_reasons(task))
                            or 'task validation failed')
        return _verdict('pipeline_error', status)

    return _verdict('pipeline_error', status or f'failed at {stage}')


def _peak(values):
    return None if not values else float(np.max(values))


def case_metrics(summary, graph, commands):
    """The comparable figures, whatever stage a run reached."""
    summary, graph, commands = summary or {}, graph or {}, commands or {}
    kept = candidates_after_filters(graph)
    filters = graph.get('candidate_filters') or {}
    generated = (filters.get('condition') or {}).get('candidates_per_layer')
    home = filters.get('home_reachable') or {}
    task = commands.get('task_validation') or {}
    conditioning = task.get('conditioning') or {}
    tracking = task.get('tracking') or {}
    peaks = task.get('peak_command_stream') or {}
    warmup = commands.get('warmup') or {}
    ladder = summary.get('spin_up_ladder') or {}

    def spread(counts):
        if not counts:
            return None
        counts = np.asarray(counts)
        return {'min': int(counts.min()), 'min_layer': int(counts.argmin()),
                'median': float(np.median(counts)), 'max': int(counts.max())}

    return {
        'complete_path': graph.get('complete_path'),
        'first_disconnected_layer': graph.get('first_disconnected_layer'),
        'layers': graph.get('layers'),
        'candidates_generated': spread(generated),
        'candidates_after_filters': spread(kept),
        'candidates_after_filters_per_layer': kept,
        'layer_0_reachable': (None if not home else
                              {'kept': home.get('kept'),
                               'of': home.get('layer_0_candidates'),
                               'routes': home.get('routes')}),
        'best_condition_lower_bound': (filters.get('best_condition_lower_bound')
                                       or {}).get('value'),
        'graph_path_max_condition': graph.get('path_max_condition'),
        'worst_condition': conditioning.get('max_condition_number'),
        'min_self_clearance_m': (task.get('self_clearance') or {}).get('min_distance_m'),
        'min_alpha_star': conditioning.get('min_alpha_star'),
        'twist_status': conditioning.get('twist_status'),
        'max_position_error_m': tracking.get('max_position_error_m'),
        'max_orientation_error_rad': tracking.get('max_orientation_error_rad'),
        'collision_found': (task.get('collision') or {}).get('collision_found'),
        'limit_violations': task.get('limit_violations'),
        'position_limit_violations': task.get('position_limit_violations'),
        'peak_velocity_rad_s': _peak(peaks.get('velocity')),
        'peak_acceleration_rad_s2': _peak(peaks.get('acceleration')),
        'peak_jerk_rad_s3': _peak(peaks.get('jerk')),
        'peak_per_joint': peaks or None,
        'velocity_budget_used': (task.get('velocity_headroom') or {}).get('max_budget_used'),
        'task_validation_passed': task.get('passed'),
        'refinement': {key: (commands.get('refinement') or {}).get(key)
                       for key in ('status', 'level_m')} if commands else None,
        'warmup': (None if not commands else
                   {'passed': (commands.get('warmup_validation') or {}).get('passed'),
                    'route_kind': warmup.get('route_kind'),
                    'duration_s': warmup.get('total_duration_s')}),
        'start_winding': commands.get('start_winding'),
        'strict_home_windings': summary.get('strict_home_windings'),
        'spin_up': {key: ladder.get(key) for key in ('derived', 'rungs_s', 'chosen_s')}
                   | {'tried': [entry.get('spin_up_s') for entry in ladder.get('tried', [])]},
        'stage_seconds': {stage['stage']: stage['seconds']
                          for stage in summary.get('stages', [])},
        'ladder_seconds': [entry.get('seconds') for entry in ladder.get('tried', [])],
    }


# --------------------------------------------------------------------------
# Running
# --------------------------------------------------------------------------

def _load(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as handle:
        return json.load(handle)


def _dump(document, path):
    def plain(value):
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, np.ndarray):
            return value.tolist()
        raise TypeError(type(value).__name__)
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=1, default=plain)


def _artifact(path):
    from ur10e_trajectory_pkg.failure_census import file_digest
    if not path or not os.path.exists(path):
        return None
    return {'path': path, 'bytes': os.path.getsize(path), 'sha256': file_digest(path)}


def _revisions(source_meta):
    from ur10e_trajectory_pkg import environment
    from ur10e_trajectory_pkg.failure_census import repository_revision
    return {'ik': repository_revision(os.path.dirname(os.path.abspath(__file__))),
            'docker_image_id': os.environ.get('UR10E_IMAGE_ID'),
            'environment': environment.describe(),
            'source': source_meta}


def run_case(args):
    from ur10e_trajectory_pkg import pipeline

    os.makedirs(args.work_dir, exist_ok=True)
    started = time.perf_counter()
    record = {'schema_version': SCHEMA_VERSION, 'case_id': args.case_id,
              'protocol': PROTOCOL, 'waypoints': args.waypoints,
              'placement_sensitivity': 'untested (nominal placement only)',
              'revisions': _revisions(_load(args.source_meta))}
    out = os.path.join(args.work_dir, 'benchmark_case.json')

    check = inspect_recording(args.csv, args.waypoints)
    record['input'] = check
    if not check['passed']:
        record['classification'] = classify(check, None, None, None)
        record['seconds'] = time.perf_counter() - started
        _dump(record, out)
        print(f"{args.case_id}: input failure: {record['classification']['detail']}")
        return 1
    record['identities'] = recording_identities(args.csv, args.waypoints)

    summary_path = os.path.join(args.work_dir, 'pipeline.json')
    argv = ['--csv', args.csv, '--waypoints', str(args.waypoints),
            '--home-json', args.home_json, '--urdf', args.urdf,
            '--work-dir', args.work_dir, '--summary', summary_path]
    if args.via_poses:
        argv += ['--via-poses', args.via_poses]
    try:
        pipeline.main(argv)
    except Exception as exc:  # recorded, never swallowed silently
        record['pipeline_exception'] = f'{type(exc).__name__}: {exc}'
    summary = _load(summary_path)
    paths = (summary or {}).get('artifacts') or {}
    graph, commands = _load(paths.get('graph')), _load(paths.get('commands'))

    ladder = (summary or {}).get('spin_up_ladder') or {}
    record['identities']['task_digest_per_rung_tried'] = {
        str(entry['spin_up_s']): task_digest_at(args.csv, args.waypoints,
                                                entry['spin_up_s'])
        for entry in ladder.get('tried', [])}
    record['classification'] = classify(check, summary, graph, commands)
    if record.get('pipeline_exception') and record['classification']['class'] != 'pass':
        record['classification'] = _verdict('pipeline_error', record['pipeline_exception'])
    record['status'] = (summary or {}).get('status')
    record['failed_stage'] = (summary or {}).get('failed_stage')
    record['metrics'] = case_metrics(summary, graph, commands)
    record['artifacts'] = {name: _artifact(path) for name, path in
                           dict(paths, pipeline=summary_path).items()}
    record['seconds'] = time.perf_counter() - started
    # Linux reports ru_maxrss in kilobytes. The stages run in this process,
    # so this is the run's peak, the figure that decides how many run at once.
    record['peak_rss_gb'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0 ** 2
    _dump(record, out)
    verdict = record['classification']
    print(f"{args.case_id}: {verdict['class']}"
          + (f" ({verdict['detail']})" if verdict['detail'] else '')
          + f" in {record['seconds']:.0f} s, peak {record['peak_rss_gb']:.2f} GB")
    return 0 if verdict['class'] == 'pass' else 1


def run_inspect(args):
    entries = _load(args.entries)
    records = []
    for entry in entries:
        check = inspect_recording(entry['csv'], args.waypoints)
        record = {'case_id': entry['case_id'], 'csv': entry['csv'], 'input': check,
                  'source': entry.get('source')}
        if check['passed']:
            record.update(recording_identities(entry['csv'], args.waypoints))
        records.append(record)
        print(f"{entry['case_id']}: input {'ok' if check['passed'] else check['problems']}"
              + (f" | task {record['task_digest'][:12]} motion "
                 f"{record['motion_digest'][:12]}" if check['passed'] else ''))
    valid = [record for record in records if record['input']['passed']]
    groups = {group['case_id']: group for group in group_identities(valid)}
    for record in records:
        record['grouping'] = groups.get(record['case_id'])
    unique = [record['case_id'] for record in valid
              if groups[record['case_id']]['duplicate_of'] is None]
    _dump({'schema_version': SCHEMA_VERSION, 'waypoints': args.waypoints,
           'digest_decimals': {'time': DIGEST_TIME_DECIMALS,
                               'rotation': DIGEST_ROTATION_DECIMALS,
                               'position': DIGEST_POSITION_DECIMALS},
           'records': records, 'unique_task_cases': unique}, args.out)
    print(f'{len(unique)} distinct IK cases of {len(records)} recordings')
    return 0 if len(valid) == len(records) else 1


# --------------------------------------------------------------------------
# Report
# --------------------------------------------------------------------------

def rung_directory(case_dir, ladder, spin_up_s):
    """Where the pipeline put one rung's artifacts: its own directory when
    more than one rung was on the ladder, the work directory otherwise."""
    if len(ladder.get('rungs_s') or []) == 1:
        return case_dir
    return os.path.join(case_dir, f'spin_up_{spin_up_s:g}s')


def rung_classifications(case_dir, input_check):
    """Every rung tried, classified from its own artifacts.

    The case record classifies the LAST attempt only, so a rung that failed
    for a reason no spin-up can fix -- a warmup, say -- would otherwise read
    as "this trajectory needed a longer spin-up".
    """
    summary = _load(os.path.join(case_dir, 'pipeline.json')) or {}
    ladder = summary.get('spin_up_ladder') or {}
    out = []
    for entry in ladder.get('tried') or []:
        directory = rung_directory(case_dir, ladder, entry['spin_up_s'])
        attempt = ({'status': 'ok'} if entry.get('returncode') == 0 else
                   {'status': entry.get('status'), 'failed_stage': entry.get('failed_stage')})
        verdict = classify(input_check, attempt,
                           _load(os.path.join(directory, 'graph.json')),
                           _load(os.path.join(directory, 'commands.json')))
        out.append(dict(verdict, spin_up_s=entry['spin_up_s'],
                        seconds=entry.get('seconds')))
    return out


def reclassify_case(case, case_dir):
    """The case's verdict recomputed from its artifacts with the current rules.

    Stored verdicts come from the revision that ran; a classifier fix must
    reach cases already on disk, and a changed verdict is shown, not hidden.
    """
    case = dict(case)
    case['stored_classification'] = case.get('classification')
    input_check = case.get('input') or {}
    rungs = rung_classifications(case_dir, input_check)
    case['rungs'] = rungs
    if not input_check.get('passed'):
        return case
    summary = _load(os.path.join(case_dir, 'pipeline.json'))
    paths = (summary or {}).get('artifacts') or {}
    case['classification'] = classify(input_check, summary, _load(paths.get('graph')),
                                      _load(paths.get('commands')))
    return case


def _fmt(value, spec, scale=1.0):
    return '—' if value is None else format(value * scale, spec)


def report(identities, cases):
    """Rows in corpus order, with a class count and the de-duplication record."""
    by_case = {case['case_id']: case for case in cases}
    rows = []
    for record in (identities or {}).get('records', []):
        case = by_case.get(record['case_id'])
        metrics = (case or {}).get('metrics') or {}
        rows.append({
            'case_id': record['case_id'],
            'grouping': record.get('grouping'),
            'task_digest': record.get('task_digest'),
            'motion_digest': record.get('motion_digest'),
            'source_digest': record['input'].get('source_digest'),
            'angular_rate_deg_s': record['input'].get('angular_rate_deg_s'),
            'ran': case is not None,
            'class': (case or {}).get('classification', {}).get('class')
                     if case else ('input' if not record['input']['passed'] else None),
            'detail': (case or {}).get('classification', {}).get('detail'),
            'stored_class': ((case or {}).get('stored_classification') or {}).get('class'),
            'rungs': (case or {}).get('rungs') or [],
            'seconds': (case or {}).get('seconds'),
            'peak_rss_gb': (case or {}).get('peak_rss_gb'),
            'metrics': metrics,
        })
    counts = {}
    for row in rows:
        if row['ran']:
            counts[row['class']] = counts.get(row['class'], 0) + 1
    return {'schema_version': SCHEMA_VERSION, 'protocol': PROTOCOL,
            'waypoints': (identities or {}).get('waypoints'),
            'class_counts': counts, 'rows': rows,
            'revisions': sorted({json.dumps((case.get('revisions') or {}).get('ik'))
                                 for case in cases})}


def markdown(document):
    lines = [f"# Benchmark, {document['waypoints']} waypoints", '',
             'Class counts: ' + ', '.join(f'{name} {count}' for name, count in
                                          sorted(document['class_counts'].items())), '',
             '| case | task | rate °/s | class | path | 1st gap | min cand (layer) '
             '| cond | clear mm | α* | pos err mm | jerk | warmup | spin-up | min | GB |',
             '|' + '---|' * 16]
    for row in document['rows']:
        m = row['metrics']
        rate = (row['angular_rate_deg_s'] or {}).get('median')
        cand = m.get('candidates_after_filters') or {}
        warm = m.get('warmup') or {}
        dup = (row['grouping'] or {}).get('duplicate_of')
        lines.append('| ' + ' | '.join([
            row['case_id'] + (f' (= {dup})' if dup else ''),
            (row['task_digest'] or '—')[:8],
            _fmt(rate, '.2f'),
            (row['class'] or 'not run') + (f": {row['detail']}" if row['detail'] else ''),
            {True: 'yes', False: 'no', None: '—'}[m.get('complete_path')],
            str(m.get('first_disconnected_layer') if m.get('first_disconnected_layer')
                is not None else '—'),
            f"{cand['min']} ({cand['min_layer']})" if cand else '—',
            _fmt(m.get('worst_condition'), '.1f'),
            _fmt(m.get('min_self_clearance_m'), '.1f', 1000.0),
            _fmt(m.get('min_alpha_star'), '.1f'),
            _fmt(m.get('max_position_error_m'), '.3f', 1000.0),
            _fmt(m.get('peak_jerk_rad_s3'), '.1f'),
            ('—' if warm.get('passed') is None else
             f"{'ok' if warm['passed'] else 'FAIL'} {_fmt(warm.get('duration_s'), '.2f')} s"),
            _fmt((m.get('spin_up') or {}).get('chosen_s'), 'g'),
            _fmt(row['seconds'], '.0f', 1.0 / 60.0),
            _fmt(row['peak_rss_gb'], '.1f'),
        ]) + ' |')
    failed_rungs = [(row, rung) for row in document['rows'] for rung in row['rungs']
                    if rung['class'] != 'pass']
    if failed_rungs:
        lines += ['', '## Failed spin-up rungs', '',
                  'Each rung is a whole cycle; a failure no spin-up can fix still '
                  'moves the ladder on.', '']
        lines += [f"- {row['case_id']} at {rung['spin_up_s']:g} s: {rung['class']}"
                  + (f" ({rung['detail']})" if rung['detail'] else '')
                  + f" — {_fmt(rung.get('seconds'), '.0f', 1.0 / 60.0)} min"
                  for row, rung in failed_rungs]
    changed = [row for row in document['rows']
               if row['ran'] and row['stored_class'] and row['stored_class'] != row['class']]
    if changed:
        lines += ['', 'Re-classified from artifacts: ' + '; '.join(
            f"{row['case_id']} {row['stored_class']} -> {row['class']}" for row in changed)]
    shared = [row for row in document['rows']
              if (row['grouping'] or {}).get('same_motion_different_task')]
    if shared:
        lines += ['', 'same_motion_different_task: ' + '; '.join(
            f"{row['case_id']} ~ {', '.join(row['grouping']['same_motion_different_task'])}"
            for row in shared)]
    return '\n'.join(lines) + '\n'


def run_report(args):
    identities = _load(args.identities)
    cases = []
    for path in sorted(glob.glob(os.path.join(args.cases, '*', 'benchmark_case.json'))):
        case = _load(path)
        if case:
            cases.append(reclassify_case(case, os.path.dirname(path)))
    document = report(identities, cases)
    _dump(document, args.out)
    if args.markdown:
        with open(args.markdown, 'w', encoding='utf-8') as handle:
            handle.write(markdown(document))
    print(json.dumps(document['class_counts']))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest='command', required=True)
    i = commands.add_parser('inspect')
    i.add_argument('--entries', required=True,
                   help='JSON list of {case_id, csv, source}')
    i.add_argument('--waypoints', type=int, required=True)
    i.add_argument('--out', required=True)
    c = commands.add_parser('case')
    c.add_argument('--case-id', required=True)
    c.add_argument('--csv', required=True)
    c.add_argument('--waypoints', type=int, required=True)
    c.add_argument('--home-json', required=True)
    c.add_argument('--via-poses', default=None)
    c.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    c.add_argument('--work-dir', required=True)
    c.add_argument('--source-meta', default=None,
                   help='JSON describing how the recording was generated')
    r = commands.add_parser('report')
    r.add_argument('--identities', required=True)
    r.add_argument('--cases', required=True, help='directory of case work dirs')
    r.add_argument('--out', required=True)
    r.add_argument('--markdown', default=None)
    args = parser.parse_args(argv)
    return {'inspect': run_inspect, 'case': run_case, 'report': run_report}[args.command](args)


if __name__ == '__main__':
    sys.exit(main())
