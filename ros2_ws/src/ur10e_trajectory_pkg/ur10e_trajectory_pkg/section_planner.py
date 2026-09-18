#!/usr/bin/env python3
"""Find a certified section of an unseen recording: at least 60 s, then longer.

Given only a SISIFOS camera_traj.csv (timestamp and q_I_G are used; the
end-effector position is held fixed), the pinned home and optional via poses,
this searches the whole recording for a contiguous section the 7-DoF
rail-plus-arm system can execute, and returns its executable plan. Nobody
supplies start samples, section lengths or precomputed candidates or graphs.

  certified  every stage passes on the section as a slice of its own: its own
             targets and spin-up, a home-reachable lifted start (direct, or
             through a via pose), a graph path over freshly generated slice
             candidates, refinement, and continuous validation of the warmup
             and the task. A graph path alone is not a certificate
  duration   recorded motion from the recorded timestamps,
             t[end - 1] - t[start]; warmup and spin-up are excluded
  longest    the longest section this search certified under its declared
             budget. Not a proof that nothing longer is executable, and a
             search that finds nothing reports exactly that, never that the
             trajectory is infeasible

Scope: orientation only, nominal placement, the provisional identity T_EG.
Translation, scaling, camera-relative motion and mount optimisation are out
of scope. Acceleration, jerk and rail limits include provisional and assumed
values, so nothing here certifies hardware.

How it searches is section_search's policy. This module runs it: every probe
is the pipeline in its own process, watched for memory and killed past a
declared limit, and every finished probe is appended to a journal, so a run
that stops (budget, crash, interruption) resumes where it stopped and a larger
budget continues the same search.

Usage:
    python3 -m ur10e_trajectory_pkg.section_planner --csv camera_traj.csv \\
        --home-json home_choice.json --via-poses poses.json \\
        --work-dir runs/sections --workers 12 --budget-probes 8

Writes <work-dir>/sections.json and, when a section is certified,
<work-dir>/best_plan.json. Exits 0 when a section of at least the target was
certified, 2 when none was found under this search, 1 when the input was
refused.
"""
import argparse
import hashlib
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import time

import numpy as np

from ur10e_trajectory_pkg import section_search as search
from ur10e_trajectory_pkg.section_search import (  # noqa: F401  (re-exported)
    LOCATED_FAILURE_MARGIN_LAYERS,
    TARGET_SECTION_S,
    samples_through_layer,
)

SCHEMA_VERSION = 2
JOURNAL_SCHEMA_VERSION = 1
LIMIT_MARGIN_BOUNDARY_RAD = 0.05
JOINTS = ('rail', 'shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3')

STATUS_FOUND = 'certified_section_found'
STATUS_NONE = 'none_found_under_this_search'
STATUS_INPUT = 'input_refused'

DEFAULT_MAX_PROBE_SAMPLES = 2000
DEFAULT_MAX_PROBE_MEMORY_GB = 12.0
MEMORY_POLL_S = 2.0

EXIT_FOUND, EXIT_INPUT, EXIT_NONE = 0, 1, 2


# --------------------------------------------------------------------------
# Reading a probe's artifacts
# --------------------------------------------------------------------------

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
    """The earliest time a failed refinement or task validation names, or None.

    Joint position excursions carry their time, as do a failed self-clearance
    or tracking check. Acceleration and jerk violations record only peaks, so
    a failure made only of those is not located.
    """
    commands = commands or {}
    times = []
    for attempt in (commands.get('refinement') or {}).get('attempts', []):
        if attempt.get('passed'):
            continue
        times += [entry['first_time_s'] for entry
                  in (attempt.get('position_limit_excursions') or {}).values()]
    task = commands.get('task_validation') or {}
    times += [entry['first_time_s'] for entry
              in (task.get('position_limit_excursions') or {}).values()]
    for key in ('self_clearance', 'tracking'):
        check = task.get(key) or {}
        failed = (check.get('passed') is False if key == 'self_clearance'
                  else check.get('within_tolerance') is False)
        if failed and check.get('at_time_s') is not None:
            times.append(check['at_time_s'])
    return min(times) if times else None


def _load(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as handle:
        return json.load(handle)


def graph_bound_samples(graph, via_offered):
    """Recorded samples a disconnected graph covered, when that bounds its start.

    Only a disconnect under the default start policy bounds the start: the
    strict-winding backstop seeds fewer starts, and a graph that never tried
    the via poses it was offered seeded fewer than the policy allows.
    """
    if not graph or graph.get('complete_path'):
        return None
    if (graph.get('search') or {}).get('exhaustive') is False:
        # A beam (fast_certify) that disconnects has only failed to look far
        # enough; it bounds nothing.
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
        return [_load(os.path.join(work_dir, 'graph.json'))]
    return [_load(os.path.join(work_dir, f"spin_up_{entry['spin_up_s']:g}s", 'graph.json'))
            for entry in ladder.get('tried') or []]


def probe_evidence(work_dir, samples, step_s, via_offered, run):
    """What a finished probe's artifacts establish, in the policy's terms."""
    from ur10e_trajectory_pkg import benchmark

    summary = _load(os.path.join(work_dir, 'pipeline.json')) or None
    if (summary or {}).get('failed_stage') == 'deadline':
        run = dict(run, resource_limited=True, killed_reason=summary.get('status'))
    if run.get('resource_limited'):
        return {'passed': False, 'classification': {
            'class': 'resource_limit', 'detail': run.get('killed_reason')},
            'graph_bound_samples': None, 'shrink_hint_samples': None}, summary, None, None, None
    paths = (summary or {}).get('artifacts') or {}
    graph, commands = _load(paths.get('graph')), _load(paths.get('commands'))
    plan_path = paths.get('task_plan')
    verdict = benchmark.classify({'passed': True}, summary, graph, commands)
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


def _route_record(graph):
    home = ((graph or {}).get('candidate_filters') or {}).get('home_reachable') or {}
    return {'routes': home.get('routes'), 'attempts': home.get('attempts'),
            'chosen_start': home.get('chosen_start')}


# --------------------------------------------------------------------------
# Running a probe
# --------------------------------------------------------------------------

def _tree_memory_gb(process):
    """Proportional set size of a process and its descendants, in GB.

    PSS rather than RSS: forked graph workers share their parent's pages, and
    summing RSS would count those pages once per worker.
    """
    import psutil

    total = 0
    try:
        members = [process] + process.children(recursive=True)
    except psutil.Error:
        return 0.0
    for member in members:
        try:
            info = member.memory_full_info()
            total += getattr(info, 'pss', None) or info.rss
        except psutil.Error:
            continue
    return total / 1024.0 ** 3


def run_pipeline_process(argv, log_path, memory_limit_gb, poll_s=MEMORY_POLL_S,
                         command=None):
    """The pipeline in its own process group, killed past memory_limit_gb.

    command replaces the pipeline module (tests run a stand-in). Returns the
    return code, wall seconds, peak memory and whether it was killed.
    """
    import psutil

    started = time.perf_counter()
    peak, killed = 0.0, None
    with open(log_path, 'w', encoding='utf-8') as log:
        command = command or [sys.executable, '-m', 'ur10e_trajectory_pkg.pipeline']
        child = subprocess.Popen([*command, *argv], stdout=log,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        watched = psutil.Process(child.pid)
        while child.poll() is None:
            peak = max(peak, _tree_memory_gb(watched))
            if memory_limit_gb is not None and peak > memory_limit_gb:
                killed = f'peak memory {peak:.2f} GB exceeded the {memory_limit_gb:g} GB limit'
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.wait()
                break
            try:
                child.wait(timeout=poll_s)
            except subprocess.TimeoutExpired:
                pass
    return {'returncode': child.returncode, 'seconds': time.perf_counter() - started,
            'peak_memory_gb': peak, 'resource_limited': killed is not None,
            'killed_reason': killed}


def first_rung_for(csv_path, count, start):
    """The spin-up the pipeline's derived ladder would try first for a slice."""
    from ur10e_trajectory_pkg.ClientNode import recorded_start_rate
    from ur10e_trajectory_pkg.pipeline import spin_up_ladder

    measured = recorded_start_rate(csv_path, count, start_index=start)
    return spin_up_ladder(measured['rate_rad_s'], measured['step_s'])[0]


def probe_name(start, samples):
    return f'start_{int(start):06d}_samples_{int(samples):06d}'


class Prober:
    """certify(start, samples, phase, reason) for the search, with a journal."""

    def __init__(self, args, timeline, config_key, run_process=run_pipeline_process):
        self.args = args
        self.timeline = timeline
        self.config_key = config_key
        self.run_process = run_process
        self.journal_path = os.path.join(args.work_dir, 'probes.jsonl')
        self.cached, self.stale = self._read_journal()
        self.ran = 0
        self.replayed = 0

    def _read_journal(self):
        cached, stale = {}, 0
        if not os.path.exists(self.journal_path):
            return cached, stale
        with open(self.journal_path, encoding='utf-8') as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    entry = json.loads(line)
                except json.JSONDecodeError:
                    continue     # a line cut short by an interruption
                if entry.get('config_key') != self.config_key:
                    stale += 1
                    continue
                cached[(entry['start'], entry['samples'])] = entry
        return cached, stale

    def argv(self, start, samples, work_dir):
        args = self.args
        if getattr(args, 'engine', 'pipeline') == 'fast':
            argv = ['--csv', args.csv, '--start-index', str(start), '--waypoints', str(samples),
                    '--home-json', args.home_json, '--graph-workers', str(args.workers),
                    '--urdf', args.urdf, '--work-dir', work_dir,
                    '--spin-up-s', str(first_rung_for(args.csv, samples, start))]
            if args.via_poses:
                argv += ['--via-poses', args.via_poses]
            return argv
        argv = ['--csv', args.csv, '--start-index', str(start), '--waypoints', str(samples),
                '--placement', 'nominal', '--home-json', args.home_json,
                '--graph-workers', str(args.workers), '--urdf', args.urdf,
                '--work-dir', work_dir]
        if args.spin_up_policy == 'first-rung':
            # Derived from the slice's own starting rate, first rung only: the
            # ladder's retries repeat a whole cycle.
            argv += ['--spin-up-s', str(first_rung_for(args.csv, samples, start))]
        if args.via_poses:
            argv += ['--via-poses', args.via_poses]
        return argv

    def __call__(self, start, samples, phase, reason):
        key = (int(start), int(samples))
        if key in self.cached:
            self.replayed += 1
            entry = dict(self.cached[key], cached=True)
            print(f'probe {probe_name(start, samples)} replayed from the journal: '
                  f"{entry['classification']['class']}", flush=True)
            return entry
        work_dir = os.path.join(self.args.work_dir, 'probes', probe_name(start, samples))
        if os.path.isdir(work_dir):
            # A probe without a journal entry never finished; start it afresh.
            shutil.rmtree(work_dir)
        os.makedirs(work_dir)
        argv = self.argv(start, samples, work_dir)
        print(f'probe {probe_name(start, samples)} ({phase}: {reason}), '
              f'{self.timeline.duration(start, samples):.1f} s recorded', flush=True)
        extra = {}
        if getattr(self.args, 'engine', 'pipeline') == 'fast' and \
                self.run_process is run_pipeline_process:
            extra['command'] = [sys.executable, '-m', 'ur10e_trajectory_pkg.fast_certify']
        run = self.run_process(argv, os.path.join(work_dir, 'pipeline.log'),
                               self.args.max_probe_memory_gb, **extra)
        evidence, summary, graph, commands, plan_path = probe_evidence(
            work_dir, samples, self.timeline.times[start + 1] - self.timeline.times[start],
            bool(self.args.via_poses), run)
        entry = search.outcome(
            start, samples, evidence['passed'], evidence['classification'],
            evidence['graph_bound_samples'], evidence['shrink_hint_samples'],
            run['resource_limited'], run['seconds'],
            journal_schema_version=JOURNAL_SCHEMA_VERSION, config_key=self.config_key,
            phase=phase, reason=reason, work_dir=work_dir, argv=argv,
            returncode=run['returncode'], peak_memory_gb=run['peak_memory_gb'],
            located_failure_time_s=evidence.get('located_failure_time_s'),
            recorded_duration_s=self.timeline.duration(start, samples),
            spin_up_s=(summary or {}).get('spin_up_s'),
            failed_stage=(summary or {}).get('failed_stage'),
            stage_seconds={s['stage']: s['seconds'] for s in (summary or {}).get('stages', [])},
            start_routes=_route_record(graph), plan_path=plan_path,
            finished_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
        with open(self.journal_path, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(entry, default=float) + '\n')
            handle.flush()
            os.fsync(handle.fileno())
        self.ran += 1
        print(f"probe {probe_name(start, samples)}: {entry['classification']} in "
              f"{run['seconds']:.0f} s, peak {run['peak_memory_gb']:.2f} GB"
              + (f", graph bound {entry['graph_bound_samples']} samples"
                 if entry['graph_bound_samples'] is not None else ''), flush=True)
        return entry


# --------------------------------------------------------------------------
# Inputs, provenance, report
# --------------------------------------------------------------------------

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
    """What a journal entry must share with this run to be reused.

    The recording, home, via poses, URDF, spin-up policy, worker-independent
    planner code revision and the per-probe memory limit (a killed probe
    under one limit is not an outcome under another).
    """
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
    """The recording checked before any probe, and every input's identity."""
    import pandas as pd

    from ur10e_trajectory_pkg import benchmark

    check = benchmark.inspect_recording(args.csv)
    recording = {'csv_path': args.csv, 'sha256': check.get('source_digest'),
                 'samples': check.get('samples'), 'duration_s': check.get('duration_s'),
                 'step_range_s': check.get('step_range_s'),
                 'columns_used': ['timestamp', 'q_I_G_x', 'q_I_G_y', 'q_I_G_z', 'q_I_G_w'],
                 'problems': check.get('problems', []), 'passed': bool(check.get('passed'))}
    timeline = None
    if recording['passed']:
        frame = pd.read_csv(args.csv, usecols=['timestamp'])
        timeline = search.Timeline(frame['timestamp'].to_numpy(dtype=float))
        recording.update(first_timestamp_s=float(timeline.times[0]),
                         last_timestamp_s=float(timeline.times[-1]))
    home = _load(args.home_json)
    via = _load(args.via_poses) if args.via_poses else None
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


def section_record(timeline, probe, boundary, lower, upper):
    """The certified section: interval, plan, margins and where it came from."""
    plan = _load(probe.get('plan_path'))
    work_dir = probe['work_dir']
    commands = _load(os.path.join(work_dir, 'commands.json')) or {}
    graph = _load(os.path.join(work_dir, 'graph.json')) or {}
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
        'evidence': {'probe': search._probe_key(probe), 'work_dir': work_dir,
                     'artifacts': {name: os.path.join(work_dir, f'{name}.json') for name in
                                   ('pipeline', 'candidates', 'graph', 'commands',
                                    'task_plan')},
                     'seconds': probe.get('seconds'),
                     'peak_memory_gb': probe.get('peak_memory_gb')},
    })
    return record, plan


def plan_problems(plan, probe, inputs):
    """Why an exported plan does not describe the section it is reported as."""
    from ur10e_trajectory_pkg.ClientNode import mount_record

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


def build_report(args, inputs, timeline, runner, found, prober, validator_limits,
                 started_at, wall_seconds):
    from ur10e_trajectory_pkg import environment
    from ur10e_trajectory_pkg.ClientNode import mount_record

    document = {
        'schema_version': SCHEMA_VERSION,
        'command': ['section_planner', *(args.argv or [])],
        'started_at': started_at,
        'wall_seconds': wall_seconds,
        'definitions': {
            'section': 'recorded samples [start_index, end_index), half-open, planned as a '
                       'slice of its own',
            'recorded_duration_s': 'timestamps[end_index - 1] - timestamps[start_index]; '
                                   'warmup and spin-up excluded',
            'certified': 'own slice targets, spin-up, home-reachable lifted start, graph '
                         'path, refinement, warmup and task all pass continuous validation',
            'longest': 'best certified section found under the declared budget',
        },
        'target_s': args.target_s,
        'inputs': inputs,
        'mount': mount_record(),
        'provenance': {'code_revision': git_revision(),
                       'image_id': os.environ.get('UR10E_IMAGE_ID'),
                       'host': socket.gethostname(),
                       'environment': environment.describe(),
                       'journal': prober.journal_path if prober else None,
                       'config_key': prober.config_key if prober else None,
                       'workers': args.workers},
        'limits': validator_limits,
        'limitations': list(LIMITATIONS),
    }
    if runner is None:
        document.update(status=STATUS_INPUT, result=None, search=None)
        return document
    summary = runner.summary()
    summary['coverage'] = runner.coverage()
    probes = runner.probes
    document['search'] = summary
    document['probes'] = [{key: p.get(key) for key in (
        'start', 'samples', 'recorded_duration_s', 'phase', 'reason', 'passed',
        'classification', 'graph_bound_samples', 'shrink_hint_samples',
        'located_failure_time_s', 'resource_limited', 'seconds', 'peak_memory_gb',
        'stage_seconds', 'spin_up_s', 'failed_stage', 'work_dir', 'cached')}
        for p in probes]
    document['resources'] = {
        'probe_seconds_total': float(sum(p.get('seconds', 0.0) for p in probes)),
        'peak_probe_memory_gb': max((p.get('peak_memory_gb') or 0.0 for p in probes),
                                    default=0.0),
        'max_probe_memory_gb': args.max_probe_memory_gb,
        'max_probe_samples': args.max_probe_samples,
        'probes_run_now': prober.ran, 'probes_replayed_from_journal': prober.replayed,
        'stale_journal_entries_ignored': prober.stale,
    }
    document['status'] = STATUS_FOUND if found else STATUS_NONE
    document['result'] = found
    if summary['stop_reason'] is None:
        document['status'] = 'search_in_progress'
    elif not found:
        document['result'] = {
            'best_section': None,
            'statement': ('no section of at least the target was certified by this '
                          f"search (stopped: {summary['stop_reason']}); this is not a "
                          'finding that the trajectory is infeasible'),
        }
    return document


def _write_json(path, document):
    temporary = f'{path}.tmp'
    with open(temporary, 'w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=1, default=float)
    os.replace(temporary, path)


def _limits_record(urdf):
    """Joint limits and their certification status, or why they are unavailable."""
    try:
        from ament_index_python.packages import get_package_share_directory

        from ur10e_trajectory_pkg import motion_limits
        from ur10e_trajectory_pkg.failure_census import _validator

        validator = _validator(urdf, get_package_share_directory('ur_description'))
        lower, upper = (np.asarray(v, dtype=float) for v in validator.robot.qlim)
        record = {'statuses': motion_limits.limit_statuses(validator),
                  'may_certify_for_hardware': motion_limits.may_certify_for_hardware(
                      validator)}
        return record, lower, upper
    except Exception as exc:  # noqa: BLE001  the report says why instead
        return {'unavailable': f'{type(exc).__name__}: {exc}',
                'may_certify_for_hardware': False}, None, None


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--csv', required=True, help='SISIFOS camera_traj.csv')
    parser.add_argument('--home-json', required=True, help='the pinned home_choice.json')
    parser.add_argument('--via-poses', default=None,
                        help='screened poses a warmup may route through')
    parser.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    parser.add_argument('--work-dir', required=True)
    parser.add_argument('--workers', type=int, default=12,
                        help='processes the graph stage validates lifted starts with')
    parser.add_argument('--target-s', type=float, default=TARGET_SECTION_S)
    parser.add_argument('--first-probe-s', type=float, default=None,
                        help='length of the first probe at each start (default: the target)')
    parser.add_argument('--spin-up-policy', choices=['first-rung', 'ladder'],
                        default='first-rung')
    parser.add_argument('--engine', choices=['fast', 'pipeline'], default='fast',
                        help='fast: fast_certify, a beam that broadens only on failure (its '
                             'failures bound nothing); pipeline: the exhaustive pipeline, '
                             'whose graph disconnects bound a start')
    parser.add_argument('--start-stride-s', type=float, default=search.DEFAULT_START_STRIDE_S,
                        help='finest spacing of the start coverage grid')
    parser.add_argument('--length-resolution-s', type=float,
                        default=search.DEFAULT_LENGTH_RESOLUTION_S,
                        help='stop refining a section length within this much motion')
    parser.add_argument('--growth', type=float, default=search.DEFAULT_GROWTH)
    parser.add_argument('--max-probes-per-start', type=int,
                        default=search.DEFAULT_MAX_PROBES_PER_START)
    parser.add_argument('--max-probe-samples', type=int, default=DEFAULT_MAX_PROBE_SAMPLES,
                        help='longest slice ever probed; memory grows with slice length')
    parser.add_argument('--max-probe-memory-gb', type=float,
                        default=DEFAULT_MAX_PROBE_MEMORY_GB,
                        help='kill a probe whose process tree exceeds this (PSS)')
    parser.add_argument('--budget-probes', type=int, default=10)
    parser.add_argument('--budget-probe-samples', type=int, default=None)
    parser.add_argument('--budget-hours', type=float, default=None,
                        help='probe hours, counted from the journal')
    parser.add_argument('--start-hints', default=None,
                        help='JSON list of {"start": index, "score": number}: reorders '
                             'starts within a coverage level, adds off-grid starts, '
                             'never removes one')
    args = parser.parse_args(argv)
    args.argv = list(argv) if argv is not None else sys.argv[1:]
    if args.engine == 'fast' and args.spin_up_policy != 'first-rung':
        parser.error('the fast engine plans the first spin-up rung only')
    return args


def load_hints(path):
    if not path:
        return None
    return {int(entry['start']): float(entry['score']) for entry in _load(path)}


def main(argv=None, run_process=run_pipeline_process):
    args = parse_args(argv)
    started, started_at = time.perf_counter(), time.strftime('%Y-%m-%dT%H:%M:%S%z')
    os.makedirs(args.work_dir, exist_ok=True)
    report_path = os.path.join(args.work_dir, 'sections.json')
    inputs, timeline = read_inputs(args)
    limits, lower, upper = _limits_record(args.urdf)
    if timeline is None:
        document = build_report(args, inputs, None, None, None, None, limits, started_at,
                                time.perf_counter() - started)
        _write_json(report_path, document)
        print(f"input refused: {inputs['recording']['problems']}", flush=True)
        return EXIT_INPUT

    config = search.SearchConfig(
        target_s=args.target_s, first_probe_s=args.first_probe_s,
        max_probe_samples=args.max_probe_samples, start_stride_s=args.start_stride_s,
        length_resolution_s=args.length_resolution_s, growth=args.growth,
        max_probes_per_start=args.max_probes_per_start,
        budget=search.Budget(args.budget_probes, args.budget_probe_samples,
                             args.budget_hours),
        hints=load_hints(args.start_hints))
    prober = Prober(args, timeline, config_key(args, inputs), run_process=run_process)
    runner = search.SectionSearch(timeline, config)
    # Certificates already in the journal (fast_section's, or an earlier
    # length run's) are where lengthening starts, not something to rediscover
    # by walking the coverage order again.
    for (start, samples), entry in sorted(prober.cached.items()):
        if entry['passed'] and start + samples <= len(timeline) and \
                timeline.duration(start, samples) >= args.target_s - search.TIME_EPS_S:
            runner.record(dict(entry, cached=True, preloaded=True))
            prober.replayed += 1

    def checkpoint(_):
        # A report after every probe, so an interrupted search still says
        # what it had established.
        _write_json(report_path, build_report(
            args, inputs, timeline, runner, None, prober, limits, started_at,
            time.perf_counter() - started))

    runner.run(prober, on_probe=checkpoint)

    found = None
    plan_path = os.path.join(args.work_dir, 'best_plan.json')
    if os.path.exists(plan_path):
        os.remove(plan_path)
    best = runner.best
    if best is not None and runner.target_met():
        probe = next(p for p in runner.probes if p['passed']
                     and (p['start'], p['samples']) == best)
        section, plan = section_record(timeline, probe, runner.boundary(*best),
                                       lower, upper)
        problems = plan_problems(plan, probe, inputs)
        if problems:
            raise RuntimeError(f'the certified plan does not match its section: {problems}')
        _write_json(plan_path, plan)
        section['plan'] = {'path': plan_path, 'sha256': sha256_of(plan_path),
                           'q_path_states': len(plan['q_path'])}
        others = [dict(s, evidence_probe={'start': s['start_index'], 'samples': s['samples']})
                  for s in runner.summary()['certified_sections']
                  if (s['start_index'], s['samples']) != best]
        found = {'best_section': section, 'other_certified_sections': others,
                 'target_met': True}
    document = build_report(args, inputs, timeline, runner, found, prober, limits,
                            started_at, time.perf_counter() - started)
    _write_json(report_path, document)
    coverage = document['search']['coverage']
    print(json.dumps({
        'status': document['status'], 'stop_reason': document['search']['stop_reason'],
        'best': None if not found else {k: found['best_section'][k] for k in (
            'start_index', 'end_index', 'recorded_duration_s')},
        'probes': len(runner.probes), 'probed_starts': coverage['probed_starts'],
        'grid_starts': coverage['grid_starts'],
        'peak_probe_memory_gb': document['resources']['peak_probe_memory_gb'],
        'report': report_path}), flush=True)
    return EXIT_FOUND if found else EXIT_NONE


if __name__ == '__main__':
    sys.exit(main())
