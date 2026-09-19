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
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import time

from ur10e_trajectory_pkg import section_search as search
from ur10e_trajectory_pkg import section_contract as contract

SCHEMA_VERSION = 2
STATUS_FOUND = 'certified_section_found'
STATUS_NONE = 'none_found_under_this_search'
STATUS_INPUT = 'input_refused'

DEFAULT_MAX_PROBE_SAMPLES = 2000
MEMORY_POLL_S = 2.0

EXIT_FOUND, EXIT_INPUT, EXIT_NONE = 0, 1, 2


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
                    '--spin-up-s', str(contract.first_rung_for(args.csv, samples, start))]
            if args.via_poses:
                argv += ['--via-poses', args.via_poses]
            return argv
        argv = ['--csv', args.csv, '--start-index', str(start), '--waypoints', str(samples),
                '--home-json', args.home_json,
                '--graph-workers', str(args.workers), '--urdf', args.urdf,
                '--work-dir', work_dir]
        if args.spin_up_policy == 'first-rung':
            # Derived from the slice's own starting rate, first rung only: the
            # ladder's retries repeat a whole cycle.
            argv += ['--spin-up-s', str(contract.first_rung_for(args.csv, samples, start))]
        if args.via_poses:
            argv += ['--via-poses', args.via_poses]
        return argv

    def __call__(self, start, samples, phase, reason):
        key = (int(start), int(samples))
        if key in self.cached:
            self.replayed += 1
            entry = dict(self.cached[key], cached=True)
            print(f'probe {contract.probe_name(start, samples)} replayed from the journal: '
                  f"{entry['classification']['class']}", flush=True)
            return entry
        work_dir = os.path.join(self.args.work_dir, 'probes',
                                contract.probe_name(start, samples))
        if os.path.isdir(work_dir):
            # A probe without a journal entry never finished; start it afresh.
            shutil.rmtree(work_dir)
        os.makedirs(work_dir)
        argv = self.argv(start, samples, work_dir)
        print(f'probe {contract.probe_name(start, samples)} ({phase}: {reason}), '
              f'{self.timeline.duration(start, samples):.1f} s recorded', flush=True)
        extra = {}
        if getattr(self.args, 'engine', 'pipeline') == 'fast' and \
                self.run_process is run_pipeline_process:
            extra['command'] = [sys.executable, '-m', 'ur10e_trajectory_pkg.fast_certify']
        run = self.run_process(argv, os.path.join(work_dir, 'pipeline.log'),
                               self.args.max_probe_memory_gb, **extra)
        evidence, summary, graph, commands, plan_path = contract.probe_evidence(
            work_dir, samples, self.timeline.times[start + 1] - self.timeline.times[start],
            bool(self.args.via_poses), run)
        entry = search.outcome(
            start, samples, evidence['passed'], evidence['classification'],
            evidence['graph_bound_samples'], evidence['shrink_hint_samples'],
            run['resource_limited'], run['seconds'],
            journal_schema_version=contract.JOURNAL_SCHEMA_VERSION,
            config_key=self.config_key,
            phase=phase, reason=reason, work_dir=work_dir, argv=argv,
            returncode=run['returncode'], peak_memory_gb=run['peak_memory_gb'],
            located_failure_time_s=evidence.get('located_failure_time_s'),
            recorded_duration_s=self.timeline.duration(start, samples),
            spin_up_s=(summary or {}).get('spin_up_s'),
            failed_stage=(summary or {}).get('failed_stage'),
            stage_seconds={s['stage']: s['seconds'] for s in (summary or {}).get('stages', [])},
            start_routes=contract.route_record(graph), plan_path=plan_path,
            finished_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
        with open(self.journal_path, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(entry, default=float) + '\n')
            handle.flush()
            os.fsync(handle.fileno())
        self.ran += 1
        print(f"probe {contract.probe_name(start, samples)}: {entry['classification']} in "
              f"{run['seconds']:.0f} s, peak {run['peak_memory_gb']:.2f} GB"
              + (f", graph bound {entry['graph_bound_samples']} samples"
                 if entry['graph_bound_samples'] is not None else ''), flush=True)
        return entry


# --------------------------------------------------------------------------
# Length-search reporting
# --------------------------------------------------------------------------

def build_report(args, inputs, timeline, runner, found, prober, validator_limits,
                 started_at, wall_seconds):
    from ur10e_trajectory_pkg import environment
    from ur10e_trajectory_pkg.target_builder import mount_record

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
        'provenance': {'code_revision': contract.git_revision(),
                       'image_id': os.environ.get('UR10E_IMAGE_ID'),
                       'host': socket.gethostname(),
                       'environment': environment.describe(),
                       'journal': prober.journal_path if prober else None,
                       'config_key': prober.config_key if prober else None,
                       'workers': args.workers},
        'limits': validator_limits,
        'limitations': list(contract.LIMITATIONS),
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
    parser.add_argument('--target-s', type=float, default=search.TARGET_SECTION_S)
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
                        default=contract.DEFAULT_MAX_PROBE_MEMORY_GB,
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


def main(argv=None, run_process=run_pipeline_process):
    args = parse_args(argv)
    started, started_at = time.perf_counter(), time.strftime('%Y-%m-%dT%H:%M:%S%z')
    os.makedirs(args.work_dir, exist_ok=True)
    report_path = os.path.join(args.work_dir, 'sections.json')
    inputs, timeline = contract.read_inputs(args)
    limits, lower, upper = contract.limits_record(args.urdf)
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
        hints=contract.load_hints(args.start_hints))
    prober = Prober(args, timeline, contract.config_key(args, inputs),
                    run_process=run_process)
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
        section, plan = contract.section_record(timeline, probe, runner.boundary(*best),
                                                lower, upper)
        problems = contract.plan_problems(plan, probe, inputs)
        if problems:
            raise RuntimeError(f'the certified plan does not match its section: {problems}')
        _write_json(plan_path, plan)
        section['plan'] = {'path': plan_path, 'sha256': contract.sha256_of(plan_path),
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
