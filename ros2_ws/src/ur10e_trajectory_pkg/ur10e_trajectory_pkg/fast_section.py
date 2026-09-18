#!/usr/bin/env python3
"""A certified section of at least 60 s, quickly, or an honest "inconclusive".

The target: a certified section of at least the target duration within a
declared wall-clock budget (default five minutes, counted from process start),
on an unseen SISIFOS recording, with nothing supplied but the recording, the
pinned home and optional via poses. The first certified section is published
the moment it passes; making it longer is a separate, resumable mode
(section_planner, which reads this run's journal).

The contract, in full:

  A certified section of at least the target within the budget when this
  search finds one -- with its executable plan published the moment it
  passes -- and otherwise INCONCLUSIVE. Nothing here ever reports that a
  trajectory has no such section: this is a beam, and an adjudication of
  five inconclusive recordings found real 60 s sections in several of
  them, at starts the beam had reached but not got through. Deciding that
  a recording holds no section needs the exhaustive pipeline
  (--engine pipeline), whose graph disconnect is a sound bound, and that
  costs tens of minutes per slice, not minutes per recording.

  Making a found section LONGER is not part of this mode either; that is
  section_planner, which reads this run's journal and continues.

  certified     fast_certify's certificate: home gate, refinement, warmup
                and continuous validation of warmup and task, exactly as
                the pipeline checks them
  answers       certified_section_found, with the plan, when one passed;
                inconclusive_under_this_budget when the budget ran out first;
                none_found_under_this_search when every scheduled attempt
                finished without one -- still inconclusive about the
                recording, since the beam is not exhaustive
  budget        wall clock from process start. Every wait on the worker
                pool ends at the deadline, and the steps that run in this
                process check it between units of work, so the answer
                lands within a second or so of the budget rather than
                after whatever step was in flight

Schedule, deterministic. Starts come from section_search's coverage order;
round r tries the starts in the first r + 2 coverage levels, each with the
search allowed levels 0..r. Round 0 is the first start, the last and the
middle at the narrowest beam; later rounds add starts between them and allow
wider searches. A start retried in a later round replays its earlier work
from the shared caches. Aiming to find a good section fast is the goal; no
schedule can promise the longest section, or any section, of an arbitrary
recording within minutes, and the report says what was and was not tried.

Usage:
    python3 -m ur10e_trajectory_pkg.fast_section --csv camera_traj.csv \\
        --home-json home_choice.json --via-poses poses.json --work-dir runs/fast

Exit codes: 0 certified, 1 input refused, 2 none found, 3 inconclusive.
"""
import argparse
import json
import os
import resource
import shutil
import sys
import time

from ur10e_trajectory_pkg import section_search as search

SCHEMA_VERSION = 1
DEFAULT_TIME_BUDGET_S = 300.0

STATUS_FOUND = 'certified_section_found'
STATUS_INCONCLUSIVE = 'inconclusive_under_this_budget'
STATUS_NONE = 'none_found_under_this_search'
STATUS_INPUT = 'input_refused'
EXIT_FOUND, EXIT_INPUT, EXIT_NONE, EXIT_INCONCLUSIVE = 0, 1, 2, 3


def schedule(order, levels, search_levels):
    """(round, start, max search level) in the order they are attempted.

    order is section_search.start_order's list of (start, level, hinted) and
    levels its coverage levels. Round r covers the starts of coverage levels
    0..r+1 (hinted starts ride with level 0) at search levels 0..r, and a
    start already attempted at a level is not attempted there again.
    """
    level_of = {}
    for start, level, _ in order:
        level_of[start] = 0 if level == 'hinted' else int(level)
    out = []
    rounds = max(search_levels, len(levels) - 1)
    for round_number in range(rounds):
        cap = min(round_number, search_levels - 1)
        for start, _, _ in order:
            if level_of[start] <= round_number + 1:
                if not any(s == start and c == cap for _, s, c in out):
                    out.append((round_number, start, cap))
    return out


def process_age_s():
    """Seconds since this process started, imports included."""
    try:
        import psutil
        return time.time() - psutil.Process().create_time()
    except Exception:  # noqa: BLE001  unmeasurable: count from now
        return 0.0


class MemoryWatch:
    """Peak proportional set size of this process and its workers, sampled."""

    def __init__(self, period_s=2.0):
        import threading

        self.peak_gb, self.samples, self._stop = 0.0, 0, threading.Event()
        self._thread = threading.Thread(target=self._run, args=(period_s,), daemon=True)
        self._thread.start()

    def _run(self, period_s):
        try:
            import psutil
            me = psutil.Process()
        except Exception:  # noqa: BLE001  unmeasurable
            return
        while not self._stop.is_set():
            total = 0
            try:
                members = [me] + me.children(recursive=True)
            except psutil.Error:
                members = [me]
            for member in members:
                try:
                    info = member.memory_full_info()
                    total += getattr(info, 'pss', None) or info.rss
                except psutil.Error:
                    continue
            self.peak_gb = max(self.peak_gb, total / 1024.0 ** 3)
            self.samples += 1
            self._stop.wait(period_s)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=5.0)
        return {'peak_tree_pss_gb': self.peak_gb, 'samples': self.samples}


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--csv', required=True)
    parser.add_argument('--home-json', required=True)
    parser.add_argument('--via-poses', default=None)
    parser.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    parser.add_argument('--work-dir', required=True)
    parser.add_argument('--target-s', type=float, default=search.TARGET_SECTION_S)
    parser.add_argument('--time-budget-s', type=float, default=DEFAULT_TIME_BUDGET_S,
                        help='wall-clock budget from process start')
    parser.add_argument('--workers', type=int, default=max(1, (os.cpu_count() or 2) - 2),
                        help='processes validating lifted starts')
    parser.add_argument('--start-stride-s', type=float, default=search.DEFAULT_START_STRIDE_S)
    parser.add_argument('--start-hints', default=None)
    args = parser.parse_args(argv)
    args.argv = list(argv) if argv is not None else sys.argv[1:]
    return args


def _write(path, document):
    temporary = f'{path}.tmp'
    with open(temporary, 'w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=1, default=float)
    os.replace(temporary, path)


def journal_pass(args, inputs, timeline, start, samples, summary, work_dir, seconds):
    """Record a certificate where section_planner's length mode will replay it."""
    from ur10e_trajectory_pkg import section_planner

    planner_args = argparse.Namespace(urdf=args.urdf, spin_up_policy='first-rung',
                                      max_probe_memory_gb=section_planner.DEFAULT_MAX_PROBE_MEMORY_GB,
                                      engine='fast')
    key = section_planner.config_key(planner_args, inputs)
    evidence, _, graph, _, plan_path = section_planner.probe_evidence(
        work_dir, samples, timeline.times[start + 1] - timeline.times[start],
        inputs['via_poses'] is not None, {'resource_limited': False})
    entry = search.outcome(
        start, samples, evidence['passed'], evidence['classification'], None, None, False,
        seconds, journal_schema_version=section_planner.JOURNAL_SCHEMA_VERSION,
        config_key=key, phase='fast', reason='fast_section certificate', work_dir=work_dir,
        argv=None, returncode=0, peak_memory_gb=None, located_failure_time_s=None,
        recorded_duration_s=timeline.duration(start, samples),
        spin_up_s=summary.get('spin_up_s'), failed_stage=None,
        stage_seconds={s['stage']: s['seconds'] for s in summary.get('stages', [])},
        start_routes=section_planner._route_record(graph), plan_path=plan_path,
        engine='fast', finished_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
    with open(os.path.join(args.work_dir, 'probes.jsonl'), 'a', encoding='utf-8') as handle:
        handle.write(json.dumps(entry, default=float) + '\n')
    return entry


def main(argv=None, certify=None, context_factory=None):
    """certify and context_factory replace fast_certify's for tests."""
    from ur10e_trajectory_pkg import fast_certify, section_planner

    age_at_entry = process_age_s()
    entered = time.perf_counter()
    watch = MemoryWatch()
    args = parse_args(argv)
    deadline = entered + max(args.time_budget_s - age_at_entry, 0.0)
    os.makedirs(args.work_dir, exist_ok=True)
    report_path = os.path.join(args.work_dir, 'fast_section.json')
    plan_path = os.path.join(args.work_dir, 'best_plan.json')
    if os.path.exists(plan_path):
        os.remove(plan_path)

    def elapsed():
        return age_at_entry + time.perf_counter() - entered

    inputs, timeline = section_planner.read_inputs(args)
    limits, lower, upper = section_planner._limits_record(args.urdf)
    document = {
        'schema_version': SCHEMA_VERSION, 'command': ['fast_section', *args.argv],
        'started_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'target_s': args.target_s, 'time_budget_s': args.time_budget_s,
        'budget_clock': 'wall clock from process start, imports and setup included',
        'contract': {
            'promise': 'a certified section of at least target_s within time_budget_s when this '
                       'search finds one, with its plan published as soon as it passes',
            'otherwise': 'inconclusive: neither status reports that the recording has no such '
                         'section. The fast search is a beam, not exhaustive',
            'to_decide_infeasibility': 'run the exhaustive engine (section_planner '
                                       '--engine pipeline) whose graph disconnect bounds a start; '
                                       'it costs tens of minutes per slice',
            'longer_sections': 'not this mode: section_planner reads this run\'s journal and '
                               'extends the certificate'},
        'inputs': inputs, 'limits': limits,
        'limitations': list(section_planner.LIMITATIONS) + [
            'The fast search is a beam over on-demand candidates, not the exhaustive graph: '
            'an attempt that finds no certificate is evidence of nothing.'],
        'provenance': {'code_revision': section_planner.git_revision(),
                       'image_id': os.environ.get('UR10E_IMAGE_ID'),
                       'workers': args.workers, 'levels': [dict(l) for l in fast_certify.LEVELS]},
    }
    if timeline is None:
        document.update(status=STATUS_INPUT, elapsed_s=elapsed(), result=None)
        _write(report_path, document)
        print(f"input refused: {inputs['recording']['problems']}", flush=True)
        return EXIT_INPUT

    from ur10e_trajectory_pkg.ClientNode import mount_record
    document['mount'] = mount_record()
    last = timeline.last_start(args.target_s)
    stride = max(1, int(round(args.start_stride_s / timeline.step_s)))
    order, levels = ([], []) if last is None else search.start_order(
        last, stride, section_planner.load_hints(args.start_hints))
    plan = schedule(order, levels, len(fast_certify.LEVELS))
    document['schedule'] = {'rounds': (plan[-1][0] + 1) if plan else 0, 'attempts_planned': len(plan),
                            'grid_starts': len(order), 'stride_samples': stride}
    attempts = []
    document['attempts'] = attempts

    with open(args.home_json, encoding='utf-8') as handle:
        choice = json.load(handle)
    via = None
    if args.via_poses:
        with open(args.via_poses, encoding='utf-8') as handle:
            via = [e['configuration'] if isinstance(e, dict) else e for e in json.load(handle)]
    certify = certify or fast_certify.certify_slice
    context = (context_factory or fast_certify.FastContext)(
        args.urdf, choice['chosen']['configuration'], via, args.workers)
    document['setup_s'] = elapsed()
    status, found = None, None
    try:
        for round_number, start, cap in plan:
            if time.perf_counter() >= deadline:
                status = STATUS_INCONCLUSIVE
                break
            samples = timeline.samples_for(start, args.target_s)
            spin_up = section_planner.first_rung_for(args.csv, samples, start)
            work_dir = os.path.join(args.work_dir, 'attempts',
                                    f'{section_planner.probe_name(start, samples)}_level_{cap}')
            if os.path.isdir(work_dir):
                shutil.rmtree(work_dir)
            began = time.perf_counter()
            summary = certify(
                context, choice, args.csv, start, samples, spin_up, work_dir,
                deadline=deadline, levels=fast_certify.LEVELS[:cap + 1])
            seconds = time.perf_counter() - began
            record = {'round': round_number, 'start': start, 'samples': samples,
                      'max_search_level': cap, 'status': summary.get('status'),
                      'failed_stage': summary.get('failed_stage'), 'seconds': seconds,
                      'finished_at_elapsed_s': elapsed(), 'work_dir': work_dir,
                      'stages': summary.get('stages'), 'search': summary.get('search')}
            attempts.append(record)
            print(f"attempt round {round_number} start {start} level<={cap}: "
                  f"{summary.get('status')} in {seconds:.1f} s (elapsed {elapsed():.1f} s)",
                  flush=True)
            if summary.get('status') == 'ok':
                found = (start, samples, summary, work_dir, seconds)
                status = STATUS_FOUND
                break
            if summary.get('failed_stage') == 'deadline':
                status = STATUS_INCONCLUSIVE
                break
            _write(report_path, dict(document, status='search_in_progress',
                                     elapsed_s=elapsed()))
        if status is None:
            status = STATUS_INCONCLUSIVE if time.perf_counter() >= deadline else STATUS_NONE
    finally:
        context.close()

    document.update(status=status, elapsed_s=elapsed(),
                    within_budget=bool(elapsed() <= args.time_budget_s + 1e-9))
    if found is not None:
        start, samples, summary, work_dir, seconds = found
        with open(os.path.join(work_dir, 'task_plan.json'), encoding='utf-8') as handle:
            plan_document = json.load(handle)
        probe = {'start': start, 'samples': samples, 'plan_path': os.path.join(
            work_dir, 'task_plan.json'), 'work_dir': work_dir,
            'classification': {'class': 'pass', 'detail': None},
            'spin_up_s': summary.get('spin_up_s'), 'seconds': seconds, 'peak_memory_gb': None,
            'start_routes': None}
        problems = section_planner.plan_problems(plan_document, probe, inputs)
        if problems:
            raise RuntimeError(f'the certified plan does not match its section: {problems}')
        # Published first, before anything else is written.
        _write(plan_path, plan_document)
        section, _ = section_planner.section_record(
            timeline, probe, {'kind': 'not_examined', 'next_sample_fails': None,
                              'statement': 'the fast mode stops at the first certificate; '
                                           'section_planner extends it'},
            lower, upper)
        section['start_route'] = (section_planner._load(
            os.path.join(work_dir, 'graph.json')) or {}).get(
            'candidate_filters', {}).get('home_reachable', {}).get('chosen_start')
        section['plan'] = {'path': plan_path, 'sha256': section_planner.sha256_of(plan_path),
                           'q_path_states': len(plan_document['q_path']),
                           'published_at_elapsed_s': elapsed()}
        document['result'] = {'best_section': section, 'target_met': True}
        journal_pass(args, inputs, timeline, start, samples, summary, work_dir, seconds)
        document['length_mode'] = (
            'python3 -m ur10e_trajectory_pkg.section_planner with the same --csv, --home-json, '
            '--via-poses and --work-dir replays this certificate from probes.jsonl and '
            'extends it')
    else:
        tried = sorted({a['start'] for a in attempts})
        document['result'] = {
            'best_section': None,
            'statement': (
                f'no certified section of {args.target_s:g} s within the '
                f'{args.time_budget_s:g} s budget; this is inconclusive, not a finding that the '
                'trajectory is infeasible' if status == STATUS_INCONCLUSIVE else
                'every scheduled fast attempt finished without a certificate; the fast search '
                'is not exhaustive, so this is not a finding that the trajectory is infeasible'),
            'starts_attempted': tried,
            'attempts_planned_not_run': len(plan) - len(attempts)}
    document['resources'] = {
        **watch.stop(),
        'peak_rss_self_gb': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0 ** 2,
        'peak_rss_children_gb': resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss / 1024.0 ** 2,
        'workers': args.workers}
    _write(report_path, document)
    best = (document['result'] or {}).get('best_section')
    print(json.dumps({'status': status, 'elapsed_s': round(document['elapsed_s'], 1),
                      'best': None if not best else {k: best[k] for k in (
                          'start_index', 'end_index', 'recorded_duration_s')},
                      'attempts': len(attempts), 'report': report_path}), flush=True)
    return {STATUS_FOUND: EXIT_FOUND, STATUS_NONE: EXIT_NONE,
            STATUS_INCONCLUSIVE: EXIT_INCONCLUSIVE}[status]


if __name__ == '__main__':
    sys.exit(main())
