#!/usr/bin/env python3
"""One command per trajectory, from a recording to a validated plan.

Running a new tumble used to be a chain of hand-driven steps, each with its
own flags and its own artifact, and every one of them had to agree about the
placement, the waypoint count and the spin-up. Disagreement was silent: a
graph built at one placement against targets built at another still produces
a path, just not one anyone checked.

    generate candidates -> graph -> commands (refine, wind, warm up, validate)

Refinement is not a separate stage here. home_pose commands refines the graph
path itself before choosing the start winding, so running path_refinement
first would do the same work twice and leave two refined paths to reconcile.

The stages are the same modules a person would run by hand, called with the
same flags, so this composes them rather than reimplementing them. Each
writes its own artifact; the summary written here says where they are, what
each stage returned, and how long it took.

The backstop: the home filter can only judge a start's own limits, so a
placement can plan with a winding-aware filter and then have no winding that
keeps the WHOLE path inside the limits. When commands reports that, the
graph is rebuilt with --strict-home-windings and commands runs again.

Usage:
    python3 -m ur10e_trajectory_pkg.pipeline \\
        --home-json home_choice.json --via-poses screened_poses.json \\
        --work-dir runs/nominal
"""
import argparse
import json
import os
import time
import traceback

import numpy as np

SCHEMA_VERSION = 1

# What a stage returning non-zero means, in the order the pipeline runs them.
STAGE_CANDIDATES = 'candidates'
STAGE_GRAPH = 'graph'
STAGE_COMMANDS = 'commands'

# Reference peak tool angular acceleration for seeding the spin-up ladder.
#
# The spin-up warps recorded time so the playback rate is tau'(t) = 3s^2 - 2s^3
# (s = t/T), whose peak rate of change of tool angular rate is w0 * 1.5 / T for
# a recording turning at w0. Requiring that peak to stay under this reference
# gives T >= w0 * 1.5 / reference, which is where the ladder starts.
#
# DECLARED STAND-IN, not a derivation. The honest bound comes from the arm's
# joint acceleration limits through the first waypoint's Jacobian; alpha*
# cannot supply it, since it answers a velocity question (J qdot = alpha v).
# This number only decides where the search STARTS. What decides whether a
# duration is accepted is the entry-state check and full continuous
# validation, exactly as for any other path.
SPIN_UP_REFERENCE_ANGULAR_ACCELERATION_RAD_S2 = 0.5

# How many rungs above the seed to try before giving up. Each rung is a whole
# pipeline run, so this is deliberately short: a trajectory needing far more
# spin-up than its starting rate implies is a result worth reporting, not
# something to search for.
SPIN_UP_LADDER_RUNGS = 4

# home_pose commands says this when no winding of the start keeps the whole
# path in limits with a valid warmup. That is the backstop's trigger, and the
# only failure the pipeline retries rather than reports.
NO_VALID_WINDING = 'no valid warmup to any legal winding of the start'

# The graph stage's failure when the candidates exist but no path spans them.
GRAPH_INCOMPLETE = 'no complete graph path'


def _run(stage, function, argv, record):
    """One stage, timed, with its argv kept for anyone repeating it by hand.

    A stage that raises is a failed stage, recorded with its traceback, not an
    exception that escapes before the run summary is written: the first
    5000-waypoint trial lost its summary that way.
    """
    started = time.perf_counter()
    entry = {'stage': stage, 'argv': list(argv)}
    try:
        code = int(function(argv) or 0)
    except (Exception, SystemExit) as exc:
        code = 1
        entry['exception'] = f'{type(exc).__name__}: {exc}'
        entry['traceback'] = traceback.format_exc()
    record.append(dict(entry, returncode=code,
                       seconds=time.perf_counter() - started))
    return code


def _load_json(path):
    if not path or not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as handle:
        return json.load(handle)


def _status_of(path):
    return (_load_json(path) or {}).get('status')


def _failure_status(stage, stages, paths):
    """What a failed stage's summary says, from what the stage left behind."""
    last = stages[-1] if stages else {}
    if last.get('exception'):
        return f"{stage} raised {last['exception']}"
    if stage == STAGE_COMMANDS:
        return _status_of(paths['commands'])
    if stage == STAGE_GRAPH:
        graph = _load_json(paths['graph'])
        if graph is not None and not graph.get('complete_path'):
            return (f"{GRAPH_INCOMPLETE}: first disconnected layer "
                    f"{graph.get('first_disconnected_layer')}")
    return None


def spin_up_ladder(rate_rad_s, step_s, rungs=SPIN_UP_LADDER_RUNGS,
                   reference=SPIN_UP_REFERENCE_ANGULAR_ACCELERATION_RAD_S2):
    """Candidate spin-up durations for a recording starting at rate_rad_s.

    The first rung is the shortest spin-up whose peak tool angular
    acceleration stays under the reference; later rungs double the time the
    ramp is given. Every rung is rounded up to a multiple of twice the
    recorded step, because apply_spin_up rounds the same way so the join
    lands on a recorded sample -- proposing durations it would silently round
    would make the ladder's rungs and the run's actual durations disagree.
    """
    quantum = 2.0 * float(step_s)
    seed = float(rate_rad_s) * 1.5 / float(reference)
    rungs_out, duration = [], max(seed, quantum)
    for _ in range(int(rungs)):
        rounded = quantum * np.ceil(duration / quantum - 1e-9)
        if not rungs_out or rounded > rungs_out[-1]:
            rungs_out.append(float(rounded))
        duration *= 2.0
    return rungs_out


def run(args):
    """Every stage in order, at the spin-up asked for or the shortest that works.

    With --spin-up-s the stages run once at that duration. Without it the
    duration is derived from the recording's starting rate and the rungs are
    tried shortest first, because a spin-up longer than the trajectory needs
    costs real time on every run of it. Each rung is a whole cycle -- the
    candidates depend on the spin-up, since it changes the targets -- so each
    gets its own directory and the summary says what each one cost.

    Returns (exit code, summary document).
    """
    from ur10e_trajectory_pkg.target_builder import recorded_start_rate
    from ur10e_trajectory_pkg.trajectory_input import DEFAULT_CSV_PATH

    os.makedirs(args.work_dir, exist_ok=True)
    ladder = {'derived': args.spin_up_s is None}
    if args.spin_up_s is not None:
        durations = [args.spin_up_s]
    else:
        # The slice's own starting rate: every slice starts from rest.
        measured = recorded_start_rate(args.csv or DEFAULT_CSV_PATH,
                                       num_waypoints=args.waypoints,
                                       start_index=getattr(args, 'start_index', 0))
        durations = spin_up_ladder(measured['rate_rad_s'], measured['step_s'])
        ladder.update(measured)
    ladder['rungs_s'] = list(durations)
    ladder['tried'] = []

    attempts = []
    for duration in durations:
        directory = (args.work_dir if len(durations) == 1
                     else os.path.join(args.work_dir, f'spin_up_{duration:g}s'))
        code, summary = _run_at(args, duration, directory)
        summary['spin_up_ladder'] = ladder
        attempts.append((code, summary))
        ladder['tried'].append({
            'spin_up_s': duration, 'returncode': code,
            'failed_stage': summary.get('failed_stage'),
            'status': summary.get('status'),
            'seconds': sum(stage['seconds'] for stage in summary['stages'])})
        if code == 0:
            ladder['chosen_s'] = duration
            return code, summary
    # Every rung failed. Report the last attempt, with the ladder alongside it
    # so the shortest failure is not mistaken for the only one tried.
    return attempts[-1]


def _run_at(args, spin_up_s, work_dir):
    """The stages, once, at one spin-up duration."""
    from ur10e_trajectory_pkg import candidate_generator, graph_planner, home_pose

    os.makedirs(work_dir, exist_ok=True)
    paths = {name: os.path.join(work_dir, f'{name}.json')
             for name in ('candidates', 'graph', 'commands', 'task_plan')}
    stages = []
    summary = {'schema_version': SCHEMA_VERSION, 'placement': 'nominal',
               'csv': args.csv, 'start_index': getattr(args, 'start_index', 0),
               'waypoints': args.waypoints, 'spin_up_s': spin_up_s,
               'home_json': args.home_json, 'via_poses': args.via_poses,
               'artifacts': paths, 'stages': stages, 'strict_home_windings': False}

    spin_up = [] if spin_up_s is None else ['--spin-up-s', str(spin_up_s)]
    # Every stage loads the recording itself, and each checks the one before
    # it loaded the same bytes, so all three are told.
    recording = [] if args.csv is None else ['--csv', args.csv]
    # A slice is part of the trajectory's identity: candidates and graph both
    # take it, and commands reads it back from the graph.
    slice_args = ([] if not getattr(args, 'start_index', 0)
                  else ['--start-index', str(args.start_index)])
    code = _run(STAGE_CANDIDATES, candidate_generator.main, [
        '--waypoints', str(args.waypoints),
        '--urdf', args.urdf, '--out', paths['candidates'], *spin_up, *recording,
        *slice_args],
        stages)
    if code:
        summary['failed_stage'] = STAGE_CANDIDATES
        return code, summary

    def graph_and_commands(strict):
        graph_argv = ['--candidates', paths['candidates'],
                      '--layers', str(args.waypoints),
                      '--urdf', args.urdf, '--out', paths['graph'], *spin_up,
                      *recording, *slice_args]
        # Validating every lifted start from the home is the graph stage's
        # longest step single-threaded; it parallelises cleanly.
        workers = getattr(args, 'graph_workers', 1) or 1
        if workers > 1:
            graph_argv += ['--workers', str(workers)]
        if args.home_json:
            graph_argv += ['--home-json', args.home_json]
        if args.via_poses:
            graph_argv += ['--via-poses', args.via_poses]
        if strict:
            graph_argv += ['--strict-home-windings']
        result = _run(STAGE_GRAPH, graph_planner.main, graph_argv, stages)
        if result:
            return STAGE_GRAPH, result
        commands_argv = ['commands', '--choice', args.home_json,
                         '--graph', paths['graph'], '--urdf', args.urdf,
                         '--out', paths['commands'], '--plan-out', paths['task_plan'],
                         *recording]
        if args.via_poses:
            commands_argv += ['--via-poses', args.via_poses]
        return STAGE_COMMANDS, _run(STAGE_COMMANDS, home_pose.main, commands_argv, stages)

    stage, code = graph_and_commands(strict=args.strict_home_windings)
    if code and stage == STAGE_COMMANDS and _status_of(paths['commands']) == NO_VALID_WINDING:
        # The filter judged the start's own limits; the whole path disagreed.
        summary['strict_home_windings'] = True
        summary['backstop'] = ('no winding kept the whole path in limits, so the '
                               'graph was rebuilt judging starts in the winding '
                               'they were stored in')
        stage, code = graph_and_commands(strict=True)
    if code:
        summary['failed_stage'] = stage
        summary['status'] = _failure_status(stage, stages, paths)
        return code, summary

    with open(paths['commands'], encoding='utf-8') as handle:
        commands = json.load(handle)
    summary.update(
        both_commands_pass=bool(commands.get('both_commands_pass')),
        start_winding=commands.get('start_winding'),
        warmup=commands.get('warmup'),
        refinement={key: commands.get('refinement', {}).get(key)
                    for key in ('status', 'level_m')},
        task_validation_passed=bool((commands.get('task_validation') or {}).get('passed')),
        plan_written=os.path.exists(paths['task_plan']))
    if summary['both_commands_pass'] and summary['plan_written']:
        summary['status'] = 'ok'
        return 0, summary
    # Every stage ran, and the trajectory still has no executable plan. Name
    # that rather than reporting a stage failure that did not happen.
    # The commands document says 'ok' when the stage merely ran to the end, so
    # it cannot be copied through: a run whose warmup failed validation was
    # summarised as status 'ok' beside failed_stage 'commands'.
    status = commands.get('status')
    summary['status'] = (status if status and status != 'ok'
                         else 'commands did not pass both validations')
    summary['failed_stage'] = STAGE_COMMANDS
    return 1, summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', default=None,
                        help='recording to plan (default: the packaged '
                             'camera_traj.csv); every stage loads it and checks '
                             'the stage before loaded the same bytes')
    parser.add_argument('--waypoints', type=int, default=500,
                        help='RECORDED waypoints; a spin-up adds samples')
    parser.add_argument('--start-index', type=int, default=0,
                        help='first recorded sample of the slice to plan')
    parser.add_argument('--spin-up-s', type=float, default=None,
                        help='prepend a spin-up so the task starts from rest; '
                             "omit to derive it from the recording's starting "
                             'rate and take the shortest that validates')
    parser.add_argument('--home-json', required=True,
                        help='home_choice.json; the plan warms up from its home')
    parser.add_argument('--via-poses', default=None,
                        help='screened poses to route a warmup through when the '
                             'straight move from home collides')
    parser.add_argument('--strict-home-windings', action='store_true',
                        help='judge layer-0 starts only in the winding they were '
                             'stored in, from the outset')
    parser.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    parser.add_argument('--graph-workers', type=int, default=1,
                        help='processes the graph stage validates lifted '
                             'starts with')
    parser.add_argument('--work-dir', required=True)
    parser.add_argument('--summary', default=None,
                        help='where to write the run summary '
                             '(default: pipeline.json in the work directory)')
    args = parser.parse_args(argv)

    try:
        code, summary = run(args)
    except Exception as exc:
        # The summary is still written: a run that raised must not leave
        # nothing behind to classify.
        code = 1
        summary = {'schema_version': SCHEMA_VERSION, 'status': f'pipeline raised '
                   f'{type(exc).__name__}: {exc}', 'traceback': traceback.format_exc(),
                   'stages': [], 'artifacts': {}}
    os.makedirs(args.work_dir, exist_ok=True)
    summary_path = args.summary or os.path.join(args.work_dir, 'pipeline.json')
    with open(summary_path, 'w', encoding='utf-8') as handle:
        json.dump(summary, handle, indent=1)
    ladder = summary.get('spin_up_ladder') or {}
    if ladder.get('derived'):
        print(f"spin-up derived from {ladder['rate_rad_s']:.4f} rad/s: "
              f"rungs {ladder['rungs_s']}, "
              f"tried {[entry['spin_up_s'] for entry in ladder['tried']]}, "
              f"chosen {ladder.get('chosen_s')}")
    for stage in summary['stages']:
        print(f"{stage['stage']:11s} returncode {stage['returncode']} "
              f"in {stage['seconds']:.1f} s")
    if summary.get('backstop'):
        print(f"backstop: {summary['backstop']}")
    if code:
        print(f"pipeline failed at {summary.get('failed_stage')}"
              + (f": {summary['status']}" if summary.get('status') else ''))
    else:
        warmup = summary.get('warmup') or {}
        duration = warmup.get('total_duration_s')
        print(f"plan written to {summary['artifacts']['task_plan']} | "
              f"winding {summary['start_winding']} | "
              f"warmup {'unknown' if duration is None else format(duration, '.2f')} s "
              f"({warmup.get('route_kind')}) | "
              f"refined at {summary['refinement']['level_m']}")
    return code


if __name__ == '__main__':
    raise SystemExit(main())
