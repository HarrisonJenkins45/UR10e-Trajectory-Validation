#!/usr/bin/env python3
"""Certify one slice quickly: a small beam first, broadened only when it fails.

The pipeline certifies a slice by generating every candidate the full seed
bank finds at every waypoint, filtering all of them, validating every lifted
start from the home, running the complete layered DP, and sweeping every
start winding for the warmup. Profiled on a passing 60 s slice (c1-anchor-s4,
601 samples, 987 s): the DP 283 s, the winding sweep 257 s, continuation
seeding 123 s, the self-clearance filter 123 s, interaction seeding 86 s,
start validation 54 s, refinement and both validations 20 s.

Most of that establishes things a first certificate does not need: the
cheapest path, the shortest warmup, every start the home reaches. This
certifier looks for the FIRST path that passes and checks it exactly as the
pipeline would:

  candidates   generated per layer, on demand, from the states the search is
               actually carrying (continuation), the tracker's configuration
               and a declared slice of the seed bank; each solve is cached
  starts       lifted layer-0 states validated from the home nearest first,
               in a worker pool, only until the level's beam is filled; via
               routes only when no direct route is admitted
  search       a beam over lifted states, with the graph's own edge rules
               (LayeredGraph.edge_successors). Half the beam is the cheapest
               states, half those with the most room to their joint limits,
               since a winding running out of range is how long paths die
  broadening   a disconnect at layer j raises the level of the layers in a
               window before j (16, 64, 256, ... layers) and resumes from the
               frontier kept at the window's start; levels widen the beam and
               the seeds. A path that fails validation is banned around the
               located failure and every level is raised
  certificate  the home gate, refinement, a warmup to the path's own start
               winding (the full winding sweep only if that fails), and full
               continuous validation of warmup and task. Identical to the
               pipeline's: only how the path was found differs

A beam is not exhaustive, so a slice this certifier fails is NOT evidence of
anything: its graph is marked exhaustive False and never bounds a start.
A pass is a certificate like any other.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
from ur10e_trajectory_pkg import plan_artifact
from ur10e_trajectory_pkg.fast_beam import (
    BeamSearch, DeadlineExpired, LEVELS, state_key,
)

SCHEMA_VERSION = 1
ENGINE = 'fast_beam'
FAST_START_POLICY = 'validated_lifted_starts_nearest_first'

MAX_VALIDATION_RETRIES = 3


# --------------------------------------------------------------------------
# Real candidates, edges and starts, with caches shared across slices
# --------------------------------------------------------------------------

def wrap_arm(row):
    row = np.asarray(row, dtype=float).copy()
    row[1:] = (row[1:] + np.pi) % (2.0 * np.pi) - np.pi
    return row


def target_key(position, quaternion):
    """A target pose as a cache key: rotation matrix, so q and -q agree."""
    from scipy.spatial.transform import Rotation

    matrix = Rotation.from_quat(quaternion).as_matrix()
    return (tuple(np.round(np.asarray(position, dtype=float), 9)),
            tuple(np.round(matrix.ravel(), 9)))


_WORKER = {}
# Swept-collision answers a worker keeps before starting afresh; bounds memory.
WORKER_SWEPT_CACHE_LIMIT = 200000
# Below these counts a layer's work is done in-process: a pool round trip
# costs more than it saves.
PARALLEL_MIN_SOLVES = 8
PARALLEL_MIN_EDGES = 48


def _worker_validator():
    from ur10e_trajectory_pkg import graph_planner
    return graph_planner._START_WORKER['validator']


def solve_seed(validator, position, quaternion, arm_seed, rail_seed, condition_limit,
               clearance_floor):
    """(row or None, clearance or None): one seeded solve, filtered as graph candidates are."""
    from ur10e_trajectory_pkg import graph_planner

    validator.reset_rng()
    result = validator._solve_waypoint_with_recovery(
        position, quaternion, np.asarray(arm_seed, dtype=float), rail_pos=float(rail_seed),
        check_jump=False, verbose=False)
    if not result['ok']:
        return None, None
    row = wrap_arm(np.concatenate(([result['rail_pos']], result['q_arm'])))
    if graph_planner._arm_condition(validator, row) > condition_limit:
        return None, None
    clearance = validator.self_clearance(row)['distance_m']
    return (row if clearance >= clearance_floor else None), clearance


def _solve_task(task):
    return solve_seed(_worker_validator(), *task)


def _refine_task(task):
    from ur10e_trajectory_pkg import path_refinement

    path, positions, quaternions, dt, rate_hz = task
    return path_refinement.refine_path(_worker_validator(), path, positions, quaternions, dt,
                                       rate_hz=rate_hz)


def _validate_task(task):
    from ur10e_trajectory_pkg import continuous_validator

    path, positions, quaternions, dt, rate_hz = task
    return continuous_validator.validate_task_command(_worker_validator(), path, positions,
                                                      quaternions, dt, rate_hz=rate_hz)


def _edges_task(task):
    from ur10e_trajectory_pkg import graph_planner

    dt, pairs = task
    graphs = _WORKER.setdefault('graphs', {})
    graph = graphs.get(dt)
    if graph is None or len(graph._swept_cache) > WORKER_SWEPT_CACHE_LIMIT:
        graph = graphs[dt] = graph_planner.LayeredGraph(_worker_validator(), {}, 1, dt=dt)
    return [graph.edge_successors(predecessor, row, dt) for predecessor, row in pairs]


class FastContext:
    """What every slice of one recording shares: validator, pool, caches."""

    def __init__(self, urdf, home, via_poses=None, workers=1, validator=None):
        self.urdf = urdf
        self.home = np.asarray(home, dtype=float)
        self.via_poses = via_poses
        self.workers = int(workers)
        if validator is None:
            from ament_index_python.packages import get_package_share_directory

            from ur10e_trajectory_pkg.planning_runtime import make_validator as _validator
            validator = _validator(urdf, get_package_share_directory('ur_description'))
        self.validator = validator
        self.pool = None
        # Set by certify_slice: every wait on the pool ends at it.
        self.deadline = None
        self.solve_cache = {}
        self.clearance_cache = {}
        self.start_cache = {}
        self.cache_stats = {'solve_hits': 0, 'solve_misses': 0, 'edge_queries': 0,
                            'start_hits': 0, 'start_misses': 0}

    def start_pool(self):
        if self.pool is None and self.workers > 1:
            import multiprocessing

            from ur10e_trajectory_pkg import graph_planner
            context = multiprocessing.get_context('fork')
            self.pool = context.Pool(self.workers, initializer=graph_planner._init_start_worker,
                                     initargs=(self.urdf,))
        return self.pool

    def _wait(self, pending):
        """A pool result, or DeadlineExpired when the deadline comes first."""
        import multiprocessing

        if self.deadline is None:
            return pending.get()
        remaining = self.deadline - time.perf_counter()
        if remaining <= 0:
            raise DeadlineExpired()
        try:
            return pending.get(timeout=remaining)
        except multiprocessing.TimeoutError:
            raise DeadlineExpired() from None

    def check_deadline(self):
        """Raise when the budget has run out; called between units of work."""
        if self.deadline is not None and time.perf_counter() >= self.deadline:
            raise DeadlineExpired()

    def map(self, function, tasks, chunksize=1):
        return self._wait(self.pool.map_async(function, tasks, chunksize=chunksize))

    def apply(self, function, arguments):
        return self._wait(self.pool.apply_async(function, arguments))

    def close(self):
        if self.pool is not None:
            self.pool.terminate()
            self.pool.join()
            self.pool = None


class SliceOracle:
    """Candidates, edges and validated starts for one slice's targets."""

    def __init__(self, context, positions, quaternions, dt, tracking=None, timings=None):
        from ur10e_trajectory_pkg import graph_planner, motion_limits

        self.context = context
        self.validator = context.validator
        self.positions = np.asarray(positions, dtype=float)
        self.quaternions = np.asarray(quaternions, dtype=float)
        self.dt = float(dt)
        self.tracking = tracking
        self.graph = graph_planner.LayeredGraph(self.validator, {}, len(self.positions), dt=dt)
        lower, upper = self.validator.robot.qlim
        self.limits = (np.asarray(lower, dtype=float), np.asarray(upper, dtype=float))
        self.condition_limit = graph_planner.GRAPH_CANDIDATE_CONDITION_LIMIT
        self.clearance_floor = motion_limits.SELF_CLEARANCE_FLOOR_M
        self.target_keys = [target_key(p, q) for p, q in zip(self.positions, self.quaternions)]
        self.timings = timings if timings is not None else {}
        self.start_record = {}
        self.layer0_rows = None

    def _time(self, name, started):
        self.timings[name] = self.timings.get(name, 0.0) + time.perf_counter() - started

    def solve_key(self, layer, arm_seed, rail_seed):
        return (self.target_keys[layer], tuple(np.round(np.asarray(arm_seed, dtype=float), 9)),
                round(float(rail_seed), 9))

    def solve_many(self, layer, seeds):
        """Rows for seeds at a layer, from the cache or solved, in seed order.

        The cache key is the target pose itself, not the layer, so a recorded
        sample shared by two slices (their targets agree; see the tests)
        solves once.
        """
        cache = self.context.solve_cache
        keys = [self.solve_key(layer, arm, rail) for arm, rail in seeds]
        todo = [(key, seed) for key, seed in zip(keys, seeds) if key not in cache]
        self.context.cache_stats['solve_hits'] += len(seeds) - len(todo)
        self.context.cache_stats['solve_misses'] += len(todo)
        tasks = [(self.positions[layer], self.quaternions[layer], arm, rail,
                  self.condition_limit, self.clearance_floor) for _, (arm, rail) in todo]
        pool = self.context.start_pool() if len(tasks) >= PARALLEL_MIN_SOLVES else None
        if pool is not None:
            results = self.context.map(_solve_task, tasks,
                                       chunksize=max(1, len(tasks) // (2 * self.context.workers)))
        else:
            results = [solve_seed(self.validator, *task) for task in tasks]
        for (key, _), (row, clearance) in zip(todo, results):
            cache[key] = row
            if row is not None:
                self.context.clearance_cache[state_key(row)] = clearance
        return [cache[key] for key in keys]

    def solve(self, layer, arm_seed, rail_seed):
        return self.solve_many(layer, [(arm_seed, rail_seed)])[0]

    def clearance(self, row):
        key = state_key(row)
        cache = self.context.clearance_cache
        if key not in cache:
            cache[key] = self.validator.self_clearance(row)['distance_m']
        return cache[key]

    def seeds(self, layer, frontier_states, level):
        from ur10e_trajectory_pkg.planning_runtime import WIDE_SEED_BANK_DEG

        spec = LEVELS[level]
        out, seen = [], set()

        def add(arm, rail):
            # Seeds a hundredth apart solve to the same candidate, which the
            # merge tolerance would then discard; do not pay for the solve.
            key = (tuple(np.round(arm, 2)), round(float(rail), 2))
            if key not in seen:
                seen.add(key)
                out.append((np.asarray(arm, dtype=float), float(rail)))

        for state in frontier_states:
            add(wrap_arm(state)[1:], state[0])
        tracking_rail = 1.5
        if self.tracking is not None:
            rails, arms = self.tracking
            tracking_rail = float(rails[layer])
            add(arms[layer], rails[layer])
        for arm_deg in WIDE_SEED_BANK_DEG[:spec['bank_seeds']]:
            for rail in spec['rails']:
                add(np.deg2rad(arm_deg), tracking_rail if rail == 'tracking' else rail)
        return out

    def candidates(self, layer, frontier_states, level):
        from ur10e_trajectory_pkg.candidate_generator import CONTINUATION_MERGE_TOLERANCE, _within

        started = time.perf_counter()
        rows = []
        for row in self.solve_many(layer, self.seeds(layer, frontier_states, level)):
            if row is None:
                continue
            if any(_within(row, other, CONTINUATION_MERGE_TOLERANCE) for other in rows):
                continue
            rows.append(row)
        self._time('candidates_s', started)
        return rows

    @property
    def rail_budget(self):
        return float(self.graph.velocity_limits[0] * self.dt)

    def step_dt(self, layer):
        return self.dt

    def rail_budget_for_layer(self, layer):
        return float(self.graph.velocity_limits[0] * self.step_dt(layer))

    def successors_batch(self, requests, layer):
        """successors() for many pairs, in the worker pool when there are enough."""
        pool = self.context.start_pool() if len(requests) >= PARALLEL_MIN_EDGES else None
        if pool is None:
            return [self.successors(*request, layer) for request in requests]
        started = time.perf_counter()
        workers = self.context.workers
        size = int(math.ceil(len(requests) / float(workers)))
        step_dt = self.step_dt(layer)
        chunks = [(step_dt, [(r[0], r[2]) for r in requests[i:i + size]])
                  for i in range(0, len(requests), size)]
        answers = [answer for chunk in self.context.map(_edges_task, chunks) for answer in chunk]
        self.context.cache_stats['edge_queries'] += len(requests)
        self._time('edges_s', started)
        return answers

    def successors(self, predecessor, predecessor_key, row, row_key, layer):
        """The graph's own edge rules. Not cached across layers: measured on a
        601-layer slice, 3% of edge queries repeated, against 2 M held."""
        started = time.perf_counter()
        self.graph.begin_layer(layer)
        out = self.graph.edge_successors(predecessor, row, self.step_dt(layer))
        self.context.cache_stats['edge_queries'] += 1
        self._time('edges_s', started)
        return out

    def _layer0_rows(self):
        """Layer-0 candidates from the widest declared seeds, at every level.

        Where the task starts decides everything after it, and 60-odd solves
        at one layer cost a fraction of a second, so starts are never narrow.
        """
        if self.layer0_rows is None:
            from ur10e_trajectory_pkg.candidate_generator import CONTINUATION_MERGE_TOLERANCE, _within

            rows = []
            for row in self.solve_many(0, self.seeds(0, [], len(LEVELS) - 1)):
                if row is not None and not any(
                        _within(row, other, CONTINUATION_MERGE_TOLERANCE) for other in rows):
                    rows.append(row)
            self.layer0_rows = rows
        return self.layer0_rows

    def _evaluate(self, stage, states):
        """Route verdicts for lifted states, cached, in the pool when there is one."""
        from ur10e_trajectory_pkg import graph_planner

        context = self.context
        via = context.via_poses if stage == 'via' else None
        keys = [(stage, state_key(s)) for s in states]
        todo = [(key, s) for key, s in zip(keys, states) if key not in context.start_cache]
        context.cache_stats['start_hits'] += len(states) - len(todo)
        context.cache_stats['start_misses'] += len(todo)
        if todo:
            tasks = [(stage, s, context.home, via, 200.0) for _, s in todo]
            pool = context.start_pool()
            if pool is not None and len(tasks) > 1:
                results = context.map(graph_planner._evaluate_start_task, tasks)
            else:
                results = [graph_planner._evaluate_start(self.validator, *task) for task in tasks]
            for (key, _), result in zip(todo, results):
                context.start_cache[key] = result
        return [context.start_cache[key] for key in keys]

    def start_frontier(self, level):
        from ur10e_trajectory_pkg import graph_planner

        started = time.perf_counter()
        spec = LEVELS[level]
        rows = self._layer0_rows()
        lifted = graph_planner.lifted_start_states(self.validator, rows, self.context.home)
        home_arm = self.context.home[1:]
        order = sorted(range(len(lifted)), key=lambda i: (
            float(np.max(np.abs(lifted[i]['state'][1:] - home_arm))), lifted[i]['candidate'],
            tuple(lifted[i]['winding'])))
        lifted = [lifted[i] for i in order]
        admit, cap = spec['start_admit'], spec['start_eval']
        batch = max(self.context.workers, 1) * 2
        record = {'policy': FAST_START_POLICY, 'home': self.context.home.tolist(),
                  'layer_0_candidates': len(rows), 'lifted_states': len(lifted),
                  'winding_aware': True, 'level': level, 'attempts': [], 'reasons': {},
                  'via_poses_offered': (0 if self.context.via_poses is None
                                        else len(self.context.via_poses))}
        admitted, routes = [], {'direct': 0, 'via': 0}
        stages = ['direct'] + (['via'] if self.context.via_poses is not None else [])
        for stage in stages:
            if admitted:
                break
            evaluated = 0
            pool = lifted if cap is None else lifted[:cap]
            for begin in range(0, len(pool), batch):
                if admit is not None and len(admitted) >= admit:
                    break
                self.context.check_deadline()
                chunk = pool[begin:begin + batch]
                for entry, (ok, reason) in zip(chunk, self._evaluate(stage, [e['state'] for e in chunk])):
                    evaluated += 1
                    if ok:
                        admitted.append(dict(entry, route_kind=stage))
                        routes[stage] += 1
                    else:
                        record['reasons'][reason] = record['reasons'].get(reason, 0) + 1
            record['attempts'].append({'seeds': 'direct' if stage == 'direct' else 'direct+via',
                                       'evaluated': evaluated, 'admitted': routes[stage]})
        record.update(kept=len(admitted), routes=routes,
                      candidates_with_a_start=len({e['candidate'] for e in admitted}),
                      removed=len(lifted) - len(admitted))
        self.start_record = record
        self.admitted = admitted
        self._time('starts_s', started)
        return [e['state'] for e in (admitted if admit is None else admitted[:admit])]


# --------------------------------------------------------------------------
# Certifying a found path, exactly as the commands stage does
# --------------------------------------------------------------------------

def fast_start_winding(validator, home, path, via_poses=None, check=None):
    """The path's own start winding first; the full winding sweep only if it fails.

    check, when given, is called before each winding is planned and raises
    DeadlineExpired when the budget is gone: the full sweep plans and
    validates a route per legal winding and is the longest step here.
    """
    from ur10e_trajectory_pkg import continuous_validator, home_pose, warmup

    for option in home_pose.start_windings(validator, path):
        if any(option['winding']):
            continue
        route = warmup.plan_route(validator, home, option['path'][0], via_poses=via_poses)
        if route['status'] != warmup.OK:
            break
        report = continuous_validator.validate_route(validator, route)
        summary = [{'winding': option['winding'], 'status': route['status'],
                    'duration_s': route.get('total_duration_s'),
                    'route_kind': route.get('route_kind'),
                    'segments': len(route.get('segments', [])),
                    'validated': bool(report['passed']),
                    'validation_failures': report['failures']}]
        if report['passed']:
            return (dict(option, warmup=route, warmup_validation=report), summary,
                    'graph_start_winding')
        break
    best, summary = sweep_start_windings(validator, home, path, via_poses, check)
    return best, summary, 'full_winding_search'


def sweep_start_windings(validator, home, path, via_poses=None, check=None):
    """home_pose.choose_start_winding, with a deadline check between windings.

    The same order and the same verdict: routes are planned for every legal
    winding, ranked by duration, and validated in that order until one passes.
    """
    from ur10e_trajectory_pkg import continuous_validator, home_pose, warmup

    options = []
    for option in home_pose.start_windings(validator, path):
        if check is not None:
            check()
        result = warmup.plan_route(validator, home, option['path'][0], via_poses=via_poses)
        options.append(dict(option, warmup=result, validation=None))
    planned = sorted((o for o in options if o['warmup']['status'] == warmup.OK),
                     key=lambda o: (o['warmup']['total_duration_s'],
                                    sum(abs(w) for w in o['winding']),
                                    tuple(o['winding'])))
    best = None
    for option in planned:
        if check is not None:
            check()
        option['validation'] = continuous_validator.validate_route(validator, option['warmup'])
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


def certify_path(validator, choice, graph, path, positions, quaternions, dt, via_poses,
                 rate_hz=200.0, deadline=None, context=None):
    """Home gate, refinement, warmup and both validations, as home_pose commands."""
    from ur10e_trajectory_pkg import continuous_validator, home_pose, path_refinement

    home = np.asarray(choice['chosen']['configuration'], dtype=float)
    timings = {}
    started = time.perf_counter()
    gate = home_pose.home_gate(validator, choice, graph)
    timings['home_gate_s'] = time.perf_counter() - started
    document = {'schema_version': home_pose.SCHEMA_VERSION, 'engine': ENGINE,
                'home': home.tolist(), 'placement': graph.get('placement', 'nominal'),
                'home_gate': gate, 'recording': graph.get('recording')}
    if not gate['passed']:
        document.update(status=gate['status'], both_commands_pass=False, timings=timings)
        return document, None
    if deadline is not None and time.perf_counter() >= deadline:
        raise DeadlineExpired()
    started = time.perf_counter()
    pool = None if context is None else context.start_pool()
    if pool is not None:
        # In a worker, so the deadline can end the wait: refinement is the one
        # step of several seconds, and in-process it could not be interrupted.
        refinement = context.apply(_refine_task, ((np.asarray(path, dtype=float), positions,
                                                   quaternions, dt, rate_hz),))
    else:
        refinement = path_refinement.refine_path(validator, path, positions, quaternions, dt,
                                                 rate_hz=rate_hz)
    timings['refinement_s'] = time.perf_counter() - started
    document.update(
        refinement={k: refinement[k] for k in ('status', 'level_m', 'attempts')},
        refinement_meets_acceptance=path_refinement.meets_acceptance(refinement))
    if refinement['status'] != 'refined':
        document.update(status='refinement_failed', both_commands_pass=False, timings=timings)
        return document, None
    if deadline is not None and time.perf_counter() >= deadline:
        raise DeadlineExpired()
    started = time.perf_counter()
    best, summary, how = fast_start_winding(validator, home, refinement['path'], via_poses,
                                            check=None if context is None else context.check_deadline)
    timings['winding_and_warmup_s'] = time.perf_counter() - started
    document.update(winding_options=summary, winding_search=how)
    if best is None:
        document.update(status='no valid warmup to any legal winding of the start',
                        both_commands_pass=False, timings=timings)
        return document, None
    started = time.perf_counter()
    reused = (not any(best['winding']) and refinement.get('validation') is not None
              and np.array_equal(np.asarray(best['path']), np.asarray(refinement['path']))
              and float(rate_hz) == float(path_refinement.REFINEMENT_RATE_HZ))
    if reused:
        # The same function on the same path at the same rate: refinement
        # accepted this path on exactly this report.
        task_report = refinement['validation']
    else:
        if deadline is not None and time.perf_counter() >= deadline:
            raise DeadlineExpired()
        if pool is not None:
            task_report = context.apply(_validate_task, ((np.asarray(best['path'], dtype=float),
                                                          positions, quaternions, dt, rate_hz),))
        else:
            task_report = continuous_validator.validate_task_command(
                validator, best['path'], positions, quaternions, dt, rate_hz=rate_hz)
    timings['task_validation_s'] = time.perf_counter() - started
    warmup_report = best['warmup_validation']
    document.update(
        status='ok', start_winding=best['winding'], command_2_start=best['path'][0].tolist(),
        warmup=plan_artifact.route_record(best['warmup']), warmup_validation=warmup_report,
        task_validation=task_report, task_validation_reused_from_refinement=bool(reused),
        both_commands_pass=bool(warmup_report['passed'] and task_report['passed']),
        timings=timings)
    return document, best


# --------------------------------------------------------------------------
# One slice
# --------------------------------------------------------------------------

def _dump(document, path):
    from ur10e_trajectory_pkg.home_pose import _dump as dump
    dump(document, path)


def located_layer(commands, dt):
    from ur10e_trajectory_pkg.section_contract import located_failure_time

    located = located_failure_time(commands)
    return None if located is None else int(np.floor(located / float(dt) + 1e-9))


def certify_slice(context, choice, csv_path, start, samples, spin_up_s, work_dir,
                  deadline=None, levels=LEVELS):
    """Certify [start, start + samples) and write pipeline-compatible artifacts.

    Returns the pipeline-style summary. status 'ok' is a certificate;
    'deadline' says the budget ran out, which says nothing about the slice.
    """
    from ur10e_trajectory_pkg import graph_planner, home_pose, motion_limits
    from ur10e_trajectory_pkg.candidate_generator import tracking_reference_path
    from ur10e_trajectory_pkg.configurations import LEGACY_MATLAB_START_Q
    from ur10e_trajectory_pkg.planning_runtime import load_trajectory, recording_record

    os.makedirs(work_dir, exist_ok=True)
    paths = {name: os.path.join(work_dir, f'{name}.json')
             for name in ('graph', 'commands', 'task_plan')}
    started = time.perf_counter()
    stages, timings = [], {}
    summary = {'schema_version': 1, 'engine': ENGINE, 'placement': 'nominal',
               'csv': csv_path, 'start_index': int(start), 'waypoints': int(samples),
               'spin_up_s': spin_up_s, 'home_json': None, 'artifacts': paths,
               'stages': stages, 'strict_home_windings': False,
               'spin_up_ladder': {'derived': False, 'rungs_s': [spin_up_s],
                                  'tried': [{'spin_up_s': spin_up_s}], 'chosen_s': None}}
    validator = context.validator
    context.deadline = deadline
    positions, quaternions, dt, metadata = load_trajectory(
        csv_path, samples, with_metadata=True, spin_up_s=spin_up_s, start_index=start)
    tracking = tracking_reference_path(validator, positions, quaternions, dt,
                                       LEGACY_MATLAB_START_Q)
    stages.append({'stage': 'targets_and_tracking', 'seconds': time.perf_counter() - started})
    oracle = SliceOracle(context, positions, quaternions, dt,
                         tracking=tracking, timings=timings)
    search = BeamSearch(oracle, len(positions), levels=levels, deadline=deadline)
    recording = recording_record(csv_path)
    commands, best, result = None, None, None
    validations = []

    def graph_document(result):
        path = result['path']
        disconnected = result['first_disconnected_layer']
        document = {
            'schema_version': graph_planner.SCHEMA_VERSION, 'build': 'generator',
            'layers': len(positions), 'recorded_waypoints': int(samples),
            'start_index': int(start), 'end_index': metadata['end_index'],
            'recorded_start_time_s': metadata['recorded_start_time_s'],
            'mount': metadata['mount'], 'spin_up': metadata.get('spin_up'),
            'start_mode': 'free', 'start_policy': FAST_START_POLICY,
            'placement': 'nominal', 'recording': recording,
            'placement_RG': np.asarray(metadata['placement_RG']).tolist(),
            'candidate_filters': {'home_reachable': oracle.start_record,
                                  'condition': {'max_condition': oracle.condition_limit},
                                  'self_clearance': {'floor_m': oracle.clearance_floor}},
            'search': {'engine': ENGINE, 'exhaustive': False,
                       'levels': [dict(spec) for spec in levels],
                       'final_levels': sorted(set(search.levels)),
                       'escalations': search.escalations, 'stats': search.stats,
                       'validation_attempts': validations,
                       'note': 'a beam search: a disconnect here bounds nothing'},
            'complete_path': bool(result['complete']),
            'first_disconnected_layer': disconnected,
            'path_cost': result['cost'],
            'path': None if path is None else [np.asarray(c).tolist() for c in path],
            'q_start': None,
        }
        if path is not None:
            document.update(
                chosen_start=np.asarray(path[0]).tolist(),
                path_max_condition=max(graph_planner._arm_condition(validator, q) for q in path),
                path_min_self_clearance_m=min(oracle.clearance(q) for q in path),
                entry_state=graph_planner.entry_state_report(
                    path, dt, oracle.graph.velocity_limits, motion_limits.acceleration_vector()))
            chosen = next((e for e in getattr(oracle, 'admitted', [])
                           if state_key(e['state']) == state_key(path[0])), None)
            oracle.start_record['chosen_start'] = (
                None if chosen is None else {'candidate': chosen['candidate'],
                                             'winding': chosen['winding'],
                                             'route_kind': chosen['route_kind']})
        else:
            last = result.get('last_connected_layer')
            document.update(last_connected_layer=last,
                            partial_path=(None if result.get('partial_path') is None else
                                          [np.asarray(c).tolist() for c in result['partial_path']]))
        return document

    try:
        while True:
            search_started = time.perf_counter()
            result = search.run()
            stages.append({'stage': 'search', 'seconds': time.perf_counter() - search_started})
            graph = graph_document(result)
            _dump(graph, paths['graph'])
            if not result['complete']:
                summary.update(status=f"no complete path under the fast search: first "
                                      f"disconnected layer {result['first_disconnected_layer']}",
                               failed_stage='graph')
                break
            certify_started = time.perf_counter()
            commands, best = certify_path(validator, choice, graph, graph['path'], positions,
                                          quaternions, dt, context.via_poses, deadline=deadline,
                                          context=context)
            stages.append({'stage': 'certify', 'seconds': time.perf_counter() - certify_started,
                           'timings': commands.get('timings')})
            validations.append({'status': commands['status'],
                                'both_commands_pass': commands['both_commands_pass']})
            _dump(commands, paths['commands'])
            if commands['both_commands_pass']:
                _dump(plan_artifact.task_plan(context.home, best['path'], graph, paths['graph'],
                                          best['winding'], route=best['warmup'],
                                          recording=recording), paths['task_plan'])
                summary.update(status='ok', both_commands_pass=True,
                               start_winding=best['winding'],
                               warmup=plan_artifact.route_record(best['warmup']),
                               refinement={k: commands['refinement'].get(k)
                                           for k in ('status', 'level_m')},
                               task_validation_passed=True, plan_written=True)
                summary['spin_up_ladder']['chosen_s'] = spin_up_s
                break
            failure_layer = located_layer(commands, dt)
            if (len(validations) > MAX_VALIDATION_RETRIES
                    or not search.reject_path(result['path'], failure_layer)):
                summary.update(status=commands['status'] if commands['status'] != 'ok'
                               else 'commands did not pass both validations',
                               failed_stage='commands')
                break
    except DeadlineExpired:
        summary.update(status='inconclusive: the time budget expired', failed_stage='deadline',
                       inconclusive=True)
        if result is not None and not os.path.exists(paths['graph']):
            _dump(graph_document(result), paths['graph'])
    summary['seconds'] = time.perf_counter() - started
    summary['search'] = {'escalations': search.escalations, 'stats': search.stats,
                         'final_levels': sorted(set(search.levels)), 'timings': timings,
                         'cache': dict(context.cache_stats),
                         'validation_attempts': validations}
    _dump(summary, os.path.join(work_dir, 'pipeline.json'))
    return summary


def main(argv=None):
    """One slice, as the pipeline command would be run: for section_planner --engine fast."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--csv', required=True)
    parser.add_argument('--start-index', type=int, default=0)
    parser.add_argument('--waypoints', type=int, required=True)
    parser.add_argument('--spin-up-s', type=float, required=True)
    parser.add_argument('--home-json', required=True)
    parser.add_argument('--via-poses', default=None)
    parser.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    parser.add_argument('--graph-workers', type=int, default=1)
    parser.add_argument('--time-budget-s', type=float, default=None)
    parser.add_argument('--work-dir', required=True)
    args = parser.parse_args(argv)
    started = time.perf_counter()
    with open(args.home_json, encoding='utf-8') as handle:
        choice = json.load(handle)
    via = None
    if args.via_poses:
        with open(args.via_poses, encoding='utf-8') as handle:
            via = [e['configuration'] if isinstance(e, dict) else e for e in json.load(handle)]
    context = FastContext(args.urdf, choice['chosen']['configuration'], via, args.graph_workers)
    try:
        deadline = None if args.time_budget_s is None else started + args.time_budget_s
        summary = certify_slice(context, choice, args.csv, args.start_index, args.waypoints,
                                args.spin_up_s, args.work_dir, deadline=deadline)
    finally:
        context.close()
    summary['home_json'] = args.home_json
    summary['via_poses'] = args.via_poses
    _dump(summary, os.path.join(args.work_dir, 'pipeline.json'))
    print(f"fast certify [{args.start_index}, {args.start_index + args.waypoints}): "
          f"{summary['status']} in {summary['seconds']:.1f} s", flush=True)
    return 0 if summary.get('status') == 'ok' else 1


if __name__ == '__main__':
    sys.exit(main())
