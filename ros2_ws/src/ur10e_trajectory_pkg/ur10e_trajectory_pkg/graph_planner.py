#!/usr/bin/env python3
"""Layered graph over per-waypoint candidates and legal lifted coordinates.

State is the full LIFTED configuration, equivalently
(layer, canonical candidate, winding vector). The winding has to persist in
the state rather than sitting on the edge: which lifts a successor can reach
depends on where the predecessor actually is, so an edge-local lift would
lose exactly the information the next transition needs.

Edges are built cheapest-check-first, because collision queries dominate:

  1. rail displacement            one subtraction
  2. reachable lifts and velocity closed form per joint
  3. swept collision              a handful of PyBullet queries
  4. cost

The graph selects a continuous path from independently generated candidates.
Warmup reachability filters layer-0 lifted starts. Acceleration, jerk and
between-waypoint tracking are checked by command certification after the path
is chosen, not treated as first-order graph edges.
"""
import argparse
import json
import sys

import numpy as np

from ur10e_trajectory_pkg import environment, motion_limits
from ur10e_trajectory_pkg.configurations import (
    ARM_SLICE,
    PERIODIC_JOINTS,
    RAIL_INDEX,
)
from ur10e_trajectory_pkg.joint_coordinates import reachable_feasible_lifts

# 2: records the effective velocity limits and the greedy reference cost.
SCHEMA_VERSION = 2

# Duration of the approach from q_start to the first waypoint. This is NOT the
# 0.1 s trajectory step: the arm has to get to the trajectory's start, and
# charging that move at the per-waypoint rate would reject it as a velocity
# violation for no physical reason.
TRANSITION_SECONDS = 2.0

# Interior samples per edge for swept collision. The endpoints are already
# known collision-free as nodes, so this asks only about the space between.
SWEPT_SAMPLES = 3
# Rounding of a cached sweep's key. Far below any collision margin, and above
# the round-off of wrapping a joint by whole turns.
SWEPT_CACHE_DECIMALS = 9

# States are keyed on rounded coordinates so that two arithmetically distinct
# routes to the same configuration share a state instead of splitting the
# frontier.
STATE_DECIMALS = 6


def _arm_condition(validator, configuration):
    singular = np.linalg.svd(validator.compute_arm_jacobian(configuration),
                             compute_uv=False)
    return float(singular[0] / singular[-1]) if singular[-1] > 1e-12 else float('inf')


def state_key(configuration):
    return tuple(np.round(np.asarray(configuration, dtype=float), STATE_DECIMALS))


# Declared before any placement's executed path was measured: the task counts
# as starting at rest if every joint's entry velocity is within this fraction
# of its velocity limit AND its entry acceleration within this fraction of its
# acceleration limit. A recorded check here; a constraint only if needed.
AT_REST_TOLERANCE_FRACTION = 0.02


def entry_state_report(path, dt, velocity_limits, acceleration_limits,
                       tolerance=AT_REST_TOLERANCE_FRACTION):
    """Per-joint entry velocity and acceleration of an executed path.

    From the first three states, as PCHIP derives them. The warmup ends at
    rest, so a joint entering above tolerance meets it with a step. With a
    spin-up the task's tool motion at the first waypoints is essentially zero,
    so any entry motion is the redundancy -- the rail sliding while the tool
    holds still -- which is a planner choice rather than a task demand.
    """
    from ur10e_trajectory_pkg.configurations import JOINT_NAMES
    from ur10e_trajectory_pkg.robot_checks import entry_state_from_prefix

    velocity, acceleration = entry_state_from_prefix(np.asarray(path[:3], float), dt)
    velocity_ratio = np.abs(velocity) / np.asarray(velocity_limits, float)
    acceleration_ratio = np.abs(acceleration) / np.asarray(acceleration_limits, float)
    worst = np.maximum(velocity_ratio, acceleration_ratio)
    dominant = int(np.argmax(worst))
    return {
        'velocity': velocity.tolist(),
        'acceleration': acceleration.tolist(),
        'velocity_ratio': velocity_ratio.tolist(),
        'acceleration_ratio': acceleration_ratio.tolist(),
        'max_velocity_ratio': float(np.max(velocity_ratio)),
        'max_acceleration_ratio': float(np.max(acceleration_ratio)),
        'dominant_joint': JOINT_NAMES[dominant],
        'dominant_kind': ('velocity' if velocity_ratio[dominant] >= acceleration_ratio[dominant]
                          else 'acceleration'),
        'tolerance_fraction': tolerance,
        'at_rest': bool(np.all(velocity_ratio <= tolerance)
                        and np.all(acceleration_ratio <= tolerance)),
        'joints_over_tolerance': [JOINT_NAMES[i] for i in range(len(worst))
                                  if worst[i] > tolerance],
    }


def path_cost(q_start, path, velocity_limits, dt,
              transition_seconds=TRANSITION_SECONDS):
    """q_start None means a free start: the path's own first state, no
    transition edge charged."""
    if q_start is None:
        return (_path_cost_from(path[0], path[1:], velocity_limits, dt)
                if len(path) > 1 else 0.0)
    return _path_cost_fixed(q_start, path, velocity_limits, dt,
                            transition_seconds)


def _path_cost_from(start, path, velocity_limits, dt):
    total, previous = 0.0, np.asarray(start, dtype=float)
    for configuration in path:
        total += edge_cost(previous, configuration, velocity_limits, dt)
        previous = np.asarray(configuration, dtype=float)
    return total


def _path_cost_fixed(q_start, path, velocity_limits, dt,
                     transition_seconds=TRANSITION_SECONDS):
    """Total edge cost of a path under the graph's own cost function.

    What makes a greedy path and a graph path comparable: the same limits and
    the same normalisation. Costs computed under different velocity limits are
    NOT comparable, since the limits are the normalisation.
    """
    total, previous = 0.0, np.asarray(q_start, dtype=float)
    for index, configuration in enumerate(path):
        step = transition_seconds if index == 0 else dt
        total += edge_cost(previous, configuration, velocity_limits, step)
        previous = np.asarray(configuration, dtype=float)
    return total


def edge_cost(predecessor, successor, velocity_limits, dt):
    """Normalised squared motion, each joint against its own budget.

    Dividing by velocity_limit * dt puts the rail's metres and the arm's
    radians on one scale, so the sum is meaningful rather than an accidental
    unit mixture.
    """
    delta = np.asarray(successor) - np.asarray(predecessor)
    budget = np.asarray(velocity_limits) * dt
    return float(np.sum((delta / budget) ** 2))


class LayeredGraph:
    def __init__(self, validator, candidates, num_layers,
                 dt=0.1, transition_seconds=TRANSITION_SECONDS,
                 velocity_limits=None, swept_samples=SWEPT_SAMPLES):
        self.validator = validator
        self.candidates = candidates
        self.num_layers = num_layers
        self.dt = dt
        self.transition_seconds = transition_seconds
        self.swept_samples = swept_samples

        self.limits = (validator.robot.qlim[0], validator.robot.qlim[1])
        # The validator's own limits unless told otherwise: the URDF's
        # per-joint arm values and the rail's capped value. A uniform arm
        # figure here once made the graph prune, lift and cost against a
        # different robot than the tracker it is compared with.
        self.velocity_limits = (
            np.asarray(validator.velocity_limits, dtype=float)
            if velocity_limits is None
            else np.asarray(velocity_limits, dtype=float))
        self.counters = {
            'pairs_considered': 0,
            'pruned_rail': 0,
            'pruned_velocity_or_lift': 0,
            'pruned_swept_collision': 0,
            'edges_built': 0,
        }
        self._swept_cache, self._swept_cache_layer = {}, None
        self.swept_cache_stats = {'queries': 0, 'hits': 0}

    def swept_collision(self, predecessor, successor):
        """Sample between two configurations, both already valid as nodes.

        Linear in joint space, which is what the PCHIP interpolation
        approximates between adjacent waypoints. Last in the pruning order
        because it is the only step that costs a physics query.

        Answers are cached by the predecessor with its arm joints wrapped into
        [-pi, pi) and by the step. Two edges that differ only by the same
        whole turns at both ends sweep the same physical configurations, and a
        lifted frontier holds many such copies: at layer 1708 of a
        5000-waypoint trial eight states were one configuration in eight
        windings. The cache holds one layer at a time.
        """
        predecessor = np.asarray(predecessor, dtype=float)
        successor = np.asarray(successor, dtype=float)
        wrapped = predecessor.copy()
        wrapped[ARM_SLICE] = (wrapped[ARM_SLICE] + np.pi) % (2.0 * np.pi) - np.pi
        key = (tuple(np.round(wrapped, SWEPT_CACHE_DECIMALS)),
               tuple(np.round(successor - predecessor, SWEPT_CACHE_DECIMALS)))
        self.swept_cache_stats['queries'] += 1
        cached = self._swept_cache.get(key)
        if cached is not None:
            self.swept_cache_stats['hits'] += 1
            return cached
        collides = False
        for fraction in np.linspace(0.0, 1.0, self.swept_samples + 2)[1:-1]:
            between = predecessor + fraction * (successor - predecessor)
            if self.validator.check_all_collisions(between):
                collides = True
                break
        self._swept_cache[key] = collides
        return collides

    def begin_layer(self, layer):
        """Drop cached sweeps from another layer; they cannot recur."""
        if layer != self._swept_cache_layer:
            self._swept_cache.clear()
            self._swept_cache_layer = layer

    def successors(self, predecessor, layer, dt):
        """Every reachable lifted successor at this layer, with its cost."""
        self.begin_layer(layer)
        out = []
        for candidate in self.candidates[layer]:
            out.extend(self.edge_successors(predecessor, candidate, dt))
        return out

    def edge_successors(self, predecessor, candidate, dt):
        """The lifted successors one candidate offers a predecessor, with costs.

        The rules of one edge, cheapest check first. Independent of the layer
        and of every other candidate, so a caller building its own frontier
        (fast_certify) applies exactly the rules this graph does.
        """
        budget = self.velocity_limits * dt
        self.counters['pairs_considered'] += 1

        # 1. rail displacement, the cheapest possible rejection
        if abs(candidate[RAIL_INDEX] - predecessor[RAIL_INDEX]) > budget[RAIL_INDEX]:
            self.counters['pruned_rail'] += 1
            return []

        # 2. reachable lifts, closed form per joint
        lifts = reachable_feasible_lifts(
            candidate[ARM_SLICE], predecessor[ARM_SLICE],
            (self.limits[0][ARM_SLICE], self.limits[1][ARM_SLICE]),
            PERIODIC_JOINTS[ARM_SLICE],
            self.velocity_limits[ARM_SLICE], dt)
        if not lifts:
            self.counters['pruned_velocity_or_lift'] += 1
            return []

        out = []
        for arm in lifts:
            successor = np.concatenate(([candidate[RAIL_INDEX]], arm))

            # 3. swept collision, the only physics query
            if self.swept_collision(predecessor, successor):
                self.counters['pruned_swept_collision'] += 1
                continue

            # 4. cost
            self.counters['edges_built'] += 1
            out.append((successor,
                        edge_cost(predecessor, successor,
                                  self.velocity_limits, dt)))
        return out

    def start_states(self, lifts=False):
        """Every layer-0 candidate as a start, optionally in every legal winding.

        Canonical only by default. Each winding of a start is a distinct graph
        state all the way down, so including them all can multiply the
        frontier by the number of lifts (up to 2 per periodic arm joint).
        """
        from itertools import product

        from ur10e_trajectory_pkg.joint_coordinates import feasible_lifts

        states = []
        for candidate in self.candidates[0]:
            candidate = np.asarray(candidate, dtype=float)
            if not lifts:
                states.append(candidate)
                continue
            options = [feasible_lifts(value, self.limits[0][ARM_SLICE][i],
                                      self.limits[1][ARM_SLICE][i],
                                      PERIODIC_JOINTS[ARM_SLICE][i])
                       for i, value in enumerate(candidate[ARM_SLICE])]
            for arm in product(*options):
                states.append(np.concatenate(([candidate[RAIL_INDEX]], arm)))
        return states

    def shortest_path(self, q_start=None, start_lifts=False, start_states=None):
        """Dynamic programming over the layered DAG.

        Edges only ever go from layer i to i+1, so one sweep per layer keeping
        the best cost per state is exact. Only the current frontier is held,
        which bounds memory regardless of depth.

        q_start None is a FREE start: a common source connects to every
        layer-0 candidate at zero cost, so the planner chooses where the task
        begins, and the path's first state is that choice. The arm is brought
        there by a separate warmup command, so no transition edge exists and
        the path has num_layers states. With a q_start, the path begins there
        and the first edge is the transition, as before.

        start_states, when given, are the free start's states exactly as they
        are -- lifted configurations the home was validated against -- in
        place of the layer-0 candidates.
        """
        first_empty = None
        if q_start is None:
            frontier = {}
            seeds = (self.start_states(start_lifts) if start_states is None
                     else [np.asarray(state, dtype=float) for state in start_states])
            for state in seeds:
                frontier.setdefault(state_key(state), (0.0, state, None))
            self.counters['start_states'] = len(frontier)
            history = [frontier]
            if not frontier:
                return None, 0, history
            layers = range(1, self.num_layers)
        else:
            frontier = {state_key(q_start): (0.0, np.asarray(q_start, float), None)}
            history = [frontier]
            layers = range(self.num_layers)

        for layer in layers:
            dt = self.transition_seconds if (layer == 0 and q_start is not None) else self.dt
            nxt = {}
            for _, (cost, configuration, _) in frontier.items():
                for successor, step in self.successors(configuration, layer, dt):
                    key = state_key(successor)
                    total = cost + step
                    if key not in nxt or total < nxt[key][0]:
                        nxt[key] = (total, successor, state_key(configuration))
            if not nxt:
                first_empty = layer
                break
            frontier = nxt
            history.append(frontier)

        self.last_history = history
        if first_empty is not None:
            return None, first_empty, history
        return best_partial_path(history), None, history


def best_partial_path(history):
    """The cheapest path through the last layer the search reached.

    For a complete search that is the shortest path. For a disconnected one it
    is the best path through the last connected layer: the prefix a section
    workflow can still certify, rather than nothing. Returns (path, cost), or
    None when not even a start state exists.
    """
    if not history or not history[-1]:
        return None
    final = min(history[-1].items(), key=lambda item: item[1][0])
    path, key = [final[1][1]], final[1][2]
    for level in range(len(history) - 2, -1, -1):
        _, configuration, parent = history[level][key]
        path.append(configuration)
        key = parent
    return list(reversed(path)), final[1][0]


# Conditioning margin on graph candidates. The continuous path's gate stays at
# 50; candidates are held to half of it so the chosen branch leaves
# refinement room to smooth without crossing the gate. Measured before
# adopting: at nominal and coupled_14 every waypoint keeps candidates at or
# below 25 (fewest 20 and 30), with the best near 6, so the tumble does not
# force a near-singular path -- the velocity-only cost chose one.
GRAPH_CANDIDATE_CONDITION_LIMIT = 25.0


def load_candidates(path, num_layers, max_condition=None):
    """Canonical generated configurations per layer.

    max_condition drops candidates whose recorded arm condition number
    exceeds it. The count removed per layer is returned in
    document['condition_filter'].
    """
    with open(path, encoding='utf-8') as handle:
        document = json.load(handle)
    layers, removed, considered = [], [], []
    for index in range(num_layers):
        entries = document['candidates'].get(str(index), [])
        rows, dropped, seen = [], 0, 0
        for entry in entries:
            seen += 1
            if (max_condition is not None
                    and entry.get('arm_condition_number', 0.0) > max_condition):
                dropped += 1
                continue
            rows.append(np.array([entry['rail_position'],
                                  *entry['q_arm_canonical']], dtype=float))
        layers.append(rows)
        removed.append(dropped)
        considered.append(seen)
    # Generated candidates per layer, before any filter. Recorded here, where
    # the document is already loaded, so a disconnected layer can be told
    # apart from an empty one without re-reading a candidates file that runs
    # to gigabytes at full recording length.
    document['condition_filter'] = {'max_condition': max_condition,
                                    'candidates_per_layer': considered,
                                    'removed_per_layer': removed,
                                    'removed_total': int(sum(removed))}
    return layers, document


def best_condition_lower_bound(document, num_layers):
    """No path can peak below this: the worst layer's best candidate.

    A lower bound on the best achievable worst candidate condition, from the
    unfiltered candidates. It does not prove a path achieving it is connected.
    """
    bests = [min((e.get('arm_condition_number', np.inf)
                  for e in document['candidates'].get(str(i), [])), default=np.inf)
             for i in range(num_layers)]
    worst = int(np.argmax(bests))
    return {'value': float(bests[worst]), 'layer': worst}


def start_winding_options(validator, configuration, home):
    """Every legal 2*pi winding of a candidate start, nearest the home first.

    The robot does not care which winding a candidate was stored in: another
    winding is the same physical start and the same tool pose, and the
    command path shifts the whole path to match when it picks one. Ordered by
    how far the arm joints sit from the home, so the winding tried first is
    the one most likely to have a short straight move.
    """
    import itertools

    from ur10e_trajectory_pkg.configurations import ARM_SLICE, PERIODIC_JOINTS
    from ur10e_trajectory_pkg.joint_coordinates import TWO_PI, feasible_lifts

    configuration = np.asarray(configuration, dtype=float)
    home = np.asarray(home, dtype=float)
    lower, upper = validator.robot.qlim
    per_joint = [feasible_lifts(value, lower[ARM_SLICE][index],
                                upper[ARM_SLICE][index],
                                PERIODIC_JOINTS[ARM_SLICE][index])
                 for index, value in enumerate(configuration[ARM_SLICE])]
    options = []
    for arm in itertools.product(*per_joint):
        wound = configuration.copy()
        wound[ARM_SLICE] = np.asarray(arm, dtype=float)
        turns = np.rint((wound[ARM_SLICE] - configuration[ARM_SLICE])
                        / TWO_PI).astype(int)
        reach = float(np.max(np.abs(wound[ARM_SLICE] - home[ARM_SLICE])))
        options.append((reach, int(np.sum(np.abs(turns))),
                        tuple(int(turn) for turn in turns), wound))
    options.sort(key=lambda entry: entry[:3])
    return [{'configuration': wound, 'winding': list(turns), 'reach_rad': reach}
            for reach, _, turns, wound in options]


HOME_START_POLICY = 'validated_lifted_starts'


def lifted_start_states(validator, candidates, home, strict_windings=False):
    """Every legal lifted layer-0 state, kept as the configuration itself.

    A start is a lifted configuration, not a canonical candidate: its winding
    is the room each joint has before its limit, and the graph carries that
    winding the whole way. The filter this replaces admitted a canonical row
    because some lift of it was reachable and then discarded which lift, so
    the graph started in canonical windings the home might not reach at all.
    That start policy disconnected a 5000-waypoint trial at layer 762, where a
    graph seeded with home-validated lifts got to 1709.

    strict_windings keeps only the winding each candidate was stored in.
    """
    out = []
    for index, row in enumerate(candidates):
        row = np.asarray(row, dtype=float)
        options = ([{'configuration': row, 'winding': [0] * 6}] if strict_windings
                   else start_winding_options(validator, row, home))
        out.extend({'candidate': index, 'winding': list(option['winding']),
                    'state': np.asarray(option['configuration'], dtype=float)}
                   for option in options)
    return out


def _evaluate_start(validator, stage, state, home, via_poses, rate_hz):
    """(admitted, reason) for one lifted start, by one route kind."""
    from ur10e_trajectory_pkg import continuous_validator as cv
    from ur10e_trajectory_pkg import warmup as wu

    if stage == 'direct':
        plan = wu.plan_warmup(validator, home, state, rate_hz=rate_hz)
        if plan['status'] != 'ok':
            return False, plan['status']
        report = cv.validate_warmup(validator, plan)
        if report['passed']:
            return True, None
        if not report['self_clearance']['passed']:
            return False, 'warmup_self_clearance'
        return False, 'warmup_collision' if report['collision_found'] else 'warmup_limits'
    two = wu.plan_two_segment_warmup(validator, home, state, via_poses,
                                     rate_hz=rate_hz, first_match=True)
    return (True, None) if two['status'] == 'ok' else (False, two['status'])


_START_WORKER = {}


def _init_start_worker(urdf):
    from ament_index_python.packages import get_package_share_directory

    from ur10e_trajectory_pkg.planning_runtime import make_validator as _validator
    _START_WORKER['validator'] = _validator(urdf, get_package_share_directory('ur_description'))


def _evaluate_start_task(task):
    return _evaluate_start(_START_WORKER['validator'], *task)


def validate_home_starts(validator, lifted, home, stage, rate_hz=200.0, via_poses=None,
                         workers=1, urdf=None):
    """The lifted starts the home reaches by one route kind, validated.

    stage 'direct' plans the straight rest-to-rest move and validates its
    stream, the self-clearance floor included; 'via' needs a two-segment route
    through a screened via pose whose segments and join validate. workers > 1
    evaluates them in a process pool, each worker with its own validator.
    """
    import multiprocessing
    import time
    from collections import Counter

    started = time.perf_counter()
    home = np.asarray(home, dtype=float)
    tasks = [(stage, entry['state'], home, via_poses, rate_hz) for entry in lifted]
    if workers > 1 and urdf is not None and len(tasks) > 1:
        context = multiprocessing.get_context('fork')
        with context.Pool(workers, initializer=_init_start_worker,
                          initargs=(urdf,)) as pool:
            results = pool.map(_evaluate_start_task, tasks, chunksize=1)
    else:
        results = [_evaluate_start(validator, *task) for task in tasks]
    admitted = [dict(entry, route_kind=stage)
                for entry, (ok, _) in zip(lifted, results) if ok]
    reasons = Counter(reason for ok, reason in results if not ok)
    return admitted, {'stage': stage, 'evaluated': len(tasks),
                      'admitted': len(admitted), 'reasons': dict(reasons),
                      'seconds': time.perf_counter() - started}


def staged_home_start_search(validator, candidates, home, run, rate_hz=200.0,
                             via_poses=None, strict_windings=False, workers=1,
                             urdf=None):
    """Plan from the lifted starts the home reaches: direct first, then via.

    run(start_states) returns (result, first_empty, graph). Every lifted
    state is validated by a direct route and the graph runs from those alone.
    Only if that leaves no complete path are the lifted states WITHOUT a
    direct route validated through via poses, and the graph runs again from
    both sets. A via state is not excluded because another winding of the
    same candidate has a direct route: a different winding can lead to a
    different long-horizon branch.

    Returns (result, first_empty, graph, record).
    """
    import time

    started = time.perf_counter()
    home = np.asarray(home, dtype=float)
    lifted = lifted_start_states(validator, candidates, home, strict_windings)
    direct, direct_stage = validate_home_starts(validator, lifted, home, 'direct',
                                                rate_hz, via_poses, workers, urdf)
    stages, admitted = [direct_stage], list(direct)
    result, first_empty, graph = run([entry['state'] for entry in admitted])
    attempts = [{'seeds': 'direct', 'start_states': len(admitted),
                 'complete_path': result is not None,
                 'first_disconnected_layer': first_empty}]
    if result is None and via_poses is not None:
        reached = {state_key(entry['state']) for entry in direct}
        remaining = [entry for entry in lifted if state_key(entry['state']) not in reached]
        via, via_stage = validate_home_starts(validator, remaining, home, 'via',
                                              rate_hz, via_poses, workers, urdf)
        stages.append(via_stage)
        admitted = direct + via
        result, first_empty, graph = run([entry['state'] for entry in admitted])
        attempts.append({'seeds': 'direct+via', 'start_states': len(admitted),
                         'complete_path': result is not None,
                         'first_disconnected_layer': first_empty})
    route_of = {state_key(entry['state']): entry for entry in admitted}
    chosen = None if result is None else route_of.get(state_key(result[0][0]))
    record = {
        'policy': HOME_START_POLICY, 'home': home.tolist(),
        'layer_0_candidates': len(candidates), 'lifted_states': len(lifted),
        'kept': len(admitted), 'removed': len(lifted) - len(admitted),
        'routes': {'direct': len(direct), 'via': len(admitted) - len(direct)},
        'via_poses_offered': 0 if via_poses is None else len(via_poses),
        'winding_aware': not strict_windings,
        'candidates_with_a_start': len({entry['candidate'] for entry in admitted}),
        'stages': stages, 'attempts': attempts,
        'reasons': stages[-1]['reasons'],
        'chosen_start': (None if chosen is None else
                         {'candidate': chosen['candidate'], 'winding': chosen['winding'],
                          'route_kind': chosen['route_kind']}),
        'seconds': time.perf_counter() - started,
    }
    return result, first_empty, graph, record


def filter_self_clearance(validator, layers, floor):
    """Drop candidates whose non-adjacent self-clearance is below floor."""
    import time

    started = time.perf_counter()
    kept, removed, queries = [], [], 0
    for rows in layers:
        survivors = []
        for row in rows:
            queries += 1
            if validator.self_clearance(row)['distance_m'] >= floor:
                survivors.append(row)
        removed.append(len(rows) - len(survivors))
        kept.append(survivors)
    seconds = time.perf_counter() - started
    return kept, {'floor_m': floor, 'removed_per_layer': removed,
                  'removed_total': int(sum(removed)), 'queries': queries,
                  'seconds': seconds,
                  'ms_per_query': 1000.0 * seconds / max(queries, 1)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidates', required=True)
    parser.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    parser.add_argument('--layers', type=int, default=500,
                        help='RECORDED waypoints; a spin-up adds samples')
    parser.add_argument('--out', default='graph.json')
    parser.add_argument('--spin-up-s', type=float, default=None,
                        help='spin-up the candidates were generated with')
    parser.add_argument('--max-candidate-condition', type=float,
                        default=GRAPH_CANDIDATE_CONDITION_LIMIT,
                        help='drop candidates above this arm condition number')
    parser.add_argument('--home', type=float, nargs=7, default=None,
                        help='restrict layer-0 starts to those this home can '
                             'warm up to directly')
    parser.add_argument('--home-json', default=None,
                        help='home_choice.json to take the chosen home from')
    parser.add_argument('--strict-home-windings', action='store_true',
                        help='judge layer-0 starts only in the winding they '
                             'were stored in; the backstop when a placement '
                             'planned winding-aware has no valid winding for '
                             'the whole path')
    parser.add_argument('--via-poses', default=None,
                        help='screened poses to route a two-segment warmup '
                             'through when the straight move collides')
    parser.add_argument('--min-self-clearance', type=float, default=None,
                        help='drop candidates below this non-adjacent '
                             'self-clearance (default SELF_CLEARANCE_FLOOR_M)')
    parser.add_argument('--csv', default=None,
                        help='recording the candidates were generated for '
                             '(default: the packaged camera_traj.csv)')
    parser.add_argument('--workers', type=int, default=1,
                        help='processes validating lifted starts from the home')
    parser.add_argument('--start-index', type=int, default=0,
                        help='first recorded sample of the slice the candidates '
                             'were generated for')
    args = parser.parse_args(argv)

    from ament_index_python.packages import get_package_share_directory

    from ur10e_trajectory_pkg.planning_runtime import (
        make_validator as _validator,
        load_trajectory,
        recording_mismatch,
        recording_record,
    )

    mesh_path = get_package_share_directory('ur_description')
    validator = _validator(args.urdf, mesh_path)
    targets, _, dt, trajectory_metadata = load_trajectory(
        args.csv, args.layers, with_metadata=True, spin_up_s=args.spin_up_s,
        start_index=args.start_index)
    recording = recording_record(args.csv)
    num_layers = len(targets)
    layers, candidates_document = load_candidates(
        args.candidates, num_layers,
        max_condition=args.max_candidate_condition)
    self_clearance_floor = (motion_limits.SELF_CLEARANCE_FLOOR_M
                            if args.min_self_clearance is None
                            else args.min_self_clearance)
    layers, self_clearance_filter = filter_self_clearance(
        validator, layers, self_clearance_floor)
    home = args.home
    if args.home_json is not None:
        with open(args.home_json, encoding='utf-8') as handle:
            home = json.load(handle)['chosen']['configuration']
    via_poses = None
    if args.via_poses is not None:
        with open(args.via_poses, encoding='utf-8') as handle:
            via_poses = [entry['configuration'] if isinstance(entry, dict) else entry
                         for entry in json.load(handle)]
    home_filter = None
    condition_bound = best_condition_lower_bound(candidates_document, num_layers)
    generated_RG = (candidates_document.get('manifest', {})
                    .get('trajectory', {}).get('placement_RG'))
    if generated_RG is None or not np.allclose(generated_RG,
                                               trajectory_metadata['placement_RG'],
                                               atol=1e-9):
        print('candidates were not generated at the fixed nominal placement')
        return 1
    mismatch = recording_mismatch(
        candidates_document.get('manifest', {}).get('inputs'), recording)
    if mismatch:
        print(f'candidates were {mismatch}')
        return 1
    if candidates_document.get('num_waypoints') != num_layers:
        print(f"candidates cover {candidates_document.get('num_waypoints')} "
              f'waypoints but this trajectory has {num_layers}; generate them '
              'with the same --waypoints and --spin-up-s')
        return 1
    # The same number of layers from another start is another slice, with
    # other targets; candidates from before slices started at sample 0.
    generated_start = (candidates_document.get('manifest', {})
                       .get('trajectory', {}).get('start_index', 0))
    if generated_start != args.start_index:
        print(f'candidates were generated for the slice starting at sample '
              f'{generated_start}, not {args.start_index}')
        return 1
    graph = LayeredGraph(validator, layers, num_layers, dt=dt)
    if home is not None:
        # The free start begins only where the home reaches: lifted states,
        # validated, directly first and through via poses only if needed.
        def run(start_states):
            fresh = LayeredGraph(validator, layers, num_layers, dt=dt)
            found, empty, _ = fresh.shortest_path(None, start_states=start_states)
            return found, empty, fresh

        result, first_empty, graph, home_filter = staged_home_start_search(
            validator, layers[0], home, run, via_poses=via_poses,
            strict_windings=args.strict_home_windings, workers=args.workers,
            urdf=args.urdf)
    else:
        result, first_empty, _ = graph.shortest_path(None)

    document = {
        'schema_version': SCHEMA_VERSION,
        'environment': environment.describe(),
        'build': 'generator',
        'layers': num_layers,
        'recorded_waypoints': args.layers,
        'start_index': args.start_index,
        'end_index': trajectory_metadata['end_index'],
        'recorded_start_time_s': trajectory_metadata['recorded_start_time_s'],
        'mount': trajectory_metadata['mount'],
        'spin_up': trajectory_metadata.get('spin_up'),
        'start_mode': 'free',
        'start_policy': (home_filter or {}).get('policy'),
        'swept_collision_cache': dict(graph.swept_cache_stats),
        'q_start': None,
        'chosen_start': (None if result is None
                         else np.asarray(result[0][0]).tolist()),
        'placement': 'nominal',
        'recording': recording,
        'candidate_filters': {
            'condition': candidates_document['condition_filter'],
            'self_clearance': self_clearance_filter,
            'home_reachable': home_filter,
            'best_condition_lower_bound': condition_bound,
        },
        'path_max_condition': (None if result is None else max(
            _arm_condition(validator, q) for q in result[0])),
        'path_min_self_clearance_m': (None if result is None else min(
            validator.self_clearance(q)['distance_m'] for q in result[0])),
        'placement_RG': np.asarray(trajectory_metadata['placement_RG']).tolist(),
        'entry_state': (None if result is None else entry_state_report(
            result[0], dt, graph.velocity_limits,
            motion_limits.acceleration_vector())),
        'counters': graph.counters,
        'velocity_limits': motion_limits.effective_limits(validator),
        'edge_velocity_limits': graph.velocity_limits.tolist(),
        'transition_seconds': None,
        'complete_path': result is not None,
        'first_disconnected_layer': first_empty,
        'path_cost': None if result is None else result[1],
        'path': None if result is None else [c.tolist() for c in result[0]],
    }
    if result is None:
        # A disconnected graph still has a best path through its last connected
        # layer: what a section can be certified on.
        partial = best_partial_path(getattr(graph, 'last_history', None))
        document.update(
            last_connected_layer=(None if partial is None else len(partial[0]) - 1),
            partial_path=(None if partial is None
                          else [np.asarray(c).tolist() for c in partial[0]]),
            partial_path_cost=None if partial is None else partial[1])
    with open(args.out, 'w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=1, sort_keys=True)

    print(f"complete_path={document['complete_path']}")
    print(f"candidates removed: condition > {args.max_candidate_condition} "
          f"{candidates_document['condition_filter']['removed_total']}, "
          f"self-clearance < {self_clearance_floor} "
          f"{self_clearance_filter['removed_total']} "
          f"({self_clearance_filter['queries']} queries, "
          f"{self_clearance_filter['ms_per_query']:.3f} ms each)")
    if home_filter is not None:
        print(f"lifted starts the home reaches: {home_filter['kept']} of "
              f"{home_filter['lifted_states']} lifted states of "
              f"{home_filter['layer_0_candidates']} candidates "
              f"(direct {home_filter['routes']['direct']}, via "
              f"{home_filter['routes']['via']}; "
              + '; '.join(f"{attempt['seeds']}: "
                          f"{'complete' if attempt['complete_path'] else 'disconnected at ' + str(attempt['first_disconnected_layer'])}"
                          for attempt in home_filter['attempts'])
              + f"; {home_filter['seconds']:.1f} s, {home_filter['reasons']})")
    # Always, not only on failure: capping candidates stops a path crossing a
    # threshold, it does not make the planner prefer good conditioning, so the
    # cheapest path drifts up to whatever the cap is. The gap between what was
    # achievable and what the path got is where that shows.
    print(f"best achievable worst candidate condition is at least "
          f"{condition_bound['value']:.2f} (layer {condition_bound['layer']})")
    if result is not None:
        print(f"path max condition {document['path_max_condition']:.2f}, "
              f"min self-clearance {document['path_min_self_clearance_m'] * 1000:.1f} mm")
    if result is None:
        print(f'first disconnected layer: {first_empty}')
    else:
        print(f'path cost: {result[1]:.4f} over {len(result[0])} states')
        print(f'start: {np.round(result[0][0], 4).tolist()}')
        entry = document['entry_state']
        print(f"entry state: at_rest={entry['at_rest']} max velocity ratio "
              f"{entry['max_velocity_ratio']:.5f} max acceleration ratio "
              f"{entry['max_acceleration_ratio']:.5f} dominant "
              f"{entry['dominant_joint']} ({entry['dominant_kind']})")
    for name, value in graph.counters.items():
        print(f'  {name:26s} {value}')
    # No complete path is a result, and a failing one: returning 0 here let
    # the pipeline hand a path of None to refinement, which raised instead of
    # reporting the disconnected layer.
    return 0 if result is not None else 1


if __name__ == '__main__':
    sys.exit(main())
