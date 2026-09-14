#!/usr/bin/env python3
"""Stage 7 cascade runner: ready poses against the layer-0 branches of each
placement, cheapest checks first.

Per placement, once:

  1. task gates on each layer-0 candidate: collision, arm conditioning, and
     alpha* against the task's own first twist
  2. continuation through layers 1 and 2 over the graph's own edges (rail
     displacement, reachable lifts, swept collision), keeping the CHEAPEST
     continuation whose entry state is within the velocity and acceleration
     limits, and counting the alternatives. Entry state is invariant under a
     lift, so this is decided once here: a continuation entering beyond a
     limit cannot be matched by any approach from rest. A candidate with no
     such continuation is dropped before anything is classified: it is not a
     task candidate, and it must not count against any ready pose
  3. branch labels over the surviving candidates

Per ready pose and branch:

  4. every winding of every member candidate within the maximum approach
     duration, with its prefix lifted along (lifted_prefix); a lift whose
     continuation leaves the joint limits is dropped as an invalid
     representation, not recorded as a failure
  5. ordered by the average-speed lower bound on duration. Exact minimum
     durations and the joint-limit check (numpy, no physics query) are
     computed lazily: an alternative is timed only while its bound is at most
     the shortest timed duration still waiting, so ties are timed too
  6. the shortest timed alternative is collision-checked, repeating until
     the first feasible approach, which is therefore the branch's shortest
     in (duration, candidate, winding) order. Alternatives never timed
     because their bound exceeded it are counted as untimed_by_bound, so
     rejection counts cover timed alternatives only

Entry state is computed once per candidate: it is invariant under a lift.
exhaustive=True times every alternative before any collision check; the pilot
runs it once and requires the same branch outcomes.

A placement whose candidates all fail steps 1-2 is no_task_candidate and no
ready pose is evaluated against it. Otherwise a ready pose is connected if any
branch has a feasible approach, and direct_approach_unconnected if none does,
with the causes kept in failure_breakdown.

Layers for a placement come from the candidate generator run on layers 0-2
of THAT placement's targets, with the nominal candidates added only as extra
seeds tagged with their origin. Seeds alone could only rediscover nominal's
branches, undercounting branches, a primary ranking key, and blurring
no_task_candidate into "nominal's branches do not carry over". The nominal
placement can use the committed full-length generator output instead. A
placement with no layers is refused rather than given invented ones.

Continuation policy and the plan: for the one layer-0 candidate the Stage 5
graph path uses, its layers 1-2 are exactly the cheapest two-step
continuation. That is one agreement, not proof, since the graph minimises over
all 500 layers. Once READY_Q is chosen, recheck its approach against the real
graph path's layers 0-2, or start the planner from that continuation.

Nothing here selects READY_Q. The pilot measures cost; selection needs the
full sweep and a doubled-pool stability check.

Pilot, 8 ready poses (3 anchors, 5 broad finalists across the rail) from a
pool of 73 (900 of 4096 samples passed the static gates), against nominal and
two generated placements. Two repeats identical; the exhaustive run agrees on
every branch outcome:

    placement      layer-0  valid  seed-only  branches  independent
    nominal        36       33     0          11        11
    translate_x+   74       70     34         13        11
    rotate_r+      58       55     33         15         8

  nominal reads the committed full-length generator output; the other two are
  generated at their placement in 1.9 s each, with nominal's candidates as
  tagged extra seeds. Regenerating nominal the same way without seeds
  reproduces the committed layers 0-2 exactly. Dropped before classifying:
  2, 4 and 2 candidates with no continuation; 1 at nominal and 1 at rotate_r+
  entering beyond the rail cap.

  Every ready pose connects at every placement. Worst branch count 10-11 over
  all branches, but 8 for every pose once branches reached only through
  nominal's seeds are excluded: at rotate_r+, 7 of 15 branches exist only
  through them. Which count ranks is a decision the sweep needs.

  Windings 40,448: 16,351 timed (982 no duration, 367 leave joint limits),
  24,097 left untimed by the bound, 481 collision-checked (174 collide).
  Collision queries per approach p50 72, p95 114, max 211.
  0.85 s per ready pose and placement at p50, 1.46 s at p95.

Projected full sweep, 29 placements against the 73-pose pool, generation
included: 31 min at p50, 53 min at p95.

Earlier pilots of this runner are superseded. The first reported 10 of 11
branches for every pose, which was minimum_duration missing windows; the
second used an upward scan that still missed 16 windings and was recorded as
finding none.

Usage (pilot):
    python3 -m ur10e_trajectory_pkg.ready_pose_runner \\
        --candidates candidates.json --out pilot.json
"""
import argparse
import heapq
import json
import os
import sys
import time
from contextlib import contextmanager

import numpy as np

from ur10e_trajectory_pkg import (
    continuous_validator,
    environment,
    motion_limits,
    ready_pose_sweep as sweep,
)
from ur10e_trajectory_pkg.configurations import ARM_SLICE
from ur10e_trajectory_pkg.graph_planner import (
    SWEPT_SAMPLES,
    LayeredGraph,
    load_candidates,
    state_key,
)
from ur10e_trajectory_pkg.joint_coordinates import TWO_PI

SCHEMA_VERSION = 1

PREFIX_LAYERS = 3                  # PCHIP's initial derivatives need three
TASK_CONDITION_THRESHOLD = 50.0    # the tracker's own singularity gate
TASK_MIN_ALPHA_STAR = 1.0
MAX_APPROACH_SECONDS = 20.0        # the upper bound minimum_duration searches
DURATION_LOWER_S = 0.2
CONTINUATION_POLICY = (
    'cheapest two-step continuation per layer-0 candidate, under the graph '
    'edge cost, among those whose PCHIP entry state is within the velocity '
    'and acceleration limits; valid and entry-feasible alternative counts '
    'are recorded')
PILOT_SELECTION_RULE = (
    'one anchor at each anchor rail position (first passing, distinct names), '
    'then the first broad finalist from each rail stratum holding no anchor, '
    'then remaining broad finalists in pool order')

NO_LAYERS = 'layers_unavailable'


# --------------------------------------------------------------------------
# Counting and timing
# --------------------------------------------------------------------------

class Meter:
    """Counters and wall time, attributed to the phase that incurred them."""

    def __init__(self):
        self.counts = {}
        self.seconds = {}
        self.current = None

    def add(self, key, amount=1):
        self.counts[key] = self.counts.get(key, 0) + int(amount)

    @contextmanager
    def phase(self, name):
        previous, self.current = self.current, name
        start = time.perf_counter()
        try:
            yield
        finally:
            self.seconds[name] = (self.seconds.get(name, 0.0)
                                  + time.perf_counter() - start)
            self.current = previous


@contextmanager
def counting_collisions(validator, meter):
    """Count every collision query, by phase, wherever it is made.

    Wraps the instance, so the static gates, the pool screen, swept edges and
    approaches are all counted by the same mechanism rather than by each
    caller remembering to.
    """
    had_own = 'check_all_collisions' in vars(validator)
    original = validator.check_all_collisions

    def counted(q_full, verbose=False):
        meter.add(f'collision_queries:{meter.current or "unphased"}')
        return original(q_full, verbose=verbose)

    validator.check_all_collisions = counted
    try:
        yield
    finally:
        if had_own:
            validator.check_all_collisions = original
        else:
            del validator.check_all_collisions


@contextmanager
def counting_static_gates(meter):
    """Count static-gate evaluations, each one also a closest-points query."""
    original = sweep.static_gates

    def counted(*args, **kwargs):
        meter.add(f'static_gate_evaluations:{meter.current or "unphased"}')
        return original(*args, **kwargs)

    sweep.static_gates = counted
    try:
        yield
    finally:
        sweep.static_gates = original


def distribution(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {'count': 0, 'p50': None, 'p95': None, 'max': None}
    return {'count': int(len(values)),
            'p50': float(np.percentile(values, 50)),
            'p95': float(np.percentile(values, 95)),
            'max': float(np.max(values))}


# --------------------------------------------------------------------------
# Per placement
# --------------------------------------------------------------------------

def layer0_task_gates(validator, configuration, twist, velocity_limits):
    """Collision, conditioning and alpha* on one layer-0 candidate.

    Returns the first failing gate's name, or None. alpha* asks whether the
    configuration can produce the task's own first twist within the velocity
    limits, which is what makes it a task candidate rather than merely a pose
    solution.
    """
    if validator.check_all_collisions(configuration):
        return 'collision'
    singular = np.linalg.svd(validator.compute_arm_jacobian(configuration),
                             compute_uv=False)
    condition = singular[0] / singular[-1] if singular[-1] > 1e-12 else np.inf
    if condition > TASK_CONDITION_THRESHOLD:
        return 'conditioning'
    result = continuous_validator.twist_alpha_star(
        validator.compute_system_jacobian(configuration), twist,
        velocity_limits)
    if not result['solved'] or result['alpha'] is None:
        return 'alpha_star_unsolved'
    if result['alpha'] < TASK_MIN_ALPHA_STAR:
        return 'alpha_star'
    return None


def prepare_placement(validator, name, layers, positions, quaternions, dt,
                      meter, velocity_limits=None, tolerance=0.35,
                      acceleration_limits=None, layer0_extra_seed_only=None):
    """Cache everything about a placement that does not depend on a ready pose.

    layers holds canonical configurations for layers 0, 1 and 2, or None when
    no candidates exist for this placement.

    layer0_extra_seed_only, aligned with layers[0], marks candidates that only
    another placement's seeds produced. The rail makes the arm redundant, so
    every such seed tends to land on its own point of the self-motion manifold
    and survive deduplication; counting them says nothing about branches. What
    matters is whether a whole BRANCH exists only through them, which is what
    branch_clusters_independent excludes.
    """
    velocity_limits = (validator.velocity_limits if velocity_limits is None
                       else np.asarray(velocity_limits, dtype=float))
    acceleration_limits = (motion_limits.acceleration_vector()
                           if acceleration_limits is None
                           else np.asarray(acceleration_limits, dtype=float))
    counts = {}
    state = {'name': name, 'counts': counts}
    if layers is None:
        state['status'] = NO_LAYERS
        return state

    layers = [[np.asarray(c, dtype=float) for c in layer]
              for layer in layers[:PREFIX_LAYERS]]
    counts['layer_candidates'] = [len(layer) for layer in layers]
    twist = continuous_validator.task_twist(
        [0.0, dt], positions[:2], quaternions[:2], 0, dt)

    graph = LayeredGraph(validator, layers, PREFIX_LAYERS, dt=dt,
                         velocity_limits=velocity_limits)
    layer1_cache, layer2_cache = {}, {}
    rejections = {}
    valid = []

    for index, candidate in enumerate(layers[0]):
        failed = layer0_task_gates(validator, candidate, twist, velocity_limits)
        if failed:
            rejections[failed] = rejections.get(failed, 0) + 1
            continue

        key = state_key(candidate)
        if key not in layer1_cache:
            layer1_cache[key] = graph.successors(candidate, 1, dt)
        best, alternatives, entry_feasible = None, 0, 0
        for middle, first_cost in layer1_cache[key]:
            middle_key = state_key(middle)
            if middle_key not in layer2_cache:
                layer2_cache[middle_key] = graph.successors(middle, 2, dt)
            for last, second_cost in layer2_cache[middle_key]:
                alternatives += 1
                velocity, acceleration = sweep.entry_state_from_prefix(
                    np.stack([candidate, middle, last]), dt)
                if (np.any(np.abs(velocity) > velocity_limits)
                        or np.any(np.abs(acceleration) > acceleration_limits)):
                    continue
                entry_feasible += 1
                rank = (round(first_cost + second_cost, 12), middle_key,
                        state_key(last))
                if best is None or rank < best[0]:
                    best = (rank, middle, last)
        if best is None:
            reason = ('entry_state_exceeds_limits' if alternatives
                      else 'no_continuation')
            rejections[reason] = rejections.get(reason, 0) + 1
            continue

        prefix = sweep.lifted_prefix(validator,
                                     np.stack([candidate, best[1], best[2]]))
        if prefix is None:
            rejections['continuation_leaves_limits'] = (
                rejections.get('continuation_leaves_limits', 0) + 1)
            continue
        entry_velocity, entry_acceleration = sweep.entry_state_from_prefix(
            prefix, dt)
        valid.append({'candidate_index': index, 'configuration': candidate,
                      'extra_seed_only': bool(
                          layer0_extra_seed_only[index]
                          if layer0_extra_seed_only is not None else False),
                      'entry_velocity': entry_velocity,
                      'entry_acceleration': entry_acceleration,
                      'prefix': prefix, 'continuation_alternatives': alternatives,
                      'entry_feasible_continuations': entry_feasible,
                      'continuation_cost': best[0][0]})

    counts['task_gate_rejections'] = rejections
    counts['valid_candidates'] = len(valid)
    counts['layer1_states_expanded'] = len(layer1_cache)
    counts['layer2_states_expanded'] = len(layer2_cache)
    counts['valid_two_step_continuations'] = int(
        sum(v['continuation_alternatives'] for v in valid))
    counts['graph'] = dict(graph.counters)

    if not valid:
        state['status'] = sweep.NO_TASK_CANDIDATE
        return state

    labels = sweep.branch_assignments([v['configuration'] for v in valid],
                                      tolerance)
    for entry, label in zip(valid, labels):
        entry['branch'] = label
    counts['branch_clusters'] = len(set(labels))
    counts['branch_clusters_independent'] = len(
        {v['branch'] for v in valid if not v['extra_seed_only']})
    counts['valid_candidates_extra_seed_only'] = sum(
        1 for v in valid if v['extra_seed_only'])
    state.update(status='ready', valid=valid, dt=dt,
                 velocity_limits=velocity_limits)
    return state


# --------------------------------------------------------------------------
# Per ready pose
# --------------------------------------------------------------------------

def _alternatives(validator, ready, members, velocity_limits, counts):
    """Every winding of a branch's members, ordered by duration lower bound."""
    out = []
    for entry in members:
        candidate = entry['configuration']
        lifts = sweep.destination_lifts(validator, candidate, ready,
                                        velocity_limits, MAX_APPROACH_SECONDS)
        counts['winding_alternatives'].append(len(lifts))
        for lift in lifts:
            if sweep.lifted_prefix(validator, entry['prefix'], lift) is None:
                counts['lift_continuation_invalid'] += 1
                continue
            out.append({
                'bound_s': sweep.duration_lower_bound(
                    ready, lift, velocity_limits, DURATION_LOWER_S),
                'candidate_index': entry['candidate_index'],
                'winding': np.rint((lift[ARM_SLICE] - candidate[ARM_SLICE])
                                   / TWO_PI).astype(int).tolist(),
                'target': lift,
                # Invariant under a lift, so cached per candidate.
                'entry_velocity': entry['entry_velocity'],
                'entry_acceleration': entry['entry_acceleration'],
            })
    out.sort(key=lambda a: (a['bound_s'], a['candidate_index'],
                            tuple(a['winding'])))
    return out


def _time_alternative(validator, ready, alternative, velocity_limits,
                      acceleration_limits):
    """Exact minimum duration and joint limits. Returns a reason code or None."""
    duration = sweep.minimum_duration(
        ready, alternative['target'], alternative['entry_velocity'],
        alternative['entry_acceleration'], velocity_limits,
        acceleration_limits, lower=DURATION_LOWER_S,
        upper=MAX_APPROACH_SECONDS)
    if duration is None:
        return sweep.REASON_NO_DURATION
    coefficients = sweep.quintic_coefficients(
        ready, alternative['target'], alternative['entry_velocity'],
        alternative['entry_acceleration'], duration)
    _, position, _, _, _ = sweep.sample_quintic(coefficients, duration)
    lower, upper = validator.robot.qlim
    if np.any(position < lower - 1e-9) or np.any(position > upper + 1e-9):
        return sweep.REASON_JOINT_LIMITS
    alternative['duration_s'] = float(duration)
    return None


def evaluate_ready_pose(validator, ready, placement, meter,
                        acceleration_limits=None, exhaustive=False):
    """One ready pose against one prepared placement."""
    ready = np.asarray(ready, dtype=float)
    if placement['status'] != 'ready':
        return {'placement': placement['name'],
                'classification': (sweep.NO_TASK_CANDIDATE
                                   if placement['status'] == sweep.NO_TASK_CANDIDATE
                                   else placement['status']),
                'branches': []}

    acceleration_limits = (motion_limits.acceleration_vector()
                           if acceleration_limits is None
                           else np.asarray(acceleration_limits, dtype=float))
    velocity_limits = placement['velocity_limits']
    step_bounds = sweep.collision_step_bounds()
    counts = {'winding_alternatives': [], 'lift_continuation_invalid': 0,
              'collision_queries_per_approach': [],
              'approaches_collision_checked': 0, 'alternatives_timed': 0,
              'untimed_by_bound': 0, 'timed_not_collision_checked': 0}
    all_outcomes = []

    by_branch = {}
    for entry in placement['valid']:
        by_branch.setdefault(entry['branch'], []).append(entry)

    branches = []
    for label in sorted(by_branch):
        with meter.phase('enumerate'):
            alternatives = _alternatives(validator, ready, by_branch[label],
                                         velocity_limits, counts)
        rejected, checked, waiting = [], [], []
        best, cursor = None, 0
        while True:
            with meter.phase('enumerate'):
                while cursor < len(alternatives) and (
                        exhaustive or not waiting
                        or alternatives[cursor]['bound_s'] <= waiting[0][0][0]):
                    alternative = alternatives[cursor]
                    reason = _time_alternative(validator, ready, alternative,
                                               velocity_limits,
                                               acceleration_limits)
                    if reason is None:
                        key = (alternative['duration_s'],
                               alternative['candidate_index'],
                               tuple(alternative['winding']))
                        heapq.heappush(waiting, (key, cursor, alternative))
                    else:
                        rejected.append({'feasible': False,
                                         'reason_code': reason})
                    cursor += 1
            if not waiting:
                break
            _, _, alternative = heapq.heappop(waiting)
            with meter.phase('collision'):
                queries = []
                result = sweep.evaluate_approach(
                    validator, ready, alternative['target'],
                    alternative['entry_velocity'],
                    alternative['entry_acceleration'], velocity_limits,
                    acceleration_limits, step_bounds=step_bounds,
                    collision_counter=queries,
                    duration=alternative['duration_s'])
            counts['approaches_collision_checked'] += 1
            counts['collision_queries_per_approach'].append(sum(queries))
            checked.append(result)
            if result['feasible']:
                best = (alternative, result)
                break
        counts['alternatives_timed'] += cursor
        counts['untimed_by_bound'] += len(alternatives) - cursor
        counts['timed_not_collision_checked'] += len(waiting)

        outcomes = rejected + checked
        all_outcomes += outcomes
        branch = {
            'branch': label,
            'members': len(by_branch[label]),
            'independent': any(not e['extra_seed_only']
                               for e in by_branch[label]),
            'alternatives': len(alternatives),
            'alternatives_timed': cursor,
            'rejected_before_collision': len(rejected),
            'collision_checked': len(checked),
            'failure_breakdown': sweep.failure_breakdown(outcomes),
            'connected': best is not None,
        }
        if best is not None:
            alternative, result = best
            branch.update(
                duration_s=result['duration_s'],
                max_peak_jerk=result['max_peak_jerk'],
                candidate_index=alternative['candidate_index'],
                winding=alternative['winding'],
                collision_achieved_step=result.get('collision_achieved_step'))
        branches.append(branch)

    classification = sweep.classify(placement['valid'], all_outcomes)
    connected = [b for b in branches if b['connected']]
    shortest = min(connected, key=lambda b: b['duration_s']) if connected else None
    return {
        'placement': placement['name'],
        'classification': classification,
        'branch_count': len(branches),
        'connected_branches': len(connected),
        'connected_independent_branches': sum(1 for b in connected
                                              if b['independent']),
        'best_duration_s': None if shortest is None else shortest['duration_s'],
        'best_max_peak_jerk': (None if shortest is None
                               else shortest['max_peak_jerk']),
        'failure_breakdown': sweep.failure_breakdown(all_outcomes),
        'branches': branches,
        'counts': counts,
    }


def branch_outcomes(results):
    """What the ranking consumes, without the counts that depend on pruning."""
    return [
        [{'placement': r['placement'], 'classification': r['classification'],
          'branches': [{k: b.get(k) for k in ('branch', 'connected',
                                               'duration_s', 'candidate_index',
                                               'winding')}
                       for b in r['branches']]}
         for r in result['per_placement']]
        for result in results
    ]


def summarise_ready_pose(per_placement):
    """Fields rank_ready_poses orders by, taken worst-case over placements.

    Placements with no task candidate are excluded throughout, as
    connectivity_score already excludes them.
    """
    eligible = [r for r in per_placement
                if r['classification'] in (sweep.CONNECTED,
                                           sweep.DIRECT_APPROACH_UNCONNECTED)]
    connected = [r for r in eligible if r['classification'] == sweep.CONNECTED]
    return {
        'connectivity': sweep.connectivity_score(
            [r['classification'] for r in per_placement
             if r['classification'] != NO_LAYERS]),
        'worst_branch_count': (min(r['connected_branches'] for r in eligible)
                               if eligible else None),
        'worst_independent_branch_count': (
            min(r.get('connected_independent_branches', r['connected_branches'])
                for r in eligible) if eligible else None),
        'worst_duration_s': (max(r['best_duration_s'] for r in connected)
                             if connected else None),
        'worst_peak_jerk': (max(r['best_max_peak_jerk'] for r in connected)
                            if connected else None),
    }


# --------------------------------------------------------------------------
# Pilot selection, the run, and projection
# --------------------------------------------------------------------------

def nominal_seeds(nominal_layers, name='nominal'):
    """Nominal candidates as tagged extra seeds, per layer."""
    return [[(np.asarray(c, dtype=float), f'{name}:layer{k}:candidate{i}')
             for i, c in enumerate(layer)]
            for k, layer in enumerate(nominal_layers)]


def generated_placement(validator, placement, nominal_RG, meter,
                        nominal_layers=None, csv_path=None):
    """Targets and layers 0-2 for one placement, generated at that placement."""
    from ur10e_trajectory_pkg.ClientNode import (
        DEFAULT_CSV_PATH,
        build_trajectory_targets,
    )
    from ur10e_trajectory_pkg import candidate_generator
    from ur10e_trajectory_pkg.configurations import LEGACY_MATLAB_START_Q, RAIL_INDEX

    placement_RG = sweep.placement_transform(placement, nominal_RG)
    (x, y, z, quaternions, times), metadata = build_trajectory_targets(
        csv_path or DEFAULT_CSV_PATH, PREFIX_LAYERS, placement_RG=placement_RG,
        return_metadata=True)
    positions = np.column_stack((x, y, z))
    dt = float(times[1] - times[0])

    extra = None if nominal_layers is None else nominal_seeds(nominal_layers)
    start = time.perf_counter()
    with meter.phase('generation'):
        candidates, records = candidate_generator.generate_layers(
            validator, positions, quaternions, dt, LEGACY_MATLAB_START_Q,
            PREFIX_LAYERS, extra_seeds=extra)
    seconds = time.perf_counter() - start

    layers = [[np.concatenate(([e['rail_position']], e['q_arm_canonical']))
               for e in candidates.get(k, [])] for k in range(PREFIX_LAYERS)]
    seed_only = candidate_generator.only_from_extra_seeds(candidates.get(0, []))
    return {
        'name': placement['name'], 'layers': layers, 'positions': positions,
        'layer0_extra_seed_only': [any(e is s for s in seed_only)
                                   for e in candidates.get(0, [])],
        'quaternions': quaternions, 'dt': dt,
        'generation': {
            'source': 'generated at this placement',
            'placement_RG': np.asarray(placement_RG).tolist(),
            'trajectory': metadata,
            'seconds': seconds,
            'solve_records': len(records),
            'candidates_per_layer': [len(candidates.get(k, []))
                                     for k in range(PREFIX_LAYERS)],
            'only_from_nominal_seeds_per_layer': [
                len(candidate_generator.only_from_extra_seeds(
                    candidates.get(k, []))) for k in range(PREFIX_LAYERS)],
        },
    }


def pilot_ready_poses(pool, count=8, strata=sweep.RAIL_STRATA, rail_travel=3.0):
    """A small spread of ready poses over rail position and provenance."""
    edges = np.linspace(0.0, rail_travel, strata + 1)

    def stratum(entry):
        rail = float(entry['configuration'][0])
        return int(np.clip(np.searchsorted(edges, rail, side='right') - 1,
                           0, strata - 1))

    chosen, names = [], set()
    anchors = [e for e in pool if e['provenance'] == 'anchor']
    for rail in sorted({float(e['configuration'][0]) for e in anchors}):
        options = [e for e in anchors if float(e['configuration'][0]) == rail]
        fresh = [e for e in options if e['anchor_name'] not in names]
        if (fresh or options) and len(chosen) < count:
            pick = (fresh or options)[0]
            chosen.append(pick)
            names.add(pick['anchor_name'])

    occupied = {stratum(e) for e in chosen}
    broad = [e for e in pool if e['provenance'] != 'anchor']
    for index in range(strata):
        if len(chosen) >= count or index in occupied:
            continue
        members = [e for e in broad if stratum(e) == index]
        if members:
            chosen.append(members[0])
    for entry in broad:
        if len(chosen) >= count:
            break
        if not any(entry is c for c in chosen):
            chosen.append(entry)
    return chosen


def run_once(validator, ready_poses, placements, meter, tolerance=0.35,
             exhaustive=False):
    """Prepare each placement, then evaluate every ready pose against it."""
    prepared = []
    for placement in placements:
        with meter.phase('placement'):
            prepared.append(prepare_placement(
                validator, placement['name'], placement['layers'],
                placement['positions'], placement['quaternions'],
                placement['dt'], meter, tolerance=tolerance,
                layer0_extra_seed_only=placement.get('layer0_extra_seed_only')))

    results, pose_seconds = [], []
    for index, entry in enumerate(ready_poses):
        per_placement = []
        for placement in prepared:
            start = time.perf_counter()
            per_placement.append(evaluate_ready_pose(
                validator, entry['configuration'], placement, meter,
                exhaustive=exhaustive))
            pose_seconds.append(time.perf_counter() - start)
        results.append({
            'ready_index': index,
            'configuration': np.asarray(entry['configuration']).tolist(),
            'provenance': entry['provenance'],
            'anchor_name': entry.get('anchor_name'),
            'per_placement': per_placement,
            'summary': summarise_ready_pose(per_placement),
        })
    placement_records = [{'name': p['name'], 'status': p['status'],
                          'counts': p['counts'],
                          'generation': source.get('generation')}
                         for p, source in zip(prepared, placements)]
    return results, placement_records, pose_seconds


def aggregate(results, placement_records, meter, pose_seconds):
    per_pose = [r for result in results for r in result['per_placement']
                if 'counts' in r]
    queries = [q for r in per_pose
               for q in r['counts']['collision_queries_per_approach']]
    windings = [w for r in per_pose for w in r['counts']['winding_alternatives']]
    return {
        'placements': placement_records,
        'collision_queries_per_approach': distribution(queries),
        'winding_alternatives_per_candidate': distribution(windings),
        'approaches_collision_checked': int(sum(
            r['counts']['approaches_collision_checked'] for r in per_pose)),
        'alternatives_timed': int(sum(
            r['counts']['alternatives_timed'] for r in per_pose)),
        'untimed_by_bound': int(sum(
            r['counts']['untimed_by_bound'] for r in per_pose)),
        'timed_not_collision_checked': int(sum(
            r['counts']['timed_not_collision_checked'] for r in per_pose)),
        'lift_continuation_invalid': int(sum(
            r['counts']['lift_continuation_invalid'] for r in per_pose)),
        'pre_collision_rejections': {
            code: int(sum(
                sum(b['failure_breakdown'].get(code, 0) for b in r['branches'])
                for r in per_pose))
            for code in (sweep.REASON_NO_DURATION, sweep.REASON_JOINT_LIMITS)},
        'collision_rejections': int(sum(
            r['failure_breakdown'].get(sweep.REASON_COLLISION, 0)
            for r in per_pose)),
        'classifications': {
            label: sum(1 for result in results for r in result['per_placement']
                       if r['classification'] == label)
            for label in (sweep.CONNECTED, sweep.DIRECT_APPROACH_UNCONNECTED,
                          sweep.NO_TASK_CANDIDATE, NO_LAYERS)},
        'seconds_per_ready_pose_placement': distribution(pose_seconds),
        'phase_seconds': dict(meter.seconds),
        'counts': dict(meter.counts),
    }


def project_runtime(pool_seconds, placement_seconds, pose_seconds, pool_size,
                    placements=29, generation_seconds=None):
    """Full-sweep wall time from pilot measurements, at p50 and p95.

    generation_seconds, measured per generated placement, is charged to every
    placement; without it generation is excluded and the projection says so.
    """
    per_pose = distribution(pose_seconds)
    fixed = pool_seconds + placements * placement_seconds
    if generation_seconds is not None:
        fixed += placements * generation_seconds
    evaluations = pool_size * placements
    return {
        'placements': placements,
        'pool_size': pool_size,
        'ready_pose_placement_evaluations': evaluations,
        'fixed_seconds': fixed,
        'p50_seconds': fixed + evaluations * per_pose['p50'],
        'p95_seconds': fixed + evaluations * per_pose['p95'],
        'generation_seconds_per_placement': generation_seconds,
        'excludes': (None if generation_seconds is not None
                     else 'candidate generation per placement'),
    }


def results_without_timing(results):
    """Results with every wall-time field removed, for the repeat comparison."""
    return json.dumps(results, sort_keys=True, default=_plain)


def _plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.bool_):
        return bool(value)
    raise TypeError(f'unserialisable {type(value).__name__}')


def manifest(args, validator, candidates_document, trajectory_metadata, dt,
             placement_names, tolerance):
    from ur10e_trajectory_pkg.failure_census import file_digest, repository_revision

    candidate_manifest = candidates_document.get('manifest', {})
    return {
        'repository_revision': repository_revision(environment.workspace_root()),
        'docker_image_id': os.environ.get('UR10E_IMAGE_ID'),
        'environment': environment.describe(),
        'inputs': {
            'urdf_path': args.urdf,
            'urdf_sha256': file_digest(args.urdf),
            'candidates_path': args.candidates,
            'candidates_sha256': file_digest(args.candidates),
            'candidates_repository_revision':
                candidate_manifest.get('repository_revision'),
            'candidates_trajectory': candidate_manifest.get('trajectory'),
            'csv_path': trajectory_metadata['csv_path'],
            'csv_sha256': file_digest(trajectory_metadata['csv_path']),
            'trajectory': trajectory_metadata,
        },
        'envelope': {
            'name': 'PROVISIONAL_STAGE7_ENVELOPE_V1',
            'definition': sweep.PROVISIONAL_STAGE7_ENVELOPE_V1,
            'placements_evaluated': placement_names,
        },
        'pool': {
            'global_samples': sweep.GLOBAL_SAMPLES,
            'sobol_seed': 0,
            'broad_finalists': sweep.BROAD_FINALISTS,
            'rail_strata': sweep.RAIL_STRATA,
            'anchor_postures_deg': [list(a) for a in sweep.ANCHOR_POSTURES_DEG],
            'anchor_rail_positions_m': [0.5, 1.5, 2.5],
            'static_gate_thresholds': {'min_clearance_m': 0.02,
                                       'min_limit_fraction': 0.05,
                                       'min_posture_margin': 0.02},
            'pilot_selection_rule': PILOT_SELECTION_RULE,
        },
        'limits': motion_limits.manifest(validator),
        'certification': sweep.certification_note(validator),
        'collision_step_bounds': sweep.collision_step_bounds().tolist(),
        'clustering': {'tolerance': tolerance,
                       'rail_scale_m': sweep.BRANCH_RAIL_SCALE_M},
        'timing_policy': {'waypoint_dt_s': dt,
                          'max_approach_s': MAX_APPROACH_SECONDS,
                          'duration_lower_s': DURATION_LOWER_S,
                          'duration_method': 'exact: lower bound or a root '
                                             'of the sampled quadratic '
                                             'constraints',
                          'ordering': 'lazy by average-speed lower bound, '
                                      'timing while bound <= shortest '
                                      'waiting duration',
                          'quintic_samples': sweep.QUINTIC_SAMPLES},
        'task_gates': {'condition_number_max': TASK_CONDITION_THRESHOLD,
                       'alpha_star_min': TASK_MIN_ALPHA_STAR,
                       'twist': 'task twist from layer 0 to layer 1'},
        'continuation_policy': CONTINUATION_POLICY,
        'graph_swept_samples': SWEPT_SAMPLES,
        'jerk_references_rad_s3': list(sweep.JERK_REFERENCES_RAD_S3),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidates', required=True,
                        help='candidate_generator output at the NOMINAL placement')
    parser.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    parser.add_argument('--pilot-count', type=int, default=8)
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--tolerance', type=float, default=0.35)
    parser.add_argument('--out', default='pilot.json')
    parser.add_argument('--placements', default='nominal',
                        help='comma-separated placement names from the envelope')
    parser.add_argument('--generate-nominal', action='store_true',
                        help='generate nominal layers instead of reading '
                             '--candidates (for checking the generator)')
    parser.add_argument('--no-exhaustive-check', action='store_true',
                        help='skip the exhaustive run that verifies pruning')
    args = parser.parse_args(argv)

    from ament_index_python.packages import get_package_share_directory

    from ur10e_trajectory_pkg.failure_census import _validator, load_trajectory

    total_start = time.perf_counter()
    validator = _validator(args.urdf, get_package_share_directory('ur_description'))
    positions, quaternions, dt, trajectory_metadata = load_trajectory(
        None, PREFIX_LAYERS, with_metadata=True)
    layers, candidates_document = load_candidates(
        args.candidates, PREFIX_LAYERS, include_oracle=False)
    nominal_RG = trajectory_metadata['placement_RG']
    envelope = {p['name']: p for p in sweep.placements()}
    names = [n.strip() for n in args.placements.split(',') if n.strip()]
    unknown = [n for n in names if n not in envelope]
    if unknown:
        parser.error(f'unknown placements {unknown}')

    generation_meter = Meter()
    placements = []
    for name in names:
        if name == 'nominal' and not args.generate_nominal:
            placements.append({
                'name': 'nominal', 'layers': layers, 'positions': positions,
                'quaternions': quaternions, 'dt': dt,
                'generation': {'source': 'committed full-length generator '
                                         'output (--candidates)'}})
        else:
            with counting_collisions(validator, generation_meter):
                placements.append(generated_placement(
                    validator, envelope[name], nominal_RG, generation_meter,
                    nominal_layers=None if name == 'nominal' else layers))
    generated = [p['generation']['seconds'] for p in placements
                 if 'seconds' in p['generation']]

    pool_meter = Meter()
    with counting_collisions(validator, pool_meter), \
            counting_static_gates(pool_meter), pool_meter.phase('pool'):
        pool, pool_summary = sweep.build_candidate_pool(validator)
    pilot = pilot_ready_poses(pool, args.pilot_count)

    runs = []
    for _ in range(args.repeats):
        meter = Meter()
        with counting_collisions(validator, meter), counting_static_gates(meter):
            results, placement_records, pose_seconds = run_once(
                validator, pilot, placements, meter, args.tolerance)
        runs.append((results, placement_records, pose_seconds, meter))

    results, placement_records, pose_seconds, meter = runs[0]
    reference = results_without_timing(results)
    deterministic = all(results_without_timing(r[0]) == reference
                        for r in runs[1:])

    exhaustive_check = {'ran': False}
    if not args.no_exhaustive_check:
        check_meter = Meter()
        with counting_collisions(validator, check_meter), \
                counting_static_gates(check_meter):
            full, _, full_seconds = run_once(validator, pilot, placements,
                                             check_meter, args.tolerance,
                                             exhaustive=True)
        exhaustive_check = {
            'ran': True,
            'branch_outcomes_agree': (
                json.dumps(branch_outcomes(full), default=_plain)
                == json.dumps(branch_outcomes(results), default=_plain)),
            'seconds_per_ready_pose_placement': distribution(full_seconds),
            'phase_seconds': dict(check_meter.seconds),
            'alternatives_timed': int(sum(
                r['counts']['alternatives_timed']
                for result in full for r in result['per_placement']
                if 'counts' in r)),
        }
    summary = aggregate(results, placement_records, meter, pose_seconds)
    projection = project_runtime(
        pool_meter.seconds['pool'],
        meter.seconds.get('placement', 0.0) / len(placements),
        pose_seconds, len(pool),
        generation_seconds=(float(np.mean(generated)) if generated else None))

    document = {
        'schema_version': SCHEMA_VERSION,
        'manifest': manifest(args, validator, candidates_document,
                             trajectory_metadata, dt, names,
                             args.tolerance),
        'generation': {'counts': generation_meter.counts,
                       'seconds': generation_meter.seconds,
                       'per_placement_seconds': generated},
        'pool': dict(pool_summary, size=len(pool),
                     seconds=pool_meter.seconds['pool'],
                     counts=pool_meter.counts),
        'pilot': [{'configuration': np.asarray(e['configuration']).tolist(),
                   'provenance': e['provenance'],
                   'anchor_name': e['anchor_name']} for e in pilot],
        'deterministic_across_repeats': deterministic,
        'exhaustive_check': exhaustive_check,
        'repeats': args.repeats,
        'repeat_seconds': [sum(r[3].seconds.values()) for r in runs],
        'summary': summary,
        'projection': projection,
        'results': results,
        'total_wall_seconds': time.perf_counter() - total_start,
        'ready_q_assigned': False,
    }
    with open(args.out, 'w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=1, default=_plain)

    print(f"pool {len(pool)} ({pool_summary}) in {pool_meter.seconds['pool']:.1f} s")
    for record in placement_records:
        print(f"placement {record['name']}: {record['status']} {record['counts']}")
    print(f"deterministic across {args.repeats} repeats: {deterministic}")
    print(f"exhaustive check: {exhaustive_check}")
    for key in ('classifications', 'collision_queries_per_approach',
                'winding_alternatives_per_candidate',
                'approaches_collision_checked',
                'alternatives_timed', 'untimed_by_bound',
                'timed_not_collision_checked',
                'lift_continuation_invalid', 'pre_collision_rejections',
                'collision_rejections', 'seconds_per_ready_pose_placement',
                'phase_seconds', 'counts'):
        print(f'  {key}: {summary[key]}')
    for result in results:
        for record in result['per_placement']:
            print(f"  ready {result['ready_index']} {result['provenance']:12s} "
                  f"{result['anchor_name'] or '':24s} rail "
                  f"{result['configuration'][0]:.2f} {record['placement']:14s}: "
                  f"{record['classification']} branches "
                  f"{record.get('connected_branches')}/"
                  f"{record.get('branch_count')} best "
                  f"{record.get('best_duration_s')} s")
    print(f"projection: {projection}")
    print(f"total wall {document['total_wall_seconds']:.1f} s")
    return 0


if __name__ == '__main__':
    sys.exit(main())
