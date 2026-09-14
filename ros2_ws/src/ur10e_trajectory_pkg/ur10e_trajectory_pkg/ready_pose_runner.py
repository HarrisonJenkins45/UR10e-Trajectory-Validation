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
  3. tolerance-cluster labels, then IK families joined by rail continuation
     (ready_pose_sweep.ik_family_labels). Families are the branches ranking
     counts, over the FULL valid set, seeded candidates included: a seeded
     candidate passing every gate proves there is something to connect to.
     Clusters, and families only extra seeds reached, are diagnostics

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
ready pose is evaluated against it; generator_alone_found_nothing records the
separate fact that no unseeded candidate survived. Otherwise a ready pose is connected if any
branch has a feasible approach, and direct_approach_unconnected if none does,
with the causes kept in failure_breakdown.

Layers for a placement come from the candidate generator run on layers 0-2
of THAT placement's targets, with the nominal candidates added only as extra
seeds tagged with their origin. Seeds alone could only rediscover nominal's
branches, undercounting branches, a primary ranking key, and blurring
no_task_candidate into "nominal's branches do not carry over". Every placement goes
through that one path, nominal included, generated first and without seeds so
its layers can seed the rest; --candidates substitutes the committed
full-length output for nominal as a check. A placement with no layers is
refused rather than given invented ones.

Continuation policy and the plan: for the one layer-0 candidate the Stage 5
graph path uses, its layers 1-2 are exactly the cheapest two-step
continuation. That is one agreement, not proof, since the graph minimises over
all 500 layers. Once READY_Q is chosen, recheck its approach against the real
graph path's layers 0-2, or start the planner from that continuation.

Nothing here selects READY_Q. The pilot measures cost; selection needs the
full sweep and a doubled-pool stability check.

Pilot, 8 ready poses (3 anchors, 5 broad finalists across the rail) from a
pool of 73 (900 of 4096 samples passed the static gates), against nominal and
two placements, all generated at their placement (about 2 s each), nominal
first and unseeded, the others with nominal's candidates as tagged extra
seeds. Two repeats identical; the exhaustive run agrees on every branch
outcome:

    placement      layer-0  valid  seed-only  clusters  families  unseeded
    nominal        36       33     0          11        5         5
    translate_x+   74       70     34         13        5         5
    rotate_r+      58       55     33         15        5         4

  Clusters overcount: family links joined clusters 0.10 to 1.46 m apart on the
  rail. The family only nominal's seeds reached at rotate_r+ is the one with
  elbow and wrist_2 signs both positive, which the generator finds unseeded at
  the other two placements. Some links pass condition numbers up to 171, above
  the task gate, so a link is kinematic, not a task-admissible move.

  Every ready pose reaches all 5 families at every placement, including poses
  that miss a cluster, so family count does not separate this pilot and
  ranking falls to worst duration, then jerk. Taken on the slowest family's
  shortest approach, worst durations are 2.28 to 4.00 s and the order is
  1, 0, 5, 4, 3, 6, 2, 7. Taken on the single fastest approach, as before,
  they were 1.09 to 2.58 s and pose 3 ranked first despite a 2.67 s family
  and the pilot's highest jerk, 153.7.

  0.86 s per ready pose and placement at p50, 1.51 s at p95; families take
  about 0.2 s per placement. Regenerating nominal unseeded reproduces the
  committed full-length generator's layers 0-2 exactly.

Projected full sweep, 29 placements against the 73-pose pool, generation
included: 32 min at p50, 55 min at p95.

Earlier pilots of this runner are superseded. The first reported 10 of 11
branches for every pose, which was minimum_duration missing windows; the
second used an upward scan that still missed 16 windings and was recorded as
finding none; the third ranked on tolerance clusters, which split families.

Usage:
    pilot       python3 -m ur10e_trajectory_pkg.ready_pose_runner \\
                    --placements nominal,translate_x+,rotate_r+ --out pilot.json
    full sweep  python3 -m ur10e_trajectory_pkg.ready_pose_runner \\
                    --placements all --all-pool --repeats 1 \\
                    --no-exhaustive-check --out sweep.json
    stability   the same with --pool-samples 8192 --pool-finalists 128
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
    counts['generator_alone_found_nothing'] = all(
        v['extra_seed_only'] for v in valid)

    with meter.phase('families'):
        families, report = sweep.ik_family_labels(
            validator, [v['configuration'] for v in valid], labels,
            positions[0], quaternions[0])
    for entry, family in zip(valid, families):
        entry['family'] = family
    counts['ik_families'] = report['families']
    counts['ik_families_independent'] = len(
        {v['family'] for v in valid if not v['extra_seed_only']})
    counts['ik_families_only_from_extra_seeds'] = sorted(
        {v['family'] for v in valid}
        - {v['family'] for v in valid if not v['extra_seed_only']})
    counts['family_report'] = report
    state.update(status='ready', valid=valid, dt=dt,
                 velocity_limits=velocity_limits,
                 limit_statuses=motion_limits.limit_statuses(validator))
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
                    duration=alternative['duration_s'],
                    duration_lower=DURATION_LOWER_S,
                    limit_statuses=placement['limit_statuses'])
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
            'family': by_branch[label][0]['family'],
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
                collision_achieved_step=result.get('collision_achieved_step'),
                binding=result.get('binding'))
        branches.append(branch)

    classification = sweep.classify(placement['valid'], all_outcomes)
    connected = [b for b in branches if b['connected']]
    shortest = min(connected, key=lambda b: b['duration_s']) if connected else None
    return dict({
        'placement': placement['name'],
        'classification': classification,
        'branch_count': len(branches),
        'connected_branches': len(connected),
        'connected_independent_branches': sum(1 for b in connected
                                              if b['independent']),
        'family_count': len({b['family'] for b in branches}),
        'connected_families': len({b['family'] for b in connected}),
        'best_duration_s': None if shortest is None else shortest['duration_s'],
        'best_max_peak_jerk': (None if shortest is None
                               else shortest['max_peak_jerk']),
        'failure_breakdown': sweep.failure_breakdown(all_outcomes),
        'branches': branches,
        'counts': counts,
    }, **family_approach_summary(branches))


def family_approach_summary(branches):
    """Each reached family's shortest approach, and the slowest of them.

    Exact from the branch records: each connected branch carries its shortest
    feasible approach, so a family's shortest is the minimum over its
    branches. Ties fall to the lower branch label.

    Ranking uses the SLOWEST family, because a ready pose is good when it
    reaches several families cheaply, not when its easiest entry is fast. Its
    jerk counterpart is the largest peak jerk among the families' shortest
    approaches, every one of which the pose may have to use.
    """
    shortest = {}
    for branch in branches:
        if not branch['connected']:
            continue
        family = branch['family']
        current = shortest.get(family)
        if current is None or ((branch['duration_s'], branch['branch'])
                               < (current['duration_s'], current['branch'])):
            shortest[family] = branch
    if not shortest:
        return {'family_shortest_duration_s': {},
                'slowest_family_duration_s': None,
                'slowest_family_binding': None,
                'family_shortest_max_peak_jerk': None}
    slowest = max(shortest.values(), key=lambda b: (b['duration_s'], b['family']))
    return {
        'slowest_family_binding': slowest.get('binding'),
        'family_shortest_duration_s': {str(f): b['duration_s']
                                       for f, b in sorted(shortest.items())},
        'slowest_family_duration_s': max(b['duration_s']
                                         for b in shortest.values()),
        'family_shortest_max_peak_jerk': max(b['max_peak_jerk']
                                             for b in shortest.values()),
    }


def branch_outcomes(results):
    """What the ranking consumes, without the counts that depend on pruning."""
    return [
        [{'placement': r['placement'], 'classification': r['classification'],
          'branches': [{k: b.get(k) for k in ('branch', 'family', 'connected',
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

    worst_family_fraction, the ranking key, is the minimum over placements of
    families reached over families available; worst_family_count is kept as a
    diagnostic, since its minimum is set by the placement with fewest families
    rather than by the pose.

    worst_duration_s and worst_peak_jerk come from family_approach_summary:
    the slowest family's shortest approach, and the largest jerk among the
    families' shortest approaches, at the worst placement. The shortest single
    approach, the former key, is kept as worst_shortest_approach_s: it
    measures only a pose's easiest entry, and ranked a pose with one fast
    family above poses reaching every family faster.
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
        'worst_family_fraction': (
            min(r['connected_families'] / r['family_count'] if r['family_count']
                else 0.0 for r in eligible) if eligible else None),
        'worst_family_count': (min(r['connected_families'] for r in eligible)
                               if eligible else None),
        'worst_independent_branch_count': (
            min(r.get('connected_independent_branches', r['connected_branches'])
                for r in eligible) if eligible else None),
        'worst_duration_s': (max(r['slowest_family_duration_s']
                                 for r in connected) if connected else None),
        'worst_duration_placement': (
            max(connected, key=lambda r: r['slowest_family_duration_s'])['placement']
            if connected else None),
        'worst_duration_binding': (
            max(connected, key=lambda r: r['slowest_family_duration_s']).get(
                'slowest_family_binding') if connected else None),
        'worst_peak_jerk': (max(r['family_shortest_max_peak_jerk']
                                for r in connected) if connected else None),
        'worst_shortest_approach_s': (max(r['best_duration_s'] for r in connected)
                                      if connected else None),
        'worst_shortest_approach_jerk': (
            max(r['best_max_peak_jerk'] for r in connected)
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


# --------------------------------------------------------------------------
# Pool stability, with its pass criteria fixed before any result is seen
# --------------------------------------------------------------------------

# A pose's score does not depend on the other poses in the pool: families,
# fractions, durations and bindings are computed per pose against placements
# generated without reference to it. So a larger pool is checked by
# evaluating only its NEW poses and merging them with the base sweep, and the
# poses the two pools share are rerun to confirm the records are identical
# across processes.
STABILITY_MARGIN_S = 0.1
STABILITY_CRITERIA = (
    'Fixed before the doubled-pool results were seen. (1) The base winner is '
    'in the merged top 3. (2) No new pose beats it decisively: ranks above it '
    'AND has higher connectivity, a higher worst family fraction, or a worst '
    'duration shorter by more than STABILITY_MARGIN_S. (3) Every shared pose '
    'reproduces its base record exactly. Pass needs all three. If (2) fails, '
    'the pool was too small: grow it again or search near the new winner; do '
    'not simply take the new pose.')


def pose_key(configuration):
    return tuple(np.round(np.asarray(configuration, dtype=float), 9).tolist())


def artifact_pose_keys(path):
    with open(path, encoding='utf-8') as handle:
        return {pose_key(r['configuration']) for r in json.load(handle)['results']}


def filter_pool(pool, skip_keys=None, only_keys=None):
    """Pool entries not in skip_keys and, when given, in only_keys."""
    out = []
    for entry in pool:
        key = pose_key(entry['configuration'])
        if skip_keys is not None and key in skip_keys:
            continue
        if only_keys is not None and key not in only_keys:
            continue
        out.append(entry)
    return out


def _ranking_records(results):
    return [dict(r['summary'], key=pose_key(r['configuration']),
                 ready_index=r['ready_index'], provenance=r['provenance'],
                 static_gates={'passed': True}) for r in results]


def _comparable(record):
    """A per-placement record without anything that may vary by process."""
    return json.dumps(record, sort_keys=True, default=_plain)


def stability_check(base_results, extra_results, shared_results,
                    margin_s=STABILITY_MARGIN_S):
    """Apply STABILITY_CRITERIA to a base sweep, its new poses and shared reruns."""
    base = _ranking_records(base_results)
    winner = sweep.rank_ready_poses(base)[0]
    base_keys = {r['key'] for r in base}
    merged = sweep.rank_ready_poses(base + _ranking_records(extra_results))
    position = next(i for i, r in enumerate(merged) if r['key'] == winner['key'])

    def value(record, name, missing):
        return missing if record.get(name) is None else record[name]

    decisive = []
    for record in merged[:position]:
        if record['key'] in base_keys:
            continue
        if (value(record, 'connectivity', -1.0) > value(winner, 'connectivity', -1.0)
                or value(record, 'worst_family_fraction', -1.0)
                > value(winner, 'worst_family_fraction', -1.0)
                or value(record, 'worst_duration_s', np.inf)
                < value(winner, 'worst_duration_s', np.inf) - margin_s):
            decisive.append(record)

    by_key = {pose_key(r['configuration']): r for r in base_results}
    mismatched = [pose_key(r['configuration']) for r in shared_results
                  if pose_key(r['configuration']) not in by_key
                  or _comparable(r['per_placement'])
                  != _comparable(by_key[pose_key(r['configuration'])]['per_placement'])]
    criteria = {
        'winner_in_merged_top_3': position < 3,
        'no_new_pose_beats_winner_decisively': not decisive,
        'shared_poses_reproduce_exactly': bool(shared_results) and not mismatched,
    }
    if not criteria['shared_poses_reproduce_exactly']:
        verdict = 'invalid: shared poses did not reproduce their base records'
    elif not criteria['no_new_pose_beats_winner_decisively']:
        verdict = ('fail: pool too small; grow it again or search near the new '
                   'winner, do not simply take the new pose')
    elif not criteria['winner_in_merged_top_3']:
        verdict = 'fail: base winner outside the merged top 3'
    else:
        verdict = 'pass'
    return {
        'criteria_text': STABILITY_CRITERIA,
        'margin_s': margin_s,
        'criteria': criteria,
        'verdict': verdict,
        'base_winner': {k: winner.get(k) for k in (
            'ready_index', 'provenance', 'connectivity', 'worst_family_fraction',
            'worst_duration_s', 'worst_peak_jerk', 'worst_duration_placement',
            'worst_duration_binding')},
        'winner_position_in_merged': position,
        'merged_pool_size': len(merged),
        'new_poses_above_winner': [
            {k: r.get(k) for k in ('ready_index', 'provenance', 'connectivity',
                                   'worst_family_fraction', 'worst_duration_s')}
            for r in merged[:position] if r['key'] not in base_keys],
        'decisive_beaters': len(decisive),
        'shared_poses_checked': len(shared_results),
        'shared_poses_mismatched': len(mismatched),
    }


def build_placements(validator, names, meter, candidates=None, csv_path=None):
    """Generate every named placement through the one path.

    names is a list of envelope names or "all". Nominal is generated first,
    unseeded, even when not listed, because its layers seed the rest;
    candidates substitutes a committed generator output for nominal.
    Returns (placements, context).
    """
    from ur10e_trajectory_pkg.failure_census import load_trajectory

    positions, quaternions, dt, trajectory_metadata = load_trajectory(
        csv_path, PREFIX_LAYERS, with_metadata=True)
    nominal_RG = trajectory_metadata['placement_RG']
    envelope = {p['name']: p for p in sweep.placements()}
    if names == 'all':
        names = [p['name'] for p in sweep.placements()]
    candidates_document = {}
    if candidates:
        layers, candidates_document = load_candidates(
            candidates, PREFIX_LAYERS, include_oracle=False)
        nominal = {'name': 'nominal', 'layers': layers, 'positions': positions,
                   'quaternions': quaternions, 'dt': dt,
                   'generation': {'source': 'committed full-length generator '
                                            'output (--candidates)'}}
    else:
        with counting_collisions(validator, meter):
            nominal = generated_placement(validator, envelope['nominal'],
                                          nominal_RG, meter)
        layers = nominal['layers']
    placements = []
    for name in names:
        if name == 'nominal':
            placements.append(nominal)
            continue
        with counting_collisions(validator, meter):
            placements.append(generated_placement(
                validator, envelope[name], nominal_RG, meter,
                nominal_layers=layers))
    return placements, {'candidates_document': candidates_document,
                        'trajectory_metadata': trajectory_metadata,
                        'nominal_RG': nominal_RG, 'dt': dt}


def load_poses(path):
    """Ready poses to evaluate, from a JSON list of configurations or records."""
    with open(path, encoding='utf-8') as handle:
        entries = json.load(handle)
    out = []
    for entry in entries:
        if not isinstance(entry, dict):
            entry = {'configuration': entry}
        out.append({'configuration': np.asarray(entry['configuration'], dtype=float),
                    'provenance': entry.get('provenance', 'given'),
                    'anchor_name': entry.get('anchor_name')})
    return out


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
             exhaustive=False, progress=False):
    """Prepare each placement, then evaluate every ready pose against it.

    progress prints one flushed line per ready pose with elapsed time and a
    naive ETA, since a full sweep otherwise says nothing until it ends.
    """
    started = time.perf_counter()
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
        if progress:
            elapsed = time.perf_counter() - started
            remaining = elapsed / (index + 1) * (len(ready_poses) - index - 1)
            print(f'progress: ready pose {index + 1}/{len(ready_poses)}, '
                  f'{elapsed / 60:.1f} min elapsed, about '
                  f'{remaining / 60:.1f} min remaining', flush=True)
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


def _binding_tally(bindings):
    counts = {}
    for binding in bindings:
        if not binding:
            continue
        key = (f"{binding['joint']}:{binding['kind']}:{binding['status']}"
               f"{'' if binding.get('active') else ':NOT_ACTIVE'}")
        counts[key] = counts.get(key, 0) + 1
    return dict(sorted(counts.items(), key=lambda item: -item[1]))


def aggregate(results, placement_records, meter, pose_seconds):
    per_pose = [r for result in results for r in result['per_placement']
                if 'counts' in r]
    queries = [q for r in per_pose
               for q in r['counts']['collision_queries_per_approach']]
    windings = [w for r in per_pose for w in r['counts']['winding_alternatives']]
    return {
        'binding_of_chosen_approaches': _binding_tally(
            b.get('binding') for r in per_pose for b in r['branches']
            if b['connected']),
        'binding_of_worst_durations': _binding_tally(
            result['summary'].get('worst_duration_binding')
            for result in results),
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
            'candidates_sha256': (file_digest(args.candidates)
                                  if args.candidates else None),
            'candidates_repository_revision':
                candidate_manifest.get('repository_revision'),
            'candidates_trajectory': candidate_manifest.get('trajectory'),
            'csv_path': trajectory_metadata['csv_path'],
            'csv_sha256': file_digest(trajectory_metadata['csv_path']),
            'trajectory': trajectory_metadata,
        },
        'envelope': {
            'name': 'PROVISIONAL_STAGE7_ENVELOPE_V2',
            'definition': sweep.PROVISIONAL_STAGE7_ENVELOPE_V2,
            'placements_evaluated': placement_names,
        },
        'pool': {
            'global_samples': args.pool_samples,
            'sobol_seed': 0,
            'broad_finalists': args.pool_finalists,
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
    parser.add_argument('--candidates', default=None,
                        help='substitute this committed generator output for '
                             'the generated nominal layers, as a check')
    parser.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    parser.add_argument('--pilot-count', type=int, default=8)
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--tolerance', type=float, default=0.35)
    parser.add_argument('--out', default='pilot.json')
    parser.add_argument('--placements', default='nominal',
                        help='comma-separated placement names from the '
                             'envelope, or "all"')
    parser.add_argument('--pool-samples', type=int, default=sweep.GLOBAL_SAMPLES,
                        help='Sobol samples screened by the static gates; '
                             'double it for the pool stability check')
    parser.add_argument('--pool-finalists', type=int,
                        default=sweep.BROAD_FINALISTS)
    parser.add_argument('--skip-poses-in', default=None,
                        help='evaluate only pool poses NOT in this artifact')
    parser.add_argument('--only-poses-in', default=None,
                        help='evaluate only pool poses that ARE in this artifact')
    parser.add_argument('--stability', nargs=3, metavar=('BASE', 'NEW', 'SHARED'),
                        default=None,
                        help='apply STABILITY_CRITERIA to three artifacts and exit')
    parser.add_argument('--poses-json', default=None,
                        help='evaluate exactly these ready poses instead of '
                             'a selection from the pool')
    parser.add_argument('--rail-velocity-cap', type=float, default=None,
                        help='sensitivity study: replace RAIL_VEL_SAFETY_CAP '
                             'for this run, before generation, so '
                             'continuations are re-derived under it')
    parser.add_argument('--all-pool', action='store_true',
                        help='evaluate every ready pose in the pool, not a '
                             'pilot selection')
    parser.add_argument('--no-exhaustive-check', action='store_true',
                        help='skip the exhaustive run that verifies pruning')
    args = parser.parse_args(argv)

    if args.stability:
        documents = []
        for path in args.stability:
            with open(path, encoding='utf-8') as handle:
                documents.append(json.load(handle))
        report = stability_check(documents[0]['results'], documents[1]['results'],
                                 documents[2]['results'])
        report['artifacts'] = {name: {'path': path,
                                      'revision': doc['manifest']['repository_revision'],
                                      'envelope': doc['manifest']['envelope']['name']}
                               for name, path, doc in zip(('base', 'new', 'shared'),
                                                          args.stability, documents)}
        with open(args.out, 'w', encoding='utf-8') as handle:
            json.dump(report, handle, indent=1, default=_plain)
        print(json.dumps(report, indent=1, default=_plain))
        return 0

    from ament_index_python.packages import get_package_share_directory

    from ur10e_trajectory_pkg.failure_census import _validator, load_trajectory

    total_start = time.perf_counter()
    validator = _validator(args.urdf, get_package_share_directory('ur_description'))
    if args.rail_velocity_cap is not None:
        validator.set_rail_velocity_cap(args.rail_velocity_cap)
    envelope = {p['name']: p for p in sweep.placements()}
    names = ([p['name'] for p in sweep.placements()]
             if args.placements.strip() == 'all'
             else [n.strip() for n in args.placements.split(',') if n.strip()])
    unknown = [n for n in names if n not in envelope]
    if unknown:
        parser.error(f'unknown placements {unknown}')

    generation_meter = Meter()
    placements, context = build_placements(validator, names, generation_meter,
                                           args.candidates)
    candidates_document = context['candidates_document']
    trajectory_metadata, dt = context['trajectory_metadata'], context['dt']
    generated = [p['generation']['seconds'] for p in placements
                 if 'seconds' in p['generation']]

    pool_meter = Meter()
    with counting_collisions(validator, pool_meter), \
            counting_static_gates(pool_meter), pool_meter.phase('pool'):
        pool, pool_summary = sweep.build_candidate_pool(
            validator, count=args.pool_samples, finalists=args.pool_finalists)
    # The pilot selection keeps one anchor per rail position, so asking it
    # for the whole pool would silently drop the others.
    full_pool_size = len(pool)
    if args.skip_poses_in or args.only_poses_in:
        pool = filter_pool(
            pool,
            skip_keys=(artifact_pose_keys(args.skip_poses_in)
                       if args.skip_poses_in else None),
            only_keys=(artifact_pose_keys(args.only_poses_in)
                       if args.only_poses_in else None))
    if args.poses_json:
        pilot = load_poses(args.poses_json)
    else:
        pilot = list(pool) if args.all_pool else pilot_ready_poses(
            pool, args.pilot_count)

    runs = []
    for _ in range(args.repeats):
        meter = Meter()
        with counting_collisions(validator, meter), counting_static_gates(meter):
            results, placement_records, pose_seconds = run_once(
                validator, pilot, placements, meter, args.tolerance,
                progress=True)
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
        'ready_pose_selection': ('entire pool' if args.all_pool
                                 else PILOT_SELECTION_RULE),
        'rail_velocity_cap_override': args.rail_velocity_cap,
        'poses_json': args.poses_json,
        'pose_filter': {'skip_poses_in': args.skip_poses_in,
                        'only_poses_in': args.only_poses_in,
                        'pool_size_before_filter': full_pool_size,
                        'evaluated': len(pilot)},
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
