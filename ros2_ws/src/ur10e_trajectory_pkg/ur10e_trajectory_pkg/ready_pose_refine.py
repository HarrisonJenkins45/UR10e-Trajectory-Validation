#!/usr/bin/env python3
"""Local refinement of ready-pose finalists, under rules fixed in advance.

The V2 sweep's top is flat and set by design rather than by region. Its two
leaders, the compact and reach_forward anchors at rail 1.5 m, both take their
worst duration from one candidate at coupled_00, bound by certified elbow
velocity; their 0.1005 s gap is the 10 degrees of elbow between the two
catalogue postures, and each pose's worst three placements lie within 0.026 s.
The pool adds broad samples chosen for diversity, not quality, so neither a
pass nor a fail of the stability check says whether the winner is near the
best available. This searches locally around the finalists instead.

REFINEMENT_RULES, fixed before the stability verdict was seen:

  starting points  the base sweep's top 7, plus any new pose in the merged
                   top 3 of base and new-pose artifacts
  search           best-improvement coordinate search over the rail and each
                   arm joint, in steps of 5 deg and 0.05 m, then 2 deg and
                   0.02 m; every trial passes joint limits and static gates
  objective        the ranking key over all placements: connectivity, worst
                   family fraction, worst duration, worst jerk
  acceptance       a refined pose is accepted only if its worst family
                   fraction is 1.0 and its worst duration beats the unrefined
                   winner's by more than STABILITY_MARGIN_S, and a full run
                   with the exhaustive check confirms it. Otherwise the
                   unrefined winner stays: an anchor posture is easier to
                   justify and reproduce

A search that stops has found no improving single-joint step, which is not
proof of a local optimum. The objective is a worst case over placements, so it
can stall on a ridge where two placements need one joint moved in opposite
directions, or two joints moved together; pose 71's three slowest placements
are all elbow-bound within 0.026 s, which is where such a ridge is likely.

Trials are pruned exactly. Every key is a worst case over placements, so a
partial evaluation bounds the final one: connectivity can only fall with each
unconnected placement, the family fraction only fall, the worst duration and
jerk only rise. A trial is abandoned once those bounds show it cannot beat the
best found so far, with placements taken in the order the current pose finds
hardest. Pruning never changes which trial wins.

Usage:
    python3 -m ur10e_trajectory_pkg.ready_pose_refine \\
        --base sweep_v2.json --new sweep_v2_new.json --out refinement.json
"""
import argparse
import json
import sys
import time

import numpy as np

from ur10e_trajectory_pkg import ready_pose_runner as runner
from ur10e_trajectory_pkg import ready_pose_sweep as sweep

SCHEMA_VERSION = 1
REFINE_STEPS = ((5.0, 0.05), (2.0, 0.02))       # (degrees, metres)
MAX_PASSES_PER_STEP = 25
BASE_STARTS = 7
NEW_POSE_TOP = 3
ACCEPT_MARGIN_S = runner.STABILITY_MARGIN_S
REFINEMENT_RULES = (
    'Fixed before the stability verdict was seen. Starts: base top 7 plus any '
    'new pose in the merged top 3. Search: best-improvement coordinate search '
    'over rail and arm joints, 5 deg / 0.05 m then 2 deg / 0.02 m, every trial '
    'within joint limits and passing static gates. Objective: the ranking key '
    'over all placements. Accept a refined pose only with worst family '
    'fraction 1.0 and worst duration beating the unrefined winner by more '
    'than STABILITY_MARGIN_S, confirmed by a full run with the exhaustive '
    'check; otherwise keep the unrefined winner.')
_EPS = 1e-9


def rank_key(summary):
    """The rank_ready_poses ordering as a tuple; smaller is better."""
    def value(name, missing):
        return missing if summary.get(name) is None else summary[name]
    return (-value('connectivity', -1.0), -value('worst_family_fraction', -1.0),
            value('worst_duration_s', np.inf), value('worst_peak_jerk', np.inf))


def cannot_beat(partial, incumbent, eligible_placements):
    """Whether a partial evaluation already proves the trial cannot win.

    Bounds, each valid because the full key is a worst case over placements:
    final connectivity is at most (eligible - unconnected so far) / eligible,
    final family fraction at most the partial minimum, final worst duration
    and jerk at least the partial maxima.
    """
    unconnected = sum(1 for r in partial
                      if r['classification'] == sweep.DIRECT_APPROACH_UNCONNECTED)
    best_connectivity = ((eligible_placements - unconnected) / eligible_placements
                         if eligible_placements else 0.0)
    connected = [r for r in partial if r['classification'] == sweep.CONNECTED]
    eligible = [r for r in partial
                if r['classification'] in (sweep.CONNECTED,
                                           sweep.DIRECT_APPROACH_UNCONNECTED)]
    best_fraction = min((r['connected_families'] / r['family_count']
                         if r.get('family_count') else 0.0 for r in eligible),
                        default=1.0)
    incumbent_connectivity, incumbent_fraction = (-rank_key(incumbent)[0],
                                                  -rank_key(incumbent)[1])
    if best_connectivity < incumbent_connectivity - _EPS:
        return True
    if best_connectivity > incumbent_connectivity + _EPS:
        return False
    if best_fraction < incumbent_fraction - _EPS:
        return True
    if best_fraction > incumbent_fraction + _EPS:
        return False
    if not connected:
        return False
    duration = max(r['slowest_family_duration_s'] for r in connected)
    incumbent_duration = rank_key(incumbent)[2]
    if duration > incumbent_duration + _EPS:
        return True
    if duration < incumbent_duration - _EPS:
        return False
    jerk = max(r['family_shortest_max_peak_jerk'] for r in connected)
    return jerk > rank_key(incumbent)[3] + _EPS


def placement_order(per_placement):
    """Hardest placements for the current pose first, so pruning bites early."""
    def key(record):
        connected = record['classification'] == sweep.CONNECTED
        fraction = (record['connected_families'] / record['family_count']
                    if record.get('family_count') else 0.0)
        return (connected, fraction,
                -(record.get('slowest_family_duration_s') or 0.0),
                record['placement'])
    return [r['placement'] for r in sorted(per_placement, key=key)]


def make_evaluator(validator, prepared, meter):
    """evaluate(configuration, order, incumbent) -> (summary, records, complete)."""
    by_name = {p['name']: p for p in prepared}
    eligible = sum(1 for p in prepared if p['status'] == 'ready')
    names = [p['name'] for p in prepared]

    def evaluate(configuration, order=None, incumbent=None):
        records = []
        for name in (order or names):
            records.append(runner.evaluate_ready_pose(
                validator, configuration, by_name[name], meter))
            if incumbent is not None and cannot_beat(records, incumbent, eligible):
                return None, records, False
        return runner.summarise_ready_pose(records), records, True

    return evaluate


def make_gate(validator):
    lower, upper = validator.robot.qlim

    def admissible(configuration):
        if np.any(configuration < lower) or np.any(configuration > upper):
            return False
        return bool(sweep.static_gates(validator, configuration)['passed'])

    return admissible


def coordinate_search(start, evaluate, admissible, steps=REFINE_STEPS,
                      max_passes=MAX_PASSES_PER_STEP, prune=True):
    """Best-improvement coordinate search on the ranking key.

    evaluate and admissible are injected so the search can be tested without
    the robot. Deterministic: joints in order, + before -, ties to the first.
    """
    current = np.asarray(start, dtype=float)
    summary, records, _ = evaluate(current)
    history = {'start_summary': summary, 'moves': [], 'trials': 0,
               'pruned': 0, 'inadmissible': 0}
    history['stops'] = []
    for degrees, metres in steps:
        stop = f'max passes ({max_passes}) reached'
        for _ in range(max_passes):
            order = placement_order(records)
            best = None
            for joint in range(len(current)):
                step = metres if joint == 0 else np.deg2rad(degrees)
                for sign in (1.0, -1.0):
                    trial = current.copy()
                    trial[joint] += sign * step
                    if not admissible(trial):
                        history['inadmissible'] += 1
                        continue
                    incumbent = best[1] if best else summary
                    history['trials'] += 1
                    trial_summary, trial_records, complete = evaluate(
                        trial, order, incumbent if prune else None)
                    if not complete:
                        history['pruned'] += 1
                        continue
                    if rank_key(trial_summary) < rank_key(incumbent):
                        best = (trial, trial_summary, trial_records)
            if best is None:
                stop = 'no improving single-joint step'
                break
            current, summary, records = best
            history['moves'].append({'step_deg': degrees, 'step_m': metres,
                                     'configuration': current.tolist(),
                                     'key': list(rank_key(summary))})
        history['stops'].append({'step_deg': degrees, 'step_m': metres,
                                 'stop': stop})
    history['final_configuration'] = current.tolist()
    history['final_summary'] = summary
    return current, summary, records, history


def accept(refined_summary, unrefined_winner_summary, margin=ACCEPT_MARGIN_S):
    """Acceptance under REFINEMENT_RULES, before confirmation."""
    fraction = refined_summary.get('worst_family_fraction')
    connectivity = refined_summary.get('connectivity')
    gain = (rank_key(unrefined_winner_summary)[2]
            - rank_key(refined_summary)[2])
    checks = {
        'connectivity_is_one': connectivity is not None
                               and abs(connectivity - 1.0) <= _EPS,
        'family_fraction_is_one': fraction is not None
                                  and abs(fraction - 1.0) <= _EPS,
        'beats_margin': gain > margin,
    }
    return dict(checks, duration_gain_s=gain,
                accepted_pending_confirmation=bool(all(checks.values())))


def starting_points(base_results, new_results=None):
    """Base top BASE_STARTS, plus new poses in the merged top NEW_POSE_TOP."""
    base = runner._ranking_records(base_results)
    ranked = sweep.rank_ready_poses(base)
    by_key = {runner.pose_key(r['configuration']): r for r in base_results}
    starts = [dict(by_key[r['key']], source='base_top')
              for r in ranked[:BASE_STARTS]]
    if new_results:
        base_keys = {r['key'] for r in base}
        merged = sweep.rank_ready_poses(base + runner._ranking_records(new_results))
        new_by_key = {runner.pose_key(r['configuration']): r for r in new_results}
        starts += [dict(new_by_key[r['key']], source='new_in_merged_top')
                   for r in merged[:NEW_POSE_TOP] if r['key'] not in base_keys]
    return starts


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', required=True)
    parser.add_argument('--new', default=None)
    parser.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    parser.add_argument('--out', default='refinement.json')
    parser.add_argument('--confirm-out', default='refined_poses.json',
                        help='poses for the confirming runner --poses-json run')
    args = parser.parse_args(argv)

    from ament_index_python.packages import get_package_share_directory

    from ur10e_trajectory_pkg.failure_census import _validator

    with open(args.base, encoding='utf-8') as handle:
        base = json.load(handle)
    new = None
    if args.new:
        with open(args.new, encoding='utf-8') as handle:
            new = json.load(handle)

    started = time.perf_counter()
    validator = _validator(args.urdf, get_package_share_directory('ur_description'))
    meter = runner.Meter()
    placements, _ = runner.build_placements(validator, 'all', meter)
    prepared = [runner.prepare_placement(
        validator, p['name'], p['layers'], p['positions'], p['quaternions'],
        p['dt'], meter, layer0_extra_seed_only=p.get('layer0_extra_seed_only'))
        for p in placements]
    evaluate = make_evaluator(validator, prepared, meter)
    admissible = make_gate(validator)

    outcomes = []

    def write(final):
        # Rewritten after every start, so a late failure loses one start, not
        # the whole run.
        document = {
            'schema_version': SCHEMA_VERSION, 'complete': final,
            'rules': REFINEMENT_RULES, 'steps': [list(s) for s in REFINE_STEPS],
            'margin_s': ACCEPT_MARGIN_S,
            'base_artifact': args.base, 'new_artifact': args.new,
            'outcomes': outcomes,
            'wall_seconds': time.perf_counter() - started,
        }
        if final and outcomes:
            unrefined = min(outcomes, key=lambda o: tuple(o['start_key']))
            refined = min(outcomes, key=lambda o: tuple(o['final_key']))
            document.update(
                unrefined_winner={k: unrefined[k] for k in (
                    'source', 'ready_index', 'provenance', 'anchor_name',
                    'start_key', 'start_configuration')},
                best_refined={k: refined[k] for k in (
                    'source', 'ready_index', 'provenance', 'anchor_name',
                    'final_key')},
                best_refined_configuration=refined['history']['final_configuration'],
                decision=accept(refined['history']['final_summary'],
                                unrefined['history']['start_summary']))
        with open(args.out, 'w', encoding='utf-8') as handle:
            json.dump(document, handle, indent=1, default=runner._plain)
        return document

    for start in starting_points(base['results'], new['results'] if new else None):
        t0 = time.perf_counter()
        configuration = np.asarray(start['configuration'], dtype=float)
        _, summary, _, history = coordinate_search(configuration, evaluate,
                                                   admissible)
        reproduces = rank_key(history['start_summary']) == rank_key(start['summary'])
        outcomes.append({
            'source': start['source'], 'ready_index': start['ready_index'],
            'provenance': start['provenance'],
            'anchor_name': start.get('anchor_name'),
            'start_configuration': configuration.tolist(),
            'start_key': list(rank_key(history['start_summary'])),
            'start_reproduces_artifact': reproduces,
            'final_key': list(rank_key(summary)),
            'history': history,
            'seconds': time.perf_counter() - t0,
        })
        print(f"start {start['source']} pose {start['ready_index']}: "
              f"{rank_key(history['start_summary'])} -> {rank_key(summary)} "
              f"in {len(history['moves'])} moves, {history['trials']} trials "
              f"({history['pruned']} pruned), reproduces={reproduces}, "
              f"{outcomes[-1]['seconds'] / 60:.1f} min; stops "
              f"{[s['stop'] for s in history['stops']]}", flush=True)
        write(final=False)

    document = write(final=True)
    with open(args.confirm_out, 'w', encoding='utf-8') as handle:
        json.dump([{'configuration': document['best_refined_configuration'],
                    'provenance': 'refined', 'anchor_name': None},
                   {'configuration': document['unrefined_winner']['start_configuration'],
                    'provenance': document['unrefined_winner']['provenance'],
                    'anchor_name': document['unrefined_winner']['anchor_name']}],
                  handle, indent=1)
    print(json.dumps({k: document[k] for k in ('unrefined_winner', 'best_refined',
                                               'decision')},
                     indent=1, default=runner._plain))
    return 0


if __name__ == '__main__':
    sys.exit(main())
