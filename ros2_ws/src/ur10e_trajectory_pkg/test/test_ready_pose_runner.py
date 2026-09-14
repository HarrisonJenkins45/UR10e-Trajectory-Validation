#!/usr/bin/env python3
"""Stage 7 runner gate: cheapest checks first, and nothing counted twice.

Synthetic placements built by forward kinematics, so every expectation is
known in advance and the suite stays fast.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg import ready_pose_runner as runner
from ur10e_trajectory_pkg import ready_pose_sweep as sweep
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

DT = 0.1
STEP = np.concatenate(([0.0], np.full(6, 0.004)))


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description')
    )


def _pose(validator, configuration):
    pose = validator.robot.fkine(configuration, end='tool0')
    return pose.t, np.roll(np.array(pose.UnitQuaternion().A), -1)


@pytest.fixture(scope='module')
def branch_a():
    return np.concatenate(([1.0], np.deg2rad([0.0, -120.0, 90.0, -80.0, 60.0, 0.0])))


@pytest.fixture(scope='module')
def branch_b(validator, branch_a):
    """branch_a's other-shoulder solution for the SAME pose at the same rail.

    Layer-0 candidates are solutions of the layer-0 pose by construction, and
    IK families are found by re-solving that pose, so a fixture posture that
    is not a solution would be refined onto another family and merged.
    """
    pose = validator.robot.fkine(branch_a, end='tool0')
    seed = np.concatenate(([branch_a[0]],
                           [-2.0933, -2.6974, 1.3721, -0.5829, 2.1016, 1.9063]))
    solution, error, _ = sweep.arm_only_ik(validator, seed, pose.t, pose.R)
    assert error < 1e-9
    return solution


@pytest.fixture(scope='module')
def placement(validator, branch_a, branch_b):
    """Two branches; layer 1 also holds a costlier decoy continuation of A,
    and layer 0 holds a candidate with no continuation at all."""
    orphan = np.concatenate(([2.5], np.deg2rad([-90.0, -60.0, 60.0, -90.0, -90.0, 0.0])))
    layers = [
        [branch_a, branch_b, orphan],
        [branch_a + STEP, branch_a + 3 * STEP, branch_b + STEP],
        [branch_a + 2 * STEP, branch_b + 2 * STEP],
    ]
    first, second = _pose(validator, branch_a), _pose(validator, branch_a + STEP)
    return {'name': 'synthetic', 'layers': layers,
            'positions': np.stack([first[0], second[0], second[0]]),
            'quaternions': np.stack([first[1], second[1], second[1]]),
            'dt': DT}


@pytest.fixture(scope='module')
def prepared(validator, placement):
    meter = runner.Meter()
    state = runner.prepare_placement(
        validator, placement['name'], placement['layers'],
        placement['positions'], placement['quaternions'], DT, meter)
    return state


@pytest.fixture(scope='module')
def ready(branch_a):
    offset = np.concatenate(([0.1], np.deg2rad([5.0, 5.0, -5.0, 5.0, 5.0, 5.0])))
    return branch_a + offset


# --------------------------------------------------------------------------
# Per placement
# --------------------------------------------------------------------------

def test_a_candidate_without_continuation_is_dropped_before_classifying(prepared):
    """It is not a task candidate, so it must not count against a ready pose."""
    assert prepared['status'] == 'ready'
    assert prepared['counts']['task_gate_rejections'] == {'no_continuation': 1}
    assert [v['candidate_index'] for v in prepared['valid']] == [0, 1]


def test_the_cheapest_continuation_is_kept_and_alternatives_counted(prepared, branch_a):
    entry = prepared['valid'][0]
    np.testing.assert_allclose(entry['prefix'][1], branch_a + STEP, atol=1e-12)
    np.testing.assert_allclose(entry['prefix'][2], branch_a + 2 * STEP, atol=1e-12)
    assert entry['continuation_alternatives'] >= 2


def test_surviving_candidates_carry_branch_labels(prepared):
    assert prepared['counts']['branch_clusters'] == 2
    assert {v['branch'] for v in prepared['valid']} == {0, 1}
    # Two different postures at one rail position: two families.
    assert {v['family'] for v in prepared['valid']} == {0, 1}
    assert prepared['counts']['ik_families'] == 2


def test_a_placement_found_only_through_seeds_is_still_a_task_placement(
        validator, placement):
    """A seeded candidate passing every gate proves there is something to
    connect to; that the generator alone found nothing is its own record."""
    state = runner.prepare_placement(
        validator, placement['name'], placement['layers'],
        placement['positions'], placement['quaternions'], DT, runner.Meter(),
        layer0_extra_seed_only=[True, True, True])
    assert state['status'] == 'ready'
    assert state['counts']['generator_alone_found_nothing'] is True
    assert state['counts']['ik_families'] == 2


def test_a_twist_the_arm_cannot_make_leaves_no_task_candidate(validator, placement,
                                                              ready):
    """170 degrees in one 0.1 s step is far beyond any wrist, so alpha* < 1 for
    every candidate, and no ready pose is evaluated against the placement."""
    from scipy.spatial.transform import Rotation

    quaternions = placement['quaternions'].copy()
    quaternions[1] = (Rotation.from_quat(quaternions[0])
                      * Rotation.from_euler('z', 170, degrees=True)).as_quat()
    meter = runner.Meter()
    state = runner.prepare_placement(
        validator, 'fast', placement['layers'], placement['positions'],
        quaternions, DT, meter)
    assert state['status'] == sweep.NO_TASK_CANDIDATE
    assert state['counts']['task_gate_rejections'].get('alpha_star', 0) == 3

    with runner.counting_collisions(validator, meter):
        result = runner.evaluate_ready_pose(validator, ready, state, meter)
    assert result['classification'] == sweep.NO_TASK_CANDIDATE
    assert not any(k.startswith('collision_queries') for k in meter.counts)


def test_a_continuation_entering_beyond_a_limit_is_dropped_per_placement(
        validator, placement, branch_a):
    """Entry state is invariant under a lift, so it is decided once here. A
    rail step of 0.09 then 0 gives PCHIP an entry velocity of 1.35 m/s against
    the 1.0 cap, and no approach from rest can end in that."""
    step = np.zeros(7)
    step[0] = 0.09
    layers = [[branch_a], [branch_a + step], [branch_a + step]]
    state = runner.prepare_placement(
        validator, 'fast_rail', layers, placement['positions'],
        placement['quaternions'], DT, runner.Meter())
    assert state['status'] == sweep.NO_TASK_CANDIDATE
    assert state['counts']['task_gate_rejections'] == {
        'entry_state_exceeds_limits': 1}


def test_branches_reached_only_through_extra_seeds_are_counted_apart(
        validator, placement, ready):
    """A branch whose every member came from another placement's seeds says
    the seeds carried over, not that the generator found it here."""
    meter = runner.Meter()
    state = runner.prepare_placement(
        validator, placement['name'], placement['layers'],
        placement['positions'], placement['quaternions'], DT, meter,
        layer0_extra_seed_only=[False, True, False])
    assert state['counts']['branch_clusters'] == 2
    assert state['counts']['branch_clusters_independent'] == 1
    assert state['counts']['valid_candidates_extra_seed_only'] == 1
    assert state['counts']['generator_alone_found_nothing'] is False
    # Families count over the full valid set; the seed-only one is a
    # diagnostic, not excluded.
    assert state['counts']['ik_families'] == 2
    assert state['counts']['ik_families_independent'] == 1
    assert len(state['counts']['ik_families_only_from_extra_seeds']) == 1

    result = runner.evaluate_ready_pose(validator, ready, state, meter)
    assert result['connected_branches'] == 2
    assert result['connected_independent_branches'] == 1
    assert result['connected_families'] == 2
    summary = runner.summarise_ready_pose([result])
    assert summary['worst_branch_count'] == 2
    assert summary['worst_independent_branch_count'] == 1


def test_a_placement_without_layers_is_refused_not_invented(validator, placement):
    state = runner.prepare_placement(
        validator, 'translate_x+', None, placement['positions'],
        placement['quaternions'], DT, runner.Meter())
    assert state['status'] == runner.NO_LAYERS


# --------------------------------------------------------------------------
# Per ready pose
# --------------------------------------------------------------------------

def test_a_nearby_ready_pose_connects_to_both_branches(validator, prepared, ready):
    result = runner.evaluate_ready_pose(validator, ready, prepared, runner.Meter())
    assert result['classification'] == sweep.CONNECTED
    assert result['connected_branches'] == 2
    assert result['best_duration_s'] > 0
    assert all(q > 0 for q in result['counts']['collision_queries_per_approach'])


def test_collision_is_checked_in_duration_order_and_stops_at_first_feasible(
        validator, prepared, ready, monkeypatch):
    """The first feasible approach checked is therefore the branch's shortest;
    everything after it is skipped and counted."""
    seen = []
    original = sweep.evaluate_approach

    def first_one_collides(*args, **kwargs):
        seen.append(kwargs['duration'])
        if len(seen) == 1:
            return {'feasible': False, 'reason_code': sweep.REASON_COLLISION,
                    'duration_s': kwargs['duration'], 'collision_queries': 1}
        return original(*args, **kwargs)

    monkeypatch.setattr(sweep, 'evaluate_approach', first_one_collides)
    single = dict(prepared, valid=[prepared['valid'][0]])
    result = runner.evaluate_ready_pose(validator, ready, single, runner.Meter())

    branch = result['branches'][0]
    counts = result['counts']
    assert len(seen) == 2
    assert seen[0] <= seen[1]
    assert branch['connected'] is True
    assert branch['duration_s'] == pytest.approx(seen[1])
    # Pre-collision rejections (joint limits, duration) are listed too; only
    # one alternative reached the collision check and failed there.
    assert branch['failure_breakdown'].get(sweep.REASON_COLLISION) == 1
    # Every alternative is accounted for exactly once.
    assert branch['alternatives'] == (branch['collision_checked']
                                      + branch['rejected_before_collision']
                                      + counts['timed_not_collision_checked']
                                      + counts['untimed_by_bound'])


def test_pruning_by_bound_changes_no_branch_outcome(validator, prepared, ready):
    """The lazy order must pick exactly what timing everything would, while
    leaving some alternatives untimed."""
    pruned = runner.evaluate_ready_pose(validator, ready, prepared, runner.Meter())
    full = runner.evaluate_ready_pose(validator, ready, prepared, runner.Meter(),
                                      exhaustive=True)
    wrap = lambda r: [{'per_placement': [r]}]
    assert runner.branch_outcomes(wrap(pruned)) == runner.branch_outcomes(wrap(full))
    assert pruned['counts']['untimed_by_bound'] > 0
    assert full['counts']['untimed_by_bound'] == 0
    assert (pruned['counts']['alternatives_timed']
            < full['counts']['alternatives_timed'])


def test_the_cached_entry_state_is_the_lifted_prefixs(validator, prepared, ready):
    """Entry state is invariant under a lift, which is what makes caching it
    per candidate exact rather than an approximation."""
    entry = prepared['valid'][0]
    lifts = sweep.destination_lifts(validator, entry['configuration'], ready,
                                    validator.velocity_limits, 20.0)
    # A lift whose continuation leaves the limits has no prefix to compare.
    prefixes = [p for p in (sweep.lifted_prefix(validator, entry['prefix'], lift)
                            for lift in lifts) if p is not None]
    assert len(prefixes) > 1
    for prefix in prefixes:
        velocity, acceleration = sweep.entry_state_from_prefix(prefix, DT)
        np.testing.assert_allclose(velocity, entry['entry_velocity'], atol=1e-9)
        np.testing.assert_allclose(acceleration, entry['entry_acceleration'],
                                   atol=1e-9)


def test_every_collision_query_is_counted_by_phase(validator, prepared, ready):
    meter = runner.Meter()
    with runner.counting_collisions(validator, meter):
        result = runner.evaluate_ready_pose(validator, ready, prepared, meter)
    assert (meter.counts['collision_queries:collision']
            == sum(result['counts']['collision_queries_per_approach']))
    assert 'check_all_collisions' not in vars(validator)


def test_counting_restores_an_existing_instance_override(validator, monkeypatch):
    stub = lambda q, verbose=False: False
    monkeypatch.setattr(validator, 'check_all_collisions', stub, raising=False)
    meter = runner.Meter()
    with runner.counting_collisions(validator, meter):
        validator.check_all_collisions(np.zeros(7))
    assert validator.check_all_collisions is stub
    assert meter.counts == {'collision_queries:unphased': 1}


def test_the_run_is_deterministic(validator, placement, ready):
    poses = [{'configuration': ready, 'provenance': 'broad_sample',
              'anchor_name': None}]
    first = runner.run_once(validator, poses, [placement], runner.Meter())[0]
    second = runner.run_once(validator, poses, [placement], runner.Meter())[0]
    assert (runner.results_without_timing(first)
            == runner.results_without_timing(second))


# --------------------------------------------------------------------------
# Summaries, selection, projection
# --------------------------------------------------------------------------

def test_no_task_candidate_placements_do_not_count_against_a_ready_pose():
    per_placement = [
        {'classification': sweep.CONNECTED, 'connected_branches': 3,
         'connected_families': 2, 'family_count': 4, 'placement': 'a',
         'best_duration_s': 2.0, 'best_max_peak_jerk': 40.0,
         'slowest_family_duration_s': 2.8, 'family_shortest_max_peak_jerk': 45.0},
        {'classification': sweep.NO_TASK_CANDIDATE},
        {'classification': sweep.CONNECTED, 'connected_branches': 1,
         'connected_families': 1, 'family_count': 1, 'placement': 'b',
         'best_duration_s': 3.5, 'best_max_peak_jerk': 20.0,
         'slowest_family_duration_s': 3.5, 'family_shortest_max_peak_jerk': 20.0},
    ]
    summary = runner.summarise_ready_pose(per_placement)
    assert summary['connectivity'] == 1.0
    assert summary['worst_family_count'] == 1
    # 2 of 4 is worse than 1 of 1: the fraction, not the count, ranks.
    assert summary['worst_family_fraction'] == 0.5
    assert summary['worst_duration_placement'] == 'b'
    assert summary['worst_branch_count'] == 1
    assert summary['worst_duration_s'] == 3.5
    assert summary['worst_peak_jerk'] == 45.0
    assert summary['worst_shortest_approach_s'] == 3.5
    assert summary['worst_shortest_approach_jerk'] == 40.0


def _branch(label, family, duration, jerk, connected=True):
    return {'branch': label, 'family': family, 'connected': connected,
            'duration_s': duration if connected else None,
            'max_peak_jerk': jerk if connected else None}


def test_a_familys_shortest_approach_is_the_minimum_over_its_branches():
    summary = runner.family_approach_summary([
        _branch(0, 0, 1.2, 30.0), _branch(1, 0, 0.9, 50.0),
        _branch(2, 1, 2.4, 10.0), _branch(3, 1, 0.5, 90.0, connected=False),
    ])
    assert summary['family_shortest_duration_s'] == {'0': 0.9, '1': 2.4}
    assert summary['slowest_family_duration_s'] == 2.4
    assert summary['family_shortest_max_peak_jerk'] == 50.0


def test_a_slow_family_outranks_one_fast_entry():
    """Ranking asks whether a pose reaches several families cheaply. One very
    fast approach must not hide a family that takes much longer."""
    def record(name, branches):
        per_placement = dict({'classification': sweep.CONNECTED,
                              'placement': 'p',
                              'connected_branches': len(branches),
                              'connected_families': len({b['family'] for b in branches}),
                              'family_count': len({b['family'] for b in branches}),
                              'best_duration_s': min(b['duration_s'] for b in branches),
                              'best_max_peak_jerk': 10.0},
                             **runner.family_approach_summary(branches))
        return dict(runner.summarise_ready_pose([per_placement]), name=name,
                    static_gates={'passed': True})

    one_fast_entry = record('one_fast_entry', [_branch(0, 0, 0.8, 20.0),
                                               _branch(1, 1, 3.0, 20.0)])
    evenly_fast = record('evenly_fast', [_branch(0, 0, 1.5, 20.0),
                                         _branch(1, 1, 1.6, 20.0)])
    assert one_fast_entry['worst_shortest_approach_s'] < evenly_fast['worst_shortest_approach_s']
    ranked = sweep.rank_ready_poses([one_fast_entry, evenly_fast])
    assert [r['name'] for r in ranked] == ['evenly_fast', 'one_fast_entry']


def test_pilot_selection_spans_rail_and_provenance():
    pool = [{'configuration': np.array([rail, 0, 0, 0, 0, 0, 0.0]),
             'provenance': 'broad_sample', 'anchor_name': None}
            for rail in np.linspace(0.05, 2.95, 32)]
    pool += [{'configuration': np.array([rail, 0, 0, 0, 0, 0, 0.0]),
              'provenance': 'anchor', 'anchor_name': name}
             for name in ('first', 'second') for rail in (0.5, 1.5, 2.5)]
    chosen = runner.pilot_ready_poses(pool)
    assert len(chosen) == 8
    assert sum(1 for c in chosen if c['provenance'] == 'anchor') == 3
    assert len({c['anchor_name'] for c in chosen if c['anchor_name']}) >= 2
    strata = {int(c['configuration'][0] / 3.0 * 8) for c in chosen}
    assert len(strata) >= 7
    assert [id(c) for c in chosen] == [id(c) for c in runner.pilot_ready_poses(pool)]


def test_nominal_seeds_carry_their_origin():
    layers = [[np.zeros(7), np.ones(7)], [np.full(7, 2.0)], []]
    seeds = runner.nominal_seeds(layers)
    assert [len(layer) for layer in seeds] == [2, 1, 0]
    assert seeds[0][1][1] == 'nominal:layer0:candidate1'
    np.testing.assert_array_equal(seeds[1][0][0], np.full(7, 2.0))


def test_projection_charges_generation_to_every_placement():
    projection = runner.project_runtime(10.0, 2.0, [1.0], pool_size=50,
                                        placements=29, generation_seconds=3.0)
    assert projection['fixed_seconds'] == pytest.approx(10.0 + 29 * 5.0)
    assert projection['excludes'] is None


def test_projection_scales_pool_and_placements():
    projection = runner.project_runtime(10.0, 2.0, [1.0, 1.0, 3.0], pool_size=50,
                                        placements=29)
    assert projection['ready_pose_placement_evaluations'] == 1450
    assert projection['fixed_seconds'] == pytest.approx(68.0)
    assert projection['p50_seconds'] == pytest.approx(68.0 + 1450 * 1.0)
    assert projection['p95_seconds'] > projection['p50_seconds']


# --------------------------------------------------------------------------
# Pool stability
# --------------------------------------------------------------------------

def _result(index, configuration, fraction, duration, connectivity=1.0,
            record='same'):
    return {'ready_index': index, 'configuration': configuration,
            'provenance': 'broad_sample',
            'per_placement': [{'placement': 'p', 'record': record}],
            'summary': {'connectivity': connectivity,
                        'worst_family_fraction': fraction,
                        'worst_duration_s': duration, 'worst_peak_jerk': 10.0}}


def _base():
    return [_result(0, [0.0] * 7, 1.0, 2.0), _result(1, [1.0] * 7, 1.0, 2.5),
            _result(2, [2.0] * 7, 0.8, 1.0)]


def test_filter_pool_skips_or_keeps_by_configuration():
    pool = [{'configuration': np.full(7, float(i))} for i in range(4)]
    keys = {runner.pose_key(np.full(7, 1.0)), runner.pose_key(np.full(7, 3.0))}
    assert [e['configuration'][0] for e in runner.filter_pool(pool, skip_keys=keys)] == [0.0, 2.0]
    assert [e['configuration'][0] for e in runner.filter_pool(pool, only_keys=keys)] == [1.0, 3.0]


def test_stability_passes_when_no_new_pose_beats_the_winner_decisively():
    base = _base()
    new = [_result(10, [5.0] * 7, 1.0, 1.95)]          # faster, within margin
    report = runner.stability_check(base, new, [base[0]])
    assert report['winner_position_in_merged'] == 1
    assert report['criteria'] == {'winner_in_merged_top_3': True,
                                  'no_new_pose_beats_winner_decisively': True,
                                  'shared_poses_reproduce_exactly': True}
    assert report['verdict'] == 'pass'


def test_a_decisively_better_new_pose_means_the_pool_was_too_small():
    base = _base()
    new = [_result(10, [5.0] * 7, 1.0, 1.5)]            # 0.5 s faster
    report = runner.stability_check(base, new, [base[0]])
    assert report['criteria']['no_new_pose_beats_winner_decisively'] is False
    assert report['verdict'].startswith('fail: pool too small')


def test_a_shared_pose_that_does_not_reproduce_invalidates_the_check():
    base = _base()
    changed = _result(0, [0.0] * 7, 1.0, 2.0, record='different')
    report = runner.stability_check(base, [], [changed])
    assert report['criteria']['shared_poses_reproduce_exactly'] is False
    assert report['verdict'].startswith('invalid')

