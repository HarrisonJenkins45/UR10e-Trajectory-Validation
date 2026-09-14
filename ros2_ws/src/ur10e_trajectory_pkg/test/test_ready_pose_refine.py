#!/usr/bin/env python3
"""Refinement gate: the search follows the ranking key, and pruning is exact.

The search is driven through injected evaluate and admissible functions, so
its logic is checked against scoring functions whose optimum is known.
"""
import itertools

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg import ready_pose_refine as refine
from ur10e_trajectory_pkg import ready_pose_sweep as sweep
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

PLACEMENTS = ('a', 'b', 'c')


def _record(name, duration, jerk=10.0, families=(2, 2), connected=True):
    return {'placement': name,
            'classification': (sweep.CONNECTED if connected
                               else sweep.DIRECT_APPROACH_UNCONNECTED),
            # As the runner records it: an unconnected placement reaches none of
            # its families, so its fraction is 0.
            'connected_families': families[0] if connected else 0,
            'family_count': families[1],
            'connected_branches': families[0] if connected else 0,
            'slowest_family_duration_s': duration if connected else None,
            'family_shortest_max_peak_jerk': jerk if connected else None,
            'best_duration_s': duration if connected else None,
            'best_max_peak_jerk': jerk if connected else None}


def _summary(records):
    from ur10e_trajectory_pkg import ready_pose_runner as runner
    return runner.summarise_ready_pose(records)


def _fake_evaluator(score_fn, calls, record_fn=None, pruned_partials=None):
    """Per-placement records from an analytic function of the configuration.

    record_fn, when given, builds the whole record, so trials can disconnect a
    placement or lose a family; pruned_partials collects the partial records
    of every pruned trial.
    """
    def evaluate(configuration, order=None, incumbent=None):
        records = []
        for name in (order or PLACEMENTS):
            calls.append(name)
            records.append(record_fn(configuration, name) if record_fn
                           else _record(name, score_fn(configuration, name)))
            if incumbent is not None and refine.cannot_beat(records, incumbent,
                                                            len(PLACEMENTS)):
                if pruned_partials is not None:
                    pruned_partials.append(list(records))
                return None, records, False
        return _summary(records), records, True
    return evaluate


def _bowl(target):
    target = np.asarray(target)
    offsets = {'a': 0.0, 'b': 0.3, 'c': 0.1}
    return lambda q, name: 1.0 + offsets[name] + float(np.sum(np.abs(q - target)))


def test_rank_key_matches_the_ranking_order():
    better = {'connectivity': 1.0, 'worst_family_fraction': 1.0,
              'worst_duration_s': 3.0, 'worst_peak_jerk': 50.0}
    worse = dict(better, worst_family_fraction=0.8, worst_duration_s=1.0)
    assert refine.rank_key(better) < refine.rank_key(worse)
    ranked = sweep.rank_ready_poses([dict(worse, static_gates={'passed': True}, n=0),
                                     dict(better, static_gates={'passed': True}, n=1)])
    assert ranked[0]['n'] == 1


def test_a_partial_evaluation_already_slower_cannot_win():
    incumbent = _summary([_record(n, 2.0) for n in PLACEMENTS])
    assert refine.cannot_beat([_record('a', 2.5)], incumbent, 3) is True
    assert refine.cannot_beat([_record('a', 1.5)], incumbent, 3) is False


def test_losing_a_family_or_a_placement_cannot_win_whatever_the_duration():
    incumbent = _summary([_record(n, 2.0) for n in PLACEMENTS])
    assert refine.cannot_beat([_record('a', 0.1, families=(1, 2))], incumbent, 3)
    assert refine.cannot_beat([_record('a', 0.1, connected=False)], incumbent, 3)


def test_a_trial_that_could_win_on_connectivity_is_never_pruned_on_duration():
    incumbent = _summary([_record('a', 2.0), _record('b', 2.0),
                          _record('c', None, connected=False)])
    assert refine.cannot_beat([_record('a', 9.0)], incumbent, 3) is False


def test_the_search_walks_to_the_known_optimum_on_the_step_lattice():
    start = np.zeros(7)
    # On the step lattice: 5 deg steps reach within 2 deg, then 2 deg steps land
    # exactly. A target 1 deg past a 5 deg multiple would stall on a tie, since
    # only strict improvement moves.
    target = np.concatenate(([0.1], np.deg2rad([10.0, -12.0, 0, 0, 0, 8.0])))
    calls = []
    _, summary, _, history = refine.coordinate_search(
        start, _fake_evaluator(_bowl(target), calls), lambda q: True)
    final = np.asarray(history['final_configuration'])
    np.testing.assert_allclose(final, target, atol=1e-9)
    assert summary['worst_duration_s'] == pytest.approx(1.3)


def test_pruning_never_changes_the_result():
    start = np.zeros(7)
    target = np.concatenate(([-0.07], np.deg2rad([7.0, 3.0, -9.0, 0, 2.0, 0])))
    pruned_calls, full_calls = [], []
    _, pruned, _, pruned_history = refine.coordinate_search(
        start, _fake_evaluator(_bowl(target), pruned_calls), lambda q: True)
    _, full, _, full_history = refine.coordinate_search(
        start, _fake_evaluator(_bowl(target), full_calls), lambda q: True,
        prune=False)
    assert pruned_history['final_configuration'] == full_history['final_configuration']
    assert refine.rank_key(pruned) == refine.rank_key(full)
    assert len(pruned_calls) < len(full_calls)
    assert pruned_history['pruned'] > 0


def test_pruning_is_exact_on_every_part_of_the_key():
    """Trials that disconnect a placement or lose a family must be pruned on
    those parts of the key, and the result must still equal full evaluation.

    The duration bowl pulls shoulder_pan toward +12 deg and shoulder_lift
    toward -12 deg, but pan beyond 7 deg disconnects placement c and lift
    below -7 deg costs a family at a, so the best reachable pose sits short
    of the bowl's centre.
    """
    target = np.concatenate(([0.0], np.deg2rad([12.0, -12.0, 0, 0, 0, 0])))
    bowl = _bowl(target)

    def record_fn(q, name):
        if name == 'c' and q[1] > np.deg2rad(7.0) + 1e-12:
            return _record(name, None, connected=False)
        families = (1, 2) if name == 'a' and q[2] < -np.deg2rad(7.0) - 1e-12 else (2, 2)
        return _record(name, bowl(q, name), families=families)

    pruned_calls, full_calls, partials = [], [], []
    _, pruned, _, pruned_history = refine.coordinate_search(
        np.zeros(7), _fake_evaluator(None, pruned_calls, record_fn, partials),
        lambda q: True)
    _, full, _, full_history = refine.coordinate_search(
        np.zeros(7), _fake_evaluator(None, full_calls, record_fn), lambda q: True,
        prune=False)

    assert pruned_history['final_configuration'] == full_history['final_configuration']
    assert refine.rank_key(pruned) == refine.rank_key(full)
    assert pruned['connectivity'] == 1.0 and pruned['worst_family_fraction'] == 1.0
    final = np.asarray(pruned_history['final_configuration'])
    assert final[1] <= np.deg2rad(7.0) + 1e-12 and final[2] >= -np.deg2rad(7.0) - 1e-12
    assert any(any(r['classification'] == sweep.DIRECT_APPROACH_UNCONNECTED
                   for r in p) for p in partials), 'connectivity branch not exercised'
    assert any(any(r['connected_families'] < r['family_count'] for r in p)
               for p in partials), 'family branch not exercised'
    assert len(pruned_calls) < len(full_calls)


def test_a_search_stop_is_reported_as_no_improving_step():
    target = np.concatenate(([0.05], np.zeros(6)))
    _, _, _, history = refine.coordinate_search(
        np.zeros(7), _fake_evaluator(_bowl(target), []), lambda q: True)
    assert [s['stop'] for s in history['stops']] == ['no improving single-joint step'] * 2


def test_inadmissible_trials_are_skipped_not_evaluated():
    start = np.zeros(7)
    target = np.concatenate(([0.1], np.zeros(6)))
    calls = []
    _, _, _, history = refine.coordinate_search(
        start, _fake_evaluator(_bowl(target), calls), lambda q: q[0] <= 0.0)
    assert history['final_configuration'][0] == 0.0
    assert history['inadmissible'] > 0


def test_acceptance_needs_full_family_fraction_and_the_margin():
    winner = {'connectivity': 1.0, 'worst_family_fraction': 1.0,
              'worst_duration_s': 2.45, 'worst_peak_jerk': 100.0}
    small = dict(winner, worst_duration_s=2.40)
    large = dict(winner, worst_duration_s=2.30)
    lost_family = dict(large, worst_family_fraction=0.8)
    assert refine.accept(small, winner)['accepted_pending_confirmation'] is False
    assert refine.accept(large, winner)['accepted_pending_confirmation'] is True
    assert refine.accept(lost_family, winner)['accepted_pending_confirmation'] is False
    lost_placement = dict(large, connectivity=0.96)
    decision = refine.accept(lost_placement, winner)
    assert decision['connectivity_is_one'] is False
    assert decision['accepted_pending_confirmation'] is False


def test_starting_points_take_the_base_top_and_new_poses_in_the_merged_top():
    def result(index, duration, fraction=1.0):
        return {'ready_index': index, 'configuration': [float(index)] * 7,
                'provenance': 'broad_sample',
                'summary': {'connectivity': 1.0, 'worst_family_fraction': fraction,
                            'worst_duration_s': duration, 'worst_peak_jerk': 1.0}}
    base = [result(i, 2.0 + 0.1 * i) for i in range(10)]
    new = [result(100, 1.0), result(101, 9.0)]
    starts = refine.starting_points(base, new)
    assert [s['ready_index'] for s in starts] == [0, 1, 2, 3, 4, 5, 6, 100]
    assert starts[-1]['source'] == 'new_in_merged_top'


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description'))


def test_a_rail_cap_override_is_applied_and_recorded(validator):
    from ur10e_trajectory_pkg import motion_limits
    original = validator.rail_velocity_cap
    try:
        validator.set_rail_velocity_cap(0.5)
        assert validator.velocity_limits[0] == pytest.approx(0.5)
        entry = motion_limits.effective_limits(validator)['velocity'][0]
        assert entry['cap_overridden'] is True
        assert entry['safety_cap'] == 0.5
        assert entry['status'] == motion_limits.ASSUMED
    finally:
        validator.set_rail_velocity_cap(original)
    assert validator.velocity_limits[0] == pytest.approx(
        min(validator.urdf_velocity_limits[0], motion_limits.RAIL_VEL_SAFETY_CAP))
    assert motion_limits.effective_limits(validator)['velocity'][0]['cap_overridden'] is False
