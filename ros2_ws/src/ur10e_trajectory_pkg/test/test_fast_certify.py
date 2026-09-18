#!/usr/bin/env python3
"""The fast certifier: its search with controlled oracles, and the equivalences
its caches rely on, checked against the real validator."""
import itertools

import numpy as np
import pytest

from ur10e_trajectory_pkg import fast_certify as fast
from ur10e_trajectory_pkg import section_planner

LOWER = np.array([0.0] + [-2 * np.pi] * 6)
UPPER = np.array([3.0] + [2 * np.pi] * 6)
WIDE = tuple(dict(spec, beam=None) for spec in fast.LEVELS)


def state(*values):
    out = np.zeros(7)
    out[:len(values)] = values
    return out


class GridOracle:
    """Layers of candidate rows and an edge rule; rows may depend on the level."""

    limits = (LOWER, UPPER)
    rail_budget = None

    def __init__(self, starts, rows, allowed, cost=None):
        self.starts = starts
        self.rows = rows                  # rows(layer, level) -> list of states
        self.allowed = allowed            # allowed(pred, row, layer) -> bool
        self.cost = cost or (lambda p, r: float(np.sum((r - p) ** 2)))
        self.calls = []

    def start_frontier(self, level):
        return list(self.starts(level))

    def candidates(self, layer, frontier, level):
        self.calls.append((layer, level))
        return list(self.rows(layer, level))

    def successors(self, predecessor, predecessor_key, row, row_key, layer):
        if not self.allowed(predecessor, row, layer):
            return []
        return [(row, self.cost(predecessor, row))]


def exhaustive_cost(starts, rows, allowed, cost, layers):
    best = {fast.state_key(s): 0.0 for s in starts}
    states = {fast.state_key(s): s for s in starts}
    for layer in range(1, layers):
        nxt, nxt_states = {}, {}
        for key, value in best.items():
            for row in rows(layer, 0):
                if allowed(states[key], row, layer):
                    total = value + cost(states[key], row)
                    k = fast.state_key(row)
                    if k not in nxt or total < nxt[k]:
                        nxt[k], nxt_states[k] = total, row
        best, states = nxt, nxt_states
    return min(best.values()) if best else None


def test_a_beam_without_a_width_finds_the_exhaustive_optimum():
    rng = np.random.default_rng(7)
    table = {layer: [state(*rng.uniform(0, 2, size=2)) for _ in range(6)] for layer in range(8)}

    def allowed(p, r, layer):
        return abs(p[0] - r[0]) < 1.2

    def cost(p, r):
        return float(np.sum((r - p) ** 2))

    oracle = GridOracle(lambda level: table[0], lambda layer, level: table[layer], allowed)
    result = fast.BeamSearch(oracle, 8, levels=WIDE).run()
    assert result['complete']
    assert result['cost'] == pytest.approx(exhaustive_cost(
        table[0], lambda layer, level: table[layer], allowed, cost, 8))
    assert len(result['path']) == 8


def test_a_disconnect_broadens_the_layers_before_it_and_resumes():
    """Level 0 offers no row at layer 5 that connects; level 1 does."""
    near, far = state(1.0), state(2.5)

    def rows(layer, level):
        if layer == 5 and level == 0:
            return [far]
        return [near]

    oracle = GridOracle(lambda level: [near], rows,
                        lambda p, r, layer: abs(p[0] - r[0]) < 1.0)
    search = fast.BeamSearch(oracle, 9)
    result = search.run()
    assert result['complete']
    assert search.escalations[0]['failed_layer'] == 5
    assert search.levels[5] == 1 and search.levels[8] == 0
    # Layers before the window were not searched again.
    assert oracle.calls.count((1, 0)) == 1


def test_a_disconnect_no_level_repairs_is_reported_where_it_happened():
    oracle = GridOracle(lambda level: [state(0.0)], lambda layer, level: [state(float(layer))],
                        lambda p, r, layer: layer < 4)
    result = fast.BeamSearch(oracle, 7).run()
    assert result['complete'] is False
    assert result['first_disconnected_layer'] == 4
    assert result['last_connected_layer'] == 3 and len(result['partial_path']) == 4


def test_the_beam_keeps_the_cheapest_and_then_the_states_with_room():
    cheap = (0.0, (0,), state(1.5, 6.2), None)          # 0.08 rad from its limit
    cramped = (0.1, (1,), state(1.5, 6.25), None)
    roomy = (0.5, (2,), state(1.5, 0.0), None)
    kept = fast.select_beam([cramped, roomy, cheap], 2, LOWER, UPPER)
    assert [entry[1] for entry in kept] == [(0,), (2,)]


def test_a_path_that_fails_validation_is_banned_and_another_is_found():
    a, b = state(0.0), state(0.5)
    oracle = GridOracle(lambda level: [a], lambda layer, level: [a, b],
                        lambda p, r, layer: True)
    search = fast.BeamSearch(oracle, 6)
    first = search.run()
    assert all(np.allclose(q, a) for q in first['path'])
    assert search.reject_path(first['path'], located_layer=3)
    second = search.run()
    assert second['complete']
    assert not np.allclose(second['path'][3], a)


def test_the_deadline_stops_the_search_without_a_verdict():
    ticks = itertools.count()
    oracle = GridOracle(lambda level: [state(0.0)], lambda layer, level: [state(0.0)],
                        lambda p, r, layer: True)
    search = fast.BeamSearch(oracle, 50, deadline=10, clock=lambda: next(ticks))
    with pytest.raises(fast.DeadlineExpired):
        search.run()


# ---- starts ---------------------------------------------------------------

class FakeRobot:
    qlim = (LOWER, UPPER)


class FakeValidator:
    robot = FakeRobot()
    velocity_limits = np.array([1.0] + [2.0] * 6)


def test_oracle_uses_recorded_elapsed_time_for_each_edge():
    context = fast.FastContext('urdf', state(1.5), validator=FakeValidator())
    positions = np.zeros((4, 3))
    quaternions = np.tile([0.0, 0.0, 0.0, 1.0], (4, 1))
    oracle = fast.SliceOracle(context, positions, quaternions, 0.1)
    assert oracle.step_dt(2) == pytest.approx(0.1)
    assert oracle.step_dt(3) == pytest.approx(0.1)
    assert oracle.rail_budget_for_layer(3) == pytest.approx(0.1)


def test_a_start_reachable_only_through_a_via_pose_fills_the_frontier(monkeypatch):
    context = fast.FastContext('urdf', state(1.5), via_poses=[state(1.0)], workers=1,
                               validator=FakeValidator())
    positions, quaternions = np.zeros((3, 3)), np.tile([0.0, 0.0, 0.0, 1.0], (3, 1))
    oracle = fast.SliceOracle(context, positions, quaternions, 0.1)
    rows = [state(1.0, 0.1), state(1.2, 0.2)]
    monkeypatch.setattr(oracle, '_layer0_rows', lambda: rows)
    seen = []

    def evaluate(stage, states):
        seen.append((stage, len(states)))
        return [(stage == 'via', None if stage == 'via' else 'no_direct_warmup')
                for _ in states]

    monkeypatch.setattr(oracle, '_evaluate', evaluate)
    frontier = oracle.start_frontier(0)
    assert frontier and oracle.start_record['routes']['direct'] == 0
    assert oracle.start_record['routes']['via'] == len(frontier)
    assert [a['seeds'] for a in oracle.start_record['attempts']] == ['direct', 'direct+via']
    assert seen[0][0] == 'direct' and seen[-1][0] == 'via'


def test_the_path_s_own_start_winding_is_tried_before_the_full_sweep(monkeypatch):
    from ur10e_trajectory_pkg import continuous_validator, home_pose, warmup

    path = np.zeros((4, 7))
    options = [{'winding': [1, 0, 0, 0, 0, 0], 'path': path + 1},
               {'winding': [0] * 6, 'path': path}]
    monkeypatch.setattr(home_pose, 'start_windings', lambda validator, p: options)
    monkeypatch.setattr(warmup, 'plan_route', lambda *a, **k: {
        'status': warmup.OK, 'total_duration_s': 2.0, 'route_kind': 'direct', 'segments': [{}]})
    verdict = {'passed': True}
    monkeypatch.setattr(continuous_validator, 'validate_route',
                        lambda validator, route: {'passed': verdict['passed'], 'failures': []})
    swept = []
    monkeypatch.setattr(fast, 'sweep_start_windings',
                        lambda *a, **k: swept.append(1) or (None, []))
    best, _, how = fast.fast_start_winding(None, np.zeros(7), path)
    assert how == 'graph_start_winding' and best['winding'] == [0] * 6 and not swept
    verdict['passed'] = False
    best, _, how = fast.fast_start_winding(None, np.zeros(7), path)
    assert how == 'full_winding_search' and swept


# ---- what a fast result may and may not claim ------------------------------

def test_the_full_winding_sweep_stops_at_the_deadline(monkeypatch):
    """The sweep plans and validates a route per legal winding; it must not run
    one more winding after the budget is gone."""
    from ur10e_trajectory_pkg import continuous_validator, home_pose, warmup

    path = np.zeros((4, 7))
    options = [{'winding': [n, 0, 0, 0, 0, 0], 'path': path + n} for n in range(1, 9)]
    monkeypatch.setattr(home_pose, 'start_windings', lambda validator, p: options)
    planned = []

    def plan_route(validator, home, start, via_poses=None, **kwargs):
        planned.append(start[0])
        return {'status': warmup.OK, 'total_duration_s': 2.0, 'route_kind': 'direct',
                'segments': [{}]}

    monkeypatch.setattr(warmup, 'plan_route', plan_route)
    monkeypatch.setattr(continuous_validator, 'validate_route',
                        lambda validator, route: {'passed': False, 'failures': []})
    ticks = itertools.count()

    def check():
        if next(ticks) >= 3:
            raise fast.DeadlineExpired()

    with pytest.raises(fast.DeadlineExpired):
        fast.sweep_start_windings(None, np.zeros(7), path, check=check)
    assert len(planned) == 3


def test_the_sweep_returns_what_home_pose_would(monkeypatch):
    from ur10e_trajectory_pkg import continuous_validator, home_pose, warmup

    path = np.zeros((3, 7))
    options = [{'winding': [1, 0, 0, 0, 0, 0], 'path': path + 1},
               {'winding': [0, 0, 0, 0, 0, 1], 'path': path + 2}]
    monkeypatch.setattr(home_pose, 'start_windings', lambda validator, p: options)
    monkeypatch.setattr(warmup, 'plan_route', lambda validator, home, start, via_poses=None, **k: {
        'status': warmup.OK, 'total_duration_s': 9.0 - start[0], 'route_kind': 'direct',
        'segments': [{}]})
    monkeypatch.setattr(continuous_validator, 'validate_route',
                        lambda validator, route: {'passed': True, 'failures': []})
    mine, summary = fast.sweep_start_windings(None, np.zeros(7), path)
    theirs, their_summary = home_pose.choose_start_winding(None, np.zeros(7), path)
    # The shorter route wins in both, and the summaries agree winding for winding.
    assert mine['winding'] == theirs['winding'] == [0, 0, 0, 0, 0, 1]
    assert [o['winding'] for o in summary] == [o['winding'] for o in their_summary]
    assert [o['validated'] for o in summary] == [o['validated'] for o in their_summary]


def test_a_fast_disconnect_never_bounds_a_start():
    graph = {'complete_path': False, 'first_disconnected_layer': 700,
             'spin_up': {'samples_added': 1}, 'search': {'engine': fast.ENGINE,
                                                          'exhaustive': False},
             'candidate_filters': {'home_reachable': {
                 'winding_aware': True, 'attempts': [{'seeds': 'direct'},
                                                     {'seeds': 'direct+via'}]}}}
    assert section_planner.graph_bound_samples(graph, via_offered=True) is None


def test_a_probe_stopped_by_its_deadline_is_a_resource_limit(tmp_path):
    import json

    with open(tmp_path / 'pipeline.json', 'w', encoding='utf-8') as handle:
        json.dump({'status': 'inconclusive: the time budget expired',
                   'failed_stage': 'deadline', 'artifacts': {}}, handle)
    evidence, *_ = section_planner.probe_evidence(str(tmp_path), 601, 0.1, True,
                                                  {'resource_limited': False})
    assert evidence['classification']['class'] == 'resource_limit'
    assert evidence['graph_bound_samples'] is None and evidence['passed'] is False


# ---- the equivalences the caches rely on, on the real validator --------------

@pytest.fixture(scope='module')
def validator():
    from ament_index_python.packages import get_package_share_directory

    from ur10e_trajectory_pkg.failure_census import _validator
    return _validator('/root/ros2_ws/ur10e.urdf', get_package_share_directory('ur_description'))


@pytest.mark.parametrize('spin_up_s', [None, 0.2])
def test_a_recorded_sample_has_the_same_target_in_every_slice(spin_up_s):
    from ur10e_trajectory_pkg.failure_census import load_trajectory

    long_positions, long_quaternions, _, long_meta = load_trajectory(
        None, 60, with_metadata=True, spin_up_s=spin_up_s, start_index=10)
    short_positions, short_quaternions, _, short_meta = load_trajectory(
        None, 30, with_metadata=True, spin_up_s=spin_up_s, start_index=30)
    added = 0 if spin_up_s is None else long_meta['spin_up']['samples_added']
    assert added == (0 if spin_up_s is None else short_meta['spin_up']['samples_added'])

    def layer(sample, start):
        return sample - start + added

    for sample in range(30 + 2 * added, 60):
        assert fast.target_key(long_positions[layer(sample, 10)],
                               long_quaternions[layer(sample, 10)]) == fast.target_key(
            short_positions[layer(sample, 30)], short_quaternions[layer(sample, 30)])


def test_a_cached_solve_is_the_solve_it_replaces(validator):
    from ur10e_trajectory_pkg.failure_census import WIDE_SEED_BANK_DEG, load_trajectory

    positions, quaternions, dt = load_trajectory(None, 20, spin_up_s=0.2)
    seeds = [(np.deg2rad(arm), 1.5) for arm in WIDE_SEED_BANK_DEG[:3]]

    def fresh_oracle():
        context = fast.FastContext('/root/ros2_ws/ur10e.urdf', np.zeros(7), workers=1,
                                   validator=validator)
        return fast.SliceOracle(context, positions, quaternions, dt)

    oracle = fresh_oracle()
    first = oracle.solve_many(12, seeds)
    oracle.solve_many(5, seeds)                      # other solves in between
    cached = oracle.solve_many(12, seeds)
    assert oracle.context.cache_stats['solve_hits'] == 3
    solved_again = fresh_oracle().solve_many(12, list(reversed(seeds)))[::-1]
    assert any(row is not None for row in first)
    for a, b, c in zip(first, cached, solved_again):
        assert (a is None and b is None and c is None) or (
            np.array_equal(a, b) and np.array_equal(a, c))


def test_the_pool_answers_exactly_what_the_process_answers(validator):
    from ur10e_trajectory_pkg.failure_census import WIDE_SEED_BANK_DEG, load_trajectory

    positions, quaternions, dt = load_trajectory(None, 12, spin_up_s=0.2)
    seeds = [(np.deg2rad(arm), rail) for arm in WIDE_SEED_BANK_DEG for rail in (0.5, 1.5)]
    serial = fast.SliceOracle(fast.FastContext('/root/ros2_ws/ur10e.urdf', np.zeros(7),
                                               workers=1, validator=validator),
                              positions, quaternions, dt)
    pooled_context = fast.FastContext('/root/ros2_ws/ur10e.urdf', np.zeros(7), workers=2,
                                      validator=validator)
    pooled = fast.SliceOracle(pooled_context, positions, quaternions, dt)
    try:
        rows_serial = serial.solve_many(8, seeds)
        rows_pooled = pooled.solve_many(8, seeds)
        for a, b in zip(rows_serial, rows_pooled):
            assert (a is None and b is None) or np.array_equal(a, b)
        found = [r for r in rows_serial if r is not None]
        assert found
        requests = [(p, fast.state_key(p), r, fast.state_key(r)) for p in found for r in found]
        requests = (requests * (fast.PARALLEL_MIN_EDGES // len(requests) + 1))[
            :fast.PARALLEL_MIN_EDGES]
        in_process = [serial.successors(*request, 9) for request in requests]
        in_pool = pooled.successors_batch(requests, 9)
        assert len(in_process) == len(in_pool)
        for a, b in zip(in_process, in_pool):
            assert len(a) == len(b)
            for (sa, ca), (sb, cb) in zip(a, b):
                assert np.array_equal(sa, sb) and ca == cb
    finally:
        pooled_context.close()


def test_a_wait_on_the_pool_ends_at_the_deadline(validator):
    import time

    context = fast.FastContext('/root/ros2_ws/ur10e.urdf', np.zeros(7), workers=2,
                               validator=validator)
    try:
        context.start_pool()
        context.deadline = time.perf_counter() + 0.5
        began = time.perf_counter()
        with pytest.raises(fast.DeadlineExpired):
            context.map(time.sleep, [30.0, 30.0])
        assert time.perf_counter() - began < 5.0
    finally:
        context.close()
