#!/usr/bin/env python3
"""The section search policy, driven by controlled fake certifiers.

Every fake decides pass or fail from (start, samples) alone, the way a
deterministic pipeline would, so each test states exactly which slices are
executable and checks what the search finds, what it claims about the
boundary, and what it says it did not cover.
"""
import numpy as np
import pytest

from ur10e_trajectory_pkg import section_search as search


def timeline(samples, step=0.1, t0=0.0):
    return search.Timeline(t0 + step * np.arange(samples))


class Fake:
    """A certifier from a rule: rule(start, samples) -> outcome keywords."""

    def __init__(self, rule):
        self.rule = rule
        self.calls = []

    def __call__(self, start, samples, phase, reason):
        self.calls.append((start, samples, phase))
        keywords = dict(self.rule(start, samples) or {})
        keywords.setdefault('passed', False)
        return search.outcome(start, samples, **keywords)


def config(**overrides):
    values = dict(target_s=10.0, max_probe_samples=2000, start_stride_s=10.0,
                  length_resolution_s=1.0, budget=search.Budget(probes=40))
    values.update(overrides)
    return search.SearchConfig(**values)


def run(line, rule, **overrides):
    fake = Fake(rule)
    return search.SectionSearch(line, config(**overrides)).run(fake), fake


# ---- time ------------------------------------------------------------------

@pytest.mark.parametrize('step, expected', [(0.1, 601), (0.25, 241), (0.2, 301),
                                            (0.05, 1201), (1.0 / 3.0, 181)])
def test_sixty_seconds_is_counted_from_the_timestamps_at_any_rate(step, expected):
    line = timeline(4000, step=step, t0=1234.5)
    assert line.samples_for(0, 60.0) == expected
    assert line.duration(0, expected) == pytest.approx(60.0)
    assert line.duration(0, expected - 1) < 60.0


def test_duration_follows_uneven_timestamps_rather_than_a_nominal_step():
    line = search.Timeline([0.0, 0.1, 0.3, 0.35, 1.0, 1.1])
    assert line.duration(1, 4) == pytest.approx(0.9)
    assert line.samples_for(0, 0.3) == 3
    assert line.last_start(1.0) == 1
    assert line.last_start(1.05) == 0
    assert line.samples_for(1, 5.0) is None


def test_the_interval_is_half_open_with_its_recorded_duration():
    interval = timeline(100, step=0.5, t0=10.0).interval(4, 21)
    assert interval['end_index'] == 25 and interval['half_open'] is True
    assert interval['start_time_s'] == pytest.approx(12.0)
    assert interval['last_sample_time_s'] == pytest.approx(22.0)
    assert interval['recorded_duration_s'] == pytest.approx(10.0)


# ---- starts ----------------------------------------------------------------

def test_coverage_levels_spread_over_the_recording_down_to_the_stride():
    levels = search.coverage_levels(400, 50)
    assert levels[0] == [0, 400]
    assert levels[1] == [200]
    assert levels[2] == [100, 300]
    starts = sorted(s for level in levels for s in level)
    assert len(starts) == len(set(starts))
    assert min(np.diff(starts)) >= 50
    assert max(np.diff(starts)) < 100


def test_hints_reorder_and_add_starts_but_never_remove_one():
    plain, _ = search.start_order(400, 50)
    hinted, _ = search.start_order(400, 50, hints={300: 5.0, 100: -1.0, 37: 9.0})
    plain_starts = [s for s, _, _ in plain]
    hinted_starts = [s for s, _, _ in hinted]
    assert set(plain_starts) <= set(hinted_starts)
    assert set(hinted_starts) - set(plain_starts) == {37}
    # Reordered within its level only: 300 now precedes 100, both after 200.
    assert hinted_starts.index(300) < hinted_starts.index(100)
    assert hinted_starts.index(200) < hinted_starts.index(300)


# ---- finding a section -----------------------------------------------------

def test_a_section_is_found_when_only_a_later_start_can_succeed():
    line = timeline(1000)
    order = [s for s, _, _ in search.start_order(line.last_start(10.0), 100)[0]]
    later = order[4]
    result, fake = run(line, lambda s, n: {'passed': s == later and n <= 300})
    assert result.best[0] == later
    assert result.target_met()
    first = [call for call in fake.calls if call[2] == search.PHASE_TARGET]
    # Every earlier start in coverage order was tried, at the target length.
    assert [c[0] for c in first] == order[:5]
    assert all(c[1] == line.samples_for(c[0], 10.0) for c in first)


def test_a_scan_that_underestimates_a_start_cannot_exclude_it():
    """A ranking scan said this start covers 50 samples, fewer than the target;
    its freshly generated slice certifies anyway."""
    line = timeline(1000)
    order = [s for s, _, _ in search.start_order(line.last_start(10.0), 100)[0]]
    underestimated = order[3]
    hints = {s: 1.0 for s in order}
    hints[underestimated] = -50.0
    result, fake = run(line, lambda s, n: {'passed': s == underestimated and n <= 200},
                       hints=hints)
    assert underestimated in [c[0] for c in fake.calls]
    assert result.best[0] == underestimated


def test_a_failure_at_one_start_bounds_nothing_at_another():
    line = timeline(1000)
    result, _ = run(line, lambda s, n: ({'graph_bound_samples': 0} if s == 0
                                        else {'passed': n <= 150}))
    assert result.best is not None and result.best[0] != 0
    rows = {r['start']: r for r in result.coverage()['starts']}
    assert rows[0]['status'] == 'bounded_below_target_by_graph'


def test_a_failed_first_probe_shrinks_toward_the_target_but_never_below_it():
    line = timeline(3000)
    result, fake = run(line, lambda s, n: {'passed': s == 0 and n <= 400,
                                           'shrink_hint_samples': 380 if n > 400 else None},
                       first_probe_s=100.0, budget=search.Budget(probes=3))
    assert fake.calls[:2] == [(0, 1001, 'target'), (0, 380, 'target')]
    assert result.best == (0, 380)


# ---- extending a section ---------------------------------------------------

def test_a_certified_section_keeps_extending_after_its_first_pass():
    line = timeline(3000)
    result, fake = run(line, lambda s, n: {'passed': s == 0 and n <= 700},
                       budget=search.Budget(probes=14))
    start, samples = result.best
    assert start == 0
    assert 700 - 10 <= samples <= 700
    assert fake.calls[0] == (0, 101, 'target')
    assert (0, 202, 'extend') in fake.calls


def test_the_boundary_names_a_failure_just_beyond_the_section_only_when_probed():
    line = timeline(3000)
    result, _ = run(line, lambda s, n: {'passed': s == 0 and n <= 437},
                    length_resolution_s=0.0, max_probes_per_start=20,
                    budget=search.Budget(probes=20))
    assert result.best == (0, 437)
    boundary = result.boundary(0, 437)
    assert boundary['kind'] == 'next_sample_failed'
    assert boundary['next_sample_fails'] is True

    coarse, _ = run(line, lambda s, n: {'passed': s == 0 and n <= 437},
                    length_resolution_s=5.0, budget=search.Budget(probes=20))
    start, samples = coarse.best
    boundary = coarse.boundary(start, samples)
    assert boundary['kind'] == 'longer_probe_failed'
    assert boundary['next_sample_fails'] is False
    assert boundary['unprobed_samples_between'] > 0

    unexamined, _ = run(line, lambda s, n: {'passed': True}, budget=search.Budget(probes=1))
    assert unexamined.boundary(*unexamined.best)['kind'] == 'not_examined'
    assert unexamined.boundary(*unexamined.best)['next_sample_fails'] is None


def test_a_graph_disconnect_bounds_its_start_and_located_failures_shrink():
    """The shape of c1-anchor-s4 at 0.1 s: the graph from start 0 reaches 1708
    samples, and 1708 fails refinement at its end, so 1703 is the section."""
    line = timeline(5000)

    def rule(s, n):
        if s != 0:
            return {}
        if n > 1708:
            return {'graph_bound_samples': 1708}
        if n > 1703:
            return {'shrink_hint_samples': 1703}
        return {'passed': True}

    result, fake = run(line, rule, target_s=60.0, start_stride_s=10.0,
                       budget=search.Budget(probes=5))
    assert fake.calls == [(0, 601, 'target'), (0, 1202, 'extend'), (0, 2000, 'extend'),
                          (0, 1708, 'extend'), (0, 1703, 'extend')]
    assert result.best == (0, 1703)
    assert result.timeline.duration(0, 1703) == pytest.approx(170.2)
    boundary = result.boundary(0, 1703)
    assert boundary['kind'] == 'longer_probe_failed'
    assert boundary['nearest_failed_samples'] == 1708
    assert result.starts[0].ceiling(2000) == 1708


def test_a_graph_bound_reached_exactly_is_reported_as_the_boundary():
    line = timeline(3000)
    result, _ = run(line, lambda s, n: ({'passed': True} if n <= 850
                                        else {'graph_bound_samples': 850}),
                    budget=search.Budget(probes=6))
    assert result.best == (0, 850)
    assert result.boundary(0, 850)['kind'] == 'graph_bound'


def test_a_probe_cut_by_resources_bounds_nothing():
    line = timeline(3000)
    result, _ = run(line, lambda s, n: ({'resource_limited': True} if n > 150
                                        else {'passed': s == 0}),
                    budget=search.Budget(probes=6))
    assert result.best == (0, 101)
    assert result.starts[0].graph_bound is None
    assert result.boundary(0, 101)['resource_limited'] is True


# ---- budget, coverage, resume ----------------------------------------------

def test_an_exhausted_budget_reports_none_found_and_what_was_left():
    line = timeline(1000)
    result, fake = run(line, lambda s, n: {}, budget=search.Budget(probes=3))
    assert len(fake.calls) == 3
    assert result.best is None and not result.target_met()
    assert result.stop_reason == 'budget_probes'
    assert result.next_planned['phase'] == 'target'
    coverage = result.coverage()
    assert coverage['probed_starts'] == 3
    assert coverage['status_counts']['not_probed'] == coverage['grid_starts'] - 3
    assert any('never probed' in statement for statement in coverage['statements'])


def test_the_sample_budget_refuses_a_probe_that_would_overrun_it():
    line = timeline(1000)
    result, fake = run(line, lambda s, n: {},
                       budget=search.Budget(probes=None, probe_samples=250))
    assert len(fake.calls) == 2
    assert result.stop_reason == 'budget_probe_samples'


def test_a_search_with_nothing_left_to_probe_says_so():
    line = timeline(150)
    result, fake = run(line, lambda s, n: {}, start_stride_s=2.0)
    assert result.stop_reason == 'no_probe_left_under_the_rules'
    assert result.coverage()['status_counts'] == {'failed_probes_bound_nothing':
                                                   len(fake.calls)}


def test_a_recording_shorter_than_the_target_is_not_searched():
    result, fake = run(timeline(50), lambda s, n: {'passed': True})
    assert fake.calls == [] and result.stop_reason == 'recording_shorter_than_target'


def test_a_target_longer_than_the_probe_cap_is_not_searched():
    result, fake = run(timeline(3000), lambda s, n: {'passed': True}, max_probe_samples=50)
    assert fake.calls == [] and result.stop_reason == 'target_exceeds_max_probe_samples'


def test_a_resumed_search_makes_the_same_decisions_and_reruns_nothing():
    line = timeline(3000)

    def rule(s, n):
        return {'passed': s in (0, 1450) and n <= 400 + s // 10, 'seconds': 60.0}

    whole, _ = run(line, rule, budget=search.Budget(probes=9))
    journal = {}

    def recording(s, n, phase, reason):
        result = Fake(rule)(s, n, phase, reason)
        journal[(s, n)] = result
        return result

    search.SectionSearch(line, config(budget=search.Budget(probes=4))).run(recording)
    fresh = Fake(rule)

    def resumed(s, n, phase, reason):
        return journal[(s, n)] if (s, n) in journal else fresh(s, n, phase, reason)

    again = search.SectionSearch(line, config(budget=search.Budget(probes=9))).run(resumed)
    assert again.decisions == whole.decisions
    assert len(fresh.calls) == 5
    assert again.consumed()['probe_hours'] == pytest.approx(9 / 60.0)


def test_the_hour_budget_counts_journal_seconds():
    line = timeline(1000)
    result, fake = run(line, lambda s, n: {'seconds': 1800.0},
                       budget=search.Budget(probes=None, probe_hours=1.0))
    assert len(fake.calls) == 2 and result.stop_reason == 'budget_probe_hours'


def test_the_summary_states_the_best_section_with_its_boundary():
    line = timeline(2000, step=0.25, t0=7.0)
    result, _ = run(line, lambda s, n: {'passed': s == 0 and n <= 300},
                    target_s=60.0, budget=search.Budget(probes=6))
    summary = result.summary()
    best = summary['best']
    assert best['start_index'] == 0
    assert best['recorded_duration_s'] == pytest.approx(0.25 * (best['samples'] - 1))
    assert best['recorded_duration_s'] >= 60.0
    assert best['boundary']['kind'] in ('longer_probe_failed', 'next_sample_failed')
    assert summary['target_met'] is True
    assert summary['config']['budget'] == {'probes': 6, 'probe_samples': None,
                                           'probe_hours': None}


def test_a_failure_whose_evidence_stops_short_steps_past_the_certified_length():
    """A disconnect that covered no more than is certified: probe a quarter of the
    gap past the certified length, not halfway to the failure."""
    line = timeline(3000)

    def rule(s, n):
        if s != 0:
            return {}
        if n > 640:
            return {'shrink_hint_samples': 627}
        return {'passed': True}

    result, fake = run(line, rule, target_s=60.0, budget=search.Budget(probes=6))
    # 627 is the failure's own evidence; once it is certified, the next probe is
    # a quarter of the way to 1202, not the 914 a bisection would try.
    assert [n for _, n, _ in fake.calls] == [601, 1202, 627, 771, 663, 636]
    assert result.best == (0, 636)
