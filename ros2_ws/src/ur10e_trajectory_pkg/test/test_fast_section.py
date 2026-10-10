#!/usr/bin/env python3
"""The fast section command: its schedule, and its answers under a budget.

The certifier is a stand-in writing the artifacts a real certification
writes (test_section_planner.write_probe), so what is tested is the command:
where it looks, when it stops, what it publishes and what it will not claim.
"""
import json
import os

import pytest

from test_section_planner import LOWER, UPPER, write_probe, write_recording

from ur10e_trajectory_pkg import fast_section, section_contract, section_planner
from ur10e_trajectory_pkg import section_search as search


def test_the_schedule_spreads_starts_before_it_widens_the_search():
    order, levels = search.start_order(400, 50)
    plan = fast_section.schedule(order, levels, 4)
    first_round = [(start, cap) for round_number, start, cap in plan if round_number == 0]
    assert first_round == [(0, 0), (400, 0), (200, 0)]
    second_round = [(start, cap) for round_number, start, cap in plan if round_number == 1]
    assert (100, 1) in second_round and (0, 1) in second_round
    assert len(plan) == len(set(plan))
    # Every grid start is scheduled, at the widest level, eventually.
    assert {start for _, start, cap in plan if cap == 3} == {s for s, _, _ in order}


class FakeContext:
    def __init__(self, *args, **kwargs):
        self.closed = False

    def close(self):
        self.closed = True


class StandInCertifier:
    """certify_slice's signature; the rule decides each slice's outcome."""

    def __init__(self, rule, csv_path):
        from ur10e_trajectory_pkg.planning_runtime import file_digest

        self.rule = rule
        self.digest = file_digest(csv_path)
        self.calls = []

    def __call__(self, context, choice, csv, start, samples, spin_up, work_dir,
                 deadline=None, levels=None):
        self.calls.append((start, samples, len(levels) - 1))
        outcome = self.rule(start, samples, len(levels) - 1)
        os.makedirs(work_dir, exist_ok=True)
        if outcome == 'deadline':
            summary = {'status': 'inconclusive: the time budget expired',
                       'failed_stage': 'deadline', 'stages': []}
        else:
            write_probe(work_dir, start, samples, outcome, csv_sha256=self.digest,
                        spin_up_s=spin_up)
            with open(os.path.join(work_dir, 'pipeline.json'), encoding='utf-8') as handle:
                summary = json.load(handle)
        return summary


@pytest.fixture
def inputs(tmp_path, monkeypatch):
    csv = str(tmp_path / 'camera_traj.csv')
    times = write_recording(csv, samples=400)
    home = str(tmp_path / 'home_choice.json')
    with open(home, 'w', encoding='utf-8') as handle:
        json.dump({'chosen': {'configuration': [1.5, 0.0, -1.31, 1.75, -2.0, -1.4, 0.0]}},
                  handle)
    monkeypatch.setattr(section_contract, 'limits_record', lambda urdf: (
        {'may_certify_for_hardware': False}, LOWER, UPPER))
    return {'csv': csv, 'home': home, 'times': times, 'work': str(tmp_path / 'fast')}


def _argv(inputs, *extra):
    return ['--csv', inputs['csv'], '--home-json', inputs['home'], '--work-dir',
            inputs['work'], '--target-s', '20', '--start-stride-s', '10', '--workers', '1',
            *extra]


def _report(inputs):
    with open(os.path.join(inputs['work'], 'fast_section.json'), encoding='utf-8') as handle:
        return json.load(handle)


def test_a_section_found_only_at_a_later_start_is_certified_and_published(inputs, monkeypatch):
    # Only the middle of the recording is executable, and only once the search
    # may widen to level 1.
    certifier = StandInCertifier(
        lambda start, samples, cap: 'pass' if start == 179 and cap >= 1
        else ('graph', 10, ('direct', 'direct+via')), inputs['csv'])
    published = []
    real_record = section_contract.section_record

    def record(*args, **kwargs):
        published.append(os.path.exists(os.path.join(inputs['work'], 'best_plan.json')))
        return real_record(*args, **kwargs)

    monkeypatch.setattr(section_contract, 'section_record', record)
    code = fast_section.main(_argv(inputs), certify=certifier, context_factory=FakeContext)
    assert code == fast_section.EXIT_FOUND
    report = _report(inputs)
    best = report['result']['best_section']
    assert report['status'] == fast_section.STATUS_FOUND
    assert best['start_index'] == 179 and best['end_index'] == 179 + 41
    assert best['recorded_duration_s'] == pytest.approx(20.0)
    assert published == [True]
    assert certifier.calls[:3] == [(0, 41, 0), (359, 41, 0), (179, 41, 0)]
    assert report['mount']['provisional'] is True
    with open(os.path.join(inputs['work'], 'probes.jsonl'), encoding='utf-8') as handle:
        journal = [json.loads(line) for line in handle]
    assert [(e['start'], e['samples'], e['passed']) for e in journal] == [(179, 41, True)]


def test_length_mode_replays_the_fast_certificate_and_extends_it(inputs):
    certifier = StandInCertifier(lambda start, samples, cap: 'pass', inputs['csv'])
    assert fast_section.main(_argv(inputs), certify=certifier,
                             context_factory=FakeContext) == fast_section.EXIT_FOUND
    from test_section_planner import StandInPipeline

    stand_in = StandInPipeline(lambda start, samples: 'pass' if samples <= 90
                               else ('refinement', 0.5 * (samples - 3)), inputs['csv'])
    code = section_planner.main(
        ['--csv', inputs['csv'], '--home-json', inputs['home'], '--work-dir', inputs['work'],
         '--target-s', '20', '--start-stride-s', '10', '--workers', '1',
         '--budget-probes', '6'], run_process=stand_in)
    assert code == section_planner.EXIT_FOUND
    assert (0, 41) not in [(s, n) for s, n, _ in stand_in.calls]
    # Lengthening starts from the certificate: the first probe grows it.
    assert stand_in.calls[0][:2] == (0, 82)
    with open(os.path.join(inputs['work'], 'sections.json'), encoding='utf-8') as handle:
        report = json.load(handle)
    assert report['resources']['probes_replayed_from_journal'] == 1
    assert report['result']['best_section']['samples'] > 41


def test_an_expired_budget_is_inconclusive_never_infeasible(inputs):
    certifier = StandInCertifier(lambda start, samples, cap: 'deadline', inputs['csv'])
    code = fast_section.main(_argv(inputs), certify=certifier, context_factory=FakeContext)
    assert code == fast_section.EXIT_INCONCLUSIVE
    report = _report(inputs)
    assert report['status'] == fast_section.STATUS_INCONCLUSIVE
    assert 'not a finding that the trajectory is infeasible' in report['result']['statement']
    assert not os.path.exists(os.path.join(inputs['work'], 'best_plan.json'))
    assert 'infeasible' not in report['status']


def test_a_budget_spent_before_any_attempt_is_inconclusive(inputs):
    certifier = StandInCertifier(lambda start, samples, cap: 'pass', inputs['csv'])
    code = fast_section.main(_argv(inputs, '--time-budget-s', '0'), certify=certifier,
                             context_factory=FakeContext)
    assert code == fast_section.EXIT_INCONCLUSIVE
    assert certifier.calls == []
    assert _report(inputs)['result']['attempts_planned_not_run'] > 0


def test_a_finished_schedule_without_a_certificate_is_none_found(inputs):
    certifier = StandInCertifier(lambda start, samples, cap: ('refinement', 5.0), inputs['csv'])
    code = fast_section.main(_argv(inputs, '--time-budget-s', '100000'), certify=certifier,
                             context_factory=FakeContext)
    assert code == fast_section.EXIT_NONE
    report = _report(inputs)
    assert report['status'] == fast_section.STATUS_NONE
    assert 'not exhaustive' in report['result']['statement']
    assert report['result']['attempts_planned_not_run'] == 0


def test_a_recording_that_cannot_be_timed_is_refused(inputs):
    import pandas as pd

    frame = pd.read_csv(inputs['csv'])
    frame.loc[5, 'timestamp'] = frame.loc[4, 'timestamp']
    frame.to_csv(inputs['csv'], index=False)
    certifier = StandInCertifier(lambda start, samples, cap: 'pass', inputs['csv'])
    code = fast_section.main(_argv(inputs), certify=certifier, context_factory=FakeContext)
    assert code == fast_section.EXIT_INPUT and certifier.calls == []


def test_length_mode_extends_a_later_certificate_before_walking_the_grid(inputs):
    certifier = StandInCertifier(
        lambda start, samples, cap: 'pass' if start == 179 else ('graph', 10, ('direct',)),
        inputs['csv'])
    assert fast_section.main(_argv(inputs), certify=certifier,
                             context_factory=FakeContext) == fast_section.EXIT_FOUND
    from test_section_planner import StandInPipeline

    stand_in = StandInPipeline(lambda start, samples: 'pass' if start == 179 and samples <= 90
                               else ('refinement', 0.5 * (samples - 3)), inputs['csv'])
    section_planner.main(
        ['--csv', inputs['csv'], '--home-json', inputs['home'], '--work-dir', inputs['work'],
         '--target-s', '20', '--start-stride-s', '10', '--workers', '1',
         '--budget-probes', '4'], run_process=stand_in)
    assert [(s, n) for s, n, _ in stand_in.calls][0] == (179, 82)
    assert 0 not in [s for s, _, _ in stand_in.calls]
