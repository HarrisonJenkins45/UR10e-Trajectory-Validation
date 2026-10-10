#!/usr/bin/env python3
"""The section command: what a probe's artifacts establish, and what it reports.

The search policy is tested in test_section_search. Here the pipeline is a
stand-in process that writes the artifacts a real run writes, so the command
is exercised end to end -- input checks, journal, evidence, report and plan
export -- without planning anything.
"""
import json
import os
import sys

import numpy as np
import pytest

from ur10e_trajectory_pkg import section_contract as contract
from ur10e_trajectory_pkg import section_planner as sections
from ur10e_trajectory_pkg import section_search as search
from ur10e_trajectory_pkg.joint_coordinates import TWO_PI

LOWER = np.array([0.0, -TWO_PI, -TWO_PI, -np.pi, -TWO_PI, -TWO_PI, -TWO_PI])
UPPER = np.array([3.0, TWO_PI, TWO_PI, np.pi, TWO_PI, TWO_PI, TWO_PI])


# ---- helpers kept from the first section planner ---------------------------

def test_the_located_failure_is_the_earliest_named_time():
    commands = {
        'refinement': {'attempts': [
            {'passed': False, 'position_limit_excursions': {
                'wrist_3_joint': {'first_time_s': 170.8}}},
            {'passed': False, 'position_limit_excursions': {
                'wrist_3_joint': {'first_time_s': 170.75},
                'elbow_joint': {'first_time_s': 171.0}}}]},
        'task_validation': {'self_clearance': {'passed': False, 'at_time_s': 171.2},
                            'tracking': {'within_tolerance': True, 'at_time_s': 3.0}}}
    assert contract.located_failure_time(commands) == 170.75
    assert contract.located_failure_time({'task_validation': {
        'limit_violations': {'elbow_joint': {'jerk': {}}}}}) is None


def test_joint_margins_and_the_joints_at_a_limit():
    path = np.array([[1.0, 0.0, -2.7, -0.4, -0.8, 2.5, -6.2],
                     [1.1, 0.1, -2.7, -0.4, -0.8, 2.5, -6.27]])
    margins = contract.joint_margins(path, LOWER, UPPER)
    assert margins['wrist_3']['to_lower'] == pytest.approx(-6.27 + TWO_PI)
    assert margins['rail']['to_upper'] == pytest.approx(1.9)
    assert contract.limit_boundary(path[-1], LOWER, UPPER) == [
        'wrist_3 at its lower limit (-6.270)']


@pytest.mark.parametrize('layer, added, expected', [(1708, 1, 1708), (2, 1, 2), (1, 1, 1),
                                                    (0, 0, 1), (9, 0, 10), (5, 2, 4)])
def test_layers_after_the_spin_up_are_recorded_samples(layer, added, expected):
    assert search.samples_through_layer(layer, added) == expected


# ---- what a graph disconnect bounds ----------------------------------------

def _graph(complete, layer=None, attempts=('direct',), winding_aware=True, added=1,
           last_connected=None):
    seeds = [{'seeds': s, 'complete_path': complete and i == len(attempts) - 1,
              'first_disconnected_layer': None if complete and i == len(attempts) - 1
              else layer, 'start_states': 4}
             for i, s in enumerate(attempts)]
    graph = {'complete_path': complete, 'first_disconnected_layer': None if complete else layer,
             'spin_up': {'samples_added': added},
             'candidate_filters': {'home_reachable': {
                 'attempts': seeds, 'winding_aware': winding_aware,
                 'routes': {'direct': 0 if 'direct+via' in attempts else 4,
                            'via': 4 if 'direct+via' in attempts else 0},
                 'chosen_start': ({'candidate': 3, 'winding': [0] * 6,
                                   'route_kind': 'via' if 'direct+via' in attempts
                                   else 'direct'} if complete else None)}}}
    if not complete:
        graph['last_connected_layer'] = layer - 1 if last_connected is None else last_connected
    return graph


def test_a_disconnect_after_trying_every_start_route_bounds_the_start():
    graph = _graph(False, layer=1709, attempts=('direct', 'direct+via'))
    assert contract.graph_bound_samples(graph, via_offered=True) == 1708
    # With no via poses offered, direct starts are all the policy has.
    assert contract.graph_bound_samples(_graph(False, layer=1709), via_offered=False) == 1708
    assert contract.graph_bound_samples(_graph(False, layer=0, attempts=(
        'direct', 'direct+via')), via_offered=True) == 0


def test_a_disconnect_that_skipped_a_route_or_used_strict_windings_bounds_nothing():
    direct_only = _graph(False, layer=1709, attempts=('direct',))
    assert contract.graph_bound_samples(direct_only, via_offered=True) is None
    strict = _graph(False, layer=1709, attempts=('direct', 'direct+via'),
                    winding_aware=False)
    assert contract.graph_bound_samples(strict, via_offered=True) is None
    assert contract.graph_bound_samples(_graph(True), via_offered=True) is None


# ---- evidence from a probe's artifacts -------------------------------------

def _write(path, document):
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(document, handle)


PASSING_COMMANDS = {
    'both_commands_pass': True, 'status': 'ok', 'start_winding': [0, 0, 0, 0, 0, 1],
    'warmup': {'route_kind': 'via', 'total_duration_s': 6.5},
    'warmup_validation': {'passed': True},
    'refinement': {'status': 'refined', 'level_m': 0.005},
    'task_validation': {'passed': True,
                        'conditioning': {'max_condition_number': 11.3,
                                         'min_alpha_star': 30.0},
                        'self_clearance': {'passed': True, 'min_distance_m': 0.0188}}}


def write_probe(work_dir, start, samples, outcome, csv_sha256=None, spin_up_s=1.0):
    """The artifacts a pipeline run leaves, for an outcome the test chooses.

    outcome: 'pass', 'via_pass', ('graph', layer, attempts), ('refinement', time_s).
    """
    from ur10e_trajectory_pkg.target_builder import mount_record

    paths = {name: os.path.join(work_dir, f'{name}.json')
             for name in ('candidates', 'graph', 'commands', 'task_plan')}
    summary = {'status': 'ok', 'artifacts': paths, 'spin_up_s': spin_up_s,
               'start_index': start, 'waypoints': samples, 'strict_home_windings': False,
               'stages': [{'stage': 'candidates', 'seconds': 1.0},
                          {'stage': 'graph', 'seconds': 2.0}],
               'spin_up_ladder': {'derived': False, 'rungs_s': [spin_up_s],
                                  'tried': [{'spin_up_s': spin_up_s}]}}
    if outcome in ('pass', 'via_pass'):
        attempts = ('direct', 'direct+via') if outcome == 'via_pass' else ('direct',)
        graph = _graph(True, attempts=attempts)
        commands = PASSING_COMMANDS
        path = [[1.0, 0.1, -1.7, -1.7, -2.6, 1.0, 0.5]] * (samples + 1)
        _write(paths['task_plan'], {
            'q_path': path, 'start_index': start, 'recorded_waypoints': samples,
            'spin_up_s': spin_up_s, 'placement': 'nominal', 'mount': mount_record(),
            'recording': {'csv_sha256': csv_sha256}, 'home': [1.5] + [0.0] * 6})
        summary['stages'].append({'stage': 'commands', 'seconds': 3.0})
    elif outcome[0] == 'graph':
        graph = _graph(False, layer=outcome[1], attempts=outcome[2])
        commands = None
        summary.update(status='no complete graph path', failed_stage='graph')
    else:
        graph = _graph(True)
        commands = dict(PASSING_COMMANDS, both_commands_pass=False, refinement={
            'status': 'refinement_failed', 'attempts': [{
                'passed': False, 'position_limit_excursions': {
                    'wrist_3_joint': {'first_time_s': outcome[1]}}}]})
        summary.update(status='refinement_failed', failed_stage='commands')
    _write(paths['graph'], graph)
    if commands is not None:
        _write(paths['commands'], commands)
    _write(os.path.join(work_dir, 'pipeline.json'), summary)


def test_a_start_reachable_only_through_a_via_pose_is_certified(tmp_path):
    write_probe(str(tmp_path), 40, 601, 'via_pass')
    evidence, _, graph, _, plan = contract.probe_evidence(
        str(tmp_path), 601, 0.1, True, {'resource_limited': False})
    assert evidence['passed'] is True
    assert evidence['graph_bound_samples'] is None
    assert plan is not None
    route = contract.route_record(graph)
    assert route['routes'] == {'direct': 0, 'via': 4}
    assert route['chosen_start']['route_kind'] == 'via'


def test_a_located_refinement_failure_suggests_a_shorter_slice_but_bounds_nothing(tmp_path):
    write_probe(str(tmp_path), 0, 1708, ('refinement', 170.80))
    evidence, *_ = contract.probe_evidence(str(tmp_path), 1708, 0.1, True,
                                           {'resource_limited': False})
    assert evidence['passed'] is False
    assert evidence['classification']['class'] == 'refinement'
    assert evidence['graph_bound_samples'] is None
    assert evidence['shrink_hint_samples'] == 1703


def test_a_graph_disconnect_in_a_probe_becomes_its_bound(tmp_path):
    write_probe(str(tmp_path), 0, 2000, ('graph', 1709, ('direct', 'direct+via')))
    evidence, *_ = contract.probe_evidence(str(tmp_path), 2000, 0.1, True,
                                           {'resource_limited': False})
    assert evidence['classification']['class'] == 'graph_connectivity'
    assert evidence['graph_bound_samples'] == 1708


def test_a_killed_probe_is_a_resource_limit_not_a_failure_of_the_slice(tmp_path):
    evidence, *_ = contract.probe_evidence(str(tmp_path), 2000, 0.1, True, {
        'resource_limited': True, 'killed_reason': 'peak memory'})
    assert evidence['classification']['class'] == 'resource_limit'
    assert evidence['graph_bound_samples'] is None


def test_the_process_watch_kills_a_probe_past_its_memory_limit(tmp_path):
    hog = [sys.executable, '-c',
           'import time; block = b"x" * (600 * 1024 * 1024); time.sleep(60)']
    run = sections.run_pipeline_process([], str(tmp_path / 'hog.log'), 0.2, poll_s=0.1,
                                        command=hog)
    assert run['resource_limited'] is True
    assert run['peak_memory_gb'] > 0.2 and run['seconds'] < 30.0
    quick = sections.run_pipeline_process([], str(tmp_path / 'ok.log'), 4.0, poll_s=0.1,
                                          command=[sys.executable, '-c', 'print(1)'])
    assert quick['returncode'] == 0 and quick['resource_limited'] is False


# ---- the command, end to end ------------------------------------------------

STEP_S = 0.5


def write_recording(path, samples=300, step=STEP_S, t0=100.0):
    import pandas as pd
    from scipy.spatial.transform import Rotation

    times = t0 + step * np.arange(samples)
    quaternions = Rotation.from_rotvec(
        np.outer(np.deg2rad(2.0) * (times - t0), [0.3, 0.2, 0.93])).as_quat()
    frame = pd.DataFrame({'timestamp': times, 'p_G_I_x': 0.0, 'p_G_I_y': 0.0,
                          'p_G_I_z': 100.0, 'q_I_G_w': quaternions[:, 3],
                          'q_I_G_x': quaternions[:, 0], 'q_I_G_y': quaternions[:, 1],
                          'q_I_G_z': quaternions[:, 2]})
    frame.to_csv(path, index=False)
    return times


class StandInPipeline:
    """run_process for main(): writes a run's artifacts per a rule on the slice."""

    def __init__(self, rule, csv_path):
        from ur10e_trajectory_pkg.planning_runtime import file_digest

        self.rule = rule
        self.digest = file_digest(csv_path)
        self.calls = []

    def __call__(self, argv, log_path, memory_limit_gb):
        value = {argv[i]: argv[i + 1] for i in range(0, len(argv) - 1)
                 if argv[i].startswith('--')}
        start, samples = int(value['--start-index']), int(value['--waypoints'])
        self.calls.append((start, samples, argv))
        write_probe(value['--work-dir'], start, samples, self.rule(start, samples),
                    csv_sha256=self.digest, spin_up_s=float(value['--spin-up-s']))
        return {'returncode': 0, 'seconds': 30.0, 'peak_memory_gb': 1.5,
                'resource_limited': False, 'killed_reason': None}


@pytest.fixture
def command_inputs(tmp_path, monkeypatch):
    csv = str(tmp_path / 'camera_traj.csv')
    times = write_recording(csv)
    home = str(tmp_path / 'home_choice.json')
    _write(home, {'chosen': {'configuration': [1.5, 0.0, -1.31, 1.75, -2.0, -1.4, 0.0]}})
    via = str(tmp_path / 'poses.json')
    _write(via, [{'configuration': [1.0] * 7}])
    monkeypatch.setattr(contract, 'limits_record', lambda urdf: (
        {'statuses': {'arm_jerk': 'assumed'}, 'may_certify_for_hardware': False},
        LOWER, UPPER))
    return {'csv': csv, 'times': times, 'home': home, 'via': via,
            'work': str(tmp_path / 'search')}


def _argv(inputs, *extra):
    return ['--csv', inputs['csv'], '--home-json', inputs['home'], '--via-poses',
            inputs['via'], '--work-dir', inputs['work'], '--target-s', '20',
            '--start-stride-s', '10', '--workers', '2', *extra]


def _later_start_rule(start, samples):
    # Start 0 disconnects early whatever the length; the middle of the
    # recording certifies up to 100 samples (49.5 s) and runs out of graph at 120.
    if start == 0:
        return ('graph', 10, ('direct', 'direct+via'))
    if start == 129:
        if samples > 121:
            return ('graph', 122, ('direct', 'direct+via'))
        return 'pass' if samples <= 100 else ('refinement', STEP_S * (samples - 3))
    return ('refinement', 5.0)


def test_the_command_finds_extends_and_reports_a_section(command_inputs):
    stand_in = StandInPipeline(_later_start_rule, command_inputs['csv'])
    code = sections.main(_argv(command_inputs, '--budget-probes', '12'),
                         run_process=stand_in)
    assert code == sections.EXIT_FOUND
    with open(os.path.join(command_inputs['work'], 'sections.json'),
              encoding='utf-8') as handle:
        report = json.load(handle)

    assert report['schema_version'] == sections.SCHEMA_VERSION
    assert report['status'] == sections.STATUS_FOUND
    assert report['mount']['name'] == 'T_EG' and report['mount']['provisional'] is True
    assert report['inputs']['recording']['sha256'] == stand_in.digest
    assert report['limits']['may_certify_for_hardware'] is False
    assert any('not hardware-certified' in text for text in report['limitations'])

    best = report['result']['best_section']
    times = command_inputs['times']
    assert best['start_index'] == 129 and best['half_open'] is True
    assert best['end_index'] == best['start_index'] + best['samples']
    assert best['recorded_duration_s'] == pytest.approx(
        times[best['end_index'] - 1] - times[best['start_index']])
    assert best['recorded_duration_s'] >= 20.0
    assert 96 <= best['samples'] <= 100
    assert best['boundary']['kind'] in ('longer_probe_failed', 'next_sample_failed')
    assert best['start_route']['chosen_start']['route_kind'] == 'direct'
    assert best['margins']['joint_limits']['rail']['to_upper'] == pytest.approx(2.0)
    with open(best['plan']['path'], encoding='utf-8') as handle:
        plan = json.load(handle)
    assert plan['start_index'] == 129 and plan['recorded_waypoints'] == best['samples']

    search = report['search']
    assert search['target_met'] is True
    assert search['coverage']['starts'][0]['status'] == 'bounded_below_target_by_graph'
    assert search['coverage']['statements']
    assert report['resources']['peak_probe_memory_gb'] == pytest.approx(1.5)
    assert report['resources']['probes_run_now'] == len(stand_in.calls)
    # Every probe asked for the slice's own first spin-up rung and the via poses.
    assert all('--via-poses' in argv and '--spin-up-s' in argv
               for _, _, argv in stand_in.calls)


def test_the_command_reports_none_found_under_its_search_not_infeasible(command_inputs):
    stand_in = StandInPipeline(lambda s, n: ('refinement', 5.0), command_inputs['csv'])
    code = sections.main(_argv(command_inputs, '--budget-probes', '3'), run_process=stand_in)
    assert code == sections.EXIT_NONE
    with open(os.path.join(command_inputs['work'], 'sections.json'),
              encoding='utf-8') as handle:
        report = json.load(handle)
    assert report['status'] == sections.STATUS_NONE
    assert report['result']['best_section'] is None
    assert 'not a finding that the trajectory is infeasible' in report['result']['statement']
    assert report['search']['stop_reason'] == 'budget_probes'
    assert report['search']['next_planned_probe'] is not None
    assert not os.path.exists(os.path.join(command_inputs['work'], 'best_plan.json'))


def test_the_command_resumes_from_its_journal(command_inputs):
    first = StandInPipeline(_later_start_rule, command_inputs['csv'])
    assert sections.main(_argv(command_inputs, '--budget-probes', '2'),
                         run_process=first) == sections.EXIT_NONE
    assert len(first.calls) == 2

    second = StandInPipeline(_later_start_rule, command_inputs['csv'])
    assert sections.main(_argv(command_inputs, '--budget-probes', '12'),
                         run_process=second) == sections.EXIT_FOUND
    ran_before = {(s, n) for s, n, _ in first.calls}
    assert not ran_before & {(s, n) for s, n, _ in second.calls}
    with open(os.path.join(command_inputs['work'], 'sections.json'),
              encoding='utf-8') as handle:
        report = json.load(handle)
    assert report['resources']['probes_replayed_from_journal'] == 2
    assert report['result']['best_section']['start_index'] == 129

    # Another home is another search: the journal's entries are not reused.
    _write(command_inputs['home'], {'chosen': {'configuration': [1.4] + [0.0] * 6}})
    third = StandInPipeline(_later_start_rule, command_inputs['csv'])
    sections.main(_argv(command_inputs, '--budget-probes', '1'), run_process=third)
    assert len(third.calls) == 1


def test_the_command_refuses_a_recording_it_cannot_time(command_inputs):
    import pandas as pd

    frame = pd.read_csv(command_inputs['csv'])
    frame.loc[50, 'timestamp'] = frame.loc[49, 'timestamp']
    frame.to_csv(command_inputs['csv'], index=False)
    stand_in = StandInPipeline(lambda s, n: 'pass', command_inputs['csv'])
    assert sections.main(_argv(command_inputs), run_process=stand_in) == sections.EXIT_INPUT
    with open(os.path.join(command_inputs['work'], 'sections.json'),
              encoding='utf-8') as handle:
        report = json.load(handle)
    assert report['status'] == sections.STATUS_INPUT
    assert report['inputs']['recording']['problems']
    assert stand_in.calls == []
