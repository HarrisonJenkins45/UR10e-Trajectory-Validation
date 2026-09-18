#!/usr/bin/env python3
"""One command per trajectory: the stages, their order, and the backstop.

The stages themselves are tested where they live. What matters here is that
the pipeline runs them in order with flags that agree about placement,
waypoints and spin-up, stops at the first failure, and rebuilds the graph
with a stricter filter exactly when the whole path turns out to have no
valid winding.
"""
import argparse
import json

import pytest

from ur10e_trajectory_pkg import pipeline


def _args(tmp_path, **overrides):
    values = {'placement': 'nominal', 'csv': None, 'waypoints': 500, 'spin_up_s': 2.0,
              'home_json': str(tmp_path / 'home.json'), 'via_poses': None,
              'strict_home_windings': False, 'urdf': '/root/ros2_ws/ur10e.urdf',
              'work_dir': str(tmp_path / 'run'), 'summary': None}
    values.update(overrides)
    return argparse.Namespace(**values)


def _stub_stages(monkeypatch, tmp_path, commands_document, codes=None):
    """Each stage records its argv and writes the artifact the next one reads."""
    from ur10e_trajectory_pkg import candidate_generator, graph_planner, home_pose

    codes = codes or {}
    seen = []

    def stage(name):
        def run(argv):
            seen.append((name, list(argv)))
            if name == 'commands':
                out = argv[argv.index('--out') + 1]
                with open(out, 'w', encoding='utf-8') as handle:
                    json.dump(commands_document, handle)
                if commands_document and not codes.get('commands'):
                    plan = argv[argv.index('--plan-out') + 1]
                    with open(plan, 'w', encoding='utf-8') as handle:
                        json.dump({'q_path': [[0.0] * 7]}, handle)
            return codes.get(name, 0)
        return run

    monkeypatch.setattr(candidate_generator, 'main', stage('candidates'))
    monkeypatch.setattr(graph_planner, 'main', stage('graph'))
    monkeypatch.setattr(home_pose, 'main', stage('commands'))
    return seen


PASSING = {'both_commands_pass': True, 'status': 'ok', 'start_winding': [0] * 6,
           'warmup': {'route_kind': 'direct', 'total_duration_s': 2.25},
           'refinement': {'status': 'refined', 'level_m': 0.005},
           'task_validation': {'passed': True}}


def test_the_stages_run_in_order_and_agree_about_the_trajectory(tmp_path, monkeypatch):
    """A graph built at one placement against targets built at another still
    produces a path, just not one anyone checked."""
    seen = _stub_stages(monkeypatch, tmp_path, PASSING)
    code, summary = pipeline.run(_args(tmp_path, via_poses=str(tmp_path / 'vias.json')))

    assert code == 0 and summary['status'] == 'ok'
    assert [name for name, _ in seen] == ['candidates', 'graph', 'commands']
    for name, argv in seen:
        if name != 'commands':
            assert '--placement' in argv and argv[argv.index('--placement') + 1] == 'nominal'
            assert '--spin-up-s' in argv and argv[argv.index('--spin-up-s') + 1] == '2.0'
    graph_argv = dict(seen)['graph']
    assert graph_argv[graph_argv.index('--layers') + 1] == '500'
    assert '--via-poses' in graph_argv
    assert '--strict-home-windings' not in graph_argv
    assert summary['both_commands_pass'] is True and summary['plan_written'] is True


def test_graph_workers_reach_only_the_graph_stage(tmp_path, monkeypatch):
    seen = _stub_stages(monkeypatch, tmp_path, PASSING)
    pipeline.run(_args(tmp_path, graph_workers=6))
    argv = dict(seen)
    assert argv['graph'][argv['graph'].index('--workers') + 1] == '6'
    assert '--workers' not in argv['candidates'] and '--workers' not in argv['commands']

    seen = _stub_stages(monkeypatch, tmp_path, PASSING)
    pipeline.run(_args(tmp_path))
    assert '--workers' not in dict(seen)['graph']


def test_a_failing_stage_stops_the_chain(tmp_path, monkeypatch):
    seen = _stub_stages(monkeypatch, tmp_path, PASSING, codes={'graph': 1})
    code, summary = pipeline.run(_args(tmp_path))
    assert code == 1
    assert summary['failed_stage'] == pipeline.STAGE_GRAPH
    assert [name for name, _ in seen] == ['candidates', 'graph']


def test_no_valid_winding_rebuilds_the_graph_with_the_stricter_filter(tmp_path, monkeypatch):
    """The filter judges a start's own limits; whether a winding keeps the
    whole path in limits is not known until the path exists."""
    blocked = {'status': pipeline.NO_VALID_WINDING, 'both_commands_pass': False}
    seen = _stub_stages(monkeypatch, tmp_path, blocked, codes={'commands': 1})
    code, summary = pipeline.run(_args(tmp_path))

    assert code == 1
    assert summary['strict_home_windings'] is True
    assert [name for name, _ in seen] == ['candidates', 'graph', 'commands',
                                          'graph', 'commands']
    retried = [argv for name, argv in seen if name == 'graph'][1]
    assert '--strict-home-windings' in retried


def test_another_commands_failure_is_not_retried(tmp_path, monkeypatch):
    """Only the winding backstop is retried; anything else is reported."""
    other = {'status': 'targets do not match the graph placement', 'both_commands_pass': False}
    seen = _stub_stages(monkeypatch, tmp_path, other, codes={'commands': 1})
    code, summary = pipeline.run(_args(tmp_path))

    assert code == 1 and summary['strict_home_windings'] is False
    assert [name for name, _ in seen] == ['candidates', 'graph', 'commands']
    assert summary['status'] == 'targets do not match the graph placement'


def test_stages_passing_without_a_plan_is_named_as_such(tmp_path, monkeypatch):
    """Every stage returning 0 while the validations did not pass is not a
    stage failure, and must not be reported as one."""
    not_passing = dict(PASSING, both_commands_pass=False, status=None,
                       task_validation={'passed': False})
    _stub_stages(monkeypatch, tmp_path, not_passing)
    code, summary = pipeline.run(_args(tmp_path))

    assert code == 1
    assert summary['failed_stage'] == pipeline.STAGE_COMMANDS
    assert summary['status'] == 'commands did not pass both validations'


def test_an_incomplete_graph_stops_the_chain_and_names_the_layer(tmp_path, monkeypatch):
    """The graph wrote complete_path false and exited 0, so commands refined a
    path of None and raised instead of reporting where the graph broke."""
    seen = _stub_stages(monkeypatch, tmp_path, PASSING, codes={'graph': 1})
    from ur10e_trajectory_pkg import graph_planner

    def incomplete(argv):
        seen.append(('graph', list(argv)))
        out = argv[argv.index('--out') + 1]
        with open(out, 'w', encoding='utf-8') as handle:
            json.dump({'complete_path': False, 'first_disconnected_layer': 762,
                       'path': None}, handle)
        return 1

    monkeypatch.setattr(graph_planner, 'main', incomplete)
    code, summary = pipeline.run(_args(tmp_path))

    assert code == 1 and summary['failed_stage'] == pipeline.STAGE_GRAPH
    assert summary['status'] == f'{pipeline.GRAPH_INCOMPLETE}: first disconnected layer 762'
    assert [name for name, _ in seen] == ['candidates', 'graph']


def test_a_stage_that_raises_is_a_recorded_stage_failure(tmp_path, monkeypatch):
    _stub_stages(monkeypatch, tmp_path, PASSING)
    from ur10e_trajectory_pkg import home_pose

    def raises(argv):
        raise IndexError('too many indices for array')

    monkeypatch.setattr(home_pose, 'main', raises)
    code, summary = pipeline.run(_args(tmp_path))

    assert code == 1 and summary['failed_stage'] == pipeline.STAGE_COMMANDS
    assert summary['status'] == 'commands raised IndexError: too many indices for array'
    last = summary['stages'][-1]
    assert last['returncode'] == 1 and 'IndexError' in last['traceback']


def test_the_summary_is_written_even_when_the_run_itself_raises(tmp_path, monkeypatch):
    from ur10e_trajectory_pkg import ClientNode

    def broken(csv_path=None, num_waypoints=500, start_index=0):
        raise ValueError('the recording is not uniformly sampled')

    monkeypatch.setattr(ClientNode, 'recorded_start_rate', broken)
    work = tmp_path / 'run'
    code = pipeline.main(['--home-json', str(tmp_path / 'home.json'),
                          '--work-dir', str(work)])
    summary = json.loads((work / 'pipeline.json').read_text())
    assert code == 1
    assert summary['status'] == ('pipeline raised ValueError: the recording is not '
                                 'uniformly sampled')


def test_a_commands_stage_that_ran_to_the_end_is_not_summarised_as_ok(tmp_path, monkeypatch):
    """The commands document says 'ok' when the stage completed, including
    when a validation then failed; copying that through read as a pass."""
    ran_but_failed = dict(PASSING, both_commands_pass=False, status='ok')
    _stub_stages(monkeypatch, tmp_path, ran_but_failed)
    code, summary = pipeline.run(_args(tmp_path))

    assert code == 1 and summary['failed_stage'] == pipeline.STAGE_COMMANDS
    assert summary['status'] == 'commands did not pass both validations'


# --------------------------------------------------------------------------
# Spin-up ladder: durations derived from the recording's starting rate
# --------------------------------------------------------------------------

def test_every_rung_lands_on_a_recorded_sample():
    """apply_spin_up rounds a duration up to a multiple of twice the recorded
    step so the join lands on a recorded sample. A ladder proposing durations
    it would silently round would disagree with the run's actual durations."""
    step = 0.1
    quantum = 2 * step
    rungs = pipeline.spin_up_ladder(0.08, step)
    for rung in rungs:
        # Not rung % quantum: 0.6 % 0.2 is 0.19999999999999998 in binary
        # floating point, so the modulo would fail on an exact multiple.
        multiples = rung / quantum
        assert multiples == pytest.approx(round(multiples), abs=1e-9)
    assert rungs == sorted(rungs)
    assert len(set(rungs)) == len(rungs)


def test_a_faster_tumble_asks_for_a_longer_spin_up():
    """The rate is what the duration is derived from: the peak tool angular
    acceleration of the ramp is w0 * 1.5 / T."""
    slow = pipeline.spin_up_ladder(0.05, 0.1)
    fast = pipeline.spin_up_ladder(0.5, 0.1)
    assert fast[0] > slow[0]

    reference = pipeline.SPIN_UP_REFERENCE_ANGULAR_ACCELERATION_RAD_S2
    # The seed is the shortest rung whose peak stays under the reference,
    # give or take the rounding onto a recorded sample.
    assert 0.5 * 1.5 / fast[0] <= reference + 1e-9


def test_the_ladder_is_short_because_each_rung_is_a_whole_run():
    """Candidates depend on the spin-up, so a rung is a full generate, graph
    and commands cycle rather than a cheap retry."""
    assert pipeline.SPIN_UP_LADDER_RUNGS <= 4
    assert len(pipeline.spin_up_ladder(0.08, 0.1)) <= pipeline.SPIN_UP_LADDER_RUNGS


def test_a_given_duration_runs_once_and_keeps_the_flat_layout(tmp_path, monkeypatch):
    seen = _stub_stages(monkeypatch, tmp_path, PASSING)
    code, summary = pipeline.run(_args(tmp_path, spin_up_s=2.0))

    assert code == 0
    assert summary['spin_up_ladder']['derived'] is False
    assert [name for name, _ in seen] == ['candidates', 'graph', 'commands']
    assert summary['artifacts']['task_plan'].endswith('run/task_plan.json')


def test_a_derived_spin_up_stops_at_the_first_rung_that_works(tmp_path, monkeypatch):
    """Each rung is a whole cycle, so the search stops as soon as one passes."""
    from ur10e_trajectory_pkg import ClientNode

    monkeypatch.setattr(ClientNode, 'recorded_start_rate',
                        lambda csv_path=None, num_waypoints=500, start_index=0: {'rate_rad_s': 0.08,
                                                                  'step_s': 0.1})
    seen = _stub_stages(monkeypatch, tmp_path, PASSING)
    code, summary = pipeline.run(_args(tmp_path, spin_up_s=None))

    ladder = summary['spin_up_ladder']
    assert code == 0 and ladder['derived'] is True
    assert ladder['chosen_s'] == ladder['rungs_s'][0]
    assert [entry['spin_up_s'] for entry in ladder['tried']] == [ladder['rungs_s'][0]]
    assert [name for name, _ in seen] == ['candidates', 'graph', 'commands']
    # Rungs get their own directories, so a failed rung's artifacts survive.
    assert 'spin_up_' in summary['artifacts']['task_plan']


def test_a_failing_rung_moves_to_the_next_and_both_are_recorded(tmp_path, monkeypatch):
    from ur10e_trajectory_pkg import ClientNode

    monkeypatch.setattr(ClientNode, 'recorded_start_rate',
                        lambda csv_path=None, num_waypoints=500, start_index=0: {'rate_rad_s': 0.08,
                                                                  'step_s': 0.1})
    calls = {'graph': 0}
    _stub_stages(monkeypatch, tmp_path, PASSING)

    from ur10e_trajectory_pkg import graph_planner
    passing_graph = graph_planner.main

    def graph(argv):
        calls['graph'] += 1
        passing_graph(argv)
        return 1 if calls['graph'] == 1 else 0

    monkeypatch.setattr(graph_planner, 'main', graph)
    code, summary = pipeline.run(_args(tmp_path, spin_up_s=None))

    ladder = summary['spin_up_ladder']
    assert code == 0
    assert [entry['spin_up_s'] for entry in ladder['tried']] == ladder['rungs_s'][:2]
    assert ladder['tried'][0]['failed_stage'] == pipeline.STAGE_GRAPH
    assert ladder['chosen_s'] == ladder['rungs_s'][1]


# --------------------------------------------------------------------------
# The recording: every stage loads it, so every stage is told
# --------------------------------------------------------------------------

def test_the_recording_reaches_every_stage_and_the_start_rate(tmp_path, monkeypatch):
    """A stage left on the packaged default would plan a different motion
    from the others; the spin-up would be derived from the wrong rate."""
    from ur10e_trajectory_pkg import ClientNode

    asked = []

    def rate(csv_path=None, num_waypoints=500, start_index=0):
        asked.append(csv_path)
        return {'rate_rad_s': 0.06, 'step_s': 0.1}

    monkeypatch.setattr(ClientNode, 'recorded_start_rate', rate)
    seen = _stub_stages(monkeypatch, tmp_path, PASSING)
    code, summary = pipeline.run(_args(tmp_path, spin_up_s=None, csv='/data/juice.csv'))

    assert code == 0 and asked == ['/data/juice.csv']
    assert summary['csv'] == '/data/juice.csv'
    assert [name for name, _ in seen] == ['candidates', 'graph', 'commands']
    for name, argv in seen:
        assert argv[argv.index('--csv') + 1] == '/data/juice.csv', name


def test_a_slice_reaches_candidates_graph_and_the_start_rate_but_not_commands(
        tmp_path, monkeypatch):
    """Commands reads the slice back from the graph; its parser takes no
    --start-index, and a spin-up must be derived at the slice's own start."""
    from ur10e_trajectory_pkg import ClientNode

    asked = []

    def rate(csv_path=None, num_waypoints=500, start_index=0):
        asked.append((num_waypoints, start_index))
        return {'rate_rad_s': 0.06, 'step_s': 0.1}

    monkeypatch.setattr(ClientNode, 'recorded_start_rate', rate)
    seen = _stub_stages(monkeypatch, tmp_path, PASSING)
    code, summary = pipeline.run(_args(tmp_path, spin_up_s=None, start_index=1200,
                                       waypoints=601))
    argv = dict(seen)
    assert code == 0 and asked == [(601, 1200)] and summary['start_index'] == 1200
    for name in ('candidates', 'graph'):
        assert argv[name][argv[name].index('--start-index') + 1] == '1200', name
    assert '--start-index' not in argv['commands']


def test_without_a_recording_no_stage_is_told_one(tmp_path, monkeypatch):
    seen = _stub_stages(monkeypatch, tmp_path, PASSING)
    code, summary = pipeline.run(_args(tmp_path))
    assert code == 0 and summary['csv'] is None
    assert all('--csv' not in argv for _, argv in seen)
