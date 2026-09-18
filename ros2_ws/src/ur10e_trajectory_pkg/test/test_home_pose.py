#!/usr/bin/env python3
"""Configured home validation, winding and command export."""
import inspect

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg import home_pose, motion_limits
from ur10e_trajectory_pkg import robot_checks as sweep
from ur10e_trajectory_pkg.joint_coordinates import TWO_PI
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description'))


@pytest.fixture(scope='module')
def compact():
    return np.concatenate(([1.5], np.deg2rad([0.0, -75.0, 100.0, -115.0, -80.0, 0.0])))


# --------------------------------------------------------------------------
# Clearance measures the arm
# --------------------------------------------------------------------------

def test_only_links_moved_by_an_arm_joint_count(validator):
    names = {validator._pb_link_name_by_index[i] for i in sweep.arm_link_indices(validator)}
    for fixed in ('rail_base_link', 'rail_carriage_link', 'base_link',
                  'base_link_inertia', 'base'):
        assert fixed not in names
    for moving in ('shoulder_link', 'forearm_link', 'wrist_3_link', 'tool0'):
        assert moving in names


def test_clearance_changes_with_the_arm_posture(validator, compact):
    """It was a constant 0.049 m, base_link_inertia against the floor."""
    lowered = compact.copy()
    lowered[2] = np.deg2rad(-20.0)                  # shoulder lift toward the floor
    raised = sweep.collision_distance(validator, compact)
    down = sweep.collision_distance(validator, lowered)
    assert raised != pytest.approx(0.049, abs=1e-3)
    assert down < raised


# --------------------------------------------------------------------------
# Singularity robustness
# --------------------------------------------------------------------------

def test_a_well_conditioned_pose_passes_the_neighbourhood(validator, compact):
    result = sweep.singularity_robustness(validator, compact)
    assert result['passed'] is True
    assert result['worst_condition'] >= result['nominal_condition']
    assert result['samples'] == 12 + sweep.SINGULARITY_RANDOM_SAMPLES


def test_a_nearly_straight_elbow_fails_the_neighbourhood(validator, compact):
    straight = compact.copy()
    straight[3] = np.deg2rad(3.0)
    assert sweep.singularity_robustness(validator, straight)['passed'] is False


def test_the_neighbourhood_is_deterministic(validator, compact):
    first = sweep.singularity_robustness(validator, compact)
    assert sweep.singularity_robustness(validator, compact) == first



# --------------------------------------------------------------------------
# Start winding
# --------------------------------------------------------------------------

def test_every_start_winding_shifts_the_whole_path(validator, compact):
    path = np.stack([compact + np.concatenate(([0.0], np.full(6, 0.01 * i)))
                     for i in range(5)])
    options = home_pose.start_windings(validator, path)
    assert len(options) > 1
    for option in options:
        shift = option['path'] - path
        np.testing.assert_allclose(shift, np.broadcast_to(shift[0], shift.shape),
                                   atol=1e-12)
        np.testing.assert_allclose(np.angle(np.exp(1j * shift[0][1:])), 0, atol=1e-9)


def test_a_winding_that_leaves_the_limits_is_not_offered(validator, compact):
    path = np.stack([compact.copy() for _ in range(3)])
    path[:, 4] = [np.pi, np.pi + 0.5, TWO_PI - 0.1]   # wrist_1 climbing near +2*pi
    for option in home_pose.start_windings(validator, path):
        assert np.all(option['path'][:, 4] <= TWO_PI + 1e-9)


def test_the_chosen_winding_has_a_valid_and_shortest_warmup(validator, compact):
    start = compact + np.concatenate(([0.2], np.deg2rad([10, -5, 5, 0, 10, 20])))
    path = np.stack([start + np.concatenate(([0.0], np.full(6, 0.005 * i)))
                     for i in range(4)])
    best, summary = home_pose.choose_start_winding(validator, compact, path)
    assert best is not None and best['warmup']['status'] == 'ok'
    assert best['warmup_validation']['passed'] is True
    # Shortest among the routes that VALIDATED, and no validated route is
    # shorter: a planned route ranks nothing until it passes validation.
    validated = [s['duration_s'] for s in summary if s['validated']]
    assert best['warmup']['total_duration_s'] == min(validated)
    refused = [s['duration_s'] for s in summary if s['validated'] is False]
    assert all(duration <= best['warmup']['total_duration_s'] for duration in refused)
    assert best['warmup']['route_kind'] == 'direct'


def test_windings_are_validated_in_rank_order_and_the_first_pass_is_kept(monkeypatch):
    """The shortest planned route failed validation and sank the run while a
    slightly longer route one winding along went untried."""
    from ur10e_trajectory_pkg import continuous_validator, warmup

    windings = [{'winding': [0] * 6, 'path': np.zeros((2, 7))},
                {'winding': [-1, 0, 0, 0, 0, 0], 'path': np.ones((2, 7))},
                {'winding': [0, 1, 0, 0, 0, 0], 'path': np.full((2, 7), 2.0)}]
    durations = {0.0: 3.68, 1.0: 3.50, 2.0: 5.05}
    validated = []

    monkeypatch.setattr(home_pose, 'start_windings', lambda validator, path: windings)
    monkeypatch.setattr(warmup, 'plan_route',
                        lambda validator, home, start, via_poses=None: {
                            'status': warmup.OK, 'segments': [{}],
                            'total_duration_s': durations[float(start[0])],
                            'route_kind': 'direct'})

    def validate(validator, route):
        validated.append(route['total_duration_s'])
        passed = route['total_duration_s'] != 3.50
        return {'passed': passed,
                'failures': [] if passed else ['self-clearance 6.5 mm below the floor']}

    monkeypatch.setattr(continuous_validator, 'validate_route', validate)
    best, summary = home_pose.choose_start_winding(None, np.zeros(7), None)

    assert best['warmup']['total_duration_s'] == 3.68
    assert best['warmup_validation']['passed'] is True
    assert validated == [3.50, 3.68]                     # never the 5.05 route
    by_duration = {entry['duration_s']: entry for entry in summary}
    assert by_duration[3.50]['validated'] is False
    assert by_duration[3.50]['validation_failures'] == ['self-clearance 6.5 mm below the floor']
    assert by_duration[5.05]['validated'] is None


def test_no_winding_whose_route_validates_is_no_winding(monkeypatch):
    from ur10e_trajectory_pkg import continuous_validator, warmup

    monkeypatch.setattr(home_pose, 'start_windings', lambda validator, path: [
        {'winding': [0] * 6, 'path': np.zeros((2, 7))}])
    monkeypatch.setattr(warmup, 'plan_route',
                        lambda validator, home, start, via_poses=None: {
                            'status': warmup.OK, 'segments': [{}],
                            'total_duration_s': 1.0, 'route_kind': 'direct'})
    monkeypatch.setattr(continuous_validator, 'validate_route',
                        lambda validator, route: {'passed': False, 'failures': ['x']})
    best, summary = home_pose.choose_start_winding(None, np.zeros(7), None)
    assert best is None and summary[0]['validated'] is False


def test_the_task_plan_carries_what_both_commands_need(compact):
    path = np.stack([compact + np.concatenate(([0.0], np.full(6, 0.01 * i))) for i in range(4)])
    graph = {'recorded_waypoints': 500, 'placement': 'nominal',
             'spin_up': {'requested_duration_s': 2.0, 'duration_s': 2.0}}
    plan = home_pose.task_plan(compact, path, graph, 'graph.json', [0, 0, 0, -1, 0, 0])
    assert plan['task_start'] == plan['q_path'][0]
    assert len(plan['q_path']) == 4 and len(plan['q_path'][0]) == 7
    assert plan['spin_up_s'] == 2.0 and plan['recorded_waypoints'] == 500
    assert plan['home'] == compact.tolist()
    assert plan['start_winding'] == [0, 0, 0, -1, 0, 0]


# --------------------------------------------------------------------------
# What the command checks before it plans
# --------------------------------------------------------------------------

def _choice(validator, configuration, ranking=(), **overrides):
    """A choice artifact whose record matches what recomputation finds."""
    chosen = {'configuration': list(configuration),
              'static_gates': sweep.static_gates(validator, configuration),
              'singularity': sweep.singularity_robustness(validator, configuration)}
    chosen.update(overrides)
    return {'chosen': chosen, 'ranking': list(ranking)}


def _graph(home, kept=12, candidates=42, **overrides):
    document = {
        'path_max_condition': 9.15,
        'candidate_filters': {
            'home_reachable': {'home': list(home), 'kept': kept,
                               'layer_0_candidates': candidates,
                               'routes': {'direct': kept, 'via': 0},
                               'winding_aware': True, 'via_poses_offered': 64,
                               'reasons': {'warmup_collision': candidates - kept}},
            'best_condition_lower_bound': {'value': 6.35, 'layer': 118}}}
    document.update(overrides)
    return document


def test_a_record_that_still_reproduces_passes_the_recheck(validator, compact):
    recheck = home_pose.recheck_home(validator, _choice(validator, compact))
    assert recheck['passed'] is True
    assert recheck['failures'] == [] and recheck['recorded_drift'] == []
    assert recheck['static_gates']['self_clearance_m'] > 0.0


def test_minimal_configured_home_passes_the_recheck(validator, compact):
    choice = {'chosen': {'configuration': compact.tolist()}}
    recheck = home_pose.recheck_home(validator, choice)
    assert recheck['passed'] is True
    assert recheck['recorded_drift'] == []


def test_a_record_that_no_longer_reproduces_is_caught(validator, compact):
    """What catches a stale or hand-edited home_choice.json: the stored
    'passed' is not evidence, agreement with recomputation is."""
    choice = _choice(validator, compact)
    choice['chosen']['static_gates']['posture_margin'] += 0.05
    recheck = home_pose.recheck_home(validator, choice)

    assert recheck['passed'] is False
    assert any('posture_margin' in line for line in recheck['recorded_drift'])
    assert recheck['failures'] == []          # the pose itself is still fine


def test_a_field_the_record_predates_is_not_compared(validator, compact):
    """The self-clearance floor was added after the home was chosen, so the
    record in hand carries no self_clearance_m. That is not drift."""
    choice = _choice(validator, compact)
    del choice['chosen']['static_gates']['self_clearance_m']
    recheck = home_pose.recheck_home(validator, choice)

    assert recheck['recorded_drift'] == [] and recheck['passed'] is True


def test_every_failing_criterion_is_named_with_its_margin(validator, compact):
    """A straight elbow loses the condition neighbourhood and the posture
    margin both, so each is named with the figure it missed by. Which comes
    first is not the point: re-homing needs to know what failed and by how
    much, so the count of named criteria is what matters."""
    straight = compact.copy()
    straight[3] = np.deg2rad(3.0)
    recheck = home_pose.recheck_home(validator, _choice(validator, straight))

    assert recheck['passed'] is False
    condition = [line for line in recheck['failures'] if 'arm condition' in line]
    assert len(condition) == 1
    assert str(int(recheck['singularity']['gate'])) in condition[0]
    assert f"{recheck['singularity']['worst_condition']:.2f}" in condition[0]

    posture = [line for line in recheck['failures'] if 'posture margin' in line]
    assert len(posture) == 1 and '0.0200 floor' in posture[0]


def test_the_thresholds_named_are_the_ones_the_gate_applies():
    """Read from static_gates' signature, so a changed gate cannot leave the
    explanation quoting a stale number."""
    thresholds = home_pose._gate_thresholds()
    applied = inspect.signature(sweep.static_gates).parameters

    assert thresholds['collision_distance_m'] == applied['min_clearance_m'].default
    assert thresholds['joint_limit_clearance'] == applied['min_limit_fraction'].default
    assert thresholds['posture_margin'] == applied['min_posture_margin'].default
    # static_gates defaults this one to None and resolves it at the floor.
    assert applied['min_self_clearance_m'].default is None
    assert thresholds['self_clearance_m'] == motion_limits.SELF_CLEARANCE_FLOOR_M


def test_no_reachable_start_is_distinct_from_no_candidates(compact):
    """The distinction that says whether to re-home or look at generation."""
    unreachable = home_pose.placement_reachability(_graph(compact, kept=0), compact)
    assert unreachable['passed'] is False
    assert unreachable['status'] == home_pose.NO_REACHABLE_START
    assert unreachable['blocked_reasons'] == {'warmup_collision': 42}

    empty = home_pose.placement_reachability(
        _graph(compact, kept=0, candidates=0), compact)
    assert empty['status'] == home_pose.NO_CANDIDATES_HERE


def test_a_graph_filtered_against_another_home_is_refused(compact):
    other = compact.copy()
    other[0] += 0.25
    verdict = home_pose.placement_reachability(_graph(other), compact)
    assert verdict['passed'] is False and verdict['status'] == home_pose.HOME_MISMATCH


def test_an_unfiltered_graph_is_not_reported_as_a_failure(compact):
    """A graph built without --home-json restricted nothing; that is a fact to
    record, not a failure to invent."""
    verdict = home_pose.placement_reachability({'candidate_filters': {}}, compact)
    assert verdict['passed'] is True and verdict['status'] == 'unfiltered'


def test_the_conditioning_cost_reports_achieved_against_achievable(compact):
    cost = home_pose.conditioning_cost(_graph(compact))
    assert cost['achieved_worst_condition'] == 9.15
    assert cost['achievable_bottleneck'] == 6.35
    assert cost['excess'] == pytest.approx(2.80)
    assert cost['bottleneck_layer'] == 118


def test_the_gate_reports_and_stops_rather_than_substituting(validator, compact):
    """A run that plans against a different home than home_choice.json is not
    comparable with any other run, so the gate never swaps one in."""
    choice = _choice(validator, compact)
    choice['chosen']['singularity']['worst_condition'] += 1.0
    gate = home_pose.home_gate(validator, choice, _graph(compact))

    assert gate['passed'] is False
    assert gate['status'] == home_pose.HOME_RECHECK_FAILED
    assert 'suggestions' not in gate
    assert gate['conditioning']['achieved_worst_condition'] == 9.15


def test_the_task_plan_names_the_recording_it_was_validated_against(compact):
    path = np.stack([compact + np.concatenate(([0.0], np.full(6, 0.01 * i)))
                     for i in range(4)])
    graph = {'recorded_waypoints': 500, 'placement': 'nominal',
             'spin_up': {'requested_duration_s': 2.0},
             'recording': {'csv_path': '/graph/side.csv', 'csv_sha256': 'aa' * 32}}
    loaded = {'csv_path': '/commands/side.csv', 'csv_sha256': 'aa' * 32}

    plan = home_pose.task_plan(compact, path, graph, 'graph.json', [0] * 6,
                               recording=loaded)
    assert plan['recording'] == loaded       # what the validating stage loaded
    fallback = home_pose.task_plan(compact, path, graph, 'graph.json', [0] * 6)
    assert fallback['recording'] == graph['recording']
