#!/usr/bin/env python3
"""Self-clearance gate: one function, the validator's adjacency rule, a 10 mm floor."""
import json

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg import continuous_validator as cv
from ur10e_trajectory_pkg import graph_planner, motion_limits
from ur10e_trajectory_pkg import ready_pose_sweep as sweep
from ur10e_trajectory_pkg import warmup
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description'))


@pytest.fixture(scope='module')
def compact():
    return np.concatenate(([1.5], np.deg2rad([0.0, -75.0, 100.0, -115.0, -80.0, 0.0])))


def test_the_floor_is_declared_once():
    assert motion_limits.SELF_CLEARANCE_FLOOR_M == 0.010


def test_a_folded_arm_has_no_clearance(validator):
    folded = np.concatenate(([1.5], np.deg2rad([0.0, 80.0, 0.0, 0.0, 0.0, 0.0])))
    assert validator.check_all_collisions(folded)
    assert validator.self_clearance(folded)['distance_m'] < motion_limits.SELF_CLEARANCE_FLOOR_M


def test_design_overlaps_are_skipped_by_the_adjacency_rule(validator, compact):
    """The base inertia link sits inside the carriage by design; the boolean
    test skips that pair and so must self_clearance, or every pose fails."""
    result = validator.self_clearance(compact)
    assert result['distance_m'] > 0.0
    if result['links'] is not None:
        a, b = (validator._link_index_by_name[n] for n in result['links'])
        assert not validator._is_adjacent(a, b)


def test_the_home_pose_clears_the_floor(validator, compact):
    assert validator.self_clearance(compact)['distance_m'] >= motion_limits.SELF_CLEARANCE_FLOOR_M


def test_static_gates_refuse_a_pose_below_the_self_clearance_floor(validator, compact,
                                                                   monkeypatch):
    gates = sweep.static_gates(validator, compact)
    assert 'self_clearance_m' in gates and gates['passed'] is True
    monkeypatch.setattr(validator, 'self_clearance',
                        lambda q, max_distance=0.05: {'distance_m': 0.004, 'links': ('a', 'b')})
    assert sweep.static_gates(validator, compact)['passed'] is False


def _poses(validator, path):
    from scipy.spatial.transform import Rotation
    poses = [validator.robot.fkine(q, end='tool0') for q in path]
    return (np.stack([p.t for p in poses]),
            np.stack([Rotation.from_matrix(p.R).as_quat() for p in poses]))


def test_a_grazing_task_path_fails_continuous_validation(validator, compact, monkeypatch):
    path = np.stack([compact + np.concatenate(([0.001 * i], np.full(6, 0.002 * i)))
                     for i in range(5)])
    positions, quaternions = _poses(validator, path)
    clear = cv.validate_task_command(validator, path, positions, quaternions, 0.1,
                                     rate_hz=50.0)
    assert clear['self_clearance']['passed'] is True
    monkeypatch.setattr(validator, 'self_clearance',
                        lambda q, max_distance=0.05: {'distance_m': 0.006,
                                                      'links': ('forearm_link', 'wrist_2_link')})
    grazing = cv.validate_task_command(validator, path, positions, quaternions, 0.1,
                                       rate_hz=50.0)
    assert grazing['self_clearance']['passed'] is False
    assert grazing['self_clearance']['links'] == ('forearm_link', 'wrist_2_link')
    assert grazing['passed'] is False


def test_a_grazing_warmup_fails_validation(validator, compact, monkeypatch):
    target = compact + np.concatenate(([0.2], np.deg2rad([10.0, -5.0, 5.0, 0.0, 10.0, 15.0])))
    plan = warmup.plan_warmup(validator, compact, target, rate_hz=100.0)
    assert cv.validate_warmup(validator, plan)['passed'] is True
    monkeypatch.setattr(validator, 'self_clearance',
                        lambda q, max_distance=0.05: {'distance_m': 0.002, 'links': ('a', 'b')})
    report = cv.validate_warmup(validator, plan)
    assert report['self_clearance']['passed'] is False and report['passed'] is False


def test_an_approach_passing_close_is_infeasible(validator, compact, monkeypatch):
    """The home and the target can both clear the floor while the approach
    between them does not, which is what the coverage sweep judges."""
    target = compact + np.concatenate(([0.3], np.deg2rad([15.0, -10.0, 10.0, 5.0, 10.0, 20.0])))
    limits = validator.velocity_limits
    accelerations = np.full(len(limits), 1.0)
    zeros = np.zeros(len(limits))
    clear = sweep.evaluate_approach(validator, compact, target, zeros, zeros,
                                    limits, accelerations)
    assert clear['feasible'] is True
    assert clear['self_clearance']['min_distance_m'] >= motion_limits.SELF_CLEARANCE_FLOOR_M

    calls = {'n': 0}

    def grazing(q, max_distance=0.05):
        calls['n'] += 1
        return {'distance_m': 0.05 if calls['n'] < 3 else 0.007,
                'links': ('forearm_link', 'wrist_2_link')}

    monkeypatch.setattr(validator, 'self_clearance', grazing)
    graze = sweep.evaluate_approach(validator, compact, target, zeros, zeros,
                                    limits, accelerations)
    assert graze['feasible'] is False
    assert graze['reason_code'] == sweep.REASON_SELF_CLEARANCE
    assert graze['self_clearance']['min_distance_m'] == 0.007


def test_graph_candidates_are_filtered_by_condition_and_self_clearance(tmp_path, validator,
                                                                       compact, monkeypatch):
    document = {'candidates': {'0': [
        {'rail_position': 1.0, 'q_arm_canonical': [0.0] * 6, 'arm_condition_number': 8.0},
        {'rail_position': 1.1, 'q_arm_canonical': [0.1] * 6, 'arm_condition_number': 40.0},
        {'rail_position': 1.2, 'q_arm_canonical': [0.2] * 6, 'arm_condition_number': 12.0},
    ], '1': [
        {'rail_position': 1.0, 'q_arm_canonical': [0.0] * 6, 'arm_condition_number': 30.0},
        {'rail_position': 1.0, 'q_arm_canonical': [0.3] * 6, 'arm_condition_number': 22.0},
    ]}}
    path = tmp_path / 'candidates.json'
    path.write_text(json.dumps(document))
    layers, loaded = graph_planner.load_candidates(path, 2, include_oracle=False,
                                                   max_condition=25.0)
    assert [len(l) for l in layers] == [2, 1]
    assert loaded['condition_filter']['removed_per_layer'] == [1, 1]
    assert graph_planner.best_condition_lower_bound(loaded, 2) == {'value': 22.0, 'layer': 1}

    monkeypatch.setattr(validator, 'self_clearance',
                        lambda q, max_distance=0.05: {'distance_m': 0.02 if q[0] < 1.15 else 0.005,
                                                      'links': None})
    kept, stats = graph_planner.filter_self_clearance(validator, layers, 0.010)
    assert [len(l) for l in kept] == [1, 1]
    assert stats['removed_total'] == 1 and stats['queries'] == 3
