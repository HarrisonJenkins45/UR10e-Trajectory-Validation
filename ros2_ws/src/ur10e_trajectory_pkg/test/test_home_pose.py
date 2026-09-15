#!/usr/bin/env python3
"""Home-pose gate: arm clearance, singularity robustness, coverage, one ranking key."""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg import home_pose
from ur10e_trajectory_pkg import ready_pose_sweep as sweep
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
    import pybullet as pb
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
# Choosing
# --------------------------------------------------------------------------

def _record(name, condition, duration, connectivity=1.0, fraction=1.0,
            gates=True, singular=True):
    return {'provenance': name, 'configuration': [0.0] * 7,
            'static_gates': {'passed': gates},
            'singularity': {'worst_condition': condition, 'passed': singular},
            'screen_passed': gates and singular,
            'coverage': {'connectivity': connectivity,
                         'worst_family_fraction': fraction,
                         'worst_duration_s': duration},
            'full_coverage': connectivity == 1.0 and fraction == 1.0,
            'passed': gates and singular and connectivity == 1.0 and fraction == 1.0}


def test_passing_poses_rank_on_neighbourhood_condition_then_duration():
    records = [_record('slow_but_robust', 8.0, 4.0), _record('fast', 12.0, 1.0),
               _record('tie_slower', 8.0, 5.0)]
    decision = home_pose.choose_home(records)
    assert decision['full_coverage_found'] is True
    assert [r['provenance'] for r in decision['ranking']] == [
        'slow_but_robust', 'tie_slower', 'fast']


def test_floors_are_not_ranking_keys():
    """A pose failing any floor is out, however good its other measures."""
    records = [_record('robust_no_coverage', 6.0, 1.0, fraction=0.8),
               _record('singular', 5.0, 1.0, singular=False),
               _record('ok', 20.0, 3.0)]
    assert home_pose.choose_home(records)['chosen']['provenance'] == 'ok'


def test_without_full_coverage_the_best_coverage_is_taken_and_reported():
    records = [_record('a', 6.0, 1.0, fraction=0.8),
               _record('b', 9.0, 1.0, fraction=0.9)]
    decision = home_pose.choose_home(records)
    assert decision['full_coverage_found'] is False
    assert decision['chosen']['provenance'] == 'b'
    assert decision['shortfall'] == {'connectivity': 1.0, 'worst_family_fraction': 0.9}


def test_coverage_joins_screening_by_configuration():
    screen_records = [dict(_record('x', 5.0, 1.0), configuration=[0.1] * 7)]
    coverage = [{'configuration': [0.1] * 7,
                 'summary': {'connectivity': 1.0, 'worst_family_fraction': 1.0,
                             'worst_duration_s': 2.0}}]
    combined = home_pose.combine(screen_records, coverage)
    assert combined[0]['full_coverage'] is True and combined[0]['passed'] is True


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
    ok = [s['duration_s'] for s in summary if s['status'] == 'ok']
    assert best['warmup']['duration_s'] == min(ok)
