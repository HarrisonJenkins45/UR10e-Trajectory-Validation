#!/usr/bin/env python3
"""Refinement gate: smooth rail, arm kept on its branch, validated or refused."""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

from ur10e_trajectory_pkg import continuous_validator
from ur10e_trajectory_pkg import path_refinement as refinement
from ur10e_trajectory_pkg.ready_pose_sweep import arm_only_ik
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

DT = 0.1
N = 14


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description'))


@pytest.fixture(scope='module')
def smooth(validator):
    """A smooth path and the targets it reaches: rail creeping, arm turning."""
    start = np.concatenate(([1.2], np.deg2rad([10.0, -110.0, 80.0, -60.0, 70.0, 20.0])))
    path = np.stack([start + np.concatenate(([0.004 * i], np.full(6, 0.003 * i)))
                     for i in range(N)])
    poses = [validator.robot.fkine(q, end='tool0') for q in path]
    positions = np.stack([p.t for p in poses])
    quaternions = np.stack([Rotation.from_matrix(p.R).as_quat() for p in poses])
    return path, positions, quaternions


def test_the_smoothed_rail_holds_the_start_and_meets_its_rms():
    rng = np.random.default_rng(3)
    rail = np.linspace(0.5, 0.8, 200) + rng.normal(0.0, 0.003, 200)
    f, _, rms = refinement.smooth_rail(rail, 0.002)
    np.testing.assert_array_equal(f[:refinement.PINNED_START_SAMPLES], rail[0])
    assert rms == pytest.approx(0.002, rel=0.05)


def test_a_looser_level_is_smoother():
    rng = np.random.default_rng(4)
    rail = np.linspace(0.5, 0.8, 200) + rng.normal(0.0, 0.003, 200)
    tight, _, _ = refinement.smooth_rail(rail, 0.001)
    loose, _, _ = refinement.smooth_rail(rail, 0.005)
    assert (np.max(np.abs(np.diff(loose, n=2))) < np.max(np.abs(np.diff(tight, n=2))))


def test_an_unreachable_target_returns_the_smoothest_fit():
    rail = np.full(50, 1.0)
    f, _, rms = refinement.smooth_rail(rail, 0.005)
    assert rms <= 0.005
    np.testing.assert_allclose(f, 1.0, atol=1e-9)


def test_the_arm_stays_on_the_path_branch_at_the_path_rail(validator, smooth):
    path, positions, quaternions = smooth
    solved, report = refinement.solve_arm_along(validator, path, path[:, 0],
                                                positions, quaternions)
    assert solved is not None
    np.testing.assert_allclose(solved, path, atol=1e-6)
    assert report['max_arm_deviation_from_graph_rad'] < 1e-6


def test_a_rail_hop_is_smoothed_out(validator, smooth):
    """The graph can hop 3 mm along the rail at one waypoint while the arm
    compensates; smoothing the rail and re-solving the arm removes it.

    The rail starts at rest here, as it does after a spin-up: refinement pins
    the start's rail samples, which assumes exactly that.
    """
    path, positions, quaternions = smooth
    at_rest = path.copy()
    at_rest[:, 0] = path[0, 0]
    targets_positions, targets_quaternions = [], []
    for q in at_rest:
        pose = validator.robot.fkine(q, end='tool0')
        targets_positions.append(pose.t)
        targets_quaternions.append(Rotation.from_matrix(pose.R).as_quat())
    targets_positions = np.stack(targets_positions)
    targets_quaternions = np.stack(targets_quaternions)

    kinked = at_rest.copy()
    kinked[7, 0] += 0.003
    rotation = Rotation.from_quat(targets_quaternions[7]).as_matrix()
    kinked[7] = arm_only_ik(validator, kinked[7], targets_positions[7], rotation)[0]
    rail, _, _ = refinement.smooth_rail(kinked[:, 0], 0.002)
    solved, _ = refinement.solve_arm_along(validator, kinked, rail, targets_positions,
                                           targets_quaternions)
    assert solved is not None
    before = np.max(np.abs(np.diff(kinked, n=2, axis=0)))
    after = np.max(np.abs(np.diff(solved, n=2, axis=0)))
    assert after < 0.5 * before, (before, after)
    np.testing.assert_allclose(solved[0], kinked[0], atol=1e-9)


def _report(passed):
    return {'passed': passed, 'entry_velocity_ratio': 0.0, 'entry_acceleration_ratio': 0.0,
            'conditioning': {'max_condition_number': 10.0, 'twist_status': 'pass'},
            'limit_violations': {} if passed else {'elbow_joint': {}},
            'collision': {'collision_found': False},
            'tracking': {'within_tolerance': True},
            'peak_command_stream': {'jerk': [1.0] * 7}}


def test_the_smoothest_passing_level_is_kept(validator, smooth, monkeypatch):
    path, positions, quaternions = smooth
    outcomes = iter([False, True, True])
    monkeypatch.setattr(continuous_validator, 'validate_task_command',
                        lambda *a, **k: _report(next(outcomes)))
    result = refinement.refine_path(validator, path, positions, quaternions, DT)
    assert result['status'] == 'refined'
    assert result['level_m'] == refinement.REFINEMENT_RMS_LEVELS_M[1]
    assert [a['passed'] for a in result['attempts']] == [False, True]


def test_if_no_level_passes_the_graph_path_is_kept(validator, smooth, monkeypatch):
    """An unvalidated path is never substituted."""
    path, positions, quaternions = smooth
    monkeypatch.setattr(continuous_validator, 'validate_task_command',
                        lambda *a, **k: _report(False))
    result = refinement.refine_path(validator, path, positions, quaternions, DT)
    assert result['status'] == 'refinement_failed'
    np.testing.assert_array_equal(result['path'], path)
    assert result['validation'] is None
    assert refinement.meets_acceptance(result) is False


def test_acceptance_needs_small_second_differences_and_no_start_motion():
    attempt = {'level_m': 0.002, 'passed': True, 'max_second_difference': 5e-4,
               'start_motion': 1e-5}
    ok = {'status': 'refined', 'level_m': 0.002, 'attempts': [attempt]}
    assert refinement.meets_acceptance(ok) is True
    rough = {'status': 'refined', 'level_m': 0.002,
             'attempts': [dict(attempt, max_second_difference=2e-3)]}
    assert refinement.meets_acceptance(rough) is False
    moving = {'status': 'refined', 'level_m': 0.002,
              'attempts': [dict(attempt, start_motion=0.0126)]}
    assert refinement.meets_acceptance(moving) is False
