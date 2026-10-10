#!/usr/bin/env python3
"""Command 2 plays the validated path: the service verifies it and never re-solves."""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

from ur10e_trajectory_pkg import Validate_trajServer as server
from ur10e_trajectory_pkg.task_path import verify_task_path
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

DT = 0.1


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description'))


@pytest.fixture(scope='module')
def path():
    start = np.concatenate(([1.2], np.deg2rad([10.0, -110.0, 80.0, -60.0, 70.0, 20.0])))
    return np.stack([start + np.concatenate(([0.002 * i], np.full(6, 0.004 * i)))
                     for i in range(12)])


def _targets(validator, path):
    poses = [validator.robot.fkine(q, end='tool0') for q in path]
    return (np.stack([p.t for p in poses]),
            np.stack([Rotation.from_matrix(p.R).as_quat() for p in poses]))


def test_the_path_that_generated_the_targets_verifies(validator, path):
    positions, quaternions = _targets(validator, path)
    report = verify_task_path(validator, path, positions, quaternions, DT)
    assert report['ok'] is True and report['failures'] == []
    assert report['max_position_error_m'] < 1e-9
    assert report['max_step_ratio'] < 1.0


def test_a_configuration_on_another_branch_misses_its_target(validator, path):
    positions, quaternions = _targets(validator, path)
    other = path.copy()
    other[6:, 3] += 0.05                   # elbow off the solution from waypoint 6
    report = verify_task_path(validator, other, positions, quaternions, DT,
                              velocity_limits=np.full(7, 10.0))
    assert report['ok'] is False
    assert any('miss their target, first at waypoint 6' in f for f in report['failures'])


def test_an_unlifted_wrap_is_a_velocity_violation(validator, path):
    """A path must arrive lifted: a raw 2*pi jump between steps is a full
    revolution, not a no-op."""
    positions, quaternions = _targets(validator, path)
    wrapped = path.copy()
    wrapped[5:, 6] -= 2 * np.pi
    report = verify_task_path(validator, wrapped, positions, quaternions, DT)
    assert report['ok'] is False
    assert any('exceed the velocity limits, first from waypoint 4' in f
               for f in report['failures'])


def test_a_colliding_configuration_fails(validator, path, monkeypatch):
    positions, quaternions = _targets(validator, path)
    monkeypatch.setattr(validator, 'check_all_collisions',
                        lambda q, verbose=False: bool(np.isclose(q[0], path[3][0])))
    report = verify_task_path(validator, path, positions, quaternions, DT)
    assert report['collisions'] == 1
    assert any('in collision, first at waypoint 3' in f for f in report['failures'])


def test_a_path_of_the_wrong_length_fails(validator, path):
    positions, quaternions = _targets(validator, path)
    report = verify_task_path(validator, path[:-1], positions, quaternions, DT)
    assert report['ok'] is False and 'shape' in report['failures'][0]


def test_the_request_must_carry_the_joint_path():
    with pytest.raises(ValueError, match='no longer re-solves'):
        server.resolve_task_path([], 10)
    with pytest.raises(ValueError, match='need 70'):
        server.resolve_task_path([0.0] * 69, 10)
    assert server.resolve_task_path([0.0] * 70, 10).shape == (10, 7)


def test_a_continuous_failure_is_explained_not_just_refused():
    """The service refuses a path whose motion between waypoints fails, and
    says why, so the caller can tell a collision from a limit or a twist."""
    passing = {'limit_violations': {}, 'position_limit_violations': [],
               'collision': {'collision_found': False},
               'tracking': {'within_tolerance': True}, 'conditioning_ok': True,
               'conditioning': {'twist_status': 'pass'}}
    assert server.continuous_failures(passing) == []
    failing = dict(passing,
                   limit_violations={'elbow_joint': {'jerk': {'peak': 1108.0, 'limit': 500.0}}},
                   collision={'collision_found': True},
                   conditioning={'twist_status': 'fail'})
    statuses = {'jerk': ['assumed'] * 7, 'velocity': ['certified'] * 7,
                'acceleration': ['provisional'] * 7}
    reasons = server.continuous_failures(failing, statuses)
    assert 'elbow_joint jerk 1.11e+03 > 500 (assumed limit)' in reasons
    assert 'collision between waypoints' in reasons
    assert 'task twist fail' in reasons


def test_the_service_validates_continuously_at_the_offline_rate():
    assert server.SERVICE_VALIDATION_HZ == 200.0



# --------------------------------------------------------------------------
# Command 1: the service plans, validates and frames the warmup itself
# --------------------------------------------------------------------------

@pytest.fixture(scope='module')
def home():
    return np.concatenate(([1.5], np.deg2rad([0.0, -75.0, 100.0, -115.0, -80.0, 0.0])))


def test_a_valid_warmup_is_framed_at_the_playback_rate(validator, home):
    target = home + np.concatenate(([0.3], np.deg2rad([15.0, -10.0, 10.0, 5.0, 20.0, 30.0])))
    ok, message, frames, plan = server.plan_and_validate_warmup(validator, home, target, 30.0)
    assert ok is True and 'Warmup validated' in message
    np.testing.assert_allclose(frames[0], home, atol=1e-12)
    np.testing.assert_allclose(frames[-1], target, atol=1e-12)
    assert len(frames) == int(np.ceil(plan['duration_s'] * 30.0)) + 1


def test_a_blocked_warmup_is_refused_and_nothing_is_framed(validator, home, monkeypatch):
    monkeypatch.setattr(validator, 'check_all_collisions', lambda q, verbose=False: True)
    target = home + np.concatenate(([0.3], np.zeros(6)))
    ok, message, frames, plan = server.plan_and_validate_warmup(validator, home, target, 30.0)
    assert ok is False and frames is None
    assert 'no_direct_warmup' in message


def test_warmup_requests_need_full_configurations():
    with pytest.raises(ValueError, match='q_target must have 7'):
        server.resolve_configuration([0.0] * 6, 'q_target')
    assert server.resolve_configuration([0.0] * 7, 'q_start').shape == (7,)
