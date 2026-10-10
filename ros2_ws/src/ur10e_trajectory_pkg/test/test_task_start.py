#!/usr/bin/env python3
"""Command 2 gate: the task plays from its first waypoint, and only if the arm
is measured to be there.

The warmup command brings the arm to the first solved configuration and the
task starts from rest, so the old 2 s unchecked transition is gone, and an
offset between the measured start and the task start would be a step command.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

from ur10e_trajectory_pkg import Validate_trajServer as server
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description'),
        framerate=30)


@pytest.fixture(scope='module')
def first():
    return np.concatenate(([1.2], np.deg2rad([10.0, -110.0, 80.0, -60.0, 70.0, 20.0])))


def _target(validator, configuration):
    pose = validator.robot.fkine(configuration, end='tool0')
    return pose.t, Rotation.from_matrix(pose.R).as_quat()


def test_the_task_plays_from_its_first_configuration_with_no_transition(validator, first):
    path = np.stack([first + 0.01 * i for i in range(11)])
    segment = {'start_idx': 0, 'end_idx': 10, 'length': 11, 'q_full': path}
    q_dot, q_interp, t_sim = validator.process_task_segment(segment, 0.1)
    assert t_sim[0] == 0.0 and t_sim[-1] == pytest.approx(1.0)
    np.testing.assert_allclose(q_interp[0], first, atol=1e-12)
    np.testing.assert_allclose(q_interp[-1], path[-1], atol=1e-12)
    assert len(q_interp) == 31                     # 1.0 s at 30 Hz, both ends
    np.testing.assert_allclose(q_dot[len(q_dot) // 2], 0.1, atol=1e-9)


def test_a_segment_not_starting_at_waypoint_0_cannot_be_played(validator, first):
    segment = {'start_idx': 3, 'end_idx': 10, 'length': 8,
               'q_full': np.stack([first] * 8)}
    with pytest.raises(ValueError, match='waypoint 0'):
        validator.process_task_segment(segment, 0.1)


def test_the_measured_start_matching_the_task_start_passes(validator, first):
    position, quaternion = _target(validator, first)
    ok, details = server.check_measured_start(validator, first, first,
                                              position, quaternion)
    assert ok is True and details['failures'] == []


def test_a_joint_offset_beyond_tolerance_is_refused(validator, first):
    position, quaternion = _target(validator, first)
    measured = first.copy()
    measured[4] += 2 * server.START_ARM_TOL_RAD
    ok, details = server.check_measured_start(validator, measured, first,
                                              position, quaternion)
    assert ok is False
    assert any('arm joint' in failure for failure in details['failures'])


def test_a_rail_offset_beyond_tolerance_is_refused(validator, first):
    position, quaternion = _target(validator, first)
    measured = first.copy()
    measured[0] += 2 * server.START_RAIL_TOL_M
    ok, details = server.check_measured_start(validator, measured, first,
                                              position, quaternion)
    assert ok is False
    assert any('rail' in failure for failure in details['failures'])


def test_a_start_that_misses_the_first_target_is_refused(validator, first):
    """Joints matching the solved configuration are not enough if that
    configuration does not reach the first target."""
    elsewhere = first.copy()
    elsewhere[2] += 0.3
    position, quaternion = _target(validator, elsewhere)
    ok, details = server.check_measured_start(validator, first, first,
                                              position, quaternion)
    assert ok is False
    assert any('first target' in failure for failure in details['failures'])
