#!/usr/bin/env python3
"""Stage 2b gate: a configuration that misses its target is not a solution.

ikine_LM's success flag measures convergence of its local search, not
distance to the target, so it reports success from a local minimum metres
away. Acceptance used to check only that flag.

Pose mismatch is its own failure class and outranks collision, singularity
and velocity in the primary reason, because those describe properties of a
configuration that does not answer the commanded target at all. Saying a
trajectory is singular, when the configuration measured was metres from where
it was asked to be, describes the wrong thing.

Every flag is kept. "Pose mismatch plus collision" does not establish that the
requested pose collides.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

from ur10e_trajectory_pkg.validation_core import (
    FAILURE_FLAGS,
    IK_ORIENTATION_TOL_RAD,
    IK_POSITION_TOL_M,
    TrajectoryValidator,
    format_failure,
)

from test_geometry_invariants import _urdf_path


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description')
    )


@pytest.fixture(scope='module')
def reachable(validator):
    q_full = np.concatenate(([1.5], np.deg2rad([0.0, -135.0, 90.0, -90.0, 45.0, 0.0])))
    pose = validator.robot.fkine(q_full, end='tool0')
    return pose.t.copy(), np.roll(np.array(pose.UnitQuaternion().A), -1)


def _solve(validator, target, quaternion, seed_deg=(0.0, -135.0, 90.0, -90.0, 45.0, 0.0)):
    validator.reset_rng()
    return validator._solve_waypoint_with_recovery(
        target, quaternion, np.deg2rad(seed_deg), rail_pos=1.5,
        check_jump=False, verbose=False,
    )


# --------------------------------------------------------------------------
# The gate itself
# --------------------------------------------------------------------------

def test_solver_success_metres_from_target_is_rejected(validator, reachable):
    position, quaternion = reachable
    result = _solve(validator, np.array([9.0, position[1], position[2]]), quaternion)
    assert not result['ok']
    assert 'POSE_MISMATCH' in result['reason']


def test_a_reachable_target_still_passes(validator, reachable):
    position, quaternion = reachable
    result = _solve(validator, position, quaternion)
    assert result['ok'], result['reason']


def test_every_accepted_result_satisfies_both_tolerances(validator, reachable):
    """The property the gate exists to guarantee."""
    position, quaternion = reachable
    for offset in (0.0, 0.05, 0.1, -0.08):
        target = position + np.array([offset, 0.0, 0.0])
        result = _solve(validator, target, quaternion)
        if not result['ok']:
            continue
        reached = validator.robot.fkine(result['q_full'], end='tool0')
        assert np.linalg.norm(reached.t - target) <= IK_POSITION_TOL_M
        relative = reached.R.T @ Rotation.from_quat(quaternion).as_matrix()
        assert Rotation.from_matrix(relative).magnitude() <= IK_ORIENTATION_TOL_RAD


def test_a_pose_mismatch_enters_the_retry_loop(validator, reachable):
    """Rejected like any other failure, not short-circuited.

    A pose mismatch must be retried, because a different branch may reach the
    target even when the first one did not.
    """
    position, quaternion = reachable
    result = _solve(validator, np.array([9.0, position[1], position[2]]), quaternion)
    assert result['attempts_used'] > 1


# --------------------------------------------------------------------------
# Classification, via the formatter rather than an if/elif chain
# --------------------------------------------------------------------------

def _details():
    return {
        'pose': 'position 1.0 m > 0.001 m; orientation 0.0 deg <= 0.1 deg.',
        'collision': 'Collision at reached target configuration: q=...',
        'singular': 'Singularity: condition number 900.00 > 50.0.',
        'arm_velocity_failed': 'Arm velocity exceeded 2.0 rad/s at joints [1].',
        'rail_velocity_failed': 'Rail velocity 3.0 m/s exceeded 0.5 m/s.',
    }


def _flags(**overrides):
    flags = {name: False for name in FAILURE_FLAGS}
    flags.update(overrides)
    return flags


@pytest.mark.parametrize('overrides', [
    {'pose_position_failed': True},
    {'pose_orientation_failed': True},
    {'pose_position_failed': True, 'pose_orientation_failed': True},
])
def test_position_orientation_and_combined_mismatches_all_classify_as_pose(overrides):
    message = format_failure(_flags(**overrides), _details())
    assert message.startswith('POSE_MISMATCH')


def test_pose_mismatch_outranks_collision_and_singularity():
    """Precedence, and the other flags still reported."""
    message = format_failure(
        _flags(pose_position_failed=True, collision=True, singular=True),
        _details())
    assert message.startswith('POSE_MISMATCH')
    assert 'Also failed: collision, singular.' in message


def test_collision_without_a_pose_mismatch_reads_as_collision():
    """The distinction that matters: this one does say the requested pose
    collides."""
    message = format_failure(_flags(collision=True), _details())
    assert message.startswith('Collision at reached target configuration')
    assert 'POSE_MISMATCH' not in message


def test_solver_failure_outranks_everything():
    message = format_failure(
        _flags(solver_failed=True, pose_position_failed=True),
        {'solver_failed': 'IK did not converge (reason=x).'})
    assert message.startswith('IK did not converge')


def test_simultaneous_failures_retain_every_flag():
    flags = _flags(pose_orientation_failed=True, collision=True,
                   arm_velocity_failed=True, rail_velocity_failed=True)
    message = format_failure(flags, _details())
    for name in ('collision', 'arm_velocity_failed', 'rail_velocity_failed'):
        assert name in message


def test_position_and_orientation_are_never_collapsed():
    assert 'pose_position_failed' in FAILURE_FLAGS
    assert 'pose_orientation_failed' in FAILURE_FLAGS


# --------------------------------------------------------------------------
# Tolerance boundaries
# --------------------------------------------------------------------------

def test_values_exactly_on_the_tolerance_pass():
    """Inclusive bounds, so a value at the limit is not a failure."""
    from ur10e_trajectory_pkg.pose_metrics import within_pose_tolerance
    assert within_pose_tolerance(IK_POSITION_TOL_M, IK_ORIENTATION_TOL_RAD)


@pytest.mark.parametrize('position_err,orientation_err', [
    (IK_POSITION_TOL_M * 1.000001, 0.0),
    (0.0, IK_ORIENTATION_TOL_RAD * 1.000001),
])
def test_values_immediately_outside_the_tolerance_fail(position_err, orientation_err):
    from ur10e_trajectory_pkg.pose_metrics import within_pose_tolerance
    assert not within_pose_tolerance(position_err, orientation_err)


def test_quaternion_sign_reversal_is_equivalent_at_the_gate(validator, reachable):
    """q and -q are the same rotation, so they must give the same verdict."""
    position, quaternion = reachable
    positive = _solve(validator, position, quaternion)
    negative = _solve(validator, position, -np.asarray(quaternion))
    assert positive['ok'] == negative['ok']
    np.testing.assert_allclose(positive['q_full'], negative['q_full'], atol=1e-9)
