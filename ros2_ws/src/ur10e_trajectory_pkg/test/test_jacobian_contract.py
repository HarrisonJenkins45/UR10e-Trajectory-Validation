#!/usr/bin/env python3
"""Stage 1 gate: the Jacobians are correct, and say which is which.

Two functions with separate jobs, both taking the complete seven-joint
configuration and returning twists in the world frame:

  compute_system_jacobian  6x7  can the rail and arm together produce a
                                commanded Cartesian motion?
  compute_arm_jacobian     6x6  is the UR arm itself near a kinematic
                                singularity, which the rail cannot rescue?

These replace a single function that took six arm angles. roboticstoolbox
padded that short vector with a trailing zero rather than raising, so every
joint shifted one position and wrist_3 was pinned to zero. The reported
condition number described a configuration the robot was not in: rank 5 and
2.5e16 on a pose whose true values are rank 6 and 66.0.

Correctness is established against central finite differences of forward
kinematics, so these tests do not simply restate the same library call.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg.configurations import (
    ARM_SLICE,
    JOINT_NAMES,
    LEGACY_MATLAB_START_Q,
    NUM_JOINTS,
)
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

EE_LINK = 'tool0'

# Step for central differences: large enough that the pose change clears
# numerical noise, small enough that second-order terms stay negligible.
FD_STEP = 1e-6
FD_TOL = 1e-6

# Postures clear of the wrist degeneracy, spanning different arm shapes.
NONSINGULAR_ARMS_DEG = (
    (0.0, -135.0, 90.0, -90.0, 45.0, 0.0),
    (30.0, -100.0, 60.0, -70.0, 80.0, 25.0),
    (-45.0, -120.0, 110.0, -60.0, -50.0, 90.0),
)


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(),
        mesh_base_path=get_package_share_directory('ur_description'),
        framerate=30,
    )


def _q(rail_m, arm_deg):
    return np.concatenate(([rail_m], np.deg2rad(arm_deg)))


def _condition(jacobian):
    sv = np.linalg.svd(jacobian, compute_uv=False)
    return sv[0] / sv[-1] if sv[-1] > 1e-12 else np.inf


def _log_so3(rotation):
    """Rotation-vector form of a rotation matrix.

    Angular error comes from the relative rotation's logarithm rather than
    from subtracting quaternion or Euler components, which are not
    differences in any useful sense and break across sign flips and wrap.
    """
    angle = np.arccos(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-12:
        return np.zeros(3)
    axis = np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ]) / (2.0 * np.sin(angle))
    return axis * angle


def _finite_difference_column(validator, q_full, joint_index, step=FD_STEP):
    """Central-difference twist for one joint, in the world frame."""
    forward, backward = np.array(q_full, float), np.array(q_full, float)
    forward[joint_index] += step
    backward[joint_index] -= step

    pose_forward = validator.robot.fkine(forward, end=EE_LINK)
    pose_backward = validator.robot.fkine(backward, end=EE_LINK)

    linear = (pose_forward.t - pose_backward.t) / (2.0 * step)
    angular = _log_so3(pose_forward.R @ pose_backward.R.T) / (2.0 * step)
    return np.concatenate((linear, angular))


# --------------------------------------------------------------------------
# Shapes, ordering and input validation
# --------------------------------------------------------------------------

def test_system_jacobian_has_one_column_per_joint(validator):
    jacobian = validator.compute_system_jacobian(_q(1.5, NONSINGULAR_ARMS_DEG[0]))
    assert jacobian.shape == (6, NUM_JOINTS)
    assert len(JOINT_NAMES) == NUM_JOINTS


def test_arm_jacobian_is_six_by_six(validator):
    jacobian = validator.compute_arm_jacobian(_q(1.5, NONSINGULAR_ARMS_DEG[0]))
    assert jacobian.shape == (6, 6)


def test_arm_jacobian_is_the_arm_columns_of_the_system_jacobian(validator):
    """Same configuration, same frame, so the two are directly comparable."""
    q_full = _q(1.2, NONSINGULAR_ARMS_DEG[1])
    np.testing.assert_allclose(
        validator.compute_arm_jacobian(q_full),
        validator.compute_system_jacobian(q_full)[:, ARM_SLICE],
        atol=1e-12,
    )


@pytest.mark.parametrize('bad_length', [1, 6, 8, 14])
def test_wrong_sized_configurations_are_rejected(validator, bad_length):
    """The six-value call is the defect this API exists to prevent.

    Six is the plausible mistake, being the arm without the rail, and it is
    exactly the shape roboticstoolbox silently padded.
    """
    for method in (validator.compute_system_jacobian, validator.compute_arm_jacobian):
        with pytest.raises(ValueError, match=str(NUM_JOINTS)):
            method(np.zeros(bad_length))


@pytest.mark.parametrize('bad_value', [np.nan, np.inf, -np.inf])
def test_non_finite_configurations_are_rejected(validator, bad_value):
    q_full = _q(1.5, NONSINGULAR_ARMS_DEG[0])
    q_full[3] = bad_value
    with pytest.raises(ValueError, match='finite'):
        validator.compute_system_jacobian(q_full)


# --------------------------------------------------------------------------
# Correctness against finite differences of forward kinematics
# --------------------------------------------------------------------------

@pytest.mark.parametrize('arm_deg', NONSINGULAR_ARMS_DEG)
def test_every_column_matches_central_differences(validator, arm_deg):
    """Independent check: differentiate FK rather than trust the same call.

    Angular rows use the relative-rotation logarithm, so this catches a
    Jacobian that is right in translation and wrong in rotation.
    """
    q_full = _q(1.4, arm_deg)
    analytic = validator.compute_system_jacobian(q_full)
    for joint_index in range(NUM_JOINTS):
        numeric = _finite_difference_column(validator, q_full, joint_index)
        np.testing.assert_allclose(
            analytic[:, joint_index], numeric, atol=FD_TOL,
            err_msg=f'column {joint_index} ({JOINT_NAMES[joint_index]}) '
                    'disagrees with finite differences',
        )


def test_rail_column_is_pure_translation_along_the_modelled_axis(validator):
    """The rail is prismatic along world X.

    The zero angular part is what makes it unable to supply angular velocity,
    which matters for the orientation-only tumbling case. Unit linear
    magnitude is what makes a metre of rail a metre of tool travel.
    """
    for rail_m in (0.0, 1.5, 3.0):
        column = validator.compute_system_jacobian(
            _q(rail_m, NONSINGULAR_ARMS_DEG[0])
        )[:, 0]
        np.testing.assert_allclose(column[3:], np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(column[:3], np.array([1.0, 0.0, 0.0]), atol=1e-9)


# --------------------------------------------------------------------------
# Singularity classification
# --------------------------------------------------------------------------

def test_legacy_start_posture_is_classified_rank_deficient(validator):
    """The legacy MATLAB posture is singular, and must be seen to be.

    wrist_2 = 0 aligns the wrist_1 and wrist_3 axes. The previous metric
    reported an unremarkable number for it, which is how a singular posture
    survived as the default seed.
    """
    q_full = np.concatenate(([1.5], LEGACY_MATLAB_START_Q[ARM_SLICE]))
    jacobian = validator.compute_arm_jacobian(q_full)
    assert np.linalg.matrix_rank(jacobian) == 5
    assert not np.isfinite(_condition(jacobian))


@pytest.mark.parametrize('arm_deg', NONSINGULAR_ARMS_DEG)
def test_nonsingular_postures_recover_full_rank(validator, arm_deg):
    jacobian = validator.compute_arm_jacobian(_q(1.5, arm_deg))
    assert np.linalg.matrix_rank(jacobian) == 6
    assert np.isfinite(_condition(jacobian))


def test_conditioning_improves_as_wrist_2_leaves_zero(validator):
    """Pin the cause, so a seed sweep is aimed at the right joint."""
    previous = np.inf
    for wrist_2_deg in (5.0, 20.0, 45.0):
        jacobian = validator.compute_arm_jacobian(
            _q(1.5, (0.0, -135.0, 90.0, -90.0, wrist_2_deg, 0.0))
        )
        assert np.linalg.matrix_rank(jacobian) == 6
        current = _condition(jacobian)
        assert current < previous
        previous = current


def test_rail_position_does_not_change_arm_conditioning(validator):
    """Sliding the base translates the arm without reorienting it.

    Arm conditioning is therefore a property of the arm's posture alone.
    Worth pinning: it is why the rail cannot rescue a wrist singularity, and
    why the two Jacobians answer different questions.
    """
    conditions = [
        _condition(validator.compute_arm_jacobian(_q(rail_m, NONSINGULAR_ARMS_DEG[2])))
        for rail_m in (0.0, 1.5, 3.0)
    ]
    for value in conditions[1:]:
        assert value == pytest.approx(conditions[0], rel=1e-9)
