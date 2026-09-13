#!/usr/bin/env python3
"""Contract tests for the Jacobian used by the singularity check.

Two defects are recorded here as strict expected failures rather than fixed.
Repairing either changes which trajectories validate, so both belong with the
verification work rather than in a cleanup branch:

1. compute_jacobian passes six arm angles to a seven-joint robot.
   roboticstoolbox pads the short vector with a trailing zero instead of
   raising, so every joint shifts one position -- the rail receives the
   shoulder pan value, and wrist_3 is pinned to zero. The condition number
   the validator reports therefore describes a configuration the robot is
   not in. Measured on one pose: rank 5 and 2.5e16 as computed, against
   rank 6 and 66.0 for the same pose with the full joint vector.

2. The home seed sits on a wrist singularity. HOME_Q has wrist_2 = 0, the
   classic UR degeneracy where the wrist_1 and wrist_3 axes align. Its true
   condition number is infinite. Defect 1 hides this, because the shifted
   metric reports an unremarkable number instead.

Fixing 1 without also moving the seed off the singularity would make every
trajectory seeded from home fail its own singularity check.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg.Validate_trajServer import HOME_Q
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

EE_LINK = 'tool0'
ARM_BASE = 'base_link'

# A pose away from the home wrist singularity, used where the test needs a
# well-conditioned configuration to compare against.
GENERIC_ARM_DEG = (0.0, -135.0, 90.0, -90.0, 45.0, 0.0)


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(),
        mesh_base_path=get_package_share_directory('ur_description'),
        framerate=30,
    )


def _q(rail_m, *arm_deg):
    return np.concatenate(([rail_m], np.deg2rad(arm_deg)))


def _condition(jacobian):
    sv = np.linalg.svd(jacobian, compute_uv=False)
    return sv[0] / sv[-1] if sv[-1] > 1e-12 else np.inf


def test_short_joint_vector_is_padded_rather_than_rejected(validator):
    """Document the library behaviour that causes the mis-indexing.

    roboticstoolbox accepts a six-element vector for a seven-joint robot and
    pads it with a trailing zero. Nothing warns. This test describes the
    toolbox, not our code, so it keeps passing after the defect is fixed and
    explains why it happened.
    """
    arm = np.deg2rad(GENERIC_ARM_DEG)
    short = validator.robot.jacobe(arm, end=EE_LINK, start=ARM_BASE)
    padded = validator.robot.jacobe(
        np.concatenate((arm, [0.0])), end=EE_LINK, start=ARM_BASE
    )
    np.testing.assert_allclose(short, padded)


def test_rail_column_has_no_angular_component(validator):
    """The rail is prismatic, so it can supply no angular velocity.

    It restages posture and so changes the arm's available angular authority,
    but contributes nothing to end-effector rotation directly. Worth pinning
    because the near-term goal is orientation-only tumbling motion, where
    this is the difference between the rail helping and the rail mattering.
    """
    for rail_m in (0.5, 1.5, 2.5):
        jacobian = validator.robot.jacob0(_q(rail_m, *GENERIC_ARM_DEG), end=EE_LINK)
        np.testing.assert_allclose(jacobian[3:, 0], np.zeros(3), atol=1e-12)


@pytest.mark.xfail(
    strict=True,
    reason='Known defect: compute_jacobian passes 6 angles to a 7-joint '
           'robot, which pads with a trailing zero and shifts every joint '
           'one position. Pass the full q vector, then delete this marker.',
)
def test_arm_jacobian_matches_the_full_joint_vector(validator):
    """The singularity metric must describe the configuration actually solved.

    Whether the metric should cover the arm subchain or all seven joints is a
    separate design decision. Either way it has to be evaluated at the real
    configuration, which is what this asserts.
    """
    q_full = _q(1.5, *GENERIC_ARM_DEG)
    as_called = validator.compute_jacobian(q_full[1:])
    at_true_configuration = validator.robot.jacobe(
        q_full, end=EE_LINK, start=ARM_BASE
    )
    np.testing.assert_allclose(as_called, at_true_configuration, atol=1e-9)


@pytest.mark.xfail(
    strict=True,
    reason='Known defect: HOME_Q has wrist_2 = 0, a UR wrist singularity, so '
           'its true condition number is infinite. Move the seed off the '
           'degeneracy, then delete this marker.',
)
def test_home_seed_is_not_singular(validator):
    """The default start pose must not sit on a singularity.

    Every retry is a small perturbation of this seed, so a singular home keeps
    the search inside a degenerate basin and makes rejection the default
    outcome rather than a finding about the trajectory.
    """
    q_full = np.concatenate(([1.5], HOME_Q[1:]))
    jacobian = validator.robot.jacobe(q_full, end=EE_LINK, start=ARM_BASE)
    assert np.linalg.matrix_rank(jacobian) == 6
    assert _condition(jacobian) < 50.0


def test_wrist_singularity_is_at_wrist_2_zero(validator):
    """Pin the cause, so the home fix is aimed at the right joint.

    Rank recovers as wrist_2 moves off zero; conditioning improves
    monotonically with it over this range.
    """
    q_singular = _q(1.5, 0.0, -135.0, 90.0, -90.0, 0.0, 0.0)
    singular = validator.robot.jacobe(q_singular, end=EE_LINK, start=ARM_BASE)
    assert np.linalg.matrix_rank(singular) == 5

    previous = np.inf
    for wrist_2_deg in (5.0, 20.0, 45.0):
        q_off = _q(1.5, 0.0, -135.0, 90.0, -90.0, wrist_2_deg, 0.0)
        jacobian = validator.robot.jacobe(q_off, end=EE_LINK, start=ARM_BASE)
        assert np.linalg.matrix_rank(jacobian) == 6
        current = _condition(jacobian)
        assert current < previous
        previous = current
