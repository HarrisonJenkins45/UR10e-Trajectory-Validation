#!/usr/bin/env python3
"""The 419-waypoint baseline is a joint-wrapping artifact.

roboticstoolbox returns every solution wrapped into [-pi, pi], but five of the
six arm joints permit +/-2*pi. A smooth motion crossing that boundary
therefore appears as a joint delta of nearly 2*pi: a representation
discontinuity, not a physical velocity.

Audited over the 500-waypoint trajectory, all 17 arm-velocity failures are
wrist_1 deltas within 0.05 rad of -2*pi, clustered at waypoints 79 to 81. With
the arm-velocity gate disabled the trajectory is one segment of 500; with it
enabled it splits into [0, 80] and [81, 499] at exactly that point.

So the trajectory is feasible end to end, and the reported 419 is the length
of the larger of two pieces cut by an angle representation. The planner will
need lifted joint values, q + 2*pi*k, bounded by each joint's real limits.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg.configurations import JOINT_NAMES
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

TWO_PI = 2.0 * np.pi

# Joints whose declared range spans 4*pi, so a lift of +/-2*pi can stay legal.
LIFTABLE_JOINTS = (
    'shoulder_pan_joint', 'shoulder_lift_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
)


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description')
    )


def test_most_arm_joints_permit_a_full_turn_either_way(validator):
    """The lift is legal, which is what makes wrapping a real loss.

    If every joint were bounded to [-pi, pi] a wrapped solution would be the
    only representation and there would be nothing to recover.
    """
    lower, upper = validator.robot.qlim
    for index, name in enumerate(JOINT_NAMES):
        if name in LIFTABLE_JOINTS:
            assert upper[index] - lower[index] == pytest.approx(2 * TWO_PI, abs=1e-6)


def test_the_elbow_cannot_be_lifted(validator):
    """The elbow spans only 2*pi, so q + 2*pi leaves its limits.

    Lift expansion must be per joint, not applied uniformly.
    """
    index = JOINT_NAMES.index('elbow_joint')
    lower, upper = validator.robot.qlim
    assert upper[index] - lower[index] == pytest.approx(TWO_PI, abs=1e-6)


def test_solutions_come_back_wrapped_into_a_single_turn(validator):
    """The source of the discontinuity.

    Every returned arm value sits in [-pi, pi], so the solver never expresses
    a configuration a full turn away even where the joint allows it.
    """
    rng = np.random.default_rng(0)
    lower, upper = validator.robot.qlim
    seen_any = False
    for _ in range(25):
        q_target = lower + rng.random(7) * (upper - lower)
        pose = validator.robot.fkine(q_target, end='tool0')
        quaternion = np.roll(np.array(pose.UnitQuaternion().A), -1)
        _, q_arm, solution = validator.solve_ik_lm(
            pose.t, quaternion, np.zeros(6), rail_seed=1.5)
        if not solution.success:
            continue
        seen_any = True
        assert np.all(np.abs(q_arm) <= np.pi + 1e-6)
    assert seen_any, 'no solve converged, so nothing was checked'


def test_a_near_two_pi_delta_is_a_wrap_not_a_velocity():
    """The audit's classification rule, as a unit.

    A delta within a small band of 2*pi is a representation discontinuity. The
    speed alone cannot distinguish the two, which is why the raw signed delta
    is recorded alongside it.
    """
    def is_wrap(delta, band=0.35):
        return abs(abs(delta) - TWO_PI) < band

    assert is_wrap(-6.2744)      # the measured wrist_1 failures
    assert is_wrap(TWO_PI)
    assert not is_wrap(0.25)     # a genuine over-speed at dt = 0.1 s
    assert not is_wrap(np.pi)


def test_unwrapping_a_delta_recovers_a_small_motion():
    """What the planner must do: choose the lift, not the wrapped value."""
    wrapped = -6.2744
    unwrapped = wrapped + TWO_PI
    assert abs(unwrapped) < 0.02
    assert abs(unwrapped) / 0.1 < 2.0, 'unwrapped motion is within the limit'
