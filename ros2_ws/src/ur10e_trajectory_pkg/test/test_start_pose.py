#!/usr/bin/env python3
"""Tests for how a validation request's start pose is resolved.

The server used to take its start pose from the /joint_states topic it
publishes to itself, so every request after the first began wherever the
previous playback stopped. The same trajectory then validated differently
depending on what had run before: 359 of 500 waypoints from cold, 500 of 500
immediately after another run. These tests pin the replacement behaviour,
where the start pose is an explicit input and nothing is inherited.
"""
import numpy as np
import pytest

from ur10e_trajectory_pkg.Validate_trajServer import (
    NUM_JOINTS,
    resolve_start_pose,
)
from ur10e_trajectory_pkg.configurations import (
    JOINT_NAMES,
    LEGACY_MATLAB_START_Q,
)


def test_omitted_field_is_rejected():
    """No default start pose.

    It used to fall back to the legacy MATLAB posture, so a caller that simply
    forgot got a confident answer computed from a configuration the robot was
    not in. That posture is singular as well, so the default also seeded the
    solver on a degeneracy.
    """
    with pytest.raises(ValueError, match='required'):
        resolve_start_pose([])


def test_the_legacy_posture_is_still_accepted_explicitly():
    """Simulation and the 370/500 regression still need it, stated outright."""
    q_start, description = resolve_start_pose(LEGACY_MATLAB_START_Q.tolist())
    np.testing.assert_allclose(q_start, LEGACY_MATLAB_START_Q)
    assert 'client' in description.lower()


def test_joint_names_and_length_agree_across_the_package():
    """One definition of the ordering, not one per module."""
    assert len(JOINT_NAMES) == NUM_JOINTS
    assert JOINT_NAMES[0] == 'linear_rail_joint'
    assert LEGACY_MATLAB_START_Q.shape == (NUM_JOINTS,)


def test_supplied_pose_is_used_verbatim():
    """A client-supplied pose must be passed through unaltered.

    Rounding or reordering it here would change which solution branch the
    inverse kinematics lands in, which is the whole thing being controlled.
    """
    requested = [1.25, 0.1, -2.0, 1.4, -0.5, 0.25, 3.0]
    q_start, description = resolve_start_pose(requested)
    np.testing.assert_allclose(q_start, requested)
    assert 'client' in description.lower()


@pytest.mark.parametrize('bad_length', [1, 6, 8, 14])
def test_wrong_length_is_rejected(bad_length):
    """Reject rather than pad or truncate.

    A six-element pose is the plausible mistake, being the arm without the
    rail. Silently accepting it would shift every joint by one position and
    validate a completely different trajectory.
    """
    with pytest.raises(ValueError, match=str(NUM_JOINTS)):
        resolve_start_pose([0.0] * bad_length)


def test_result_does_not_alias_the_home_constant():
    """A returned pose must not alias the shared legacy constant."""
    before = LEGACY_MATLAB_START_Q.copy()
    q_start, _ = resolve_start_pose(LEGACY_MATLAB_START_Q.tolist())
    q_start[0] = 99.0
    np.testing.assert_allclose(LEGACY_MATLAB_START_Q, before)


def test_resolution_depends_only_on_its_argument():
    """Same input, same output, no matter what ran before.

    This is the property the old topic-driven version lacked.
    """
    first, _ = resolve_start_pose([0.4, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0])
    resolve_start_pose([2.9, 1.0, -2.0, 0.5, 0.1, 0.2, 0.3])
    second, _ = resolve_start_pose([0.4, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(first, second)


def test_legacy_posture_has_one_entry_per_actuated_joint():
    assert LEGACY_MATLAB_START_Q.shape == (NUM_JOINTS,)
    assert LEGACY_MATLAB_START_Q[0] == 0.0, 'rail starts at its zero end'
