#!/usr/bin/env python3
"""Limits carry their provenance, and provisional cannot certify.

The repository does not contain enough physical data to certify higher-order
motion limits. Recording that alongside each number is what stops an assumed
value becoming an accepted one, which is how the legacy start posture
survived as a de facto start pose for as long as it did.
"""
import numpy as np
import pytest

from ur10e_trajectory_pkg import motion_limits as ml
from ur10e_trajectory_pkg.configurations import JOINT_NAMES, NUM_JOINTS


def test_every_limit_names_its_source_and_status():
    manifest = ml.manifest()
    for name in ('applied_arm_velocity', 'arm_acceleration', 'arm_jerk',
                 'rail_velocity', 'rail_acceleration', 'rail_jerk'):
        entry = manifest[name]
        assert entry['source'], f'{name} has no provenance'
        assert entry['status'] in (ml.CERTIFIED, ml.PROVISIONAL, ml.ASSUMED)
        assert entry['units']


def test_published_arm_velocity_limits_are_certified():
    """The one group Universal Robots actually publishes."""
    for name in JOINT_NAMES[1:]:
        assert ml.ARM_VELOCITY[name].status == ml.CERTIFIED
    assert ml.ARM_VELOCITY['shoulder_pan_joint'].value == pytest.approx(
        np.deg2rad(120.0))
    assert ml.ARM_VELOCITY['wrist_3_joint'].value == pytest.approx(
        np.deg2rad(180.0))


def test_the_applied_cap_is_conservative_against_every_published_limit():
    """Nothing measured against 2 rad/s was ever optimistic."""
    for name in JOINT_NAMES[1:]:
        assert ml.APPLIED_ARM_VELOCITY.value <= ml.ARM_VELOCITY[name].value


def test_acceleration_is_provisional_and_jerk_is_merely_assumed():
    """UR states acceleration limits are not public and offers 800 deg/s^2 as
    a deployment recommendation. No jerk limit is published at all."""
    assert ml.ARM_ACCELERATION.status == ml.PROVISIONAL
    assert ml.ARM_JERK.status == ml.ASSUMED
    assert 'recommendation' in ml.ARM_ACCELERATION.source


def test_nothing_about_the_rail_is_certified():
    """Its URDF value carries a TODO and the README calls the figures
    uncertain."""
    for limit in (ml.RAIL_VELOCITY, ml.RAIL_ACCELERATION, ml.RAIL_JERK):
        assert limit.status != ml.CERTIFIED


def test_hardware_certification_is_blocked_and_says_by_what():
    """A selection made under provisional limits is provisional, and this is
    what a caller checks rather than remembering."""
    assert ml.may_certify_for_hardware() is False
    blocking = ml.certification_status()['blocking']
    assert 'arm_jerk' in blocking
    assert 'rail_acceleration' in blocking


def test_limit_vectors_follow_the_shared_joint_ordering():
    for vector in (ml.velocity_vector(), ml.acceleration_vector(),
                   ml.jerk_vector()):
        assert vector.shape == (NUM_JOINTS,)


def test_the_uniform_cap_is_the_default_so_recorded_results_stay_comparable():
    """Switching to published per-joint limits is correct for hardware and
    changes every result measured on this branch, so it is opt-in."""
    uniform = ml.velocity_vector(uniform_cap=True)
    published = ml.velocity_vector(uniform_cap=False)
    np.testing.assert_allclose(uniform[1:], ml.APPLIED_ARM_VELOCITY.value)
    assert np.all(published[1:] >= uniform[1:])
    assert not np.allclose(published[1:], uniform[1:])
