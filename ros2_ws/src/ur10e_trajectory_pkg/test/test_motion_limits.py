#!/usr/bin/env python3
"""Limits carry their provenance, and provisional cannot certify.

The repository does not contain enough physical data to certify higher-order
motion limits. Recording that alongside each number is what stops an assumed
value becoming an accepted one, which is how the legacy start posture
survived as a de facto start pose for as long as it did.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg import motion_limits as ml
from ur10e_trajectory_pkg.configurations import JOINT_NAMES, NUM_JOINTS
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description')
    )


def test_every_limit_names_its_source_and_status():
    manifest = ml.manifest()
    for name in ('retired_uniform_cap', 'arm_acceleration', 'arm_jerk',
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


def test_the_urdf_agrees_with_the_published_limits(validator):
    """The URDF is the source; this table only cross-checks it.

    Our URDF carries Universal Robots\' values unmodified, so a mismatch means
    either the URDF was edited or the reference is stale.
    """
    from_urdf = ml.velocity_vector(validator)
    for index, name in enumerate(JOINT_NAMES[1:], start=1):
        assert from_urdf[index] == pytest.approx(ml.ARM_VELOCITY[name].value,
                                                 rel=1e-9)


def test_the_retired_cap_was_below_the_wrist_rating():
    """It held the wrists to roughly two thirds of their rated speed, and the
    wrists are what performs a tumble."""
    assert ml.RETIRED_UNIFORM_CAP.status == ml.ASSUMED
    assert ml.RETIRED_UNIFORM_CAP.value < ml.ARM_VELOCITY['wrist_1_joint'].value
    ratio = ml.RETIRED_UNIFORM_CAP.value / ml.ARM_VELOCITY['wrist_1_joint'].value
    assert 0.6 < ratio < 0.7


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


def test_limit_vectors_follow_the_shared_joint_ordering(validator):
    for vector in (ml.velocity_vector(validator), ml.acceleration_vector(),
                   ml.jerk_vector()):
        assert vector.shape == (NUM_JOINTS,)


def test_the_rail_keeps_its_deliberate_safety_cap(validator):
    """The arm takes the manufacturer\'s values; the rail does not.

    We have no trustworthy data for the rail drive, so its URDF figure is
    clamped on purpose rather than trusted.
    """
    from ur10e_trajectory_pkg.validation_core import RAIL_VEL_SAFETY_CAP
    assert ml.velocity_vector(validator)[0] <= RAIL_VEL_SAFETY_CAP


