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
    for name in ('arm_acceleration', 'arm_jerk',
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
    """By NAME, not just shape. A URDF that swapped two arm joints would
    still produce seven values, each applied to the wrong joint."""
    assert validator.joint_names == tuple(JOINT_NAMES)
    for vector in (ml.velocity_vector(validator), ml.acceleration_vector(),
                   ml.jerk_vector()):
        assert vector.shape == (NUM_JOINTS,)


def test_the_rail_keeps_its_deliberate_safety_cap(validator):
    """The arm takes the manufacturer\'s values; the rail does not.

    We have no trustworthy data for the rail drive, so its URDF figure is
    clamped on purpose rather than trusted.
    """
    urdf_rail = validator.urdf_velocity_limits[0]
    assert urdf_rail is not None
    assert ml.RAIL_VEL_SAFETY_CAP == pytest.approx(0.5)
    assert ml.velocity_vector(validator)[0] == pytest.approx(
        min(urdf_rail, ml.RAIL_VEL_SAFETY_CAP))


def test_the_rail_cap_is_defined_once():
    """The validator's cap and the Limit describing it cannot disagree."""
    from ur10e_trajectory_pkg import validation_core
    assert validation_core.RAIL_VEL_SAFETY_CAP is ml.RAIL_VEL_SAFETY_CAP
    assert ml.RAIL_VELOCITY.value == ml.RAIL_VEL_SAFETY_CAP


def test_certification_reports_the_worst_status_not_the_best():
    """min() over a boolean key returned certified if ANY joint was."""
    assert ml.worst_status([ml.CERTIFIED, ml.ASSUMED, ml.PROVISIONAL]) == ml.ASSUMED
    assert ml.worst_status([ml.PROVISIONAL, ml.CERTIFIED]) == ml.PROVISIONAL
    assert ml.worst_status([ml.CERTIFIED, ml.CERTIFIED]) == ml.CERTIFIED


def test_arm_velocity_is_unverified_until_its_source_is_checked(validator):
    """Without a validator nothing was read, so the reference table's status
    would describe a number the code does not use."""
    assert ml.certification_status()['limits']['arm_velocity'] == ml.UNVERIFIED
    assert (ml.certification_status(validator)['limits']['arm_velocity']
            == ml.CERTIFIED)


def test_an_edited_urdf_loses_arm_velocity_certification(validator):
    from types import SimpleNamespace

    edited = np.array(validator.velocity_limits, dtype=float)
    edited[5] *= 0.9
    fake = SimpleNamespace(velocity_limits=edited,
                           urdf_velocity_limits=edited.tolist(),
                           joint_names=validator.joint_names)
    assert ml.urdf_agrees_with_published(fake)['wrist_2_joint'] is False
    assert ml.certification_status(fake)['limits']['arm_velocity'] == ml.ASSUMED
    entry = ml.effective_limits(fake)['velocity'][5]
    assert entry['agrees_with_published'] is False
    assert entry['status'] == ml.ASSUMED


def test_effective_limits_record_what_was_enforced(validator):
    """An artifact must record the values the code USED, beside the raw URDF
    values and the cap, not the reference table."""
    import json

    document = ml.effective_limits(validator)
    np.testing.assert_array_equal(document['velocity_vector'],
                                  validator.velocity_limits)
    assert document['joint_names'] == list(JOINT_NAMES)
    rail = document['velocity'][0]
    assert rail['effective'] == pytest.approx(min(rail['urdf'], rail['safety_cap']))
    for entry in document['velocity'][1:]:
        assert entry['effective'] == pytest.approx(entry['urdf'])
        assert entry['agrees_with_published'] is True
    np.testing.assert_array_equal(document['acceleration_vector'],
                                  ml.acceleration_vector())

    assert ml.manifest()['effective'] is None
    with_validator = ml.manifest(validator)
    assert with_validator['effective']['velocity_vector'] == document['velocity_vector']
    json.dumps(with_validator)


def test_limit_statuses_are_in_joint_order(validator):
    statuses = ml.limit_statuses(validator)
    assert len(statuses['velocity']) == len(statuses['acceleration']) == NUM_JOINTS
    assert statuses['velocity'][0] == ml.RAIL_VELOCITY.status
    assert statuses['acceleration'][0] == ml.RAIL_ACCELERATION.status
    assert statuses['acceleration'][1] == ml.ARM_ACCELERATION.status
