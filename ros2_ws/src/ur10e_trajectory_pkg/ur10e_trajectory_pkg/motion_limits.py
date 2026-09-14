#!/usr/bin/env python3
"""Motion limits, each carrying where it came from and whether it is certified.

Most of these are not certified, and the difference matters: a provisional
limit can support a provisional selection but cannot certify a ready pose or
clear a trajectory for hardware. Recording provenance alongside the number is
what keeps an assumed value from quietly becoming an accepted one, which is
how LEGACY_MATLAB_START_Q survived as long as it did.

What is actually known:

  velocity, arm     AUTHORITATIVE. Universal Robots publishes 120 deg/s for
                    shoulder pan and lift, 180 deg/s for the rest. The 2 rad/s
                    used so far is uniform and conservative against both.

  acceleration, arm PROVISIONAL. The pinned official description states
                    outright that acceleration limits are not publicly
                    available. UR recommends keeping MoveJ acceleration below
                    800 deg/s^2, about 14 rad/s^2, but presents it as a
                    deployment recommendation rather than a
                    configuration-independent hardware limit: real capability
                    depends on payload, centre of gravity, inertia, posture
                    and controller.

  jerk, arm         ASSUMED. UR exposes commanded acceleration for movej but
                    publishes no corresponding jerk limit. The value here is
                    a placeholder and must not be used to certify anything.

  rail, all         ASSUMED. The URDF's 5 m/s carries a TODO and the README
                    marks its acceleration figures as uncertain. Nothing about
                    the rail is trustworthy until the drive, gearing, carriage
                    load and vendor limits are obtained.

Certification needs a chosen UR execution controller and payload
configuration, and the rail's actual hardware data. Until then peak
acceleration and jerk are worth RECORDING and not worth gating on.
"""
import numpy as np

from ur10e_trajectory_pkg.configurations import JOINT_NAMES

CERTIFIED = 'certified'
PROVISIONAL = 'provisional'
ASSUMED = 'assumed'


class Limit:
    """One limit, its units, its source and its status."""

    def __init__(self, value, units, status, source):
        self.value = float(value)
        self.units = units
        self.status = status
        self.source = source

    def as_dict(self):
        return {'value': self.value, 'units': self.units,
                'status': self.status, 'source': self.source}

    def __repr__(self):
        return f'Limit({self.value} {self.units}, {self.status})'


_UR_PUBLISHED = 'Universal Robots published UR10e joint limits'
_UR_DEPLOYMENT = ('Universal Robots deployment guidance: keep MoveJ '
                  'acceleration below 800 deg/s^2; a recommendation, not a '
                  'configuration-independent hardware limit')
_NO_PUBLIC_JERK = ('no public UR jerk limit; movej exposes commanded '
                   'acceleration only')
_RAIL_UNKNOWN = ('rail drive, gearing and carriage load unknown; URDF value '
                 'carries a TODO and the README marks it uncertain')

# Published per-joint velocity limits. The uniform 2 rad/s used so far is
# below all of these, so nothing measured against it was ever optimistic.
ARM_VELOCITY = {
    'shoulder_pan_joint': Limit(np.deg2rad(120.0), 'rad/s', CERTIFIED, _UR_PUBLISHED),
    'shoulder_lift_joint': Limit(np.deg2rad(120.0), 'rad/s', CERTIFIED, _UR_PUBLISHED),
    'elbow_joint': Limit(np.deg2rad(180.0), 'rad/s', CERTIFIED, _UR_PUBLISHED),
    'wrist_1_joint': Limit(np.deg2rad(180.0), 'rad/s', CERTIFIED, _UR_PUBLISHED),
    'wrist_2_joint': Limit(np.deg2rad(180.0), 'rad/s', CERTIFIED, _UR_PUBLISHED),
    'wrist_3_joint': Limit(np.deg2rad(180.0), 'rad/s', CERTIFIED, _UR_PUBLISHED),
}

ARM_ACCELERATION = Limit(np.deg2rad(800.0), 'rad/s^2', PROVISIONAL, _UR_DEPLOYMENT)
ARM_JERK = Limit(500.0, 'rad/s^3', ASSUMED, _NO_PUBLIC_JERK)

RAIL_VELOCITY = Limit(1.0, 'm/s', PROVISIONAL,
                      'RAIL_VEL_SAFETY_CAP, a deliberate derate below the '
                      'URDF value rather than a measured capability')
RAIL_ACCELERATION = Limit(5.0, 'm/s^2', ASSUMED, _RAIL_UNKNOWN)
RAIL_JERK = Limit(100.0, 'm/s^3', ASSUMED, _RAIL_UNKNOWN)

# The uniform arm velocity cap the validator has used throughout. Kept as its
# own entry rather than silently replaced: every result on this branch was
# measured against it, and raising it to the published values would change
# them all.
APPLIED_ARM_VELOCITY = Limit(2.0, 'rad/s', PROVISIONAL,
                             'uniform application cap, conservative against '
                             'every published UR10e joint limit')


def velocity_vector(uniform_cap=True):
    """Velocity limits in JOINT_NAMES order.

    uniform_cap keeps the 2 rad/s the whole branch was measured against.
    Setting it False uses the published per-joint values, which is correct for
    hardware but changes every recorded result.
    """
    arm = ([APPLIED_ARM_VELOCITY.value] * 6 if uniform_cap
           else [ARM_VELOCITY[name].value for name in JOINT_NAMES[1:]])
    return np.array([RAIL_VELOCITY.value] + arm)


def acceleration_vector():
    return np.array([RAIL_ACCELERATION.value] + [ARM_ACCELERATION.value] * 6)


def jerk_vector():
    return np.array([RAIL_JERK.value] + [ARM_JERK.value] * 6)


def certification_status():
    """What may and may not be certified with what is currently known."""
    entries = {
        'arm_velocity': min((ARM_VELOCITY[n].status for n in JOINT_NAMES[1:]),
                            key=lambda s: (s != CERTIFIED)),
        'applied_arm_velocity': APPLIED_ARM_VELOCITY.status,
        'arm_acceleration': ARM_ACCELERATION.status,
        'arm_jerk': ARM_JERK.status,
        'rail_velocity': RAIL_VELOCITY.status,
        'rail_acceleration': RAIL_ACCELERATION.status,
        'rail_jerk': RAIL_JERK.status,
    }
    return {
        'limits': entries,
        'all_certified': all(status == CERTIFIED for status in entries.values()),
        'blocking': sorted(name for name, status in entries.items()
                           if status != CERTIFIED),
    }


def may_certify_for_hardware():
    """False while any limit is provisional or assumed.

    Deliberately a function rather than a comment. A selection made under
    provisional limits is provisional, and this is what a caller checks
    instead of remembering.
    """
    return certification_status()['all_certified']


def manifest():
    """Every limit with its provenance, for an artifact to record."""
    return {
        'arm_velocity': {name: ARM_VELOCITY[name].as_dict()
                         for name in JOINT_NAMES[1:]},
        'applied_arm_velocity': APPLIED_ARM_VELOCITY.as_dict(),
        'arm_acceleration': ARM_ACCELERATION.as_dict(),
        'arm_jerk': ARM_JERK.as_dict(),
        'rail_velocity': RAIL_VELOCITY.as_dict(),
        'rail_acceleration': RAIL_ACCELERATION.as_dict(),
        'rail_jerk': RAIL_JERK.as_dict(),
        'certification': certification_status(),
    }
