#!/usr/bin/env python3
"""Pose error between a commanded target and a reached configuration.

Two quantities, kept separate because they have different units and want
different tolerances: a translation norm in metres and an SO(3) angle in
radians.

Orientation error is the angle of the relative rotation, not a difference of
quaternion or Euler components. Those are not differences in any useful
sense: quaternions double-cover, so q and -q describe the same rotation while
subtracting componentwise gives a large spurious error, and Euler angles wrap
and gimbal. The input trajectory contains seven such sign reversals, so this
is a real case and not a theoretical one.

Nothing here gates anything yet. Stage 2b decides tolerances and turns these
into an acceptance check; until then they are recorded observationally, which
is why the census stores raw errors rather than pass/fail flags.
"""
import numpy as np

# Provisional limits, recorded alongside raw errors so a later change of mind
# does not require re-running anything. NOT yet enforced anywhere.
#
# Deliberately independent of ikine_LM's own tol: that measures convergence of
# the solver's local search, not distance to the target, so it cannot stand in
# for either of these. A reaching solve was measured at 1e-4 m, so the
# position limit sits an order of magnitude above normal numerical error.
PROVISIONAL_POSITION_TOL_M = 1e-3
PROVISIONAL_ORIENTATION_TOL_RAD = np.deg2rad(0.1)


def quaternion_to_matrix(quaternion):
    """Rotation matrix from an [x, y, z, w] quaternion.

    Sign-insensitive by construction: q and -q give the same matrix.
    """
    x, y, z, w = (float(v) for v in quaternion)
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError('quaternion has near-zero norm')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def orientation_error_rad(rotation_a, rotation_b):
    """Angle of the rotation taking A to B, in radians, always in [0, pi]."""
    relative = np.asarray(rotation_a).T @ np.asarray(rotation_b)
    cosine = (np.trace(relative) - 1.0) / 2.0
    return float(np.arccos(np.clip(cosine, -1.0, 1.0)))


def position_error_m(reached, target):
    """Euclidean distance between reached and commanded positions."""
    return float(np.linalg.norm(np.asarray(reached) - np.asarray(target)))


def pose_error(reached_pose, target_position, target_quaternion):
    """(position_error_m, orientation_error_rad) for one reached pose.

    reached_pose is anything exposing .t and .R, i.e. a spatialmath SE3.
    """
    return (
        position_error_m(reached_pose.t, target_position),
        orientation_error_rad(
            reached_pose.R, quaternion_to_matrix(target_quaternion)
        ),
    )


def within_provisional_tolerance(position_err_m, orientation_err_rad):
    """Both errors inside the provisional limits.

    Provided so the census can flag likely-viable candidates without those
    limits being load-bearing anywhere. Raw errors are stored regardless.
    """
    return (position_err_m <= PROVISIONAL_POSITION_TOL_M
            and orientation_err_rad <= PROVISIONAL_ORIENTATION_TOL_RAD)
