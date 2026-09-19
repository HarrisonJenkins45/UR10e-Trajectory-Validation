#!/usr/bin/env python3
"""Frame contract and rigid-transform operations for the orientation task.

Four frames, and only four:

  I   arena / inertial source frame, where the SISIFOS trajectory lives
  R   rail base, the URDF's rail_base_link. FIXED. Every IK target is
      expressed here
  C   carriage / UR base_link. MOVES with the rail, so it can never be the
      frame a target is expressed in
  G   end effector, tool0

The rail coordinate appears only inside forward kinematics:

    T_RG(q) = T_RC(q_rail) @ T_CG(q_arm)

The target builder holds position fixed and expresses every target in R. A
client must never subtract the rail position or express a target relative to
the moving arm base.

The URDF's `world` is an identity anchor for rail_base_link, not the arena.
The 25 mm carriage origin is internal to forward kinematics and must never be
added to a client target.

Orientation-only placement
--------------------------
The target builder preserves the recording's absolute q_I_G orientations and
sets one fixed target position. Its relative-pose formulation is:

    dT(t)   = inv(T_IG(0)) @ T_IG(t)      motion relative to its own start
    T_RG(t) = T_RG(0) @ dT(t)             placed at a chosen start pose

SISIFOS translation, scaling and arena calibration are outside this task.
"""
import numpy as np
from scipy.spatial.transform import Rotation

# Named so that code and tests can refer to the contract rather than restating
# link names. The service declares TARGET_FRAME as the frame its targets are
# in, so a caller cannot be left guessing from comments.
ARENA_FRAME = 'arena'
TARGET_FRAME = 'rail_base_link'
CARRIAGE_FRAME = 'base_link'
END_EFFECTOR_FRAME = 'tool0'


def make_transform(rotation=None, translation=None):
    """4x4 homogeneous transform from a rotation and a translation."""
    matrix = np.eye(4)
    if rotation is not None:
        rotation = np.asarray(rotation, dtype=float)
        if rotation.shape == (4,):
            rotation = Rotation.from_quat(rotation).as_matrix()
        matrix[:3, :3] = rotation
    if translation is not None:
        matrix[:3, 3] = np.asarray(translation, dtype=float)
    return matrix


def validate_transform(transform, name='transform'):
    """Reject anything that is not a finite member of SE(3).

    Shape alone is not enough. A calibration with a scaled, skewed or
    reflected rotation block still multiplies cleanly and produces targets
    that look plausible, so the error surfaces as a trajectory that misses
    rather than as an exception. Checks orthonormality, a determinant of +1
    which rules out reflections, and the homogeneous bottom row.
    """
    matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f'{name} must be 4x4, got shape {matrix.shape}')
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f'{name} contains non-finite values')

    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8):
        raise ValueError(f'{name} rotation block is not orthonormal')
    determinant = float(np.linalg.det(rotation))
    if not np.isclose(determinant, 1.0, atol=1e-8):
        raise ValueError(
            f'{name} rotation has determinant {determinant:.6f}, expected +1; '
            'a value near -1 is a reflection, not a rotation'
        )
    if not np.allclose(matrix[3, :], [0.0, 0.0, 0.0, 1.0], atol=1e-12):
        raise ValueError(f'{name} bottom row is not [0, 0, 0, 1]')
    return matrix


def invert(transform):
    """Inverse of a homogeneous transform, without a general matrix inverse.

    Uses the rigid-body structure, so the result stays exactly orthonormal
    rather than accumulating the drift a numerical inverse would introduce.
    """
    transform = np.asarray(transform, dtype=float)
    rotation = transform[:3, :3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ transform[:3, 3]
    return inverse


def compose(*transforms):
    """Left-to-right composition: compose(T_AB, T_BC) is T_AC."""
    result = np.eye(4)
    for transform in transforms:
        result = result @ np.asarray(transform, dtype=float)
    return result


def to_position_quaternion(transform):
    """Split a transform into (position, [x, y, z, w] quaternion)."""
    transform = np.asarray(transform, dtype=float)
    quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
    return transform[:3, 3].copy(), quaternion


def relative_motion(poses):
    """Motion of a pose sequence relative to its own first entry.

    dT(t) = inv(T(0)) @ T(t). Independent of where the sequence sits, which
    is what makes placement a separate, free choice.
    """
    poses = np.asarray(poses, dtype=float)
    first_inverse = invert(poses[0])
    return np.stack([first_inverse @ pose for pose in poses])


def place_relative_motion(motion, start_pose_RG):
    """Put a relative motion into the rail-base frame at a chosen start pose.

    T_RG(t) = T_RG(0) @ dT(t)

    The current target builder supplies a fixed position and the recording's
    first absolute orientation. No translation scale is applied.
    """
    motion = np.asarray(motion, dtype=float)
    return np.stack([start_pose_RG @ step for step in motion])


def poses_from_positions_quaternions(positions, quaternions):
    """Build transforms from parallel position and [x, y, z, w] arrays.

    Quaternion sign is irrelevant here by construction: q and -q give the same
    rotation matrix, so the seven sign reversals in the input trajectory
    cannot perturb a target.
    """
    positions = np.atleast_2d(np.asarray(positions, dtype=float))
    quaternions = np.atleast_2d(np.asarray(quaternions, dtype=float))
    if len(positions) != len(quaternions):
        raise ValueError(
            f'{len(positions)} positions but {len(quaternions)} quaternions'
        )
    matrices = Rotation.from_quat(quaternions).as_matrix()
    poses = np.tile(np.eye(4), (len(positions), 1, 1))
    poses[:, :3, :3] = matrices
    poses[:, :3, 3] = positions
    return poses
