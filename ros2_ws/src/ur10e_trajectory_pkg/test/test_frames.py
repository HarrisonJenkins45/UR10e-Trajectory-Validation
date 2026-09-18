#!/usr/bin/env python3
"""Stage 2 gate: the frame contract holds, end to end.

Four frames and only four: the arena I, the FIXED rail base R where every
target lives, the carriage C which moves with the rail, and the end effector
G. Targets convert once, T_RG = inv(T_IR) @ T_IG, and the rail coordinate
appears only inside forward kinematics, T_RG(q) = T_RC(q_rail) @ T_CG(q_arm).

The failure this prevents is silent. A target expressed against the moving
carriage frame disagrees with the solver by the rail position plus the 25 mm
mount height, and an array of numbers with no frame attached gives nobody a
way to notice.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

from ur10e_trajectory_pkg import frames
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

TOL = 1e-12
FK_TOL = 1e-9

# Carriage mount height from the URDF's linear_rail_joint origin. Internal to
# forward kinematics; a client target must never carry it.
CARRIAGE_MOUNT_Z_M = 0.025


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description')
    )


def _pose(roll_deg, pitch_deg, yaw_deg, translation):
    rotation = Rotation.from_euler(
        'xyz', [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix()
    return frames.make_transform(rotation=rotation, translation=translation)


def _angle_between(rotation_a, rotation_b):
    return Rotation.from_matrix(np.asarray(rotation_a).T @ np.asarray(rotation_b)
                                ).magnitude()


# --------------------------------------------------------------------------
# Transform algebra
# --------------------------------------------------------------------------

def test_identity_calibration_leaves_poses_unchanged():
    poses = np.stack([_pose(10, 20, 30, [1.0, 2.0, 3.0]),
                      _pose(-5, 45, 90, [0.1, -0.2, 0.3])])
    np.testing.assert_allclose(
        frames.arena_to_rail_base(poses, np.eye(4)), poses, atol=TOL)


def test_translation_and_quarter_turn_give_the_hand_calculated_result():
    """A 90 degree yaw about Z with a known offset, worked out by hand.

    The arena point (2, 0, 0) with the rail base at (1, 0, 0) rotated +90
    degrees about Z sits 1 m in front of the rail base along arena X. Yawing
    the frame by +90 maps that onto the rail base's -Y axis.
    """
    calibration = _pose(0, 0, 90, [1.0, 0.0, 0.0])
    target = frames.make_transform(translation=[2.0, 0.0, 0.0])
    result = frames.arena_to_rail_base(np.stack([target]), calibration)[0]
    np.testing.assert_allclose(result[:3, 3], [0.0, -1.0, 0.0], atol=1e-12)


def test_transform_and_inverse_round_trip():
    calibration = _pose(15, -35, 120, [0.4, -1.2, 2.5])
    poses = np.stack([_pose(5, 10, 15, [1.0, 1.0, 1.0]),
                      _pose(-60, 0, 200, [-0.5, 0.25, 3.0])])
    in_rail = frames.arena_to_rail_base(poses, calibration)
    recovered = np.stack([calibration @ pose for pose in in_rail])
    np.testing.assert_allclose(recovered, poses, atol=1e-12)


def test_invert_is_exact_for_rigid_transforms():
    """Uses the rigid structure rather than a general matrix inverse, so the
    rotation block stays exactly orthonormal."""
    transform = _pose(33, -77, 155, [3.0, -2.0, 1.0])
    product = frames.invert(transform) @ transform
    np.testing.assert_allclose(product, np.eye(4), atol=1e-14)


def test_quaternion_sign_reversal_gives_the_same_target():
    """q and -q are the same rotation.

    The input trajectory contains seven such reversals, so this is a real
    case: a componentwise treatment would produce a large spurious error at
    each one.
    """
    quaternion = Rotation.from_euler('xyz', [20, 40, 60], degrees=True).as_quat()
    positive = frames.poses_from_positions_quaternions([[1.0, 2.0, 3.0]],
                                                       [quaternion])
    negative = frames.poses_from_positions_quaternions([[1.0, 2.0, 3.0]],
                                                       [-quaternion])
    np.testing.assert_allclose(positive, negative, atol=1e-14)


def test_calibration_must_be_a_single_static_transform():
    """An N x 4 array must be refused, not quietly reduced to row zero.

    The previous conversion accepted a per-waypoint quaternion array and used
    only its first row, so a caller could pass a time-varying calibration and
    see no sign that the rest was discarded. The rail base does not move.
    """
    poses = np.stack([np.eye(4)])
    with pytest.raises(ValueError, match='static'):
        frames.arena_to_rail_base(poses, np.tile(np.eye(4), (5, 1, 1)))


# --------------------------------------------------------------------------
# Relative motion and placement
# --------------------------------------------------------------------------

def test_relative_motion_starts_at_identity_and_ignores_placement():
    poses = np.stack([_pose(0, 0, 0, [5.0, 5.0, 5.0]),
                      _pose(0, 0, 30, [5.1, 5.0, 5.0]),
                      _pose(0, 0, 60, [5.2, 5.0, 5.0])])
    motion = frames.relative_motion(poses)
    np.testing.assert_allclose(motion[0], np.eye(4), atol=1e-12)

    # Same motion somewhere else entirely gives the same relative sequence.
    shifted = np.stack([_pose(0, 0, 90, [-2.0, 7.0, 1.0]) @ pose for pose in poses])
    np.testing.assert_allclose(frames.relative_motion(shifted), motion, atol=1e-12)


def test_scaling_touches_translation_only():
    """Scaling rotation would reproduce a different tumble."""
    motion = np.stack([_pose(0, 0, 45, [0.2, 0.0, 0.0])])
    scaled = frames.scale_translation(motion, 0.5)
    np.testing.assert_allclose(scaled[0, :3, 3], [0.1, 0.0, 0.0], atol=TOL)
    np.testing.assert_allclose(scaled[0, :3, :3], motion[0, :3, :3], atol=TOL)


def test_a_pure_tumble_is_not_magnified():
    """Zero displacement must be left alone rather than scaled up.

    The current trajectory holds position constant, so the scale factor has
    to come out 1.0 rather than dividing by zero.
    """
    motion = np.stack([np.eye(4), _pose(0, 0, 90, [0.0, 0.0, 0.0])])
    scaled, factor = frames.scale_to_bound(motion, 1.0)
    assert factor == 1.0
    np.testing.assert_allclose(scaled, motion, atol=TOL)


def test_placement_sets_the_first_pose_exactly():
    placement = _pose(10, 20, 30, [1.0, 0.5, 0.5])
    motion = frames.relative_motion(
        np.stack([_pose(0, 0, 0, [0, 0, 0]), _pose(0, 0, 45, [0, 0, 0])]))
    placed, _ = frames.place_relative_motion(motion, placement)
    np.testing.assert_allclose(placed[0], placement, atol=1e-12)


# --------------------------------------------------------------------------
# URDF agreement
# --------------------------------------------------------------------------

def test_world_to_rail_base_is_identity(validator):
    """`world` anchors rail_base_link and is not the arena frame."""
    pose = validator.robot.fkine(np.zeros(7), end='rail_base_link')
    np.testing.assert_allclose(pose.A, np.eye(4), atol=FK_TOL)


@pytest.mark.parametrize('rail_m', [0.0, 1.5, 3.0])
def test_rail_base_to_carriage_is_rail_along_x_plus_mount_height(validator, rail_m):
    """The rail coordinate and the 25 mm mount live here, inside FK.

    A client target must carry neither.
    """
    q_full = np.concatenate(([rail_m], np.deg2rad([0.0, -135.0, 90.0, -90.0, 45.0, 0.0])))
    pose = validator.robot.fkine(q_full, end=frames.CARRIAGE_FRAME)
    np.testing.assert_allclose(
        pose.t, [rail_m, 0.0, CARRIAGE_MOUNT_Z_M], atol=FK_TOL)
    np.testing.assert_allclose(pose.R, np.eye(3), atol=FK_TOL)


def test_changing_the_rail_seed_does_not_move_client_targets():
    """Targets are in the fixed rail-base frame, so the rail cannot affect
    them. If it did, the client would be expressing targets against the
    moving carriage."""
    pytest.importorskip('pandas')
    from ur10e_trajectory_pkg.ClientNode import build_trajectory_targets
    try:
        first = build_trajectory_targets(num_waypoints=8)
    except FileNotFoundError:
        pytest.skip('packaged trajectory CSV not present')
    second = build_trajectory_targets(num_waypoints=8)
    for a, b in zip(first[:4], second[:4]):
        np.testing.assert_allclose(np.asarray(a), np.asarray(b), atol=TOL)


# --------------------------------------------------------------------------
# Golden end-to-end round trip
# --------------------------------------------------------------------------

def test_golden_round_trip_through_a_rotated_arena(validator):
    """Configuration -> FK -> arena -> client conversion -> back to T_RG.

    Catches a frame error anywhere in the chain, including one that happens
    to cancel under an identity calibration, which is why the arena pose here
    is both rotated and translated.
    """
    q_full = np.concatenate(([1.2], np.deg2rad([20.0, -110.0, 80.0, -70.0, 55.0, 30.0])))
    original_RG = validator.robot.fkine(q_full, end=frames.END_EFFECTOR_FRAME).A

    calibration_IR = _pose(12.0, -34.0, 56.0, [7.0, -3.0, 1.5])
    in_arena = calibration_IR @ original_RG

    recovered = frames.arena_to_rail_base(np.stack([in_arena]), calibration_IR)[0]

    np.testing.assert_allclose(recovered[:3, 3], original_RG[:3, 3], atol=1e-9)
    assert _angle_between(recovered[:3, :3], original_RG[:3, :3]) < 1e-9


def test_golden_round_trip_survives_relative_motion_and_placement(validator):
    """The placement formulation must not perturb the motion it places.

    Take poses in the rail frame, convert to relative motion, place them back
    at their own first pose, and require the originals back.
    """
    configurations = [
        np.concatenate(([1.0], np.deg2rad([0.0, -120.0, 90.0, -80.0, 45.0, 0.0]))),
        np.concatenate(([1.0], np.deg2rad([10.0, -115.0, 85.0, -75.0, 50.0, 15.0]))),
        np.concatenate(([1.0], np.deg2rad([20.0, -110.0, 80.0, -70.0, 55.0, 30.0]))),
    ]
    poses_RG = np.stack([
        validator.robot.fkine(q, end=frames.END_EFFECTOR_FRAME).A
        for q in configurations
    ])

    motion = frames.relative_motion(poses_RG)
    replaced, factor = frames.place_relative_motion(motion, poses_RG[0])

    assert factor == pytest.approx(1.0)
    np.testing.assert_allclose(replaced, poses_RG, atol=1e-9)


def test_target_frame_is_declared_and_validated():
    """A frame-free array of numbers gives nobody a way to notice a mismatch."""
    from ur10e_trajectory_pkg.Validate_trajServer import resolve_target_frame

    assert resolve_target_frame(frames.TARGET_FRAME) == frames.TARGET_FRAME
    with pytest.raises(ValueError, match='required'):
        resolve_target_frame('')
    with pytest.raises(ValueError, match=frames.TARGET_FRAME):
        resolve_target_frame(frames.CARRIAGE_FRAME)
