#!/usr/bin/env python3
"""Geometry and kinematics invariants for the rail + UR10e rig.

These pin down the properties that are easy to break silently: the rail
being a real 7th DOF, the environment actually covering the rail's travel,
and the RViz markers agreeing with the bodies the collision checker uses.

Each test here corresponds to something that has actually gone wrong or was
one edit away from going wrong, not to a hypothetical. In particular
test_environment_spans_full_rail_travel guards a bug that silently disabled
floor and wall collision checking over the outer half of the rail.

Run with:  colcon test --packages-select ur10e_trajectory_pkg
"""
import os
from pathlib import Path

import numpy as np
import pybullet as pb
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

# The rail's travel, as declared by linear_rail_joint's <limit> in the URDF.
# Several tests below assert that other geometry is consistent with this, so
# it is the one place the range is written down.
RAIL_LOWER_M = 0.0
RAIL_UPPER_M = 3.0

# Tolerance for float comparisons of positions/extents, in metres. Generous
# relative to the quantities involved (centimetres and up) but far tighter
# than any misalignment worth catching.
TOL_M = 1e-6

# Forward-kinematics agreement tolerance, looser than TOL_M because it comes
# out of an IK/FK chain rather than a literal.
FK_TOL_M = 1e-4


def _urdf_path():
    """Locate ur10e.urdf without depending on the container's absolute paths.

    UR10E_URDF wins if set; otherwise walk up from this file to the workspace
    root, which is where the URDF lives.
    """
    override = os.environ.get('UR10E_URDF')
    if override:
        return override
    # .../ros2_ws/src/ur10e_trajectory_pkg/test/this_file.py -> .../ros2_ws
    return str(Path(__file__).resolve().parents[3] / 'ur10e.urdf')


def _q(rail_m, *arm_deg):
    """Build a q_full vector. The rail is METRES, the six arm joints DEGREES.

    Mixing those units up is easy and silent -- deg2rad over the whole vector
    turns a 1.5 m rail command into 0.026 m -- so every pose in this file is
    built through here rather than written out inline.
    """
    assert len(arm_deg) == 6, 'expected 6 arm joint angles'
    return np.concatenate(([rail_m], np.deg2rad(arm_deg)))


# Home configuration seeded by Validate_trajServer when no hardware feedback
# exists. Kept in sync with that node's current_joint_positions.
HOME_ARM_DEG = (0.0, -135.0, 90.0, -90.0, 0.0, 0.0)


@pytest.fixture(scope='module')
def validator():
    """One validator for the whole module.

    Construction loads the URDF into both roboticstoolbox and pybullet and
    costs a second or so, which is worth paying once rather than per test.
    """
    return TrajectoryValidator(
        _urdf_path(),
        mesh_base_path=get_package_share_directory('ur_description'),
        framerate=30,
    )


def _env_body_extents(validator, body_id):
    """Return (centre, full_extents) in world coordinates for an env body.

    getCollisionShapeData reports FULL extents for GEOM_BOX, i.e. the same
    convention Marker.scale uses, so the two are directly comparable.
    """
    centre, _ = pb.getBasePositionAndOrientation(
        body_id, physicsClientId=validator._pb_client
    )
    shape_data = pb.getCollisionShapeData(
        body_id, -1, physicsClientId=validator._pb_client
    )
    return np.asarray(centre), np.asarray(shape_data[0][3])


# --------------------------------------------------------------------------
# The rail as a genuine 7th DOF
# --------------------------------------------------------------------------

def test_rail_is_the_first_actuated_joint(validator):
    """q_full[0] must be the rail.

    Everything downstream indexes it positionally -- rail velocity checks,
    the IK seed, the joint names Validate_trajServer publishes -- so a
    reordering here would mis-drive the robot rather than raise.
    """
    assert validator._q_link_names[0] == 'rail_carriage_link'


def test_seven_actuated_degrees_of_freedom(validator):
    """Six arm joints plus the rail.

    A URDF edit that drops the rail joint, which has happened twice in this
    branch's history, shows up here.
    """
    assert len(validator._q_link_names) == 7


def test_rail_limits_match_the_declared_travel(validator):
    lower, upper = validator._rail_limits
    assert lower == pytest.approx(RAIL_LOWER_M, abs=TOL_M)
    assert upper == pytest.approx(RAIL_UPPER_M, abs=TOL_M)


def test_rail_zero_is_the_home_end_and_base_tracks_the_rail(validator):
    """The arm base must sit at exactly the rail's value along world X.

    This encodes the convention that rail 0 is the origin end rather than
    the rail's midpoint. Centring the rail's visual box would break it.
    """
    for rail_m in (RAIL_LOWER_M, 1.5, RAIL_UPPER_M):
        base = validator.robot.fkine(
            _q(rail_m, *HOME_ARM_DEG), end='base_link'
        ).t
        assert base[0] == pytest.approx(rail_m, abs=FK_TOL_M)
        assert base[1] == pytest.approx(0.0, abs=FK_TOL_M)


# --------------------------------------------------------------------------
# The environment must actually cover where the robot can go
# --------------------------------------------------------------------------

@pytest.mark.parametrize('body_name', ['floor_id', 'wall_id'])
def test_environment_spans_full_rail_travel(validator, body_name):
    """Regression test.

    The floor and wall were once centred on the origin, spanning X = -1.5 to
    +1.5, while the rail spans 0 to 3. Past the rail's midpoint the arm was
    outside both bodies entirely, so no environment collision could ever be
    reported there and unsafe trajectories validated clean. Nothing raised;
    the checker simply had nothing to hit.
    """
    centre, extents = _env_body_extents(validator, getattr(validator, body_name))
    x_min = centre[0] - extents[0] / 2.0
    x_max = centre[0] + extents[0] / 2.0
    assert x_min <= RAIL_LOWER_M + TOL_M, (
        f'{body_name} starts at X={x_min:.3f}, leaving rail travel from '
        f'{RAIL_LOWER_M} uncovered'
    )
    assert x_max >= RAIL_UPPER_M - TOL_M, (
        f'{body_name} ends at X={x_max:.3f}, leaving rail travel out to '
        f'{RAIL_UPPER_M} uncovered'
    )


def test_wall_and_floor_are_distinguished_by_orientation(validator):
    """The names must follow the geometry.

    These two were swapped once already, which made the horizontal floor
    render as a wall and every collision message name the wrong surface.
    A wall is thin in Y; a floor is thin in Z.
    """
    _, wall_extents = _env_body_extents(validator, validator.wall_id)
    _, floor_extents = _env_body_extents(validator, validator.floor_id)
    assert np.argmin(wall_extents) == 1, 'wall_id is not thin in Y'
    assert np.argmin(floor_extents) == 2, 'floor_id is not thin in Z'


# --------------------------------------------------------------------------
# RViz markers vs the bodies actually used for collision
# --------------------------------------------------------------------------

def test_markers_match_the_collision_bodies(validator):
    """obstacle_markers and validation_core hardcode the same geometry twice.

    Nothing links the two, so editing one alone makes RViz show a scene the
    validator is not checking. Compare them as unordered sets of
    (centre, full extents) so this survives renaming or reordering and fails
    only on genuine divergence.
    """
    rclpy = pytest.importorskip('rclpy')
    from ur10e_trajectory_pkg.obstacle_markers import ObstacleMarkerPublisher

    rclpy.init()
    try:
        node = ObstacleMarkerPublisher()
        captured = []
        node.pub.publish = captured.append  # intercept rather than publish
        node.publish_markers()
        node.destroy_node()
    finally:
        rclpy.shutdown()

    assert captured, 'publish_markers() published nothing'
    marker_boxes = {
        (
            round(m.pose.position.x, 6),
            round(m.pose.position.y, 6),
            round(m.pose.position.z, 6),
            round(m.scale.x, 6),
            round(m.scale.y, 6),
            round(m.scale.z, 6),
        )
        for m in captured[0].markers
    }

    collision_boxes = set()
    for body_id in (validator.wall_id, validator.floor_id):
        centre, extents = _env_body_extents(validator, body_id)
        collision_boxes.add(
            tuple(round(float(v), 6) for v in (*centre, *extents))
        )

    assert marker_boxes == collision_boxes, (
        'RViz markers and collision bodies disagree.\n'
        f'  markers only:   {sorted(marker_boxes - collision_boxes)}\n'
        f'  collision only: {sorted(collision_boxes - marker_boxes)}'
    )


def test_markers_are_published_in_the_world_frame(validator):
    """Markers must be published in the URDF root frame.

    Their poses are absolute, so they only line up with the collision bodies
    if RViz interprets them in that frame.
    """
    rclpy = pytest.importorskip('rclpy')
    from ur10e_trajectory_pkg.obstacle_markers import ObstacleMarkerPublisher

    rclpy.init()
    try:
        node = ObstacleMarkerPublisher()
        captured = []
        node.pub.publish = captured.append
        node.publish_markers()
        node.destroy_node()
    finally:
        rclpy.shutdown()

    assert all(m.header.frame_id == 'world' for m in captured[0].markers)


# --------------------------------------------------------------------------
# The collision checker itself
# --------------------------------------------------------------------------

@pytest.mark.parametrize('rail_m', [RAIL_LOWER_M, 1.5, RAIL_UPPER_M])
def test_home_pose_is_collision_free_across_the_rail(validator, rail_m):
    """Home must be usable anywhere on the rail.

    This also catches an environment body accidentally intersecting the rig,
    which would make every trajectory fail validation from the first waypoint.
    """
    assert validator.check_all_collisions(_q(rail_m, *HOME_ARM_DEG)) is False


def test_self_collision_is_detected(validator):
    """Folding the upper arm into the base must register."""
    assert validator.check_all_collisions(_q(1.5, 0, 80, 0, 0, 0, 0)) is True


@pytest.mark.parametrize('rail_m', [RAIL_LOWER_M, 1.5])
def test_floor_collision_is_detected_within_the_covered_region(validator, rail_m):
    """Dipping the wrist through the floor must register.

    Limited to rail positions whose reachable volume currently sits over the
    floor. See test_environment_covers_the_reachable_workspace for why that
    is not the whole rail.
    """
    assert validator.check_all_collisions(_q(rail_m, 0, -50, 120, -90, 0, 0)) is True


# Furthest the tool reaches from its own base along X, measured by sweeping
# shoulder_lift and elbow through their ranges at rail 0 (tool0 X spans
# -1.304 to +1.304). The arm therefore overhangs each end of the rail by
# roughly this much.
MAX_ARM_REACH_M = 1.30


@pytest.mark.parametrize('body_name', ['floor_id', 'wall_id'])
def test_environment_covers_the_reachable_workspace(validator, body_name):
    """The environment must cover everywhere the arm can actually go.

    Covering only the rail's travel is not enough: the arm overhangs both
    ends by its own reach. In those two strips the collision checker has no
    floor to hit, so a trajectory driving the tool through floor level
    validates clean.
    """
    centre, extents = _env_body_extents(validator, getattr(validator, body_name))
    x_min = centre[0] - extents[0] / 2.0
    x_max = centre[0] + extents[0] / 2.0
    assert x_min <= RAIL_LOWER_M - MAX_ARM_REACH_M + TOL_M
    assert x_max >= RAIL_UPPER_M + MAX_ARM_REACH_M - TOL_M


def test_floor_collision_is_detected_past_the_rail_end(validator):
    """The gap the widened environment closes.

    While the planes spanned only the rail's 0 to 3 m travel, this pose
    reached past the end with no floor beneath it and validated clean. The
    arm overhangs each rail end by its own 1.3 m reach, so that region is
    reachable and had to be modelled before collision clearance could mean
    anything there.
    """
    q_dipping = _q(RAIL_UPPER_M, 0, -50, 120, -90, 0, 0)
    wrist_x = validator.robot.fkine(q_dipping, end='wrist_3_link').t[0]
    assert wrist_x > RAIL_UPPER_M, 'pose no longer overhangs the rail end'
    assert validator.check_all_collisions(q_dipping) is True
