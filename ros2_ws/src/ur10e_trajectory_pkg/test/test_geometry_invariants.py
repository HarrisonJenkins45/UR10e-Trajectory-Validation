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


# A legacy arm posture, used where only the base's position matters.
# Validate_trajServer no longer seeds from it: q_start is now a required
# request field. With the payload mounted it is NOT collision-free (the
# payload points into the floor); see EXAMPLE_HOME_ARM_DEG.
HOME_ARM_DEG = (0.0, -135.0, 90.0, -90.0, 0.0, 0.0)

# The configured home in examples/home_choice.json, the one plans start from.
# Wrist_3 at 90 turns the payload's wings across the rail; along it (wrist_3
# 0), homing toward rail 0 swings a wing into the switch-end wall.
EXAMPLE_HOME_ARM_DEG = (0.0, -75.0, 100.0, -115.0, 100.0, 90.0)


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

    The rail is bolted to a wall and the model's frame is the wall's: +y up,
    +z out of the wall. So the wall the rail is mounted on is thin in Z, and
    the floor below is thin in Y. An earlier model had these the other way
    round, reading the frame as a floor-mounted rail.
    """
    _, wall_extents = _env_body_extents(validator, validator.wall_id)
    _, floor_extents = _env_body_extents(validator, validator.floor_id)
    assert np.argmin(wall_extents) == 2, 'wall_id is not thin in Z'
    assert np.argmin(floor_extents) == 1, 'floor_id is not thin in Y'


# The rig's end walls stand 2 ft beyond each end of the rail.
END_WALL_GAP_M = 0.6096


@pytest.mark.parametrize('body_name, rail_end, inward', [
    ('switch_end_wall_id', RAIL_LOWER_M, +1.0),
    ('far_end_wall_id', RAIL_UPPER_M, -1.0),
])
def test_end_walls_stand_two_feet_beyond_the_rail_ends(validator, body_name,
                                                       rail_end, inward):
    """Each end wall is thin in X, its inner face exactly 2 ft past its rail end.

    The face, not the centre, is what sets clearance: a wall centred on the
    2 ft line would overstate the room by half its thickness.
    """
    centre, extents = _env_body_extents(validator, getattr(validator, body_name))
    assert np.argmin(extents) == 0, f'{body_name} is not thin in X'
    inner_face = centre[0] + inward * extents[0] / 2.0
    assert inner_face == pytest.approx(rail_end - inward * END_WALL_GAP_M, abs=TOL_M)


# The switch-end wall is 2 ft high, measured up from the floor.
SWITCH_END_WALL_HEIGHT_M = 0.6096


@pytest.mark.parametrize('body_name', ['switch_end_wall_id', 'far_end_wall_id'])
def test_end_walls_rise_from_the_floor_and_meet_the_mounting_wall(validator, body_name):
    """An end wall must stand on the floor and reach back to the mounting wall.

    It runs from the floor upward in Y, and from the mounting wall to the
    floor's outer edge in Z, so no reach past a rail end slips under or
    behind it.
    """
    centre, extents = _env_body_extents(validator, getattr(validator, body_name))
    floor_c, floor_e = _env_body_extents(validator, validator.floor_id)
    wall_c, wall_e = _env_body_extents(validator, validator.wall_id)
    assert centre[1] - extents[1] / 2.0 <= floor_c[1] + TOL_M
    assert centre[2] - extents[2] / 2.0 <= wall_c[2] + wall_e[2] / 2.0 + TOL_M
    assert centre[2] + extents[2] / 2.0 >= floor_c[2] + floor_e[2] / 2.0 - TOL_M


def test_end_wall_heights(validator):
    """The switch-end wall is 2 ft high; the far-end wall is full height.

    The switch-end height decides whether the arm can reach over that wall,
    so it is pinned exactly: its top is 0.390 m below the rail.
    """
    floor_c, _ = _env_body_extents(validator, validator.floor_id)
    wall_c, wall_e = _env_body_extents(validator, validator.wall_id)
    centre, extents = _env_body_extents(validator, validator.switch_end_wall_id)
    assert centre[1] - extents[1] / 2.0 == pytest.approx(floor_c[1], abs=TOL_M)
    assert centre[1] + extents[1] / 2.0 == pytest.approx(
        floor_c[1] + SWITCH_END_WALL_HEIGHT_M, abs=TOL_M)
    centre, extents = _env_body_extents(validator, validator.far_end_wall_id)
    assert centre[1] + extents[1] / 2.0 >= wall_c[1] + wall_e[1] / 2.0 - TOL_M


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
    for body_id, _ in validator.env_bodies:
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
    assert validator.check_all_collisions(_q(rail_m, *EXAMPLE_HOME_ARM_DEG)) is False


def test_the_example_home_is_the_configured_one():
    import json

    path = Path(_urdf_path()).parent / 'examples' / 'home_choice.json'
    configured = json.loads(path.read_text())['chosen']['configuration']
    np.testing.assert_allclose(configured[1:], np.deg2rad(EXAMPLE_HOME_ARM_DEG), atol=1e-12)


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


def test_wall_collision_is_detected_past_the_rail_end(validator):
    """The gap the widened environment closes.

    While the planes spanned only the rail's 0 to 3 m travel, this pose
    reached past the end with no floor beneath it and validated clean. The
    arm overhangs each rail end by its own 1.3 m reach, so that region is
    reachable and had to be modelled before collision clearance could mean
    anything there.
    """
    # The arm is mounted turned 180 deg, so at shoulder_pan 0 it reaches
    # toward -x: this pose dips into the mounting wall past the rail's 0 end.
    q_dipping = _q(RAIL_LOWER_M, 0, -50, 120, -90, 0, 0)
    wrist_x = validator.robot.fkine(q_dipping, end='wrist_3_link').t[0]
    assert wrist_x < RAIL_LOWER_M, 'pose no longer overhangs the rail end'
    assert validator.check_all_collisions(q_dipping) is True


# Arm stretched straight out along the rail: shoulder_lift and elbow 0. The
# arm is mounted turned 180 deg, so shoulder_pan 0 points it toward -x (the
# switch end) and 180 toward +x; pan turns it in the wall's plane. Level (pan
# 0 or 180) it lies 0.29 m below the rail, which clears the short switch-end
# wall, so the switch-end pose is turned 30 deg down to reach below that
# wall's top. Wrist_1 90 and wrist_2 -90 point tool0 +z, and the payload,
# straight out of the mounting wall; wrist_3 90 turns its wings off the
# floor. So only the arm meets the end walls.
_WRISTS_OUT_OF_THE_WALL = (90, -90, 90)
_STRETCHED_TOWARD_SWITCH_END = (30, 0, 0, *_WRISTS_OUT_OF_THE_WALL)
_STRETCHED_TOWARD_FAR_END = (180, 0, 0, *_WRISTS_OUT_OF_THE_WALL)


@pytest.mark.parametrize('rail_m, arm_deg, wall', [
    (RAIL_LOWER_M, _STRETCHED_TOWARD_SWITCH_END, 'switch-end wall'),
    (RAIL_UPPER_M, _STRETCHED_TOWARD_FAR_END, 'far-end wall'),
])
def test_end_wall_collision_is_detected(validator, rail_m, arm_deg, wall, capsys):
    """Reaching past a rail end into its end wall must register, by name.

    The arm's reach exceeds the 2 ft gap, so this pose crosses the wall's
    plane, below the wall's top. Before the end walls were modelled it was
    clear: the mounting wall and floor run parallel to the rail and the arm
    stays clear of both.
    """
    q = _q(rail_m, *arm_deg)
    tool_x = validator.robot.fkine(q, end='tool0').t[0]
    beyond = (tool_x < RAIL_LOWER_M - END_WALL_GAP_M if rail_m == RAIL_LOWER_M
              else tool_x > RAIL_UPPER_M + END_WALL_GAP_M)
    assert beyond, f'tool0 at X={tool_x:.3f} no longer reaches the {wall}'
    assert validator.check_all_collisions(q, verbose=True) is True
    assert f'vs {wall}' in capsys.readouterr().out


def test_arm_reaches_over_the_short_switch_end_wall(validator):
    """The switch-end wall is short enough to reach over; the far-end is not.

    Turned 20 deg up from level, the stretched arm passes the switch-end
    wall's plane above its 2 ft top and is clear. The same reach past the
    far end meets the full-height wall. A switch-end wall modelled full
    height would reject reaches the rig allows.
    """
    over = _q(RAIL_LOWER_M, -20, 0, 0, *_WRISTS_OUT_OF_THE_WALL)
    tool = validator.robot.fkine(over, end='tool0').t
    assert tool[0] < RAIL_LOWER_M - END_WALL_GAP_M, 'pose no longer reaches past the wall'
    assert validator.check_all_collisions(over) is False

    into_far = _q(RAIL_UPPER_M, 200, 0, 0, *_WRISTS_OUT_OF_THE_WALL)
    assert validator.robot.fkine(into_far, end='tool0').t[0] > RAIL_UPPER_M + END_WALL_GAP_M
    assert validator.check_all_collisions(into_far) is True


@pytest.mark.parametrize('rail_m, arm_deg', [
    (1.5, _STRETCHED_TOWARD_SWITCH_END),
    (1.5, _STRETCHED_TOWARD_FAR_END),
])
def test_stretched_arm_is_clear_mid_rail(validator, rail_m, arm_deg):
    """The same stretched poses clear both end walls from the rail's middle.

    Guards against an end wall placed inside the room, which would fail every
    plan that reaches along the rail.
    """
    assert validator.check_all_collisions(_q(rail_m, *arm_deg)) is False


@pytest.mark.parametrize('rail_m, arm_deg', [
    (RAIL_LOWER_M, _STRETCHED_TOWARD_SWITCH_END),
    (RAIL_UPPER_M, _STRETCHED_TOWARD_FAR_END),
])
def test_clearance_counts_the_end_walls(validator, rail_m, arm_deg):
    """robot_checks.collision_distance must see the end walls too.

    Clearance screens home poses and warmup via poses. A body missing from it
    would rank a pose that touches an end wall as comfortably clear.
    """
    from ur10e_trajectory_pkg import robot_checks

    assert robot_checks.collision_distance(validator, _q(rail_m, *arm_deg)) <= 0.0


# --------------------------------------------------------------------------
# The payload on the flange
# --------------------------------------------------------------------------

INCH_M = 0.0254


def _payload_boxes(validator):
    """(centre in the payload frame, full extents) of each payload collision box."""
    link = {name: index for index, name in validator._pb_link_name_by_index.items()}['payload']
    shapes = pb.getCollisionShapeData(validator.robot_id, link,
                                      physicsClientId=validator._pb_client)
    boxes = [(np.asarray(s[5]), np.asarray(s[3])) for s in shapes if s[2] == pb.GEOM_BOX]
    return sorted(boxes, key=lambda box: box[0][2])      # boom, prism, wings


def test_the_payload_frame_is_tool0(validator):
    """The payload is described in tool0's frame, so the two must coincide."""
    q = _q(1.5, 0, -75, 100, -115, 100, 0)
    np.testing.assert_allclose(validator.robot.fkine(q, end='payload').A,
                               validator.robot.fkine(q, end='tool0').A, atol=1e-9)


def test_the_payload_is_a_padded_boom_and_prism_out_along_tool_z(validator):
    """A 1 ft, 1 in boom, then the 6 x 6 x 18 in prism.

    The prism's bottom face is centred on the boom tip, and its geometric
    centre, 21 in out, is where target_builder puts the centre of rotation.
    Collision boxes are padded: the boom to 1.5 in square, the prism by 2 cm
    on every side.
    """
    from ur10e_trajectory_pkg.target_builder import MOUNT_OFFSET_M

    (boom_c, boom_e), (prism_c, prism_e), _ = _payload_boxes(validator)
    np.testing.assert_allclose(boom_e, [1.5 * INCH_M, 1.5 * INCH_M, 12 * INCH_M], atol=1e-6)
    assert boom_c[2] - boom_e[2] / 2 == pytest.approx(0.0, abs=1e-6)
    assert boom_c[2] + boom_e[2] / 2 == pytest.approx(12 * INCH_M, abs=1e-6)
    np.testing.assert_allclose(prism_e, [6 * INCH_M + 0.04, 6 * INCH_M + 0.04,
                                         18 * INCH_M + 0.04], atol=1e-6)
    assert prism_c[2] - prism_e[2] / 2 == pytest.approx(12 * INCH_M - 0.02, abs=1e-6)
    assert prism_c[2] == pytest.approx(MOUNT_OFFSET_M, abs=1e-6), 'COR is not the body centre'
    assert prism_c[2] + prism_e[2] / 2 == pytest.approx(30 * INCH_M + 0.02, abs=1e-6)
    np.testing.assert_allclose(boom_c[:2], 0.0, atol=1e-9)
    np.testing.assert_allclose(prism_c[:2], 0.0, atol=1e-9)


def test_the_wings_span_tool_x_flush_with_the_prism_top(validator):
    """Two flat 6 x 12 x 0.125 in wings out of the prism's +x and -x faces.

    Together they are one 30 x 6 x 0.125 in span along tool0 x, flush with
    the prism's top face 30 in from the flange, padded 2 cm on every side.
    Their padded top meets the padded prism's top exactly.
    """
    _, (prism_c, prism_e), (wing_c, wing_e) = _payload_boxes(validator)
    np.testing.assert_allclose(wing_e, [30 * INCH_M + 0.04, 6 * INCH_M + 0.04,
                                        0.125 * INCH_M + 0.04], atol=1e-6)
    np.testing.assert_allclose(wing_c[:2], 0.0, atol=1e-9)
    assert wing_c[2] + wing_e[2] / 2 == pytest.approx(30 * INCH_M + 0.02, abs=1e-6)
    assert wing_c[2] + wing_e[2] / 2 == pytest.approx(prism_c[2] + prism_e[2] / 2, abs=1e-6)
    assert np.argmax(wing_e) == 0, 'the wings no longer span tool0 x'


def test_environment_clearance_counts_the_payload(validator):
    """The payload moves with the arm, so clearance gates must see it."""
    from ur10e_trajectory_pkg import robot_checks

    names = {validator._pb_link_name_by_index[i]
             for i in robot_checks.arm_link_indices(validator)}
    assert 'payload' in names


@pytest.mark.parametrize('wrist_2_deg, clear', [(-80, False), (100, True)])
def test_the_payload_decides_which_way_the_home_can_face(validator, wrist_2_deg, clear):
    """The old home pointed tool0 +z at the mounting wall: harmless for a
    bare flange, but it drives the 30 in payload into the wall. The same
    posture with wrist_2 turned 180 deg points it out of the wall."""
    q = _q(1.5, 0, -75, 100, -115, wrist_2_deg, 0)
    assert validator.check_all_collisions(q) is (not clear)


def bare_flange_urdf(directory):
    """A copy of the rig URDF with the payload removed, written into directory.

    For regressions recorded on a bare flange, before the payload was
    mounted: their solver and tracking results are about the arm's
    kinematics, and the payload would collide first and mask them.
    """
    import xml.etree.ElementTree as ET

    tree = ET.parse(_urdf_path())
    root = tree.getroot()
    for element in list(root):
        if (element.tag == 'link' and element.get('name') == 'payload') or (
                element.tag == 'joint' and element.get('name') == 'flange-payload'):
            root.remove(element)
    path = Path(directory) / 'ur10e_bare_flange.urdf'
    tree.write(path)
    return str(path)


def test_the_bare_flange_model_has_no_payload(validator, tmp_path):
    bare = TrajectoryValidator(bare_flange_urdf(tmp_path),
                               mesh_base_path=get_package_share_directory('ur_description'))
    assert 'payload' not in bare._pb_link_name_by_index.values()
    assert 'payload' in validator._pb_link_name_by_index.values()
    q = _q(1.5, *HOME_ARM_DEG)
    assert validator.check_all_collisions(q) is True     # the payload meets the floor
    assert bare.check_all_collisions(q) is False
