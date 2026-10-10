#!/usr/bin/env python3
"""Slices of a recording, and the partial path a disconnected graph leaves.

A section is certified on a slice, so the slice's targets must be exactly the
full recording's targets over the same samples: then a path planned on the
recording, restricted to the slice, is a path for the slice.
"""
import numpy as np
import pandas as pd
import pytest
from scipy.spatial.transform import Rotation

from ur10e_trajectory_pkg import target_builder as targets
from ur10e_trajectory_pkg import graph_planner
from ur10e_trajectory_pkg import plan_artifact
from ur10e_trajectory_pkg import trajectory_input
from ur10e_trajectory_pkg.planning_runtime import file_digest


@pytest.fixture
def packaged():
    if file_digest(targets.DEFAULT_CSV_PATH) is None:
        pytest.skip('packaged trajectory CSV not present')
    return targets.DEFAULT_CSV_PATH


def _rotations(quaternions):
    return Rotation.from_quat(np.asarray(quaternions)).as_matrix()


def test_loaded_input_and_path_adapter_build_identical_targets(packaged):
    source = trajectory_input.load(packaged)
    direct = targets.build_targets(source, 20, start_index=25, spin_up_s=0.4)
    via_path = targets.build_trajectory_targets(
        packaged, 20, start_index=25, spin_up_s=0.4)
    for actual, expected in zip(direct, via_path):
        np.testing.assert_array_equal(actual, expected)


def test_attitude_only_csv_needs_no_translation_columns(tmp_path):
    quaternions = Rotation.from_euler('xyz', [[10, 20, 30], [12, 23, 34],
                                                  [14, 26, 38]], degrees=True).as_quat()
    frame = pd.DataFrame(quaternions, columns=[
        'q_I_G_x', 'q_I_G_y', 'q_I_G_z', 'q_I_G_w'])
    frame.insert(0, 'timestamp', [0.0, 0.1, 0.2])
    csv = tmp_path / 'attitude.csv'
    frame.to_csv(csv, index=False)
    attitude_only = targets.build_trajectory_targets(csv, 3)
    frame[['p_G_I_x', 'p_G_I_y', 'p_G_I_z']] = [100.0, -50.0, 7.0]
    frame.to_csv(csv, index=False)
    with_translation = targets.build_trajectory_targets(csv, 3)
    for index in (0, 1, 2, 4):
        np.testing.assert_allclose(attitude_only[index], with_translation[index])
    np.testing.assert_allclose(_rotations(attitude_only[3]),
                               _rotations(with_translation[3]))


def test_a_slice_is_the_full_recording_over_the_same_samples(packaged):
    full_x, full_y, full_z, full_q, full_t = targets.build_trajectory_targets(packaged, 60)
    x, y, z, q, t = targets.build_trajectory_targets(packaged, 20, start_index=25)
    np.testing.assert_allclose(np.column_stack([x, y, z]),
                               np.column_stack([full_x, full_y, full_z])[25:45], atol=1e-12)
    np.testing.assert_allclose(_rotations(q), _rotations(full_q[25:45]), atol=1e-12)
    assert t[0] == 0.0
    np.testing.assert_allclose(t, full_t[25:45] - full_t[25], atol=1e-12)


def test_each_slice_derives_its_spin_up_from_its_own_start(packaged):
    measured = targets.recorded_start_rate(packaged, 40, start_index=300)
    _, metadata = targets.build_trajectory_targets(packaged, 40, start_index=300,
                                                  spin_up_s=0.4, return_metadata=True)
    assert measured['start_index'] == 300
    assert metadata['spin_up']['recorded_start_rate_rad_s'] == pytest.approx(
        measured['rate_rad_s'], rel=1e-9)
    assert measured['rate_rad_s'] != pytest.approx(
        targets.recorded_start_rate(packaged, 40)['rate_rad_s'], rel=1e-6)


def test_the_slice_and_its_mount_are_recorded(packaged):
    _, metadata = targets.build_trajectory_targets(packaged, 601, start_index=1200,
                                                  return_metadata=True)
    assert (metadata['start_index'], metadata['end_index']) == (1200, 1801)
    assert metadata['recorded_duration_s'] == pytest.approx(60.0)
    mount = metadata['mount']
    assert mount['name'] == 'T_EG' and mount['provisional'] is True
    assert mount['transform'] == targets.PROVISIONAL_T_EG.tolist()
    assert 'tool frame E' in mount['direction'] and 'target body frame G' in mount['direction']


def test_the_centre_of_rotation_is_the_body_centre_along_tool_z():
    """T_EG is a pure 21 in (0.5334 m) translation along tool0's +z: the
    centre of the 18 in body that starts at the end of the 1 ft boom."""
    expected = np.eye(4)
    expected[2, 3] = 0.5334
    assert targets.MOUNT_OFFSET_M == pytest.approx(0.5334, abs=1e-12)
    np.testing.assert_allclose(targets.PROVISIONAL_T_EG, expected, atol=1e-12)


def test_the_centre_of_rotation_holds_still_while_the_flange_orbits_it(packaged):
    """The placement point is the centre of rotation, not the flange.

    Each tool target, carried 21 in out along its own z, lands on the
    placement exactly; the flange itself sweeps a sphere of that radius
    about it as the target tumbles.
    """
    x, y, z, quaternions, _ = targets.build_trajectory_targets(packaged, 200,
                                                               start_index=300)
    tool = np.column_stack((x, y, z))
    centre = tool + targets.MOUNT_OFFSET_M * _rotations(quaternions)[:, :, 2]
    assert np.allclose(centre, targets.LEGACY_PLACEMENT_POSITION_RG, atol=1e-9)
    assert np.allclose(np.linalg.norm(tool - centre, axis=1), 0.5334, atol=1e-9)
    assert np.ptp(tool, axis=0).max() > 0.05, 'the flange no longer moves'


def test_a_slice_past_the_end_of_the_recording_is_refused(packaged):
    with pytest.raises(ValueError, match='from sample 4990'):
        targets.build_trajectory_targets(packaged, 20, start_index=4990)


def test_a_plan_rebuilds_its_own_slice(packaged):
    home = [0.0] * 7
    plan = {'schema_version': plan_artifact.SCHEMA_VERSION,
            'home': home, 'task_start': home, 'q_path': [home] * 12,
            'warmup_route': {'route_kind': 'direct', 'rest_points': [home, home],
                             'segment_durations_s': [1.0], 'dwell_s': 0.0},
            'recorded_waypoints': 2, 'spin_up_s': 2.0, 'placement': 'nominal',
            'start_index': 700, 'mount': targets.mount_record(),
            'target_frame': 'rail_base_link',
            'recording': {'csv_path': packaged,
                          'csv_sha256': trajectory_input.source_digest(packaged)}}
    x, y, z, q, _ = plan_artifact.plan_targets(plan)
    expected = targets.build_trajectory_targets(packaged, 2, spin_up_s=2.0, start_index=700)
    np.testing.assert_allclose(_rotations(q), _rotations(expected[3]), atol=1e-12)


class _Robot:
    qlim = (np.array([0.0] + [-2 * np.pi] * 6), np.array([3.0] + [2 * np.pi] * 6))


class _Validator:
    robot = _Robot()
    velocity_limits = np.array([1.0, 2.09, 2.09, 3.14, 3.14, 3.14, 3.14])

    def check_all_collisions(self, q, verbose=False):
        return False


def test_a_disconnected_graph_keeps_its_best_path_through_the_last_connected_layer():
    base = np.array([1.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    layers = [[base, base + 0.02], [base + 0.03], [base + 0.05, base + 0.01],
              [base + np.concatenate(([2.0], np.zeros(6)))]]          # rail jump
    graph = graph_planner.LayeredGraph(_Validator(), layers, 4, dt=0.1)
    result, gap, history = graph.shortest_path(None)
    assert result is None and gap == 3

    path, cost = graph_planner.best_partial_path(graph.last_history)
    assert len(path) == 3                                    # layers 0..2
    assert cost == min(entry[0] for entry in history[-1].values())
    for index, state in enumerate(path):
        assert any(np.allclose(state, row) for row in layers[index])


def test_a_complete_graph_returns_its_shortest_path_as_the_best_partial_one():
    base = np.array([1.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    layers = [[base, base + 0.02], [base + 0.03], [base + 0.05]]
    graph = graph_planner.LayeredGraph(_Validator(), layers, 3, dt=0.1)
    result, gap, _ = graph.shortest_path(None)
    assert gap is None
    path, cost = graph_planner.best_partial_path(graph.last_history)
    assert cost == result[1]
    np.testing.assert_array_equal(np.stack(path), np.stack(result[0]))


def test_no_start_state_has_no_partial_path():
    assert graph_planner.best_partial_path([{}]) is None
    assert graph_planner.best_partial_path(None) is None


def test_the_recording_is_turned_so_the_boom_points_out_of_the_wall(packaged):
    """The lab orientation keeps the boom (G's +z) on the room side throughout.

    The legacy placement laid the recording's inertial axes on the rail's,
    and the boom pointed into the mounting wall for a third of the packaged
    recording. Turned by R_RI, every recorded boom direction has a positive
    component out of the wall: the tumble cones about the outward normal.
    """
    source = trajectory_input.load(packaged)
    lab = targets.lab_orientation(source)
    np.testing.assert_allclose(lab @ lab.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(lab) == pytest.approx(1.0, abs=1e-12)
    booms = lab @ _rotations(source.orientations_xyzw)[:, :, 2].T
    assert booms[2].min() > 0.4, 'a recorded boom direction points at the wall'


def test_each_target_is_the_recorded_attitude_turned_by_the_lab_orientation(packaged):
    source = trajectory_input.load(packaged)
    lab = targets.lab_orientation(source)
    _, _, _, quaternions, _ = targets.build_targets(source, 50, start_index=700)
    turned = lab @ _rotations(source.orientations_xyzw[700:750])
    tool = _rotations(quaternions)                # T_EG has no rotation: tool = G
    np.testing.assert_allclose(tool, turned, atol=1e-9)


def test_the_lab_orientation_is_deterministic(packaged):
    source = trajectory_input.load(packaged)
    first = targets.lab_orientation(source)
    targets._lab_orientations.clear()
    assert np.array_equal(targets.lab_orientation(source), first)
