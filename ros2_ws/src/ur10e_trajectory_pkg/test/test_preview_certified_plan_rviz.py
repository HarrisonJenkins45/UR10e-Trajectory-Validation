"""The RViz Target frame must use the planned body pose, not a camera frame."""

import csv
import hashlib
from itertools import islice

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from ur10e_trajectory_pkg import (
    ClientNode,
    motion_limits,
    preview_certified_plan_rviz as preview,
)


def test_target_frame_composes_mount_and_slerps_from_nonidentity_start(
    monkeypatch,
):
    start = Rotation.from_euler("x", 90, degrees=True)
    turn = Rotation.from_euler("z", 90, degrees=True)
    end = turn * start
    mount_rotation = Rotation.from_euler("y", 30, degrees=True)
    mount = np.eye(4)
    mount[:3, :3] = mount_rotation.as_matrix()
    mount[:3, 3] = [0.1, 0.0, 0.0]

    monkeypatch.setattr(
        ClientNode,
        "mount_record",
        lambda: {"name": "T_EG", "transform": mount.tolist()},
    )
    monkeypatch.setattr(
        ClientNode,
        "plan_targets",
        lambda plan, csv_path: (
            [1.0, 1.0],
            [0.5, 0.5],
            [0.5, 0.5],
            [start.as_quat(), -end.as_quat()],
            [0.0, 1.0],
        ),
    )
    plan = {
        "mount": {"name": "T_EG", "transform": mount.tolist()},
        "q_path": [[0.0] * 7, [0.0] * 7],
    }

    positions, quaternions = preview.target_poses(
        plan, "unused.csv", [0.0, 0.5, 1.0]
    )

    first_expected = start * mount_rotation
    last_expected = end * mount_rotation
    first_offset = start.apply(mount[:3, 3])
    last_offset = end.apply(mount[:3, 3])
    np.testing.assert_allclose(
        positions[0], [1.0, 0.5, 0.5] + first_offset, atol=1e-12
    )
    np.testing.assert_allclose(
        positions[-1], [1.0, 0.5, 0.5] + last_offset, atol=1e-12
    )
    np.testing.assert_allclose(
        positions[1], (positions[0] + positions[-1]) / 2, atol=1e-12
    )
    for actual, expected in (
        (quaternions[0], first_expected),
        (quaternions[-1], last_expected),
    ):
        np.testing.assert_allclose(
            Rotation.from_quat(actual).as_matrix(),
            expected.as_matrix(),
            atol=1e-12,
        )
    midpoint = Rotation.from_quat(quaternions[1])
    assert np.isclose((midpoint * first_expected.inv()).magnitude(), np.pi / 4)


def test_target_frame_rejects_mount_changed_since_plan(monkeypatch):
    monkeypatch.setattr(
        ClientNode,
        "mount_record",
        lambda: {"name": "T_EG", "transform": np.eye(4).tolist()},
    )
    changed = np.eye(4)
    changed[0, 3] = 0.1
    with pytest.raises(ValueError, match="differs"):
        preview.target_poses(
            {"mount": {"name": "T_EG", "transform": changed.tolist()}},
            "unused.csv",
            [0.0, 1.0],
        )


def test_recording_digest_and_slice_timing_are_checked(tmp_path):
    recording = tmp_path / "camera_traj.csv"
    with recording.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["timestamp"])
        writer.writerows(([value] for value in (0.0, 0.1, 0.2, 0.3)))
    digest = hashlib.sha256(recording.read_bytes()).hexdigest()
    plan = {
        "recording": {"csv_sha256": digest},
        "start_index": 1,
        "recorded_waypoints": 3,
    }
    assert preview.recording_step(recording, plan) == pytest.approx(0.1)
    plan["recording"]["csv_sha256"] = "wrong"
    with pytest.raises(ValueError, match="SHA-256"):
        preview.recording_step(recording, plan)


def test_display_loop_reverses_joint_and_target_frames_together():
    joints = np.arange(21).reshape(3, 7)
    positions = np.arange(9).reshape(3, 3)
    orientations = np.arange(12).reshape(3, 4)

    one_shot = list(preview.replay_passes(joints, positions, orientations))
    assert len(one_shot) == 1
    for actual, expected in zip(
        one_shot[0], (joints, positions, orientations)
    ):
        np.testing.assert_array_equal(actual, expected)

    passes = list(
        islice(preview.replay_passes(joints, positions, orientations, True), 3)
    )
    for actual, expected in zip(
        passes[1], (joints[::-1], positions[::-1], orientations[::-1])
    ):
        np.testing.assert_array_equal(actual, expected)
    for actual, expected in zip(
        passes[2], (joints, positions, orientations)
    ):
        np.testing.assert_array_equal(actual, expected)


def test_full_display_cycle_returns_home_without_a_pose_jump():
    home = np.zeros(7)
    home[0] = 1.5
    via = home.copy()
    via[0] = 1.48
    start = home.copy()
    start[0] = 1.46
    end = home.copy()
    end[0] = 1.44
    warmup = [
        np.stack((home, via)),
        np.stack((via, start)),
    ]
    task = np.stack((start, end))
    positions = np.array([[1.0, 0.5, 0.5], [1.0, 0.5, 0.5]])
    orientations = np.array([[0.0, 0.0, 0.0, 1.0],
                             [0.0, 0.0, 0.1, 0.995]])

    passes = list(preview.full_cycle_passes(
        home, warmup, task, positions, orientations, 10.0, 0.2
    ))

    assert [item[0] for item in passes] == [
        "home", "warmup", "via dwell", "warmup", "task",
        "task return", "warmup return", "via dwell", "warmup return",
    ]
    for before, after in zip(passes, passes[1:]):
        for component in (1, 2, 3):
            np.testing.assert_allclose(before[component][-1],
                                       after[component][0])
    for component in (1, 2, 3):
        np.testing.assert_allclose(passes[-1][component][-1],
                                   passes[0][component][0])
    np.testing.assert_allclose(passes[5][1], task[::-1])
    np.testing.assert_allclose(passes[5][3], orientations[::-1])


def test_archived_warmup_is_retimed_to_the_current_rail_cap():
    rests = np.zeros((3, 7))
    rests[:, 0] = [1.5, 0.6314602052823286, 0.60]
    stored = np.asarray([1.6285121150956339, 2.0])

    actual = preview.current_warmup_durations(rests, stored)

    expected = 15.0 * abs(rests[1, 0] - rests[0, 0]) / (
        8.0 * motion_limits.RAIL_VEL_SAFETY_CAP
    )
    assert actual[0] == pytest.approx(expected)
    assert actual[0] == pytest.approx(3.2570242301912677)
    assert actual[1] == pytest.approx(stored[1])
    peak_rail_speed = 15.0 * abs(rests[1, 0] - rests[0, 0]) / (8.0 * actual[0])
    assert peak_rail_speed <= 0.5 + 1e-12
