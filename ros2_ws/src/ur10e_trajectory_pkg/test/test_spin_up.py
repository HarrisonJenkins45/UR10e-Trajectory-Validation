#!/usr/bin/env python3
"""Spin-up gate: the task starts from rest and then reproduces the recording.

The recording begins at full tumble rate, so without a spin-up the arm would
have to be moving when the task starts. The spin-up warps recorded time so the
playback rate rises smoothly from zero, and every recorded pose is kept.
"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from ur10e_trajectory_pkg import frames
from ur10e_trajectory_pkg.target_builder import (
    SPIN_UP_PROFILE,
    apply_spin_up,
    build_trajectory_targets,
    recorded_start_rate,
    spin_up_recorded_time,
)

DT = 0.1


def _recorded_motion(count=40, rate=0.08):
    """A constant-rate tumble about a tilted axis, with a small drift."""
    axis = np.array([1.0, 2.0, 0.5]) / np.linalg.norm([1.0, 2.0, 0.5])
    times = np.arange(count) * DT
    poses = np.tile(np.eye(4), (count, 1, 1))
    poses[:, :3, :3] = Rotation.from_rotvec(np.outer(times * rate, axis)).as_matrix()
    poses[:, :3, 3] = np.outer(times, [0.01, 0.0, -0.005])
    return poses, times


def test_the_playback_rate_starts_at_zero_and_joins_at_one():
    T = 2.0
    t = np.linspace(0.0, 4.0, 4001)
    tau = spin_up_recorded_time(t, T)
    rate = np.gradient(tau, t)
    assert tau[0] == 0.0
    assert rate[0] == pytest.approx(0.0, abs=1e-6)
    assert np.all(np.diff(tau) >= -1e-12), 'recorded time never runs backwards'
    np.testing.assert_allclose(rate[t > T + 0.01], 1.0, atol=1e-6)
    assert spin_up_recorded_time(np.array([T]), T)[0] == pytest.approx(T / 2)


def test_the_first_pose_is_the_recorded_first_pose_at_rest():
    motion, times = _recorded_motion()
    warped, out_times, record = apply_spin_up(motion, times, 2.0)
    np.testing.assert_allclose(warped[0], motion[0], atol=1e-12)
    first_step = np.linalg.norm(
        (Rotation.from_matrix(warped[1, :3, :3])
         * Rotation.from_matrix(warped[0, :3, :3]).inv()).as_rotvec()) / DT
    assert first_step < 0.01 * record['recorded_start_rate_rad_s']
    assert record['task_starts_at_rest'] is True
    assert record['profile'] == SPIN_UP_PROFILE


def test_every_recorded_pose_is_reproduced_after_the_spin_up():
    motion, times = _recorded_motion()
    warped, out_times, record = apply_spin_up(motion, times, 2.0)
    added = record['samples_added']
    assert added == 10
    assert len(warped) == len(motion) + added
    np.testing.assert_allclose(np.diff(out_times), DT, atol=1e-12)
    np.testing.assert_array_equal(warped[2 * added:], motion[added:])


def test_the_angular_rate_ramps_monotonically_to_the_recorded_rate():
    motion, times = _recorded_motion(rate=0.08)
    warped, _, record = apply_spin_up(motion, times, 2.0)
    rotations = Rotation.from_matrix(warped[:, :3, :3])
    rates = np.linalg.norm((rotations[1:] * rotations[:-1].inv()).as_rotvec(),
                           axis=1) / DT
    ramp = rates[:2 * record['samples_added']]
    assert np.all(np.diff(ramp) >= -1e-9)
    np.testing.assert_allclose(rates[-5:], 0.08, rtol=1e-6)
    assert record['peak_spin_up_angular_acceleration_rad_s2'] == pytest.approx(
        0.08 * 1.5 / 2.0, rel=1e-6)


def test_the_duration_rounds_up_to_land_the_join_on_a_recorded_sample():
    motion, times = _recorded_motion()
    _, _, record = apply_spin_up(motion, times, 0.25)
    assert record['duration_s'] == pytest.approx(0.4)
    assert record['requested_duration_s'] == 0.25
    assert record['samples_added'] == 2


def test_the_builder_is_unchanged_without_a_spin_up():
    try:
        targets, metadata = build_trajectory_targets(num_waypoints=12,
                                                     return_metadata=True)
    except FileNotFoundError:
        pytest.skip('packaged trajectory CSV not present')
    assert metadata['spin_up'] is None
    assert metadata['num_samples'] == 12 == len(targets[0])


def test_the_builder_with_a_spin_up_starts_at_the_same_pose():
    try:
        plain = build_trajectory_targets(num_waypoints=40)
        (x, y, z, quaternions, times), metadata = build_trajectory_targets(
            num_waypoints=40, return_metadata=True, spin_up_s=2.0)
    except FileNotFoundError:
        pytest.skip('packaged trajectory CSV not present')
    assert metadata['spin_up']['samples_added'] == 10
    assert len(x) == 50 == metadata['num_samples']
    np.testing.assert_allclose([x[0], y[0], z[0]],
                               [plain[0][0], plain[1][0], plain[2][0]], atol=1e-12)
    first = frames.poses_from_positions_quaternions([[0, 0, 0]], [quaternions[0]])[0]
    reference = frames.poses_from_positions_quaternions([[0, 0, 0]], [plain[3][0]])[0]
    np.testing.assert_allclose(first[:3, :3], reference[:3, :3], atol=1e-9)
    # After the spin-up the recorded targets resume, shifted by 10 samples.
    np.testing.assert_allclose(quaternions[20:] * np.sign(quaternions[20:, 3:4]),
                               plain[3][10:] * np.sign(plain[3][10:, 3:4]), atol=1e-9)


def test_the_initial_rate_matches_what_the_spin_up_records():
    """A spin-up rule has to know the recording's starting rate before it
    picks a duration, and that figure must be the same one apply_spin_up
    reports afterwards -- otherwise the rule and the record disagree."""
    before = recorded_start_rate(num_waypoints=500)
    assert before['rate_rad_s'] > 0.0
    assert before['step_s'] > 0.0

    *_, times = build_trajectory_targets(num_waypoints=500, return_metadata=False)
    assert before['step_s'] == pytest.approx(float(np.diff(times)[0]), rel=1e-12)

    _targets, metadata = build_trajectory_targets(
        num_waypoints=500, return_metadata=True, spin_up_s=2.0)
    recorded = metadata['spin_up']['recorded_start_rate_rad_s']
    assert before['rate_rad_s'] == pytest.approx(recorded, rel=1e-9)
