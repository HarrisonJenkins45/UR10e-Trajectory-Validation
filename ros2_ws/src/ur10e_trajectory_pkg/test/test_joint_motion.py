"""Validation, service playback and RViz must use one joint motion curve."""

from types import SimpleNamespace

import numpy as np
import pytest

from ur10e_trajectory_pkg import continuous_validator, joint_motion, warmup
from ur10e_trajectory_pkg import preview_certified_plan_rviz as preview
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator


def _path():
    first = np.array([1.2, 0.3, -1.8, 1.5, -2.0, 0.6, -0.2])
    changes = np.array([
        [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00],
        [0.01, 0.02, 0.04, 0.01, 0.03, 0.02, 0.01],
        [0.03, 0.05, 0.05, 0.04, 0.08, 0.03, 0.04],
        [0.04, 0.06, 0.08, 0.08, 0.09, 0.07, 0.05],
    ])
    return first + changes


def _plan(path, duration=0.6):
    home = path[0].copy()
    home[0] -= 0.1
    return {
        'home': home.tolist(), 'q_path': path.tolist(),
        'recorded_waypoints': len(path), 'spin_up_s': None,
        'warmup_route': {
            'rest_points': [home.tolist(), path[0].tolist()],
            'segment_durations_s': [duration], 'dwell_s': 0.5,
        },
    }


def test_service_rviz_and_validator_use_the_same_task_curve():
    path, dt, rate_hz = _path(), 0.1, 30.0
    times, positions, velocities = joint_motion.task_playback(path, dt, rate_hz)

    segment = {'start_idx': 0, 'q_full': path}
    service_velocities, service_positions, service_times = (
        TrajectoryValidator.process_task_segment(
            SimpleNamespace(framerate=rate_hz), segment, dt))
    np.testing.assert_allclose(service_times, times, atol=0, rtol=0)
    np.testing.assert_allclose(service_positions, positions, atol=0, rtol=0)
    np.testing.assert_allclose(service_velocities, velocities, atol=0, rtol=0)

    _, rviz_positions, rviz_times, _ = preview.playback_frames(
        _plan(path), dt, rate_hz)
    np.testing.assert_allclose(rviz_times, times, atol=0, rtol=0)
    np.testing.assert_allclose(rviz_positions, positions, atol=0, rtol=0)

    dense_times, curve, dense_positions = continuous_validator.interpolate(
        path, joint_motion.task_waypoint_times(len(path), dt), 200.0)
    np.testing.assert_allclose(curve(times), positions, atol=0, rtol=0)
    np.testing.assert_allclose(curve(dense_times), dense_positions, atol=0, rtol=0)
    assert dense_times[-1] < times[-1]  # validation keeps its half-open grid


def test_preview_and_warmup_planner_use_the_same_quintic():
    path, rate_hz = _path(), 30.0
    plan = _plan(path)
    warmup_frames, _, _, durations = preview.playback_frames(plan, 0.1, rate_hz)
    rests = np.asarray(plan['warmup_route']['rest_points'])
    times, expected, velocity, acceleration = joint_motion.sample_rest_to_rest(
        rests[0], rests[1], durations[0], rate_hz)
    assert warmup.sample_rest_to_rest is joint_motion.sample_rest_to_rest
    np.testing.assert_allclose(warmup_frames[0], expected, atol=0, rtol=0)
    np.testing.assert_allclose(expected[0], rests[0], atol=1e-12)
    np.testing.assert_allclose(expected[-1], rests[1], atol=1e-12)
    np.testing.assert_allclose(velocity[[0, -1]], 0.0, atol=1e-12)
    np.testing.assert_allclose(acceleration[[0, -1]], 0.0, atol=1e-12)
    assert times[-1] == pytest.approx(durations[0])


@pytest.mark.parametrize('dt,rate', [(0.0, 30.0), (0.1, 0.0),
                                     (float('nan'), 30.0)])
def test_task_motion_refuses_invalid_timing(dt, rate):
    with pytest.raises(ValueError):
        joint_motion.task_playback(_path(), dt, rate)


def test_warmup_motion_refuses_invalid_endpoints_and_timing():
    home = _path()[0]
    with pytest.raises(ValueError, match='warmup motion'):
        joint_motion.sample_rest_to_rest(home, np.full(7, np.nan), 1.0)
    with pytest.raises(ValueError, match='warmup motion'):
        joint_motion.sample_rest_to_rest(home, home, 0.0)
