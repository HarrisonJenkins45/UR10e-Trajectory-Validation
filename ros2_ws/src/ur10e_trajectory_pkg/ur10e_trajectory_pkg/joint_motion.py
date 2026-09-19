"""ROS-free joint motion used by validation and playback.

Warmup is a rest-to-rest quintic; the task is PCHIP through the lifted
seven-joint waypoint path. Validation may inspect that curve at a finer rate
than playback, but neither consumer may construct a different curve.
"""

import numpy as np
from scipy.interpolate import PchipInterpolator

from ur10e_trajectory_pkg.configurations import NUM_JOINTS


CONTROLLER_HZ = 500.0


def sample_rest_to_rest(home, start, duration, rate_hz=CONTROLLER_HZ):
    """Quintic warmup positions, velocities and accelerations, ends included."""
    home, start = np.asarray(home, float), np.asarray(start, float)
    if (home.shape != (NUM_JOINTS,) or start.shape != (NUM_JOINTS,) or
            not np.all(np.isfinite(home)) or not np.all(np.isfinite(start)) or
            not np.isfinite(duration) or duration <= 0 or
            not np.isfinite(rate_hz) or rate_hz <= 0):
        raise ValueError('warmup motion needs finite 7-joint endpoints, '
                         'positive duration and positive rate')
    count = max(2, int(np.ceil(duration * rate_hz)) + 1)
    times = np.linspace(0.0, duration, count)
    s = times / duration
    shape = 10 * s**3 - 15 * s**4 + 6 * s**5
    rate = (30 * s**2 - 60 * s**3 + 30 * s**4) / duration
    curvature = (60 * s - 180 * s**2 + 120 * s**3) / duration**2
    delta = start - home
    return (times, home + np.outer(shape, delta), np.outer(rate, delta),
            np.outer(curvature, delta))


def task_curve(path, waypoint_times):
    """The single continuous joint-space curve for a lifted task path."""
    path = np.asarray(path, dtype=float)
    times = np.asarray(waypoint_times, dtype=float)
    if (path.ndim != 2 or path.shape[1] != NUM_JOINTS or len(path) < 2 or
            times.shape != (len(path),) or not np.all(np.isfinite(path)) or
            not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0)):
        raise ValueError('task motion needs finite (N, 7) joints and increasing times')
    return PchipInterpolator(times, path, axis=0)


def task_waypoint_times(count, dt):
    """Waypoint times relative to the task start, which is time zero."""
    if count < 2 or not np.isfinite(dt) or dt <= 0:
        raise ValueError('task motion needs at least two waypoints and positive dt')
    return np.arange(count) * float(dt)


def task_playback(path, dt, rate_hz):
    """Timestamped task positions and velocities at the publishing rate.

    Include both endpoints, as the service and RViz have always done. The
    last interval can be fractionally shorter than 1/rate_hz because the
    recorded task duration need not be an integer number of display frames.
    """
    if not np.isfinite(rate_hz) or rate_hz <= 0:
        raise ValueError('playback rate must be positive and finite')
    path = np.asarray(path, dtype=float)
    times = task_waypoint_times(len(path), dt)
    curve = task_curve(path, times)
    count = max(2, int(round(times[-1] * rate_hz)) + 1)
    sampled_times = np.linspace(0.0, times[-1], count)
    return (sampled_times, curve(sampled_times),
            curve.derivative(1)(sampled_times))


def task_validation_samples(path, waypoint_times, rate_hz=CONTROLLER_HZ):
    """A finer, half-open sample grid over the same curve for safety checks."""
    if not np.isfinite(rate_hz) or rate_hz <= 0:
        raise ValueError('validation rate must be positive and finite')
    times = np.asarray(waypoint_times, dtype=float)
    curve = task_curve(path, times)
    dense_times = np.arange(times[0], times[-1], 1.0 / rate_hz)
    return dense_times, curve, curve(dense_times)
