#!/usr/bin/env python3
"""Stage 6 gate: validate the trajectory that is actually played back.

The graph proves safety within its own discrete model: secant velocity
between waypoints, three linearly interpolated collision samples per edge.
Playback follows a PCHIP interpolant, a different joint-space curve through
the same endpoints, so the graph's verdict is about its edges rather than
about the motion performed.

Run against the first full graph path, the trajectory proper passed every
check while the APPROACH failed three: peak velocity 1.158 times the limit,
a singular configuration between the endpoints, and a commanded-twist margin
of 0.864. Assigning the approach two seconds makes it admissible under a
secant check; it does not construct a dynamically smooth motion.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg import continuous_validator as cv
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

DT = 0.1
LIMITS = np.array([1.0] + [2.0] * 6)


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description')
    )


@pytest.fixture(scope='module')
def smooth_path():
    """Small, even steps, so the interpolant has nothing to overshoot."""
    return np.stack([
        np.concatenate(([0.5 + 0.002 * i],
                        np.deg2rad([0.0, -135.0 + 0.2 * i, 90.0, -90.0, 45.0, 0.0])))
        for i in range(12)
    ])


def _times(count, dt=DT):
    return np.arange(count) * dt


def test_the_interpolant_is_sampled_far_above_the_waypoint_rate(smooth_path):
    """Checking at the waypoint rate re-asks what the graph already answered.

    The interpolant only departs from the secant BETWEEN waypoints, so that is
    where the check has to look.
    """
    times = _times(len(smooth_path))
    dense_times, _, _ = cv.interpolate(smooth_path, times, rate_hz=200.0)
    assert len(dense_times) > 10 * len(smooth_path)


def test_derivatives_come_from_the_interpolant_not_from_differencing(smooth_path):
    """Otherwise the answer would depend on the sampling rate."""
    times = _times(len(smooth_path))
    coarse_times, coarse_interp, _ = cv.interpolate(smooth_path, times, 100.0)
    fine_times, fine_interp, _ = cv.interpolate(smooth_path, times, 800.0)

    coarse = cv.derivative_extremes(coarse_interp, coarse_times)
    fine = cv.derivative_extremes(fine_interp, fine_times)
    np.testing.assert_allclose(coarse['velocity'], fine['velocity'], rtol=0.05)


def test_a_smooth_path_reports_no_limit_violations(smooth_path):
    times = _times(len(smooth_path))
    dense_times, interpolator, _ = cv.interpolate(smooth_path, times, 200.0)
    extremes = cv.derivative_extremes(interpolator, dense_times)
    assert cv.limit_violations(extremes, LIMITS) == {}


def test_an_interpolant_can_exceed_the_secant_velocity():
    """The reason this gate exists.

    Two endpoints whose difference is inside the limit can still be joined by
    a curve that is not. A secant check cannot see it.
    """
    path = np.zeros((4, 7))
    path[1, 1] = 0.19       # just inside 2.0 rad/s * 0.1 s
    path[2, 1] = 0.0
    path[3, 1] = 0.19
    times = _times(4)

    secant = np.max(np.abs(np.diff(path, axis=0)) / DT)
    dense_times, interpolator, _ = cv.interpolate(path, times, rate_hz=500.0)
    actual = np.max(np.abs(interpolator.derivative(1)(dense_times)))
    assert secant <= LIMITS[1] + 1e-9
    assert actual > secant


def test_collision_checking_reports_its_convergence(validator, smooth_path):
    """A clean result at one sample count means nothing on its own."""
    times = _times(len(smooth_path))
    _, interpolator, _ = cv.interpolate(smooth_path, times, rate_hz=200.0)
    result = cv.collision_convergence(validator, interpolator, times,
                                      ladder=(50, 100, 200))
    assert len(result['ladder']) == 3
    assert result['converged'] is True
    assert result['collision_found'] is False


def test_desired_pose_between_waypoints_uses_slerp():
    """Interpolating quaternion components directly leaves the unit sphere and
    describes a rotation the task never asked for."""
    from scipy.spatial.transform import Rotation

    start = Rotation.identity().as_quat()
    end = Rotation.from_euler('z', 90, degrees=True).as_quat()
    positions = np.zeros((2, 3))
    _, rotation = cv.desired_pose_at([0.0, 1.0], positions, [start, end], 0.5)

    angle = Rotation.from_matrix(rotation).magnitude()
    assert np.rad2deg(angle) == pytest.approx(45.0, abs=1e-6)


def test_tracking_is_measured_between_waypoints_not_only_at_them(validator,
                                                                 smooth_path):
    """A joint-space curve through two correct endpoints need not stay on the
    Cartesian path between them."""
    times = _times(len(smooth_path))
    dense_times, interpolator, _ = cv.interpolate(smooth_path, times, rate_hz=100.0)
    poses = [validator.robot.fkine(q, end='tool0') for q in smooth_path]
    positions = np.stack([p.t for p in poses])
    quaternions = np.stack([np.roll(np.array(p.UnitQuaternion().A), -1)
                            for p in poses])

    report = cv.tracking_error(validator, interpolator, dense_times, times,
                               positions, quaternions)
    assert report['max_position_error_m'] >= 0.0
    assert 'at_time_s' in report


def test_twist_margin_reports_alpha_star(validator, smooth_path):
    """alpha* below 1 means the commanded motion cannot be tracked at that
    rate, whatever the condition number says."""
    times = _times(len(smooth_path))
    dense_times, interpolator, _ = cv.interpolate(smooth_path, times, rate_hz=100.0)
    report = cv.twist_margin(validator, interpolator, dense_times, LIMITS)
    assert report['min_alpha_star'] > 0.0
    assert report['feasible'] is True


def test_a_stationary_trajectory_has_unbounded_twist_margin(validator):
    """No commanded motion cannot be infeasible."""
    path = np.tile(np.concatenate(([1.0], np.deg2rad([0.0, -135.0, 90.0, -90.0, 45.0, 0.0]))),
                   (5, 1))
    times = _times(5)
    dense_times, interpolator, _ = cv.interpolate(path, times, rate_hz=100.0)
    report = cv.twist_margin(validator, interpolator, dense_times, LIMITS)
    assert report['feasible'] is True


def test_declared_higher_order_limits_are_starting_values_not_inherited():
    """Nothing upstream ever bounded acceleration or jerk, so these are to be
    measured against rather than trusted."""
    assert cv.ARM_ACCELERATION_LIMIT_RAD_S2 > 0
    assert cv.ARM_JERK_LIMIT_RAD_S3 > cv.ARM_ACCELERATION_LIMIT_RAD_S2
    assert cv.RAIL_ACCELERATION_LIMIT_M_S2 > 0
