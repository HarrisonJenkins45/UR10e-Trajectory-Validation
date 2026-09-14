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


def test_alpha_star_is_at_least_the_reciprocal_of_velocity_usage(validator):
    """Self-consistency, and the check that caught a silent solver failure.

    When the requested twist is J @ qdot, scaling that same qdot by
    1 / usage is feasible by construction, so alpha* can never be below it --
    even on a rank-deficient Jacobian. A result under that bound is a
    numerical failure, not physical infeasibility.

    Measured at the singular start, the LP returned 0.0 while reporting
    'optimal' until the twist was normalised before solving.
    """
    for arm_deg in ([0.0, -135.0, 90.0, -90.0, 0.0, 0.0],      # singular
                    [0.0, -135.0, 90.0, -90.0, 45.0, 0.0],
                    [30.0, -100.0, 60.0, -70.0, 80.0, 25.0]):
        configuration = np.concatenate(([1.0], np.deg2rad(arm_deg)))
        jacobian = validator.compute_system_jacobian(configuration)
        rates = np.array([0.3, 1.5, 1.0, 0.8, 2.3, 0.5, 0.4])   # usage > 1
        usage = float(np.max(np.abs(rates) / LIMITS))

        result = cv.twist_alpha_star(jacobian, jacobian @ rates, LIMITS)
        assert result['solved']
        assert result['alpha'] >= 1.0 / usage - 1e-6, (
            f'alpha* {result["alpha"]:.6f} below the feasible bound '
            f'{1.0 / usage:.6f}; the LP did not find the optimum'
        )


def test_the_result_retains_status_and_residuals(validator):
    """So a caller can tell an optimum that satisfies the constraints from one
    that merely claims to."""
    configuration = np.concatenate(([1.0], np.deg2rad([0.0, -135.0, 90.0, -90.0, 45.0, 0.0])))
    jacobian = validator.compute_system_jacobian(configuration)
    result = cv.twist_alpha_star(jacobian, jacobian @ (0.1 * np.ones(7)), LIMITS)

    for key in ('alpha', 'status', 'solved', 'equality_residual'):
        assert key in result
    assert result['equality_residual'] < 1e-6


def test_solver_failure_is_distinct_from_a_feasible_zero():
    """Collapsing both to 0.0 reported a solver failure as infeasibility.

    An unsolved problem has alpha None and solved False; a genuinely
    unachievable twist has a numeric alpha with solved True.
    """
    # A twist with a component no joint can produce: the Jacobian is all zero,
    # so only alpha = 0 satisfies the equality.
    impossible = cv.twist_alpha_star(np.zeros((6, 7)), np.ones(6), LIMITS)
    assert impossible['solved'] is True
    assert impossible['alpha'] == pytest.approx(0.0, abs=1e-9)
    assert impossible['alpha'] is not None


def test_alpha_star_is_infinite_for_no_commanded_motion(validator):
    """No motion cannot be infeasible."""
    jacobian = validator.compute_system_jacobian(
        np.concatenate(([1.0], np.deg2rad([0.0, -135.0, 90.0, -90.0, 45.0, 0.0]))))
    assert cv.twist_alpha_star(jacobian, np.zeros(6), LIMITS)['alpha'] == np.inf


def test_alpha_star_scales_with_the_requested_rate(validator):
    """Halving the demanded twist must double the achievable scaling."""
    configuration = np.concatenate(
        ([1.0], np.deg2rad([0.0, -135.0, 90.0, -90.0, 45.0, 0.0])))
    jacobian = validator.compute_system_jacobian(configuration)
    twist = jacobian @ (0.1 * np.ones(7))

    full = cv.twist_alpha_star(jacobian, twist, LIMITS)['alpha']
    half = cv.twist_alpha_star(jacobian, 0.5 * twist, LIMITS)['alpha']
    assert half == pytest.approx(2.0 * full, rel=1e-4)


def test_task_twist_comes_from_the_targets_not_the_joint_path():
    """Deriving the twist from the joint velocity being evaluated is circular.

    v = J qdot is producible by that very J by construction, so alpha* could
    never fall below the reciprocal of the velocity usage and the test would
    be vacuous. The task twist depends only on the SE(3) targets, so it is
    identical whichever interpolant is chosen.
    """
    from scipy.spatial.transform import Rotation

    positions = np.array([[0.0, 0.0, 0.0], [0.2, 0.0, 0.0]])
    quaternions = np.stack([
        Rotation.identity().as_quat(),
        Rotation.from_euler('z', 30, degrees=True).as_quat(),
    ])
    twist = cv.task_twist([0.0, 0.1], positions, quaternions, 0, 0.1)

    np.testing.assert_allclose(twist[:3], [2.0, 0.0, 0.0], atol=1e-9)
    assert np.rad2deg(np.linalg.norm(twist[3:])) == pytest.approx(300.0, rel=1e-6)


def test_the_angular_twist_is_expressed_in_the_world_frame():
    """The Jacobian returns a world-frame twist, so this must match it.

    log(R_prev^T R_next) is expressed in the STARTING BODY frame, and pairing
    it with a world-frame translation gives a six-vector mixing two frames.
    A start orientation of identity hides the error completely, since the two
    frames coincide there, so this test starts elsewhere and rotates about an
    axis that moves under that start.
    """
    from scipy.spatial.transform import Rotation

    # Start rotated 90 deg about world X, so world Z maps to body -Y.
    start = Rotation.from_euler('x', 90, degrees=True)
    # Then rotate a further 30 deg about the WORLD Z axis.
    delta_world = Rotation.from_euler('z', 30, degrees=True)
    end = delta_world * start

    positions = np.zeros((2, 3))
    twist = cv.task_twist([0.0, 0.1], positions,
                          [start.as_quat(), end.as_quat()], 0, 0.1)

    # A world-frame angular velocity must point along world Z.
    axis = twist[3:] / np.linalg.norm(twist[3:])
    np.testing.assert_allclose(axis, [0.0, 0.0, 1.0], atol=1e-9)

    # The body-frame form would point along body Z, which is world -Y here.
    body = (Rotation.from_matrix(start.as_matrix().T @ end.as_matrix())
            .as_rotvec())
    body_axis = body / np.linalg.norm(body)
    assert not np.allclose(body_axis, [0.0, 0.0, 1.0], atol=1e-6)


def test_task_twist_is_sign_insensitive_to_the_quaternions():
    """q and -q are the same rotation, and the input carries sign flips."""
    from scipy.spatial.transform import Rotation

    positions = np.zeros((2, 3))
    first = Rotation.identity().as_quat()
    second = Rotation.from_euler('y', 20, degrees=True).as_quat()
    positive = cv.task_twist([0.0, 0.1], positions, [first, second], 0, 0.1)
    negative = cv.task_twist([0.0, 0.1], positions, [first, -second], 0, 0.1)
    np.testing.assert_allclose(positive, negative, atol=1e-9)


def test_conditioning_records_where_the_worst_value_occurs(validator,
                                                           smooth_path):
    """An infinite value at t=0 proves nothing about the interior.

    The legacy start posture is itself singular, so the first sample is
    guaranteed to report infinity whatever the trajectory does.
    """
    times = _times(len(smooth_path))
    dense_times, interpolator, _ = cv.interpolate(smooth_path, times, 100.0)
    report = cv.conditioning_and_twist(validator, interpolator, dense_times,
                                       LIMITS)
    for key in ('max_condition_at_time_s', 'interior_max_condition_number',
                'exceeds_only_at_start', 'seconds_above_condition_threshold'):
        assert key in report


def test_command_stream_jerk_exceeds_the_interpolant_derivative(smooth_path):
    """PCHIP is only C1, so acceleration jumps at every knot.

    The interpolant's own third derivative sees the within-segment polynomial
    and misses those jumps, so it is not a bound on what the controller
    receives. Measured on the first graph path: 61.6 analytic against 342.4
    from the command stream.
    """
    times = _times(len(smooth_path))
    dense_times, interpolator, dense_path = cv.interpolate(
        smooth_path, times, 200.0)
    analytic = cv.derivative_extremes(interpolator, dense_times)
    commanded = cv.command_stream_derivatives(dense_times, dense_path)
    assert np.max(commanded['jerk']) > np.max(analytic['jerk'])


def test_declared_higher_order_limits_are_starting_values_not_inherited():
    """Nothing upstream ever bounded acceleration or jerk, so these are to be
    measured against rather than trusted."""
    assert cv.ARM_ACCELERATION_LIMIT_RAD_S2 > 0
    assert cv.ARM_JERK_LIMIT_RAD_S3 > cv.ARM_ACCELERATION_LIMIT_RAD_S2
    assert cv.RAIL_ACCELERATION_LIMIT_M_S2 > 0


def test_an_unsolved_lp_makes_the_trajectory_indeterminate_not_passing(validator,
                                                                       smooth_path):
    """One unsolved sample among many passing ones must not read as a pass.

    An LP that did not solve says nothing about feasibility, so folding it
    into a boolean loses exactly the case worth knowing about.
    """
    times = _times(len(smooth_path))
    dense_times, interpolator, _ = cv.interpolate(smooth_path, times, 100.0)

    original = cv.twist_alpha_star
    calls = {'n': 0}

    def sometimes_failing(jacobian, twist, limits):
        calls['n'] += 1
        if calls['n'] == 3:
            return {'alpha': None, 'status': 'simulated failure',
                    'solved': False, 'equality_residual': None}
        return original(jacobian, twist, limits)

    cv.twist_alpha_star = sometimes_failing
    try:
        report = cv.conditioning_and_twist(
            validator, interpolator, dense_times, LIMITS,
            task_twists=[np.array([0.01, 0, 0, 0, 0, 0.01])] * len(times),
            waypoint_times=times)
    finally:
        cv.twist_alpha_star = original

    assert report['twist_unsolved_samples'] >= 1
    assert report['twist_status'] == 'indeterminate'
    assert report['twist_feasible'] is None


def test_twist_status_is_not_applicable_without_task_twists(validator,
                                                            smooth_path):
    """Absent a task, there is no twist to be feasible for."""
    times = _times(len(smooth_path))
    dense_times, interpolator, _ = cv.interpolate(smooth_path, times, 100.0)
    report = cv.conditioning_and_twist(validator, interpolator, dense_times,
                                       LIMITS)
    assert report['twist_status'] == 'not_applicable'


def test_conditioning_gates_the_overall_result(validator):
    """A trajectory through a singular configuration must not pass.

    The overall verdict previously ignored the conditioning threshold
    entirely, so a path could report passed while passing through a
    singularity.
    """
    singular = np.concatenate(([1.0], np.deg2rad([0.0, -135.0, 90.0, -90.0, 0.0, 0.0])))
    path = np.stack([singular + i * 1e-4 for i in range(4)])
    times = _times(4)
    poses = [validator.robot.fkine(q, end='tool0') for q in path]
    positions = np.stack([p.t for p in poses])
    quaternions = np.stack([np.roll(np.array(p.UnitQuaternion().A), -1)
                            for p in poses])

    report = cv.validate(validator, path, times, positions, quaternions,
                         LIMITS, rate_hz=100.0)
    assert report['conditioning']['max_condition_number'] > 50.0
    assert report['conditioning_ok'] is False
    assert report['passed'] is False


def test_recovery_mode_waives_conditioning_and_nothing_else(validator):
    """The exception exists so a robot parked singular can leave, not to
    weaken every approach."""
    singular = np.concatenate(([1.0], np.deg2rad([0.0, -135.0, 90.0, -90.0, 0.0, 0.0])))
    path = np.stack([singular + i * 1e-4 for i in range(4)])
    times = _times(4)
    poses = [validator.robot.fkine(q, end='tool0') for q in path]
    positions = np.stack([p.t for p in poses])
    quaternions = np.stack([np.roll(np.array(p.UnitQuaternion().A), -1)
                            for p in poses])

    waived = cv.validate(validator, path, times, positions, quaternions,
                         LIMITS, rate_hz=100.0, recovery_mode=True)
    assert waived['conditioning_ok'] is True

    # Velocity is still enforced: the same path at an impossible rate fails.
    fast = np.stack([singular + i * 0.5 for i in range(4)])
    report = cv.validate(validator, fast, times, positions, quaternions,
                         LIMITS, rate_hz=100.0, recovery_mode=True)
    assert report['limit_violations']
    assert report['passed'] is False
