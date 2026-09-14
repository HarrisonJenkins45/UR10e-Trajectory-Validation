#!/usr/bin/env python3
"""Validate the command trajectory that is actually played back.

The graph proves safety only within its own discrete model: secant velocity
between waypoints, and three linearly interpolated collision samples per
edge. Playback does not follow that model. It follows a PCHIP interpolant,
which takes a different joint-space curve between the same endpoints and can
exceed the velocity the endpoint difference implies.

So the graph's verdict is a statement about its edges, not about the motion
the robot performs. This module checks the real thing, at controller
resolution:

  limits        position, velocity, acceleration and jerk, per joint
  collision     along the actual interpolant, with a resolution convergence
                check rather than a fixed sample count taken on faith
  tracking      reached pose against an SE(3) interpolation of the desired
                pose between waypoints, since a joint-space curve through two
                correct endpoints need not stay on the Cartesian path
  conditioning  arm singularity margin and TRUE commanded-twist feasibility
                between waypoints, the latter solved as a linear program over
                the redundant joint space rather than inferred from velocity
                headroom, which is a different quantity

The approach from q_start is checked separately. Assigning it two seconds
makes it admissible under a velocity limit; it does not make it a
dynamically smooth motion, and treating it as one waypoint interval would
hide that.

Nothing here is a graph cost. A trajectory that passes the graph and fails
this is a real finding about the discrete model, which is the point.
"""
import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.optimize import linprog
from scipy.spatial.transform import Rotation, Slerp

from ur10e_trajectory_pkg.configurations import ARM_SLICE, JOINT_NAMES
from ur10e_trajectory_pkg.pose_metrics import (
    IK_ORIENTATION_TOL_RAD,
    IK_POSITION_TOL_M,
    orientation_error_rad,
    quaternion_to_matrix,
)

EE_LINK = 'tool0'

# Controller rate. Checking at the waypoint rate would re-ask the question the
# graph already answered; the interpolant only differs from the secant
# BETWEEN waypoints, so the check has to look there.
CONTROLLER_HZ = 500.0

# Limits beyond the first-order ones the graph enforces. Declared here because
# nothing upstream has ever bounded them, so these are starting values to be
# measured against rather than inherited numbers.
ARM_ACCELERATION_LIMIT_RAD_S2 = 15.0
ARM_JERK_LIMIT_RAD_S3 = 500.0
RAIL_ACCELERATION_LIMIT_M_S2 = 5.0
RAIL_JERK_LIMIT_M_S3 = 100.0

# Sample counts for the collision convergence test. A single fixed count
# cannot distinguish "no collision" from "not sampled finely enough".
COLLISION_SAMPLE_LADDER = (200, 400, 800, 1600)


def interpolate(path, times, rate_hz=CONTROLLER_HZ):
    """PCHIP through the commanded configurations, at controller resolution.

    PCHIP because that is what the service plays back. A different
    interpolant would validate a trajectory nobody runs.
    """
    path = np.asarray(path, dtype=float)
    times = np.asarray(times, dtype=float)
    interpolator = PchipInterpolator(times, path, axis=0)
    dense_times = np.arange(times[0], times[-1], 1.0 / rate_hz)
    return dense_times, interpolator, interpolator(dense_times)


def derivative_extremes(interpolator, dense_times):
    """Peak absolute velocity, acceleration and jerk per joint.

    Taken from the interpolant's own derivatives rather than by differencing
    samples, so the answer does not depend on the sampling rate.
    """
    return {
        'velocity': np.max(np.abs(interpolator.derivative(1)(dense_times)), axis=0),
        'acceleration': np.max(np.abs(interpolator.derivative(2)(dense_times)), axis=0),
        'jerk': np.max(np.abs(interpolator.derivative(3)(dense_times)), axis=0),
    }


def limit_violations(extremes, velocity_limits):
    """Per joint, which of the four limits the interpolant exceeds."""
    acceleration_limits = np.array(
        [RAIL_ACCELERATION_LIMIT_M_S2] + [ARM_ACCELERATION_LIMIT_RAD_S2] * 6)
    jerk_limits = np.array([RAIL_JERK_LIMIT_M_S3] + [ARM_JERK_LIMIT_RAD_S3] * 6)

    out = {}
    for index, name in enumerate(JOINT_NAMES):
        exceeded = {
            'velocity': float(extremes['velocity'][index]) > velocity_limits[index],
            'acceleration': (float(extremes['acceleration'][index])
                             > acceleration_limits[index]),
            'jerk': float(extremes['jerk'][index]) > jerk_limits[index],
        }
        if any(exceeded.values()):
            out[name] = {
                key: {'peak': float(extremes[key][index]),
                      'limit': float([velocity_limits, acceleration_limits,
                                      jerk_limits][i][index])}
                for i, key in enumerate(('velocity', 'acceleration', 'jerk'))
                if exceeded[key]
            }
    return out


def position_limit_violations(validator, dense_path):
    lower, upper = validator.robot.qlim
    below = np.any(dense_path < lower - 1e-9, axis=0)
    above = np.any(dense_path > upper + 1e-9, axis=0)
    return [JOINT_NAMES[i] for i in range(len(JOINT_NAMES)) if below[i] or above[i]]


def collision_along_path(validator, interpolator, times, samples):
    """First colliding time on the interpolant, or None.

    Samples the interpolant rather than the straight line between waypoints,
    because those are different curves and only one of them is played.
    """
    for instant in np.linspace(times[0], times[-1], samples):
        if validator.check_all_collisions(interpolator(instant)):
            return float(instant)
    return None


def collision_convergence(validator, interpolator, times,
                          ladder=COLLISION_SAMPLE_LADDER):
    """Re-check at rising resolutions until the verdict stops changing.

    A clean result at one sample count means nothing on its own: a thin
    collision can hide between samples. Agreement across a doubling ladder is
    weak evidence of convergence, and a verdict that changes partway up is
    strong evidence the coarse answer was wrong.
    """
    verdicts = []
    for samples in ladder:
        verdicts.append({'samples': samples,
                         'first_collision_time':
                             collision_along_path(validator, interpolator,
                                                  times, samples)})
    hit = [v for v in verdicts if v['first_collision_time'] is not None]
    return {
        'ladder': verdicts,
        'collision_found': bool(hit),
        'converged': len({v['first_collision_time'] is None for v in verdicts}) == 1,
    }


def desired_pose_at(times, positions, quaternions, instant):
    """Desired pose between waypoints, interpolated on SE(3).

    Translation linearly, rotation by SLERP. Interpolating quaternion
    components directly would leave the unit sphere and describe a rotation
    the task never asked for.
    """
    times = np.asarray(times, dtype=float)
    index = int(np.clip(np.searchsorted(times, instant) - 1, 0, len(times) - 2))
    span = times[index + 1] - times[index]
    fraction = 0.0 if span <= 0 else (instant - times[index]) / span

    position = ((1.0 - fraction) * np.asarray(positions[index])
                + fraction * np.asarray(positions[index + 1]))
    rotations = Rotation.from_quat([quaternions[index], quaternions[index + 1]])
    slerp = Slerp([0.0, 1.0], rotations)
    return position, slerp([fraction])[0].as_matrix()


def tracking_error(validator, interpolator, dense_times, waypoint_times,
                   positions, quaternions):
    """Worst pose error between waypoints, not merely at them.

    A joint-space curve through two correct endpoints need not stay on the
    Cartesian path between them, and nothing so far has looked.
    """
    worst_position = worst_orientation = 0.0
    worst_time = None
    for instant in dense_times:
        reached = validator.robot.fkine(interpolator(instant), end=EE_LINK)
        want_position, want_rotation = desired_pose_at(
            waypoint_times, positions, quaternions, instant)
        position_error = float(np.linalg.norm(reached.t - want_position))
        angle_error = orientation_error_rad(reached.R, want_rotation)
        if position_error > worst_position:
            worst_position, worst_time = position_error, float(instant)
        worst_orientation = max(worst_orientation, angle_error)
    return {
        'max_position_error_m': worst_position,
        'max_orientation_error_rad': worst_orientation,
        'at_time_s': worst_time,
        'within_tolerance': (worst_position <= IK_POSITION_TOL_M
                             and worst_orientation <= IK_ORIENTATION_TOL_RAD),
    }


def velocity_headroom(interpolator, dense_times, velocity_limits):
    """Fraction of the velocity budget the command stream actually uses.

    Named for what it measures. An earlier version called its reciprocal
    alpha* and described it as commanded-twist feasibility, which it is not:
    it never touches the Jacobian or a desired Cartesian twist, so it can only
    ever report joint velocity headroom. The two answer different questions
    and can disagree, because a configuration can have ample joint headroom
    and still be unable to produce a particular twist direction.
    """
    rates = interpolator.derivative(1)(dense_times)
    usage = np.max(np.abs(rates) / velocity_limits, axis=1)
    worst = int(np.argmax(usage))
    return {
        'max_budget_used': float(usage[worst]),
        'at_time_s': float(dense_times[worst]),
        'within_limits': bool(usage[worst] <= 1.0),
    }


def twist_alpha_star(jacobian, desired_twist, velocity_limits):
    """Largest scaling of a commanded twist a bounded joint velocity can make.

        max alpha  s.t.  J qdot = alpha * v,  |qdot| <= qdot_max

    A genuine linear program over the redundant joint space, not a headroom
    ratio. alpha >= 1 means the twist is achievable at this configuration;
    below 1 it cannot be tracked at the requested rate, whatever the condition
    number says.
    """
    desired_twist = np.asarray(desired_twist, dtype=float)
    if np.linalg.norm(desired_twist) <= 1e-12:
        return np.inf                       # no motion cannot be infeasible

    joints = jacobian.shape[1]
    # Variables [qdot (n), alpha]; maximise alpha by minimising -alpha.
    objective = np.zeros(joints + 1)
    objective[-1] = -1.0
    equality = np.hstack([jacobian, -desired_twist.reshape(-1, 1)])
    bounds = [(-v, v) for v in velocity_limits] + [(0.0, None)]

    solution = linprog(objective, A_eq=equality, b_eq=np.zeros(jacobian.shape[0]),
                       bounds=bounds, method='highs')
    return float(solution.x[-1]) if solution.success else 0.0


def conditioning_and_twist(validator, interpolator, dense_times,
                           velocity_limits, condition_threshold=50.0):
    """Arm conditioning and true twist feasibility between waypoints.

    Records WHERE the worst conditioning occurs and for how long the threshold
    is exceeded. Without that, an infinite value proves nothing about the
    interior: the legacy start posture is itself singular, so t = 0 is
    guaranteed to report infinity whatever the trajectory does.
    """
    conditions = np.empty(len(dense_times))
    alphas = np.empty(len(dense_times))
    for index, instant in enumerate(dense_times):
        configuration = interpolator(instant)
        singular = np.linalg.svd(
            validator.compute_arm_jacobian(configuration), compute_uv=False)
        conditions[index] = (singular[0] / singular[-1] if singular[-1] > 1e-12
                             else np.inf)
        twist = (validator.compute_system_jacobian(configuration)
                 @ interpolator.derivative(1)(instant))
        alphas[index] = twist_alpha_star(
            validator.compute_system_jacobian(configuration), twist,
            velocity_limits)

    exceeded = conditions > condition_threshold
    interior = dense_times > dense_times[0]
    worst = int(np.argmax(np.nan_to_num(conditions, posinf=1e308)))
    worst_alpha = int(np.argmin(alphas))
    return {
        'max_condition_number': float(conditions[worst]),
        'max_condition_at_time_s': float(dense_times[worst]),
        'seconds_above_condition_threshold': float(
            np.sum(exceeded) * (dense_times[1] - dense_times[0])),
        'exceeds_only_at_start': bool(exceeded.any()
                                      and not (exceeded & interior).any()),
        'interior_max_condition_number': float(
            np.max(conditions[interior]) if interior.any() else 0.0),
        'min_alpha_star': float(alphas[worst_alpha]),
        'min_alpha_star_at_time_s': float(dense_times[worst_alpha]),
        'twist_feasible': bool(alphas.min() >= 1.0),
    }


def command_stream_derivatives(dense_times, dense_path):
    """Acceleration and jerk by finite difference of the COMMAND STREAM.

    PCHIP is only C1, so acceleration can jump at every knot. Sampling the
    interpolant's own third derivative sees the within-segment polynomial jerk
    and misses those discontinuities entirely, so it is not a bound. The
    controller receives the sampled stream, and this is what that stream
    actually does.
    """
    step = dense_times[1] - dense_times[0]
    velocity = np.diff(dense_path, axis=0) / step
    acceleration = np.diff(velocity, axis=0) / step
    jerk = np.diff(acceleration, axis=0) / step
    return {
        'velocity': np.max(np.abs(velocity), axis=0),
        'acceleration': np.max(np.abs(acceleration), axis=0),
        'jerk': np.max(np.abs(jerk), axis=0),
    }


def validate(validator, path, waypoint_times, positions, quaternions,
             velocity_limits, rate_hz=CONTROLLER_HZ, condition_threshold=50.0):
    """Full continuous check of one command trajectory."""
    dense_times, interpolator, dense_path = interpolate(
        path, waypoint_times, rate_hz)

    # Two derivative views, deliberately both. The interpolant's own
    # derivatives describe the within-segment polynomial; the finite
    # differences describe the sampled stream the controller receives, which
    # includes the acceleration jumps PCHIP leaves at every knot.
    analytic = derivative_extremes(interpolator, dense_times)
    commanded = command_stream_derivatives(dense_times, dense_path)

    report = {
        'controller_hz': rate_hz,
        'samples': int(len(dense_times)),
        'peak_analytic': {k: v.tolist() for k, v in analytic.items()},
        'peak_command_stream': {k: v.tolist() for k, v in commanded.items()},
        'limit_violations': limit_violations(commanded, velocity_limits),
        'position_limit_violations': position_limit_violations(
            validator, dense_path),
        'collision': collision_convergence(validator, interpolator,
                                           waypoint_times),
        'tracking': tracking_error(validator, interpolator, dense_times,
                                   waypoint_times, positions, quaternions),
        'conditioning': conditioning_and_twist(
            validator, interpolator, dense_times, velocity_limits,
            condition_threshold),
        'velocity_headroom': velocity_headroom(interpolator, dense_times,
                                               velocity_limits),
    }
    report['passed'] = bool(
        not report['limit_violations']
        and not report['position_limit_violations']
        and not report['collision']['collision_found']
        and report['tracking']['within_tolerance']
        and report['conditioning']['twist_feasible']
    )
    return report
