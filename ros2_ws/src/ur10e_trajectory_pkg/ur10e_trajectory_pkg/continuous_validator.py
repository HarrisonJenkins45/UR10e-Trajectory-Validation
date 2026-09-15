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

Measurement on the graph path, world-frame task twist, split at the approach
boundary with the boundary knot in the trajectory. Reproduce with
python3 -m ur10e_trajectory_pkg.continuous_validator --graph <graph.json>
(200 Hz, every 5th sample):

    interval     limits           twist  min alpha*  at t    unsolved  max cond
    approach     either           fail       0.000    0.00         0       inf
    trajectory   URDF per-joint   pass      31.49     3.075        0      10.4
    trajectory   retired 2.0 cap  pass      20.48     2.025        0      10.4

The trajectory's twist demand is small next to what the arm can deliver: at
its tightest sample it could be scaled about 31 times before a joint limit
binds. Its angular demand peaks near 0.08 rad/s.

CORRECTION. Earlier revisions recorded 1.277, and 1.897 under URDF limits,
read as "the task demands 78% of the arm's authority". Both were a single
sample exactly ON the t = 2.0 s knot scored against the APPROACH twist
(0.58 m/s, 1.21 rad/s), because the interval lookup assigned a knot to the
interval ending there. segment_index now uses half-open intervals, and the
report names the time and interval of the binding sample so a misassignment
is visible.

The approach fails for a real reason rather than a numerical one: zero
unsolved LPs, and an alpha* of exactly 0 at the singular start, where the
commanded task twist is unachievable at any rate.
"""
import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.optimize import linprog
from scipy.spatial.transform import Rotation, Slerp

from ur10e_trajectory_pkg import motion_limits
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

# Acceleration and jerk limits come from motion_limits, with their provenance.
# A second table here once said 15 rad/s^2 while motion_limits said 800 deg/s^2
# (13.96), so Stage 7 timed approaches against one figure and this module
# validated against another.

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


def limit_violations(extremes, velocity_limits, acceleration_limits=None,
                     jerk_limits=None):
    """Per joint, which of the four limits the interpolant exceeds."""
    if acceleration_limits is None:
        acceleration_limits = motion_limits.acceleration_vector()
    if jerk_limits is None:
        jerk_limits = motion_limits.jerk_vector()

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


def self_clearance_along(validator, interpolator, times,
                         samples=COLLISION_SAMPLE_LADDER[-1], floor=None):
    """Smallest non-adjacent self-clearance on the interpolant.

    At the same instants as the finest collision check. A path grazing a
    self-contact fails here explicitly, instead of flickering in and out of
    the boolean collision test as sampling shifts.
    """
    floor = motion_limits.SELF_CLEARANCE_FLOOR_M if floor is None else floor
    worst = {'distance_m': np.inf, 'links': None}
    worst_time = None
    for instant in np.linspace(times[0], times[-1], samples):
        result = validator.self_clearance(interpolator(instant))
        if result['distance_m'] < worst['distance_m']:
            worst, worst_time = result, float(instant)
    return {'min_distance_m': float(worst['distance_m']), 'links': worst['links'],
            'at_time_s': worst_time, 'samples': int(samples), 'floor_m': floor,
            'passed': bool(worst['distance_m'] >= floor)}


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


def segment_index(times, instant):
    """The waypoint interval [t_i, t_i+1) containing an instant.

    Half-open, so a sample exactly ON a knot belongs to the interval that
    STARTS there. searchsorted's default left side assigns it to the interval
    that ends there instead, and at the approach boundary that scored the
    first trajectory sample against the approach's twist.
    """
    times = np.asarray(times, dtype=float)
    return int(np.clip(np.searchsorted(times, instant, side='right') - 1,
                       0, len(times) - 2))


def desired_pose_at(times, positions, quaternions, instant):
    """Desired pose between waypoints, interpolated on SE(3).

    Translation linearly, rotation by SLERP. Interpolating quaternion
    components directly would leave the unit sphere and describe a rotation
    the task never asked for.
    """
    times = np.asarray(times, dtype=float)
    index = segment_index(times, instant)
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

    Returns a dict, not a float, because solver failure and a feasible
    optimum of zero are different answers and collapsing them hides the one
    that matters. An earlier version returned 0.0 for both, which reported a
    rank-deficient Jacobian as physically infeasible when the LP had simply
    not solved.

    The equality residual is retained so a caller can tell an optimum that
    satisfies the constraints from one that merely claims to.
    """
    desired_twist = np.asarray(desired_twist, dtype=float)
    if np.linalg.norm(desired_twist) <= 1e-12:
        return {'alpha': np.inf, 'status': 'no_commanded_motion',
                'solved': True, 'equality_residual': 0.0}

    # Solve against a UNIT twist. With the raw twist the equality constraint
    # mixes the Jacobian's scale with the twist's, and on a near-singular
    # Jacobian HiGHS returned alpha = 0 while reporting 'optimal' -- a
    # degenerate point, not the optimum. Normalising conditions the problem;
    # alpha for the original twist is then beta / |v|.
    magnitude = float(np.linalg.norm(desired_twist))
    unit_twist = desired_twist / magnitude

    joints = jacobian.shape[1]
    objective = np.zeros(joints + 1)
    objective[-1] = -1.0
    equality = np.hstack([jacobian, -unit_twist.reshape(-1, 1)])
    bounds = [(-v, v) for v in velocity_limits] + [(0.0, None)]

    solution = linprog(objective, A_eq=equality,
                       b_eq=np.zeros(jacobian.shape[0]),
                       bounds=bounds, method='highs')
    if not solution.success:
        return {'alpha': None, 'status': str(solution.message),
                'solved': False, 'equality_residual': None}

    rates = solution.x[:joints]
    alpha = float(solution.x[-1]) / magnitude
    residual = float(np.max(np.abs(jacobian @ rates - alpha * desired_twist)))
    return {'alpha': alpha, 'status': 'optimal', 'solved': True,
            'equality_residual': residual,
            'bound_residual': float(np.max(np.abs(rates)
                                           - np.asarray(velocity_limits)))}


def task_twist(times, positions, quaternions, index, dt):
    """Desired twist from the SE(3) TARGET trajectory, not from a joint path.

    Translation by difference, rotation by log(R_i^T R_i+1) / dt. Deriving the
    twist from the joint velocity being evaluated makes the question circular:
    v = J qdot is producible by that very J by construction, even when it is
    rank deficient, so alpha* could never fall below the reciprocal of the
    velocity usage. Task feasibility has to ask about the motion the task
    demands, which is independent of whichever interpolant was chosen.
    """
    index = int(np.clip(index, 0, len(positions) - 2))
    linear = (np.asarray(positions[index + 1])
              - np.asarray(positions[index])) / dt

    start = Rotation.from_quat(quaternions[index]).as_matrix()
    end = Rotation.from_quat(quaternions[index + 1]).as_matrix()

    # WORLD-frame angular velocity: log(R_next R_prev^T), not
    # log(R_prev^T R_next). The latter is expressed in the STARTING BODY
    # frame, and pairing it with a world-frame translation produces a
    # six-vector mixing two frames, which then gets multiplied against a
    # world-frame Jacobian. The error is invisible whenever the start
    # orientation is identity, because the frames coincide there.
    angular = Rotation.from_matrix(end @ start.T).as_rotvec() / dt
    return np.concatenate((linear, angular))


def conditioning_and_twist(validator, interpolator, dense_times,
                           velocity_limits, condition_threshold=50.0,
                           task_twists=None, waypoint_times=None):
    """Arm conditioning and task-twist feasibility between waypoints.

    Records WHERE the worst conditioning occurs and for how long the threshold
    is exceeded. Without that an infinite value proves nothing about the
    interior, since the legacy start posture is itself singular and guarantees
    infinity at t = 0.

    task_twists, when given, are the twists the TASK demands, so alpha* asks a
    real question. Without them the check reports conditioning only and says
    so, rather than computing a tautology.
    """
    conditions = np.empty(len(dense_times))
    alphas = []
    alpha_times = []
    alpha_segments = []
    unsolved = 0
    for index, instant in enumerate(dense_times):
        configuration = interpolator(instant)
        singular = np.linalg.svd(
            validator.compute_arm_jacobian(configuration), compute_uv=False)
        conditions[index] = (singular[0] / singular[-1] if singular[-1] > 1e-12
                             else np.inf)
        if task_twists is None:
            continue
        segment = min(segment_index(waypoint_times, instant),
                      len(task_twists) - 1)
        result = twist_alpha_star(
            validator.compute_system_jacobian(configuration),
            task_twists[segment], velocity_limits)
        if result['solved'] and result['alpha'] is not None:
            alphas.append(result['alpha'])
            alpha_times.append(float(instant))
            alpha_segments.append(segment)
        else:
            unsolved += 1

    exceeded = conditions > condition_threshold
    interior = dense_times > dense_times[0]
    worst = int(np.argmax(np.nan_to_num(conditions, posinf=1e308)))
    report = {
        'max_condition_number': float(conditions[worst]),
        'max_condition_at_time_s': float(dense_times[worst]),
        'seconds_above_condition_threshold': float(
            np.sum(exceeded) * (dense_times[1] - dense_times[0])),
        'exceeds_only_at_start': bool(exceeded.any()
                                      and not (exceeded & interior).any()),
        'interior_max_condition_number': float(
            np.max(conditions[interior]) if interior.any() else 0.0),
        'twist_evaluated': task_twists is not None,
        'twist_unsolved_samples': unsolved,
    }
    # Four states, not a boolean. An LP that did not solve says nothing about
    # feasibility, and folding it into a pass was how one unsolved sample
    # among many passing ones produced True.
    if task_twists is None:
        report['min_alpha_star'] = None
        report['twist_status'] = 'not_applicable'
    elif unsolved:
        report['min_alpha_star'] = float(min(alphas)) if alphas else None
        report['twist_status'] = 'indeterminate'
    elif not alphas:
        report['min_alpha_star'] = None
        report['twist_status'] = 'indeterminate'
    else:
        report['min_alpha_star'] = float(min(alphas))
        report['twist_status'] = 'pass' if min(alphas) >= 1.0 else 'fail'
    # Where the binding sample is, and which task interval it was scored
    # against. A minimum sitting on a knot, scored against the wrong
    # interval, is exactly what these would have exposed.
    if alphas:
        worst_alpha = int(np.argmin(alphas))
        report['min_alpha_star_at_time_s'] = alpha_times[worst_alpha]
        report['min_alpha_star_segment'] = alpha_segments[worst_alpha]
    report['twist_feasible'] = (report['twist_status'] == 'pass'
                                if report['twist_status'] in ('pass', 'fail')
                                else None)
    return report


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
             velocity_limits=None, rate_hz=CONTROLLER_HZ,
             condition_threshold=50.0, recovery_mode=False):
    """Full continuous check of one command trajectory.

    velocity_limits defaults to what the validator enforces: the URDF's
    per-joint arm values and the rail's capped value.
    """
    if velocity_limits is None:
        velocity_limits = validator.velocity_limits
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
        'self_clearance': self_clearance_along(validator, interpolator,
                                               waypoint_times),
        'tracking': tracking_error(validator, interpolator, dense_times,
                                   waypoint_times, positions, quaternions),
        'conditioning': conditioning_and_twist(
            validator, interpolator, dense_times, velocity_limits,
            condition_threshold,
            task_twists=[
                task_twist(waypoint_times, positions, quaternions, i,
                           waypoint_times[i + 1] - waypoint_times[i])
                for i in range(len(waypoint_times) - 1)
            ],
            waypoint_times=waypoint_times),
        'velocity_headroom': velocity_headroom(interpolator, dense_times,
                                               velocity_limits),
    }
    # recovery_mode waives CONDITIONING only, and only while no Cartesian task
    # is active. Velocity, acceleration, jerk, collision and joint limits are
    # never waived: the exception exists so a robot parked at a singular pose
    # can leave it, not to weaken every approach.
    conditioning = report['conditioning']
    conditioning_ok = recovery_mode or (
        conditioning['max_condition_number'] <= condition_threshold)
    report['recovery_mode'] = recovery_mode
    report['conditioning_ok'] = bool(conditioning_ok)

    report['passed'] = bool(
        not report['limit_violations']
        and not report['position_limit_violations']
        and not report['collision']['collision_found']
        and report['self_clearance']['passed']
        and report['tracking']['within_tolerance']
        and conditioning_ok
        and conditioning['twist_status'] in ('pass', 'not_applicable')
    )
    return report


# --------------------------------------------------------------------------
# The two commands: warmup, then the task from its chosen start
# --------------------------------------------------------------------------

def validate_task_command(validator, path, positions, quaternions, dt,
                          velocity_limits=None, rate_hz=CONTROLLER_HZ,
                          condition_threshold=50.0):
    """Command 2: the task played from its first waypoint, with no transition.

    Waypoint times start at 0 and nothing is prepended: the warmup has brought
    the arm to path[0] at rest. The entry velocity and acceleration are
    recorded against the limits, since the warmup ENDS at rest and any entry
    state is a step at handover; a spin-up is what keeps them small.
    """
    velocity_limits = (validator.velocity_limits if velocity_limits is None
                       else np.asarray(velocity_limits, dtype=float))
    path = np.asarray(path, dtype=float)
    times = np.arange(len(path)) * float(dt)
    report = validate(validator, path, times, positions, quaternions,
                      velocity_limits=velocity_limits, rate_hz=rate_hz,
                      condition_threshold=condition_threshold)
    interpolator = PchipInterpolator(times, path, axis=0)
    entry_velocity = np.abs(interpolator.derivative(1)(0.0))
    entry_acceleration = np.abs(interpolator.derivative(2)(0.0))
    report.update(
        command='task',
        transition_s=0.0,
        entry_velocity=entry_velocity.tolist(),
        entry_velocity_ratio=float(np.max(entry_velocity / velocity_limits)),
        entry_acceleration=entry_acceleration.tolist(),
        entry_acceleration_ratio=float(np.max(
            entry_acceleration / motion_limits.acceleration_vector())),
    )
    return report


def validate_warmup(validator, warmup_result, velocity_limits=None):
    """Command 1: the warmup's controller-rate stream, checked sample by sample.

    Limits by finite difference of the stream the controller receives, joint
    limits, collision at EVERY sample (the stream is dense, so there is no
    resolution to converge), and rest at both ends. Arm conditioning is
    recorded but does not gate: no Cartesian task is active during a warmup,
    which is exactly the case recovery_mode exists for.
    """
    velocity_limits = (validator.velocity_limits if velocity_limits is None
                       else np.asarray(velocity_limits, dtype=float))
    report = {'command': 'warmup', 'status': warmup_result['status']}
    if warmup_result['status'] != 'ok':
        report['passed'] = False
        report['reason'] = warmup_result.get('reason')
        return report

    times = np.asarray(warmup_result['times'], dtype=float)
    positions = np.asarray(warmup_result['positions'], dtype=float)
    commanded = command_stream_derivatives(times, positions)
    step = times[1] - times[0]
    first_velocity = np.abs(positions[1] - positions[0]) / step
    last_velocity = np.abs(positions[-1] - positions[-2]) / step

    collisions = [float(times[i]) for i, q in enumerate(positions)
                  if validator.check_all_collisions(q)]
    clearances = [validator.self_clearance(q) for q in positions]
    closest = int(np.argmin([c['distance_m'] for c in clearances]))
    floor = motion_limits.SELF_CLEARANCE_FLOOR_M
    conditions = []
    for q in positions[::max(1, len(positions) // 400)]:
        singular = np.linalg.svd(validator.compute_arm_jacobian(q),
                                 compute_uv=False)
        conditions.append(singular[0] / singular[-1] if singular[-1] > 1e-12
                          else np.inf)
    report.update(
        samples=int(len(times)),
        duration_s=float(times[-1]),
        peak_command_stream={k: v.tolist() for k, v in commanded.items()},
        limit_violations=limit_violations(commanded, velocity_limits),
        position_limit_violations=position_limit_violations(validator, positions),
        collision_queries=int(len(positions)),
        collision_times_s=collisions[:10],
        collision_found=bool(collisions),
        self_clearance={'min_distance_m': float(clearances[closest]['distance_m']),
                        'links': clearances[closest]['links'],
                        'at_time_s': float(times[closest]),
                        'samples': int(len(positions)), 'floor_m': floor,
                        'passed': bool(clearances[closest]['distance_m'] >= floor)},
        # First and last sampled steps: a rest-to-rest move approaches zero
        # speed at both ends at the stream's resolution.
        start_speed_ratio=float(np.max(first_velocity / velocity_limits)),
        end_speed_ratio=float(np.max(last_velocity / velocity_limits)),
        max_arm_condition_number=float(np.max(conditions)),
        conditioning_gates=False,
    )
    report['passed'] = bool(not report['limit_violations']
                            and not report['position_limit_violations']
                            and not report['collision_found']
                            and report['self_clearance']['passed'])
    return report


# --------------------------------------------------------------------------
# Committed measurement, so recorded figures can be reproduced
# --------------------------------------------------------------------------

def graph_path_twist_margins(validator, path, waypoint_times, positions,
                             quaternions, velocity_limits, rate_hz, stride,
                             boundary_s):
    """alpha* and conditioning on each side of the approach boundary.

    The approach is [0, boundary) and the trajectory [boundary, end), with the
    boundary knot itself belonging to the trajectory: segment_index assigns it
    to the interval it starts.
    """
    dense_times, interpolator, _ = interpolate(path, waypoint_times, rate_hz)
    twists = [task_twist(waypoint_times, positions, quaternions, i,
                         waypoint_times[i + 1] - waypoint_times[i])
              for i in range(len(waypoint_times) - 1)]
    out = {}
    for label, selected in (
            ('approach', dense_times[dense_times < boundary_s]),
            ('trajectory', dense_times[dense_times >= boundary_s])):
        out[label] = conditioning_and_twist(
            validator, interpolator, selected[::stride], velocity_limits,
            task_twists=twists, waypoint_times=waypoint_times)
    return out


def _json_default(value):
    """NumPy scalars and arrays as their JSON equivalents, booleans kept."""
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(f'unserialisable {type(value).__name__}')


def main(argv=None):
    import argparse
    import json

    from ament_index_python.packages import get_package_share_directory

    from ur10e_trajectory_pkg.failure_census import _validator, load_trajectory
    from ur10e_trajectory_pkg.graph_planner import TRANSITION_SECONDS

    parser = argparse.ArgumentParser(
        description='Twist margin and conditioning along a graph path, split '
                    'at the approach boundary.')
    parser.add_argument('--graph', required=True,
                        help='graph_planner output containing a path')
    parser.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    parser.add_argument('--rate', type=float, default=200.0)
    parser.add_argument('--stride', type=int, default=5)
    parser.add_argument('--limits', choices=['urdf', 'retired_uniform_cap'],
                        default='urdf')
    parser.add_argument('--out', default=None)
    parser.add_argument('--command', choices=['graph_margins', 'task'],
                        default='graph_margins',
                        help='task: validate command 2 on a free-start graph '
                             'path, from its first waypoint with no transition')
    args = parser.parse_args(argv)

    validator = _validator(args.urdf, get_package_share_directory('ur_description'))
    with open(args.graph, encoding='utf-8') as handle:
        graph = json.load(handle)
    path = np.asarray(graph['path'], dtype=float)

    if args.command == 'task':
        if graph.get('start_mode') != 'free':
            parser.error('--command task needs a free-start graph artifact')
        spin_up = graph.get('spin_up')
        targets, task_quaternions, dt, _ = load_trajectory(
            None, graph['recorded_waypoints'], with_metadata=True,
            spin_up_s=None if spin_up is None else spin_up['requested_duration_s'])
        if len(targets) != len(path):
            parser.error(f'path has {len(path)} states but the trajectory has '
                         f'{len(targets)} samples')
        report = validate_task_command(validator, path, targets,
                                       task_quaternions, dt, rate_hz=args.rate)
        document = {'command': 'task', 'graph': args.graph, 'rate_hz': args.rate,
                    'spin_up': spin_up, 'report': report}
        conditioning = report['conditioning']
        print(f"task passed={report['passed']} limit_violations="
              f"{list(report['limit_violations'])} collision="
              f"{report['collision']['collision_found']} tracking="
              f"{report['tracking']['within_tolerance']} max_cond="
              f"{conditioning['max_condition_number']:.1f} twist="
              f"{conditioning['twist_status']} min_alpha*="
              f"{conditioning['min_alpha_star']} entry_velocity_ratio="
              f"{report['entry_velocity_ratio']:.4f} entry_acceleration_ratio="
              f"{report['entry_acceleration_ratio']:.4f}")
        if args.out:
            with open(args.out, 'w', encoding='utf-8') as handle:
                json.dump(document, handle, indent=1, default=_json_default)
        return 0 if report['passed'] else 1
    layers = len(path) - 1
    targets, quaternions, dt, _ = load_trajectory(None, layers, with_metadata=True)

    start = validator.robot.fkine(path[0], end=EE_LINK)
    positions = np.vstack([start.t, targets])
    quaternions = np.vstack([np.roll(np.array(start.UnitQuaternion().A), -1),
                             quaternions])
    times = np.concatenate(([0.0], TRANSITION_SECONDS + np.arange(layers) * dt))

    if args.limits == 'urdf':
        limits = validator.velocity_limits
    else:
        limits = np.concatenate((
            [validator.velocity_limits[0]],
            [motion_limits.RETIRED_UNIFORM_CAP.value] * 6))

    report = graph_path_twist_margins(validator, path, times, positions,
                                      quaternions, limits, args.rate,
                                      args.stride, TRANSITION_SECONDS)
    document = {'limits': args.limits, 'velocity_limits': limits.tolist(),
                'rate_hz': args.rate, 'stride': args.stride,
                'intervals': report}
    for label, entry in report.items():
        print(f"{label:10s} twist={entry['twist_status']:13s} "
              f"min_alpha*={entry['min_alpha_star']} "
              f"at t={entry.get('min_alpha_star_at_time_s')} "
              f"segment={entry.get('min_alpha_star_segment')} "
              f"unsolved={entry['twist_unsolved_samples']} "
              f"max_cond={entry['max_condition_number']:.1f} "
              f"at t={entry['max_condition_at_time_s']:.2f}")
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as handle:
            json.dump(document, handle, indent=1)
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
