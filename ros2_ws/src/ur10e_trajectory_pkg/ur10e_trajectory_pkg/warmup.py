#!/usr/bin/env python3
"""Warmup command: bring the arm from home to the task's chosen start, at rest.

The task trajectory now starts from rest (a spin-up) at a start the graph
planner chooses, so reaching it is a separate, simple command: a rest-to-rest
move in joint space, ending EXACTLY at that lifted configuration, since the
task path is continuous from it.

The move is a quintic with zero velocity and acceleration at both ends, which
is monotone in every joint, so it cannot leave the joint limits if its
endpoints are inside them. Its minimum duration has a closed form rather than
a sampled search:

    peak |velocity|     = 15 |D| / (8 T)          at T / 2
    peak |acceleration| = 10 |D| / (sqrt(3) T^2)  at T (3 +/- sqrt(3)) / 6
    peak |jerk|         = 60 |D| / T^3            at 0 and T

so T = max over joints of 15|D|/(8V), sqrt(10|D| / (sqrt(3) A)) and
cbrt(60|D|/J), and no dense check at controller rate can find a limit
exceeded between samples. The straight move must also keep the
self-clearance floor.

The straight move is collision-checked at the per-joint resolution bound. If
it is blocked the command reports NO_DIRECT_WARMUP rather than inventing a
detour: finding a way around an obstacle is a planning problem this does not
solve.
"""
import argparse
import json
import sys

import numpy as np

from ur10e_trajectory_pkg import motion_limits
from ur10e_trajectory_pkg import robot_checks
from ur10e_trajectory_pkg.joint_motion import CONTROLLER_HZ, sample_rest_to_rest

OK = 'ok'
NO_DIRECT_WARMUP = 'no_direct_warmup'
DIRECT_BELOW_SELF_CLEARANCE = 'direct_warmup_below_self_clearance'
NO_TWO_SEGMENT_WARMUP = 'no_two_segment_warmup'

# The robot stands still at the via pose between the two segments. Both
# segments already start and end at rest, so the dwell adds no motion; it
# exists so the concatenation is a command the controller can be handed as
# one piece, with an unambiguous rest point where the first move ends.
WARMUP_VIA_DWELL_S = 0.5
OUTSIDE_JOINT_LIMITS = 'endpoint_outside_joint_limits'
WARMUP_PROFILE = ('rest-to-rest quintic in joint space, minimum duration from '
                  'the closed-form peak velocity and acceleration')
MINIMUM_DURATION_S = 0.2


def rest_to_rest_duration(home, start, velocity_limits, acceleration_limits,
                          lower=MINIMUM_DURATION_S, jerk_limits=None):
    """Shortest rest-to-rest quintic duration, and the joint and limit binding it.

    jerk_limits, when given, bounds the quintic's peak jerk 60|D|/T^3, reached
    at both ends. Continuous validation gates on jerk, so a duration sized on
    velocity and acceleration alone planned a 0.48 s rail move as ok that then
    failed validation at 104 against the rail's 100 m/s^3.
    """
    displacement = np.abs(np.asarray(start, float) - np.asarray(home, float))
    by_velocity = 15.0 * displacement / (8.0 * np.asarray(velocity_limits, float))
    by_acceleration = np.sqrt(10.0 * displacement
                              / (np.sqrt(3.0) * np.asarray(acceleration_limits, float)))
    by_jerk = (np.zeros_like(displacement) if jerk_limits is None
               else np.cbrt(60.0 * displacement / np.asarray(jerk_limits, float)))
    stacked = np.stack([by_velocity, by_acceleration, by_jerk])
    candidates = stacked.max(axis=0)
    joint = int(np.argmax(candidates))
    duration = max(lower, float(candidates[joint]))
    kind = ('minimum_duration' if duration > candidates[joint]
            else ('velocity', 'acceleration', 'jerk')[int(np.argmax(stacked[:, joint]))])
    return duration, joint, kind


def plan_warmup(validator, home, start, velocity_limits=None,
                acceleration_limits=None, step_bounds=None,
                rate_hz=CONTROLLER_HZ, jerk_limits=None):
    """Rest-to-rest warmup from home to start, or the reason there is none."""
    home = np.asarray(home, dtype=float)
    start = np.asarray(start, dtype=float)
    velocity_limits = (validator.velocity_limits if velocity_limits is None
                       else np.asarray(velocity_limits, dtype=float))
    acceleration_limits = (motion_limits.acceleration_vector()
                           if acceleration_limits is None
                           else np.asarray(acceleration_limits, dtype=float))
    jerk_limits = (motion_limits.jerk_vector() if jerk_limits is None
                   else np.asarray(jerk_limits, dtype=float))
    record = {'home': home.tolist(), 'start': start.tolist(),
              'profile': WARMUP_PROFILE,
              'velocity_limits': velocity_limits.tolist(),
              'acceleration_limits': acceleration_limits.tolist()}

    lower, upper = validator.robot.qlim
    for name, point in (('home', home), ('start', start)):
        if np.any(point < lower - 1e-9) or np.any(point > upper + 1e-9):
            return dict(record, status=OUTSIDE_JOINT_LIMITS,
                        reason=f'{name} is outside the joint limits')

    duration, joint, kind = rest_to_rest_duration(home, start, velocity_limits,
                                                  acceleration_limits,
                                                  jerk_limits=jerk_limits)
    times, position, velocity, acceleration = sample_rest_to_rest(
        home, start, duration, rate_hz)
    statuses = motion_limits.limit_statuses(validator)
    record['jerk_limits'] = jerk_limits.tolist()
    record['peak_jerk_ratio'] = float(np.max(
        60.0 * np.abs(start - home) / duration ** 3 / jerk_limits))
    stats = {}
    collides = robot_checks.approach_collides(validator, position, step_bounds,
                                       stats=stats)
    record.update(
        duration_s=duration,
        binding={'joint': validator.joint_names[joint], 'kind': kind,
                 'status': (statuses[kind][joint]
                            if kind in statuses else None)},
        peak_velocity_ratio=float(np.max(np.abs(velocity) / velocity_limits)),
        peak_acceleration_ratio=float(np.max(np.abs(acceleration)
                                             / acceleration_limits)),
        rate_hz=rate_hz, samples=int(len(times)), **stats)
    if collides:
        return dict(record, status=NO_DIRECT_WARMUP,
                    reason='the straight rest-to-rest move collides; a way '
                           'around the obstacle needs a planner')
    # The self-clearance floor too, not collision alone. Without it a move
    # passing 6.5 mm from itself planned as ok, was ranked first among the
    # start windings, and only then failed validation; a route through a via
    # pose was never tried for that winding.
    clearance = robot_checks.approach_self_clearance(validator, position)
    floor = motion_limits.SELF_CLEARANCE_FLOOR_M
    record.update(min_self_clearance_m=clearance['min_distance_m'],
                  self_clearance_links=clearance['links'],
                  self_clearance_queries=clearance['samples'])
    if clearance['min_distance_m'] < floor:
        return dict(record, status=DIRECT_BELOW_SELF_CLEARANCE,
                    reason=f"the straight move passes "
                           f"{clearance['min_distance_m'] * 1000:.1f} mm from "
                           f"itself, below the {floor * 1000:.0f} mm floor")
    return dict(record, status=OK, times=times.tolist(),
                positions=position.tolist())


def _assemble_route(segments, points, dwell_s, rate_hz):
    """A route document from legs already planned, so none is planned twice."""
    return {'status': OK,
            'segments': [dict(leg, index=index,
                              begins_at=np.asarray(points[index], dtype=float).tolist(),
                              ends_at=np.asarray(points[index + 1], dtype=float).tolist())
                         for index, leg in enumerate(segments)],
            'rest_points': [np.asarray(point, dtype=float).tolist() for point in points],
            'dwell_s': float(dwell_s),
            'total_duration_s': float(sum(leg['duration_s'] for leg in segments)
                                      + dwell_s * (len(segments) - 1)),
            'rate_hz': rate_hz}


def route_from_rest_points(validator, rest_points, rate_hz=CONTROLLER_HZ,
                           dwell_s=WARMUP_VIA_DWELL_S, velocity_limits=None,
                           acceleration_limits=None):
    """A warmup route through the given rest points, one segment per leg.

    The caller names where the robot should come to rest; each leg is planned
    here rather than taken from the caller, so a route that arrives over the
    wire is still the server's own motion. One rest point pair is the common
    case, two legs is a route around an obstacle, and more needs no new
    format.
    """
    points = [np.asarray(point, dtype=float) for point in rest_points]
    if len(points) < 2:
        return {'status': OUTSIDE_JOINT_LIMITS,
                'reason': 'a route needs at least a start and an end'}
    segments = []
    for index in range(len(points) - 1):
        leg = plan_warmup(validator, points[index], points[index + 1],
                          velocity_limits, acceleration_limits, rate_hz=rate_hz)
        if leg['status'] != OK:
            return {'status': leg['status'], 'reason': leg.get('reason'),
                    'failed_segment': index, 'segments': segments,
                    'rest_points': [point.tolist() for point in points]}
        segments.append(leg)
    return _assemble_route(segments, points, dwell_s, rate_hz)


def plan_route(validator, home, start, via_poses=None, rate_hz=CONTROLLER_HZ,
               dwell_s=WARMUP_VIA_DWELL_S, velocity_limits=None,
               acceleration_limits=None):
    """The route from home to start: straight if it can be, around if it must.

    One segment when the direct move plans; two through a screened via pose
    when it does not and via_poses are offered. The result is the same shape
    either way, so nothing downstream has to know which case it got.
    """
    # Each leg is planned once. plan_warmup sweeps the move for collision,
    # so re-planning a leg already planned pays that sweep again for an
    # identical answer.
    direct = plan_warmup(validator, home, start, velocity_limits,
                         acceleration_limits, rate_hz=rate_hz)
    if direct['status'] == OK:
        return dict(_assemble_route([direct], [home, start], dwell_s, rate_hz),
                    route_kind='direct')
    if via_poses is None:
        return dict(direct, route_kind='direct', failed_segment=0)
    two = plan_two_segment_warmup(validator, home, start, via_poses,
                                  rate_hz=rate_hz, dwell_s=dwell_s,
                                  velocity_limits=velocity_limits,
                                  acceleration_limits=acceleration_limits)
    if two['status'] != OK:
        return dict(two, route_kind='via')
    return dict(_assemble_route([two['first'], two['second']],
                                [home, two['via'], start], dwell_s, rate_hz),
                route_kind='via', via=two['via'], via_index=two.get('via_index'))


def concatenate_segments(first, second, dwell_s=WARMUP_VIA_DWELL_S,
                         rate_hz=CONTROLLER_HZ):
    """Two warmup segments and the rest dwell between them, as one command.

    Both segments end and begin at rest, so the dwell holds the via pose
    exactly and the joined stream has no step in velocity at the seam.
    """
    step = 1.0 / rate_hz
    first_times = np.asarray(first['times'], dtype=float)
    first_positions = np.asarray(first['positions'], dtype=float)
    second_times = np.asarray(second['times'], dtype=float)
    second_positions = np.asarray(second['positions'], dtype=float)

    dwell_samples = max(int(round(dwell_s * rate_hz)), 1)
    dwell_times = first_times[-1] + step * np.arange(1, dwell_samples + 1)
    dwell_positions = np.repeat(first_positions[-1][None, :], dwell_samples, axis=0)
    shifted = dwell_times[-1] + step + (second_times - second_times[0])

    return {'status': OK,
            'times': np.concatenate((first_times, dwell_times, shifted)).tolist(),
            'positions': np.concatenate((first_positions, dwell_positions,
                                         second_positions)).tolist(),
            'rate_hz': rate_hz, 'dwell_s': float(dwell_samples * step),
            'via': first_positions[-1].tolist()}


def plan_two_segment_warmup(validator, home, start, via_poses,
                            rate_hz=CONTROLLER_HZ, dwell_s=WARMUP_VIA_DWELL_S,
                            velocity_limits=None, acceleration_limits=None,
                            first_match=False):
    """Home to start via a resting intermediate pose, when the straight move collides.

    A blocked direct quintic means the straight line hits something, not that
    the start is out of reach. Each via pose is one the home screening already
    admitted, so it clears the static gates including self-clearance. Both
    segments must plan and pass warmup validation, and the concatenation is
    validated again as the single command it will be sent as.

    Ranked by total duration. first_match stops at the first via that passes,
    for callers testing many starts rather than choosing one route.
    """
    from ur10e_trajectory_pkg import continuous_validator

    home = np.asarray(home, dtype=float)
    start = np.asarray(start, dtype=float)
    considered, best = [], None
    for index, via in enumerate(via_poses):
        via = np.asarray(via, dtype=float)
        note = {'via_index': index, 'via': via.tolist()}
        first = plan_warmup(validator, home, via, velocity_limits,
                            acceleration_limits, rate_hz=rate_hz)
        if first['status'] != OK:
            considered.append(dict(note, rejected='first_segment_' + first['status']))
            continue
        second = plan_warmup(validator, via, start, velocity_limits,
                             acceleration_limits, rate_hz=rate_hz)
        if second['status'] != OK:
            considered.append(dict(note, rejected='second_segment_' + second['status']))
            continue
        reports = [continuous_validator.validate_warmup(validator, first),
                   continuous_validator.validate_warmup(validator, second)]
        if not all(report['passed'] for report in reports):
            considered.append(dict(note, rejected='segment_validation'))
            continue
        combined = concatenate_segments(first, second, dwell_s, rate_hz)
        joined = continuous_validator.validate_warmup(validator, combined)
        if not joined['passed']:
            considered.append(dict(note, rejected='joined_validation'))
            continue
        total = float(first['duration_s'] + combined['dwell_s'] + second['duration_s'])
        considered.append(dict(note, rejected=None, total_duration_s=total))
        candidate = dict(note, status=OK, total_duration_s=total,
                         first_duration_s=first['duration_s'],
                         second_duration_s=second['duration_s'],
                         dwell_s=combined['dwell_s'],
                         min_self_clearance_m=min(
                             report['self_clearance']['min_distance_m']
                             for report in reports + [joined]),
                         first=first, second=second, joined=combined)
        if best is None or total < best['total_duration_s']:
            best = candidate
        if first_match:
            break
    if best is None:
        return {'status': NO_TWO_SEGMENT_WARMUP, 'home': home.tolist(),
                'start': start.tolist(), 'considered': considered,
                'reason': 'no screened via pose gave two validated segments'}
    return dict(best, considered=considered, home=home.tolist(),
                start=start.tolist())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--home', required=True,
                        help='JSON list of 7 joint values, or a file holding one')
    parser.add_argument('--graph', required=True,
                        help='graph_planner output; its chosen_start is the target')
    parser.add_argument('--urdf', default='/root/ros2_ws/ur10e.urdf')
    parser.add_argument('--out', default='warmup.json')
    args = parser.parse_args(argv)

    from ament_index_python.packages import get_package_share_directory

    from ur10e_trajectory_pkg.planning_runtime import make_validator as _validator

    try:
        home = json.loads(args.home)
    except json.JSONDecodeError:
        with open(args.home, encoding='utf-8') as handle:
            home = json.load(handle)
    with open(args.graph, encoding='utf-8') as handle:
        start = json.load(handle)['chosen_start']
    validator = _validator(args.urdf, get_package_share_directory('ur_description'))
    result = plan_warmup(validator, home, start)
    with open(args.out, 'w', encoding='utf-8') as handle:
        json.dump(result, handle, indent=1)
    print({k: v for k, v in result.items() if k not in ('times', 'positions')})
    return 0 if result['status'] == OK else 1


if __name__ == '__main__':
    sys.exit(main())
