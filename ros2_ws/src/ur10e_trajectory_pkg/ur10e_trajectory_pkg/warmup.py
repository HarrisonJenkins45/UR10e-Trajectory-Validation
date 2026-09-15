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

so T = max over joints of 15|D|/(8V) and sqrt(10|D| / (sqrt(3) A)), and no
dense check at controller rate can find a limit exceeded between samples.

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
from ur10e_trajectory_pkg import ready_pose_sweep as sweep

OK = 'ok'
NO_DIRECT_WARMUP = 'no_direct_warmup'
OUTSIDE_JOINT_LIMITS = 'endpoint_outside_joint_limits'
WARMUP_PROFILE = ('rest-to-rest quintic in joint space, minimum duration from '
                  'the closed-form peak velocity and acceleration')
CONTROLLER_HZ = 500.0
MINIMUM_DURATION_S = 0.2


def rest_to_rest_duration(home, start, velocity_limits, acceleration_limits,
                          lower=MINIMUM_DURATION_S):
    """Shortest rest-to-rest quintic duration, and the joint and limit binding it."""
    displacement = np.abs(np.asarray(start, float) - np.asarray(home, float))
    by_velocity = 15.0 * displacement / (8.0 * np.asarray(velocity_limits, float))
    by_acceleration = np.sqrt(10.0 * displacement
                              / (np.sqrt(3.0) * np.asarray(acceleration_limits, float)))
    candidates = np.maximum(by_velocity, by_acceleration)
    joint = int(np.argmax(candidates))
    duration = max(lower, float(candidates[joint]))
    kind = ('minimum_duration' if duration > candidates[joint]
            else 'velocity' if by_velocity[joint] >= by_acceleration[joint]
            else 'acceleration')
    return duration, joint, kind


def sample_rest_to_rest(home, start, duration, rate_hz=CONTROLLER_HZ):
    """Positions, velocities, accelerations at controller rate, ends included."""
    home, start = np.asarray(home, float), np.asarray(start, float)
    count = max(2, int(np.ceil(duration * rate_hz)) + 1)
    times = np.linspace(0.0, duration, count)
    s = times / duration
    shape = 10 * s**3 - 15 * s**4 + 6 * s**5
    rate = (30 * s**2 - 60 * s**3 + 30 * s**4) / duration
    curvature = (60 * s - 180 * s**2 + 120 * s**3) / duration**2
    delta = start - home
    return (times, home + np.outer(shape, delta), np.outer(rate, delta),
            np.outer(curvature, delta))


def plan_warmup(validator, home, start, velocity_limits=None,
                acceleration_limits=None, step_bounds=None,
                rate_hz=CONTROLLER_HZ):
    """Rest-to-rest warmup from home to start, or the reason there is none."""
    home = np.asarray(home, dtype=float)
    start = np.asarray(start, dtype=float)
    velocity_limits = (validator.velocity_limits if velocity_limits is None
                       else np.asarray(velocity_limits, dtype=float))
    acceleration_limits = (motion_limits.acceleration_vector()
                           if acceleration_limits is None
                           else np.asarray(acceleration_limits, dtype=float))
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
                                                  acceleration_limits)
    times, position, velocity, acceleration = sample_rest_to_rest(
        home, start, duration, rate_hz)
    statuses = motion_limits.limit_statuses(validator)
    stats = {}
    collides = sweep.approach_collides(validator, position, step_bounds,
                                       stats=stats)
    record.update(
        duration_s=duration,
        binding={'joint': validator.joint_names[joint], 'kind': kind,
                 'status': (statuses[kind][joint]
                            if kind in ('velocity', 'acceleration') else None)},
        peak_velocity_ratio=float(np.max(np.abs(velocity) / velocity_limits)),
        peak_acceleration_ratio=float(np.max(np.abs(acceleration)
                                             / acceleration_limits)),
        rate_hz=rate_hz, samples=int(len(times)), **stats)
    if collides:
        return dict(record, status=NO_DIRECT_WARMUP,
                    reason='the straight rest-to-rest move collides; a way '
                           'around the obstacle needs a planner')
    return dict(record, status=OK, times=times.tolist(),
                positions=position.tolist())


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

    from ur10e_trajectory_pkg.failure_census import _validator

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
