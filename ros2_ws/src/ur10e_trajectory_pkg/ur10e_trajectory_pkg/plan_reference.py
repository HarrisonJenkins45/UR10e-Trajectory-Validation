"""Time-parameterized joint reference for executing a certified plan.

Hardware execution follows the same curves the validator checked: the
rest-to-rest quintic warmup legs, with the plan's dwell between legs, then the
PCHIP task curve through q_path at the recording step. Nothing here invents a
different path.

A reference may be slowed uniformly by time_scale <= 1. That keeps the
geometric joint path and scales every velocity by time_scale and every
acceleration by its square, so a slowed plan stays inside the limits the full-
speed plan was certified against. It does not reproduce the recorded tumble
rate, which is acceptable for a demonstration but not for an experiment.
"""

import numpy as np

from ur10e_trajectory_pkg import joint_motion
from ur10e_trajectory_pkg.configurations import NUM_JOINTS

# The quintic's peak speed is this multiple of its average speed.
QUINTIC_PEAK = 15.0 / 8.0


class Segment:
    """One piece of the reference: a name, a duration, and q(t), qd(t)."""

    def __init__(self, name, duration, evaluate):
        if not np.isfinite(duration) or duration < 0:
            raise ValueError(f'segment {name} has invalid duration {duration}')
        self.name = name
        self.duration = float(duration)
        self._evaluate = evaluate

    def evaluate(self, t):
        return self._evaluate(min(max(t, 0.0), self.duration))


def _configuration(values, name):
    values = np.asarray(values, dtype=float)
    if values.shape != (NUM_JOINTS,) or not np.all(np.isfinite(values)):
        raise ValueError(f'{name} must be {NUM_JOINTS} finite joint values')
    return values


def quintic_segment(name, start, end, duration):
    """Rest-to-rest quintic, the same profile as joint_motion.sample_rest_to_rest."""
    start = _configuration(start, f'{name} start')
    end = _configuration(end, f'{name} end')
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError(f'{name} needs a positive duration')
    delta = end - start

    def evaluate(t):
        s = t / duration
        shape = 10 * s**3 - 15 * s**4 + 6 * s**5
        rate = (30 * s**2 - 60 * s**3 + 30 * s**4) / duration
        return start + shape * delta, rate * delta

    return Segment(name, duration, evaluate)


def hold_segment(name, configuration, duration):
    configuration = _configuration(configuration, name)
    zero = np.zeros(NUM_JOINTS)
    return Segment(name, duration, lambda t: (configuration.copy(), zero.copy()))


def task_segment(path, dt):
    path = np.asarray(path, dtype=float)
    times = joint_motion.task_waypoint_times(len(path), dt)
    curve = joint_motion.task_curve(path, times)
    rate = curve.derivative(1)

    def evaluate(t):
        return np.asarray(curve(t), dtype=float), np.asarray(rate(t), dtype=float)

    return Segment('task', times[-1], evaluate)


def _route_segments(rests, durations, dwell, label):
    segments = []
    for index, duration in enumerate(durations):
        if index and dwell > 0:
            segments.append(hold_segment('via dwell', rests[index], dwell))
        segments.append(quintic_segment(
            f'{label} {index + 1}/{len(durations)}', rests[index],
            rests[index + 1], duration))
    return segments


def _route(plan):
    route = plan['warmup_route']
    return (np.asarray(route['rest_points'], dtype=float),
            np.asarray(route['segment_durations_s'], dtype=float),
            float(route['dwell_s']))


def plan_segments(plan, dt, settle_s=1.0, include_task=True):
    """Warmup legs (with the certified dwell), a settle at rest, then the task.

    The task starts from rest (the plan's spin-up), so pausing at its start
    configuration adds no certified motion; it only gives the operator and the
    hardware a clean handover between the two phases.
    """
    rests, durations, dwell = _route(plan)
    segments = _route_segments(rests, durations, dwell, 'warmup')
    if include_task:
        if settle_s > 0:
            segments.append(hold_segment('settle', rests[-1], settle_s))
        segments.append(task_segment(plan['q_path'], dt))
    return segments


def task_only_segments(plan, dt, settle_s=1.0):
    """A settle at the task start, then the certified task (no warmup)."""
    task_start = np.asarray(plan['q_path'][0], dtype=float)
    segments = [hold_segment('settle', task_start, settle_s)] if settle_s > 0 else []
    return segments + [task_segment(plan['q_path'], dt)]


def at_task_start(plan, measured, tolerance, rail_realign_limit=0.05):
    """True when the arm is at the task start and the rail close enough to re-align."""
    measured = _configuration(measured, 'measured pose')
    task_start = np.asarray(plan['q_path'][0], dtype=float)
    tolerance = np.asarray(tolerance, dtype=float)
    return bool(np.all(np.abs(measured[1:] - task_start[1:]) <= tolerance[1:]) and
                abs(measured[0] - task_start[0]) <= rail_realign_limit)


def reverse_warmup_segments(plan):
    """The certified warmup path traversed backwards: task start to home.

    Same rest points, legs, durations and dwell, so the same swept geometry
    and the same speed and acceleration magnitudes as the certified warmup.
    """
    rests, durations, dwell = _route(plan)
    return _route_segments(rests[::-1], durations[::-1], dwell, 'return')


def move_segment(start, goal, max_speed, minimum_duration=2.0, name='move to home'):
    """A slow rest-to-rest move whose peak speed stays under max_speed per joint.

    This is NOT a certified motion: it has no collision or clearance check.
    """
    start = _configuration(start, f'{name} start')
    goal = _configuration(goal, f'{name} goal')
    max_speed = np.asarray(max_speed, dtype=float)
    if max_speed.shape != (NUM_JOINTS,) or np.any(max_speed <= 0):
        raise ValueError('move needs a positive speed limit per joint')
    duration = max(minimum_duration,
                   float(np.max(QUINTIC_PEAK * np.abs(goal - start) / max_speed)))
    return quintic_segment(name, start, goal, duration)


def approach_segments(plan, measured, tolerance, arm_limit, max_speed, settle_s=1.0,
                      rail_realign_limit=0.05):
    """How to bring the robot from its measured pose to the plan's home.

    Returns (segments, description), with segments in plan time. Raises
    ValueError when there is no motion this function is willing to choose:

      at home        nothing to do.
      at task start  (arm within tolerance, rail within rail_realign_limit)
                     re-align slowly, then retrace the certified warmup
                     backwards (e.g. after a warmup-only run), so
                     demonstrations repeat. The rail tracks open-loop in
                     0.1 mm/s steps and can end a few cm off; correcting it
                     with the arm held keeps the arm's clearance, because the
                     mounting wall and floor both run parallel to the rail.
      arm near home  (every arm joint within arm_limit): first straighten
                     the arm with the rail held, then move the rail with the
                     arm held at home. The modelled wall and floor are uniform
                     along the rail's travel, so translating the home posture
                     keeps the clearance it was screened with; the small arm
                     move is uncertified and should be watched.
      otherwise      refuse: a large uncertified arm sweep is the operator's
                     call, made from the pendant.

    max_speed is per joint and in plan time: divide wall-clock limits by the
    reference's time_scale before passing them in.
    """
    measured = _configuration(measured, 'measured pose')
    home = np.asarray(plan['home'], dtype=float)
    task_start = np.asarray(plan['q_path'][0], dtype=float)
    tolerance = np.asarray(tolerance, dtype=float)
    if np.all(np.abs(measured - home) <= tolerance):
        return [], 'at plan home'
    if at_task_start(plan, measured, tolerance, rail_realign_limit):
        return ([move_segment(measured, task_start, max_speed, minimum_duration=1.0,
                              name='align to task start')]
                + reverse_warmup_segments(plan)
                + [hold_segment('at home', home, settle_s)],
                'returning along the certified warmup')
    arm_offset = np.abs(measured[1:] - home[1:])
    if np.any(arm_offset > arm_limit):
        worst = int(np.argmax(arm_offset))
        raise ValueError(
            f'arm is {arm_offset[worst]:.3f} rad from plan home on joint {worst + 1}; '
            f'move it to plan home from the pendant '
            f'(degrees {np.round(np.degrees(home[1:]), 2).tolist()})')
    segments = []
    arm_home = measured.copy()
    arm_home[1:] = home[1:]
    if np.any(arm_offset > tolerance[1:]):
        segments.append(move_segment(measured, arm_home, max_speed, name='straighten arm'))
    if abs(measured[0] - home[0]) > tolerance[0]:
        segments.append(move_segment(arm_home, home, max_speed, name='rail to home'))
    segments.append(hold_segment('at home', home, settle_s))
    return segments, 'moving to plan home'


class Reference:
    """Consecutive segments played at time_scale, in wall-clock seconds."""

    def __init__(self, segments, time_scale=1.0):
        if not segments:
            raise ValueError('a reference needs at least one segment')
        if not np.isfinite(time_scale) or not 0 < time_scale <= 1:
            raise ValueError('time_scale must be in (0, 1]')
        self.segments = list(segments)
        self.time_scale = float(time_scale)
        self._starts = np.cumsum([0.0] + [s.duration for s in self.segments])
        self.plan_duration = float(self._starts[-1])
        self.duration = self.plan_duration / self.time_scale

    def sample(self, wall_t):
        """Reference position, wall-clock velocity and phase name at wall_t.

        Before 0 the reference holds its start, after the end it holds its
        final configuration at rest.
        """
        tau = min(max(wall_t * self.time_scale, 0.0), self.plan_duration)
        index = int(np.searchsorted(self._starts, tau, side='right')) - 1
        index = min(max(index, 0), len(self.segments) - 1)
        segment = self.segments[index]
        q, qd = segment.evaluate(tau - self._starts[index])
        if wall_t >= self.duration:
            qd = np.zeros(NUM_JOINTS)
        return q, qd * self.time_scale, segment.name

    @property
    def start(self):
        return self.sample(0.0)[0]

    @property
    def end(self):
        return self.sample(self.duration)[0]

    def peak_speeds(self, plan_rate_hz=500.0):
        """Largest |wall-clock velocity| per joint, sampled finely in plan time."""
        count = max(2, int(np.ceil(self.plan_duration * plan_rate_hz)) + 1)
        peaks = np.zeros(NUM_JOINTS)
        for tau in np.linspace(0.0, self.plan_duration, count):
            peaks = np.maximum(peaks, np.abs(self.sample(tau / self.time_scale)[1]))
        return peaks
