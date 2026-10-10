#!/usr/bin/env python3
"""Choosing which turn a joint angle is on.

Inverse kinematics returns every revolute solution wrapped into [-pi, pi], but
five of the six arm joints declare +/-2*pi. A smooth motion crossing that
boundary therefore appears as a delta of nearly a full turn. Measured on the
500-waypoint trajectory, all 17 arm-velocity failures were wrist_1 deltas
within 0.05 rad of -2*pi: the real motion was under 0.02 rad, the gate rejects
above 0.2 rad at a 0.1 s step, and the trajectory was cut in two as a result.

A lifted value

    q(k) = q + 2*pi*k

is the same physical configuration, so forward kinematics, the Jacobian and
collision geometry are all invariant under it. Only the continuous
coordinates change, which is exactly what velocity, interpolation and playback
depend on.

Which lift to take depends on the caller:

  greedy tracker   the feasible lift nearest the previous COMMANDED
                   configuration, so the recorded motion is continuous
  fresh entry      nearest the actual start or seed posture, never the
                   perturbed numerical seed, which is an implementation
                   detail of the retry loop
  graph            every lift reachable from a predecessor, since the choice
                   is what the planner is for

Two joints are special and both matter:

  the rail         prismatic, never periodic, never lifted
  the elbow        declares only [-pi, pi], a 2*pi span, so no alternative
                   lift fits. A boundary crossing there is a genuine large
                   move and must stay rejected

np.unwrap is deliberately not used: it knows neither joint limits nor which
joints are periodic, so it would invent values outside the elbow's range.
"""
import numpy as np

TWO_PI = 2.0 * np.pi


def feasible_lifts(value, lower, upper, periodic):
    """Every q + 2*pi*k for one joint that stays inside its limits.

    Returned in order of increasing |k|, then increasing value, so callers
    that break ties by taking the first get a deterministic answer.
    """
    if not periodic:
        return [float(value)] if lower <= value <= upper else []

    lowest = int(np.ceil((lower - value) / TWO_PI - 1e-9))
    highest = int(np.floor((upper - value) / TWO_PI + 1e-9))
    if highest < lowest:
        return []
    turns = sorted(range(lowest, highest + 1), key=lambda k: (abs(k), k))
    return [float(value + TWO_PI * k) for k in turns]


def winding_numbers(lifted, canonical):
    """Turns separating a lifted configuration from its canonical form."""
    lifted = np.asarray(lifted, dtype=float)
    canonical = np.asarray(canonical, dtype=float)
    return np.rint((lifted - canonical) / TWO_PI).astype(int)


def nearest_feasible_lift(q_canonical, q_reference, joint_limits, periodic):
    """Per joint, the legal lift closest to the reference configuration.

    Ties, which occur exactly at a half turn from the reference, resolve to
    the smaller |k| and then the lower value, so repeated runs agree.
    """
    q_canonical = np.asarray(q_canonical, dtype=float)
    q_reference = np.asarray(q_reference, dtype=float)
    lower, upper = joint_limits
    chosen = np.empty_like(q_canonical)

    for index, value in enumerate(q_canonical):
        options = feasible_lifts(value, lower[index], upper[index],
                                 periodic[index])
        if not options:
            # Outside its limits already; leave it be so the limit check, not
            # this helper, is what reports the problem.
            chosen[index] = value
            continue
        chosen[index] = min(
            options,
            key=lambda option: (round(abs(option - q_reference[index]), 12),
                                abs(option - value), option),
        )
    return chosen


def reachable_feasible_lifts(q_canonical, q_predecessor, joint_limits,
                             periodic, velocity_limits, dt):
    """Every legal lift a predecessor can actually reach within one step.

    Per joint, keeps the lifts inside limits whose displacement from the
    predecessor is within velocity_limit * dt, then returns the combinations.
    Filtering per joint before combining keeps this small: usually one lift per
    joint survives, and the product only grows where the motion genuinely has
    alternatives.

    Returns a list of configurations. An empty list means some joint has no
    reachable lift, which is itself the answer that no transition exists.
    """
    q_canonical = np.asarray(q_canonical, dtype=float)
    q_predecessor = np.asarray(q_predecessor, dtype=float)
    lower, upper = joint_limits

    per_joint = []
    for index, value in enumerate(q_canonical):
        options = feasible_lifts(value, lower[index], upper[index],
                                 periodic[index])
        budget = float(velocity_limits[index]) * float(dt)
        reachable = [option for option in options
                     if abs(option - q_predecessor[index]) <= budget]
        if not reachable:
            return []
        per_joint.append(reachable)

    rows = [[]]
    for options in per_joint:
        rows = [row + [option] for row in rows for option in options]
    return [np.asarray(row, dtype=float) for row in rows]
