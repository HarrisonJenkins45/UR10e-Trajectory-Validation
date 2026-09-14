#!/usr/bin/env python3
"""Stage 7: choose a ready pose, coupled to the approach that has to reach it.

A ready pose cannot be ranked on static gates alone. What makes one good is
that it connects to SEVERAL layer-0 branches cheaply and smoothly, and that
is a property of the approach, so the evaluator has to exist before a winner
is picked.

Three classifications per (ready pose, placement), kept distinct because they
mean different things and one of them is not about the ready pose at all:

  no_task_candidate      no generated layer-0 candidate passes pose,
                         collision, conditioning and alpha*. This says the
                         GENERATOR found nothing here, not that the placement
                         is intrinsically infeasible; candidate generation may
                         simply be incomplete
  approach_unconnected   task-feasible candidates exist and this ready pose
                         cannot reach any of them. This one IS about the
                         ready pose
  connected              at least one valid approach exists

A placement classified no_task_candidate must not count against a ready
pose's connectivity score. There was nothing there to connect to.

Jerk is NOT a rejection gate. The limit is assumed, with no published UR jerk
figure behind it, so rejecting on it would invent infeasibility. It is a
provisional design constraint and a ranking signal, reported against several
declared references rather than one.

Jerk is also meaningless unless timing is standardised, since it scales
strongly with duration. Every candidate therefore uses the same retiming
policy: a quintic from rest at the ready pose to the layer-0 entry state,
matching position, velocity AND acceleration there, with the duration taken
as the shortest satisfying velocity and provisional acceleration limits.
Comparing jerk across candidates only means something because the duration
was derived the same way for each.

Selection order: hard feasibility and connectivity first, duration second,
jerk third.

Nothing here certifies anything. The envelope is a software robustness
envelope rather than the physical operating envelope, and
motion_limits.may_certify_for_hardware() is False, so the output is a
PROVISIONAL ready pose.
"""
import numpy as np

from ur10e_trajectory_pkg import frames, motion_limits
from ur10e_trajectory_pkg.configurations import ARM_SLICE, JOINT_NAMES, RAIL_INDEX

SCHEMA_VERSION = 1

# Software robustness envelope, NOT the physical operating envelope.
# Certification later needs the permitted arena volume, attitude range,
# calibration uncertainty and an obstacle survey, none of which this
# repository contains.
PROVISIONAL_STAGE7_ENVELOPE_V1 = {
    'translation_m': 0.25,
    'rotation_deg': 15.0,
    'coupled_samples': 16,
    'scale': 1.0,          # relative-motion scale does not move layer 0
}

# Jerk references to report against. The 500 rad/s^3 figure is assumed, so a
# single number would read as a threshold; several make the ordering visible
# without implying any of them is the limit.
JERK_REFERENCES_RAD_S3 = (100.0, 250.0, 500.0, 1000.0)

# Characteristic length that makes the arm Jacobian dimensionless. The arm's
# own reach, so the scaling is a property of the robot rather than a tuned
# constant.
CHARACTERISTIC_LENGTH_M = 1.3


def placements(envelope=PROVISIONAL_STAGE7_ENVELOPE_V1, seed=0):
    """29 deterministic placements: nominal, 12 axis extrema, 16 coupled."""
    translation = envelope['translation_m']
    rotation = envelope['rotation_deg']
    out = [{'name': 'nominal', 'translation': np.zeros(3), 'rotation_deg': np.zeros(3)}]

    for axis in range(3):
        for sign in (-1.0, 1.0):
            shift = np.zeros(3)
            shift[axis] = sign * translation
            out.append({'name': f'translate_{"xyz"[axis]}{"+" if sign > 0 else "-"}',
                        'translation': shift, 'rotation_deg': np.zeros(3)})
    for axis in range(3):
        for sign in (-1.0, 1.0):
            turn = np.zeros(3)
            turn[axis] = sign * rotation
            out.append({'name': f'rotate_{"rpy"[axis]}{"+" if sign > 0 else "-"}',
                        'translation': np.zeros(3), 'rotation_deg': turn})

    # Deterministic coupled samples. A fixed generator rather than Sobol so
    # the set is reproducible without another dependency.
    rng = np.random.default_rng(seed)
    for index in range(envelope['coupled_samples']):
        out.append({
            'name': f'coupled_{index:02d}',
            'translation': rng.uniform(-translation, translation, 3),
            'rotation_deg': rng.uniform(-rotation, rotation, 3),
        })
    return out


def placement_transform(placement, nominal_RG):
    """Apply a placement offset to the nominal layer-0 placement."""
    from scipy.spatial.transform import Rotation
    offset = frames.make_transform(
        rotation=Rotation.from_euler('xyz', placement['rotation_deg'],
                                     degrees=True).as_matrix(),
        translation=placement['translation'])
    return offset @ np.asarray(nominal_RG, dtype=float)


# --------------------------------------------------------------------------
# Static gates on a ready-pose candidate
# --------------------------------------------------------------------------

def joint_limit_clearance(validator, configuration):
    """Smallest distance to a limit, as a fraction of that joint's range.

    Normalised per joint so the rail's metres and the arm's radians compare,
    and dimensionless so it can be ranked.
    """
    lower, upper = validator.robot.qlim
    span = upper - lower
    margin = np.minimum(configuration - lower, upper - configuration)
    return float(np.min(margin / np.where(span > 0, span, 1.0)))


def posture_margin(validator, configuration):
    """Dimensionless conditioning of the arm, in [0, 1], higher is better.

    The reciprocal condition number of the arm Jacobian with its linear rows
    scaled by the arm's own reach. The raw condition number mixes metres and
    radians, so it changes with the choice of length unit and cannot be
    ranked; scaling by a characteristic length removes that.
    """
    jacobian = validator.compute_arm_jacobian(configuration).copy()
    jacobian[:3, :] /= CHARACTERISTIC_LENGTH_M
    singular = np.linalg.svd(jacobian, compute_uv=False)
    return float(singular[-1] / singular[0]) if singular[0] > 1e-12 else 0.0


def collision_distance(validator, configuration, max_distance=0.5):
    """Closest approach to the environment, not merely a collision flag.

    Clearance is what distinguishes two poses that are both collision-free,
    and the whole point of ranking is to tell them apart.
    """
    import pybullet as pb

    for pb_index, value in zip(validator._pb_joint_indices, configuration):
        pb.resetJointState(validator.robot_id, pb_index, float(value),
                           physicsClientId=validator._pb_client)
    closest = max_distance
    for body in (validator.floor_id, validator.wall_id):
        for contact in pb.getClosestPoints(
                bodyA=validator.robot_id, bodyB=body, distance=max_distance,
                physicsClientId=validator._pb_client):
            name = validator._pb_link_name_by_index.get(contact[3])
            if name in validator._rail_link_names:
                continue
            closest = min(closest, float(contact[8]))
    return closest


def static_gates(validator, configuration, min_clearance_m=0.02,
                 min_limit_fraction=0.05, min_posture_margin=0.02):
    """Whether a candidate ready pose is admissible at all."""
    clearance = collision_distance(validator, configuration)
    limits = joint_limit_clearance(validator, configuration)
    margin = posture_margin(validator, configuration)
    return {
        'collision_distance_m': clearance,
        'joint_limit_clearance': limits,
        'posture_margin': margin,
        'passed': bool(clearance >= min_clearance_m
                       and limits >= min_limit_fraction
                       and margin >= min_posture_margin),
    }


# --------------------------------------------------------------------------
# The approach, with a standardised retiming
# --------------------------------------------------------------------------

def quintic_coefficients(start, end, end_velocity, end_acceleration, duration):
    """Quintic matching position, velocity and acceleration at the task entry.

    Starts from rest with zero acceleration, which is what a parked robot is.
    Matching velocity AND acceleration at the far end matters: a join that
    only matches position creates a step in velocity, and one that matches
    velocity but not acceleration creates a jerk spike exactly at handover.
    """
    T = float(duration)
    a0, a1, a2 = np.asarray(start, float), np.zeros_like(start), np.zeros_like(start)
    end = np.asarray(end, float)
    ve, ae = np.asarray(end_velocity, float), np.asarray(end_acceleration, float)

    a3 = (20 * (end - a0) - (8 * ve + 0.0) * T - (3 * 0.0 - ae) * T ** 2) / (2 * T ** 3)
    a4 = (30 * (a0 - end) + (14 * ve) * T + (3 * 0.0 - 2 * ae) * T ** 2) / (2 * T ** 4)
    a5 = (12 * (end - a0) - 6 * ve * T + ae * T ** 2) / (2 * T ** 5)
    return np.stack([a0, a1, a2, a3, a4, a5])


def sample_quintic(coefficients, duration, samples=400):
    """Position and its three derivatives, taken analytically.

    Differentiating the polynomial rather than differencing samples, so the
    reported jerk does not depend on the sample count.
    """
    times = np.linspace(0.0, duration, samples)

    def evaluate(order):
        rows = []
        for power in range(6):
            if power < order:
                rows.append(np.zeros_like(times))
                continue
            scale = 1.0
            for step in range(order):
                scale *= power - step
            rows.append(scale * times ** (power - order))
        return np.einsum('kt,kj->tj', np.stack(rows), coefficients)

    return times, evaluate(0), evaluate(1), evaluate(2), evaluate(3)


def minimum_duration(start, end, end_velocity, end_acceleration,
                     velocity_limits, acceleration_limits,
                     lower=0.2, upper=20.0, tolerance=0.01):
    """Shortest duration satisfying velocity and provisional acceleration.

    The same policy for every candidate, which is what makes their jerk
    comparable: jerk scales strongly with duration, so candidates timed
    differently cannot be ranked against each other at all.
    """
    def feasible(duration):
        coefficients = quintic_coefficients(start, end, end_velocity,
                                            end_acceleration, duration)
        _, _, velocity, acceleration, _ = sample_quintic(coefficients, duration)
        return (np.all(np.abs(velocity) <= velocity_limits)
                and np.all(np.abs(acceleration) <= acceleration_limits))

    if not feasible(upper):
        return None
    low, high = lower, upper
    if feasible(low):
        return low
    while high - low > tolerance:
        middle = 0.5 * (low + high)
        if feasible(middle):
            high = middle
        else:
            low = middle
    return high


# --------------------------------------------------------------------------
# Evaluating one approach
# --------------------------------------------------------------------------

NO_TASK_CANDIDATE = 'no_task_candidate'
APPROACH_UNCONNECTED = 'approach_unconnected'
CONNECTED = 'connected'


def evaluate_approach(validator, ready, target, entry_velocity,
                      entry_acceleration, velocity_limits,
                      acceleration_limits, jerk_references=JERK_REFERENCES_RAD_S3):
    """One ready pose to one layer-0 candidate, under the standard retiming.

    Feasibility here means velocity, acceleration, collision and joint limits.
    Jerk is measured and reported against several references, never used to
    reject: the limit is assumed, with no published UR figure behind it, so
    rejecting on it would invent infeasibility.
    """
    duration = minimum_duration(ready, target, entry_velocity,
                                entry_acceleration, velocity_limits,
                                acceleration_limits)
    if duration is None:
        return {'feasible': False, 'reason': 'no duration satisfies velocity '
                                             'and acceleration limits'}

    coefficients = quintic_coefficients(ready, target, entry_velocity,
                                        entry_acceleration, duration)
    _, position, velocity, acceleration, jerk = sample_quintic(
        coefficients, duration)

    lower, upper = validator.robot.qlim
    if np.any(position < lower - 1e-9) or np.any(position > upper + 1e-9):
        return {'feasible': False, 'reason': 'approach leaves joint limits',
                'duration_s': duration}

    for configuration in position[::10]:
        if validator.check_all_collisions(configuration):
            return {'feasible': False, 'reason': 'approach collides',
                    'duration_s': duration}

    peak_jerk = np.max(np.abs(jerk), axis=0)
    integrated = np.trapz(np.abs(jerk), dx=duration / (len(jerk) - 1), axis=0)
    return {
        'feasible': True,
        'duration_s': float(duration),
        'peak_velocity': np.max(np.abs(velocity), axis=0).tolist(),
        'peak_acceleration': np.max(np.abs(acceleration), axis=0).tolist(),
        'peak_jerk': peak_jerk.tolist(),
        'integrated_jerk': integrated.tolist(),
        'max_peak_jerk': float(np.max(peak_jerk)),
        # Reported against several references rather than one, so the ordering
        # is visible without implying any of them is a limit.
        'jerk_within_reference': {
            str(reference): bool(np.max(peak_jerk[ARM_SLICE]) <= reference)
            for reference in jerk_references
        },
        'matches_entry_state': True,     # by construction of the quintic
    }


def classify(task_candidates, approaches):
    """Three outcomes, kept distinct because they mean different things.

    no_task_candidate is NOT a statement about the ready pose, and must not
    count against its connectivity: there was nothing there to reach.
    """
    if not task_candidates:
        return NO_TASK_CANDIDATE
    if not any(a['feasible'] for a in approaches):
        return APPROACH_UNCONNECTED
    return CONNECTED


def connectivity_score(classifications):
    """Fraction of ELIGIBLE placements this ready pose connects to.

    Placements with no task-feasible candidate are excluded from the
    denominator rather than scored as failures.
    """
    eligible = [c for c in classifications if c != NO_TASK_CANDIDATE]
    if not eligible:
        return None
    return sum(1 for c in eligible if c == CONNECTED) / len(eligible)


def rank_ready_poses(records):
    """Hard feasibility and connectivity first, duration second, jerk third.

    Jerk never excludes a candidate; it only orders candidates that are
    already feasible and equally connected.
    """
    def key(record):
        connectivity = record['connectivity'] if record['connectivity'] is not None else -1.0
        return (-connectivity,
                record['worst_duration_s'] if record['worst_duration_s'] is not None else np.inf,
                record['worst_peak_jerk'] if record['worst_peak_jerk'] is not None else np.inf)

    return sorted([r for r in records if r['static_gates']['passed']], key=key)


def certification_note():
    """Why nothing here certifies a ready pose."""
    return {
        'envelope': 'PROVISIONAL_STAGE7_ENVELOPE_V1, a software robustness '
                    'envelope rather than the physical operating envelope',
        'limits': motion_limits.certification_status(),
        'may_certify_for_hardware': motion_limits.may_certify_for_hardware(),
        'result_status': 'provisional',
    }
