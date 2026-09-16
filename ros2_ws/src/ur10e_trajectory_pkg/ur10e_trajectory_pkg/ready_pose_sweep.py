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
  direct_approach_unconnected
                         task-feasible candidates exist and no DIRECT quintic
                         from this ready pose reaches any of them. This one IS
                         about the ready pose. The reasons stay visible through
                         failure_breakdown rather than being collapsed
  connected              at least one valid approach exists

A branch is an IK FAMILY (ik_family_labels): with the rail fixed the arm has a
handful of solutions for a pose, and as the rail moves each traces a
continuous curve. Tolerance clusters (branch_assignments) split one family
into several whenever the arm moves more than the tolerance along the rail,
so they are kept as a diagnostic and families are what ranking counts.

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
# V2 rotates about the layer-0 TARGET, in world (rail-base) axes. V1 rotated
# about the rail-base origin, so a 15 degree attitude change also translated
# the target by up to 0.46 m: 14 of 29 placements moved it further than the
# declared 0.25 m, three put it under the floor or into the wall, and the
# ranking was largely set by those shifts. Artifacts from the two definitions
# must not be compared.
PROVISIONAL_STAGE7_ENVELOPE_V2 = {
    'translation_m': 0.25,
    'rotation_deg': 15.0,
    'coupled_samples': 16,
    'scale': 1.0,          # relative-motion scale does not move layer 0
    'rotation_centre': 'layer-0 target (the placement origin; the trajectory '
                       'holds position constant)',
    'rotation_axes': 'world (rail-base frame), so placement names keep '
                     'their meaning',
    'name': 'PROVISIONAL_STAGE7_ENVELOPE_V2',
}

# Jerk references to report against. The 500 rad/s^3 figure is assumed, so a
# single number would read as a threshold; several make the ordering visible
# without implying any of them is the limit.
JERK_REFERENCES_RAD_S3 = (100.0, 250.0, 500.0, 1000.0)

# Characteristic length that makes the arm Jacobian dimensionless. The arm's
# own reach, so the scaling is a property of the robot rather than a tuned
# constant.
CHARACTERISTIC_LENGTH_M = 1.3

# Collision-check resolution, PER JOINT. A single scalar applied across the
# configuration vector silently means "0.05 metres" for the rail and "0.05
# radians" for the arm, which are different quantities that happen to share a
# number. Stated separately so neither is inherited from the other.
COLLISION_STEP_RAIL_M = 0.05
COLLISION_STEP_ARM_RAD = 0.05


# Self-clearance resolution, finer than the collision resolution. Measured
# rather than assumed: across 421 feasible approaches for the chosen home,
# the worst clearances sit at the floor itself (minimum 10.0 mm, 5% within
# 1 mm of it), so what the coarser stride steps over is exactly what decides
# the verdict. A boolean collision test tolerates a coarse stride because
# links either touch or do not; a margin does not.
SELF_CLEARANCE_STEP_RAIL_M = 0.01
SELF_CLEARANCE_STEP_ARM_RAD = 0.01


def collision_step_bounds():
    """Per-joint resolution bound, in JOINT_NAMES order."""
    return np.array([COLLISION_STEP_RAIL_M] + [COLLISION_STEP_ARM_RAD] * 6)


def self_clearance_step_bounds():
    """Per-joint resolution bound for the clearance query, finer than above."""
    return np.array([SELF_CLEARANCE_STEP_RAIL_M]
                    + [SELF_CLEARANCE_STEP_ARM_RAD] * 6)


def placements(envelope=PROVISIONAL_STAGE7_ENVELOPE_V2, seed=0):
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
    """Apply a placement offset to the nominal layer-0 placement.

        T = Trans(translation) . Trans(p0) . Rot . Trans(-p0) . nominal

    with p0 the nominal target position and Rot about WORLD axes. Rotation
    therefore turns the target's orientation in place and never moves it; only
    the declared translation does.
    """
    from scipy.spatial.transform import Rotation
    nominal_RG = np.asarray(nominal_RG, dtype=float)
    centre = nominal_RG[:3, 3]
    rotation = frames.make_transform(
        rotation=Rotation.from_euler('xyz', placement['rotation_deg'],
                                     degrees=True).as_matrix(),
        translation=np.zeros(3))
    return (frames.make_transform(rotation=np.eye(3),
                                  translation=np.asarray(placement['translation'])
                                  + centre)
            @ rotation
            @ frames.make_transform(rotation=np.eye(3), translation=-centre)
            @ nominal_RG)


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


def arm_link_indices(validator):
    """PyBullet link indices moved by an arm joint: a revolute joint in their chain.

    Everything else is fixed to the world or rides the carriage without
    rotating, so its distance to the environment says nothing about the arm's
    posture. Derived from the model's joint tree rather than a list of names,
    so a renamed or added fixed link cannot slip back in. Cached on the
    validator.
    """
    import pybullet as pb

    cached = getattr(validator, '_arm_link_indices', None)
    if cached is not None:
        return cached
    client = validator._pb_client
    parents, revolute = {}, {}
    for index in range(pb.getNumJoints(validator.robot_id, physicsClientId=client)):
        info = pb.getJointInfo(validator.robot_id, index, physicsClientId=client)
        revolute[index] = info[2] == pb.JOINT_REVOLUTE
        parents[index] = info[16]
    indices = set()
    for index in parents:
        link = index
        while link != -1:
            if revolute[link]:
                indices.add(index)
                break
            link = parents[link]
    validator._arm_link_indices = frozenset(indices)
    return validator._arm_link_indices


def collision_distance(validator, configuration, max_distance=0.5):
    """Closest approach of the ARM to the environment, not merely a flag.

    Clearance is what distinguishes two poses that are both collision-free.
    Only links moved by an arm joint count (arm_link_indices). It used to skip
    only the two rail links, so base_link_inertia -- fixed to the carriage --
    set a constant 0.049 m against the floor for every posture, and the gate
    measured nothing about the arm.
    """
    import pybullet as pb

    for pb_index, value in zip(validator._pb_joint_indices, configuration):
        pb.resetJointState(validator.robot_id, pb_index, float(value),
                           physicsClientId=validator._pb_client)
    arm_links = arm_link_indices(validator)
    closest = max_distance
    for body in (validator.floor_id, validator.wall_id):
        for contact in pb.getClosestPoints(
                bodyA=validator.robot_id, bodyB=body, distance=max_distance,
                physicsClientId=validator._pb_client):
            if contact[3] not in arm_links:
                continue
            closest = min(closest, float(contact[8]))
    return closest


# Singularity robustness: the arm must stay within the task's conditioning gate
# under small joint errors, not merely at the nominal joints. Posture margin at
# the nominal joints missed poses that exceeded the gate a few degrees away.
TASK_CONDITION_GATE = 50.0
SINGULARITY_NEIGHBOURHOOD_DEG = 5.0
SINGULARITY_RANDOM_SAMPLES = 64
SINGULARITY_SEED = 20260915


def arm_condition_number(validator, configuration):
    singular = np.linalg.svd(validator.compute_arm_jacobian(configuration),
                             compute_uv=False)
    return float(singular[0] / singular[-1]) if singular[-1] > 1e-12 else np.inf


def singularity_robustness(validator, configuration,
                           degrees=SINGULARITY_NEIGHBOURHOOD_DEG,
                           random_samples=SINGULARITY_RANDOM_SAMPLES,
                           seed=SINGULARITY_SEED, gate=TASK_CONDITION_GATE):
    """Worst arm condition number over a +/-degrees neighbourhood.

    The neighbourhood is every arm joint alone at +degrees and -degrees, plus
    random_samples configurations with all arm joints offset uniformly within
    +/-degrees, from a fixed seed. The rail does not enter the arm Jacobian,
    so it is not perturbed.
    """
    configuration = np.asarray(configuration, dtype=float)
    step = np.deg2rad(degrees)
    offsets = []
    for joint in range(6):
        for sign in (1.0, -1.0):
            offset = np.zeros(6)
            offset[joint] = sign * step
            offsets.append(offset)
    rng = np.random.default_rng(seed)
    offsets += list(rng.uniform(-step, step, (random_samples, 6)))

    nominal = arm_condition_number(validator, configuration)
    worst, worst_offset = nominal, np.zeros(6)
    for offset in offsets:
        trial = configuration.copy()
        trial[ARM_SLICE] += offset
        condition = arm_condition_number(validator, trial)
        if condition > worst:
            worst, worst_offset = condition, offset
    return {'nominal_condition': nominal, 'worst_condition': worst,
            'worst_offset_deg': np.rad2deg(worst_offset).tolist(),
            'neighbourhood_deg': degrees, 'samples': len(offsets),
            'random_samples': random_samples, 'seed': seed, 'gate': gate,
            'passed': bool(worst <= gate)}


def static_gates(validator, configuration, min_clearance_m=0.02,
                 min_limit_fraction=0.05, min_posture_margin=0.02,
                 min_self_clearance_m=None):
    """Whether a candidate ready pose is admissible at all.

    collision_distance measures robot-to-ENVIRONMENT clearance only, so it
    cannot see the arm folded into itself. A broadly sampled pool contains
    plenty of self-colliding configurations, so the boolean self-collision
    check is a hard gate here rather than something the approach discovers
    later.

    Self-clearance between non-adjacent links is a floor too
    (motion_limits.SELF_CLEARANCE_FLOOR_M), measured by the validator's one
    self_clearance function, so a pose that merely avoids contact by a
    fraction of a millimetre is not admitted.
    """
    if min_self_clearance_m is None:
        min_self_clearance_m = motion_limits.SELF_CLEARANCE_FLOOR_M
    in_collision = bool(validator.check_all_collisions(configuration))
    self_clearance = validator.self_clearance(configuration)
    clearance = collision_distance(validator, configuration)
    limits = joint_limit_clearance(validator, configuration)
    margin = posture_margin(validator, configuration)
    return {
        'in_collision': in_collision,
        'collision_distance_m': clearance,
        'joint_limit_clearance': limits,
        'posture_margin': margin,
        'self_clearance_m': self_clearance['distance_m'],
        'self_clearance_links': self_clearance['links'],
        'passed': bool(not in_collision
                       and self_clearance['distance_m'] >= min_self_clearance_m
                       and clearance >= min_clearance_m
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


# Samples at which velocity and acceleration are checked, in normalised time.
# Shared by sample_quintic and the exact duration solve, so both answer the
# question for the same instants.
QUINTIC_SAMPLES = 400


def sample_quintic(coefficients, duration, samples=QUINTIC_SAMPLES):
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


# Normalised-time basis of the quintic from rest to (D, ve, ae), s = t / T:
#
#     T   * velocity(s)     = D F1(s) + ve T G1(s) + ae T^2 H1(s)
#     T^2 * acceleration(s) = D F2(s) + ve T G2(s) + ae T^2 H2(s)
#
# Derived from quintic_coefficients, so each velocity or acceleration bound at
# a sample is a quadratic inequality in T.
_S = np.linspace(0.0, 1.0, QUINTIC_SAMPLES)
_F1 = 30 * _S**2 - 60 * _S**3 + 30 * _S**4
_G1 = -12 * _S**2 + 28 * _S**3 - 15 * _S**4
_H1 = 1.5 * _S**2 - 4 * _S**3 + 2.5 * _S**4
_F2 = 60 * _S - 180 * _S**2 + 120 * _S**3
_G2 = -24 * _S + 84 * _S**2 - 60 * _S**3
_H2 = 3 * _S - 12 * _S**2 + 10 * _S**3


def duration_lower_bound(start, end, velocity_limits, lower=0.2):
    """No approach can be shorter: no joint can average more than its limit."""
    displacement = np.abs(np.asarray(end, float) - np.asarray(start, float))
    return max(float(lower),
               float(np.max(displacement / np.asarray(velocity_limits, float))))


def _sampled_feasible(start, end, end_velocity, end_acceleration, duration,
                      velocity_limits, acceleration_limits):
    """The package's own check: the limits at the quintic's samples."""
    coefficients = quintic_coefficients(start, end, end_velocity,
                                        end_acceleration, duration)
    _, _, velocity, acceleration, _ = sample_quintic(coefficients, duration)
    return bool(np.all(np.abs(velocity) <= velocity_limits)
                and np.all(np.abs(acceleration) <= acceleration_limits))


def _violated_intervals(c2, c1, c0):
    """Open intervals of T on which c2 T^2 + c1 T + c0 > 0, as (lo, hi) arrays.

    At most two per row. A row's boundary points satisfy it, so the violated
    sets are open and the feasible set is closed.
    """
    inf = np.inf
    los, his = [], []
    with np.errstate(divide='ignore', invalid='ignore'):
        discriminant = c1 * c1 - 4.0 * c2 * c0
        root = np.sqrt(np.maximum(discriminant, 0.0))
        # Stable roots: q / c2 and c0 / q.
        q = -0.5 * (c1 + np.where(c1 >= 0.0, root, -root))
        first = np.where(q != 0.0, q / c2, 0.0)
        second = np.where(q != 0.0, c0 / q, 0.0)
        small, large = np.minimum(first, second), np.maximum(first, second)

        upward = c2 > 0.0
        real = discriminant >= 0.0
        # Opens upward: violated outside the roots, or everywhere without them.
        mask = upward & real
        los += [np.full(mask.sum(), -inf), large[mask]]
        his += [small[mask], np.full(mask.sum(), inf)]
        mask = upward & ~real
        los.append(np.full(mask.sum(), -inf))
        his.append(np.full(mask.sum(), inf))
        # Opens downward: violated between the roots.
        mask = (c2 < 0.0) & (discriminant > 0.0)
        los.append(small[mask])
        his.append(large[mask])
        # Linear.
        linear = c2 == 0.0
        crossing = np.where(c1 != 0.0, -c0 / c1, 0.0)
        mask = linear & (c1 > 0.0)
        los.append(crossing[mask])
        his.append(np.full(mask.sum(), inf))
        mask = linear & (c1 < 0.0)
        los.append(np.full(mask.sum(), -inf))
        his.append(crossing[mask])
        mask = linear & (c1 == 0.0) & (c0 > 0.0)
        los.append(np.full(mask.sum(), -inf))
        his.append(np.full(mask.sum(), inf))
    return np.concatenate(los), np.concatenate(his)


def _first_uncovered(lo, hi, start):
    """Smallest T >= start lying in none of the open intervals (lo, hi).

    Sorted by lo, the running maximum of hi is how far the union reaches; the
    first interval starting at or beyond that reach leaves a gap there.
    """
    order = np.argsort(lo, kind='stable')
    lo, hi = lo[order], hi[order]
    reach = np.maximum.accumulate(np.maximum(hi, start))
    before = np.concatenate(([start], reach[:-1]))
    gaps = np.flatnonzero(lo >= before)
    return float(before[gaps[0]]) if len(gaps) else float(reach[-1])


def minimum_duration(start, end, end_velocity, end_acceleration,
                     velocity_limits, acceleration_limits,
                     lower=0.2, upper=20.0):
    """Shortest duration satisfying velocity and provisional acceleration, exactly.

    The same policy for every candidate, which is what makes their jerk
    comparable: jerk scales strongly with duration, so candidates timed
    differently cannot be ranked against each other at all.

    Feasibility is NOT monotone in duration once the entry state is nonzero:
    a long quintic swings away and back to arrive moving, so the feasible
    durations can form windows. Two searches have missed them. Bisecting down
    from the upper bound missed every winding of 5 of 34 nominal candidates;
    an upward scan in 1.1x steps then missed 16 of 710 windings whose windows
    spanned ratios as narrow as 1.007, and a finer ratio only narrows the
    miss. So there is no search:

      1. an entry state beyond the limits admits no duration
      2. in normalised time every velocity and acceleration bound at each of
         the QUINTIC_SAMPLES samples is a quadratic inequality in T, violated
         on at most two open intervals
      3. the answer is the first T at or above the average-speed lower bound
         covered by none of them, found by one sorted sweep

    The result is confirmed with the package's own sampled check, nudged up by
    one part in 10^9 when rounding at an exact root misses it. The constraint
    set is the sampled one, as before: this answers the same question
    exactly, not a finer one.
    """
    velocity_limits = np.asarray(velocity_limits, dtype=float)
    acceleration_limits = np.asarray(acceleration_limits, dtype=float)
    end_velocity = np.asarray(end_velocity, dtype=float)
    end_acceleration = np.asarray(end_acceleration, dtype=float)
    if (np.any(np.abs(end_velocity) > velocity_limits)
            or np.any(np.abs(end_acceleration) > acceleration_limits)):
        return None

    first = duration_lower_bound(start, end, velocity_limits, lower)
    if first > upper:
        return None

    D = (np.asarray(end, float) - np.asarray(start, float))[:, None]
    ve, ae = end_velocity[:, None], end_acceleration[:, None]
    V, A = velocity_limits[:, None], acceleration_limits[:, None]
    # Each row: c2 T^2 + c1 T + c0 <= 0.
    c2 = np.concatenate([ae * _H1, -ae * _H1, ae * _H2 - A, -ae * _H2 - A]).ravel()
    c1 = np.concatenate([ve * _G1 - V, -ve * _G1 - V, ve * _G2, -ve * _G2]).ravel()
    c0 = np.concatenate([D * _F1, -D * _F1, D * _F2, -D * _F2]).ravel()
    lo, hi = _violated_intervals(c2, c1, c0)

    candidate = _first_uncovered(lo, hi, first)
    # Rounding at a root can leave the candidate a hair inside an interval;
    # step past it and sweep again, a bounded number of times.
    for _ in range(8):
        if candidate > upper:
            return None
        for trial in (candidate, candidate * (1.0 + 1e-9)):
            if trial <= upper and _sampled_feasible(
                    start, end, end_velocity, end_acceleration, trial,
                    velocity_limits, acceleration_limits):
                return trial
        candidate = _first_uncovered(lo, hi, candidate * (1.0 + 1e-6))
    return None


# --------------------------------------------------------------------------
# Evaluating one approach
# --------------------------------------------------------------------------

NO_TASK_CANDIDATE = 'no_task_candidate'
DIRECT_APPROACH_UNCONNECTED = 'direct_approach_unconnected'
CONNECTED = 'connected'


def collision_check_indices(position, step_bounds=None):
    """Indices to check so no joint moves more than its own bound between them.

    Walks the curve rather than estimating a count from total variation and
    sampling uniformly in time. A quintic has nonuniform speed, so a uniform
    time stride does NOT bound the coordinate difference: it oversamples where
    the motion is slow and undersamples exactly where it is fastest, which is
    where an obstacle is most likely to be stepped over.

    Bounded by the supplied discretisation: if adjacent samples already exceed
    the bound, every one is checked and the achieved step is reported so a
    caller can tell.
    """
    position = np.asarray(position, dtype=float)
    bounds = collision_step_bounds() if step_bounds is None else np.asarray(step_bounds)
    chosen = [0]
    for index in range(1, len(position)):
        # Ratio against each joint's OWN bound, so metres and radians are not
        # compared against one shared number.
        step = float(np.max(np.abs(position[index] - position[chosen[-1]]) / bounds))
        if step > 1.0:
            # index is already too far, so take the LAST sample still inside
            # the bound. Taking index itself would leave a gap wider than the
            # bound, which is the error this function exists to prevent.
            previous = index - 1
            chosen.append(previous if previous > chosen[-1] else index)
    if chosen[-1] != len(position) - 1:
        chosen.append(len(position) - 1)
    return np.asarray(chosen)


def achieved_step(position, indices, step_bounds=None):
    """Worst consecutive step as a FRACTION of each joint's own bound.

    Returns a dimensionless ratio, so a value at or below 1.0 means every
    joint stayed inside its own limit. A raw coordinate difference could not
    say that, since the rail's and the arm's limits are different quantities.
    """
    bounds = collision_step_bounds() if step_bounds is None else np.asarray(step_bounds)
    checked = np.asarray(position, dtype=float)[np.asarray(indices)]
    if len(checked) < 2:
        return 0.0
    return float(np.max(np.abs(np.diff(checked, axis=0)) / bounds))


def approach_collides(validator, position, step_bounds=None, counter=None,
                      stats=None):
    """Collision along an approach, at a bounded per-joint resolution.

    counter, when given, receives the number of configurations ACTUALLY
    queried, which stops at the first collision; the corrected resolution makes
    that cost path-dependent rather than a fixed figure that could be assumed.
    stats, when given, also receives the planned query count and the achieved
    step ratio, so a caller can see whether the resolution bound held.
    """
    position = np.asarray(position, dtype=float)
    indices = (np.array([0]) if len(position) < 2
               else collision_check_indices(position, step_bounds))
    queried = 0
    collides = False
    for index in indices:
        queried += 1
        if validator.check_all_collisions(position[index]):
            collides = True
            break
    if counter is not None:
        counter.append(queried)
    if stats is not None:
        stats['collision_queries'] = queried
        stats['collision_queries_planned'] = int(len(indices))
        stats['collision_achieved_step'] = achieved_step(position, indices,
                                                         step_bounds)
    return collides


def approach_self_clearance(validator, position, step_bounds=None):
    """Worst non-adjacent self-clearance along an approach.

    On its own, finer sampling: the collision stride may step over the
    closest pass, which decides nothing for a boolean test but is the whole
    answer for a margin. step_bounds, when given, overrides it.
    """
    position = np.asarray(position, dtype=float)
    if step_bounds is None:
        step_bounds = self_clearance_step_bounds()
    indices = (np.array([0]) if len(position) < 2
               else collision_check_indices(position, step_bounds))
    worst, links, at = np.inf, None, None
    for index in indices:
        result = validator.self_clearance(position[index])
        if result['distance_m'] < worst:
            worst, links, at = result['distance_m'], result['links'], int(index)
    return {'min_distance_m': float(worst), 'links': links, 'at_sample': at,
            'samples': int(len(indices)),
            'achieved_step': achieved_step(position, indices, step_bounds)}


# Largest arm step between consecutive prefix layers that can be a genuine
# motion rather than an unlifted wrap. Half a turn in one 0.1 s layer is
# 31 rad/s, far beyond any joint's limit.
PREFIX_CONTINUITY_RAD = np.pi


def lifted_prefix(validator, prefix, destination=None):
    """A continuous lifted prefix, the only form entry_state_from_prefix takes.

    Candidates are stored canonically in [-pi, pi], so a prefix crossing a
    wrap differentiates to about 60 rad/s, minimum_duration returns None and
    the ready pose is silently classified unconnected. Each layer is lifted
    toward the previous LIFTED layer instead.

    destination, when given, is a lifted representation of layer 0 chosen by
    destination_lifts. The whole prefix follows it: shifting layer 0 by 2*pi*k
    without shifting layers 1 and 2 would reintroduce the same discontinuity
    one layer later.

    Returns None when no continuous lift exists within joint limits, which
    means this continuation is invalid from this representation.
    """
    from ur10e_trajectory_pkg.configurations import PERIODIC_JOINTS
    from ur10e_trajectory_pkg.joint_coordinates import (
        TWO_PI,
        nearest_feasible_lift,
    )

    prefix = np.asarray(prefix, dtype=float)
    lower, upper = validator.robot.qlim
    arm_limits = (lower[ARM_SLICE], upper[ARM_SLICE])
    periodic = PERIODIC_JOINTS[ARM_SLICE]

    first = prefix[0] if destination is None else np.asarray(destination, float)
    if destination is not None:
        if abs(first[RAIL_INDEX] - prefix[0][RAIL_INDEX]) > 1e-9:
            raise ValueError('destination rail differs from layer 0')
        turns = (first[ARM_SLICE] - prefix[0][ARM_SLICE]) / TWO_PI
        if (np.any(np.abs(turns - np.rint(turns)) > 1e-9)
                or np.any(np.rint(turns)[~np.asarray(periodic)] != 0)):
            raise ValueError('destination is not a lift of layer 0')

    lifted = [first]
    for layer in prefix[1:]:
        arm = nearest_feasible_lift(layer[ARM_SLICE], lifted[-1][ARM_SLICE],
                                    arm_limits, periodic)
        lifted.append(np.concatenate(([layer[RAIL_INDEX]], arm)))
    lifted = np.stack(lifted)

    if np.any(lifted < lower - 1e-9) or np.any(lifted > upper + 1e-9):
        return None
    if np.any(np.abs(np.diff(lifted[:, ARM_SLICE], axis=0))
              >= PREFIX_CONTINUITY_RAD):
        return None
    return lifted


def entry_state_from_prefix(prefix, dt):
    """Velocity and acceleration the TASK starts with at layer 0.

    Not a property of the layer-0 configuration. SciPy's PCHIP fixes its
    initial derivatives from the first three points, so the entry state the
    approach has to match depends on the continuation through layers 1 and 2.

    Matching only layer 0 is valid solely if the robot stops there and the
    task starts from rest, which would need an explicit task ramp-in;
    without one it reintroduces the velocity and acceleration discontinuity
    at handover that the quintic exists to avoid.
    """
    from scipy.interpolate import PchipInterpolator

    prefix = np.asarray(prefix, dtype=float)
    if len(prefix) < 3:
        raise ValueError(
            'entry state needs at least three layers: PCHIP derives its '
            'initial derivatives from the first three configurations'
        )
    if np.any(np.abs(np.diff(prefix[:, ARM_SLICE], axis=0))
              >= PREFIX_CONTINUITY_RAD):
        raise ValueError(
            'prefix is not continuous: an arm joint steps half a turn or more '
            'between layers, which is an unlifted wrap. Build it with '
            'lifted_prefix'
        )
    times = np.arange(len(prefix)) * dt
    interpolator = PchipInterpolator(times, prefix, axis=0)
    return (interpolator.derivative(1)(0.0), interpolator.derivative(2)(0.0))


def destination_lifts(validator, target, ready, velocity_limits, duration):
    """Feasible winding representations of a destination, from this ready pose.

    Winding expansion belongs to the RUNNER, not to evaluate_approach, which
    evaluates whichever single representation it is handed. Which lifts are
    reachable depends on where the approach starts and how long it has, so
    they cannot be enumerated when the candidate is generated.
    """
    from ur10e_trajectory_pkg.configurations import PERIODIC_JOINTS
    from ur10e_trajectory_pkg.joint_coordinates import reachable_feasible_lifts

    limits = (validator.robot.qlim[0][ARM_SLICE],
              validator.robot.qlim[1][ARM_SLICE])
    arms = reachable_feasible_lifts(
        np.asarray(target)[ARM_SLICE], np.asarray(ready)[ARM_SLICE],
        limits, PERIODIC_JOINTS[ARM_SLICE],
        np.asarray(velocity_limits)[ARM_SLICE], duration)
    return [np.concatenate(([np.asarray(target)[RAIL_INDEX]], arm))
            for arm in arms]


# Machine-readable failure reasons. The prose reason stays for people; these
# are what a runner counts, so the causes behind direct_approach_unconnected
# are not collapsed into one label.
REASON_NO_DURATION = 'no_duration'
REASON_JOINT_LIMITS = 'joint_limits'
REASON_COLLISION = 'collision'
REASON_SELF_CLEARANCE = 'self_clearance'


def binding_limit(velocity, acceleration, velocity_limits, acceleration_limits,
                  duration, statuses, lower, active=0.999):
    """Which joint and which limit set this approach's duration.

    The minimum duration is where some velocity or acceleration bound becomes
    active, so the largest peak-to-limit ratio names it. Its provenance status
    shows whether a certified limit or an assumed one is driving a ranking.
    A duration at the lower bound with no active limit is reported as such.

    statuses is motion_limits.limit_statuses(validator), which takes velocity
    provenance from the URDF check rather than the reference table. lower is
    the caller's duration lower bound, passed in rather than repeated.
    """
    velocity_ratio = (np.max(np.abs(velocity), axis=0)
                      / np.asarray(velocity_limits, dtype=float))
    acceleration_ratio = (np.max(np.abs(acceleration), axis=0)
                          / np.asarray(acceleration_limits, dtype=float))
    if velocity_ratio.max() >= acceleration_ratio.max():
        kind, ratios = 'velocity', velocity_ratio
    else:
        kind, ratios = 'acceleration', acceleration_ratio
    joint = int(np.argmax(ratios))
    ratio = float(ratios[joint])
    if ratio < active and duration <= lower + 1e-9:
        kind = 'minimum_duration'
    return {'joint': JOINT_NAMES[joint], 'kind': kind, 'ratio': ratio,
            'active': bool(ratio >= active),
            'status': (statuses[kind][joint] if kind != 'minimum_duration'
                       else None)}


def evaluate_approach(validator, ready, target, entry_velocity,
                      entry_acceleration, velocity_limits,
                      acceleration_limits, jerk_references=JERK_REFERENCES_RAD_S3,
                      step_bounds=None, collision_counter=None,
                      duration=None, duration_lower=0.2, limit_statuses=None,
                      min_self_clearance_m=None):
    """One ready pose to one layer-0 candidate, under the standard retiming.

    Feasibility here means velocity, acceleration, collision, self-clearance
    and joint limits. The self-clearance floor applies along the approach,
    not only at its ends: a home and a target that both clear the floor can
    still pass close through it.

    Jerk is measured and reported against several references, never used to
    reject: the limit is assumed, with no published UR figure behind it, so
    rejecting on it would invent infeasibility.

    Every result carries reason_code (None when feasible) and, once collision
    was checked, collision_queries and the achieved step ratio.
    collision_counter, a list, receives the queries actually made.

    Note the quintic's shape depends on the duration whenever the entry state
    is nonzero, through its ve*T and ae*T^2 terms. Changing velocity limits
    changes the duration and so can change the collision and joint-limit
    verdicts, not just the timing.

    duration, when given, is a minimum duration the caller already computed
    under the same limits, so a runner sorting alternatives by duration does
    not repeat the bisection for the ones it then collision-checks.
    """
    if duration is None:
        duration = minimum_duration(ready, target, entry_velocity,
                                    entry_acceleration, velocity_limits,
                                    acceleration_limits, lower=duration_lower)
    if duration is None:
        return {'feasible': False, 'reason_code': REASON_NO_DURATION,
                'reason': 'no duration satisfies velocity and acceleration '
                          'limits'}

    coefficients = quintic_coefficients(ready, target, entry_velocity,
                                        entry_acceleration, duration)
    _, position, velocity, acceleration, jerk = sample_quintic(
        coefficients, duration)

    lower, upper = validator.robot.qlim
    if np.any(position < lower - 1e-9) or np.any(position > upper + 1e-9):
        return {'feasible': False, 'reason_code': REASON_JOINT_LIMITS,
                'reason': 'approach leaves joint limits',
                'duration_s': duration}

    # Resolution scaled to the actual joint displacement rather than a fixed
    # stride. A broadly sampled ready pose can be most of a joint range away
    # from its target, and 40 samples over a large move can step straight
    # through an obstacle.
    if min_self_clearance_m is None:
        min_self_clearance_m = motion_limits.SELF_CLEARANCE_FLOOR_M
    stats = {}
    if approach_collides(validator, position, step_bounds,
                         counter=collision_counter, stats=stats):
        return dict({'feasible': False, 'reason_code': REASON_COLLISION,
                     'reason': 'direct quintic collides; a collision-free '
                               'approach may still exist around the obstacle',
                     'duration_s': duration}, **stats)

    # Deliberately not the collision step_bounds: the margin needs the finer
    # stride, and it is only paid on approaches that already passed collision.
    self_clearance = approach_self_clearance(validator, position)
    if self_clearance['min_distance_m'] < min_self_clearance_m:
        return dict({'feasible': False, 'reason_code': REASON_SELF_CLEARANCE,
                     'reason': 'approach passes within the self-clearance '
                               'floor of %.3f m' % min_self_clearance_m,
                     'self_clearance': self_clearance,
                     'duration_s': duration}, **stats)

    peak_jerk = np.max(np.abs(jerk), axis=0)
    if limit_statuses is None:
        limit_statuses = motion_limits.limit_statuses(validator)
    binding = binding_limit(velocity, acceleration, velocity_limits,
                            acceleration_limits, duration, limit_statuses,
                            duration_lower)
    integrated = np.trapz(np.abs(jerk), dx=duration / (len(jerk) - 1), axis=0)
    return dict({
        'feasible': True,
        'reason_code': None,
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
        'binding': binding,
        'self_clearance': self_clearance,
        'matches_entry_state': True,     # by construction of the quintic
    }, **stats)


# Rail distance is divided by this before comparison with the angular
# tolerance, so the default 0.35 admits 1.05 m of rail difference within one
# branch. Rail position is a continuous choice rather than a discrete IK
# branch, which is why it is weighted loosely.
BRANCH_RAIL_SCALE_M = 3.0


def branch_assignments(configurations, tolerance=0.35):
    """Branch label for each configuration, in INPUT order.

    Labels are order-invariant: the greedy pass runs in canonical order and a
    label is its representative's rank in that order, so shuffling the input
    permutes the labels with it and changes nothing else. A runner needs
    membership, not just a count, to stop once a branch has a feasible
    approach.

    Wrapped joint distance, so two lifts of one configuration share a label.
    """
    arrays = [np.asarray(c, dtype=float) for c in configurations]
    # Canonical order before the greedy pass. Greedy clustering is
    # order-dependent in general, and branch count is a primary ranking key.
    order = sorted(range(len(arrays)),
                   key=lambda i: tuple(np.round(arrays[i], 9)))

    representatives = []
    labels = [None] * len(arrays)
    for index in order:
        arm = arrays[index][ARM_SLICE]
        rail = float(arrays[index][RAIL_INDEX])
        for label, (other_rail, other_arm) in enumerate(representatives):
            wrapped = np.abs(np.angle(np.exp(1j * (arm - other_arm))))
            if (np.max(wrapped) <= tolerance
                    and abs(rail - other_rail) / BRANCH_RAIL_SCALE_M <= tolerance):
                labels[index] = label
                break
        else:
            labels[index] = len(representatives)
            representatives.append((rail, arm))
    return labels


def branch_clusters(configurations, tolerance=0.35):
    """Number of distinct solution branches among reached configurations.

    Reaching one layer-0 candidate is not connectivity: the graph needs
    alternatives, and a ready pose that can only enter through a single
    branch is fragile however cheap that entry is.
    """
    return len(set(branch_assignments(configurations, tolerance)))


# Rail continuation that decides whether two clusters are one IK family.
FAMILY_RAIL_STEP_M = 0.01
FAMILY_ARRIVAL_TOL_RAD = 1e-6
FAMILY_SOLVE_TOL = 1e-11          # converged fixed-rail solve, error norm
FAMILY_LOST_ERROR = 1e-7          # a step that cannot re-solve loses the family
FAMILY_LOST_CONDITION = 1e4       # nor may it pass through a singularity
FAMILY_SOLVE_ITERATIONS = 60
FAMILY_MAX_JOINT_STEP_RAD = 0.2


def arm_only_ik(validator, configuration, position, rotation):
    """Fixed-rail, damped Gauss-Newton solve from a nearby configuration.

    Returns (configuration, error norm, arm condition number). Used only to
    follow a solution continuously along the rail, where the previous step is
    always an excellent seed; it is not a general IK solver.
    """
    from scipy.spatial.transform import Rotation

    q = np.asarray(configuration, dtype=float).copy()
    error = np.inf
    for _ in range(FAMILY_SOLVE_ITERATIONS):
        pose = validator.robot.fkine(q, end='tool0')
        residual = np.concatenate([
            np.asarray(position) - pose.t,
            Rotation.from_matrix(rotation @ pose.R.T).as_rotvec()])
        error = float(np.linalg.norm(residual))
        if error < FAMILY_SOLVE_TOL:
            break
        jacobian = validator.compute_arm_jacobian(q)
        step = np.linalg.solve(jacobian.T @ jacobian + 1e-10 * np.eye(6),
                               jacobian.T @ residual)
        norm = np.linalg.norm(step)
        q[ARM_SLICE] += (step if norm < FAMILY_MAX_JOINT_STEP_RAD
                         else step * FAMILY_MAX_JOINT_STEP_RAD / norm)
    singular = np.linalg.svd(validator.compute_arm_jacobian(q), compute_uv=False)
    return q, error, float(singular[0] / max(singular[-1], 1e-15))


def _continues_to(validator, start, target, position, rotation):
    """Follow start along the rail to target's rail; does it arrive at target?"""
    q = np.asarray(start, dtype=float).copy()
    lower, upper = validator.robot.qlim
    worst = 0.0
    span = abs(target[RAIL_INDEX] - start[RAIL_INDEX])
    rails = np.linspace(start[RAIL_INDEX], target[RAIL_INDEX],
                        max(2, int(np.ceil(span / FAMILY_RAIL_STEP_M)) + 1))
    for rail in rails[1:]:
        q[RAIL_INDEX] = rail
        q, error, condition = arm_only_ik(validator, q, position, rotation)
        worst = max(worst, condition)
        # Only the elbow is bounded short of a full turn; the periodic joints
        # are compared wrapped, so their windings do not matter here.
        if (error > FAMILY_LOST_ERROR or condition > FAMILY_LOST_CONDITION
                or q[3] < lower[3] or q[3] > upper[3]):
            return False, worst
    arrival = np.max(np.abs(np.angle(np.exp(1j * (q[ARM_SLICE]
                                                  - target[ARM_SLICE])))))
    return bool(arrival < FAMILY_ARRIVAL_TOL_RAD), worst


def _family_signature(configuration):
    """Signs no continuous path can flip without passing a singularity."""
    return (int(np.sign(np.sin(configuration[3]))),
            int(np.sign(np.sin(configuration[5]))))


def ik_family_labels(validator, configurations, cluster_labels, position,
                     quaternion):
    """IK family per configuration, joining clusters by rail continuation.

    Each cluster is represented by its canonical-order first member, refined
    by a fixed-rail solve. Two clusters with the same elbow and wrist_2 signs
    are one family if either representative, followed along the rail to the
    other's rail position, arrives at it without losing the solution or
    passing a singularity. A cluster is assumed to lie within one family; its
    members are within the clustering tolerance of each other.

    Family links are KINEMATIC. They ignore collision, and a connecting path
    may pass condition numbers above the task gate, so "one family" does not
    mean the task can move between the two entries. A failed continuation is
    treated as two families, so counts can only come out high, never low.

    Returns (labels, report). Labels are ranks of each family's smallest
    cluster label, so they are invariant under input order.
    """
    from scipy.spatial.transform import Rotation

    rotation = Rotation.from_quat(quaternion).as_matrix()
    arrays = [np.asarray(c, dtype=float) for c in configurations]
    clusters = sorted(set(cluster_labels))
    representatives = {}
    refinement = 0.0
    for cluster in clusters:
        members = [a for a, label in zip(arrays, cluster_labels) if label == cluster]
        first = min(members, key=lambda c: tuple(np.round(c, 9)))
        representatives[cluster] = arm_only_ik(validator, first, position,
                                               rotation)[0]
        # Candidates are solutions of this pose already, so refinement should
        # barely move them. A large value means a configuration was passed
        # that is not a solution, and it may have been refined onto another
        # family.
        refinement = max(refinement, float(np.max(np.abs(np.angle(np.exp(
            1j * (representatives[cluster][ARM_SLICE] - first[ARM_SLICE])))))))

    parent = {c: c for c in clusters}

    def root(cluster):
        while parent[cluster] != cluster:
            cluster = parent[cluster]
        return cluster

    links, attempts = [], 0
    for index, a in enumerate(clusters):
        for b in clusters[index + 1:]:
            if root(a) == root(b):
                continue
            if (_family_signature(representatives[a])
                    != _family_signature(representatives[b])):
                continue
            attempts += 1
            same, worst = _continues_to(validator, representatives[a],
                                        representatives[b], position, rotation)
            if not same:
                same, other = _continues_to(validator, representatives[b],
                                            representatives[a], position,
                                            rotation)
                worst = max(worst, other)
            if same:
                parent[max(root(a), root(b))] = min(root(a), root(b))
                links.append({'clusters': [a, b],
                              'rail_gap_m': float(abs(
                                  representatives[a][RAIL_INDEX]
                                  - representatives[b][RAIL_INDEX])),
                              'worst_condition': worst})

    roots = sorted({root(c) for c in clusters})
    rank = {r: i for i, r in enumerate(roots)}
    labels = [rank[root(label)] for label in cluster_labels]
    return labels, {'families': len(roots), 'continuations_attempted': attempts,
                    'max_refinement_rad': refinement,
                    'links': links,
                    'signatures': {rank[r]: list(_family_signature(representatives[r]))
                                   for r in roots}}


def classify(task_candidates, approaches):
    """Three outcomes, kept distinct because they mean different things.

    no_task_candidate is NOT a statement about the ready pose, and must not
    count against its connectivity: there was nothing there to reach.

    direct_approach_unconnected is named precisely. A colliding direct quintic
    proves only that THIS approach fails, not that no collision-free approach
    exists; establishing the latter needs a planner that searches around the
    obstacle, which this does not do.
    """
    if not task_candidates:
        return NO_TASK_CANDIDATE
    if not any(a['feasible'] for a in approaches):
        return DIRECT_APPROACH_UNCONNECTED
    return CONNECTED


def failure_breakdown(approaches):
    """Count of each failure reason_code across infeasible approaches.

    Kept beside classify rather than inside it: the three-way classification
    stays stable, and the causes behind direct_approach_unconnected stay
    visible instead of collapsing into one label.
    """
    counts = {}
    for approach in approaches:
        if not approach['feasible']:
            code = approach.get('reason_code') or 'unknown'
            counts[code] = counts.get(code, 0) + 1
    return counts


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

    worst_duration_s is the slowest IK family's shortest approach at the worst
    placement, not the pose's single fastest entry.

    Jerk never excludes a candidate; it only orders candidates that are
    already feasible and equally connected.
    """
    def key(record):
        connectivity = (record['connectivity']
                        if record['connectivity'] is not None else -1.0)
        # Worst-case IK-FAMILY connectivity outranks duration: reaching one
        # layer-0 candidate is not the same as having alternatives, and a
        # ready pose with a single entry family is fragile however fast.
        # The FRACTION of each placement's families reached, minimum over
        # placements: a raw count's minimum is set by whichever placement has
        # fewest families, which says nothing about the pose. Tolerance
        # clusters are not counted: they split one family along the rail.
        branches = record.get('worst_family_fraction', 0.0) or 0.0
        return (-connectivity, -branches,
                record['worst_duration_s'] if record['worst_duration_s'] is not None else np.inf,
                record['worst_peak_jerk'] if record['worst_peak_jerk'] is not None else np.inf)

    return sorted([r for r in records if r['static_gates']['passed']], key=key)


def certification_note(validator=None):
    """Why nothing here certifies a ready pose, and under which limits.

    Pass the validator the sweep ran with, so the note records the velocity
    limits actually enforced rather than only the reference tables.
    """
    return {
        'envelope': 'PROVISIONAL_STAGE7_ENVELOPE_V2, a software robustness '
                    'envelope rather than the physical operating envelope',
        'limits': motion_limits.certification_status(validator),
        'effective_limits': (None if validator is None
                             else motion_limits.effective_limits(validator)),
        'may_certify_for_hardware': motion_limits.may_certify_for_hardware(
            validator),
        'result_status': 'provisional',
    }


# --------------------------------------------------------------------------
# Candidate pool: broad sampling, with tagged anchors as controls
# --------------------------------------------------------------------------

GLOBAL_SAMPLES = 4096
BROAD_FINALISTS = 64
RAIL_STRATA = 8

# Known postures kept as CONTROLS, never as replacements for broad finalists.
# If a sampler or evaluator breaks, these are the rows whose behaviour is
# already understood, so a nonsensical global result shows up against them.
# Their provenance is recorded so their influence stays visible.
ANCHOR_POSTURES_DEG = (
    ('upstream_default', (0.0, -90.0, 0.0, -90.0, 0.0, 0.0)),
    ('legacy_off_degeneracy', (0.0, -135.0, 90.0, -90.0, 45.0, 0.0)),
    ('elbow_up_wrist_turned', (90.0, -60.0, 60.0, -90.0, 90.0, 0.0)),
    ('elbow_down_mirrored', (-90.0, -120.0, -60.0, -60.0, -90.0, 45.0)),
    ('reach_forward', (0.0, -100.0, 110.0, -120.0, 60.0, -90.0)),
    ('reach_back', (-45.0, -160.0, 120.0, -40.0, -60.0, 90.0)),
    ('shoulder_reversed', (180.0, -90.0, 90.0, -90.0, 90.0, 180.0)),
    ('compact', (0.0, -75.0, 100.0, -115.0, -80.0, 0.0)),
)


def sample_pool(validator, count=GLOBAL_SAMPLES, seed=0):
    """Deterministic low-discrepancy samples over rail and CANONICAL arm space.

    Arm joints are sampled in [-pi, pi] rather than over their full +/-2*pi
    range. Sampling the wider range would produce the same physical posture
    several times under different windings, wasting the budget on duplicates;
    the feasible windings that matter are enumerated per destination during
    approach evaluation, where they actually depend on something.
    """
    from scipy.stats import qmc

    lower, upper = validator.robot.qlim
    engine = qmc.Sobol(d=7, scramble=True, seed=seed)
    unit = engine.random(count)

    low = np.concatenate(([lower[RAIL_INDEX]], np.full(6, -np.pi)))
    high = np.concatenate(([upper[RAIL_INDEX]], np.full(6, np.pi)))
    return qmc.scale(unit, low, high)


def wrapped_distance(first, second, rail_travel=3.0):
    """Normalised distance: wrapped angles, rail scaled by its own travel."""
    first, second = np.asarray(first, float), np.asarray(second, float)
    arm = np.abs(np.angle(np.exp(1j * (first[ARM_SLICE] - second[ARM_SLICE]))))
    rail = abs(first[RAIL_INDEX] - second[RAIL_INDEX]) / rail_travel
    return float(np.sqrt(rail ** 2 + np.sum(arm ** 2)))


def select_finalists(candidates, count=BROAD_FINALISTS, strata=RAIL_STRATA,
                     rail_travel=3.0):
    """Diverse finalists, stratified across the rail.

    Farthest-point selection within each stratum, so the finalists spread over
    posture space AND over rail position. Without stratification the diversity
    measure alone can cluster every finalist at one end of the travel.
    """
    candidates = np.asarray(candidates, dtype=float)
    per_stratum = max(1, count // strata)
    edges = np.linspace(0.0, rail_travel, strata + 1)

    chosen = []
    for index in range(strata):
        members = candidates[(candidates[:, RAIL_INDEX] >= edges[index])
                             & (candidates[:, RAIL_INDEX] <= edges[index + 1])]
        if not len(members):
            continue
        picked = [members[0]]
        while len(picked) < per_stratum and len(picked) < len(members):
            distances = [min(wrapped_distance(m, p, rail_travel) for p in picked)
                         for m in members]
            picked.append(members[int(np.argmax(distances))])
        chosen.extend(picked)
    return np.asarray(chosen)


def anchor_pool(rail_positions=(0.5, 1.5, 2.5)):
    """Tagged control configurations, with provenance.

    Deliberately separate from the broad finalists so their influence on any
    selection stays visible. They must never replace a broad finalist.
    """
    out = []
    for name, arm_deg in ANCHOR_POSTURES_DEG:
        for rail in rail_positions:
            out.append({
                'configuration': np.concatenate(([rail], np.deg2rad(arm_deg))),
                'provenance': 'anchor',
                'anchor_name': name,
            })
    return out


def build_candidate_pool(validator, count=GLOBAL_SAMPLES,
                         finalists=BROAD_FINALISTS, seed=0):
    """Broad finalists plus tagged anchors, each carrying its provenance.

    Operationally odd global samples are meant to be rejected by the explicit
    clearance, posture, joint-limit and connectivity criteria, NOT by biasing
    the sampler toward configurations this trajectory already visits. That
    bias is how a fixture-specific constant gets created.
    """
    sampled = sample_pool(validator, count, seed)
    admissible = [q for q in sampled if static_gates(validator, q)['passed']]
    broad = select_finalists(admissible, finalists) if admissible else np.empty((0, 7))

    pool = [{'configuration': q, 'provenance': 'broad_sample',
             'anchor_name': None} for q in broad]
    pool += [a for a in anchor_pool()
             if static_gates(validator, a['configuration'])['passed']]
    return pool, {'sampled': int(count), 'passed_static_gates': len(admissible),
                  'broad_finalists': len(broad),
                  'anchors_retained': len(pool) - len(broad)}
