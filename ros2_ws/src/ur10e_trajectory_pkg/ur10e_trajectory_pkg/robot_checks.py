"""Runtime geometry and IK checks used by home validation and warmup."""

import numpy as np

from ur10e_trajectory_pkg import motion_limits
from ur10e_trajectory_pkg.configurations import ARM_SLICE

FAMILY_SOLVE_TOL = 1e-11
FAMILY_SOLVE_ITERATIONS = 60
FAMILY_MAX_JOINT_STEP_RAD = 0.2

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


# Self-clearance needs a finer stride than a boolean collision query: a
# coarse sample can miss the minimum distance between links.
SELF_CLEARANCE_STEP_RAIL_M = 0.01
SELF_CLEARANCE_STEP_ARM_RAD = 0.01


def collision_step_bounds():
    """Per-joint resolution bound, in JOINT_NAMES order."""
    return np.array([COLLISION_STEP_RAIL_M] + [COLLISION_STEP_ARM_RAD] * 6)


def self_clearance_step_bounds():
    """Per-joint resolution bound for the clearance query, finer than above."""
    return np.array([SELF_CLEARANCE_STEP_RAIL_M]
                    + [SELF_CLEARANCE_STEP_ARM_RAD] * 6)


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
    """Whether the configured home pose is admissible.

    collision_distance measures robot-to-ENVIRONMENT clearance only, so it
    cannot see the arm folded into itself. The boolean self-collision check
    is therefore a separate hard gate.

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
