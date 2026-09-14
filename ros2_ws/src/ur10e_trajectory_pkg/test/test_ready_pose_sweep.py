#!/usr/bin/env python3
"""Stage 7 gate: ready-pose selection coupled to the approach evaluator.

A ready pose cannot be ranked on static gates alone, so the evaluator has to
exist before a winner is picked. Three classifications are kept distinct
because one of them is not about the ready pose at all, and jerk never
rejects a candidate because its limit is assumed.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg import motion_limits
from ur10e_trajectory_pkg import ready_pose_sweep as sweep
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

ACCELERATION = motion_limits.acceleration_vector()


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description')
    )


@pytest.fixture(scope='module')
def VELOCITY(validator):
    """Per-joint ceilings read from the URDF, not a typed-in table."""
    return motion_limits.velocity_vector(validator)


@pytest.fixture(scope='module')
def ready():
    return np.concatenate(([1.0], np.deg2rad([0.0, -120.0, 90.0, -80.0, 60.0, 0.0])))


# --------------------------------------------------------------------------
# The envelope
# --------------------------------------------------------------------------

def test_the_envelope_has_29_deterministic_placements():
    first = sweep.placements()
    assert len(first) == 29
    names = [p['name'] for p in first]
    assert names[0] == 'nominal'
    assert len({*names}) == 29

    again = sweep.placements()
    for left, right in zip(first, again):
        np.testing.assert_allclose(left['translation'], right['translation'])


def test_the_envelope_is_named_provisional_not_physical():
    """A software robustness envelope, not the arena's operating envelope.

    Certification needs the permitted volume, attitude range, calibration
    uncertainty and an obstacle survey, none of which exist here.
    """
    assert 'PROVISIONAL' in 'PROVISIONAL_STAGE7_ENVELOPE_V2'
    assert sweep.PROVISIONAL_STAGE7_ENVELOPE_V2['scale'] == 1.0
    note = sweep.certification_note()
    assert note['result_status'] == 'provisional'
    assert note['may_certify_for_hardware'] is False


def test_the_certification_note_records_the_enforced_limits(validator):
    note = sweep.certification_note(validator)
    np.testing.assert_array_equal(note['effective_limits']['velocity_vector'],
                                  validator.velocity_limits)
    assert note['may_certify_for_hardware'] is False


def test_axis_extrema_reach_the_declared_bounds():
    envelope = sweep.PROVISIONAL_STAGE7_ENVELOPE_V2
    extremes = [p for p in sweep.placements() if p['name'].startswith(('translate', 'rotate'))]
    assert len(extremes) == 12
    assert max(np.max(np.abs(p['translation'])) for p in extremes) == pytest.approx(
        envelope['translation_m'])
    assert max(np.max(np.abs(p['rotation_deg'])) for p in extremes) == pytest.approx(
        envelope['rotation_deg'])


def _nominal_RG():
    from scipy.spatial.transform import Rotation
    from ur10e_trajectory_pkg import frames
    return frames.make_transform(
        rotation=Rotation.from_euler('xyz', [20.0, -35.0, 60.0], degrees=True).as_matrix(),
        translation=[1.0, 0.5, 0.5])


def test_rotation_turns_the_target_in_place():
    """V1 rotated about the rail-base origin, so a 15 degree attitude change
    moved the target by up to 0.46 m, under the floor or into the wall."""
    from scipy.spatial.transform import Rotation
    nominal = _nominal_RG()
    placement = {'name': 'rotate', 'translation': np.zeros(3),
                 'rotation_deg': np.array([15.0, -10.0, 5.0])}
    placed = sweep.placement_transform(placement, nominal)
    np.testing.assert_allclose(placed[:3, 3], nominal[:3, 3], atol=1e-12)
    # In WORLD axes: the rotation pre-multiplies the nominal orientation.
    expected = Rotation.from_euler('xyz', [15.0, -10.0, 5.0], degrees=True).as_matrix() @ nominal[:3, :3]
    np.testing.assert_allclose(placed[:3, :3], expected, atol=1e-12)


def test_translation_moves_the_target_by_exactly_the_offset():
    nominal = _nominal_RG()
    placement = {'name': 'both', 'translation': np.array([0.1, -0.2, 0.05]),
                 'rotation_deg': np.array([15.0, 15.0, -15.0])}
    placed = sweep.placement_transform(placement, nominal)
    np.testing.assert_allclose(placed[:3, 3] - nominal[:3, 3], [0.1, -0.2, 0.05],
                               atol=1e-12)


def test_no_placement_moves_the_target_beyond_the_declared_translation():
    nominal = _nominal_RG()
    bound = sweep.PROVISIONAL_STAGE7_ENVELOPE_V2['translation_m']
    for placement in sweep.placements():
        shift = sweep.placement_transform(placement, nominal)[:3, 3] - nominal[:3, 3]
        assert np.max(np.abs(shift)) <= bound + 1e-12, placement['name']


def test_the_envelope_records_its_rotation_centre_and_axes():
    envelope = sweep.PROVISIONAL_STAGE7_ENVELOPE_V2
    assert 'target' in envelope['rotation_centre']
    assert 'world' in envelope['rotation_axes']
    assert envelope['name'] == 'PROVISIONAL_STAGE7_ENVELOPE_V2'


# --------------------------------------------------------------------------
# Static gates are dimensionless and rankable
# --------------------------------------------------------------------------

def test_posture_margin_is_dimensionless_and_bounded(validator, ready):
    """The raw condition number mixes metres and radians, so it changes with
    the choice of length unit and cannot be ranked."""
    margin = sweep.posture_margin(validator, ready)
    assert 0.0 <= margin <= 1.0


def test_a_singular_posture_scores_zero_margin(validator):
    singular = np.concatenate(([1.0], np.deg2rad([0.0, -135.0, 90.0, -90.0, 0.0, 0.0])))
    assert sweep.posture_margin(validator, singular) < 1e-6


def test_joint_limit_clearance_is_a_fraction_of_each_range(validator):
    lower, upper = validator.robot.qlim
    middle = 0.5 * (lower + upper)
    assert sweep.joint_limit_clearance(validator, middle) == pytest.approx(0.5)
    assert sweep.joint_limit_clearance(validator, lower.copy()) == pytest.approx(0.0)


def test_collision_clearance_is_a_distance_not_a_flag(validator, ready):
    """Clearance is what separates two poses that are both collision-free."""
    distance = sweep.collision_distance(validator, ready)
    assert distance > 0.0
    assert not validator.check_all_collisions(ready)


# --------------------------------------------------------------------------
# Standardised retiming
# --------------------------------------------------------------------------

def test_the_quintic_matches_position_velocity_and_acceleration_at_entry(ready):
    """A join matching only position steps the velocity; matching velocity but
    not acceleration spikes the jerk exactly at handover."""
    target = ready + 0.1
    entry_velocity = np.full(7, 0.05)
    entry_acceleration = np.full(7, 0.02)
    duration = 3.0

    coefficients = sweep.quintic_coefficients(ready, target, entry_velocity,
                                              entry_acceleration, duration)
    _, position, velocity, acceleration, _ = sweep.sample_quintic(
        coefficients, duration, samples=200)

    np.testing.assert_allclose(position[0], ready, atol=1e-9)
    np.testing.assert_allclose(velocity[0], 0.0, atol=1e-9)
    np.testing.assert_allclose(acceleration[0], 0.0, atol=1e-9)
    np.testing.assert_allclose(position[-1], target, atol=1e-6)
    np.testing.assert_allclose(velocity[-1], entry_velocity, atol=1e-5)
    np.testing.assert_allclose(acceleration[-1], entry_acceleration, atol=1e-4)


def test_jerk_falls_as_duration_rises(ready):
    """Why timing has to be standardised before jerk can be compared.

    Jerk scales strongly with duration, so candidates timed differently cannot
    be ranked against one another at all.
    """
    target = ready + 0.2
    peaks = []
    for duration in (1.0, 2.0, 4.0):
        coefficients = sweep.quintic_coefficients(ready, target, np.zeros(7),
                                                  np.zeros(7), duration)
        _, _, _, _, jerk = sweep.sample_quintic(coefficients, duration, 200)
        peaks.append(float(np.max(np.abs(jerk))))
    assert peaks[0] > peaks[1] > peaks[2]


def test_minimum_duration_is_the_same_policy_for_every_candidate(ready, VELOCITY):
    target = ready + 0.3
    duration = sweep.minimum_duration(ready, target, np.zeros(7), np.zeros(7),
                                      VELOCITY, ACCELERATION)
    assert duration is not None

    coefficients = sweep.quintic_coefficients(ready, target, np.zeros(7),
                                              np.zeros(7), duration)
    _, _, velocity, acceleration, _ = sweep.sample_quintic(coefficients, duration)
    assert np.all(np.abs(velocity) <= VELOCITY + 1e-6)
    assert np.all(np.abs(acceleration) <= ACCELERATION + 1e-6)


def _quintic_feasible(start, end, ve, ae, duration, velocity, acceleration):
    coefficients = sweep.quintic_coefficients(start, end, ve, ae, duration)
    _, _, v, a, _ = sweep.sample_quintic(coefficients, duration)
    return (np.all(np.abs(v) <= velocity + 1e-12)
            and np.all(np.abs(a) <= acceleration + 1e-12))


def test_a_moving_entry_is_timed_inside_its_window_not_missed(ready, VELOCITY):
    """With a nonzero entry state a long approach swings away and back, so it
    fails where a moderate one passes. Here a 1 m rail move entering with 40%
    of the rail's acceleration limit is feasible only between about 2.3 and
    7.8 s. Testing the upper bound first and bisecting down returned None."""
    target = ready.copy()
    target[0] += 1.0
    zeros = np.zeros(7)
    entry_acceleration = np.zeros(7)
    entry_acceleration[0] = 0.4 * ACCELERATION[0]
    assert not _quintic_feasible(ready, target, zeros, entry_acceleration, 20.0,
                                 VELOCITY, ACCELERATION), 'fixture needs a window'

    duration = sweep.minimum_duration(ready, target, zeros, entry_acceleration,
                                      VELOCITY, ACCELERATION)
    assert duration is not None
    assert 2.0 < duration < 7.8
    assert _quintic_feasible(ready, target, zeros, entry_acceleration, duration,
                             VELOCITY, ACCELERATION)
    for shorter in np.arange(0.2, duration - 1e-6, 0.005):
        assert not _quintic_feasible(ready, target, zeros, entry_acceleration,
                                     shorter, VELOCITY, ACCELERATION)


def test_the_exact_duration_agrees_with_a_brute_force_scan(ready, VELOCITY):
    """No scan step to fall between. Random entry states and destinations; the
    exact answer must pass the sampled check, nothing on a fine grid below it
    may, and when it reports None nothing on the grid may pass at all."""
    rng = np.random.default_rng(7)
    for _ in range(12):
        target = ready + rng.uniform(-1.0, 1.0, 7) * np.concatenate(([0.8], np.full(6, 2.0)))
        entry_velocity = rng.uniform(-0.6, 0.6, 7) * VELOCITY
        entry_acceleration = rng.uniform(-0.6, 0.6, 7) * ACCELERATION
        duration = sweep.minimum_duration(ready, target, entry_velocity,
                                          entry_acceleration, VELOCITY,
                                          ACCELERATION)
        bound = sweep.duration_lower_bound(ready, target, VELOCITY)
        top = 20.0 if duration is None else duration - 1e-6
        grid = np.arange(bound, top, 0.004 if duration is not None else 0.02)
        assert not any(_quintic_feasible(ready, target, entry_velocity,
                                         entry_acceleration, T, VELOCITY,
                                         ACCELERATION) for T in grid)
        if duration is not None:
            assert _quintic_feasible(ready, target, entry_velocity,
                                     entry_acceleration, duration, VELOCITY,
                                     ACCELERATION)


def test_the_lower_bound_is_the_average_speed_limit(ready, VELOCITY):
    target = ready.copy()
    target[4] += 1.0
    assert sweep.duration_lower_bound(ready, target, VELOCITY) == pytest.approx(
        1.0 / VELOCITY[4])
    assert sweep.duration_lower_bound(ready, ready, VELOCITY) == 0.2


def test_an_entry_state_beyond_the_limits_has_no_duration(ready, VELOCITY):
    """No quintic can end in a state the joints cannot hold."""
    entry_velocity = np.zeros(7)
    entry_velocity[0] = 1.2 * VELOCITY[0]
    assert sweep.minimum_duration(ready, ready + 0.1, entry_velocity,
                                  np.zeros(7), VELOCITY, ACCELERATION) is None


def test_no_duration_is_shorter_than_the_average_speed_bound(ready, VELOCITY):
    """No joint can average more than its limit, which is what lets the scan
    start above the lower bound rather than at it."""
    target = ready.copy()
    target[4] += 2 * np.pi
    duration = sweep.minimum_duration(ready, target, np.zeros(7), np.zeros(7),
                                      VELOCITY, ACCELERATION)
    assert duration >= 2 * np.pi / VELOCITY[4]


# --------------------------------------------------------------------------
# Jerk ranks, never rejects
# --------------------------------------------------------------------------

def test_jerk_is_reported_against_several_references(validator, ready, VELOCITY):
    result = sweep.evaluate_approach(validator, ready, ready + 0.05,
                                     np.zeros(7), np.zeros(7),
                                     VELOCITY, ACCELERATION)
    assert result['feasible'] is True
    assert len(result['jerk_within_reference']) == len(sweep.JERK_REFERENCES_RAD_S3)
    assert 'peak_jerk' in result and 'integrated_jerk' in result


def test_a_feasible_approach_names_its_binding_limit(validator, ready, VELOCITY):
    """The minimum duration is where a bound becomes active, so the binding
    joint and kind are named, with the limit's provenance status."""
    target = ready.copy()
    target[5] += 1.5                       # wrist_2, velocity-limited move
    result = sweep.evaluate_approach(validator, ready, target, np.zeros(7),
                                     np.zeros(7), VELOCITY, ACCELERATION)
    assert result['feasible'] is True
    binding = result['binding']
    assert binding['active'] is True
    assert binding['ratio'] == pytest.approx(1.0, abs=1e-3)
    assert binding['joint'] in ('wrist_2_joint',)
    assert binding['kind'] in ('velocity', 'acceleration')
    expected = (motion_limits.ARM_VELOCITY['wrist_2_joint'].status
                if binding['kind'] == 'velocity'
                else motion_limits.ARM_ACCELERATION.status)
    assert binding['status'] == expected


def test_a_rail_binding_reports_the_rail_limits_status():
    velocity = np.zeros((10, 7))
    acceleration = np.zeros((10, 7))
    acceleration[3, 0] = 5.0
    binding = sweep.binding_limit(velocity, acceleration, np.ones(7),
                                  np.full(7, 5.0), duration=2.0)
    assert binding['joint'] == 'linear_rail_joint'
    assert binding['kind'] == 'acceleration'
    assert binding['status'] == motion_limits.RAIL_ACCELERATION.status


def test_exceeding_the_assumed_jerk_reference_does_not_make_it_infeasible(
        validator, ready, VELOCITY):
    """Nothing may be called hardware-infeasible for exceeding an assumed
    value. No public UR jerk limit exists."""
    result = sweep.evaluate_approach(validator, ready, ready + 0.4,
                                     np.zeros(7), np.zeros(7),
                                     VELOCITY, ACCELERATION)
    assert result['feasible'] is True
    assert motion_limits.ARM_JERK.status == motion_limits.ASSUMED


# --------------------------------------------------------------------------
# Classification and scoring
# --------------------------------------------------------------------------

def test_no_task_candidate_is_not_a_statement_about_the_ready_pose():
    """The generator found nothing there; the placement may still be feasible."""
    assert sweep.classify([], []) == sweep.NO_TASK_CANDIDATE


def test_a_colliding_quintic_only_proves_the_DIRECT_approach_fails():
    """Named precisely, because it does not prove what the shorter name would.

    A colliding direct quintic says nothing about whether a collision-free
    approach exists around the obstacle; establishing that needs a planner
    that searches, which this does not do.
    """
    assert sweep.classify([object()], [{'feasible': False}]) == \
        sweep.DIRECT_APPROACH_UNCONNECTED
    assert 'direct' in sweep.DIRECT_APPROACH_UNCONNECTED


def test_connected_requires_only_one_valid_approach():
    assert sweep.classify([object()], [{'feasible': False},
                                       {'feasible': True}]) == sweep.CONNECTED


def test_connectivity_ignores_placements_with_nothing_to_connect_to():
    """A placement where no task candidate exists must not count against a
    ready pose. There was nothing there to reach."""
    classifications = [sweep.CONNECTED, sweep.NO_TASK_CANDIDATE,
                       sweep.NO_TASK_CANDIDATE]
    assert sweep.connectivity_score(classifications) == pytest.approx(1.0)

    mixed = [sweep.CONNECTED, sweep.DIRECT_APPROACH_UNCONNECTED,
             sweep.NO_TASK_CANDIDATE]
    assert sweep.connectivity_score(mixed) == pytest.approx(0.5)


def test_connectivity_is_undefined_when_no_placement_is_eligible():
    assert sweep.connectivity_score([sweep.NO_TASK_CANDIDATE] * 3) is None


def test_ranking_puts_connectivity_before_duration_before_jerk():
    """Jerk only orders candidates already feasible and equally connected."""
    records = [
        {'name': 'well_connected_slow', 'connectivity': 1.0,
         'worst_duration_s': 5.0, 'worst_peak_jerk': 900.0,
         'static_gates': {'passed': True}},
        {'name': 'less_connected_fast', 'connectivity': 0.5,
         'worst_duration_s': 1.0, 'worst_peak_jerk': 10.0,
         'static_gates': {'passed': True}},
        {'name': 'well_connected_smooth', 'connectivity': 1.0,
         'worst_duration_s': 5.0, 'worst_peak_jerk': 50.0,
         'static_gates': {'passed': True}},
    ]
    order = [r['name'] for r in sweep.rank_ready_poses(records)]
    assert order[0] == 'well_connected_smooth'
    assert order[1] == 'well_connected_slow'
    assert order[2] == 'less_connected_fast'


def test_a_candidate_failing_static_gates_is_not_ranked_at_all():
    records = [{'name': 'unsafe', 'connectivity': 1.0, 'worst_duration_s': 1.0,
                'worst_peak_jerk': 1.0, 'static_gates': {'passed': False}}]
    assert sweep.rank_ready_poses(records) == []


# --------------------------------------------------------------------------
# The four fixes
# --------------------------------------------------------------------------

def test_self_collision_is_a_hard_static_gate(validator):
    """collision_distance sees the environment only, not the arm folded into
    itself, and a broadly sampled pool is full of such configurations."""
    folded = np.concatenate(([1.5], np.deg2rad([0.0, 80.0, 0.0, 0.0, 0.0, 0.0])))
    assert validator.check_all_collisions(folded)
    gates = sweep.static_gates(validator, folded)
    assert gates['in_collision'] is True
    assert gates['passed'] is False


def test_the_collision_resolution_bound_actually_holds(ready):
    """Assert the bound directly, not a proxy for it.

    Estimating a count from total variation and sampling uniformly in time
    does not bound the coordinate difference: a quintic has nonuniform speed,
    so a uniform stride oversamples where the motion is slow and undersamples
    exactly where it is fastest, which is where an obstacle is most likely to
    be stepped over.
    """
    target = ready + np.array([1.2, 2.0, -2.5, 1.8, -2.2, 2.4, -1.9])
    coefficients = sweep.quintic_coefficients(ready, target, np.zeros(7),
                                              np.zeros(7), 3.0)
    _, position, _, _, _ = sweep.sample_quintic(coefficients, 3.0, samples=2000)

    for scale in (0.4, 1.0, 4.0):
        bounds = sweep.collision_step_bounds() * scale
        indices = sweep.collision_check_indices(position, bounds)
        assert sweep.achieved_step(position, indices, bounds) <= 1.0 + 1e-9, (
            'a checked pair moved further than its own per-joint bound'
        )


def test_a_tighter_bound_checks_more_points(ready):
    target = ready + 1.0
    coefficients = sweep.quintic_coefficients(ready, target, np.zeros(7),
                                              np.zeros(7), 3.0)
    _, position, _, _, _ = sweep.sample_quintic(coefficients, 3.0, samples=2000)
    tight = sweep.collision_step_bounds() * 0.2
    loose = sweep.collision_step_bounds() * 4.0
    assert (len(sweep.collision_check_indices(position, tight))
            > len(sweep.collision_check_indices(position, loose)))


def test_resolution_follows_the_move_size(ready):
    """A larger move must draw more checks at the same bound."""
    small = sweep.sample_quintic(
        sweep.quintic_coefficients(ready, ready + 0.02, np.zeros(7),
                                   np.zeros(7), 3.0), 3.0, 2000)[1]
    large = sweep.sample_quintic(
        sweep.quintic_coefficients(ready, ready + 2.0, np.zeros(7),
                                   np.zeros(7), 3.0), 3.0, 2000)[1]
    assert (len(sweep.collision_check_indices(large))
            > len(sweep.collision_check_indices(small)))


# --------------------------------------------------------------------------
# Entry state and winding expansion
# --------------------------------------------------------------------------

def test_the_entry_state_needs_three_layers_not_one():
    """PCHIP fixes its initial derivatives from the first three points.

    Matching only layer 0 is valid solely if the robot stops there and the
    task starts from rest, which would need an explicit ramp-in; without one
    it reintroduces the discontinuity the quintic exists to avoid.
    """
    prefix = np.stack([np.full(7, 0.1 * i) for i in range(3)])
    velocity, acceleration = sweep.entry_state_from_prefix(prefix, 0.1)
    assert velocity.shape == (7,)
    assert acceleration.shape == (7,)
    assert np.any(np.abs(velocity) > 1e-9), 'a moving task cannot enter at rest'

    with pytest.raises(ValueError, match='three layers'):
        sweep.entry_state_from_prefix(prefix[:2], 0.1)


def test_the_entry_state_depends_on_the_continuation(ready):
    """Two prefixes sharing layer 0 but diverging after it must differ.

    This is why layer 0 alone is not enough to time the approach.
    """
    base = np.stack([ready, ready + 0.05, ready + 0.10])
    other = np.stack([ready, ready + 0.05, ready + 0.30])
    first, _ = sweep.entry_state_from_prefix(base, 0.1)
    second, _ = sweep.entry_state_from_prefix(other, 0.1)
    assert not np.allclose(first, second)


def test_winding_expansion_belongs_to_the_runner(validator, ready, VELOCITY):
    """evaluate_approach evaluates one supplied representation.

    Which lifts are reachable depends on where the approach starts and how
    long it has, so they cannot be enumerated when a candidate is generated.
    """
    target = ready.copy()
    target[4] += 2 * np.pi
    lifts = sweep.destination_lifts(validator, target, ready, VELOCITY, 5.0)
    assert lifts, 'the lifted destination is reachable and must be offered'
    assert any(abs(lift[4] - ready[4]) < 0.5 for lift in lifts)


def test_branch_clusters_count_distinct_solutions_not_lifts():
    """Reaching one candidate is not connectivity; the graph needs
    alternatives, and two lifts of one configuration are not two."""
    base = np.concatenate(([1.0], np.deg2rad([0.0, -120.0, 90.0, -80.0, 60.0, 0.0])))
    lifted = base.copy()
    lifted[4] += 2 * np.pi
    other = np.concatenate(([1.0], np.deg2rad([60.0, -90.0, -60.0, 40.0, -70.0, 120.0])))

    assert sweep.branch_clusters([base, lifted]) == 1
    assert sweep.branch_clusters([base, other]) == 2


def test_ranking_puts_branch_connectivity_before_duration():
    """A single entry branch is fragile however fast it is."""
    records = [
        {'name': 'one_branch_fast', 'connectivity': 1.0, 'worst_family_fraction': 0.5,
         'worst_duration_s': 1.0, 'worst_peak_jerk': 10.0,
         'static_gates': {'passed': True}},
        {'name': 'three_branches_slow', 'connectivity': 1.0,
         'worst_family_fraction': 1.0, 'worst_duration_s': 4.0,
         'worst_peak_jerk': 80.0, 'static_gates': {'passed': True}},
    ]
    assert sweep.rank_ready_poses(records)[0]['name'] == 'three_branches_slow'


# --------------------------------------------------------------------------
# The candidate pool
# --------------------------------------------------------------------------

def test_sampling_avoids_duplicate_winding_representations(validator):
    """Sampling the full +/-2*pi range would spend the budget on the same
    physical posture under different windings.

    The windings that matter depend on the destination, so they are enumerated
    during approach evaluation instead.
    """
    samples = sweep.sample_pool(validator, count=64, seed=0)
    assert np.all(np.abs(samples[:, 1:]) <= np.pi + 1e-9)


def test_sampling_is_deterministic(validator):
    first = sweep.sample_pool(validator, count=32, seed=0)
    np.testing.assert_allclose(first, sweep.sample_pool(validator, 32, seed=0))


def test_finalists_are_stratified_across_the_rail(validator):
    """Diversity alone can cluster every finalist at one end of the travel."""
    rng = np.random.default_rng(0)
    candidates = np.column_stack([
        rng.uniform(0.0, 3.0, 400),
        rng.uniform(-np.pi, np.pi, (400, 6)).reshape(400, 6),
    ])
    finalists = sweep.select_finalists(candidates, count=24, strata=6)
    occupied = np.unique(np.clip((finalists[:, 0] / 3.0 * 6).astype(int), 0, 5))
    assert len(occupied) >= 5


def test_anchors_are_tagged_and_never_replace_broad_finalists():
    """Controls, kept visible. If a sampler or evaluator breaks, these are the
    rows whose behaviour is already understood."""
    anchors = sweep.anchor_pool()
    assert anchors
    assert all(a['provenance'] == 'anchor' for a in anchors)
    assert all(a['anchor_name'] for a in anchors)
    assert len({a['anchor_name'] for a in anchors}) == len(sweep.ANCHOR_POSTURES_DEG)


def test_the_resolution_bound_is_per_joint_not_one_shared_number():
    """0.05 applied across the vector means metres for the rail and radians
    for the arm, which are different quantities sharing a number."""
    bounds = sweep.collision_step_bounds()
    assert bounds[0] == sweep.COLLISION_STEP_RAIL_M
    assert np.all(bounds[1:] == sweep.COLLISION_STEP_ARM_RAD)
    assert len(bounds) == 7


def test_a_rail_only_move_is_bounded_in_metres(ready):
    """The rail bound must apply to the rail, whatever the arm bound is."""
    target = ready.copy()
    target[0] += 1.0
    coefficients = sweep.quintic_coefficients(ready, target, np.zeros(7),
                                              np.zeros(7), 3.0)
    _, position, _, _, _ = sweep.sample_quintic(coefficients, 3.0, 2000)
    indices = sweep.collision_check_indices(position)
    rail_steps = np.abs(np.diff(position[indices][:, 0]))
    assert np.max(rail_steps) <= sweep.COLLISION_STEP_RAIL_M + 1e-9


def test_branch_clustering_is_invariant_under_input_order():
    """Greedy clustering is order-dependent in general, and branch count is
    now a primary ranking key."""
    rng = np.random.default_rng(0)
    configurations = [np.concatenate(([rng.uniform(0, 3)],
                                      rng.uniform(-np.pi, np.pi, 6)))
                      for _ in range(40)]
    reference = sweep.branch_clusters(configurations)
    for seed in range(5):
        shuffled = list(configurations)
        np.random.default_rng(seed).shuffle(shuffled)
        assert sweep.branch_clusters(shuffled) == reference


def test_branch_count_sensitivity_to_tolerance_is_reported():
    """The count depends on the tolerance, so nearby values must be checked
    rather than one figure trusted."""
    rng = np.random.default_rng(1)
    configurations = [np.concatenate(([rng.uniform(0, 3)],
                                      rng.uniform(-np.pi, np.pi, 6)))
                      for _ in range(40)]
    counts = [sweep.branch_clusters(configurations, tolerance=t)
              for t in (0.25, 0.35, 0.50)]
    assert counts == sorted(counts, reverse=True), (
        'a looser tolerance must not increase the cluster count'
    )


# --------------------------------------------------------------------------
# Runner interfaces: membership, reasons, and one enforced prefix lift
# --------------------------------------------------------------------------

def test_branch_assignments_give_membership_invariant_under_input_order():
    """A runner stopping once a branch has a feasible approach needs to know
    WHICH branch each candidate is in, not just how many there are."""
    rng = np.random.default_rng(2)
    configurations = [np.concatenate(([rng.uniform(0, 3)],
                                      rng.uniform(-np.pi, np.pi, 6)))
                      for _ in range(40)]
    reference = sweep.branch_assignments(configurations)
    assert len(reference) == len(configurations)
    assert sweep.branch_clusters(configurations) == len(set(reference))
    for seed in range(5):
        permutation = np.random.default_rng(seed).permutation(len(configurations))
        shuffled = [configurations[i] for i in permutation]
        labels = sweep.branch_assignments(shuffled)
        assert labels == [reference[i] for i in permutation]


def test_two_lifts_of_one_configuration_share_a_branch_label():
    base = np.concatenate(([1.0], np.deg2rad([0.0, -120.0, 90.0, -80.0, 60.0, 0.0])))
    lifted = base.copy()
    lifted[4] += 2 * np.pi
    labels = sweep.branch_assignments([base, lifted])
    assert labels[0] == labels[1]


def test_an_approach_with_no_feasible_duration_says_so(validator, ready, VELOCITY):
    target = ready.copy()
    target[0] += 100.0
    result = sweep.evaluate_approach(validator, ready, target, np.zeros(7),
                                     np.zeros(7), VELOCITY, ACCELERATION)
    assert result['feasible'] is False
    assert result['reason_code'] == sweep.REASON_NO_DURATION


def test_an_approach_leaving_joint_limits_says_so(validator, ready, VELOCITY):
    target = ready.copy()
    target[3] = np.pi + 0.2            # elbow beyond its +/-pi limit
    result = sweep.evaluate_approach(validator, ready, target, np.zeros(7),
                                     np.zeros(7), VELOCITY, ACCELERATION)
    assert result['feasible'] is False
    assert result['reason_code'] == sweep.REASON_JOINT_LIMITS


def test_a_colliding_approach_counts_the_queries_actually_made(
        validator, ready, VELOCITY, monkeypatch):
    """The counter used to be unreachable from evaluate_approach, and it
    reported planned queries even when the first one collided."""
    monkeypatch.setattr(validator, 'check_all_collisions',
                        lambda q, verbose=False: True)
    counter = []
    result = sweep.evaluate_approach(validator, ready, ready + 0.3, np.zeros(7),
                                     np.zeros(7), VELOCITY, ACCELERATION,
                                     collision_counter=counter)
    assert result['feasible'] is False
    assert result['reason_code'] == sweep.REASON_COLLISION
    assert counter == [1]
    assert result['collision_queries'] == 1
    assert result['collision_queries_planned'] > 1


def test_a_feasible_approach_reports_its_collision_resolution(
        validator, ready, VELOCITY):
    counter = []
    result = sweep.evaluate_approach(validator, ready, ready + 0.05, np.zeros(7),
                                     np.zeros(7), VELOCITY, ACCELERATION,
                                     collision_counter=counter)
    assert result['feasible'] is True
    assert result['reason_code'] is None
    assert counter == [result['collision_queries']]
    assert result['collision_queries'] == result['collision_queries_planned']
    assert result['collision_achieved_step'] <= 1.0 + 1e-9


def test_failure_breakdown_keeps_the_causes_apart():
    approaches = [
        {'feasible': False, 'reason_code': sweep.REASON_COLLISION},
        {'feasible': False, 'reason_code': sweep.REASON_COLLISION},
        {'feasible': False, 'reason_code': sweep.REASON_NO_DURATION},
        {'feasible': True, 'reason_code': None},
    ]
    assert sweep.classify([object()], approaches) == sweep.CONNECTED
    assert sweep.failure_breakdown(approaches) == {
        sweep.REASON_COLLISION: 2, sweep.REASON_NO_DURATION: 1}


def _wrapping_prefix(ready):
    """Canonical candidates whose wrist_3 crosses +pi between layers."""
    prefix = np.stack([ready.copy() for _ in range(3)])
    prefix[:, 6] = [np.pi - 0.02, -np.pi + 0.02, -np.pi + 0.06]
    return prefix


def test_an_unlifted_prefix_is_refused_rather_than_differentiated(ready):
    """Differentiating across the wrap gives about 60 rad/s, minimum_duration
    returns None, and the ready pose is silently called unconnected."""
    with pytest.raises(ValueError, match='not continuous'):
        sweep.entry_state_from_prefix(_wrapping_prefix(ready), 0.1)


def test_lifted_prefix_removes_the_wrap(validator, ready):
    lifted = sweep.lifted_prefix(validator, _wrapping_prefix(ready))
    assert lifted is not None
    np.testing.assert_allclose(lifted[:, 6],
                               [np.pi - 0.02, np.pi + 0.02, np.pi + 0.06])
    velocity, _ = sweep.entry_state_from_prefix(lifted, 0.1)
    assert abs(velocity[6]) < 1.0


def test_a_lifted_destination_carries_the_whole_prefix_with_it(validator, ready):
    """Shifting layer 0 by a full turn without layers 1 and 2 reintroduces the
    discontinuity one layer later."""
    prefix = np.stack([ready, ready + 0.01, ready + 0.02])
    destination = prefix[0].copy()
    destination[4] += 2 * np.pi
    lifted = sweep.lifted_prefix(validator, prefix, destination)
    assert lifted is not None
    np.testing.assert_allclose(lifted[:, 4] - prefix[:, 4], 2 * np.pi, atol=1e-12)
    np.testing.assert_allclose(lifted[:, 5], prefix[:, 5], atol=1e-12)


def test_a_destination_that_is_not_a_lift_is_refused(validator, ready):
    prefix = np.stack([ready, ready + 0.01, ready + 0.02])
    moved = prefix[0].copy()
    moved[4] += 0.5
    with pytest.raises(ValueError, match='not a lift'):
        sweep.lifted_prefix(validator, prefix, moved)
    rail_moved = prefix[0].copy()
    rail_moved[0] += 0.1
    with pytest.raises(ValueError, match='rail'):
        sweep.lifted_prefix(validator, prefix, rail_moved)


def test_a_lift_whose_continuation_leaves_the_limits_is_invalid(validator, ready):
    """Near the limit, the shifted layer 0 fits and layer 2 does not, so this
    representation has no valid continuation."""
    prefix = np.stack([ready.copy() for _ in range(3)])
    prefix[:, 4] = [-0.1, -0.05, 0.2]
    destination = prefix[0].copy()
    destination[4] += 2 * np.pi
    assert sweep.lifted_prefix(validator, prefix, destination) is None


# --------------------------------------------------------------------------
# IK families
# --------------------------------------------------------------------------

def _pose(validator, configuration):
    from scipy.spatial.transform import Rotation
    pose = validator.robot.fkine(configuration, end='tool0')
    return pose.t, Rotation.from_matrix(pose.R).as_quat(), pose.R


def test_two_rail_samples_of_one_family_share_a_label(validator, ready):
    """Tolerance clusters split a family whenever the arm moves along the
    rail; continuation along the rail joins them again."""
    position, quaternion, rotation = _pose(validator, ready)
    moved = ready.copy()
    moved[0] += 0.6
    moved, error, _ = sweep.arm_only_ik(validator, moved, position, rotation)
    assert error < 1e-9
    assert np.max(np.abs(moved[1:] - ready[1:])) > 0.35, 'must be two clusters'

    configurations = [ready, moved]
    clusters = sweep.branch_assignments(configurations)
    assert clusters[0] != clusters[1]
    families, report = sweep.ik_family_labels(validator, configurations, clusters,
                                              position, quaternion)
    assert families[0] == families[1]
    assert report['families'] == 1
    assert report['links'][0]['rail_gap_m'] == pytest.approx(0.6, abs=1e-9)


def test_same_rail_solutions_with_different_shoulders_are_two_families(validator,
                                                                       ready):
    """Same pose, same rail, same elbow and wrist_2 signs, 2.1 rad apart:
    no rail continuation turns one into the other."""
    position, quaternion, rotation = _pose(validator, ready)
    seed = np.concatenate(([ready[0]],
                           [-2.0933, -2.6974, 1.3721, -0.5829, 2.1016, 1.9063]))
    other, error, _ = sweep.arm_only_ik(validator, seed, position, rotation)
    assert error < 1e-9
    assert sweep._family_signature(other) == sweep._family_signature(ready)
    assert np.max(np.abs(np.angle(np.exp(1j * (other[1:] - ready[1:]))))) > 2.0

    configurations = [ready, other]
    clusters = sweep.branch_assignments(configurations)
    families, report = sweep.ik_family_labels(validator, configurations, clusters,
                                              position, quaternion)
    assert families[0] != families[1]
    assert report['continuations_attempted'] == 1


def test_family_labels_are_invariant_under_input_order(validator, ready):
    position, quaternion, rotation = _pose(validator, ready)
    moved = ready.copy()
    moved[0] += 0.6
    moved = sweep.arm_only_ik(validator, moved, position, rotation)[0]
    seed = np.concatenate(([ready[0]],
                           [-2.0933, -2.6974, 1.3721, -0.5829, 2.1016, 1.9063]))
    other = sweep.arm_only_ik(validator, seed, position, rotation)[0]

    configurations = [ready, moved, other]
    reference, _ = sweep.ik_family_labels(
        validator, configurations, sweep.branch_assignments(configurations),
        position, quaternion)
    shuffled = [other, ready, moved]
    labels, _ = sweep.ik_family_labels(
        validator, shuffled, sweep.branch_assignments(shuffled),
        position, quaternion)
    assert labels == [reference[2], reference[0], reference[1]]

