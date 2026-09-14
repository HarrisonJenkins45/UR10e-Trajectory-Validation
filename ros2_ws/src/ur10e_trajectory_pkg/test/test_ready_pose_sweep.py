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

VELOCITY = motion_limits.velocity_vector()
ACCELERATION = motion_limits.acceleration_vector()


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description')
    )


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
    assert 'PROVISIONAL' in 'PROVISIONAL_STAGE7_ENVELOPE_V1'
    assert sweep.PROVISIONAL_STAGE7_ENVELOPE_V1['scale'] == 1.0
    note = sweep.certification_note()
    assert note['result_status'] == 'provisional'
    assert note['may_certify_for_hardware'] is False


def test_axis_extrema_reach_the_declared_bounds():
    envelope = sweep.PROVISIONAL_STAGE7_ENVELOPE_V1
    extremes = [p for p in sweep.placements() if p['name'].startswith(('translate', 'rotate'))]
    assert len(extremes) == 12
    assert max(np.max(np.abs(p['translation'])) for p in extremes) == pytest.approx(
        envelope['translation_m'])
    assert max(np.max(np.abs(p['rotation_deg'])) for p in extremes) == pytest.approx(
        envelope['rotation_deg'])


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


def test_minimum_duration_is_the_same_policy_for_every_candidate(ready):
    target = ready + 0.3
    duration = sweep.minimum_duration(ready, target, np.zeros(7), np.zeros(7),
                                      VELOCITY, ACCELERATION)
    assert duration is not None

    coefficients = sweep.quintic_coefficients(ready, target, np.zeros(7),
                                              np.zeros(7), duration)
    _, _, velocity, acceleration, _ = sweep.sample_quintic(coefficients, duration)
    assert np.all(np.abs(velocity) <= VELOCITY + 1e-6)
    assert np.all(np.abs(acceleration) <= ACCELERATION + 1e-6)


# --------------------------------------------------------------------------
# Jerk ranks, never rejects
# --------------------------------------------------------------------------

def test_jerk_is_reported_against_several_references(validator, ready):
    result = sweep.evaluate_approach(validator, ready, ready + 0.05,
                                     np.zeros(7), np.zeros(7),
                                     VELOCITY, ACCELERATION)
    assert result['feasible'] is True
    assert len(result['jerk_within_reference']) == len(sweep.JERK_REFERENCES_RAD_S3)
    assert 'peak_jerk' in result and 'integrated_jerk' in result


def test_exceeding_the_assumed_jerk_reference_does_not_make_it_infeasible(
        validator, ready):
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


def test_approach_collision_resolution_follows_the_move_size(validator, ready):
    """A fixed stride coarsens silently as moves grow.

    A broadly sampled ready pose can sit most of a joint range from its
    target, where 40 samples can step straight through an obstacle.
    """
    small = np.stack([ready + i * 1e-4 for i in range(400)])
    large = np.stack([ready + i * 5e-3 for i in range(400)])
    calls = {'n': 0}
    original = validator.check_all_collisions

    def counting(q, verbose=False):
        calls['n'] += 1
        return original(q, verbose)

    validator.check_all_collisions = counting
    try:
        calls['n'] = 0
        sweep.approach_collides(validator, small)
        small_calls = calls['n']
        calls['n'] = 0
        sweep.approach_collides(validator, large)
        large_calls = calls['n']
    finally:
        validator.check_all_collisions = original

    assert large_calls > small_calls


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
        {'name': 'one_branch_fast', 'connectivity': 1.0, 'worst_branch_count': 1,
         'worst_duration_s': 1.0, 'worst_peak_jerk': 10.0,
         'static_gates': {'passed': True}},
        {'name': 'three_branches_slow', 'connectivity': 1.0,
         'worst_branch_count': 3, 'worst_duration_s': 4.0,
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
