#!/usr/bin/env python3
"""Stage 1B.5 gate: the census observes without disturbing, and says what it
actually measured.

A diagnostic that changes the thing it measures is worse than none, and a
census that reports segment membership as feasibility would restate the
confusion it exists to resolve.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg import failure_census
from ur10e_trajectory_pkg.configurations import LEGACY_MATLAB_START_Q
from ur10e_trajectory_pkg.pose_metrics import (
    IK_ORIENTATION_TOL_RAD,
    IK_POSITION_TOL_M,
)
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

WAYPOINTS = 40
DT = 0.02


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description')
    )


@pytest.fixture(scope='module')
def trajectory(validator):
    """A short synthetic arc, reachable at mid-rail.

    Synthetic rather than the real CSV so the test stays fast and independent
    of the packaged data file.
    """
    reference = validator.robot.fkine(
        np.concatenate(([1.5], np.deg2rad([0.0, -135.0, 90.0, -90.0, 45.0, 0.0]))),
        end='tool0',
    )
    quaternion = np.roll(np.array(reference.UnitQuaternion().A), -1)
    positions = np.tile(reference.t, (WAYPOINTS, 1))
    positions[:, 0] += np.linspace(0.0, 0.20, WAYPOINTS)
    return positions, np.tile(quaternion, (WAYPOINTS, 1))


def _segments(validator, trajectory, recorder=None):
    positions, quaternions = trajectory
    return validator.find_feasible_segments(
        positions[:, 0], positions[:, 1], positions[:, 2], quaternions,
        LEGACY_MATLAB_START_Q, min_length=5, dt_waypoint=DT, verbose=False,
        recorder=recorder,
    )


def test_recording_does_not_change_the_result(validator, trajectory):
    """The production answer must be identical with diagnostics on.

    This is what makes the census evidence about the pipeline rather than
    about the census.
    """
    validator.reset_rng()
    without = _segments(validator, trajectory)

    records = []
    validator.reset_rng()
    with_recording = _segments(validator, trajectory, recorder=records.append)

    assert records, 'recorder captured nothing'
    assert len(without) == len(with_recording)
    for plain, observed in zip(without, with_recording):
        assert plain['start_idx'] == observed['start_idx']
        assert plain['end_idx'] == observed['end_idx']
        assert plain['length'] == observed['length']


def test_records_are_deterministic(validator, trajectory):
    first, second = [], []
    validator.reset_rng()
    _segments(validator, trajectory, recorder=first.append)
    validator.reset_rng()
    _segments(validator, trajectory, recorder=second.append)

    assert len(first) == len(second)
    for left, right in zip(first, second):
        assert left['waypoint_index'] == right['waypoint_index']
        assert left['attempt'] == right['attempt']
        assert left['accepted'] == right['accepted']
        np.testing.assert_allclose(left['seed_arm'], right['seed_arm'], atol=1e-12)


def test_record_count_matches_actual_solver_calls(validator, trajectory):
    """Attempt accounting has to be trustworthy for the census to mean
    anything."""
    calls = []
    original = validator.solve_ik_lm
    validator.solve_ik_lm = lambda *a, **k: (calls.append(1), original(*a, **k))[1]
    records = []
    try:
        validator.reset_rng()
        _segments(validator, trajectory, recorder=records.append)
    finally:
        validator.solve_ik_lm = original

    assert len(records) == len(calls)


def test_failed_solves_are_recorded_not_dropped(validator, trajectory):
    """evaluate() returns None on solver failure.

    If the census only saw its return value, every failed solve would vanish
    from the record exactly when it matters most.
    """
    positions, quaternions = trajectory
    unreachable = np.array([9.0, positions[0, 1], positions[0, 2]])
    records = []
    validator.reset_rng()
    validator._solve_waypoint_with_recovery(
        unreachable, quaternions[0], np.deg2rad([0.0, -135.0, 90.0, -90.0, 45.0, 0.0]),
        rail_pos=1.5, check_jump=False, verbose=False, recorder=records.append,
    )
    assert records
    assert all(record['position_error_m'] is not None for record in records)

    # These used to be ACCEPTED. The solver reports success from a local
    # minimum metres away, because its flag measures convergence of the local
    # search rather than distance to target, and acceptance checked only that
    # flag. The stage 2b pose gate closes it.
    assert not any(record['accepted'] for record in records)
    assert all(record['gate_pose_position'] for record in records)
    assert all(record['position_error_m'] > 1.0 for record in records)
    assert not any(failure_census.viable(record) for record in records)


def test_every_gate_is_recorded_independently(validator, trajectory):
    """Simultaneous failures must survive.

    evaluate() collapses arm and rail velocity into one flag, and
    _failure_reason applies precedence and describes only the last attempt.
    Neither can reconstruct a waypoint that was both singular and colliding.
    """
    records = []
    validator.reset_rng()
    _segments(validator, trajectory, recorder=records.append)
    for key in failure_census.GATE_KEYS:
        assert all(key in record for record in records)


def test_raw_pose_errors_are_stored_for_every_finite_attempt(validator, trajectory):
    """Stored raw, never thresholded, so stage 2b can change tolerances
    without re-running the census."""
    records = []
    validator.reset_rng()
    _segments(validator, trajectory, recorder=records.append)
    finite = [r for r in records if r['configuration_finite']]
    assert finite
    for record in finite:
        assert isinstance(record['position_error_m'], float)
        assert isinstance(record['orientation_error_rad'], float)
        assert 0.0 <= record['orientation_error_rad'] <= np.pi


def test_entry_kind_distinguishes_fresh_from_continuation(validator, trajectory):
    """Fresh entries and continuations are different questions.

    Conflating them is what makes the largest-segment length read as a count
    of independently feasible waypoints.
    """
    records = []
    validator.reset_rng()
    _segments(validator, trajectory, recorder=records.append)
    kinds = {record['entry_kind'] for record in records}
    assert kinds <= {'fresh', 'continuation'}
    assert 'fresh' in kinds


# --------------------------------------------------------------------------
# Viability: conditioning alone is never enough
# --------------------------------------------------------------------------

def _record(**overrides):
    base = dict(
        configuration_finite=True,
        gate_solver_failed=False,
        position_error_m=0.0,
        orientation_error_rad=0.0,
        gate_pose_position=False,
        gate_pose_orientation=False,
        gate_singular=False,
        gate_arm_velocity=None,
        gate_rail_velocity=None,
        gate_collision=False,
    )
    base.update(overrides)
    return base


def test_viable_requires_both_pose_errors_and_all_gates():
    assert failure_census.viable(_record())


@pytest.mark.parametrize('overrides', [
    {'position_error_m': IK_POSITION_TOL_M * 10,
     'gate_pose_position': True},
    {'orientation_error_rad': IK_ORIENTATION_TOL_RAD * 10,
     'gate_pose_orientation': True},
    {'gate_singular': True},
    {'gate_collision': True},
    {'gate_arm_velocity': True},
    {'gate_rail_velocity': True},
    {'gate_solver_failed': True},
    {'configuration_finite': False},
])
def test_any_single_failure_makes_a_record_non_viable(overrides):
    """A well-conditioned configuration that misses the target is not a
    solution.

    This is the check that was missing when a wide seed bank was said to
    prove an alternative branch exists: conditioning passed, orientation and
    collision were never examined.
    """
    assert not failure_census.viable(_record(**overrides))


def test_good_conditioning_with_a_missed_target_is_not_viable():
    """The specific shape of the earlier overclaim, pinned."""
    assert not failure_census.viable(
        _record(gate_singular=False, position_error_m=0.0075,
                gate_pose_position=True)
    )


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------

def test_summary_separates_segment_membership_from_feasibility(validator, trajectory):
    """Largest-segment membership and per-waypoint feasibility are different
    columns, and neither is derived from the other."""
    records = []
    validator.reset_rng()
    segments = _segments(validator, trajectory, recorder=records.append)
    for record in records:
        record['mode'] = 'tracking'

    summary = failure_census.summarise(records, segments, WAYPOINTS)
    row = summary['waypoints'][0]
    assert 'in_largest_segment' in row
    assert 'tracking:accepted' in row
    assert 'tracking:viable' in row
    assert len(summary['waypoints']) == WAYPOINTS


def test_schema_is_versioned():
    assert isinstance(failure_census.SCHEMA_VERSION, int)


def test_tracking_defers_velocity_limits_to_the_validator():
    """A literal 2.0 here once reached every offline tool built on
    run_tracking while the service used the URDF."""
    assert failure_census._thresholds(None)['max_joint_vel_threshold'] is None
    assert failure_census._thresholds('arm_velocity')['max_joint_vel_threshold'] == float('inf')


def test_tracking_is_sensitive_to_the_enforced_arm_limits(validator, trajectory):
    """500/500 cannot detect a limit change: its fastest accepted secant is
    about 6% of any limit in use. So shrink the enforced limits below this
    arc's joint speeds and require the gate to fire, which proves the value
    the validator enforces is the value tracking uses."""
    positions, quaternions = trajectory
    baseline, _ = failure_census.run_tracking(validator, positions, quaternions,
                                              DT, LEGACY_MATLAB_START_Q)
    assert not [r for r in baseline if r['gate_arm_velocity'] is True]

    original = validator._arm_vel_limits
    validator._arm_vel_limits = np.full(6, 1e-3)
    try:
        shrunk, _ = failure_census.run_tracking(validator, positions, quaternions,
                                                DT, LEGACY_MATLAB_START_Q)
    finally:
        validator._arm_vel_limits = original
    assert [r for r in shrunk if r['gate_arm_velocity'] is True]


def test_wide_seed_bank_is_fixed_and_explicit():
    """Part of the record, not a detail of whoever ran it.

    Diagnostic probes for whether a viable branch exists, not candidate
    postures: stage 1C selects those under its own criteria.
    """
    bank = failure_census.WIDE_SEED_BANK_DEG
    assert len(bank) >= 6
    assert all(len(seed) == 6 for seed in bank)
    assert len({tuple(seed) for seed in bank}) == len(bank)


def test_census_targets_match_what_the_client_sends():
    """The census must measure the trajectory the service actually receives.

    An earlier version rebuilt the client's frame conversion and omitted its
    hand-placement offset, putting every target in the floor: collision then
    fired on all 5500 tracking attempts and the largest segment vanished,
    while production found 417. The census now delegates to the client's own
    builder, and this pins that it keeps doing so.
    """
    from ur10e_trajectory_pkg.ClientNode import (
        LEGACY_PLACEMENT_POSITION_RG,
        build_trajectory_targets,
    )

    pytest.importorskip('pandas')
    try:
        x, y, z, quaternions, times = build_trajectory_targets(num_waypoints=12)
    except FileNotFoundError:
        pytest.skip('packaged trajectory CSV not present')

    targets, census_quaternions, dt = failure_census.load_trajectory(None, 12)
    np.testing.assert_allclose(targets[:, 0], x)
    np.testing.assert_allclose(targets[:, 1], y)
    np.testing.assert_allclose(targets[:, 2], z)
    np.testing.assert_allclose(census_quaternions, quaternions)
    assert dt == pytest.approx(float(times[1] - times[0]))

    # The placement is a fixture standing in for a calibrated arena pose, not
    # a measurement, and it is load-bearing: at the origin the targets sit in
    # the floor and collision fires on every attempt.
    assert np.any(LEGACY_PLACEMENT_POSITION_RG != 0.0)
