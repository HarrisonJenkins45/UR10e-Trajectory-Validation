#!/usr/bin/env python3
"""Stage 3 gate: joint values sit on the turn nearest where we already are.

Inverse kinematics returns every revolute solution wrapped into [-pi, pi]
while five of the six arm joints declare +/-2*pi, so a smooth motion crossing
that boundary read as a delta of nearly a full turn. That cut the 500-waypoint
trajectory in two and produced the 419 baseline.

Lifting is applied before any velocity check and the lifted values propagate
onward, so the same continuous coordinates reach the next seed, the stored
segment, the interpolation and the playback command. The physical
configuration is unchanged, which these tests assert directly against forward
kinematics, the Jacobian and collision.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg.configurations import (
    ARM_SLICE,
    JOINT_NAMES,
    LEGACY_MATLAB_START_Q,
    PERIODIC_JOINTS,
)
from ur10e_trajectory_pkg.joint_coordinates import (
    TWO_PI,
    feasible_lifts,
    nearest_feasible_lift,
    reachable_feasible_lifts,
    winding_numbers,
)
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

ELBOW = JOINT_NAMES.index('elbow_joint')
WRIST_1 = JOINT_NAMES.index('wrist_1_joint')


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description')
    )


@pytest.fixture(scope='module')
def arm_limits(validator):
    return (validator.robot.qlim[0][ARM_SLICE], validator.robot.qlim[1][ARM_SLICE])


ARM_PERIODIC = PERIODIC_JOINTS[ARM_SLICE]


# --------------------------------------------------------------------------
# Lift generation
# --------------------------------------------------------------------------

def test_every_generated_lift_is_inside_its_own_joint_limits(validator):
    lower, upper = validator.robot.qlim
    rng = np.random.default_rng(0)
    for _ in range(200):
        index = int(rng.integers(0, len(JOINT_NAMES)))
        value = float(rng.uniform(-np.pi, np.pi))
        for lift in feasible_lifts(value, lower[index], upper[index],
                                   PERIODIC_JOINTS[index]):
            assert lower[index] - 1e-9 <= lift <= upper[index] + 1e-9


def test_a_joint_spanning_four_pi_offers_an_alternative_turn():
    options = feasible_lifts(0.0, -TWO_PI, TWO_PI, periodic=True)
    assert sorted(options) == pytest.approx([-TWO_PI, 0.0, TWO_PI])


def test_the_elbow_has_no_alternative_lift():
    """Its declared span is exactly 2*pi, so q + 2*pi always leaves it.

    A boundary crossing there is a genuine large move and must stay rejected,
    which is why lifting has to be per joint rather than uniform.
    """
    assert feasible_lifts(0.5, -np.pi, np.pi, periodic=True) == [0.5]


def test_the_rail_is_never_lifted():
    """Prismatic, so 2*pi means nothing to it."""
    assert feasible_lifts(1.5, 0.0, 3.0, periodic=False) == [1.5]


def test_a_periodic_value_outside_its_limits_is_lifted_back_inside():
    """Being out of range is not fatal for a periodic joint.

    5.0 rad sits outside [-pi, pi], but 5.0 - 2*pi does not, and it is the
    same physical angle. Recovering it is the point of lifting.
    """
    options = feasible_lifts(5.0, -np.pi, np.pi, periodic=True)
    assert options == pytest.approx([5.0 - TWO_PI])


def test_no_lift_exists_when_the_range_is_narrower_than_a_turn():
    """A joint with a sub-2*pi range can be genuinely unreachable."""
    assert feasible_lifts(1.0, -0.1, 0.1, periodic=True) == []


def test_a_non_periodic_value_outside_its_limits_yields_no_lift():
    """The rail cannot be rescued by arithmetic."""
    assert feasible_lifts(9.0, 0.0, 3.0, periodic=False) == []


def test_lifts_at_the_limit_boundary_are_accepted_not_dropped():
    """A value landing exactly on +/-2*pi is legal, and float arithmetic must
    not lose it."""
    options = feasible_lifts(0.0, -TWO_PI, TWO_PI, periodic=True)
    assert any(abs(option - TWO_PI) < 1e-9 for option in options)


# --------------------------------------------------------------------------
# Choosing a lift
# --------------------------------------------------------------------------

def test_nearest_lift_recovers_the_measured_wrist_1_transition(arm_limits):
    """The real case: -6.2744 rad becomes a motion under 0.02 rad.

    At a 0.1 s step the gate rejects above 0.2 rad, so the canonical value
    failed by a factor of thirty while the physical motion was tiny.
    """
    previous = np.zeros(6)
    previous[WRIST_1 - 1] = 0.005
    canonical = np.zeros(6)
    canonical[WRIST_1 - 1] = 0.005 - 6.2744

    lifted = nearest_feasible_lift(canonical, previous, arm_limits, ARM_PERIODIC)
    assert abs(lifted[WRIST_1 - 1] - previous[WRIST_1 - 1]) < 0.02


def test_winding_numbers_report_the_turns_taken(arm_limits):
    previous = np.zeros(6)
    canonical = np.zeros(6)
    canonical[WRIST_1 - 1] = -TWO_PI + 0.01
    lifted = nearest_feasible_lift(canonical, previous, arm_limits, ARM_PERIODIC)
    assert winding_numbers(lifted, canonical)[WRIST_1 - 1] == 1


def test_tie_breaking_is_deterministic(arm_limits):
    """Exactly half a turn away, the smaller |k| wins, every time."""
    canonical = np.zeros(6)
    canonical[WRIST_1 - 1] = np.pi
    reference = np.zeros(6)
    first = nearest_feasible_lift(canonical, reference, arm_limits, ARM_PERIODIC)
    for _ in range(5):
        np.testing.assert_allclose(
            nearest_feasible_lift(canonical, reference, arm_limits, ARM_PERIODIC),
            first, atol=1e-15)


def test_a_multi_turn_start_pose_keeps_its_winding(arm_limits):
    """A measured start already a full turn out must not snap to canonical.

    If the robot is physically at wrist_1 = +2*pi, commanding it to 0 is a
    full revolution, not a no-op.
    """
    measured = np.zeros(6)
    measured[WRIST_1 - 1] = TWO_PI - 0.01
    canonical = np.zeros(6)
    canonical[WRIST_1 - 1] = -0.01

    lifted = nearest_feasible_lift(canonical, measured, arm_limits, ARM_PERIODIC)
    assert lifted[WRIST_1 - 1] == pytest.approx(TWO_PI - 0.01, abs=1e-9)
    assert winding_numbers(lifted, canonical)[WRIST_1 - 1] == 1


def test_reachable_lifts_respect_the_velocity_budget(arm_limits):
    predecessor = np.zeros(6)
    canonical = np.zeros(6)
    canonical[WRIST_1 - 1] = -TWO_PI + 0.01

    within = reachable_feasible_lifts(
        canonical, predecessor, arm_limits, ARM_PERIODIC, [2.0] * 6, 0.1)
    assert within, 'the lifted motion is small and must be reachable'
    assert all(abs(row[WRIST_1 - 1]) <= 0.2 + 1e-9 for row in within)

    # A budget too small for any turn leaves nothing.
    assert reachable_feasible_lifts(
        canonical, predecessor, arm_limits, ARM_PERIODIC, [1e-6] * 6, 0.1) == []


# --------------------------------------------------------------------------
# Physical invariance
# --------------------------------------------------------------------------

def test_forward_kinematics_is_invariant_under_a_legal_lift(validator):
    q_full = np.concatenate(([1.5], np.deg2rad([10.0, -120.0, 80.0, -70.0, 40.0, 20.0])))
    lifted = q_full.copy()
    lifted[WRIST_1] += TWO_PI

    original = validator.robot.fkine(q_full, end='tool0')
    shifted = validator.robot.fkine(lifted, end='tool0')
    np.testing.assert_allclose(shifted.t, original.t, atol=1e-9)
    np.testing.assert_allclose(shifted.R, original.R, atol=1e-9)


def test_the_jacobian_is_invariant_under_a_legal_lift(validator):
    q_full = np.concatenate(([1.5], np.deg2rad([10.0, -120.0, 80.0, -70.0, 40.0, 20.0])))
    lifted = q_full.copy()
    lifted[WRIST_1] += TWO_PI
    np.testing.assert_allclose(
        validator.compute_system_jacobian(lifted),
        validator.compute_system_jacobian(q_full), atol=1e-9)


def test_collision_is_invariant_under_a_legal_lift(validator):
    q_full = np.concatenate(([1.5], np.deg2rad([10.0, -120.0, 80.0, -70.0, 40.0, 20.0])))
    lifted = q_full.copy()
    lifted[WRIST_1] += TWO_PI
    assert (validator.check_all_collisions(lifted)
            == validator.check_all_collisions(q_full))


# --------------------------------------------------------------------------
# End to end on the real trajectory
# --------------------------------------------------------------------------

@pytest.fixture(scope='module')
def tracked(validator):
    pytest.importorskip('pandas')
    from ur10e_trajectory_pkg.failure_census import load_trajectory, run_tracking
    try:
        targets, quaternions, dt = load_trajectory(None, 500)
    except FileNotFoundError:
        pytest.skip('packaged trajectory CSV not present')
    return run_tracking(validator, targets, quaternions, dt,
                        LEGACY_MATLAB_START_Q)


def test_the_trajectory_is_one_segment_with_the_velocity_gate_enabled(tracked):
    """The result lifting was meant to restore.

    Before it, the trajectory split into [0, 80] and [81, 499] at the wrap.
    The gate is ON here: this is not the ablation run.
    """
    _, segments = tracked
    assert len(segments) == 1
    assert segments[0]['start_idx'] == 0
    assert segments[0]['end_idx'] == 499
    assert segments[0]['length'] == 500


def test_no_arm_velocity_failures_remain(tracked):
    """All 17 were wraps, so all 17 should be gone."""
    records, _ = tracked
    assert not [r for r in records if r['gate_arm_velocity'] is True]


def test_accepted_transitions_carry_no_full_turn_excursion(tracked):
    """The stored configurations are continuous.

    This is what reaches the interpolation and the playback command, so a
    residual 2*pi step here would become a commanded full revolution.
    """
    records, _ = tracked
    deltas = [np.asarray(r['arm_delta_rad']) for r in records
              if r['accepted'] and r['arm_delta_rad']]
    assert deltas
    worst = max(float(np.max(np.abs(d))) for d in deltas)
    assert worst < 0.2, f'largest accepted joint step is {worst:.4f} rad'


def test_interpolated_velocity_has_no_wrap_spike(validator, tracked):
    """A 2*pi position step becomes an enormous velocity spike downstream.

    Checks the interpolated trajectory the service actually plays back, not
    just the waypoints.
    """
    _, segments = tracked
    segment = max(segments, key=lambda s: s['length'])
    _, q_interp = validator.process_feasible_segment(
        segment, LEGACY_MATLAB_START_Q, 0.1, verbose=False)

    steps = np.abs(np.diff(q_interp[:, ARM_SLICE], axis=0))
    assert float(np.max(steps)) < 0.2
