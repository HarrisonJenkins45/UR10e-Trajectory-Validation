#!/usr/bin/env python3
"""Stage 0 gate: every solver path is deterministic, including failures.

Seeding our own retry generator was not sufficient. ikine_LM defaults to
slimit=100, so on failing to converge from the supplied seed it retries from
up to 99 further configurations of its own choosing, drawn from an unseeded
generator. Easy targets converge in one search and hide this; hard ones do
not. Measured before the fix, five identical calls on an unreachable target
each burned 100 searches and returned five different configurations.

The earlier determinism check passed only because the sampled trajectory's
waypoints mostly converge first try. These tests exercise the failing path
directly, which is where the nondeterminism actually lived.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from spatialmath import SE3, UnitQuaternion

from ur10e_trajectory_pkg.validation_core import (
    IK_SEARCH_LIMIT,
    TrajectoryValidator,
)

from test_geometry_invariants import _urdf_path

REPEATS = 5

# Reachable at mid-rail, and far past the combined workspace. The second is
# the interesting one: it forces the failure path every time.
REACHABLE_X = 1.60
UNREACHABLE_X = 8.00


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(),
        mesh_base_path=get_package_share_directory('ur_description'),
    )


@pytest.fixture(scope='module')
def nominal_pose(validator):
    """A pose reached from a known configuration, to borrow an orientation."""
    q_full = np.concatenate(([1.5], np.deg2rad([0.0, -135.0, 90.0, -90.0, 0.0, 0.0])))
    pose = validator.robot.fkine(q_full, end='tool0')
    quaternion = np.roll(np.array(pose.UnitQuaternion().A), -1)  # wxyz -> xyzw
    return pose.t, quaternion


def _solve(validator, target, quaternion):
    rotation = UnitQuaternion(quaternion[3], quaternion[:3]).R
    return validator.robot.ikine_LM(
        SE3.Rt(rotation, target),
        end='tool0',
        q0=np.concatenate(([1.5], np.deg2rad([0.0, -135.0, 90.0, -90.0, 0.0, 0.0]))),
        mask=[1, 1, 1, 1, 1, 1],
        tol=1e-4,
        slimit=IK_SEARCH_LIMIT,
        seed=validator._seed,
    )


def test_search_limit_is_one():
    """Retry policy belongs to _solve_waypoint_with_recovery alone.

    A second search loop nested inside the solver made the attempt accounting
    meaningless and determinism unprovable.
    """
    assert IK_SEARCH_LIMIT == 1


def test_repeated_solves_of_a_reachable_target_agree(validator, nominal_pose):
    position, quaternion = nominal_pose
    target = np.array([REACHABLE_X, position[1], position[2]])
    solutions = [_solve(validator, target, quaternion) for _ in range(REPEATS)]

    assert all(s.success for s in solutions)
    for solution in solutions[1:]:
        np.testing.assert_allclose(solution.q, solutions[0].q, atol=1e-12)


def test_repeated_solves_of_an_out_of_reach_target_agree(validator, nominal_pose):
    """The path that used to be nondeterministic.

    Deliberately asserts nothing about success: see
    test_reported_success_does_not_mean_the_target_was_reached. What matters
    here is that five identical calls return one identical answer, which
    before the search limit was pinned they did not.
    """
    position, quaternion = nominal_pose
    target = np.array([UNREACHABLE_X, position[1], position[2]])
    solutions = [_solve(validator, target, quaternion) for _ in range(REPEATS)]

    for solution in solutions[1:]:
        np.testing.assert_allclose(solution.q, solutions[0].q, atol=1e-12)
        assert solution.success == solutions[0].success
        assert solution.searches == solutions[0].searches
        assert solution.iterations == solutions[0].iterations
        assert solution.reason == solutions[0].reason


def test_a_solve_costs_one_search_not_a_hundred(validator, nominal_pose):
    """No hidden second search loop, on either the reaching or failing path."""
    position, quaternion = nominal_pose
    for target_x in (REACHABLE_X, UNREACHABLE_X):
        target = np.array([target_x, position[1], position[2]])
        assert _solve(validator, target, quaternion).searches == IK_SEARCH_LIMIT


def test_reported_success_does_not_mean_the_target_was_reached(validator,
                                                               nominal_pose):
    """Document the solver contract, because it is not the obvious one.

    ikine_LM's residual measures convergence of its local search, not distance
    to the target, and success is that residual against tol. So the solver can
    settle in a local minimum, report success, and sit metres away. Measured
    at x = 8.0: success True, residual 3.6e-05, actual position error 6.29 m.

    This test describes roboticstoolbox, not our code, so it keeps passing
    after the gate below is fixed and explains why the gate was needed.
    """
    position, quaternion = nominal_pose
    target = np.array([UNREACHABLE_X, position[1], position[2]])
    solution = _solve(validator, target, quaternion)
    reached = validator.robot.fkine(solution.q, end='tool0').t
    assert np.linalg.norm(reached - target) > 1.0
    assert solution.residual < 1e-3


@pytest.mark.xfail(
    strict=True,
    reason='Known defect: _solve_waypoint_with_recovery.evaluate() gates on '
           'sol.success alone and never checks that forward kinematics '
           'reproduces the commanded pose, so a local minimum metres from '
           'the target can be accepted. Belongs with stage 5, whose gate is '
           'that every accepted solution independently reproduces its target.',
)
def test_recovery_rejects_solutions_that_do_not_reach_the_target(validator,
                                                                 nominal_pose):
    """A configuration that does not reach the commanded pose is not a
    solution, whatever the solver reports."""
    position, quaternion = nominal_pose
    target = np.array([UNREACHABLE_X, position[1], position[2]])
    validator.reset_rng()
    result = validator._solve_waypoint_with_recovery(
        target, quaternion, np.deg2rad([0.0, -135.0, 90.0, -90.0, 0.0, 0.0]),
        rail_pos=1.5, verbose=False,
    )
    if result['ok']:
        reached = validator.robot.fkine(result['q_full'], end='tool0').t
        assert np.linalg.norm(reached - target) < 1e-3
    else:
        assert 'reach' in (result['reason'] or '').lower()


def test_waypoint_recovery_is_reproducible(validator, nominal_pose):
    """End to end through our own retry loop, not just the raw solver.

    Covers both generators at once: ours for the seed perturbations, the
    solver's for anything it does internally.
    """
    position, quaternion = nominal_pose
    target = np.array([REACHABLE_X, position[1], position[2]])
    seed_arm = np.deg2rad([0.0, -135.0, 90.0, -90.0, 0.0, 0.0])

    results = []
    for _ in range(3):
        validator.reset_rng()
        results.append(
            validator._solve_waypoint_with_recovery(
                target, quaternion, seed_arm, rail_pos=1.5, verbose=False
            )
        )

    for result in results[1:]:
        assert result['ok'] == results[0]['ok']
        assert result['reason'] == results[0]['reason']
        np.testing.assert_allclose(result['q_arm'], results[0]['q_arm'], atol=1e-12)
        assert result['rail_pos'] == pytest.approx(results[0]['rail_pos'], abs=1e-12)
