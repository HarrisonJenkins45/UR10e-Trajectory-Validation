#!/usr/bin/env python3
"""Stage 4 gate: the seed experiment is controlled, and its labels are honest.

Corrected tracking already reaches every waypoint, so this measures candidate
diversity and the robustness of discovery, not reachability.

Two things make the difference interpretable. The rail seed comes from the
corrected tracking path rather than a fixed 1.5 m, so the arm effect is not
confounded with a rail offset, which is what invalidated the earlier reading.
And two lifts of one solution are one candidate wearing two coordinates, so
they must not inflate a diversity count.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg import candidate_generator as generator
from ur10e_trajectory_pkg.configurations import (
    ARM_SLICE,
    LEGACY_MATLAB_START_Q,
    RAIL_INDEX,
)
from ur10e_trajectory_pkg.joint_coordinates import TWO_PI
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

WAYPOINTS = 6


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description')
    )


@pytest.fixture(scope='module')
def trajectory(validator):
    """A short reachable arc, so the tests do not depend on the packaged CSV."""
    q_full = np.concatenate(([1.5], np.deg2rad([0.0, -135.0, 90.0, -90.0, 45.0, 0.0])))
    pose = validator.robot.fkine(q_full, end='tool0')
    quaternion = np.roll(np.array(pose.UnitQuaternion().A), -1)
    positions = np.tile(pose.t, (WAYPOINTS, 1))
    positions[:, 0] += np.linspace(0.0, 0.05, WAYPOINTS)
    return positions, np.tile(quaternion, (WAYPOINTS, 1))


# --------------------------------------------------------------------------
# The experiment is controlled
# --------------------------------------------------------------------------

def test_the_rail_grid_and_fixed_arm_seed_are_declared():
    """Part of the record, not a choice made at the console.

    The upstream posture is used for rail isolation precisely because it is
    external to this project, so it cannot have been picked to flatter a
    result.
    """
    assert generator.RAIL_GRID_M[0] == 0.0
    assert generator.RAIL_GRID_M[-1] == 3.0
    assert len(set(generator.RAIL_GRID_M)) == len(generator.RAIL_GRID_M)
    assert len(generator.UPSTREAM_ARM_SEED_DEG) == 6


def test_rail_seeds_come_from_the_corrected_tracking_path(validator, trajectory):
    """The confound that invalidated the earlier experiment.

    Seeding every waypoint at a fixed 1.5 m measures reachability from a rail
    position tracking never visits, so an arm effect and a rail offset become
    indistinguishable.
    """
    positions, quaternions = trajectory
    rail_seeds, arm_configs = generator.tracking_reference_path(
        validator, positions, quaternions, 0.1, LEGACY_MATLAB_START_Q)

    assert rail_seeds[0] == pytest.approx(LEGACY_MATLAB_START_Q[RAIL_INDEX])
    assert len(rail_seeds) == WAYPOINTS
    assert arm_configs.shape == (WAYPOINTS, 6)
    # The tracker does not sit at the midpoint, so this is a real difference.
    assert not np.allclose(rail_seeds, 1.5)


def test_each_solve_resets_the_recovery_generator(validator, trajectory):
    """A candidate must depend on its own seeds, not on run order."""
    positions, quaternions = trajectory
    first = generator.solve_one(
        validator, positions[0], quaternions[0],
        np.deg2rad(generator.UPSTREAM_ARM_SEED_DEG), 1.0, {'mode': 'test'})
    generator.solve_one(
        validator, positions[1], quaternions[1],
        np.deg2rad([30.0, -100.0, 60.0, -70.0, 80.0, 25.0]), 2.5, {'mode': 'test'})
    again = generator.solve_one(
        validator, positions[0], quaternions[0],
        np.deg2rad(generator.UPSTREAM_ARM_SEED_DEG), 1.0, {'mode': 'test'})

    assert len(first) == len(again)
    for left, right in zip(first, again):
        np.testing.assert_allclose(left['seed_arm'], right['seed_arm'], atol=1e-12)
        assert left['accepted'] == right['accepted']


def test_velocity_is_not_gated_during_node_generation(validator, trajectory):
    """Velocity belongs to an edge, not to a node.

    Enforcing it here would discard candidates that are perfectly good
    successors of some other predecessor.
    """
    positions, quaternions = trajectory
    records = generator.solve_one(
        validator, positions[0], quaternions[0],
        np.deg2rad(generator.UPSTREAM_ARM_SEED_DEG), 1.0, {'mode': 'test'})
    assert all(r['gate_arm_velocity'] is None for r in records)
    assert all(r['gate_rail_velocity'] is None for r in records)


def test_both_the_rail_seed_and_the_returned_rail_are_recorded(validator, trajectory):
    """The solver is free to move along the redundant rail direction, so the
    seed does not determine the result and both are needed."""
    positions, quaternions = trajectory
    records = generator.solve_one(
        validator, positions[0], quaternions[0],
        np.deg2rad(generator.UPSTREAM_ARM_SEED_DEG), 0.5, {'mode': 'test'})
    for record in records:
        assert record['rail_seed'] == pytest.approx(0.5)
        assert 'rail_position' in record


# --------------------------------------------------------------------------
# Deduplication distinguishes representation from diversity
# --------------------------------------------------------------------------

def _record(canonical, rail, lifted=None, **extra):
    base = {
        'accepted': True,
        'waypoint_index': 0,
        'q_arm_canonical': list(canonical),
        'rail_position': rail,
        'q_full': list(lifted if lifted is not None else [rail, *canonical]),
        'winding': [0] * 6,
        'mode': 'test',
        'attempt': 0,
        'solver_iterations': 3,
        'position_error_m': 0.0,
        'orientation_error_rad': 0.0,
        'arm_condition_number': 10.0,
    }
    base.update(extra)
    return base


def test_two_lifts_of_one_solution_are_one_candidate():
    """A winding difference is a coordinate change, not a new branch.

    Counting them separately would inflate every diversity measure, and the
    graph would see alternatives that do not exist.
    """
    canonical = [0.1, -2.0, 1.0, -1.0, 0.5, 0.2]
    lifted = [1.5, 0.1, -2.0, 1.0, -1.0, 0.5, 0.2 + TWO_PI]
    candidates = generator.collect_candidates([
        _record(canonical, 1.5),
        _record(canonical, 1.5, lifted=lifted, winding=[0, 0, 0, 0, 0, 1]),
    ])
    assert len(candidates[0]) == 1
    assert len(candidates[0][0]['lifted_forms']) == 2


def test_different_canonical_values_are_different_candidates():
    candidates = generator.collect_candidates([
        _record([0.1, -2.0, 1.0, -1.0, 0.5, 0.2], 1.5),
        _record([2.9, -1.2, -1.0, -2.0, -0.5, 0.2], 1.5),
    ])
    assert len(candidates[0]) == 2


def test_a_different_rail_position_is_a_different_candidate():
    """The rail is the redundancy, so two rail positions are two ways of
    doing the task rather than one."""
    canonical = [0.1, -2.0, 1.0, -1.0, 0.5, 0.2]
    candidates = generator.collect_candidates([
        _record(canonical, 0.5), _record(canonical, 2.5),
    ])
    assert len(candidates[0]) == 2


def test_deduplication_is_deterministic():
    canonical = [0.1, -2.0, 1.0, -1.0, 0.5, 0.2]
    records = [_record(canonical, 1.5) for _ in range(4)]
    first = generator.collect_candidates(records)
    for _ in range(3):
        assert (generator.collect_candidates(records)[0][0]['q_arm_canonical']
                == first[0][0]['q_arm_canonical'])


def test_rejected_solves_never_become_candidates():
    candidates = generator.collect_candidates([
        _record([0.1, -2.0, 1.0, -1.0, 0.5, 0.2], 1.5, accepted=False),
    ])
    assert candidates == {}


# --------------------------------------------------------------------------
# Provenance
# --------------------------------------------------------------------------

def test_every_candidate_records_which_seeds_produced_it():
    """Needed to rank seed generators by marginal contribution, and to tell
    an interaction effect from a main effect."""
    canonical = [0.1, -2.0, 1.0, -1.0, 0.5, 0.2]
    candidates = generator.collect_candidates([
        _record(canonical, 1.5, mode='arm_isolation', arm_seed_number=2,
                rail_seed_number=None, rail_seed=0.4),
        _record(canonical, 1.5, mode='interaction', arm_seed_number=5,
                rail_seed_number=3, rail_seed=1.5),
    ])
    provenance = candidates[0][0]['provenance']
    assert {p['mode'] for p in provenance} == {'arm_isolation', 'interaction'}
    assert {p['arm_seed_number'] for p in provenance} == {2, 5}
    assert all('attempt' in p and 'solver_iterations' in p for p in provenance)


def test_candidates_carry_their_quality_metrics():
    """Pose error, conditioning and rail position travel with the node, so
    edge construction does not have to re-derive them."""
    candidates = generator.collect_candidates(
        [_record([0.1, -2.0, 1.0, -1.0, 0.5, 0.2], 1.5)])
    entry = candidates[0][0]
    for field in ('position_error_m', 'orientation_error_rad',
                  'arm_condition_number', 'rail_position'):
        assert field in entry


# --------------------------------------------------------------------------
# The generator produces a usable node set
# --------------------------------------------------------------------------

def test_generation_yields_several_candidates_per_waypoint(validator, trajectory):
    """The graph needs alternatives at every layer, not one covering seed."""
    positions, quaternions = trajectory
    rail_seeds, _ = generator.tracking_reference_path(
        validator, positions, quaternions, 0.1, LEGACY_MATLAB_START_Q)
    arm_seeds = [np.deg2rad(seed) for seed in generator.WIDE_SEED_BANK_DEG[:3]]

    records = generator.run_arm_isolation(
        validator, positions, quaternions, rail_seeds, arm_seeds)
    candidates = generator.collect_candidates(records)

    assert set(candidates) == set(range(WAYPOINTS))
    assert all(len(entries) >= 1 for entries in candidates.values())


def test_every_candidate_reaches_its_target(validator, trajectory):
    """Nodes are statically valid by construction, so the graph can assume it."""
    positions, quaternions = trajectory
    rail_seeds, _ = generator.tracking_reference_path(
        validator, positions, quaternions, 0.1, LEGACY_MATLAB_START_Q)
    records = generator.run_arm_isolation(
        validator, positions, quaternions, rail_seeds,
        [np.deg2rad(generator.WIDE_SEED_BANK_DEG[2])])

    for index, entries in generator.collect_candidates(records).items():
        for entry in entries:
            reached = validator.robot.fkine(entry['lifted_forms'][0], end='tool0')
            assert np.linalg.norm(reached.t - positions[index]) <= 1e-3
