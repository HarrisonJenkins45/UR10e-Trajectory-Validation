#!/usr/bin/env python3
"""Stage 5 gate: the layered graph, and the oracle that makes it testable.

One binary question before any cost tuning: does a complete path exist from
the actual q_start through every layer?

The oracle needs two builds. Injecting the corrected greedy configuration at
every layer guarantees a valid path exists, so a failure there is the graph's
fault. Excluding it tests whether independent candidate generation suffices.
Without that separation a failure is ambiguous between a graph defect and an
incomplete candidate set.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg import graph_planner
from ur10e_trajectory_pkg.configurations import (
    ARM_SLICE,
    JOINT_NAMES,
    LEGACY_MATLAB_START_Q,
    RAIL_INDEX,
)
from ur10e_trajectory_pkg.joint_coordinates import TWO_PI
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

LAYERS = 8


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description')
    )


@pytest.fixture(scope='module')
def oracle(validator):
    """A short, genuinely feasible reference path, built by forward kinematics.

    Each step is small, so every transition is comfortably inside the velocity
    budget and the graph must be able to follow it.
    """
    path = []
    for index in range(LAYERS):
        configuration = np.concatenate((
            [0.5 + 0.01 * index],
            np.deg2rad([0.0, -135.0 + 0.5 * index, 90.0, -90.0, 45.0, 0.0]),
        ))
        path.append(configuration)
    return path


@pytest.fixture(scope='module')
def graph(validator, oracle):
    layers = graph_planner.inject_oracle([[] for _ in range(LAYERS)], oracle)
    return graph_planner.LayeredGraph(validator, layers, LAYERS, dt=0.1)


# --------------------------------------------------------------------------
# State carries the winding
# --------------------------------------------------------------------------

def test_state_is_the_full_lifted_configuration():
    """Not the canonical candidate.

    Which lifts a successor can reach depends on where the predecessor
    actually is, so a winding stored only on an edge would be lost before the
    next transition is evaluated.
    """
    canonical = np.zeros(7)
    lifted = canonical.copy()
    lifted[4] += TWO_PI
    assert graph_planner.state_key(canonical) != graph_planner.state_key(lifted)


def test_two_routes_to_one_configuration_share_a_state():
    """Otherwise the frontier splits on floating-point noise alone."""
    first = np.array([0.5, 0.1, -2.0, 1.0, -1.0, 0.5, 0.2])
    second = first + 1e-12
    assert graph_planner.state_key(first) == graph_planner.state_key(second)


def test_a_successor_keeps_the_winding_it_was_reached_by(validator, oracle):
    """The lifted configuration propagates into the next layer.

    A predecessor a full turn out must produce successors near it, not near
    the canonical form.
    """
    # wrist_1 sits at -90 deg here, so +2*pi lands at 4.71 rad, inside its
    # +/-2*pi limits. Picking a joint whose lift leaves its range would test
    # the limit check instead of winding propagation.
    wrist_1 = JOINT_NAMES.index('wrist_1_joint')
    shifted = np.asarray(oracle[0]).copy()
    shifted[wrist_1] += TWO_PI
    layers = graph_planner.inject_oracle([[] for _ in range(LAYERS)], oracle)
    graph = graph_planner.LayeredGraph(validator, layers, LAYERS, dt=0.1)

    successors = graph.successors(shifted, 1, dt=0.1)
    assert successors, 'no successor reachable from the lifted predecessor'
    for configuration, _ in successors:
        assert abs(configuration[wrist_1] - shifted[wrist_1]) <= 0.2 + 1e-9


# --------------------------------------------------------------------------
# Edge cost
# --------------------------------------------------------------------------

def test_the_graph_uses_the_validators_velocity_limits_by_default(validator):
    """The URDF's per-joint values and the rail's cap, the same limits the
    tracker it is compared with enforces. A uniform 2.0 rad/s default once
    pruned, lifted and costed against a different robot."""
    graph = graph_planner.LayeredGraph(validator, [[]], 1, dt=0.1)
    np.testing.assert_array_equal(graph.velocity_limits,
                                  validator.velocity_limits)


def test_an_explicit_limit_vector_overrides_the_default(validator):
    limits = np.array([0.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    graph = graph_planner.LayeredGraph(validator, [[]], 1, dt=0.1,
                                       velocity_limits=limits)
    np.testing.assert_array_equal(graph.velocity_limits, limits)


def test_path_cost_charges_the_first_edge_at_the_transition_duration():
    limits = np.array([1.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0])
    start = np.zeros(7)
    path = [np.full(7, 0.1), np.full(7, 0.15)]
    expected = (graph_planner.edge_cost(start, path[0], limits,
                                        graph_planner.TRANSITION_SECONDS)
                + graph_planner.edge_cost(path[0], path[1], limits, 0.1))
    assert graph_planner.path_cost(start, path, limits, 0.1) == pytest.approx(expected)


def test_cost_normalises_each_joint_by_its_own_budget():
    """The rail's metres and the arm's radians are not comparable raw.

    A move using the whole budget on one joint costs 1, whichever joint it is.
    """
    limits = np.array([1.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0])
    start = np.zeros(7)

    rail_only = start.copy()
    rail_only[RAIL_INDEX] = 1.0 * 0.1
    arm_only = start.copy()
    arm_only[1] = 2.0 * 0.1

    assert graph_planner.edge_cost(start, rail_only, limits, 0.1) == pytest.approx(1.0)
    assert graph_planner.edge_cost(start, arm_only, limits, 0.1) == pytest.approx(1.0)


def test_cost_is_zero_for_no_motion():
    limits = np.ones(7) * 2.0
    configuration = np.array([0.5, 0.1, -2.0, 1.0, -1.0, 0.5, 0.2])
    assert graph_planner.edge_cost(configuration, configuration, limits, 0.1) == 0.0


def test_the_approach_edge_uses_its_own_duration():
    """q_start to layer 0 is an approach, not a trajectory step.

    Charging it at the 0.1 s per-waypoint rate would reject it as a velocity
    violation for no physical reason.
    """
    assert graph_planner.TRANSITION_SECONDS > 0.1


# --------------------------------------------------------------------------
# Pruning
# --------------------------------------------------------------------------

def test_rail_displacement_is_rejected_before_anything_else(validator, oracle):
    """The cheapest check runs first, so collision queries stay rare."""
    layers = [[np.concatenate(([2.9], np.asarray(oracle[0])[ARM_SLICE]))]
              for _ in range(LAYERS)]
    graph = graph_planner.LayeredGraph(validator, layers, LAYERS, dt=0.1)

    assert graph.successors(oracle[0], 1, dt=0.1) == []
    assert graph.counters['pruned_rail'] == 1
    assert graph.counters['pruned_swept_collision'] == 0


def test_an_unreachable_joint_step_is_pruned(validator, oracle):
    far = np.asarray(oracle[0]).copy()
    far[2] += 1.0                  # far beyond 120 deg/s * 0.1 s = 0.21 rad
    layers = [[far] for _ in range(LAYERS)]
    graph = graph_planner.LayeredGraph(validator, layers, LAYERS, dt=0.1)

    assert graph.successors(oracle[0], 1, dt=0.1) == []
    assert graph.counters['pruned_velocity_or_lift'] == 1


def test_swept_collision_samples_between_the_endpoints(validator, oracle):
    """Both endpoints are already valid as nodes, so only the space between
    them is in question."""
    graph = graph_planner.LayeredGraph(validator, [[]], 1, dt=0.1,
                                       swept_samples=3)
    assert not graph.swept_collision(np.asarray(oracle[0]), np.asarray(oracle[1]))


# --------------------------------------------------------------------------
# The oracle
# --------------------------------------------------------------------------

def test_the_validation_build_finds_a_complete_path(graph, oracle):
    """A path is known to exist, so any failure here is the graph's."""
    result, first_empty, _ = graph.shortest_path(oracle[0])
    assert first_empty is None
    assert result is not None
    path, _ = result
    assert len(path) == LAYERS + 1


def test_the_optimal_cost_does_not_exceed_the_injected_path(graph, oracle):
    """The injected path is one candidate route, so the optimum is at most
    its cost."""
    result, _, _ = graph.shortest_path(oracle[0])
    path, cost = result

    reference = 0.0
    previous = np.asarray(oracle[0])
    for index in range(LAYERS):
        dt = graph.transition_seconds if index == 0 else graph.dt
        step = np.asarray(oracle[index])
        reference += graph_planner.edge_cost(previous, step,
                                             graph.velocity_limits, dt)
        previous = step
    assert cost <= reference + 1e-9


def test_an_empty_layer_reports_where_it_disconnected(validator, oracle):
    """A generator-only build that fails must say which layer, not just no."""
    layers = graph_planner.inject_oracle([[] for _ in range(LAYERS)], oracle)
    layers[4] = []
    graph = graph_planner.LayeredGraph(validator, layers, LAYERS, dt=0.1)

    result, first_empty, _ = graph.shortest_path(oracle[0])
    assert result is None
    assert first_empty == 4


def test_the_search_is_deterministic(graph, oracle):
    first, _, _ = graph.shortest_path(oracle[0])
    for _ in range(3):
        again, _, _ = graph.shortest_path(oracle[0])
        assert again[1] == pytest.approx(first[1])
        np.testing.assert_allclose(np.asarray(again[0]), np.asarray(first[0]),
                                   atol=1e-12)


def test_the_returned_path_has_no_velocity_violation(graph, oracle):
    """The property the whole gate exists to establish."""
    result, _, _ = graph.shortest_path(oracle[0])
    path, _ = result
    for index in range(1, len(path)):
        dt = graph.transition_seconds if index == 1 else graph.dt
        speed = np.abs(np.asarray(path[index]) - np.asarray(path[index - 1])) / dt
        assert np.all(speed <= graph.velocity_limits + 1e-9)


def test_the_returned_path_has_continuous_windings(graph, oracle):
    """No state may jump a full turn from its predecessor."""
    result, _, _ = graph.shortest_path(oracle[0])
    path, _ = result
    for index in range(1, len(path)):
        delta = np.abs(np.asarray(path[index])[ARM_SLICE]
                       - np.asarray(path[index - 1])[ARM_SLICE])
        assert np.all(delta < TWO_PI / 2)


# --------------------------------------------------------------------------
# Free start
# --------------------------------------------------------------------------

def test_a_free_start_chooses_among_first_waypoint_candidates(validator, oracle):
    """A common source reaches every layer-0 candidate at zero cost; the arm
    is brought to the chosen start by a separate warmup, so no transition
    edge exists."""
    decoy = np.asarray(oracle[0]).copy()
    decoy[2] += 0.3
    layers = graph_planner.inject_oracle([[] for _ in range(LAYERS)], oracle)
    layers[0] = [decoy] + list(layers[0])
    graph = graph_planner.LayeredGraph(validator, layers, LAYERS, dt=0.1)

    result, first_empty, _ = graph.shortest_path()
    assert first_empty is None
    path, cost = result
    assert len(path) == LAYERS
    assert graph.counters['start_states'] == 2
    np.testing.assert_allclose(path[0], oracle[0], atol=1e-12)

    fixed, _, _ = graph.shortest_path(oracle[0])
    assert cost <= fixed[1] + 1e-12
    assert cost == pytest.approx(graph_planner.path_cost(None, path,
                                                         graph.velocity_limits, 0.1))


def test_a_free_start_is_deterministic(validator, oracle):
    layers = graph_planner.inject_oracle([[] for _ in range(LAYERS)], oracle)
    graph = graph_planner.LayeredGraph(validator, layers, LAYERS, dt=0.1)
    first, _, _ = graph.shortest_path()
    again, _, _ = graph.shortest_path()
    np.testing.assert_allclose(np.asarray(again[0]), np.asarray(first[0]), atol=1e-12)


def test_an_empty_first_layer_disconnects_a_free_start_at_layer_zero(validator, oracle):
    layers = graph_planner.inject_oracle([[] for _ in range(LAYERS)], oracle)
    layers[0] = []
    graph = graph_planner.LayeredGraph(validator, layers, LAYERS, dt=0.1)
    result, first_empty, _ = graph.shortest_path()
    assert result is None and first_empty == 0


def test_start_lifts_add_every_legal_winding(validator, oracle):
    layers = graph_planner.inject_oracle([[] for _ in range(LAYERS)], oracle)
    graph = graph_planner.LayeredGraph(validator, layers, LAYERS, dt=0.1)
    canonical = graph.start_states()
    lifted = graph.start_states(lifts=True)
    assert len(canonical) == 1 and len(lifted) > 1
    for state in lifted:
        np.testing.assert_allclose(
            np.angle(np.exp(1j * (state[1:] - canonical[0][1:]))), 0, atol=1e-9)


# --------------------------------------------------------------------------
# Executed-path entry state
# --------------------------------------------------------------------------

def test_a_path_at_rest_passes_the_declared_tolerance(oracle):
    at_rest = [np.asarray(oracle[0])] * 3
    report = graph_planner.entry_state_report(
        at_rest, 0.1, np.full(7, 1.0), np.full(7, 5.0))
    assert report['at_rest'] is True
    assert report['joints_over_tolerance'] == []
    assert report['tolerance_fraction'] == graph_planner.AT_REST_TOLERANCE_FRACTION


def test_a_sliding_rail_is_named_as_the_entry_motion(oracle):
    """The tool can hold still while the rail slides and the arm compensates;
    the report must say which joint moves, not only that something does."""
    start = np.asarray(oracle[0])
    path = [start + np.concatenate(([0.01 * i], np.zeros(6))) for i in range(3)]
    report = graph_planner.entry_state_report(path, 0.1, np.full(7, 1.0),
                                              np.full(7, 5.0))
    assert report['at_rest'] is False
    assert report['dominant_joint'] == 'linear_rail_joint'
    assert report['velocity_ratio'][0] == pytest.approx(0.1, rel=1e-6)
    assert report['joints_over_tolerance'] == ['linear_rail_joint']

