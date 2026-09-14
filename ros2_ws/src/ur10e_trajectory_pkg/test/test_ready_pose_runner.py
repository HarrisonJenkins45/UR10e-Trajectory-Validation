#!/usr/bin/env python3
"""Stage 7 runner gate: cheapest checks first, and nothing counted twice.

Synthetic placements built by forward kinematics, so every expectation is
known in advance and the suite stays fast.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg import ready_pose_runner as runner
from ur10e_trajectory_pkg import ready_pose_sweep as sweep
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

DT = 0.1
STEP = np.concatenate(([0.0], np.full(6, 0.004)))


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description')
    )


def _pose(validator, configuration):
    pose = validator.robot.fkine(configuration, end='tool0')
    return pose.t, np.roll(np.array(pose.UnitQuaternion().A), -1)


@pytest.fixture(scope='module')
def branch_a():
    return np.concatenate(([1.0], np.deg2rad([0.0, -120.0, 90.0, -80.0, 60.0, 0.0])))


@pytest.fixture(scope='module')
def branch_b():
    return np.concatenate(([1.0], np.deg2rad([40.0, -100.0, 70.0, -60.0, 80.0, 30.0])))


@pytest.fixture(scope='module')
def placement(validator, branch_a, branch_b):
    """Two branches; layer 1 also holds a costlier decoy continuation of A,
    and layer 0 holds a candidate with no continuation at all."""
    orphan = np.concatenate(([2.5], np.deg2rad([-90.0, -60.0, 60.0, -90.0, -90.0, 0.0])))
    layers = [
        [branch_a, branch_b, orphan],
        [branch_a + STEP, branch_a + 3 * STEP, branch_b + STEP],
        [branch_a + 2 * STEP, branch_b + 2 * STEP],
    ]
    first, second = _pose(validator, branch_a), _pose(validator, branch_a + STEP)
    return {'name': 'synthetic', 'layers': layers,
            'positions': np.stack([first[0], second[0], second[0]]),
            'quaternions': np.stack([first[1], second[1], second[1]]),
            'dt': DT}


@pytest.fixture(scope='module')
def prepared(validator, placement):
    meter = runner.Meter()
    state = runner.prepare_placement(
        validator, placement['name'], placement['layers'],
        placement['positions'], placement['quaternions'], DT, meter)
    return state


@pytest.fixture(scope='module')
def ready(branch_a):
    offset = np.concatenate(([0.1], np.deg2rad([5.0, 5.0, -5.0, 5.0, 5.0, 5.0])))
    return branch_a + offset


# --------------------------------------------------------------------------
# Per placement
# --------------------------------------------------------------------------

def test_a_candidate_without_continuation_is_dropped_before_classifying(prepared):
    """It is not a task candidate, so it must not count against a ready pose."""
    assert prepared['status'] == 'ready'
    assert prepared['counts']['task_gate_rejections'] == {'no_continuation': 1}
    assert [v['candidate_index'] for v in prepared['valid']] == [0, 1]


def test_the_cheapest_continuation_is_kept_and_alternatives_counted(prepared, branch_a):
    entry = prepared['valid'][0]
    np.testing.assert_allclose(entry['prefix'][1], branch_a + STEP, atol=1e-12)
    np.testing.assert_allclose(entry['prefix'][2], branch_a + 2 * STEP, atol=1e-12)
    assert entry['continuation_alternatives'] >= 2


def test_surviving_candidates_carry_branch_labels(prepared):
    assert prepared['counts']['branch_clusters'] == 2
    assert {v['branch'] for v in prepared['valid']} == {0, 1}


def test_a_twist_the_arm_cannot_make_leaves_no_task_candidate(validator, placement,
                                                              ready):
    """170 degrees in one 0.1 s step is far beyond any wrist, so alpha* < 1 for
    every candidate, and no ready pose is evaluated against the placement."""
    from scipy.spatial.transform import Rotation

    quaternions = placement['quaternions'].copy()
    quaternions[1] = (Rotation.from_quat(quaternions[0])
                      * Rotation.from_euler('z', 170, degrees=True)).as_quat()
    meter = runner.Meter()
    state = runner.prepare_placement(
        validator, 'fast', placement['layers'], placement['positions'],
        quaternions, DT, meter)
    assert state['status'] == sweep.NO_TASK_CANDIDATE
    assert state['counts']['task_gate_rejections'].get('alpha_star', 0) == 3

    with runner.counting_collisions(validator, meter):
        result = runner.evaluate_ready_pose(validator, ready, state, meter)
    assert result['classification'] == sweep.NO_TASK_CANDIDATE
    assert not any(k.startswith('collision_queries') for k in meter.counts)


def test_a_continuation_entering_beyond_a_limit_is_dropped_per_placement(
        validator, placement, branch_a):
    """Entry state is invariant under a lift, so it is decided once here. A
    rail step of 0.09 then 0 gives PCHIP an entry velocity of 1.35 m/s against
    the 1.0 cap, and no approach from rest can end in that."""
    step = np.zeros(7)
    step[0] = 0.09
    layers = [[branch_a], [branch_a + step], [branch_a + step]]
    state = runner.prepare_placement(
        validator, 'fast_rail', layers, placement['positions'],
        placement['quaternions'], DT, runner.Meter())
    assert state['status'] == sweep.NO_TASK_CANDIDATE
    assert state['counts']['task_gate_rejections'] == {
        'entry_state_exceeds_limits': 1}


def test_a_placement_without_layers_is_refused_not_invented(validator, placement):
    state = runner.prepare_placement(
        validator, 'translate_x+', None, placement['positions'],
        placement['quaternions'], DT, runner.Meter())
    assert state['status'] == runner.NO_LAYERS


# --------------------------------------------------------------------------
# Per ready pose
# --------------------------------------------------------------------------

def test_a_nearby_ready_pose_connects_to_both_branches(validator, prepared, ready):
    result = runner.evaluate_ready_pose(validator, ready, prepared, runner.Meter())
    assert result['classification'] == sweep.CONNECTED
    assert result['connected_branches'] == 2
    assert result['best_duration_s'] > 0
    assert all(q > 0 for q in result['counts']['collision_queries_per_approach'])


def test_collision_is_checked_in_duration_order_and_stops_at_first_feasible(
        validator, prepared, ready, monkeypatch):
    """The first feasible approach checked is therefore the branch's shortest;
    everything after it is skipped and counted."""
    seen = []
    original = sweep.evaluate_approach

    def first_one_collides(*args, **kwargs):
        seen.append(kwargs['duration'])
        if len(seen) == 1:
            return {'feasible': False, 'reason_code': sweep.REASON_COLLISION,
                    'duration_s': kwargs['duration'], 'collision_queries': 1}
        return original(*args, **kwargs)

    monkeypatch.setattr(sweep, 'evaluate_approach', first_one_collides)
    single = dict(prepared, valid=[prepared['valid'][0]])
    result = runner.evaluate_ready_pose(validator, ready, single, runner.Meter())

    branch = result['branches'][0]
    assert len(seen) == 2
    assert seen[0] <= seen[1]
    assert branch['connected'] is True
    assert branch['duration_s'] == pytest.approx(seen[1])
    # Pre-collision rejections (joint limits, duration) are listed too; only
    # one alternative reached the collision check and failed there.
    assert branch['failure_breakdown'].get(sweep.REASON_COLLISION) == 1
    assert (result['counts']['alternatives_skipped']
            == branch['alternatives_timed'] - 2)


def test_every_collision_query_is_counted_by_phase(validator, prepared, ready):
    meter = runner.Meter()
    with runner.counting_collisions(validator, meter):
        result = runner.evaluate_ready_pose(validator, ready, prepared, meter)
    assert (meter.counts['collision_queries:collision']
            == sum(result['counts']['collision_queries_per_approach']))
    assert 'check_all_collisions' not in vars(validator)


def test_counting_restores_an_existing_instance_override(validator, monkeypatch):
    stub = lambda q, verbose=False: False
    monkeypatch.setattr(validator, 'check_all_collisions', stub, raising=False)
    meter = runner.Meter()
    with runner.counting_collisions(validator, meter):
        validator.check_all_collisions(np.zeros(7))
    assert validator.check_all_collisions is stub
    assert meter.counts == {'collision_queries:unphased': 1}


def test_the_run_is_deterministic(validator, placement, ready):
    poses = [{'configuration': ready, 'provenance': 'broad_sample',
              'anchor_name': None}]
    first = runner.run_once(validator, poses, [placement], runner.Meter())[0]
    second = runner.run_once(validator, poses, [placement], runner.Meter())[0]
    assert (runner.results_without_timing(first)
            == runner.results_without_timing(second))


# --------------------------------------------------------------------------
# Summaries, selection, projection
# --------------------------------------------------------------------------

def test_no_task_candidate_placements_do_not_count_against_a_ready_pose():
    per_placement = [
        {'classification': sweep.CONNECTED, 'connected_branches': 3,
         'best_duration_s': 2.0, 'best_max_peak_jerk': 40.0},
        {'classification': sweep.NO_TASK_CANDIDATE},
        {'classification': sweep.CONNECTED, 'connected_branches': 1,
         'best_duration_s': 3.5, 'best_max_peak_jerk': 20.0},
    ]
    summary = runner.summarise_ready_pose(per_placement)
    assert summary['connectivity'] == 1.0
    assert summary['worst_branch_count'] == 1
    assert summary['worst_duration_s'] == 3.5
    assert summary['worst_peak_jerk'] == 40.0


def test_pilot_selection_spans_rail_and_provenance():
    pool = [{'configuration': np.array([rail, 0, 0, 0, 0, 0, 0.0]),
             'provenance': 'broad_sample', 'anchor_name': None}
            for rail in np.linspace(0.05, 2.95, 32)]
    pool += [{'configuration': np.array([rail, 0, 0, 0, 0, 0, 0.0]),
              'provenance': 'anchor', 'anchor_name': name}
             for name in ('first', 'second') for rail in (0.5, 1.5, 2.5)]
    chosen = runner.pilot_ready_poses(pool)
    assert len(chosen) == 8
    assert sum(1 for c in chosen if c['provenance'] == 'anchor') == 3
    assert len({c['anchor_name'] for c in chosen if c['anchor_name']}) >= 2
    strata = {int(c['configuration'][0] / 3.0 * 8) for c in chosen}
    assert len(strata) >= 7
    assert [id(c) for c in chosen] == [id(c) for c in runner.pilot_ready_poses(pool)]


def test_projection_scales_pool_and_placements():
    projection = runner.project_runtime(10.0, 2.0, [1.0, 1.0, 3.0], pool_size=50,
                                        placements=29)
    assert projection['ready_pose_placement_evaluations'] == 1450
    assert projection['fixed_seconds'] == pytest.approx(68.0)
    assert projection['p50_seconds'] == pytest.approx(68.0 + 1450 * 1.0)
    assert projection['p95_seconds'] > projection['p50_seconds']
