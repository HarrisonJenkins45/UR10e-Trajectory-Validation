#!/usr/bin/env python3
"""Refinement gate: smooth rail, arm kept on its branch, validated or refused."""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

from ur10e_trajectory_pkg import continuous_validator
from ur10e_trajectory_pkg import path_refinement as refinement
from ur10e_trajectory_pkg.robot_checks import arm_only_ik
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path

DT = 0.1
N = 14


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description'))


@pytest.fixture(scope='module')
def smooth(validator):
    """A smooth path and the targets it reaches: rail creeping, arm turning."""
    start = np.concatenate(([1.2], np.deg2rad([10.0, -110.0, 80.0, -60.0, 70.0, 20.0])))
    path = np.stack([start + np.concatenate(([0.004 * i], np.full(6, 0.003 * i)))
                     for i in range(N)])
    poses = [validator.robot.fkine(q, end='tool0') for q in path]
    positions = np.stack([p.t for p in poses])
    quaternions = np.stack([Rotation.from_matrix(p.R).as_quat() for p in poses])
    return path, positions, quaternions


def test_the_smoothed_rail_holds_the_start_and_meets_its_rms():
    rng = np.random.default_rng(3)
    rail = np.linspace(0.5, 0.8, 200) + rng.normal(0.0, 0.003, 200)
    f, _, rms = refinement.smooth_rail(rail, 0.002)
    np.testing.assert_array_equal(f[:refinement.PINNED_START_SAMPLES], rail[0])
    assert rms == pytest.approx(0.002, rel=0.05)


def test_a_looser_level_is_smoother():
    rng = np.random.default_rng(4)
    rail = np.linspace(0.5, 0.8, 200) + rng.normal(0.0, 0.003, 200)
    tight, _, _ = refinement.smooth_rail(rail, 0.001)
    loose, _, _ = refinement.smooth_rail(rail, 0.005)
    assert (np.max(np.abs(np.diff(loose, n=2))) < np.max(np.abs(np.diff(tight, n=2))))


def _dense_smooth_rail(rail, rms_target, pinned=refinement.PINNED_START_SAMPLES):
    """The dense formulation smooth_rail used, kept as the reference."""
    n = len(rail)
    D = np.zeros((n - 2, n))
    for i in range(n - 2):
        D[i, i:i + 3] = (1.0, -2.0, 1.0)
    DtD = D.T @ D
    fixed = np.full(pinned, rail[0])

    def fit(lam):
        A = np.eye(n - pinned) + lam * DtD[pinned:, pinned:]
        b = rail[pinned:] - lam * DtD[pinned:, :pinned] @ fixed
        f = np.concatenate((fixed, np.linalg.solve(A, b)))
        return f, float(np.sqrt(np.mean((f - rail) ** 2)))

    low, high = -8.0, 14.0
    f_high, rms_high = fit(10.0 ** high)
    if rms_high <= rms_target:
        return f_high, 10.0 ** high, rms_high
    for _ in range(60):
        middle = 0.5 * (low + high)
        if fit(10.0 ** middle)[1] > rms_target:
            high = middle
        else:
            low = middle
    f, rms = fit(10.0 ** low)
    return f, 10.0 ** low, rms


@pytest.mark.parametrize('n, target', [(200, 0.002), (57, 0.001), (5, 0.0005)])
def test_the_banded_solve_gives_the_dense_fit(n, target):
    """Same system, same answer: only the storage and the solve changed."""
    rng = np.random.default_rng(n)
    rail = np.linspace(0.5, 0.9, n) + rng.normal(0.0, 0.004, n)
    f, lam, rms = refinement.smooth_rail(rail, target)
    f_ref, lam_ref, rms_ref = _dense_smooth_rail(rail, target)
    np.testing.assert_allclose(f, f_ref, atol=1e-9)
    assert lam == pytest.approx(lam_ref, rel=1e-6)
    assert rms == pytest.approx(rms_ref, abs=1e-9)


def test_the_bands_are_those_of_the_dense_normal_matrix():
    n, pinned = 12, refinement.PINNED_START_SAMPLES
    D = np.zeros((n - 2, n))
    for i in range(n - 2):
        D[i, i:i + 3] = (1.0, -2.0, 1.0)
    dense = D.T @ D
    main, first, second, coupling = refinement.second_difference_normal_bands(n, pinned)
    block = dense[pinned:, pinned:]
    np.testing.assert_array_equal(main, np.diag(block, 0))
    np.testing.assert_array_equal(first, np.diag(block, 1))
    np.testing.assert_array_equal(second, np.diag(block, 2))
    np.testing.assert_array_equal(coupling, dense[pinned:, :pinned])
    assert not np.any(np.triu(block, 3))


def test_a_full_recording_length_smooths_in_linear_memory():
    """5000 waypoints held three 5000 x 5000 arrays at once; banded, the rail
    of a full recording smooths quickly."""
    import time
    rng = np.random.default_rng(7)
    rail = np.cumsum(rng.normal(0.0, 0.002, 5001)) + 1.5
    started = time.perf_counter()
    f, _, rms = refinement.smooth_rail(rail, 0.005, bounds=(0.0, 3.0))
    assert time.perf_counter() - started < 30.0
    assert len(f) == 5001 and rms <= 0.005 + 1e-9


def test_an_unreachable_target_returns_the_smoothest_fit():
    rail = np.full(50, 1.0)
    f, _, rms = refinement.smooth_rail(rail, 0.005)
    assert rms <= 0.005
    np.testing.assert_allclose(f, 1.0, atol=1e-9)


def test_the_arm_stays_on_the_path_branch_at_the_path_rail(validator, smooth):
    path, positions, quaternions = smooth
    solved, report = refinement.solve_arm_along(validator, path, path[:, 0],
                                                positions, quaternions)
    assert solved is not None
    np.testing.assert_allclose(solved, path, atol=1e-6)
    assert report['max_arm_deviation_from_graph_rad'] < 1e-6


def test_a_rail_hop_is_smoothed_out(validator, smooth):
    """The graph can hop 3 mm along the rail at one waypoint while the arm
    compensates; smoothing the rail and re-solving the arm removes it.

    The rail starts at rest here, as it does after a spin-up: refinement pins
    the start's rail samples, which assumes exactly that.
    """
    path, positions, quaternions = smooth
    at_rest = path.copy()
    at_rest[:, 0] = path[0, 0]
    targets_positions, targets_quaternions = [], []
    for q in at_rest:
        pose = validator.robot.fkine(q, end='tool0')
        targets_positions.append(pose.t)
        targets_quaternions.append(Rotation.from_matrix(pose.R).as_quat())
    targets_positions = np.stack(targets_positions)
    targets_quaternions = np.stack(targets_quaternions)

    kinked = at_rest.copy()
    kinked[7, 0] += 0.003
    rotation = Rotation.from_quat(targets_quaternions[7]).as_matrix()
    kinked[7] = arm_only_ik(validator, kinked[7], targets_positions[7], rotation)[0]
    rail, _, _ = refinement.smooth_rail(kinked[:, 0], 0.002)
    solved, _ = refinement.solve_arm_along(validator, kinked, rail, targets_positions,
                                           targets_quaternions)
    assert solved is not None
    before = np.max(np.abs(np.diff(kinked, n=2, axis=0)))
    after = np.max(np.abs(np.diff(solved, n=2, axis=0)))
    assert after < 0.5 * before, (before, after)
    np.testing.assert_allclose(solved[0], kinked[0], atol=1e-9)


def _report(passed):
    return {'passed': passed, 'entry_velocity_ratio': 0.0, 'entry_acceleration_ratio': 0.0,
            'conditioning': {'max_condition_number': 10.0, 'twist_status': 'pass'},
            'conditioning_ok': True,
            'limit_violations': {} if passed else {'elbow_joint': {}},
            'position_limit_violations': [], 'position_limit_excursions': {},
            'collision': {'collision_found': False},
            'self_clearance': {'min_distance_m': 0.03, 'links': None, 'passed': True},
            'tracking': {'within_tolerance': True},
            'peak_command_stream': {'jerk': [1.0] * 7}}


def test_the_smoothest_passing_level_is_kept(validator, smooth, monkeypatch):
    path, positions, quaternions = smooth
    outcomes = iter([False, True, True])
    monkeypatch.setattr(continuous_validator, 'validate_task_command',
                        lambda *a, **k: _report(next(outcomes)))
    result = refinement.refine_path(validator, path, positions, quaternions, DT)
    assert result['status'] == 'refined'
    assert result['level_m'] == refinement.REFINEMENT_RMS_LEVELS_M[1]
    assert [a['passed'] for a in result['attempts']] == [False, True]


def test_if_no_level_passes_the_graph_path_is_kept(validator, smooth, monkeypatch):
    """An unvalidated path is never substituted."""
    path, positions, quaternions = smooth
    monkeypatch.setattr(continuous_validator, 'validate_task_command',
                        lambda *a, **k: _report(False))
    result = refinement.refine_path(validator, path, positions, quaternions, DT)
    assert result['status'] == 'refinement_failed'
    np.testing.assert_array_equal(result['path'], path)
    assert result['validation'] is None
    assert refinement.meets_acceptance(result) is False


def test_acceptance_is_full_validation_and_start_motion_within_the_at_rest_tolerance():
    """One acceptance everywhere. Second differences are diagnostics now; the
    jerk gate in full validation catches kinks directly."""
    attempt = {'level_m': 0.002, 'passed': True, 'max_second_difference': 5e-4,
               'start_motion': 1e-5}
    ok = {'status': 'refined', 'level_m': 0.002, 'attempts': [attempt]}
    assert refinement.meets_acceptance(ok) is True
    rough = {'status': 'refined', 'level_m': 0.002,
             'attempts': [dict(attempt, max_second_difference=2e-3)]}
    assert refinement.meets_acceptance(rough) is True
    within = {'status': 'refined', 'level_m': 0.002,
              'attempts': [dict(attempt, start_motion=0.0126)]}
    assert refinement.meets_acceptance(within) is True
    moving = {'status': 'refined', 'level_m': 0.002,
              'attempts': [dict(attempt, start_motion=0.03)]}
    assert refinement.meets_acceptance(moving) is False
    failed = {'status': 'refined', 'level_m': 0.002,
              'attempts': [dict(attempt, passed=False)]}
    assert refinement.meets_acceptance(failed) is False


def test_smoothing_stays_inside_the_rail_limits():
    """A rail that runs to the end of its travel and stops is the shape that
    breaks: the smoother rounds the corner and undershoots past the limit.
    coupled_08 left the limit by 8 mm that way, and every level failed on
    position limits with nothing else wrong."""
    n = 200
    rail = np.concatenate((np.linspace(0.6, 0.0, n // 2), np.zeros(n - n // 2)))

    free, _, _ = refinement.smooth_rail(rail, 0.005)
    assert free.min() < 0.0

    bounded, _, rms = refinement.smooth_rail(rail, 0.005, bounds=(0.0, 3.0))
    assert bounded.min() >= 0.0
    assert bounded.max() <= 3.0
    # The reported deviation is the one the caller gets, measured after the
    # clip rather than before it.
    assert rms == pytest.approx(float(np.sqrt(np.mean((bounded - rail) ** 2))))


def _attempt_stub(level, passed, violations=None):
    return {'level_m': level, 'passed': passed, 'limit_violations': violations or {},
            'collision': False, 'self_clearance_passed': True, 'tracking': True}


def test_the_fallback_level_is_not_the_smoothest_rung():
    """10 mm must sit below the declared ladder, not on top of it: as the
    smoothest rung the smoothest-passing rule would hand it to every
    placement, and more smoothing means more deviation from the branch the
    graph chose."""
    assert refinement.FALLBACK_RMS_LEVEL_M not in refinement.REFINEMENT_RMS_LEVELS_M
    assert refinement.FALLBACK_RMS_LEVEL_M > max(refinement.REFINEMENT_RMS_LEVELS_M)


def test_the_note_records_that_only_assumed_limits_refused():
    """coupled_14's case: 5 mm fails on rail jerk alone, an assumed limit,
    by 11%. The artifact has to say so, so the precedent is visible."""
    attempts = [
        _attempt_stub(0.005, False, {'linear_rail_joint': {'jerk': {'peak': 111.46, 'limit': 100.0}}}),
        _attempt_stub(0.002, False, {'linear_rail_joint': {'jerk': {'peak': 1067.8, 'limit': 100.0}}}),
        _attempt_stub(0.001, False, {'linear_rail_joint': {'jerk': {'peak': 2242.0, 'limit': 100.0}}}),
        _attempt_stub(0.010, True),
    ]
    for attempt in attempts:
        if attempt['limit_violations']:
            attempt['limit_violation_provenance'] = {
                'linear_rail_joint.jerk': {
                    'peak': attempt['limit_violations']['linear_rail_joint']['jerk']['peak'],
                    'limit': 100.0, 'status': 'assumed',
                    'exceedance_fraction': attempt['limit_violations']['linear_rail_joint']['jerk']['peak'] / 100.0 - 1.0}}

    note = refinement.fallback_note(refinement.REFINEMENT_RMS_LEVELS_M, attempts)
    assert note['declared_levels_m'] == list(refinement.REFINEMENT_RMS_LEVELS_M)
    assert [d['level_m'] for d in note['declared_attempts']] == [0.005, 0.002, 0.001]
    first = note['declared_attempts'][0]
    assert first['all_violated_limits_assumed'] is True
    assert first['worst_exceedance_fraction'] == pytest.approx(0.1146, abs=1e-4)
    assert first['other_failures'] == []


def test_violation_provenance_separates_assumed_from_certified(validator):
    """A jerk limit with no vendor figure and a certified velocity limit read
    the same in a violations dict; the artifact must tell them apart."""
    violations = {'linear_rail_joint': {'jerk': {'peak': 111.46, 'limit': 100.0}},
                  'elbow_joint': {'velocity': {'peak': 4.0, 'limit': 3.14}}}
    provenance = refinement.violation_provenance(validator, violations)
    assert provenance['linear_rail_joint.jerk']['status'] == 'assumed'
    assert provenance['linear_rail_joint.jerk']['exceedance_fraction'] == pytest.approx(0.1146, abs=1e-4)
    assert provenance['elbow_joint.velocity']['status'] in ('certified', 'provisional', 'assumed')
