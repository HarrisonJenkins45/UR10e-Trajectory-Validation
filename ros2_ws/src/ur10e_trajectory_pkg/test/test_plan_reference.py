"""The executed reference must be the curves the validator checked."""

import numpy as np
import pytest

from ur10e_trajectory_pkg import joint_motion, plan_reference


HOME = np.array([1.5, 0.0, -1.3, 1.7, -2.0, -1.4, 0.0])
VIA = HOME + np.array([0.05, 0.2, -0.1, 0.1, 0.0, 0.1, 0.5])
START = VIA + np.array([0.02, -0.1, 0.05, 0.0, 0.1, 0.0, 0.3])


def synthetic_plan():
    path = START + np.outer(np.linspace(0.0, 1.0, 21), [0.1, 0.2, 0, 0, 0.1, 0, 0.4])
    return {
        'home': HOME, 'q_path': path,
        'warmup_route': {'rest_points': np.array([HOME, VIA, START]),
                         'segment_durations_s': [2.0, 1.5], 'dwell_s': 0.5},
    }


def test_warmup_legs_match_the_validated_quintic():
    segment = plan_reference.quintic_segment('leg', HOME, VIA, 2.0)
    times, positions, velocities, _ = joint_motion.sample_rest_to_rest(HOME, VIA, 2.0, 50.0)
    for t, q, qd in zip(times, positions, velocities):
        q_ref, qd_ref = segment.evaluate(t)
        np.testing.assert_allclose(q_ref, q, atol=1e-12)
        np.testing.assert_allclose(qd_ref, qd, atol=1e-12)


def test_task_matches_the_validated_pchip_curve():
    path = synthetic_plan()['q_path']
    segment = plan_reference.task_segment(path, 0.1)
    times, positions, velocities = joint_motion.task_playback(path, 0.1, 30.0)
    for t, q, qd in zip(times, positions, velocities):
        q_ref, qd_ref = segment.evaluate(t)
        np.testing.assert_allclose(q_ref, q, atol=1e-12)
        np.testing.assert_allclose(qd_ref, qd, atol=1e-12)


def test_plan_segments_are_continuous_and_include_dwell_and_settle():
    segments = plan_reference.plan_segments(synthetic_plan(), 0.1, settle_s=1.0)
    assert [s.name for s in segments] == [
        'warmup 1/2', 'via dwell', 'warmup 2/2', 'settle', 'task']
    reference = plan_reference.Reference(segments)
    assert reference.duration == pytest.approx(2.0 + 0.5 + 1.5 + 1.0 + 2.0)
    np.testing.assert_allclose(reference.start, HOME)
    np.testing.assert_allclose(reference.end, synthetic_plan()['q_path'][-1])
    boundaries = np.cumsum([s.duration for s in segments])[:-1]
    for t in boundaries:
        before, after = reference.sample(t - 1e-9)[0], reference.sample(t + 1e-9)[0]
        np.testing.assert_allclose(before, after, atol=1e-6)


def test_warmup_mode_ends_at_the_task_start():
    segments = plan_reference.plan_segments(synthetic_plan(), 0.1, include_task=False)
    np.testing.assert_allclose(plan_reference.Reference(segments).end, START)


def test_time_scale_keeps_the_path_and_scales_velocity():
    segments = plan_reference.plan_segments(synthetic_plan(), 0.1)
    full = plan_reference.Reference(segments)
    slow = plan_reference.Reference(segments, time_scale=0.25)
    assert slow.duration == pytest.approx(4 * full.duration)
    for t in np.linspace(0.0, full.duration, 37):
        q_full, qd_full, phase_full = full.sample(t)
        q_slow, qd_slow, phase_slow = slow.sample(4 * t)
        np.testing.assert_allclose(q_slow, q_full, atol=1e-12)
        np.testing.assert_allclose(qd_slow, 0.25 * qd_full, atol=1e-12)
        assert phase_slow == phase_full
    np.testing.assert_allclose(slow.peak_speeds(), 0.25 * full.peak_speeds(), rtol=1e-9)
    with pytest.raises(ValueError):
        plan_reference.Reference(segments, time_scale=1.5)


def test_reference_holds_at_rest_after_the_end():
    reference = plan_reference.Reference(plan_reference.plan_segments(synthetic_plan(), 0.1))
    q, qd, _ = reference.sample(reference.duration + 5.0)
    np.testing.assert_allclose(q, reference.end)
    assert not np.any(qd)


def test_move_to_home_respects_the_speed_limit():
    limit = np.array([0.02] + [0.2] * 6)
    move = plan_reference.move_segment(START, HOME, limit)
    reference = plan_reference.Reference([move])
    assert np.all(reference.peak_speeds() <= limit + 1e-9)
    np.testing.assert_allclose(reference.end, HOME)
    assert move.duration >= 2.0


TOLERANCE = np.array([0.01] + [0.03] * 6)
APPROACH_SPEED = np.array([0.04] + [0.4] * 6)


def approach(measured, arm_limit=0.2):
    return plan_reference.approach_segments(
        synthetic_plan(), measured, TOLERANCE, arm_limit, APPROACH_SPEED)


def test_reverse_warmup_retraces_the_certified_route():
    forward = plan_reference.Reference(
        plan_reference.plan_segments(synthetic_plan(), 0.1, include_task=False))
    back = plan_reference.Reference(plan_reference.reverse_warmup_segments(synthetic_plan()))
    assert back.duration == pytest.approx(forward.duration)
    for t in np.linspace(0.0, forward.duration, 23):
        np.testing.assert_allclose(back.sample(forward.duration - t)[0],
                                   forward.sample(t)[0], atol=1e-9)


def test_approach_from_home_is_empty():
    assert approach(HOME + 0.001) == ([], 'at plan home')


def test_approach_from_task_start_returns_along_the_warmup():
    segments, description = approach(START)
    assert 'certified warmup' in description
    reference = plan_reference.Reference(segments)
    np.testing.assert_allclose(reference.start, START)
    np.testing.assert_allclose(reference.end, HOME)


def test_approach_straightens_the_arm_before_moving_the_rail():
    measured = HOME + np.array([-1.2, 0.1, 0, 0, -0.05, 0, 0])
    segments, _ = approach(measured)
    assert [s.name for s in segments] == ['straighten arm', 'rail to home', 'at home']
    straighten, rail = segments[0], segments[1]
    # The rail is held while the arm moves, and the arm while the rail moves.
    for t in np.linspace(0.0, straighten.duration, 9):
        assert straighten.evaluate(t)[0][0] == pytest.approx(measured[0])
    for t in np.linspace(0.0, rail.duration, 9):
        np.testing.assert_allclose(rail.evaluate(t)[0][1:], HOME[1:])
    reference = plan_reference.Reference(segments)
    np.testing.assert_allclose(reference.end, HOME)
    assert np.all(reference.peak_speeds() <= APPROACH_SPEED + 1e-9)


def test_approach_refuses_a_large_uncertified_arm_sweep():
    with pytest.raises(ValueError, match='pendant'):
        approach(HOME + np.array([0, 0, 0.5, 0, 0, 0, 0]))
