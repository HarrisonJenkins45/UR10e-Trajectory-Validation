#!/usr/bin/env python3
"""Command 1 as a route: one leg per call, measured at every resting point.

A start whose straight move collides is blocked, not unreachable, so the
warmup may stand still at an intermediate pose and go around. The server
plans every leg itself and refuses the whole route if any leg fails, and the
arm's position at each dwell is checked against what that leg plans to start
from -- the only evidence a previous leg arrived where it planned, since this
node does not subscribe to the joint states it publishes.
"""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg import Validate_trajServer as server
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description'),
        framerate=30)


@pytest.fixture(scope='module')
def home():
    return np.concatenate(([1.5], np.deg2rad([0.0, -75.0, 100.0, -115.0, -80.0, 0.0])))


@pytest.fixture(scope='module')
def via(home):
    return home + np.concatenate(([0.3], np.deg2rad([10.0, 5.0, -5.0, 5.0, 0.0, 10.0])))


@pytest.fixture(scope='module')
def start(via):
    return via + np.concatenate(([0.4], np.deg2rad([15.0, -5.0, 5.0, -10.0, 10.0, 5.0])))


def test_a_two_segment_route_plays_one_leg_at_a_time(validator, home, via, start):
    """Each call plays one leg and reports which, so the caller knows another
    is due and where the arm should be when it asks."""
    points = [home, via, start]

    ok_first, message_first, frames_first, route = server.plan_and_validate_route(
        validator, points, 0, home, 30.0)
    assert ok_first is True, message_first
    assert 'segment 1 of 2' in message_first
    assert len(route['segments']) == 2
    np.testing.assert_allclose(frames_first[0], home, atol=1e-9)
    np.testing.assert_allclose(frames_first[-1], via, atol=1e-9)

    ok_second, message_second, frames_second, _ = server.plan_and_validate_route(
        validator, points, 1, via, 30.0)
    assert ok_second is True, message_second
    assert 'segment 2 of 2' in message_second
    np.testing.assert_allclose(frames_second[0], via, atol=1e-9)
    np.testing.assert_allclose(frames_second[-1], start, atol=1e-9)


def test_a_failing_leg_is_refused_by_name(validator, home, via, start, monkeypatch):
    """"The warmup failed" does not say which leg is blocked, and the arm must
    not be left standing at a via pose it cannot leave: the whole route is
    validated before the first leg plays."""
    from ur10e_trajectory_pkg import continuous_validator as cv

    good = {'passed': True, 'limit_violations': {}, 'position_limit_violations': [],
            'collision_found': False,
            'self_clearance': {'passed': True, 'min_distance_m': 0.02}}
    bad = dict(good, passed=False, collision_found=True)
    verdicts = iter([good, bad])
    monkeypatch.setattr(cv, 'validate_warmup',
                        lambda validator, record, limits=None: next(verdicts))

    ok, message, frames, _ = server.plan_and_validate_route(
        validator, [home, via, start], 0, home, 30.0)
    assert ok is False
    assert 'segment 2 of 2' in message
    assert 'collision along the move' in message
    assert frames is None


def test_a_dwell_that_did_not_arrive_is_refused(validator, home, via, start):
    """The leg out of the via pose is only safe from where it was planned to
    start, so a measured position that drifted is refused rather than played."""
    drifted = via + np.concatenate(([0.05], np.zeros(6)))
    ok, message, frames, _ = server.plan_and_validate_route(
        validator, [home, via, start], 1, drifted, 30.0)
    assert ok is False
    assert 'dwell at resting point 2' in message
    assert 'rail' in message
    assert frames is None

    turned = via + np.concatenate(([0.0], np.deg2rad([0.0, 0.0, 3.0, 0.0, 0.0, 0.0])))
    ok_arm, message_arm, _, _ = server.plan_and_validate_route(
        validator, [home, via, start], 1, turned, 30.0)
    assert ok_arm is False
    assert 'arm joint' in message_arm


def test_the_straight_case_is_the_one_leg_route(validator, home, start):
    """One leg must not pay for the format: the same call with no via poses."""
    ok, message, frames, route = server.plan_and_validate_route(
        validator, [home, start], 0, home, 30.0)
    assert ok is True, message
    assert len(route['segments']) == 1
    # A straight move reads as a warmup; "segment 1 of 1" is noise in a log.
    assert 'Warmup validated' in message and 'segment' not in message
    np.testing.assert_allclose(frames[-1], start, atol=1e-9)


def test_rest_points_must_end_at_the_task_start(home, via, start):
    """A route whose last resting point is not the task start would leave the
    arm somewhere command 2 was not validated from."""
    flat = np.concatenate([home, via, start]).tolist()
    points = server.resolve_rest_points(flat, home, start)
    assert len(points) == 3

    with pytest.raises(ValueError, match='last resting point'):
        server.resolve_rest_points(flat, home, via)
    with pytest.raises(ValueError, match=r'N >= 2'):
        server.resolve_rest_points(list(home), home, start)
    empty = server.resolve_rest_points([], home, start)
    assert len(empty) == 2
    np.testing.assert_allclose(empty[0], home)
    np.testing.assert_allclose(empty[1], start)
