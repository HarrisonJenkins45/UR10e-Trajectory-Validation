#!/usr/bin/env python3
"""Warmup gate: home to the chosen start, rest to rest, or an honest refusal."""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg import motion_limits, warmup
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description'))


@pytest.fixture(scope='module')
def home():
    return np.concatenate(([1.5], np.deg2rad([0.0, -75.0, 100.0, -115.0, -80.0, 0.0])))


@pytest.fixture(scope='module')
def start(home):
    return home + np.concatenate(([0.4], np.deg2rad([20.0, -10.0, 15.0, 5.0, 30.0, 45.0])))


def test_the_warmup_ends_exactly_at_the_start_and_at_rest(validator, home, start):
    result = warmup.plan_warmup(validator, home, start)
    assert result['status'] == warmup.OK
    positions = np.asarray(result['positions'])
    np.testing.assert_allclose(positions[0], home, atol=1e-12)
    np.testing.assert_allclose(positions[-1], start, atol=1e-12)
    _, _, velocity, acceleration = warmup.sample_rest_to_rest(
        home, start, result['duration_s'])
    np.testing.assert_allclose(velocity[[0, -1]], 0.0, atol=1e-12)
    np.testing.assert_allclose(acceleration[[0, -1]], 0.0, atol=1e-12)


def test_the_closed_form_duration_touches_a_limit_and_never_exceeds_one(validator, home, start):
    """Sampled densely, the binding limit is reached and none is exceeded."""
    result = warmup.plan_warmup(validator, home, start, rate_hz=5000.0)
    peak = max(result['peak_velocity_ratio'], result['peak_acceleration_ratio'])
    assert peak <= 1.0 + 1e-9
    assert peak == pytest.approx(1.0, abs=1e-4)
    assert result['binding']['kind'] in ('velocity', 'acceleration')


def test_the_binding_joint_is_the_slowest_to_move(home):
    velocity = np.ones(7)
    acceleration = np.full(7, 100.0)
    start = home.copy()
    start[4] += 2.0
    duration, joint, kind = warmup.rest_to_rest_duration(home, start, velocity,
                                                         acceleration)
    assert joint == 4 and kind == 'velocity'
    assert duration == pytest.approx(15.0 * 2.0 / 8.0)


def test_no_motion_takes_the_minimum_duration(validator, home):
    result = warmup.plan_warmup(validator, home, home)
    assert result['status'] == warmup.OK
    assert result['duration_s'] == warmup.MINIMUM_DURATION_S
    assert result['binding']['kind'] == 'minimum_duration'


def test_a_blocked_straight_move_reports_no_direct_warmup(validator, home, start,
                                                          monkeypatch):
    monkeypatch.setattr(validator, 'check_all_collisions',
                        lambda q, verbose=False: True)
    result = warmup.plan_warmup(validator, home, start)
    assert result['status'] == warmup.NO_DIRECT_WARMUP
    assert 'positions' not in result
    assert result['collision_queries'] == 1


def test_an_endpoint_outside_the_joint_limits_is_refused(validator, home):
    outside = home.copy()
    outside[3] = np.pi + 0.1
    result = warmup.plan_warmup(validator, home, outside)
    assert result['status'] == warmup.OUTSIDE_JOINT_LIMITS


def test_the_binding_status_comes_from_the_urdf_check(validator, home):
    start = home.copy()
    start[0] += 1.2                  # rail-dominated, inside the 0-3 m travel
    result = warmup.plan_warmup(validator, home, start)
    assert result['binding']['joint'] == 'linear_rail_joint'
    statuses = motion_limits.limit_statuses(validator)
    kind = result['binding']['kind']
    assert result['binding']['status'] == statuses[kind][0]
