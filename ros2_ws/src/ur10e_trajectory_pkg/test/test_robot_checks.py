"""Runtime checks retained from the ready-pose study."""

import numpy as np
import pytest

from ur10e_trajectory_pkg import robot_checks as checks


def _quintic_path(start, end, samples=2000):
    time = np.linspace(0.0, 1.0, samples)
    blend = 10 * time**3 - 15 * time**4 + 6 * time**5
    return np.asarray(start) + np.outer(blend, np.asarray(end) - start)


def test_collision_sampling_respects_each_joints_step_bound():
    start = np.zeros(7)
    end = np.array([1.2, 2.0, -2.5, 1.8, -2.2, 2.4, -1.9])
    path = _quintic_path(start, end)
    for scale in (0.4, 1.0, 4.0):
        bounds = checks.collision_step_bounds() * scale
        indices = checks.collision_check_indices(path, bounds)
        assert checks.achieved_step(path, indices, bounds) <= 1.0 + 1e-9


def test_tighter_bound_checks_more_and_rail_bound_is_in_metres():
    start = np.zeros(7)
    end = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    path = _quintic_path(start, end)
    tight = checks.collision_step_bounds() * 0.5
    loose = checks.collision_step_bounds() * 2.0
    indices = checks.collision_check_indices(path)
    assert len(checks.collision_check_indices(path, tight)) > len(
        checks.collision_check_indices(path, loose))
    assert np.max(np.diff(path[indices, 0])) <= checks.COLLISION_STEP_RAIL_M + 1e-9


def test_entry_state_requires_three_continuous_layers():
    prefix = np.stack([np.full(7, 0.1 * index) for index in range(3)])
    velocity, acceleration = checks.entry_state_from_prefix(prefix, 0.1)
    assert velocity.shape == acceleration.shape == (7,)
    assert np.any(np.abs(velocity) > 1e-9)
    with pytest.raises(ValueError, match='three layers'):
        checks.entry_state_from_prefix(prefix[:2], 0.1)

    discontinuous = prefix.copy()
    discontinuous[1, 6] += 2 * np.pi
    with pytest.raises(ValueError, match='not continuous'):
        checks.entry_state_from_prefix(discontinuous, 0.1)


def test_entry_state_depends_on_the_continuation():
    start = np.zeros(7)
    first = np.stack([start, start + 0.05, start + 0.10])
    second = np.stack([start, start + 0.05, start + 0.30])
    first_velocity, _ = checks.entry_state_from_prefix(first, 0.1)
    second_velocity, _ = checks.entry_state_from_prefix(second, 0.1)
    assert not np.allclose(first_velocity, second_velocity)
