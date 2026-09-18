#!/usr/bin/env python3
"""The demo client runs an exported plan: warmup first, then the validated path."""
import json

import numpy as np
import pytest

from ur10e_trajectory_pkg import ClientNode as client


def _plan(tmp_path, **overrides):
    home = [1.5, 0.0, -1.309, 1.745, -2.007, -1.396, 0.0]
    q_path = [[0.53, 0.68, -1.27, 2.05, -3.86, -2.35, -1.27 + 0.001 * i] for i in range(12)]
    plan = {'home': home, 'q_path': q_path, 'task_start': q_path[0],
            'recorded_waypoints': 2, 'spin_up_s': 2.0, 'placement': 'nominal',
            'target_frame': 'rail_base_link'}
    plan.update(overrides)
    path = tmp_path / 'plan.json'
    path.write_text(json.dumps(plan))
    return path


def test_a_valid_plan_loads_with_arrays(tmp_path):
    plan = client.load_task_plan(_plan(tmp_path))
    assert plan['q_path'].shape == (12, 7)
    np.testing.assert_array_equal(plan['task_start'], plan['q_path'][0])


def test_a_plan_missing_a_field_is_refused(tmp_path):
    path = _plan(tmp_path)
    data = json.loads(path.read_text())
    del data['spin_up_s']
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='missing'):
        client.load_task_plan(path)


def test_the_warmup_target_must_be_the_path_start(tmp_path):
    with pytest.raises(ValueError, match='first configuration'):
        client.load_task_plan(_plan(tmp_path, task_start=[0.0] * 7))


def test_targets_in_the_carriage_frame_are_refused(tmp_path):
    with pytest.raises(ValueError, match='rail_base_link'):
        client.load_task_plan(_plan(tmp_path, target_frame='base_link'))


def test_rebuilt_targets_must_match_the_path_one_to_one(tmp_path):
    """recorded_waypoints 2 with a 2 s spin-up rebuilds 12 targets, matching
    the 12-row path; a 13-row path does not match."""
    try:
        plan = client.load_task_plan(_plan(tmp_path))
        x, *_ = client.plan_targets(plan)
    except FileNotFoundError:
        pytest.skip('packaged trajectory CSV not present')
    assert len(x) == 12
    longer = client.load_task_plan(_plan(
        tmp_path, q_path=plan['q_path'].tolist() + [plan['q_path'][-1].tolist()],
        task_start=plan['q_path'][0].tolist()))
    with pytest.raises(ValueError, match='targets number'):
        client.plan_targets(longer)
