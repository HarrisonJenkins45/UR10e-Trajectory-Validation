#!/usr/bin/env python3
"""The demo client runs an exported plan: warmup first, then the validated path."""
import json

import numpy as np
import pytest

from ur10e_trajectory_pkg import plan_artifact, target_builder, trajectory_input


def _plan(tmp_path, **overrides):
    home = [1.5, 0.0, -1.309, 1.745, -2.007, -1.396, 0.0]
    q_path = [[0.53, 0.68, -1.27, 2.05, -3.86, -2.35, -1.27 + 0.001 * i] for i in range(12)]
    plan = {'schema_version': plan_artifact.SCHEMA_VERSION,
            'home': home, 'q_path': q_path, 'task_start': q_path[0],
            'recorded_waypoints': 2, 'start_index': 0, 'spin_up_s': 2.0,
            'placement': 'nominal', 'mount': target_builder.mount_record(),
            'target_frame': 'rail_base_link',
            'recording': {'csv_path': trajectory_input.DEFAULT_CSV_PATH,
                          'csv_sha256': trajectory_input.source_digest(
                              trajectory_input.DEFAULT_CSV_PATH) or 'a' * 64},
            'warmup_route': {'route_kind': 'direct',
                             'rest_points': [home, q_path[0]],
                             'segment_durations_s': [5.0], 'dwell_s': 0.0,
                             'total_duration_s': 5.0}}
    plan.update(overrides)
    path = tmp_path / 'plan.json'
    path.write_text(json.dumps(plan))
    return path


def test_a_valid_plan_loads_with_arrays(tmp_path):
    plan = plan_artifact.load_plan(_plan(tmp_path))
    assert plan['q_path'].shape == (12, 7)
    np.testing.assert_array_equal(plan['task_start'], plan['q_path'][0])


def test_a_plan_missing_a_field_is_refused(tmp_path):
    path = _plan(tmp_path)
    data = json.loads(path.read_text())
    del data['spin_up_s']
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='missing'):
        plan_artifact.load_plan(path)


def test_the_warmup_target_must_be_the_path_start(tmp_path):
    with pytest.raises(ValueError, match='first configuration'):
        plan_artifact.load_plan(_plan(tmp_path, task_start=[0.0] * 7))


def test_targets_in_the_carriage_frame_are_refused(tmp_path):
    with pytest.raises(ValueError, match='rail_base_link'):
        plan_artifact.load_plan(_plan(tmp_path, target_frame='base_link'))


def test_nonfinite_joint_coordinates_are_refused(tmp_path):
    with pytest.raises(ValueError, match='non-finite'):
        plan_artifact.load_plan(_plan(tmp_path, home=[float('nan')] * 7))


def test_malformed_warmup_is_refused_by_the_shared_loader(tmp_path):
    path = _plan(tmp_path)
    plan = json.loads(path.read_text())
    plan['warmup_route'] = {'rest_points': [plan['home'], plan['task_start']],
                            'segment_durations_s': [float('inf')], 'dwell_s': 0.0}
    path.write_text(json.dumps(plan))
    with pytest.raises(ValueError, match='invalid segment durations'):
        plan_artifact.load_plan(path)


def test_new_schema_is_not_silently_treated_as_current(tmp_path):
    with pytest.raises(ValueError, match='schema_version'):
        plan_artifact.load_plan(_plan(tmp_path, schema_version=2))


@pytest.mark.parametrize('field', ['schema_version', 'warmup_route', 'mount',
                                   'recording', 'start_index'])
def test_execution_fields_cannot_be_inferred(tmp_path, field):
    path = _plan(tmp_path)
    data = json.loads(path.read_text())
    del data[field]
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='missing'):
        plan_artifact.load_plan(path)


def test_a_different_tool_mount_is_refused_when_plan_loads(tmp_path):
    with pytest.raises(ValueError, match='T_EG differs'):
        plan_artifact.load_plan(_plan(
            tmp_path, mount={'name': 'T_EG', 'transform':
                             [[1, 0, 0, 0.1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]}))


def test_rebuilt_targets_must_match_the_path_one_to_one(tmp_path):
    """recorded_waypoints 2 with a 2 s spin-up rebuilds 12 targets, matching
    the 12-row path; a 13-row path does not match."""
    try:
        plan = plan_artifact.load_plan(_plan(tmp_path))
        x, *_ = plan_artifact.plan_targets(plan)
    except FileNotFoundError:
        pytest.skip('packaged trajectory CSV not present')
    assert len(x) == 12
    longer = plan_artifact.load_plan(_plan(
        tmp_path, q_path=plan['q_path'].tolist() + [plan['q_path'][-1].tolist()],
        task_start=plan['q_path'][0].tolist()))
    with pytest.raises(ValueError, match='targets number'):
        plan_artifact.plan_targets(longer)
