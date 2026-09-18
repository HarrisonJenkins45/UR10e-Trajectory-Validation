#!/usr/bin/env python3
"""The benchmark harness: input checks, the three identities, and classification.

The classifier decides whether a trajectory failed for a reason segmentation
could address, so each class is pinned against the artifact shape it reads.
"""
import json

import numpy as np
import pandas as pd
import pytest
from scipy.spatial.transform import Rotation

from ur10e_trajectory_pkg import benchmark, home_pose, pipeline
from ur10e_trajectory_pkg import graph_planner

DT = 0.1


def _recording(path, count=30, rate=(0.05, -0.03, 0.04), attitude=None,
               time_offset=0.0, positions=None, flip_signs=False):
    """A SISIFOS-shaped CSV with a tri-axial tumble."""
    t = np.arange(count) * DT
    rotations = Rotation.from_rotvec(np.outer(t, rate)
                                     + np.outer(np.sin(2 * t), [0.02, 0.01, -0.02]))
    if attitude is not None:
        rotations = attitude * rotations
    quaternions = rotations.as_quat()
    if flip_signs:
        quaternions[1::2] *= -1.0
    columns = {'timestamp': t + time_offset}
    base = (np.tile([6.6e6, 2.0e5, -1.0e5], (count, 1)) if positions is None
            else positions)
    for index, axis in enumerate('xyz'):
        columns[f'p_G_I_{axis}'] = base[:, index]
        columns[f'p_C_I_{axis}'] = base[:, index] + 46.5
    for index, axis in enumerate('xyzw'):
        columns[f'q_I_G_{axis}'] = quaternions[:, index]
    columns.update({'q_I_C_x': 0.0, 'q_I_C_y': 0.0, 'q_I_C_z': 0.0, 'q_I_C_w': 1.0})
    pd.DataFrame(columns).to_csv(path, index=False)
    return path


# --------------------------------------------------------------------------
# Input
# --------------------------------------------------------------------------

def test_a_well_formed_recording_passes_and_reports_its_rate(tmp_path):
    record = benchmark.inspect_recording(_recording(tmp_path / 'ok.csv'), 20)
    assert record['passed'] is True and record['problems'] == []
    assert record['samples'] == 30 and record['step_s'] == pytest.approx(DT)
    assert 1.0 < record['angular_rate_deg_s']['median'] < 10.0
    assert record['source_digest']


def test_missing_columns_are_an_input_failure(tmp_path):
    path = tmp_path / 'bad.csv'
    pd.read_csv(_recording(tmp_path / 'ok.csv')).drop(columns=['q_I_G_w']).to_csv(
        path, index=False)
    record = benchmark.inspect_recording(path)
    assert record['passed'] is False and 'q_I_G_w' in record['problems'][0]


def test_uneven_timestamps_are_an_input_failure(tmp_path):
    path = _recording(tmp_path / 'uneven.csv')
    frame = pd.read_csv(path)
    frame.loc[10:, 'timestamp'] += 0.01
    frame.to_csv(path, index=False)
    record = benchmark.inspect_recording(path)
    assert record['passed'] is False and 'uniformly' in record['problems'][0]


def test_unnormalised_quaternions_are_an_input_failure(tmp_path):
    path = _recording(tmp_path / 'scaled.csv')
    frame = pd.read_csv(path)
    frame['q_I_G_w'] *= 1.01
    frame.to_csv(path, index=False)
    record = benchmark.inspect_recording(path)
    assert record['passed'] is False and 'norm' in record['problems'][0]


def test_too_few_samples_for_the_run_is_an_input_failure(tmp_path):
    record = benchmark.inspect_recording(_recording(tmp_path / 'ok.csv'), 5000)
    assert record['passed'] is False and 'fewer than' in record['problems'][0]


# --------------------------------------------------------------------------
# Identities
# --------------------------------------------------------------------------

def _frame(path):
    frame = pd.read_csv(path)
    return frame['timestamp'], frame[['q_I_G_x', 'q_I_G_y', 'q_I_G_z', 'q_I_G_w']]


def test_the_motion_digest_ignores_sign_time_offset_and_initial_attitude(tmp_path):
    reference = benchmark.motion_digest(*_frame(_recording(tmp_path / 'a.csv')))
    turned = Rotation.from_euler('xyz', [30, -50, 110], degrees=True)
    for name, kwargs in (('sign', {'flip_signs': True}),
                         ('offset', {'time_offset': 1234.5}),
                         ('attitude', {'attitude': turned})):
        other = _recording(tmp_path / f'{name}.csv', **kwargs)
        assert benchmark.motion_digest(*_frame(other)) == reference, name
    faster = _recording(tmp_path / 'faster.csv', rate=(0.08, -0.03, 0.04))
    assert benchmark.motion_digest(*_frame(faster)) != reference


def test_the_task_digest_ignores_camera_and_translation_but_not_attitude(tmp_path):
    """Nominal placement takes its rotation from the first attitude, so the
    same tumble from another starting attitude is another IK case."""
    from ur10e_trajectory_pkg.ClientNode import build_trajectory_targets

    def digest(path):
        return benchmark.task_digest(*build_trajectory_targets(path, 20, spin_up_s=0.4))

    reference = digest(_recording(tmp_path / 'a.csv'))
    moved = np.column_stack([np.arange(30) * 7.5e3, np.arange(30) ** 2, -np.arange(30)])
    assert digest(_recording(tmp_path / 'translated.csv', positions=moved)) == reference
    assert digest(_recording(tmp_path / 'signs.csv', flip_signs=True)) == reference
    turned = Rotation.from_euler('xyz', [30, -50, 110], degrees=True)
    assert digest(_recording(tmp_path / 'turned.csv', attitude=turned)) != reference


def test_digests_fold_negative_zero_and_sub_quantum_noise():
    times = np.arange(5) * DT
    quaternions = Rotation.from_rotvec(np.outer(times, [0.1, 0.0, 0.0])).as_quat()
    x = np.zeros(5)
    noisy = x - 1e-15
    assert benchmark.task_digest(x, x, x, quaternions, times) == benchmark.task_digest(
        noisy, x, x, quaternions, times)


def test_grouping_deduplicates_on_task_and_labels_shared_motion():
    records = [{'case_id': 'a', 'motion_digest': 'm1', 'task_digest': 't1'},
               {'case_id': 'b', 'motion_digest': 'm1', 'task_digest': 't1'},
               {'case_id': 'c', 'motion_digest': 'm1', 'task_digest': 't2'},
               {'case_id': 'd', 'motion_digest': 'm2', 'task_digest': 't3'}]
    groups = {g['case_id']: g for g in benchmark.group_identities(records)}
    assert groups['a']['duplicate_of'] is None and groups['b']['duplicate_of'] == 'a'
    assert groups['c']['duplicate_of'] is None
    assert groups['a']['same_motion_different_task'] == ['c']
    assert groups['c']['same_motion_different_task'] == ['a', 'b']
    assert groups['d']['same_motion_different_task'] == []


# --------------------------------------------------------------------------
# Classification
# --------------------------------------------------------------------------

PASSED_INPUT = {'passed': True, 'problems': []}


def _graph(generated, removed_condition=None, removed_clearance=None, first=None,
           complete=False, home_kept=None):
    layers = len(generated)
    filters = {'condition': {'candidates_per_layer': generated,
                             'removed_per_layer': removed_condition or [0] * layers},
               'self_clearance': {'removed_per_layer': removed_clearance or [0] * layers}}
    if home_kept is not None:
        filters['home_reachable'] = {'kept': home_kept, 'layer_0_candidates': generated[0],
                                     'routes': {'direct': home_kept, 'via': 0}}
    return {'candidate_filters': filters, 'complete_path': complete,
            'first_disconnected_layer': first, 'layers': layers}


def _failed(stage, status=None):
    return {'status': status, 'failed_stage': stage, 'stages': []}


def test_counts_after_filters_subtract_each_filter_and_take_the_home_at_layer_0():
    graph = _graph([10, 8, 6], removed_condition=[1, 2, 3], removed_clearance=[0, 1, 3],
                   home_kept=4)
    assert benchmark.candidates_after_filters(graph) == [4, 5, 0]
    assert benchmark.candidates_after_filters({'candidate_filters': {}}) is None


def test_an_input_failure_is_classified_before_any_stage():
    verdict = benchmark.classify({'passed': False, 'problems': ['timestamps']}, None,
                                 None, None)
    assert verdict == {'class': 'input', 'detail': 'timestamps'}


def test_a_passing_run_is_a_pass():
    assert benchmark.classify(PASSED_INPUT, {'status': 'ok'}, None, None)['class'] == 'pass'


def test_status_ok_beside_a_failed_stage_is_not_a_pass():
    """Pipelines before the fix summarised a commands stage that ran to the
    end, then failed warmup validation, as status 'ok'."""
    commands = {'status': 'ok', 'refinement': {'status': 'refined'},
                'refinement_meets_acceptance': True,
                'warmup_validation': {'passed': False},
                'task_validation': {'passed': True}}
    verdict = benchmark.classify(PASSED_INPUT, _failed(pipeline.STAGE_COMMANDS, 'ok'),
                                 None, commands)
    assert verdict['class'] == 'warmup_home'


def test_every_rung_is_classified_from_its_own_artifacts(tmp_path):
    """A rung failing on its warmup must not read as needing more spin-up."""
    case = tmp_path / 'case'
    first, second = case / 'spin_up_0.4s', case / 'spin_up_0.6s'
    first.mkdir(parents=True)
    second.mkdir()
    (first / 'commands.json').write_text(json.dumps({
        'status': 'ok', 'refinement': {'status': 'refined'},
        'refinement_meets_acceptance': True, 'warmup_validation': {'passed': False},
        'task_validation': {'passed': True}}))
    (case / 'pipeline.json').write_text(json.dumps({
        'status': 'ok', 'artifacts': {'commands': str(second / 'commands.json')},
        'spin_up_ladder': {'rungs_s': [0.4, 0.6, 1.0, 2.0], 'tried': [
            {'spin_up_s': 0.4, 'returncode': 1, 'failed_stage': 'commands',
             'status': 'ok', 'seconds': 1800.0},
            {'spin_up_s': 0.6, 'returncode': 0, 'status': 'ok', 'seconds': 900.0}]}}))
    rungs = benchmark.rung_classifications(str(case), PASSED_INPUT)
    assert [(rung['spin_up_s'], rung['class']) for rung in rungs] == [
        (0.4, 'warmup_home'), (0.6, 'pass')]

    stored = {'case_id': 'x', 'input': PASSED_INPUT,
              'classification': {'class': 'warmup_home', 'detail': None}}
    reclassified = benchmark.reclassify_case(stored, str(case))
    assert reclassified['classification']['class'] == 'pass'
    assert reclassified['stored_classification']['class'] == 'warmup_home'


def test_an_emptied_layer_is_candidate_discovery_not_connectivity():
    graph = _graph([9, 5, 4], removed_clearance=[0, 1, 4], first=2)
    verdict = benchmark.classify(PASSED_INPUT, _failed(pipeline.STAGE_GRAPH), graph, None)
    assert verdict['class'] == 'candidate_discovery' and 'layer 2' in verdict['detail']


def test_a_populated_but_unreached_layer_is_graph_connectivity():
    graph = _graph([9, 5, 4], first=2)
    verdict = benchmark.classify(PASSED_INPUT, _failed(pipeline.STAGE_GRAPH), graph, None)
    assert verdict['class'] == 'graph_connectivity' and '4 candidates' in verdict['detail']


def test_no_reachable_start_at_layer_0_is_warmup_home():
    graph = _graph([9, 5], first=0, home_kept=0)
    verdict = benchmark.classify(PASSED_INPUT, _failed(pipeline.STAGE_GRAPH), graph, None)
    assert verdict['class'] == 'warmup_home'


def test_a_crashed_generator_is_candidate_discovery():
    verdict = benchmark.classify(PASSED_INPUT, _failed(pipeline.STAGE_CANDIDATES), None, None)
    assert verdict['class'] == 'candidate_discovery'


@pytest.mark.parametrize('status', [pipeline.NO_VALID_WINDING, home_pose.NO_REACHABLE_START,
                                    home_pose.HOME_RECHECK_FAILED])
def test_home_and_winding_statuses_are_warmup_home(status):
    verdict = benchmark.classify(PASSED_INPUT, _failed(pipeline.STAGE_COMMANDS, status),
                                 None, None)
    assert verdict == {'class': 'warmup_home', 'detail': status}


def test_an_unrefined_path_is_refinement_even_though_validation_also_fails():
    commands = {'refinement': {'status': 'refinement_failed'},
                'refinement_meets_acceptance': False,
                'task_validation': {'passed': False, 'limit_violations': {}}}
    verdict = benchmark.classify(PASSED_INPUT, _failed(pipeline.STAGE_COMMANDS),
                                 None, commands)
    assert verdict['class'] == 'refinement'


def test_a_failed_warmup_is_warmup_home():
    commands = {'refinement': {'status': 'refined'}, 'refinement_meets_acceptance': True,
                'warmup_validation': {'passed': False},
                'task_validation': {'passed': True}}
    verdict = benchmark.classify(PASSED_INPUT, _failed(pipeline.STAGE_COMMANDS),
                                 None, commands)
    assert verdict['class'] == 'warmup_home'


def test_continuous_failures_name_every_reason():
    commands = {'refinement': {'status': 'refined'}, 'refinement_meets_acceptance': True,
                'warmup_validation': {'passed': True},
                'task_validation': {
                    'passed': False,
                    'limit_violations': {'wrist_1_joint': {'jerk': {}},
                                         'linear_rail_joint': {'velocity': {}}},
                    'collision': {'collision_found': True},
                    'self_clearance': {'passed': False, 'min_distance_m': 0.0081},
                    'tracking': {'within_tolerance': True},
                    'conditioning_ok': False,
                    'conditioning': {'max_condition_number': 61.2,
                                     'twist_status': 'infeasible'}}}
    verdict = benchmark.classify(PASSED_INPUT, _failed(pipeline.STAGE_COMMANDS),
                                 None, commands)
    assert verdict['class'] == 'continuous_limit'
    for reason in ('linear_rail_joint.velocity', 'wrist_1_joint.jerk', 'collision',
                   'self-clearance 8.1 mm', 'conditioning 61.2', 'twist infeasible'):
        assert reason in verdict['detail']


def test_the_graph_records_generated_candidates_per_layer(tmp_path):
    """What lets a disconnected layer be told from an empty one, without
    re-reading a candidates file that runs to gigabytes."""
    entry = {'rail_position': 0.5, 'q_arm_canonical': [0.0] * 6,
             'arm_condition_number': 10.0}
    document = {'candidates': {
        '0': [entry, dict(entry, arm_condition_number=80.0)],
        '1': [entry, dict(entry, provenance_tag='tracking_oracle')]}}
    path = tmp_path / 'candidates.json'
    path.write_text(json.dumps(document))
    _, loaded = graph_planner.load_candidates(str(path), 2, include_oracle=False,
                                              max_condition=25.0)
    assert loaded['condition_filter']['candidates_per_layer'] == [2, 1]
    assert loaded['condition_filter']['removed_per_layer'] == [1, 0]


def test_the_report_counts_classes_and_carries_duplicates():
    identities = {'waypoints': 500, 'records': [
        {'case_id': 'a', 'input': {'passed': True}, 'task_digest': 't1',
         'motion_digest': 'm1', 'grouping': {'duplicate_of': None,
                                             'same_motion_different_task': []}},
        {'case_id': 'b', 'input': {'passed': True}, 'task_digest': 't1',
         'motion_digest': 'm1', 'grouping': {'duplicate_of': 'a',
                                             'same_motion_different_task': []}}]}
    cases = [{'case_id': 'a', 'classification': {'class': 'pass', 'detail': None},
              'metrics': {'complete_path': True}, 'seconds': 60.0, 'peak_rss_gb': 1.0}]
    document = benchmark.report(identities, cases)
    assert document['class_counts'] == {'pass': 1}
    rows = {row['case_id']: row for row in document['rows']}
    assert rows['b']['ran'] is False and rows['b']['grouping']['duplicate_of'] == 'a'
    text = benchmark.markdown(document)
    assert 'b (= a)' in text and 'not run' in text
