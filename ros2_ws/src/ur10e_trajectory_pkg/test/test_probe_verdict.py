"""The production probe classifier distinguishes search failures by evidence."""

import pytest

from ur10e_trajectory_pkg import home_pose, pipeline, probe_verdict

PASSED_INPUT = {'passed': True, 'problems': []}


def graph(generated, removed_condition=None, removed_clearance=None, first=None,
          home_kept=None):
    layers = len(generated)
    filters = {'condition': {'candidates_per_layer': generated,
                             'removed_per_layer': removed_condition or [0] * layers},
               'self_clearance': {'removed_per_layer': removed_clearance or [0] * layers}}
    if home_kept is not None:
        filters['home_reachable'] = {'kept': home_kept,
                                     'layer_0_candidates': generated[0]}
    return {'candidate_filters': filters, 'complete_path': False,
            'first_disconnected_layer': first}


def failed(stage, status=None):
    return {'status': status, 'failed_stage': stage, 'stages': []}


def test_filter_counts_and_pass_contract():
    item = graph([10, 8, 6], [1, 2, 3], [0, 1, 3], home_kept=4)
    assert probe_verdict.candidates_after_filters(item) == [4, 5, 0]
    assert probe_verdict.classify(PASSED_INPUT, {'status': 'ok'}, None, None) == {
        'class': 'pass', 'detail': None}
    assert probe_verdict.classify({'passed': False, 'problems': ['timestamps']},
                                  None, None, None)['class'] == 'input'


@pytest.mark.parametrize('item, expected', [
    (graph([9, 5, 4], removed_clearance=[0, 1, 4], first=2),
     'candidate_discovery'),
    (graph([9, 5, 4], first=2), 'graph_connectivity'),
    (graph([9, 5], first=0, home_kept=0), 'warmup_home'),
])
def test_graph_disconnection_classification(item, expected):
    assert probe_verdict.classify(PASSED_INPUT, failed(pipeline.STAGE_GRAPH),
                                  item, None)['class'] == expected


@pytest.mark.parametrize('status', [pipeline.NO_VALID_WINDING,
                                    home_pose.NO_REACHABLE_START,
                                    home_pose.HOME_RECHECK_FAILED])
def test_home_failures_are_not_task_failures(status):
    assert probe_verdict.classify(PASSED_INPUT,
                                  failed(pipeline.STAGE_COMMANDS, status),
                                  None, None)['class'] == 'warmup_home'


def test_validation_failure_keeps_all_reasons():
    commands = {'refinement': {'status': 'refined'}, 'refinement_meets_acceptance': True,
                'warmup_validation': {'passed': True},
                'task_validation': {
                    'passed': False,
                    'limit_violations': {'wrist_1_joint': {'jerk': {}},
                                         'linear_rail_joint': {'velocity': {}}},
                    'collision': {'collision_found': True},
                    'self_clearance': {'passed': False, 'min_distance_m': 0.0081},
                    'conditioning_ok': False,
                    'conditioning': {'max_condition_number': 61.2,
                                     'twist_status': 'infeasible'}}}
    result = probe_verdict.classify(PASSED_INPUT, failed(pipeline.STAGE_COMMANDS),
                                    None, commands)
    assert result['class'] == 'continuous_limit'
    for reason in ('linear_rail_joint.velocity', 'wrist_1_joint.jerk', 'collision',
                   'self-clearance 8.1 mm', 'conditioning 61.2', 'twist infeasible'):
        assert reason in result['detail']


def test_ok_status_cannot_hide_failed_warmup():
    commands = {'refinement': {'status': 'refined'}, 'refinement_meets_acceptance': True,
                'warmup_validation': {'passed': False}, 'task_validation': {'passed': True}}
    result = probe_verdict.classify(PASSED_INPUT,
                                    failed(pipeline.STAGE_COMMANDS, 'ok'), None, commands)
    assert result['class'] == 'warmup_home'
