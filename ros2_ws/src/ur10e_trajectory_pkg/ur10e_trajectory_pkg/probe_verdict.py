"""Classify one pipeline probe from its saved stage artifacts."""
import numpy as np

def candidates_after_filters(graph):
    """Candidates left per layer after the condition, clearance and home filters.

    None when the graph predates the per-layer count. Layer 0 is what the home
    filter kept, since that is the last filter it sees.
    """
    filters = (graph or {}).get('candidate_filters') or {}
    condition = filters.get('condition') or {}
    generated = condition.get('candidates_per_layer')
    if generated is None:
        return None
    kept = np.asarray(generated, dtype=int) - np.asarray(
        condition.get('removed_per_layer', 0), dtype=int)
    clearance = (filters.get('self_clearance') or {}).get('removed_per_layer')
    if clearance is not None:
        kept = kept - np.asarray(clearance, dtype=int)
    home = filters.get('home_reachable')
    if home is not None and len(kept):
        kept[0] = int(home['kept'])
    return kept.tolist()


def _verdict(name, detail=None):
    return {'class': name, 'detail': detail}


def _continuous_reasons(task):
    reasons = []
    violations = task.get('limit_violations') or {}
    if violations:
        reasons.append('limits ' + ', '.join(
            f'{joint}.{kind}' for joint, kinds in sorted(violations.items())
            for kind in sorted(kinds)))
    if task.get('position_limit_violations'):
        reasons.append('position limits')
    if (task.get('collision') or {}).get('collision_found'):
        reasons.append('collision')
    clearance = task.get('self_clearance') or {}
    if clearance and not clearance.get('passed', True):
        reasons.append(f"self-clearance {1000.0 * clearance['min_distance_m']:.1f} mm")
    tracking = task.get('tracking') or {}
    if tracking and not tracking.get('within_tolerance', True):
        reasons.append('tracking')
    conditioning = task.get('conditioning') or {}
    if task.get('conditioning_ok') is False:
        reasons.append(f"conditioning {conditioning.get('max_condition_number', float('nan')):.1f}")
    if conditioning.get('twist_status') not in (None, 'pass'):
        reasons.append(f"twist {conditioning['twist_status']}")
    return reasons


def classify(input_check, summary, graph, commands):
    """One failure class per run, from the artifacts the run left."""
    from ur10e_trajectory_pkg import home_pose, pipeline

    if not (input_check or {}).get('passed'):
        return _verdict('input', '; '.join((input_check or {}).get('problems', [])))
    if summary is None:
        return _verdict('pipeline_error', 'no pipeline summary was written')
    # A pass needs no failed stage as well as status 'ok': pipelines before
    # the fix summarised a completed-but-failed commands stage as 'ok'.
    if summary.get('status') == 'ok' and not summary.get('failed_stage'):
        return _verdict('pass')
    stage, status = summary.get('failed_stage'), summary.get('status')

    if stage == pipeline.STAGE_CANDIDATES:
        return _verdict('candidate_discovery', 'candidate generation exited non-zero')

    if stage == pipeline.STAGE_GRAPH:
        if graph is None:
            return _verdict('pipeline_error', 'the graph stage refused before '
                                              'writing a graph')
        if graph.get('complete_path'):
            return _verdict('pipeline_error', 'graph stage failed with a complete path')
        layer = graph.get('first_disconnected_layer')
        kept = candidates_after_filters(graph)
        home = (graph.get('candidate_filters') or {}).get('home_reachable')
        if layer == 0 and home and home.get('layer_0_candidates') and not home.get('kept'):
            return _verdict('warmup_home', f"none of {home['layer_0_candidates']} "
                                           'layer-0 candidates reachable from the home')
        if kept is not None and layer is not None and layer < len(kept):
            if kept[layer] <= 0:
                return _verdict('candidate_discovery',
                                f'layer {layer} has no candidate left after the filters')
            return _verdict('graph_connectivity',
                            f'layer {layer} has {kept[layer]} candidates but no legal '
                            f'transition reaches them')
        return _verdict('graph_connectivity', f'first disconnected layer {layer} '
                                              '(per-layer counts unavailable)')

    if stage == pipeline.STAGE_COMMANDS:
        if status in (pipeline.NO_VALID_WINDING, home_pose.NO_REACHABLE_START,
                      home_pose.HOME_RECHECK_FAILED, home_pose.HOME_MISMATCH):
            return _verdict('warmup_home', status)
        if status == home_pose.NO_CANDIDATES_HERE:
            return _verdict('candidate_discovery', status)
        if commands is None:
            return _verdict('pipeline_error', status or 'no commands artifact')
        refinement = commands.get('refinement') or {}
        if (refinement.get('status') == 'refinement_failed'
                or commands.get('refinement_meets_acceptance') is False):
            return _verdict('refinement', refinement.get('status'))
        warmup = commands.get('warmup_validation')
        if warmup is not None and not warmup.get('passed'):
            return _verdict('warmup_home', 'warmup validation failed')
        task = commands.get('task_validation')
        if task is not None and not task.get('passed'):
            return _verdict('continuous_limit', '; '.join(_continuous_reasons(task))
                            or 'task validation failed')
        return _verdict('pipeline_error', status)

    return _verdict('pipeline_error', status or f'failed at {stage}')
