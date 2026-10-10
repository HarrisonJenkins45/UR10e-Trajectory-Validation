"""ROS-free contract for an exported warmup and task plan.

This module checks the artifact's structure and source recording. It does not
certify motion: only the planner's independent validation can do that.
"""

import json

import numpy as np

from ur10e_trajectory_pkg import frames, target_builder, trajectory_input


SCHEMA_VERSION = 1
PLAN_KEYS = ('schema_version', 'home', 'warmup_route', 'q_path', 'task_start',
             'recorded_waypoints', 'start_index', 'spin_up_s', 'placement',
             'mount', 'target_frame', 'recording')


def route_record(route):
    """Store warmup rest points and durations, not unverified motion samples."""
    return {'route_kind': route.get('route_kind'),
            'rest_points': route['rest_points'],
            'dwell_s': route['dwell_s'],
            'segment_durations_s': [leg['duration_s'] for leg in route['segments']],
            'total_duration_s': route['total_duration_s']}


def task_plan(home, lifted_path, graph, graph_path, winding, route, recording):
    """Export the validated path, warmup, and inputs needed to rebuild targets.

    The caller must have independently certified both motions before writing
    this record. Serialization itself is not a motion-validity check.
    """
    if route is None:
        raise ValueError('a certified warmup route is required to export a plan')
    if not recording or not recording.get('csv_sha256'):
        raise ValueError('the source recording digest is required to export a plan')
    if not graph.get('mount'):
        raise ValueError('the tool mount is required to export a plan')
    spin_up = graph.get('spin_up')
    lifted_path = np.asarray(lifted_path, dtype=float)
    return {
        'schema_version': SCHEMA_VERSION,
        'home': np.asarray(home, dtype=float).tolist(),
        'warmup_route': route_record(route),
        'q_path': lifted_path.tolist(),
        'task_start': lifted_path[0].tolist(),
        'start_winding': list(winding),
        'recorded_waypoints': graph['recorded_waypoints'],
        'start_index': int(graph.get('start_index', 0)),
        'spin_up_s': None if spin_up is None else spin_up['requested_duration_s'],
        'placement': graph.get('placement', 'nominal'),
        'mount': graph['mount'],
        'target_frame': frames.TARGET_FRAME,
        'source_graph': graph_path,
        'recording': recording,
    }


def validate_plan(plan):
    """Return an array-backed plan after checking the execution-facing shape."""
    if not isinstance(plan, dict):
        raise ValueError('plan must be a JSON object')
    missing = [key for key in PLAN_KEYS if key not in plan]
    if missing:
        raise ValueError(f'plan is missing {missing}')
    if plan['schema_version'] != SCHEMA_VERSION:
        raise ValueError('unsupported plan schema_version')
    count = plan['recorded_waypoints']
    if isinstance(count, bool) or not isinstance(count, int) or count < 2:
        raise ValueError('plan recorded_waypoints must be an integer >= 2')
    start_index = plan['start_index']
    if isinstance(start_index, bool) or not isinstance(start_index, int) or start_index < 0:
        raise ValueError('plan start_index must be a nonnegative integer')
    spin_up = plan['spin_up_s']
    if spin_up is not None and (not isinstance(spin_up, (int, float)) or
                                not np.isfinite(spin_up) or spin_up <= 0):
        raise ValueError('plan spin_up_s must be positive and finite')
    q_path = np.asarray(plan['q_path'], dtype=float)
    if q_path.ndim != 2 or q_path.shape[1] != 7 or len(q_path) < 2:
        raise ValueError(f'plan q_path has shape {q_path.shape}, expected (N, 7)')
    home = np.asarray(plan['home'], dtype=float)
    if home.shape != (7,):
        raise ValueError(f'plan home has {home.size} values, expected 7')
    task_start = np.asarray(plan['task_start'], dtype=float)
    if task_start.shape != (7,) or not all(np.all(np.isfinite(array)) for array in
                                        (home, q_path, task_start)):
        raise ValueError('plan has invalid or non-finite joint coordinates')
    if not np.allclose(task_start, q_path[0], atol=1e-12, rtol=0):
        raise ValueError('plan task_start is not the first configuration of q_path')
    if plan['target_frame'] != frames.TARGET_FRAME:
        raise ValueError(f"plan targets are in {plan['target_frame']!r}, "
                         f'not {frames.TARGET_FRAME!r}')
    recorded = plan['recording']
    if (not isinstance(recorded, dict) or not recorded.get('csv_path') or
            not isinstance(recorded.get('csv_sha256'), str) or
            len(recorded['csv_sha256']) != 64 or
            any(char not in '0123456789abcdef' for char in recorded['csv_sha256'])):
        raise ValueError('plan recording requires a path and SHA-256 digest')
    mount_transform(plan)
    resolved = dict(plan, q_path=q_path, home=home, task_start=q_path[0])
    route = plan['warmup_route']
    if not isinstance(route, dict) or not {'rest_points', 'segment_durations_s', 'dwell_s'} <= route.keys():
        raise ValueError('plan warmup_route is missing rest points, durations, or dwell')
    points = np.asarray(route['rest_points'], dtype=float)
    if points.ndim != 2 or points.shape[1] != 7 or len(points) < 2:
        raise ValueError(f'plan warmup_route rest_points has shape '
                         f'{points.shape}, expected (N, 7)')
    if not np.all(np.isfinite(points)):
        raise ValueError('plan warmup_route has non-finite rest points')
    if not np.allclose(points[0], home, atol=1e-9, rtol=0):
        raise ValueError('plan warmup_route does not start at the home')
    if not np.allclose(points[-1], q_path[0], atol=1e-9, rtol=0):
        raise ValueError('plan warmup_route does not end at the task start')
    durations = np.asarray(route['segment_durations_s'], dtype=float)
    if (durations.shape != (len(points) - 1,) or
            not np.all(np.isfinite(durations)) or np.any(durations <= 0.0)):
        raise ValueError('plan warmup_route has invalid segment durations')
    dwell = route['dwell_s']
    if not isinstance(dwell, (int, float)) or not np.isfinite(dwell) or dwell < 0:
        raise ValueError('plan warmup_route has invalid dwell_s')
    resolved['warmup_route'] = dict(route, rest_points=points)
    return resolved


def load_plan(path):
    """Read the plan artifact without requiring ROS or a service client."""
    with open(path, encoding='utf-8') as handle:
        return validate_plan(json.load(handle))


def mount_transform(plan):
    """Require the recorded tool mount to match the current target builder."""
    recorded = plan.get('mount')
    if not isinstance(recorded, dict):
        raise ValueError('plan has no recorded tool mount')
    current = target_builder.mount_record()
    if recorded.get('name') != current['name']:
        raise ValueError('plan mount name differs from the current target builder')
    mount = frames.validate_transform(recorded.get('transform'), 'plan T_EG')
    if not np.allclose(mount, current['transform'], atol=1e-12, rtol=0):
        raise ValueError('plan T_EG differs from the current target builder')
    return mount


def plan_targets(plan, csv_path=None, trajectory=None):
    """Rebuild exactly the targets this plan was validated against.

    A relocated recording is accepted only when its content digest matches.
    """
    from ur10e_trajectory_pkg.planning_runtime import recording_mismatch

    plan = validate_plan(plan)
    recorded = plan['recording']
    csv_path = csv_path or (trajectory.path if trajectory is not None else None) \
        or recorded['csv_path']
    if trajectory is None:
        trajectory = trajectory_input.load(csv_path)
    elif trajectory.path != str(csv_path):
        raise ValueError('trajectory object and csv_path name different inputs')
    current = {'csv_path': str(csv_path), 'csv_sha256': trajectory.sha256}
    mismatch = recording_mismatch(recorded, current)
    if mismatch:
        raise ValueError(f'the plan was {mismatch}')
    start_index = plan['start_index']
    if plan['placement'] != 'nominal':
        raise ValueError('only the fixed nominal target placement is supported')
    x, y, z, quaternions, times = target_builder.build_targets(
        trajectory, plan['recorded_waypoints'],
        spin_up_s=plan['spin_up_s'], start_index=start_index)
    if len(x) != len(plan['q_path']):
        raise ValueError(f"the plan's q_path has {len(plan['q_path'])} "
                         f'configurations but its targets number {len(x)}')
    return x, y, z, quaternions, times
