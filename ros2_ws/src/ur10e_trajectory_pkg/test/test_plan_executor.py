"""plan_executor stage construction: warmup and task at their own time scales.

Uses the rig's committed plan (plans/wallmount_20260929) and the packaged
recording it was certified on.
"""

import contextlib
import time
from pathlib import Path

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from ur10e_trajectory_pkg import plan_executor  # noqa: E402

WORKSPACE = Path(__file__).resolve().parents[3]
PLAN = WORKSPACE / 'plans' / 'wallmount_20260929' / 'best_plan.json'
CSV = WORKSPACE / 'src' / 'ur10e_trajectory_pkg' / 'ur10e_trajectory_pkg' / 'camera_traj.csv'
URDF = WORKSPACE / 'ur10e.urdf'

pytestmark = pytest.mark.skipif(not (PLAN.exists() and CSV.exists()),
                                reason='committed plan or packaged recording missing')


@contextlib.contextmanager
def executor_at(where, **params):
    params = {'plan': str(PLAN), 'csv': str(CSV), 'urdf': str(URDF), 'mode': 'full',
              'check_moves': False, **params}
    args = ['--ros-args']
    for name, value in params.items():
        args += ['-p', f'{name}:={value}']
    rclpy.init(args=args)
    node = plan_executor.PlanExecutor()
    try:
        pose = {'home': node.home, 'task start': node.task_start,
                'task end': np.asarray(node.plan['q_path'][-1], dtype=float)}[where]
        node.measurement = (pose.copy(), np.zeros(len(pose)), time.monotonic())
        node.device_status = {'arm': 'ready', 'rail': 'ready'}
        node.rail_homed = True
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def reference_stages(node, stages):
    """Build every pending stage in order, as next_stage would."""
    built, pending = [stages[0]], list(stages[1:])
    while pending:
        stage, rest = node.split(pending.pop(0)(node.measurement[0]))
        pending = rest + pending
        built.append(stage)
    return built


def test_from_home_the_warmup_and_the_task_run_at_their_own_scales():
    with executor_at('home', time_scale=0.1, task_time_scale=1.0) as node:
        stages, refusal = node.build_sequence(include_plan=True)
        assert refusal is None
        warmup, task = reference_stages(node, stages)
        assert 'warmup' in warmup.label and warmup.reference.time_scale == 0.1
        assert task.label == 'task' and task.reference.time_scale == 1.0
        # The section is 60 s of recording plus a 0.4 s spin-up; the task
        # stage adds a 1 s settle.
        assert task.reference.duration == pytest.approx(62.2, abs=1.5)
        np.testing.assert_allclose(warmup.reference.end, task.reference.start, atol=1e-9)


def test_from_the_task_start_only_the_task_runs():
    with executor_at('task start', time_scale=0.1, task_time_scale=1.0) as node:
        stages, refusal = node.build_sequence(include_plan=True)
        assert refusal is None and len(stages) == 1
        assert stages[0].reference.time_scale == 1.0
        assert 'warmup already done' in stages[0].label


def test_a_warmup_too_fast_is_refused_but_does_not_block_the_task_alone():
    with executor_at('home', time_scale=1.0) as node:
        stages, refusal = node.build_sequence(include_plan=True)
        assert stages is None and refusal.startswith('warmup: speed limits exceeded')
    with executor_at('task start', time_scale=1.0) as node:
        stages, refusal = node.build_sequence(include_plan=True)
        assert refusal is None and stages[0].reference.time_scale == 1.0


def test_task_time_scale_defaults_to_time_scale():
    with executor_at('home', time_scale=0.1) as node:
        assert node.task_time_scale == 0.1
        warmup, task = reference_stages(node, node.build_sequence(include_plan=True)[0])
        assert task.reference.time_scale == 0.1


def test_warmup_mode_runs_no_task():
    with executor_at('home', mode='warmup', task_time_scale=1.0) as node:
        stages, refusal = node.build_sequence(include_plan=True)
        assert refusal is None
        assert [s.label for s in reference_stages(node, stages)] == [
            'at plan home, then the warmup']


def test_from_the_task_end_a_checked_move_returns_the_arm_home_then_the_plan_runs():
    with executor_at('task end', check_moves=True, task_time_scale=1.0) as node:
        assert node.validator is not None
        stages, refusal = node.build_sequence(include_plan=True)
        assert refusal is None, refusal
        first, task = reference_stages(node, stages)
        names = [segment.name for segment in first.reference.segments]
        assert names[0] == 'straighten arm' and 'warmup 1/1' in names
        np.testing.assert_allclose(first.reference.start, node.measurement[0], atol=1e-9)
        assert task.reference.time_scale == 1.0


def test_a_blocked_return_is_refused_with_the_reason(monkeypatch):
    from ur10e_trajectory_pkg import warmup
    with executor_at('task end', check_moves=True) as node:
        monkeypatch.setattr(warmup, 'plan_warmup', lambda *args, **kwargs: {
            'status': warmup.NO_DIRECT_WARMUP, 'reason': 'the straight move collides'})
        stages, refusal = node.build_sequence(include_plan=True)
        assert stages is None
        assert 'no_direct_warmup' in refusal and 'from the pendant' in refusal


def test_without_a_collision_model_a_far_arm_is_left_to_the_pendant():
    with executor_at('task end', check_moves=False) as node:
        stages, refusal = node.build_sequence(include_plan=True)
        assert stages is None and 'no collision model loaded' in refusal


def test_an_unhomed_rail_is_checked_at_plan_home_then_homed_before_the_approach():
    with executor_at('task end', check_moves=True) as node:
        node.rail_homed = False
        node.measurement[0][0] = -27.55            # meaningless until homed
        stages, refusal = node.build_sequence(include_plan=True)
        assert refusal is None, refusal
        assert stages[0].label == 'straighten arm'
        assert isinstance(stages[1](node.measurement[0]), plan_executor.HomeRailStage)
