"""The one RViz launch supports live playback and certified-plan preview."""

from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from ur10e_trajectory_pkg import VisualizeTraj_RvizPlayback as rviz_launch


def _context(plan='', csv=''):
    context = LaunchContext()
    context.launch_configurations['plan'] = plan
    context.launch_configurations['csv'] = csv
    return context


def test_live_mode_has_no_competing_joint_state_publisher():
    assert rviz_launch.plan_player(_context()) == []


def test_plan_mode_requires_both_inputs_and_adds_one_player():
    with pytest.raises(ValueError, match='plan and csv'):
        rviz_launch.plan_player(_context(plan='best_plan.json'))
    with pytest.raises(ValueError, match='plan and csv'):
        rviz_launch.plan_player(_context(csv='camera_traj.csv'))
    assert len(rviz_launch.plan_player(
        _context('best_plan.json', 'camera_traj.csv'))) == 1


def test_one_launch_keeps_obstacles_and_target_display():
    description = rviz_launch.generate_launch_description()
    assert sum(isinstance(entity, Node) for entity in description.entities) == 3
    assert sum(isinstance(entity, OpaqueFunction)
               for entity in description.entities) == 1
    config = (Path(__file__).resolve().parents[1] / 'rviz' /
              'target_preview.rviz').read_text(encoding='utf-8')
    assert '/obstacle_markers' in config
    assert '/target_frame_markers' in config
    assert 'Class: rviz_default_plugins/MoveCamera' in config
