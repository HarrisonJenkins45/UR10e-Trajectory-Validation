"""RViz display for live service playback or a saved certified plan.

Without plan/csv, the existing validation service may publish /joint_states.
With both arguments, the certified-plan player publishes /joint_states and the
Target frame instead. Do not run both motion sources at once.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def plan_player(context):
    """Add playback only when both required plan inputs are supplied."""
    plan = LaunchConfiguration('plan').perform(context)
    csv = LaunchConfiguration('csv').perform(context)
    if bool(plan) != bool(csv):
        raise ValueError('plan and csv must be supplied together')
    if not plan:
        return []
    return [Node(
        package='ur10e_trajectory_pkg',
        executable='preview_certified_plan_rviz',
        arguments=['--plan', plan, '--csv', csv, '--loop-full'],
        output='screen',
    )]


def generate_launch_description():
    urdf_path = os.path.expanduser('~/ros2_ws/ur10e.urdf')
    with open(urdf_path, encoding='utf-8') as handle:
        robot_description = handle.read()
    rviz_config = os.path.join(
        get_package_share_directory('ur10e_trajectory_pkg'),
        'rviz', 'target_preview.rviz',
    )
    return LaunchDescription([
        DeclareLaunchArgument('plan', default_value='',
                              description='Optional certified best_plan.json'),
        DeclareLaunchArgument('csv', default_value='',
                              description='Recording used to certify the plan'),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}],
             output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', rviz_config], output='screen'),
        Node(package='ur10e_trajectory_pkg', executable='obstacle_markers',
             output='screen'),
        OpaqueFunction(function=plan_player),
    ])
