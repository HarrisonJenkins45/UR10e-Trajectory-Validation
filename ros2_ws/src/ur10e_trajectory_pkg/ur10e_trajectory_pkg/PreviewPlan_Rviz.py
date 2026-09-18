"""RViz preview of a certified plan and the SISIFOS Target (G) frame."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = os.path.expanduser("~/ros2_ws/ur10e.urdf")
    with open(urdf_path, encoding="utf-8") as handle:
        robot_description = handle.read()
    rviz_config = os.path.join(
        get_package_share_directory("ur10e_trajectory_pkg"),
        "rviz",
        "target_preview.rviz",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "plan", description="Certified best_plan.json"
            ),
            DeclareLaunchArgument(
                "csv", description="Recording used by the plan"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
            Node(
                package="ur10e_trajectory_pkg",
                executable="preview_certified_plan_rviz",
                arguments=[
                    "--plan",
                    LaunchConfiguration("plan"),
                    "--csv",
                    LaunchConfiguration("csv"),
                    "--loop-full",
                ],
                output="screen",
            ),
        ]
    )
