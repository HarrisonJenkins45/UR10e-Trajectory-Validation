#!/usr/bin/env python3
"""Preview an exported certified task plan and its Target frame in RViz.

This replays the joint coordinates using the same quintic warmup and PCHIP
task interpolation as the validation service. The Target frame is the planned
SISIFOS body frame G in rail_base_link, held still during warmup and following
the planned spin-up and tumble during the task. This is visualization only:
it publishes /joint_states, TF and markers, not robot commands.
"""

import argparse
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from ur10e_trajectory_pkg import (
    joint_motion, motion_limits, plan_artifact, trajectory_input,
)
from ur10e_trajectory_pkg.configurations import JOINT_NAMES


def recording_step(csv_path, plan, trajectory=None):
    trajectory = trajectory or trajectory_input.load(csv_path)
    if trajectory.path != str(csv_path):
        raise ValueError("trajectory object and csv_path name different inputs")
    expected = plan["recording"]["csv_sha256"]
    if trajectory.sha256 != expected:
        raise ValueError("recording SHA-256 does not match the certified plan")
    start = int(plan["start_index"])
    count = int(plan["recorded_waypoints"])
    try:
        return trajectory.slice(start, count).step_s
    except ValueError as exc:
        raise ValueError("recording does not contain the plan section") from exc


def check_warmup_durations(rests, stored_durations):
    """Refuse a stored warmup that violates the current rail cap.

    Playback must use the certified durations, not silently retime the plan.
    """
    rail_distance = np.abs(np.diff(rests[:, 0]))
    rail_minimum = (
        15.0 * rail_distance / (8.0 * motion_limits.RAIL_VEL_SAFETY_CAP)
    )
    if np.any(stored_durations + 1e-9 < rail_minimum):
        raise ValueError('stored warmup exceeds the current rail speed cap; '
                         'replan and recertify before playback')


def playback_frames(plan, dt, rate_hz):
    """The service's two joint streams and the task frame times."""
    path = np.asarray(plan["q_path"], dtype=float)
    home = np.asarray(plan["home"], dtype=float)
    if path.ndim != 2 or path.shape[1] != len(JOINT_NAMES) or len(path) < 2:
        raise ValueError("plan q_path must be an (N, 7) array with N >= 2")
    if (
        home.shape != (len(JOINT_NAMES),)
        or not np.all(np.isfinite(home))
        or not np.all(np.isfinite(path))
    ):
        raise ValueError(
            "plan has an invalid home or non-finite joint coordinates"
        )
    route = plan["warmup_route"]
    rests = np.asarray(route["rest_points"], dtype=float)
    durations = np.asarray(route["segment_durations_s"], dtype=float)
    if (
        rests.ndim != 2
        or rests.shape[1] != len(JOINT_NAMES)
        or len(rests) < 2
        or len(durations) != len(rests) - 1
        or not np.all(np.isfinite(rests))
        or not np.all(np.isfinite(durations))
        or np.any(durations <= 0.0)
    ):
        raise ValueError("plan has an invalid warmup route")
    if not np.allclose(rests[0], home, atol=1e-9) or not np.allclose(
        rests[-1], path[0], atol=1e-9
    ):
        raise ValueError("warmup endpoints do not match home and task start")
    expected_count = int(plan["recorded_waypoints"])
    if plan.get("spin_up_s") is not None:
        expected_count += int(
            np.ceil(float(plan["spin_up_s"]) / (2.0 * dt) - 1e-9)
        )
    if len(path) != expected_count:
        raise ValueError(
            "plan path length disagrees with recording and spin-up"
        )

    check_warmup_durations(rests, durations)
    warmup = [
        joint_motion.sample_rest_to_rest(rests[i], rests[i + 1], durations[i], rate_hz)[1]
        for i in range(len(durations))
    ]
    sampled_times, task, _ = joint_motion.task_playback(path, dt, rate_hz)
    return warmup, task, sampled_times, durations


def target_poses(plan, csv_path, sampled_times, trajectory=None):
    """Planned body-frame G poses at task playback times, in rail_base_link.

    Use the same target builder as the IK and service, then apply the mount
    transform recorded in the plan: T_RG = T_RE @ T_EG. Interpolate rotation
    by SLERP, as continuous validation does, not by quaternion components.
    """
    mount = plan_artifact.mount_transform(plan)

    if trajectory is None:
        x, y, z, quaternions, times = plan_artifact.plan_targets(plan, csv_path=csv_path)
    else:
        x, y, z, quaternions, times = plan_artifact.plan_targets(
            plan, csv_path=csv_path, trajectory=trajectory)
    positions_e = np.column_stack((x, y, z))
    rotations_e = Rotation.from_quat(quaternions)
    positions_g = positions_e + rotations_e.apply(mount[:3, 3])
    rotations_g = rotations_e * Rotation.from_matrix(mount[:3, :3])
    times = np.asarray(times, dtype=float)
    sampled_times = np.asarray(sampled_times, dtype=float)
    if (
        len(times) != len(plan["q_path"])
        or len(sampled_times) == 0
        or sampled_times[0] < times[0] - 1e-9
        or sampled_times[-1] > times[-1] + 1e-9
    ):
        raise ValueError("target and joint playback times disagree")
    positions = np.column_stack(
        [
            np.interp(sampled_times, times, positions_g[:, axis])
            for axis in range(3)
        ]
    )
    orientations = Slerp(times, rotations_g)(sampled_times).as_quat()
    return positions, orientations


def replay_passes(task, positions, orientations, loop=False):
    """Keep joint and Target frames paired during display-only reversal."""
    yield task, positions, orientations
    while loop:
        yield task[::-1], positions[::-1], orientations[::-1]
        yield task, positions, orientations


def full_cycle_passes(home, warmup, task, positions, orientations,
                      rate_hz, dwell_s):
    """Display home, warmup, task, then retrace every pose back to home.

    The reverse task is for visualization only. Its positions are continuous
    with the forward task, but its turnaround dynamics are not certified.
    """
    first_position, first_orientation = positions[0], orientations[0]

    def held(frames):
        count = len(frames)
        return (
            np.repeat(first_position[None, :], count, axis=0),
            np.repeat(first_orientation[None, :], count, axis=0),
        )

    home_frames = np.repeat(
        np.asarray(home)[None, :], max(1, int(round(rate_hz))), axis=0
    )
    yield "home", home_frames, *held(home_frames)
    dwell_count = max(0, int(round(dwell_s * rate_hz)))
    for index, frames in enumerate(warmup):
        yield "warmup", frames, *held(frames)
        if index + 1 < len(warmup) and dwell_count:
            dwell = np.repeat(frames[-1][None, :], dwell_count, axis=0)
            yield "via dwell", dwell, *held(dwell)
    yield "task", task, positions, orientations
    yield "task return", task[::-1], positions[::-1], orientations[::-1]
    for index in range(len(warmup) - 1, -1, -1):
        frames = warmup[index][::-1]
        yield "warmup return", frames, *held(frames)
        if index > 0 and dwell_count:
            dwell = np.repeat(frames[-1][None, :], dwell_count, axis=0)
            yield "via dwell", dwell, *held(dwell)


def _non_ros_args(argv):
    """Keep argparse strict while accepting ROS launch's trailing --ros-args."""
    from rclpy.utilities import remove_ros_args
    return remove_ros_args(argv)[1:]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--plan", required=True, help="certified best_plan.json"
    )
    parser.add_argument(
        "--csv", required=True, help="the recording used to certify the plan"
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=30.0,
        help="RViz playback rate; 30 Hz matches the validation service",
    )
    looping = parser.add_mutually_exclusive_group()
    looping.add_argument(
        "--loop",
        action="store_true",
        help="repeat the task forward and backward for display only",
    )
    looping.add_argument(
        "--loop-full",
        action="store_true",
        help="repeat home, warmup and task with a display-only reverse return",
    )
    args = parser.parse_args(_non_ros_args(sys.argv))
    if not np.isfinite(args.rate_hz) or args.rate_hz <= 0.0:
        parser.error("--rate-hz must be positive and finite")
    plan = plan_artifact.load_plan(args.plan)
    trajectory = trajectory_input.load(args.csv)
    dt = recording_step(args.csv, plan, trajectory=trajectory)
    warmup, task, task_times, _ = playback_frames(
        plan, dt, args.rate_hz
    )
    target_positions, target_orientations = target_poses(
        plan, args.csv, task_times, trajectory=trajectory
    )

    import rclpy
    from geometry_msgs.msg import Point, TransformStamped
    from sensor_msgs.msg import JointState
    from tf2_ros import TransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init()
    node = rclpy.create_node("certified_plan_rviz_preview")
    joint_publisher = node.create_publisher(JointState, "/joint_states", 10)
    marker_publisher = node.create_publisher(
        MarkerArray, "/target_frame_markers", 10
    )
    tf_broadcaster = TransformBroadcaster(node)

    def markers(stamp):
        array = MarkerArray()
        for axis, color in enumerate(
            ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.3, 1.0))
        ):
            marker = Marker()
            marker.header.frame_id = "Target"
            marker.header.stamp = stamp
            marker.ns = "Target_axes"
            marker.id = axis
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            end = [0.0, 0.0, 0.0]
            end[axis] = 0.25
            marker.points = [
                Point(x=0.0, y=0.0, z=0.0),
                Point(x=end[0], y=end[1], z=end[2]),
            ]
            marker.scale.x, marker.scale.y, marker.scale.z = 0.012, 0.035, 0.05
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            marker.frame_locked = True
            array.markers.append(marker)
        label = Marker()
        label.header.frame_id = "Target"
        label.header.stamp = stamp
        label.ns = "Target_label"
        label.id = 3
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.z = 0.33
        label.pose.orientation.w = 1.0
        label.scale.z = 0.09
        label.color.r, label.color.g, label.color.b, label.color.a = (
            1.0,
            1.0,
            0.5,
            1.0,
        )
        label.text = "Target (G)"
        label.frame_locked = True
        array.markers.append(label)
        return array

    def publish(configuration, position, orientation):
        stamp = node.get_clock().now().to_msg()
        message = JointState()
        message.header.stamp = stamp
        message.name = list(JOINT_NAMES)
        message.position = np.asarray(configuration, dtype=float).tolist()
        joint_publisher.publish(message)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = plan["target_frame"]
        transform.child_frame_id = "Target"
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation.x = float(orientation[0])
        transform.transform.rotation.y = float(orientation[1])
        transform.transform.rotation.z = float(orientation[2])
        transform.transform.rotation.w = float(orientation[3])
        tf_broadcaster.sendTransform(transform)
        marker_publisher.publish(markers(stamp))

    def stream(frames, positions, orientations):
        for frame, position, orientation in zip(
            frames, positions, orientations
        ):
            if not rclpy.ok():
                break
            publish(frame, position, orientation)
            # Never publish catch-up bursts after a slow frame: a burst could
            # make a planned 0.5 m/s rail step appear much faster in RViz.
            time.sleep(1.0 / args.rate_hz)

    def held_target(count, position, orientation):
        return (
            np.repeat(position[None, :], count, axis=0),
            np.repeat(orientation[None, :], count, axis=0),
        )

    try:
        print("Waiting for robot_state_publisher...", flush=True)
        until = time.monotonic() + 10.0
        while (
            joint_publisher.get_subscription_count() == 0
            or marker_publisher.get_subscription_count() == 0
        ) and time.monotonic() < until:
            time.sleep(0.1)
        if (
            joint_publisher.get_subscription_count() == 0
            or marker_publisher.get_subscription_count() == 0
        ):
            raise RuntimeError(
                "robot or Target marker subscriber missing; start RViz with "
                "VisualizeTraj_RvizPlayback.py first"
            )
        first_position, first_orientation = (
            target_positions[0],
            target_orientations[0],
        )
        if args.loop_full:
            print(
                "Full display loop: home -> warmup -> task -> reverse task "
                "-> reverse warmup -> home. Return is not certified.",
                flush=True,
            )
            while rclpy.ok():
                cycle = full_cycle_passes(
                    plan["home"], warmup, task, target_positions,
                    target_orientations, args.rate_hz,
                    plan["warmup_route"]["dwell_s"],
                )
                for phase, frames, positions, orientations in cycle:
                    if not rclpy.ok():
                        break
                    print(f"RViz phase: {phase}", flush=True)
                    stream(frames, positions, orientations)
            return
        home_frames = np.repeat(
            np.asarray(plan["home"])[None, :], int(args.rate_hz), axis=0
        )
        stream(
            home_frames,
            *held_target(len(home_frames), first_position, first_orientation),
        )
        for number, frames in enumerate(warmup):
            print(f"Warmup leg {number + 1}/{len(warmup)}", flush=True)
            stream(
                frames,
                *held_target(len(frames), first_position, first_orientation),
            )
            if number + 1 < len(warmup):
                dwell = np.repeat(
                    frames[-1][None, :],
                    int(round(plan["warmup_route"]["dwell_s"] * args.rate_hz)),
                    axis=0,
                )
                stream(
                    dwell,
                    *held_target(
                        len(dwell), first_position, first_orientation
                    ),
                )
        task_duration = (len(task) - 1) / args.rate_hz
        print(
            f"Tumbling section: {len(task)} frames over {task_duration:.1f} s",
            flush=True,
        )
        time.sleep(1.0)
        if args.loop:
            print(
                "Display loop active: reverse playback is not a certified "
                "command.",
                flush=True,
            )
        for frames, positions, orientations in replay_passes(
            task, target_positions, target_orientations, args.loop
        ):
            if not rclpy.ok():
                break
            stream(frames, positions, orientations)
        if not args.loop:
            hold_count = int(3.0 * args.rate_hz)
            last = np.repeat(task[-1][None, :], hold_count, axis=0)
            stream(
                last,
                *held_target(
                    len(last), target_positions[-1], target_orientations[-1]
                ),
            )
        print("RViz playback complete.", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
