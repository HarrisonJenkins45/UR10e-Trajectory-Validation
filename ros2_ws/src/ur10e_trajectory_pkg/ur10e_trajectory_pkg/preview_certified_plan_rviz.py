#!/usr/bin/env python3
"""Preview an exported certified task plan and its Target frame in RViz.

This replays the joint coordinates using the same quintic warmup and PCHIP
task interpolation as the validation service. The Target frame is the planned
SISIFOS body frame G in rail_base_link, held still during warmup and following
the planned spin-up and tumble during the task. This is visualization only:
it publishes /joint_states, TF and markers, not robot commands.
"""

import argparse
import csv
import hashlib
import json
import time
from pathlib import Path

import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.spatial.transform import Rotation, Slerp

from ur10e_trajectory_pkg import motion_limits
from ur10e_trajectory_pkg.configurations import JOINT_NAMES
from ur10e_trajectory_pkg.warmup import sample_rest_to_rest


def recording_step(csv_path, plan):
    digest = hashlib.sha256(Path(csv_path).read_bytes()).hexdigest()
    expected = (plan.get("recording") or {}).get("csv_sha256")
    if not expected or digest != expected:
        raise ValueError("recording SHA-256 does not match the certified plan")
    with open(csv_path, newline="", encoding="utf-8") as handle:
        times = np.asarray(
            [float(row["timestamp"]) for row in csv.DictReader(handle)]
        )
    start = int(plan["start_index"])
    count = int(plan["recorded_waypoints"])
    selected = times[start:start + count]
    if len(selected) != count or count < 2:
        raise ValueError("recording does not contain the plan section")
    steps = np.diff(selected)
    if not np.all(np.isfinite(steps)) or np.any(steps <= 0.0):
        raise ValueError("recording timestamps are not strictly increasing")
    if not np.allclose(steps, steps[0], atol=1e-9):
        raise ValueError("recording section is not uniformly sampled")
    return float(steps[0])


def current_warmup_durations(rests, stored_durations):
    """Do not replay an older plan above today's rail speed cap.

    The service replans each warmup leg under its current validator limits.
    An exported plan only records the older durations, so the preview must
    lengthen any leg that would exceed the current rail cap. Other limits
    remain satisfied by lengthening the same rest-to-rest path.
    """
    rail_distance = np.abs(np.diff(rests[:, 0]))
    rail_minimum = (
        15.0 * rail_distance / (8.0 * motion_limits.RAIL_VEL_SAFETY_CAP)
    )
    return np.maximum(stored_durations, rail_minimum)


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
    route = plan.get("warmup_route")
    if route is None:
        raise ValueError("plan has no certified warmup route")
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

    durations = current_warmup_durations(rests, durations)
    warmup = [
        sample_rest_to_rest(rests[i], rests[i + 1], durations[i], rate_hz)[1]
        for i in range(len(durations))
    ]
    times = np.arange(len(path)) * dt
    count = max(2, int(round(times[-1] * rate_hz)) + 1)
    sampled_times = np.linspace(0.0, times[-1], count)
    task = PchipInterpolator(times, path, axis=0)(sampled_times)
    return warmup, task, sampled_times, durations


def target_poses(plan, csv_path, sampled_times):
    """Planned body-frame G poses at task playback times, in rail_base_link.

    Use the same target builder as the IK and service, then apply the mount
    transform recorded in the plan: T_RG = T_RE @ T_EG. Interpolate rotation
    by SLERP, as continuous validation does, not by quaternion components.
    """
    from ur10e_trajectory_pkg.ClientNode import mount_record, plan_targets
    from ur10e_trajectory_pkg.frames import validate_transform

    recorded_mount = plan.get("mount") or {}
    current_mount = mount_record()
    if recorded_mount.get("name") != current_mount["name"]:
        raise ValueError(
            "plan mount name differs from the current target builder"
        )
    mount = validate_transform(recorded_mount.get("transform"), "plan T_EG")
    if not np.allclose(mount, current_mount["transform"], atol=1e-12):
        raise ValueError("plan T_EG differs from the current target builder")

    x, y, z, quaternions, times = plan_targets(plan, csv_path=csv_path)
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
    args = parser.parse_args()
    if not np.isfinite(args.rate_hz) or args.rate_hz <= 0.0:
        parser.error("--rate-hz must be positive and finite")
    with open(args.plan, encoding="utf-8") as handle:
        plan = json.load(handle)
    if plan.get("target_frame") != "rail_base_link":
        raise ValueError("plan target frame is not rail_base_link")
    dt = recording_step(args.csv, plan)
    warmup, task, task_times, warmup_durations = playback_frames(
        plan, dt, args.rate_hz
    )
    recorded_durations = np.asarray(
        plan["warmup_route"]["segment_durations_s"], dtype=float
    )
    if np.any(warmup_durations > recorded_durations + 1e-9):
        print(
            "Preview retimed the archived warmup to the current "
            f"{motion_limits.RAIL_VEL_SAFETY_CAP:.2f} m/s rail cap: "
            f"{warmup_durations.tolist()} s. Revalidate before commanding.",
            flush=True,
        )
    target_positions, target_orientations = target_poses(
        plan, args.csv, task_times
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
                "PreviewPlan_Rviz.py first"
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
