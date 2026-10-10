"""Construct rail-base tool targets from a validated orientation trajectory.

No ROS client or service dependency belongs in this module. The fixed-position
orientation-only task, placement, mount, and spin-up rules live here so every
planner and display consumer uses the same targets.
"""

import numpy as np
from scipy.spatial.transform import Rotation

from ur10e_trajectory_pkg import frames, trajectory_input


# Fixed placement of the trajectory's first pose in the rail-base frame.
#
# These numbers reproduce the legacy target placement exactly. They are a
# fixture, not a calibration: the old code
# reached the same place through a stand-in arm-base pose, an identity
# rotation and a separate hand-tuned +0.5 m offset in Y and Z applied after
# the conversion, which together hid the placement rather than stating it.
LEGACY_PLACEMENT_POSITION_RG = np.array([1.0, 0.5, 0.5])

DEFAULT_CSV_PATH = trajectory_input.DEFAULT_CSV_PATH
DEFAULT_NUM_WAYPOINTS = 500

# T_EG: the pose of the target body frame G (the SISIFOS frame whose attitude
# q_I_G records) expressed in the robot tool frame E (tool0), so that
# p^E = T_EG p^G. The tool is commanded to T_RE(t) = T_RG(t) @ inv(T_EG),
# which keeps the target's motion what the recording says whatever the mount.
# (The robot-side frames module calls the tool pose T_RG; that G is tool0.)
#
# G's origin is the centre of rotation: the point held at the placement while
# the target tumbles about it. It is the geometric centre of the payload's
# main 6 x 6 x 18 in body, which runs from 12 in (the end of the 1 ft boom)
# to 30 in out along tool0's +z: MOUNT_OFFSET_M, 21 in. The flange orbits it
# at that radius. G's axes are tool0's: no rotation. The geometric centre,
# not a measured centre of mass: the wings at the body's top move the real
# one toward them.
#
# PROVISIONAL. The flange-to-mock-up interface is not specified, and the
# absence of a measured transform does not make the rotation a free variable:
# it is pinned here, not optimised, and recorded as provisional in every
# artifact until the hardware interface fixes it. Not named T_GA: SISIFOS
# uses A for the target's centre of mass.
MOUNT_OFFSET_M = 21 * 0.0254
PROVISIONAL_T_EG = frames.make_transform(translation=[0.0, 0.0, MOUNT_OFFSET_M])
MOUNT_TRANSFORM_NAME = 'T_EG'
MOUNT_TRANSFORM_DIRECTION = ('pose of the target body frame G in the robot tool '
                             'frame E (tool0): p^E = T_EG p^G; the tool is '
                             'commanded to T_RE = T_RG @ inv(T_EG)')


def mount_record():
    """The mount transform every target was built with, as artifacts record it."""
    return {'name': MOUNT_TRANSFORM_NAME, 'direction': MOUNT_TRANSFORM_DIRECTION,
            'transform': PROVISIONAL_T_EG.tolist(), 'provisional': True}

# Profile of the spin-up prepended to the recorded motion. The recording starts
# at full tumble rate, so without a spin-up the task begins moving and the arm
# must already be moving to match it. Recorded time is warped as
#
#     tau(t) = t^3 / T^2 - t^4 / (2 T^3)   for 0 <= t <= T
#     tau(t) = t - T / 2                   for t >= T
#
# so the playback rate tau'(t) = 3 s^2 - 2 s^3 (s = t / T) rises from 0 to 1
# with zero rate AND zero rate-of-change at t = 0, and joins the recording
# with rate 1 and zero rate-of-change at t = T. The first pose is the recorded
# first pose, held at rest; every recorded pose is reproduced after T.
SPIN_UP_PROFILE = 'smoothstep playback rate: tau(t) = t^3/T^2 - t^4/(2 T^3)'


def spin_up_recorded_time(times, duration):
    """Recorded time tau(t) for output times t under a spin-up of length duration."""
    times = np.asarray(times, dtype=float)
    T = float(duration)
    ramp = times ** 3 / T ** 2 - times ** 4 / (2.0 * T ** 3)
    return np.where(times <= T, ramp, times - T / 2.0)


def recorded_start_rate(csv_path=DEFAULT_CSV_PATH,
                        num_waypoints=DEFAULT_NUM_WAYPOINTS, start_index=0):
    """The recording's angular rate at its first step, and its sample step.

    The spin-up exists because the recording is already turning at t = 0, so
    playing it directly would command that rate as a step. How long the
    spin-up must be therefore follows from this number, which is why it is
    available BEFORE a duration is chosen rather than only in the record
    apply_spin_up writes afterwards.

    Deliberately the same definition apply_spin_up uses -- the rotation
    between consecutive recorded samples, over the step -- so a rule built on
    this figure and the record written later cannot disagree. For a slice, the
    rate at the slice's own first sample: each slice starts from rest.
    """
    from scipy.spatial.transform import Rotation

    selected = trajectory_input.load(csv_path).slice(start_index, num_waypoints)
    quaternions = selected.orientations_xyzw
    times = selected.times_s
    steps = np.diff(times)
    step = float(steps[0])
    if not np.allclose(steps, step, atol=1e-9):
        raise ValueError('the recording is not uniformly sampled')
    rotations = Rotation.from_quat(quaternions)
    first = float(np.linalg.norm(
        (rotations[1] * rotations[0].inv()).as_rotvec()) / step)
    return {'rate_rad_s': first, 'step_s': step,
            'num_waypoints': int(num_waypoints), 'start_index': int(start_index)}


def apply_spin_up(motion, recorded_times, duration):
    """Resample a relative motion so it starts from rest.

    duration is rounded UP to a multiple of twice the recorded step, so the
    join at T maps onto a recorded sample and only the spin-up itself is
    interpolated (rotation by SLERP, translation linearly). All recorded motion
    is kept; the output is T / 2 longer than the recording.

    Returns (motion, output_times, record).
    """
    from scipy.spatial.transform import Rotation, Slerp

    recorded_times = np.asarray(recorded_times, dtype=float)
    steps = np.diff(recorded_times)
    step = float(steps[0])
    if not np.allclose(steps, step, atol=1e-9):
        raise ValueError('spin-up needs uniformly sampled recorded times')
    half_samples = int(np.ceil(float(duration) / (2.0 * step) - 1e-9))
    T = 2.0 * half_samples * step
    if T <= 0.0:
        raise ValueError('spin-up duration must be positive')

    count = len(recorded_times) + half_samples
    output_times = recorded_times[0] + np.arange(count) * step
    tau = recorded_times[0] + spin_up_recorded_time(output_times - recorded_times[0], T)
    tau = np.clip(tau, recorded_times[0], recorded_times[-1])

    motion = np.asarray(motion, dtype=float)
    rotations = Rotation.from_matrix(motion[:, :3, :3])
    warped = np.tile(np.eye(4), (count, 1, 1))
    warped[:, :3, :3] = Slerp(recorded_times, rotations)(tau).as_matrix()
    for axis in range(3):
        warped[:, axis, 3] = np.interp(tau, recorded_times, motion[:, axis, 3])
    # After the join every output sample IS a recorded sample; copy them so no
    # interpolation round-off touches the reproduced motion.
    warped[2 * half_samples:] = motion[half_samples:]

    rates = np.linalg.norm(
        (rotations[1:] * rotations[:-1].inv()).as_rotvec(), axis=1) / step
    record = {
        'duration_s': T,
        'requested_duration_s': float(duration),
        'profile': SPIN_UP_PROFILE,
        'recorded_time_consumed_s': T / 2.0,
        'samples_added': half_samples,
        'recorded_start_rate_rad_s': float(rates[0]),
        'peak_spin_up_angular_acceleration_rad_s2': float(rates[0] * 1.5 / T),
        'task_starts_at_rest': True,
    }
    return warped, output_times, record


# The recording's attitude is relative to its inertial frame I, and nothing
# ties I to the lab: a fixed rotation of the whole recording reproduces the
# same tumble facing another way. The legacy placement laid I's axes on the
# rail frame's, which pointed the payload at the mounting wall for a third of
# the packaged recording. Instead the boom, G's +z, is turned to stay as
# close to straight out of the wall (+z of the rail frame) as the tumble
# allows: the axis of the narrowest cone holding every recorded boom
# direction becomes the wall's outward normal.
OUTWARD_AXIS_R = np.array([0.0, 0.0, 1.0])
# Candidate cone axes: a Fibonacci sphere, so the choice is deterministic.
CONE_AXIS_CANDIDATES = 20000

_lab_orientations = {}


def _fibonacci_sphere(count):
    k = np.arange(count) + 0.5
    polar = np.arccos(1.0 - 2.0 * k / count)
    azimuth = np.pi * (1.0 + 5.0 ** 0.5) * k
    return np.column_stack((np.cos(azimuth) * np.sin(polar),
                            np.sin(azimuth) * np.sin(polar), np.cos(polar)))


def _rotation_onto(source, target):
    """The smallest rotation taking unit vector source onto unit vector target."""
    axis = np.cross(source, target)
    sine, cosine = np.linalg.norm(axis), float(np.dot(source, target))
    if sine < 1e-12:
        if cosine > 0:
            return np.eye(3)
        # Opposite: half a turn about any axis perpendicular to source.
        helper = np.eye(3)[int(np.argmin(np.abs(source)))]
        axis = np.cross(source, helper)
        return Rotation.from_rotvec(np.pi * axis / np.linalg.norm(axis)).as_matrix()
    return Rotation.from_rotvec(axis / sine * np.arctan2(sine, cosine)).as_matrix()


def lab_orientation(trajectory):
    """R_RI: the rail-frame orientation of the recording's inertial frame.

    From the WHOLE recording, never a slice, so every slice of one recording
    shares it and a slice's targets stay exactly the full recording's targets
    over the same samples. The cone axis d maximises the least d . z_G over
    every recorded boom direction z_G (in I); R_RI is the smallest rotation
    taking d onto the wall's outward normal. Cached per recording digest.
    """
    key = trajectory.sha256
    if key not in _lab_orientations:
        booms = Rotation.from_quat(trajectory.orientations_xyzw).as_matrix()[:, :, 2]
        candidates = _fibonacci_sphere(CONE_AXIS_CANDIDATES)
        least = np.full(len(candidates), np.inf)
        for chunk in np.array_split(booms, max(1, len(booms) // 500)):
            least = np.minimum(least, (candidates @ chunk.T).min(axis=1))
        axis = candidates[int(np.argmax(least))]
        _lab_orientations[key] = _rotation_onto(axis, OUTWARD_AXIS_R)
    return _lab_orientations[key].copy()


def nominal_placement(first_quaternion, lab_from_inertial=None):
    """The start pose T_RG(t0): the fixed position, and the recording turned.

    The rotation is R_RI @ R_IG(t0), so T_RG(0) @ dT(t) gives
    R_RG(t) = R_RI @ R_IG(t): the recorded attitudes, turned by the fixed
    lab orientation R_RI (lab_orientation). With R_RI the identity, the
    default, this is the legacy placement that laid I's axes on the rail's.
    """
    rotation = Rotation.from_quat(first_quaternion).as_matrix()
    if lab_from_inertial is not None:
        rotation = np.asarray(lab_from_inertial, dtype=float) @ rotation
    return frames.make_transform(rotation=rotation,
                                 translation=LEGACY_PLACEMENT_POSITION_RG)


def build_targets(trajectory, num_waypoints=DEFAULT_NUM_WAYPOINTS,
                  return_metadata=False, spin_up_s=None,
                  start_index=0):
    """End-effector targets in the RAIL-BASE frame, per the frame contract.

    Reproduces relative motion at the fixed nominal position:

        dT(t)   = inv(T_IG(t0)) @ T_IG(t)
        T_RG(t) = T_RG(t0) @ dT(t)
        T_RE(t) = T_RG(t) @ inv(T_EG)            the tool, through the mount

    over the slice of num_waypoints recorded samples starting at start_index
    (t0), with times rebased so the slice starts at zero. The nominal
    placement takes its rotation from the slice's first sample, so a slice's
    targets are exactly the full recording's targets over the same samples:
    a path planned on the full recording, restricted to the slice, tracks the
    slice. A spin-up is applied to the slice itself, so every slice starts
    from rest.

    The rail coordinate never appears. It belongs inside forward kinematics,
    and subtracting it here is what made the client and solver disagree.

    The planner, diagnostics, client, and RViz use this one conversion.
    Returns (x, y, z, quaternions, times).

    spin_up_s, when given, prepends a spin-up (apply_spin_up) so the task
    starts from rest; num_waypoints then counts RECORDED samples, and the
    output is longer by the samples the spin-up adds. Off by default, so
    every existing baseline is unchanged.
    """
    selected = trajectory.slice(start_index, num_waypoints)
    quaternions_I = selected.orientations_xyzw

    # Translation from SISIFOS is deliberately outside this orientation-only
    # contract. Relative motion cancels a constant input position, so zero is
    # sufficient and attitude-only recordings need no p_G_I columns.
    positions_I = np.zeros((len(selected.times_s), 3), dtype=np.float64)
    recorded_time = selected.times_s
    sim_time = recorded_time - recorded_time[0]

    poses_I = frames.poses_from_positions_quaternions(positions_I, quaternions_I)
    motion = frames.relative_motion(poses_I)
    spin_up = None
    if spin_up_s is not None:
        motion, sim_time, spin_up = apply_spin_up(motion, sim_time, spin_up_s)

    lab_from_inertial = lab_orientation(trajectory)
    placement_RG = nominal_placement(quaternions_I[0], lab_from_inertial)
    # Orientation-only contract: position is fixed and no SISIFOS translation
    # or spatial scaling participates in the commanded motion.
    poses_RG = frames.place_relative_motion(motion, placement_RG)
    tool_from_target = frames.invert(PROVISIONAL_T_EG)
    poses_RG = np.stack([pose @ tool_from_target for pose in poses_RG])

    positions_RG = poses_RG[:, :3, 3]
    quaternions_RG = np.stack([
        frames.to_position_quaternion(pose)[1] for pose in poses_RG
    ])
    targets = (positions_RG[:, 0], positions_RG[:, 1], positions_RG[:, 2],
               quaternions_RG, sim_time)
    if not return_metadata:
        return targets
    # Everything needed to reproduce these targets from the CSV, for the
    # manifest a diagnostic artifact carries.
    return targets, {
        'csv_path': trajectory.path,
        'num_waypoints': int(num_waypoints),
        'start_index': int(start_index),
        'end_index': int(start_index) + int(num_waypoints),
        'recorded_start_time_s': float(recorded_time[0]),
        'recorded_duration_s': float(recorded_time[-1] - recorded_time[0]),
        'mount': mount_record(),
        'placement_RG': np.asarray(placement_RG).tolist(),
        'lab_orientation_RI': lab_from_inertial.tolist(),
        'target_frame': frames.TARGET_FRAME,
        'spin_up': spin_up,
        'num_samples': int(len(sim_time)),
    }


def build_trajectory_targets(csv_path=DEFAULT_CSV_PATH,
                             num_waypoints=DEFAULT_NUM_WAYPOINTS,
                             return_metadata=False, spin_up_s=None,
                             start_index=0, trajectory=None):
    """Path-based stage adapter for the object-based build_targets interface."""
    if trajectory is None:
        trajectory = trajectory_input.load(csv_path)
    elif trajectory.path != str(csv_path):
        raise ValueError('trajectory object and csv_path name different inputs')
    return build_targets(trajectory, num_waypoints,
                         return_metadata=return_metadata, spin_up_s=spin_up_s,
                         start_index=start_index)
