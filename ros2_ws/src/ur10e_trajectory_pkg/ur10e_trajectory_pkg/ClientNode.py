import numpy as np
import rclpy
import pandas as pd
from rclpy.node import Node
from ur10e_interfaces.srv import ValidateTrajectory
from ur10e_trajectory_pkg import frames
from ur10e_trajectory_pkg.configurations import LEGACY_MATLAB_START_Q
from scipy.spatial.transform import Rotation as R



TEST_VALID_TRAJ=False

# Start configuration sent with every request, as
# [rail_m, shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3].
#
# This is the LEGACY SIMULATION START, not a measured pose. It is where the
# Gazebo bridge parks the simulated arm, so it is correct for simulation runs
# and reproduces the historical baseline. It is not where the physical robot
# begins, and it is singular at the wrist. Replace it with a real measurement
# once VICON or driver feedback exists.
#
# The server no longer defaults to it when q_start is omitted, so the
# assumption is stated here, at the call site that owns it.
Q_START = LEGACY_MATLAB_START_Q.tolist()

class TrajectoryClientNode(Node):

    def __init__(self):
        super().__init__('trajectory_client_node')
        self.cli = self.create_client(ValidateTrajectory, 'validate_trajectory')


        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for validate_trajectory service...')

    def send_request(self, x_pts, y_pts, z_pts, quat, simTime, q_start=None):
        req = ValidateTrajectory.Request()
        req.ee_positions_x = x_pts
        req.ee_positions_y = y_pts
        req.ee_positions_z = z_pts
        # quat is an (N, 4) array of [x, y, z, w] quaternions -- ROS service
        # fields can't carry a 2D array directly, so flatten it the same way
        # ee_positions_x/y/z are already plain 1D lists. Validate_trajServer.py
        # reshapes it back to (N, 4) on the way in.
        req.ee_quat = np.asarray(quat).flatten().tolist()
        req.sim_time=simTime
        # Start pose, required by the service: it rejects an omitted field
        # rather than defaulting, so the assumption lives here at the call
        # site rather than inside the server.
        req.q_start = Q_START if q_start is None else list(q_start)
        # Targets are in the fixed rail-base frame, never the moving
        # carriage frame. Declared so the server can reject a mismatch
        # instead of silently validating the wrong trajectory.
        req.target_frame = frames.TARGET_FRAME

        self.future = self.cli.call_async(req)
        return self.future

    @staticmethod
    def get_end_effector_in_base_frame(p_G_I, q_I_G, p_B_I, q_I_B, TARGET_BOUND_M):
        """Converts target position/orientation from the arena world frame
        (I) to the UR10e base frame (B), with spatial scaling to bound
        trajectory extent. Frame conventions match README section 5:

          R_XY maps Y-frame coordinates into X-frame: v^X = R_XY v^Y.
          r_YX = Y - X, the vector from X to Y.

        So q_I_B, as named, is R_IB (maps B-frame vectors into I-frame).
        To express an I-frame vector/orientation in the B frame we need
        its inverse, R_BI = R_IB^-1 -- applied consistently below for
        both the position and orientation transforms.

        p_G_I / q_I_G here stand in for the target (A/G, see module-level
        note in main()) expressed in the arena world frame I. p_B_I and
        q_I_B are VICON placeholders (identity/offset stand-ins) per the
        README's file-structure notes -- real values arrive once VICON is
        integrated (README "Connecting to Hardware").
        """
        # 1. Spatial scaling
        max_disp = np.max(np.linalg.norm(p_G_I - p_G_I[0], axis=1))
        scale_spatial = TARGET_BOUND_M / max_disp if max_disp > 1e-9 else 1.0

        p_rel_scaled = scale_spatial * (p_G_I - p_G_I[0])
        p_G_I_scaled = p_G_I[0] + p_rel_scaled  # + np.array([0.5,0.5,0.5],dtype=np.float64)



        # 2. Transform scaled target position to base frame.
        # r_I_B == R_IB (maps B -> I); we need R_BI == r_I_B.inv() to go
        # the other way, I -> B. (p_G_I_scaled - p_B_I) is r_{GB}, the
        # vector from B to G, expressed in I-frame components since both
        # operands are given in I-frame coordinates -- .inv().apply(...)
        # re-expresses that same vector in B-frame components.
        r_I_B = R.from_quat(q_I_B[0]) if q_I_B.ndim > 1 else R.from_quat(q_I_B)
        p_G_B = r_I_B.inv().apply(p_G_I_scaled - p_B_I)

        # 3. Orientations relative to base frame: R_BG = R_BI * R_IG.
        r_I_G = R.from_quat(q_I_G)
        r_B_G = r_I_B.inv() * r_I_G
        q_B_G = r_B_G.as_quat()

        return p_G_B, q_B_G




# Placement of the trajectory's FIRST pose in the rail-base frame: where the
# reproduced relative motion is put in the arena. Six free degrees of freedom,
# and the thing a later placement optimisation varies.
#
# These numbers reproduce the legacy pipeline exactly, so the 419-waypoint
# baseline is preserved. They are a fixture, not a calibration: the old code
# reached the same place through a stand-in arm-base pose, an identity
# rotation and a separate hand-tuned +0.5 m offset in Y and Z applied after
# the conversion, which together hid the placement rather than stating it.
LEGACY_PLACEMENT_POSITION_RG = np.array([1.0, 0.5, 0.5])

DEFAULT_CSV_PATH = ('/root/ros2_ws/src/ur10e_trajectory_pkg/'
                    'ur10e_trajectory_pkg/camera_traj.csv')
DEFAULT_NUM_WAYPOINTS = 500

# Fits the relative motion's TRANSLATION inside this radius. Rotation is never
# scaled: the tumble is the thing being reproduced. The current trajectory
# holds position constant, so this is inactive and the factor comes out 1.0.
TARGET_BOUND_M = 1.0


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


def legacy_placement(first_quaternion):
    """The placement the old pipeline implied, stated outright.

    Rotation is taken from the trajectory's first sample so that
    T_RG(0) @ dT(t) reproduces the original absolute orientations, which the
    old code passed through unchanged under an identity calibration.
    """
    return frames.make_transform(rotation=first_quaternion,
                                 translation=LEGACY_PLACEMENT_POSITION_RG)


def build_trajectory_targets(csv_path=DEFAULT_CSV_PATH,
                             num_waypoints=DEFAULT_NUM_WAYPOINTS,
                             placement_RG=None, bound_m=TARGET_BOUND_M,
                             return_metadata=False, spin_up_s=None):
    """End-effector targets in the RAIL-BASE frame, per the frame contract.

    Reproduces relative motion, then places it:

        dT(t)   = inv(T_IG(0)) @ T_IG(t)
        T_RG(t) = T_RG(0) @ dT(t)

    The rail coordinate never appears. It belongs inside forward kinematics,
    and subtracting it here is what made the client and solver disagree.

    Extracted from main() so diagnostics measure the same targets the service
    is sent. Returns (x, y, z, quaternions, times).

    spin_up_s, when given, prepends a spin-up (apply_spin_up) so the task
    starts from rest; num_waypoints then counts RECORDED samples, and the
    output is longer by the samples the spin-up adds. Off by default, so
    every existing baseline is unchanged.
    """
    frame = pd.read_csv(csv_path)
    quaternions_I = frame[['q_I_G_x', 'q_I_G_y', 'q_I_G_z', 'q_I_G_w']].to_numpy(
        dtype=np.float64)[:num_waypoints]

    # Position is held constant: the near-term goal is pure tumbling motion,
    # so only orientation varies along the trajectory.
    position_I = frame[['p_G_I_x', 'p_G_I_y', 'p_G_I_z']].to_numpy(
        dtype=np.float64)[0]
    positions_I = np.tile(position_I, (num_waypoints, 1))
    sim_time = frame['timestamp'].to_numpy(dtype=np.float64)[:num_waypoints]

    poses_I = frames.poses_from_positions_quaternions(positions_I, quaternions_I)
    motion = frames.relative_motion(poses_I)
    spin_up = None
    if spin_up_s is not None:
        motion, sim_time, spin_up = apply_spin_up(motion, sim_time, spin_up_s)

    is_legacy = placement_RG is None
    if is_legacy:
        placement_RG = legacy_placement(quaternions_I[0])
    poses_RG, factor = frames.place_relative_motion(
        motion, placement_RG, bound_m)

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
        'csv_path': str(csv_path),
        'num_waypoints': int(num_waypoints),
        'placement_RG': np.asarray(placement_RG).tolist(),
        'placement_is_legacy_fixture': is_legacy,
        'bound_m': None if bound_m is None else float(bound_m),
        'translation_scale_factor': float(factor),
        'target_frame': frames.TARGET_FRAME,
        'spin_up': spin_up,
        'num_samples': int(len(sim_time)),
    }


def main(args=None):
    rclpy.init(args=args)
    client_node = TrajectoryClientNode()


    # ------------- Start SISFOS Logic ------------#

    x_pts, y_pts, z_pts, q_B_G, simTime = build_trajectory_targets()

    # ------------- End SISIFOS Logic ------------#

    #Override SISFOS request with a know valid trajectory
    if TEST_VALID_TRAJ:
        client_node.get_logger().info('Using a known valid trajectory')

        # --- Simulation Timing Configuration ---
        t_traj = 10.0  # Time to follow the trajectory (seconds)
        tTransition = 2.0  # Time moving from Home to circle start (seconds)
        tFinal = t_traj + tTransition
        radius = 0.80
        omega = 2.0 * np.pi * 0.03
        numWayPts = 100
        # Generate time vector for waypoints
        tWaypoints = np.linspace(tTransition, tFinal, numWayPts)
        t_rel = tWaypoints - tTransition  # Relative time array
        # Vectorized trajectory generation in YZ plane (x = 0.2 m offset)
        x_pts = np.full(numWayPts, 0.2)
        y_pts = 0.0 + radius * np.cos(omega * t_rel)
        z_pts = 0.5 + radius * np.sin(omega * t_rel)
        q_B_G = np.tile(np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64), (numWayPts, 1))
        # simTime must be re-generated at numWayPts length too -- the SISIFOS
        # simTime above is 300 samples; sending it alongside these 100-sample
        # position/quat arrays would send mismatched lengths to the server.
        simTime = tWaypoints

    # Send positions to ROS service
    future = client_node.send_request(
        x_pts.tolist(), y_pts.tolist(), z_pts.tolist(), q_B_G, simTime.tolist()
    )

    # Block until validation server finishes processing
    rclpy.spin_until_future_complete(client_node, future)

    # Handle server response
    response = future.result()
    if response.success:
        client_node.get_logger().info(f'Success: {response.message}')
    else:
        client_node.get_logger().error(f'Failed: {response.message}')

    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()