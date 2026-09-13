import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState  # Standard ROS msg for joint encoders
from ur10e_interfaces.srv import ValidateTrajectory

from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

# Module-level debug toggles (easier to find/flip than buried in __init__)
SKIP_COLLISION = False         # bypass collision checking for debugging
USE_SEGMENT_FINDER = True    # use find_feasible_segments + the 1st viable segment
                               # instead of process_matlab_validation's all-or-nothing
                               # validation, for REAL requests via validation_callback
MIN_SEGMENT_LENGTH = 10       # shortest run of feasible waypoints accepted as a segment
DEFAULT_T_TRAJ = 10.0         # matches process_matlab_validation's own default t_traj;
                               # used to derive dt_waypoint for the segment-finder path,
                               # since incoming requests carry positions but not timing


# Fallback start configuration, used only when a request omits q_start.
# Matches qHome from the MATLAB script: [0, -135, 90, -90, 0, 0] deg for the
# six arm joints, with the rail at its zero (origin) end.
HOME_Q = np.deg2rad(np.array([0.0, 0.0, -135.0, 90.0, -90.0, 0.0, 0.0]))

# Number of actuated joints: the rail plus the six arm joints.
NUM_JOINTS = 7


def resolve_start_pose(q_start_field):
    """Turn a request's q_start field into a start configuration.

    Returns (q_start, description). An empty field means 'use home'; a
    full-length field is taken as given. Any other length raises rather than
    being padded or truncated into something that would quietly validate the
    wrong trajectory.

    Deliberately a pure function of the request. The server used to read its
    start pose from the /joint_states topic it publishes to itself, so each
    request began wherever the previous playback stopped and the same input
    could validate differently depending on what ran before.
    """
    if len(q_start_field) == 0:
        return HOME_Q.copy(), 'home (q_start omitted from request)'
    q_start = np.asarray(q_start_field, dtype=float)
    if q_start.shape != (NUM_JOINTS,):
        raise ValueError(
            f'q_start must have {NUM_JOINTS} elements '
            f'[rail_m, 6x arm_rad], got {q_start.size}'
        )
    return q_start, 'q_start supplied by client'


class TrajectoryValidationNode(Node):

    def __init__(self):
        super().__init__('trajectory_validation_node')

        # NO /joint_states SUBSCRIPTION HERE, deliberately.
        #
        # This node publishes playback frames to /joint_states. Subscribing to
        # the same topic meant it consumed its own output: after a playback the
        # stored pose was that trajectory's final frame, so the next request
        # started from there rather than from home. The same input then
        # validated differently depending on what had run before (measured:
        # 359/500 waypoints cold, 500/500 straight after a previous run). It
        # also picked up frames from any other node publishing joint states.
        #
        # The start pose is now an explicit request field. Closed-loop use
        # against real hardware needs genuine feedback, but that has to arrive
        # on a topic this node does not itself publish to.

        # Publisher to directly drive joint states in visualization/sim
        # Method syntax: create_publisher(msg_type, topic_name, qos_profile)
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10
        )

        # Service Server
        self.srv = self.create_service(
            ValidateTrajectory, 'validate_trajectory', self.validation_callback
        )

        # Playback Timer Attributes
        self.playback_timer = None
        self.playback_frames = None
        self.current_frame_idx = 0
        self.framerate = 30
        self.playbackFrameRate=60

        self.joint_names = [
            'linear_rail_joint',
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        from ament_index_python.packages import get_package_share_directory
        ur_description_share = get_package_share_directory('ur_description')
        self.validator = TrajectoryValidator(
            '/root/ros2_ws/ur10e.urdf',
            mesh_base_path=ur_description_share,
            framerate=self.framerate
        )

        if SKIP_COLLISION:
            # Signature must match check_all_collisions(self, q_full,
            # verbose=False) exactly -- both find_feasible_segments and
            # process_matlab_validation call it with verbose=verbose as a
            # keyword, so a lambda that only accepts q will TypeError the
            # first time collision checking actually runs.
            self.validator.check_all_collisions = lambda q, verbose=False: False
            self.get_logger().info('SKIP_COLLISION=True -- collision checking disabled for this run.')

        self.get_logger().info('Trajectory Validation Service is online and ready.')





    def validation_callback(self, request, response):
        self.get_logger().info('Received trajectory validation request...')

        try:
            q_start, start_desc = resolve_start_pose(request.q_start)
        except ValueError as exc:
            self.get_logger().error(f'Validation failed: {exc}')
            response.success = False
            response.message = f'Error: {exc}'
            response.joint_velocities = []
            return response

        self.get_logger().info(f'Start pose: {start_desc} -> {np.round(q_start, 4).tolist()}')


        ee_x = np.array(request.ee_positions_x)
        ee_y = np.array(request.ee_positions_y)
        ee_z = np.array(request.ee_positions_z)
        # request.ee_quat arrives as a flat float64[] (ROS service fields
        # can't carry a 2D array directly) -- ClientNode.py sends it via
        # q_B_G.flatten().tolist(), so it unflattens back to (N, 4) here,
        # one [x, y, z, w] quaternion per (ee_x, ee_y, ee_z) waypoint.
        ee_quat = np.array(request.ee_quat).reshape(-1, 4)
        simTime=np.array(request.sim_time)

        if USE_SEGMENT_FINDER:
            num_waypts = len(ee_x)
            # process_matlab_validation derives dt_waypoint internally as
            # (t_final - t_transition) / (num_waypts - 1), which simplifies
            # to t_traj / (num_waypts - 1); matched here since incoming
            # requests carry positions only, no explicit timing.
            # dt_waypoint = DEFAULT_T_TRAJ / (num_waypts - 1)
            dt_waypoint= simTime[1]-simTime[0]
            segments = self.validator.find_feasible_segments(
                ee_x, ee_y, ee_z, ee_quat, q_start, min_length=MIN_SEGMENT_LENGTH,
                dt_waypoint=dt_waypoint, verbose=True
            )
            if not segments:
                is_valid = False
                q_dot_matrix, q_interp = np.array([]), np.array([])
                message = (f'No feasible segment of length >= {MIN_SEGMENT_LENGTH} found across '
                           f'{num_waypts} waypoints [start: {start_desc}]')
            else:
                # segment = segments[0]  # 1st viable segment is used as the full trajectory
                segment= max(segments,key=lambda s: s['length'])
                is_valid = True
                q_dot_matrix, q_interp = self.validator.process_feasible_segment(
                    segment, q_start, dt_waypoint, verbose=True
                )
                # message = (f'Using 1st feasible segment [{segment["start_idx"]}, {segment["end_idx"]}] '
                #            f'(length={segment["length"]}/{num_waypts}) as the full trajectory')
                message = (f'Using largest feasible segment [{segment["start_idx"]}, {segment["end_idx"]}] '
                           f'(length={segment["length"]}/{num_waypts}) as the full trajectory '
                           f'[start: {start_desc}]')
        else:
            # Run validation and generate interpolated trajectory frames
            is_valid, q_dot_matrix, q_interp, message = self.validator.process_matlab_validation(
                ee_x, ee_y, ee_z, ee_quat, q_start
            )

        if is_valid:
            response.success = True
            response.message = message if USE_SEGMENT_FINDER else 'Trajectory is valid. Starting streaming playback...'
            response.joint_velocities = q_dot_matrix.flatten().tolist()

            # Start streaming the trajectory over /joint_states
            self.start_trajectory_playback(q_interp)
        else:
            response.success = False
            response.message = f'Validation failed: {message}'
            response.joint_velocities = []

        return response

    def start_trajectory_playback(self, q_interp):
        """Initializes timer to stream trajectory frames sequentially."""
        if self.playback_timer is not None:
            self.playback_timer.cancel()

        self.playback_frames = q_interp
        self.current_frame_idx = 0
        timer_period = 1.0 / self.playbackFrameRate

        self.playback_timer = self.create_timer(timer_period, self.publish_next_frame)
        self.get_logger().info(f'Streaming {len(q_interp)} frames at {self.framerate} Hz...')

    def publish_next_frame(self):
        """Timer callback that publishes one joint state frame at a time."""
        if self.current_frame_idx >= len(self.playback_frames):
            self.get_logger().info('Trajectory playback complete.')
            self.playback_timer.cancel()
            self.playback_timer = None
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.playback_frames[self.current_frame_idx, :].tolist()

        self.joint_state_pub.publish(msg)
        self.current_frame_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryValidationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
