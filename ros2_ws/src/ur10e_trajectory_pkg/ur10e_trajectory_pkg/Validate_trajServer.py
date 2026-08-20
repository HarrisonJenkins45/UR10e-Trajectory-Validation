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


class TrajectoryValidationNode(Node):

    def __init__(self):
        super().__init__('trajectory_validation_node')

        # In a real-hardware setup, this would stay None until the actual robot
        # publishes its first /joint_states reading. But nothing in this
        # simulate-only setup does that (spawning into Gazebo doesn't publish
        # feedback), and the goal here is just to display the trajectory, not
        # closed-loop control -- so seed a sensible starting point instead of
        # blocking forever on external feedback that will never arrive.
        # Matches qHome from the MATLAB script: [0, -135, 90, -90, 0, 0] deg
        # for the 6 arm joints, rail assumed to start at 0.
        self.current_joint_positions = np.deg2rad(
            np.array([0.0, 0.0, -135.0, 90.0, -90.0, 0.0, 0.0])
        )

        # Subscriber to read current joint states
        # Arguments: Message Type, Topic Name, Callback Function, QoS/Queue Size
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

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





    def joint_state_callback(self, msg):
        joint_map = dict(zip(msg.name, msg.position))
        try:
            self.current_joint_positions = np.array(
                [joint_map[name] for name in self.joint_names]
            )
        except KeyError as e:
            self.get_logger().warn_once(
                f'Joint name mismatch in /joint_states: Missing key {e}'
            )

    def validation_callback(self, request, response):
        self.get_logger().info('Received trajectory validation request...')

        if self.current_joint_positions is None:
            self.get_logger().error('Validation failed: No /joint_states received yet!')
            response.success = False
            response.message = 'Error: Hardware joint states unavailable.'
            response.joint_velocities = []
            return response

        q_start = self.current_joint_positions.copy()
        
        ee_x = np.array(request.ee_positions_x)
        ee_y = np.array(request.ee_positions_y)
        ee_z = np.array(request.ee_positions_z)
        # request.ee_quat arrives as a flat float64[] (ROS service fields
        # can't carry a 2D array directly) -- ClientNode.py sends it via
        # q_B_G.flatten().tolist(), so it unflattens back to (N, 4) here,
        # one [x, y, z, w] quaternion per (ee_x, ee_y, ee_z) waypoint.
        ee_quat = np.array(request.ee_quat).reshape(-1, 4)

        if USE_SEGMENT_FINDER:
            num_waypts = len(ee_x)
            # process_matlab_validation derives dt_waypoint internally as
            # (t_final - t_transition) / (num_waypts - 1), which simplifies
            # to t_traj / (num_waypts - 1); matched here since incoming
            # requests carry positions only, no explicit timing.
            dt_waypoint = DEFAULT_T_TRAJ / (num_waypts - 1)
            segments = self.validator.find_feasible_segments(
                ee_x, ee_y, ee_z, ee_quat, q_start, min_length=MIN_SEGMENT_LENGTH,
                dt_waypoint=dt_waypoint, verbose=True
            )
            if not segments:
                is_valid = False
                q_dot_matrix, q_interp = np.array([]), np.array([])
                message = f'No feasible segment of length >= {MIN_SEGMENT_LENGTH} found across {num_waypts} waypoints'
            else:
                segment = segments[0]  # 1st viable segment is used as the full trajectory
                # segment= max(segments,key=lambda s: s['length'])
                is_valid = True
                q_dot_matrix, q_interp = self.validator.process_feasible_segment(
                    segment, q_start, dt_waypoint, verbose=True
                )
                message = (f'Using 1st feasible segment [{segment["start_idx"]}, {segment["end_idx"]}] '
                           f'(length={segment["length"]}/{num_waypts}) as the full trajectory')
                # message = (f'Using largest feasible segment [{segment["start_idx"]}, {segment["end_idx"]}] '
                #            f'(length={segment["length"]}/{num_waypts}) as the full trajectory')
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
        timer_period = 1.0 / self.framerate

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

