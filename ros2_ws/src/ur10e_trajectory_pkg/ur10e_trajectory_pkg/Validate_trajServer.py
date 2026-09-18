import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState  # Standard ROS msg for joint encoders
from ur10e_interfaces.srv import ExecuteWarmup, ValidateTrajectory

from ur10e_trajectory_pkg import frames
from ur10e_trajectory_pkg.configurations import JOINT_NAMES, NUM_JOINTS
from ur10e_trajectory_pkg import warmup
from ur10e_trajectory_pkg.motion_limits import limit_statuses
from ur10e_trajectory_pkg.continuous_validator import (
    validate_task_command,
)
from ur10e_trajectory_pkg.task_path import verify_task_path
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

# Module-level debug toggle (easier to find/flip than buried in __init__)
SKIP_COLLISION = False         # bypass collision checking for debugging

# Rate at which the service validates the motion BETWEEN waypoints before
# playing: the same continuous checks run offline, on the exact path received.
# 200 Hz matches the offline command-2 validation (about 11 s end to end).
SERVICE_VALIDATION_HZ = 200.0

# How far the MEASURED start may sit from the first solved configuration. The
# task plays from that configuration with no transition, so a larger offset
# would be a step command at t = 0. Provisional: set from nothing measured
# yet, to be replaced by encoder and controller tracking data.
START_RAIL_TOL_M = 0.005
START_ARM_TOL_RAD = 0.01


def resolve_target_frame(target_frame_field):
    """Reject targets that are not in the fixed rail-base frame.

    Validated rather than trusted. The rail base does not move; the UR
    base_link rides the carriage, so targets expressed against it disagree
    with the solver by the rail position plus the carriage mount height. That
    mismatch produces confident, wrong answers with nothing to notice it by.
    """
    if not target_frame_field:
        raise ValueError(
            f'target_frame is required and must be "{frames.TARGET_FRAME}"'
        )
    if target_frame_field != frames.TARGET_FRAME:
        raise ValueError(
            f'targets must be expressed in "{frames.TARGET_FRAME}", got '
            f'"{target_frame_field}". The carriage frame moves with the rail '
            'and cannot be the frame a target is expressed in'
        )
    return target_frame_field


def resolve_start_pose(q_start_field):
    """Turn a request's q_start field into a start configuration.

    The start pose is required. It used to default to a legacy MATLAB posture
    when omitted, which meant a caller that simply forgot got a confident
    answer computed from a configuration the robot was not in. That posture is
    also singular, so the default quietly seeded the solver on a degeneracy.

    Simulation and regression callers that genuinely want it must pass
    configurations.LEGACY_MATLAB_START_Q explicitly, so the assumption appears
    at the call site.
    """
    if len(q_start_field) == 0:
        raise ValueError(
            'q_start is required: send the configuration the arm is actually '
            f'in, as {NUM_JOINTS} values [rail_m, 6x arm_rad]. For the legacy '
            'simulation start, pass configurations.LEGACY_MATLAB_START_Q '
            'explicitly rather than relying on a default'
        )
    q_start = np.asarray(q_start_field, dtype=float)
    if q_start.shape != (NUM_JOINTS,):
        raise ValueError(
            f'q_start must have {NUM_JOINTS} elements '
            f'[rail_m, 6x arm_rad], got {q_start.size}'
        )
    return q_start, 'q_start supplied by client'


def resolve_task_path(q_path_field, num_waypoints):
    """The request's joint path as an (N, 7) array, or a reason it is unusable.

    Required: the service plays the path the planner produced and validation
    checked, and no longer re-solves, since a re-solve could follow a
    different branch.
    """
    if len(q_path_field) == 0:
        raise ValueError(
            'q_path is required: send the validated joint path, flattened '
            f'[N x {NUM_JOINTS}], one configuration per target. The service no '
            'longer re-solves the trajectory, because a re-solve can follow a '
            'different branch from the one that was validated')
    q_path = np.asarray(q_path_field, dtype=float)
    if q_path.size != num_waypoints * NUM_JOINTS:
        raise ValueError(
            f'q_path has {q_path.size} values; {num_waypoints} targets need '
            f'{num_waypoints * NUM_JOINTS} ({NUM_JOINTS} per target)')
    return q_path.reshape(num_waypoints, NUM_JOINTS)


def resolve_configuration(field, name):
    """A request's 7-value configuration field, or a reason it is unusable."""
    if len(field) != NUM_JOINTS:
        raise ValueError(f'{name} must have {NUM_JOINTS} values '
                         f'[rail_m, 6x arm_rad], got {len(field)}')
    return np.asarray(field, dtype=float)


def resolve_rest_points(field, q_start, q_target):
    """The route's resting points, or the straight move when none was sent."""
    if len(field) == 0:
        return [np.asarray(q_start, dtype=float), np.asarray(q_target, dtype=float)]
    if len(field) % NUM_JOINTS or len(field) < 2 * NUM_JOINTS:
        raise ValueError(f'rest_points must be a flattened [N x {NUM_JOINTS}] with '
                         f'N >= 2, got {len(field)} values')
    points = np.asarray(field, dtype=float).reshape(-1, NUM_JOINTS)
    if not np.allclose(points[-1], np.asarray(q_target, dtype=float), atol=1e-9):
        raise ValueError('the route\'s last resting point is not q_target')
    return [row for row in points]


def check_measured_rest_point(q_measured, rest_point, label):
    """Whether the arm is standing at a planned resting point, joints only.

    The tolerances the task start uses, and no forward-kinematics check: an
    intermediate pose is somewhere to stand still, not a pose the task was
    solved for, so there is no target to miss.
    """
    q_measured = np.asarray(q_measured, dtype=float)
    planned = np.asarray(rest_point, dtype=float)
    rail_offset = abs(q_measured[0] - planned[0])
    arm_offset = float(np.max(np.abs(q_measured[1:] - planned[1:])))
    failures = []
    if rail_offset > START_RAIL_TOL_M:
        failures.append(f'rail {rail_offset:.4f} m from {label} '
                        f'(tolerance {START_RAIL_TOL_M} m)')
    if arm_offset > START_ARM_TOL_RAD:
        failures.append(f'arm joint {arm_offset:.4f} rad from {label} '
                        f'(tolerance {START_ARM_TOL_RAD} rad)')
    return not failures, {'rail_offset_m': float(rail_offset),
                          'arm_offset_rad': arm_offset, 'failures': failures}


def plan_and_validate_route(validator, rest_points, segment_index, q_measured,
                            playback_hz):
    """Command 1, one leg per call: plan the whole route, play one leg of it.

    The whole route is planned and validated on every call, so a route whose
    second leg is blocked is refused before the first is played and the arm
    is not left standing at a via pose it cannot leave.

    The measured configuration is checked against the resting point this leg
    starts from. At a dwell that check is the only evidence the previous leg
    arrived where it planned: this node does not subscribe to /joint_states,
    so comparing against its own published frames would be the node checking
    its own output.

    Returns (ok, message, frames, route).
    """
    from ur10e_trajectory_pkg.continuous_validator import validate_route

    legs = len(rest_points) - 1
    # The straight move is the common case and reads as a warmup, not as
    # "segment 1 of 1": the route wording appears only when there is a route.
    many = legs > 1
    route = warmup.route_from_rest_points(validator, rest_points)
    if route['status'] != warmup.OK:
        failed = route.get('failed_segment')
        where = ('' if failed is None or not many
                 else f'segment {failed + 1} of {legs}: ')
        return (False, f"No warmup{' route' if many else ''}: {where}"
                       f"{route['status']} ({route.get('reason')})", None, route)
    report = validate_route(validator, route)
    if not report['passed']:
        return (False, f"Warmup{' route' if many else ''} failed validation: "
                       + '; '.join(report['failures']), None, route)
    if not 0 <= segment_index < legs:
        return (False, f'segment_index {segment_index} is outside a route of '
                       f'{legs} segments', None, route)
    segment = route['segments'][segment_index]
    label = ('the warmup start' if segment_index == 0
             else f'the dwell at resting point {segment_index + 1}')
    ok, details = check_measured_rest_point(q_measured, segment['begins_at'], label)
    if not ok:
        return (False, f'Arm is not at {label}: ' + '; '.join(details['failures']),
                None, route)
    _, frames, _, _ = warmup.sample_rest_to_rest(
        np.asarray(segment['begins_at'], dtype=float),
        np.asarray(segment['ends_at'], dtype=float),
        segment['duration_s'], playback_hz)
    binding = segment['binding']
    label = (f'Warmup segment {segment_index + 1} of {legs} validated'
             if many else 'Warmup validated')
    return (True,
            f"{label}: {segment['duration_s']:.3f} s rest to rest, bound by "
            f"{binding['joint']} {binding['kind']} ({binding['status']})",
            frames, route)


def plan_and_validate_warmup(validator, q_start, q_target, playback_hz):
    """Command 1 for a straight move: the one-leg case of a route.

    Returns (ok, message, frames, plan), the plan being that single leg.
    """
    ok, message, frames, route = plan_and_validate_route(
        validator, [q_start, q_target], 0, q_start, playback_hz)
    segments = route.get('segments') or []
    return ok, message, frames, (segments[0] if segments else route)


def continuous_failures(report, statuses=None):
    """Human-readable reasons a continuous validation report did not pass.

    statuses is motion_limits.limit_statuses(validator). With it, every
    exceeded limit names its provenance, so a refusal on an assumed limit
    (jerk, today, on every joint) says so rather than reading as a hardware
    limit.
    """
    failures = []
    for joint, kinds in sorted((report.get('limit_violations') or {}).items()):
        for kind, detail in sorted(kinds.items()):
            text = f'{joint} {kind} {detail["peak"]:.3g} > {detail["limit"]:.3g}'
            if statuses is not None and kind in statuses:
                text += f' ({statuses[kind][JOINT_NAMES.index(joint)]} limit)'
            failures.append(text)
    if report.get('position_limit_violations'):
        failures.append('joint limits left between waypoints')
    if report.get('collision', {}).get('collision_found'):
        failures.append('collision between waypoints')
    if not report.get('tracking', {}).get('within_tolerance', True):
        failures.append('tool leaves the target path between waypoints')
    if not report.get('conditioning_ok', True):
        failures.append('arm conditioning above the gate')
    status = report.get('conditioning', {}).get('twist_status')
    if status not in (None, 'pass', 'not_applicable'):
        failures.append(f'task twist {status}')
    return failures


def check_measured_start(validator, q_measured, first_configuration,
                         target_position, target_quaternion):
    """Whether the arm is where the task begins, before anything is played.

    Two checks, both needed. The measured joints must match the first solved
    configuration within START_RAIL_TOL_M and START_ARM_TOL_RAD, since the
    task has no transition to absorb an offset; and forward kinematics of the
    measured joints must reach the first target within the IK pose
    tolerances, since joints can match a configuration that is not the one
    the task was solved for only if the solve itself was wrong.

    Returns (ok, details).
    """
    from scipy.spatial.transform import Rotation

    from ur10e_trajectory_pkg.pose_metrics import (
        IK_ORIENTATION_TOL_RAD,
        IK_POSITION_TOL_M,
    )

    q_measured = np.asarray(q_measured, dtype=float)
    first = np.asarray(first_configuration, dtype=float)
    rail_offset = abs(q_measured[0] - first[0])
    arm_offset = float(np.max(np.abs(q_measured[1:] - first[1:])))
    pose = validator.robot.fkine(q_measured, end='tool0')
    position_error = float(np.linalg.norm(pose.t - np.asarray(target_position)))
    orientation_error = float(np.linalg.norm(
        (Rotation.from_quat(target_quaternion).inv()
         * Rotation.from_matrix(pose.R)).as_rotvec()))
    failures = []
    if rail_offset > START_RAIL_TOL_M:
        failures.append(f'rail {rail_offset:.4f} m from the task start '
                        f'(tolerance {START_RAIL_TOL_M} m)')
    if arm_offset > START_ARM_TOL_RAD:
        failures.append(f'arm joint {arm_offset:.4f} rad from the task start '
                        f'(tolerance {START_ARM_TOL_RAD} rad)')
    if position_error > IK_POSITION_TOL_M:
        failures.append(f'first target missed by {position_error:.4f} m')
    if orientation_error > IK_ORIENTATION_TOL_RAD:
        failures.append(f'first target orientation off by '
                        f'{np.rad2deg(orientation_error):.3f} deg')
    return not failures, {
        'rail_offset_m': float(rail_offset), 'arm_offset_rad': arm_offset,
        'position_error_m': position_error,
        'orientation_error_rad': orientation_error, 'failures': failures,
    }


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

        # Service Servers: command 1 (warmup), then command 2 (the task)
        self.srv = self.create_service(
            ValidateTrajectory, 'validate_trajectory', self.validation_callback
        )
        self.warmup_srv = self.create_service(
            ExecuteWarmup, 'execute_warmup', self.warmup_callback
        )

        # Playback Timer Attributes
        self.playback_timer = None
        self.playback_frames = None
        self.current_frame_idx = 0
        # Frames are generated AND published at this rate. Publishing at a
        # different rate from the one frames were generated at replays the
        # motion at the wrong speed: 30 Hz frames used to be published at
        # 60 Hz, showing every trajectory at twice real speed.
        self.framerate = 30

        self.joint_names = list(JOINT_NAMES)

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
            resolve_target_frame(request.target_frame)
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

        num_waypts = len(ee_x)
        dt_waypoint = simTime[1] - simTime[0]
        q_dot_matrix, q_interp = np.array([]), np.array([])
        try:
            q_path = resolve_task_path(request.q_path, num_waypts)
        except ValueError as exc:
            is_valid, message = False, str(exc)
        else:
            # Play exactly the validated path: verify it here, check the arm
            # is at its first configuration, and never re-solve.
            verification = verify_task_path(
                self.validator, q_path, np.column_stack((ee_x, ee_y, ee_z)),
                ee_quat, dt_waypoint)
            start_ok, start_details = check_measured_start(
                self.validator, q_start, q_path[0], (ee_x[0], ee_y[0], ee_z[0]),
                ee_quat[0])
            if not verification['ok']:
                is_valid = False
                message = ('The joint path does not verify against its targets: '
                           + '; '.join(verification['failures']))
            elif not start_ok:
                is_valid = False
                message = ('Measured start is not the path start: '
                           + '; '.join(start_details['failures'])
                           + '. Run the warmup command first.')
            else:
                # The discrete checks see waypoints only. The motion between
                # them -- interpolation overshoot, acceleration, jerk,
                # collision, conditioning, alpha* -- is validated here on the
                # exact path received, so nothing plays that was not
                # continuously validated.
                continuous = validate_task_command(
                    self.validator, q_path, np.column_stack((ee_x, ee_y, ee_z)),
                    ee_quat, dt_waypoint, rate_hz=SERVICE_VALIDATION_HZ)
                is_valid = bool(continuous['passed'])
                if not is_valid:
                    message = ('Continuous validation of the joint path failed: '
                               + '; '.join(continuous_failures(
                                   continuous, limit_statuses(self.validator))))
            if is_valid:
                segment = {'start_idx': 0, 'end_idx': num_waypts - 1,
                           'length': num_waypts, 'q_full': q_path}
                q_dot_matrix, q_interp, _ = self.validator.process_task_segment(
                    segment, dt_waypoint, verbose=True)
                message = (f'Verified joint path over all {num_waypts} waypoints '
                           f'(worst step {verification["max_step_ratio"]:.3f}x the '
                           f'velocity limits); playing it from the measured start '
                           f'with no transition [start: {start_desc}]')

        if is_valid:
            response.success = True
            response.message = message
            response.joint_velocities = q_dot_matrix.flatten().tolist()

            # Start streaming the trajectory over /joint_states
            self.start_trajectory_playback(q_interp)
        else:
            response.success = False
            response.message = f'Validation failed: {message}'
            response.joint_velocities = []

        return response

    def warmup_callback(self, request, response):
        """Command 1: validate and play the warmup to the task's start."""
        self.get_logger().info('Received warmup request...')
        response.duration_s = 0.0
        response.end_configuration = []
        response.segment_index = 0
        response.segment_count = 0
        try:
            q_start = resolve_configuration(request.q_start, 'q_start')
            q_target = resolve_configuration(request.q_target, 'q_target')
            rest_points = resolve_rest_points(request.rest_points, q_start, q_target)
        except ValueError as exc:
            response.success = False
            response.message = f'Error: {exc}'
            return response
        segment_index = int(getattr(request, 'segment_index', 0))
        ok, message, frames, route = plan_and_validate_route(
            self.validator, rest_points, segment_index, q_start, self.framerate)
        response.success = ok
        response.message = message
        response.segment_count = len(rest_points) - 1
        if ok:
            segment = route['segments'][segment_index]
            response.segment_index = segment_index
            response.duration_s = float(segment['duration_s'])
            response.end_configuration = list(segment['ends_at'])
            self.start_trajectory_playback(frames)
        else:
            self.get_logger().error(message)
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
