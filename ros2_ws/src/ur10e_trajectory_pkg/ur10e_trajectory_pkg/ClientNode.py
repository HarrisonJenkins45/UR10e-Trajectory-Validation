import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from ur10e_interfaces.srv import ExecuteWarmup, ValidateTrajectory
from ur10e_trajectory_pkg import frames
from ur10e_trajectory_pkg.plan_artifact import load_plan, plan_targets

# The plan a demo run executes: exported by home_pose commands --plan-out,
# only for a plan whose warmup and task both passed validation.
DEFAULT_PLAN_PATH = '/root/ros2_ws/task_plan.json'
# Extra wait after the warmup's duration before sending the task, so the
# warmup playback has finished when command 2 arrives.
PLAYBACK_MARGIN_S = 1.0

class TrajectoryClientNode(Node):

    def __init__(self):
        super().__init__('trajectory_client_node')
        self.cli = self.create_client(ValidateTrajectory, 'validate_trajectory')
        self.warmup_cli = self.create_client(ExecuteWarmup, 'execute_warmup')

        for client, name in ((self.warmup_cli, 'execute_warmup'),
                             (self.cli, 'validate_trajectory')):
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name} service...')

    def send_warmup(self, q_start, q_target, rest_points=None, segment_index=0):
        """Command 1: one leg of the route, from where the arm is measured to be.

        rest_points names where the route stands still; the server plans every
        leg from them itself. One call per leg, because the server checks
        q_start against the leg's planned start and that is the only
        measurement it gets.
        """
        req = ExecuteWarmup.Request()
        req.q_start = np.asarray(q_start, dtype=float).tolist()
        req.q_target = np.asarray(q_target, dtype=float).tolist()
        req.rest_points = ([] if rest_points is None else
                           np.asarray(rest_points, dtype=float).flatten().tolist())
        req.segment_index = int(segment_index)
        return self.warmup_cli.call_async(req)

    def send_request(self, x_pts, y_pts, z_pts, quat, simTime, q_start, q_path):
        """Command 2: the task, as the validated joint path from its start."""
        req = ValidateTrajectory.Request()
        req.ee_positions_x = list(x_pts)
        req.ee_positions_y = list(y_pts)
        req.ee_positions_z = list(z_pts)
        # quat is an (N, 4) array of [x, y, z, w] quaternions, flattened
        # because service fields cannot carry a 2D array; the server reshapes.
        req.ee_quat = np.asarray(quat).flatten().tolist()
        req.sim_time = list(simTime)
        # Where the arm is: after command 1, the warmup's end configuration.
        req.q_start = np.asarray(q_start, dtype=float).tolist()
        # The validated joint path the service verifies and plays, never
        # re-solving it.
        req.q_path = np.asarray(q_path, dtype=float).flatten().tolist()
        # Targets are in the fixed rail-base frame, never the moving carriage.
        req.target_frame = frames.TARGET_FRAME

        self.future = self.cli.call_async(req)
        return self.future


def main(args=None):
    """Command 1 then command 2, from an exported, validated plan.

    ros2 run ur10e_trajectory_pkg trajectory_client [plan.json]

    The simulated arm starts at the plan's home. The warmup route brings it
    to the task start, one rest-to-rest leg per call so the server can check
    where the arm actually is at each pause, and the task then plays from the
    last leg's end configuration as the exact joint path that was validated.
    """
    from rclpy.utilities import remove_ros_args

    rclpy.init(args=args)
    argv = remove_ros_args(sys.argv if args is None else args)[1:]
    plan_path = argv[0] if argv else DEFAULT_PLAN_PATH
    client_node = TrajectoryClientNode()
    logger = client_node.get_logger()

    try:
        plan = load_plan(plan_path)
        x_pts, y_pts, z_pts, q_B_G, simTime = plan_targets(plan)
    except (OSError, ValueError) as exc:
        logger.error(f'Cannot run plan {plan_path}: {exc}')
        client_node.destroy_node()
        rclpy.shutdown()
        return

    # Command 1: the certified warmup route from home to the task start.
    route = plan['warmup_route']
    rest_points = route['rest_points']
    legs = len(rest_points) - 1
    measured = plan['home']
    warmup = None
    for segment_index in range(legs):
        future = client_node.send_warmup(measured, plan['task_start'],
                                         rest_points=rest_points,
                                         segment_index=segment_index)
        rclpy.spin_until_future_complete(client_node, future)
        warmup = future.result()
        if not warmup.success:
            logger.error(f'Warmup refused: {warmup.message}')
            client_node.destroy_node()
            rclpy.shutdown()
            return
        logger.info(f'{warmup.message}; waiting {warmup.duration_s:.2f} s '
                    'for it to play')
        time.sleep(warmup.duration_s + PLAYBACK_MARGIN_S)
        # Where the next leg starts from. In simulation the arm arrives
        # exactly where the leg planned; on hardware a fresh encoder reading
        # belongs here, which is what the server checks it against.
        measured = warmup.end_configuration

    # Command 2: the task from where the warmup ended.
    future = client_node.send_request(
        x_pts, y_pts, z_pts, q_B_G, simTime,
        q_start=warmup.end_configuration, q_path=plan['q_path'])
    rclpy.spin_until_future_complete(client_node, future)
    response = future.result()
    if response.success:
        logger.info(f'Success: {response.message}')
    else:
        logger.error(f'Failed: {response.message}')

    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
