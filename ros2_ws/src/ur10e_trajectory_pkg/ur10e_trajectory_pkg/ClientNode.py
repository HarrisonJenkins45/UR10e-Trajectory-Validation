import numpy as np
import rclpy
import pandas as pd
from rclpy.node import Node
from ur10e_interfaces.srv import ValidateTrajectory
from scipy.spatial.transform import Rotation as R



TEST_VALID_TRAJ=False

class TrajectoryClientNode(Node):

    def __init__(self):
        super().__init__('trajectory_client_node')
        self.cli = self.create_client(ValidateTrajectory, 'validate_trajectory')


        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for validate_trajectory service...')

    def send_request(self, x_pts, y_pts, z_pts, quat, simTime):
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



        self.future = self.cli.call_async(req)
        return self.future

    @staticmethod
    def get_end_effector_in_base_frame(p_G_I, q_I_G, p_B_I, q_I_B, TARGET_BOUND_M):
        """Converts camera position/orientation from inertial (I) frame to UR10e base (B) frame

        with spatial scaling to bound maximum trajectory extent.
        """
        # 1. Spatial scaling
        max_disp = np.max(np.linalg.norm(p_G_I - p_G_I[0], axis=1))
        scale_spatial = TARGET_BOUND_M / max_disp if max_disp > 1e-9 else 1.0

        p_rel_scaled = scale_spatial * (p_G_I - p_G_I[0])
        p_G_I_scaled = p_G_I[0] + p_rel_scaled

        # 2. Transform scaled target position to base frame
        r_I_B = R.from_quat(q_I_B[0]) if q_I_B.ndim > 1 else R.from_quat(q_I_B)
        p_G_B = r_I_B.apply(p_G_I_scaled - p_B_I)

        # 3. Orientations relative to base frame
        r_I_G = R.from_quat(q_I_G)
        r_B_G = r_I_B.inv() * r_I_G
        q_B_G = r_B_G.as_quat()

        return p_G_B, q_B_G


def main(args=None):
    rclpy.init(args=args)
    client_node = TrajectoryClientNode()


    # ------------- Start SISFOS Logic ------------#

    #Read in the trajectory from SISIFOSp_G_I
    csv_path='/root/ros2_ws/src/ur10e_trajectory_pkg/ur10e_trajectory_pkg/camera_traj.csv'
    df = pd.read_csv(csv_path)
    num_pts= 300# or p_G_I.shape[0]

    # Slice trajectory to match num_pts
    q_I_G = df[['q_I_G_x', 'q_I_G_y', 'q_I_G_z', 'q_I_G_w']].to_numpy(dtype=np.float64)[:num_pts]
    p_G_I = df[['p_G_I_x', 'p_G_I_y', 'p_G_I_z']].to_numpy(dtype=np.float64)[:num_pts]


    simTime = df['timestamp'].to_numpy(dtype=np.float64)[:num_pts]

    TARGET_BOUND_M = 1.0  # Fit trajectory inside 1-meter radius

    # TODO: Update with real VICON readings when integrated
    p_B_I = p_G_I[0] - np.array([1.0, 0.0, 0.0])  # Base offset 1m back from initial point
    q_I_B = np.tile(np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64), (num_pts, 1))

    #Compute transformation
    p_G_B, q_B_G = client_node.get_end_effector_in_base_frame(p_G_I, q_I_G, p_B_I, q_I_B, TARGET_BOUND_M)
    x_pts=p_G_B[:,0]
    y_pts=p_G_B[:,1]
    z_pts=p_G_B[:,2]

    # ------------- End SISIFOS Logic ------------#

    #Override SISFOS request with a know valid trajectory
    if TEST_VALID_TRAJ:
        self.get_logger().info('Using a known valid trajectory')

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
