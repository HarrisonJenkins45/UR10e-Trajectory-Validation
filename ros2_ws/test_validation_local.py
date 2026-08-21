#!/usr/bin/env python3
"""Standalone, ROS-free test harness for TrajectoryValidator.

Run this directly on your Mac to iterate on validation logic in seconds, with no
Docker/ROS/Gazebo round-trip.
"""

import os
import time
import matplotlib.backend_bases
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from validation_core import TrajectoryValidator

# Compatibility shim for matplotlib backend
if not hasattr(matplotlib.backend_bases.FigureCanvasBase, 'tostring_rgb'):

  def _tostring_rgb(self):
    buf = np.asarray(self.buffer_rgba())
    return buf[:, :, :3].tobytes()

  matplotlib.backend_bases.FigureCanvasBase.tostring_rgb = _tostring_rgb

# Paths & Flags
# Adjust relative path to your local mesh folder if needed
URDF_PATH = 'ur10e.urdf'          # adjust to wherever your local copy lives
MESH_BASE_PATH = '/Users/jenkinsh21/Desktop/GT/Research/DCSL/ASTROS_Sims/ur_description_ur10e'
SKIP_COLLISION = False
RECORD_MOVIE = True
MOVIE_PATH = 'trajectory_preview.gif'
TEST_SEGMENT_FINDER = False
USE_SEGMENT_FINDER = True
MIN_SEGMENT_LENGTH = 10

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


def main():
  mesh_path = MESH_BASE_PATH if os.path.exists(MESH_BASE_PATH) else None
  print(mesh_path)
  validator = TrajectoryValidator(
        URDF_PATH, mesh_base_path=mesh_path, framerate=30
    )

  if SKIP_COLLISION:
        validator.check_all_collisions = lambda q, verbose=False: False
        print('SKIP_COLLISION=True -- collision checking disabled.\n')

    # Initial joint Angles (Home)
  q_home = np.deg2rad([0.0, 0.0, -135.0, 90.0, -90.0, 0.0, 0.0])

    # Read trajectory CSV
  df = pd.read_csv('/Users/jenkinsh21/Desktop/GT/Research/DCSL/ASTROS_Sims/SISIFOS/trajectory/Config_1_RF_Hubble/Agent_0/camera_traj.csv')

  num_pts = 300  # or p_G_I.shape[0]

  # Slice trajectory to match num_pts
  q_I_G = df[['q_I_G_x', 'q_I_G_y', 'q_I_G_z', 'q_I_G_w']].to_numpy(dtype=np.float64)[:num_pts]
  p_G_I = df[['p_G_I_x', 'p_G_I_y', 'p_G_I_z']].to_numpy(dtype=np.float64)[:num_pts]
  sim_time = df[['timestamp']].to_numpy(dtype=np.float64)[:num_pts]
  print(sim_time.tolist())

#   TARGET_BOUND_M = 1.0  # Fit trajectory inside 1-meter radius
#   p_B_I = p_G_I[0] - np.array([1.0, 0.0, 0.0])  # Base offset 1m back from initial point
#   q_I_B = np.tile(np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64), (num_pts, 1))

# # Frame Transformations
#   p_G_B, q_B_G = get_end_effector_in_base_frame(p_G_I, q_I_G, p_B_I, q_I_B, TARGET_BOUND_M)
#   ee_x, ee_y, ee_z = p_G_B[:, 0], p_G_B[:, 1], p_G_B[:, 2]

#   t_traj, t_transition = 10.0, 2.0
#   t_rel = np.linspace(0, t_traj, num_pts)

#   print(f'Testing {num_pts} waypoints, q_start (deg) = {np.rad2deg(q_home).round(1)}\n')
#   start = time.time()

#   if USE_SEGMENT_FINDER:
#         dt_waypoint = t_rel[1] - t_rel[0]
#         segments = validator.find_feasible_segments(
#             ee_x,
#             ee_y,
#             ee_z,
#             q_B_G,              # Pass orientation quaternions [x, y, z, w]
#             q_home,
#             min_length=MIN_SEGMENT_LENGTH,
#             dt_waypoint=dt_waypoint,
#             verbose=True,
#         )
#         if not segments:
#             is_valid = False
#             q_dot, q_interp = np.array([]), np.array([])
#             message = f'No feasible segment of length >= {MIN_SEGMENT_LENGTH} found across {num_pts} waypoints'
#         else:
#             segment = max(segments, key=lambda s: s['length'])
#             is_valid = True
#             q_dot, q_interp = validator.process_feasible_segment(
#                 segment, q_home, dt_waypoint, t_transition=t_transition, verbose=True
#             )
#             message = f'Using largest feasible segment [{segment["start_idx"]}, {segment["end_idx"]}] (length={segment["length"]}/{num_pts}) as full trajectory'
#   else:
#         is_valid, q_dot, q_interp, message = validator.process_matlab_validation(
#             ee_x, 
#             ee_y, 
#             ee_z, 
#             q_B_G,              # Pass orientation quaternions [x, y, z, w]
#             q_home, 
#             t_transition=t_transition,
#             t_traj=t_traj,
#             verbose=True
#         )
#   elapsed = time.time() - start

#   print(f"\nResult: {'VALID' if is_valid else 'INVALID'} -- {message}")
#   print(f'Solve time: {elapsed:.2f}s')

#   if is_valid and RECORD_MOVIE:
#     print(f'\nRendering animation to {MOVIE_PATH} ({len(q_interp)} frames)...')
#     record_start = time.time()
#     validator.robot.plot(
#         q_interp,
#         backend='pyplot',
#         movie=MOVIE_PATH,
#         dt=1.0 / validator.framerate,
#         block=False,
#     )
#     print(
#         f'Saved animation to {MOVIE_PATH} in'
#         f' {time.time() - record_start:.1f}s'
#     )


if __name__ == '__main__':
  main()
