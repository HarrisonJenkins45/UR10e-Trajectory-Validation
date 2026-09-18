#!/usr/bin/env python3
"""Verify the exact joint path the task service will play.

The planner produces a path and offline validation checks it; the service then
plays it. Re-solving the trajectory at the service could follow a different
branch, so the service instead re-checks the path it was given, with discrete
checks that are cheap enough to run on every request:

  shape           one 7-vector per target
  joint limits    every configuration inside the URDF limits
  velocity        every step within velocity_limit * dt, on the raw (lifted)
                  difference, so an unlifted wrap shows up as a violation
  pose            every configuration reaches its target within the IK
                  tolerances, so the path is a solution of THESE targets
  collision       no configuration in collision

Continuous properties -- the interpolant between waypoints, jerk, alpha* --
are validated offline by continuous_validator.validate_task_command on this
same path.
"""
import numpy as np
from scipy.spatial.transform import Rotation

from ur10e_trajectory_pkg.pose_metrics import IK_ORIENTATION_TOL_RAD, IK_POSITION_TOL_M

EE_LINK = 'tool0'


def verify_task_path(validator, q_path, positions, quaternions, dt,
                     velocity_limits=None):
    """Discrete checks on a joint path against its targets. Returns a report."""
    q_path = np.asarray(q_path, dtype=float)
    positions = np.asarray(positions, dtype=float)
    quaternions = np.asarray(quaternions, dtype=float)
    failures = []
    if q_path.ndim != 2 or q_path.shape[1] != 7 or len(q_path) != len(positions):
        return {'ok': False, 'failures': [
            f'q_path has shape {q_path.shape}, expected ({len(positions)}, 7)']}
    velocity_limits = (validator.velocity_limits if velocity_limits is None
                       else np.asarray(velocity_limits, dtype=float))

    lower, upper = validator.robot.qlim
    outside = np.flatnonzero(np.any((q_path < lower - 1e-9) | (q_path > upper + 1e-9), axis=1))
    if len(outside):
        failures.append(f'{len(outside)} configurations outside joint limits, '
                        f'first at waypoint {int(outside[0])}')

    step_ratio = np.abs(np.diff(q_path, axis=0)) / (velocity_limits * float(dt))
    worst_step = float(step_ratio.max()) if len(step_ratio) else 0.0
    over = np.flatnonzero(np.any(step_ratio > 1.0 + 1e-9, axis=1))
    if len(over):
        failures.append(f'{len(over)} steps exceed the velocity limits, first from '
                        f'waypoint {int(over[0])} (worst {worst_step:.2f}x)')

    position_errors, orientation_errors, collisions = [], [], []
    for index, configuration in enumerate(q_path):
        pose = validator.robot.fkine(configuration, end=EE_LINK)
        position_errors.append(float(np.linalg.norm(pose.t - positions[index])))
        orientation_errors.append(float(np.linalg.norm(
            (Rotation.from_quat(quaternions[index]).inv()
             * Rotation.from_matrix(pose.R)).as_rotvec())))
        if validator.check_all_collisions(configuration):
            collisions.append(index)
    missed = [i for i in range(len(q_path))
              if position_errors[i] > IK_POSITION_TOL_M
              or orientation_errors[i] > IK_ORIENTATION_TOL_RAD]
    if missed:
        failures.append(f'{len(missed)} configurations miss their target, '
                        f'first at waypoint {missed[0]}')
    if collisions:
        failures.append(f'{len(collisions)} configurations in collision, '
                        f'first at waypoint {collisions[0]}')

    return {
        'ok': not failures,
        'failures': failures,
        'waypoints': int(len(q_path)),
        'max_step_ratio': worst_step,
        'max_position_error_m': max(position_errors),
        'max_orientation_error_rad': max(orientation_errors),
        'collisions': len(collisions),
    }
