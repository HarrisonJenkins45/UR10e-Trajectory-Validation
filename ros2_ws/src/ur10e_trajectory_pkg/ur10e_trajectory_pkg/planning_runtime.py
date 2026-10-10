"""Small shared inputs for the production candidate and graph stages.

This module contains no census modes or placement sweep. Separate planning
processes use these helpers to agree on source bytes, seeds, and targets.
"""
import hashlib
import os
import subprocess

import numpy as np

from ur10e_trajectory_pkg import trajectory_input
from ur10e_trajectory_pkg.configurations import LEGACY_MATLAB_START_Q


WIDE_SEED_BANK_DEG = (
    (0.0, -135.0, 90.0, -90.0, 0.0, 0.0),
    (0.0, -135.0, 90.0, -90.0, 45.0, 0.0),
    (0.0, -90.0, 0.0, -90.0, 0.0, 0.0),
    (90.0, -60.0, 60.0, -90.0, 90.0, 0.0),
    (-90.0, -120.0, -60.0, -60.0, -90.0, 45.0),
    (45.0, -100.0, 110.0, -120.0, 60.0, -90.0),
    (-45.0, -160.0, 120.0, -40.0, -60.0, 90.0),
    (180.0, -90.0, 90.0, -90.0, 90.0, 180.0),
)
MIN_SEGMENT_LENGTH = 10


def file_digest(path):
    """SHA-256 of an input file, or None if it cannot be read."""
    try:
        with open(path, 'rb') as handle:
            return hashlib.sha256(handle.read()).hexdigest()
    except OSError:
        return None


def recording_record(csv_path=None):
    """The recording a stage loaded: path and digest of its bytes."""
    path = str(csv_path or trajectory_input.DEFAULT_CSV_PATH)
    return {'csv_path': path, 'csv_sha256': trajectory_input.source_digest(path)}


def recording_mismatch(recorded, current):
    """Why an upstream artifact was built from other bytes, or None."""
    if current['csv_sha256'] is None:
        return f"the recording {current['csv_path']} cannot be read"
    if not isinstance(recorded, dict) or not recorded.get('csv_sha256'):
        return 'missing the source recording digest'
    expected, source = recorded['csv_sha256'], recorded.get('csv_path')
    if expected == current['csv_sha256']:
        return None
    return (f'built from {source} (sha256 {str(expected)[:12]}) but this '
            f"stage loads {current['csv_path']} "
            f"(sha256 {current['csv_sha256'][:12]})")


def repository_revision(path):
    """Commit of the workspace, with the container's injected revision as fallback."""
    from_env = os.environ.get('UR10E_GIT_REVISION')
    if from_env:
        return from_env.strip()
    try:
        result = subprocess.run(
            ['git', '-c', f'safe.directory={path}', '-C', str(path), 'rev-parse', 'HEAD'],
            capture_output=True, text=True, timeout=10, check=False)
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() or None


def make_validator(urdf_path, mesh_path):
    """Construct the same validator in every planning process."""
    from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

    return TrajectoryValidator(urdf_path, mesh_base_path=mesh_path)


def load_trajectory(csv_path, num_waypoints, with_metadata=False, spin_up_s=None,
                    start_index=0):
    """Return fixed-position targets for a recorded orientation slice."""
    from ur10e_trajectory_pkg.target_builder import build_trajectory_targets

    result = build_trajectory_targets(
        csv_path or trajectory_input.DEFAULT_CSV_PATH, num_waypoints,
        return_metadata=with_metadata, spin_up_s=spin_up_s, start_index=start_index)
    if with_metadata:
        (x, y, z, quaternions, times), metadata = result
        return (np.column_stack((x, y, z)), quaternions,
                float(times[1] - times[0]), metadata)
    x, y, z, quaternions, times = result
    return np.column_stack((x, y, z)), quaternions, float(times[1] - times[0])


def run_tracking(validator, targets, quaternions, dt, q_start):
    """Greedy reference path used to seed candidate generation."""
    records = []
    segments = validator.find_feasible_segments(
        targets[:, 0], targets[:, 1], targets[:, 2], quaternions, q_start,
        min_length=MIN_SEGMENT_LENGTH, dt_waypoint=dt, verbose=False,
        recorder=records.append, condition_number_threshold=50.0,
        max_joint_vel_threshold=None)
    for record in records:
        record['mode'] = 'tracking'
    return records, segments


def candidate_manifest(args, validator, trajectory_metadata, modes, dt):
    """Reproducibility details stored with generated candidates."""
    from ur10e_trajectory_pkg import environment, motion_limits
    from ur10e_trajectory_pkg.pose_metrics import IK_ORIENTATION_TOL_RAD, IK_POSITION_TOL_M
    from ur10e_trajectory_pkg.validation_core import IK_SEARCH_LIMIT, IK_SOLVER_TOL

    workspace = environment.workspace_root()
    return {
        'repository_revision': repository_revision(workspace),
        'docker_image_id': os.environ.get('UR10E_IMAGE_ID'),
        'inputs': {
            'urdf_path': args.urdf,
            'urdf_sha256': file_digest(args.urdf),
            'csv_path': trajectory_metadata['csv_path'],
            'csv_sha256': recording_record(trajectory_metadata['csv_path'])['csv_sha256'],
        },
        'solver': {
            'tolerance': IK_SOLVER_TOL,
            'search_limit': IK_SEARCH_LIMIT,
            'rng_seed': validator._seed,
        },
        'gate_thresholds': {
            'condition_number': 50.0,
            'velocity_limits': motion_limits.effective_limits(validator),
            'rail_position_limits_m': list(validator._rail_limits),
            'pose_position_m': IK_POSITION_TOL_M,
            'pose_orientation_rad': IK_ORIENTATION_TOL_RAD,
        },
        'trajectory': dict(trajectory_metadata, dt_waypoint_s=dt),
        'run': {
            'q_start': LEGACY_MATLAB_START_Q.tolist(),
            'q_start_name': 'LEGACY_MATLAB_START_Q',
            'independent_solve_rail_seed_m': 1.5,
            'min_segment_length': MIN_SEGMENT_LENGTH,
            'modes': list(modes),
            'wide_seed_bank_deg': [list(s) for s in WIDE_SEED_BANK_DEG],
        },
    }
