"""Read and validate orientation trajectories at the file boundary.

This is the only production module that parses a trajectory CSV. Planning,
target construction, diagnostics, and RViz consume its timestamp/quaternion
arrays and source identity instead of implementing their own readers. The
current on-disk format is the SISIFOS-shaped ``timestamp, q_I_G_*`` CSV; a
different source format can be adapted here without changing the planner.
"""

from dataclasses import dataclass
from io import BytesIO
from pathlib import Path
import hashlib

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


REQUIRED_COLUMNS = ('timestamp', 'q_I_G_x', 'q_I_G_y', 'q_I_G_z', 'q_I_G_w')
STEP_TOLERANCE_S = 1e-9
QUATERNION_NORM_TOLERANCE = 1e-6
DEFAULT_CSV_PATH = ('/root/ros2_ws/src/ur10e_trajectory_pkg/'
                    'ur10e_trajectory_pkg/camera_traj.csv')


@dataclass(frozen=True)
class TrajectorySlice:
    """Consecutive samples, retaining their original timestamps."""

    times_s: np.ndarray
    orientations_xyzw: np.ndarray

    @property
    def step_s(self):
        return float(self.times_s[1] - self.times_s[0])


@dataclass(frozen=True)
class TrajectoryInput:
    """Validated source trajectory, independent of ROS and the robot model."""

    path: str
    sha256: str
    times_s: np.ndarray
    orientations_xyzw: np.ndarray

    def slice(self, start_index, samples):
        start, count = int(start_index), int(samples)
        if start < 0:
            raise ValueError(f'start_index must be non-negative, got {start}')
        if count < 2:
            raise ValueError(f'a trajectory slice needs at least two samples, got {count}')
        if start + count > len(self.times_s):
            raise ValueError(f'{self.path} holds {len(self.times_s)} samples, fewer than '
                             f'the {count} waypoints asked for from sample {start}')
        return TrajectorySlice(self.times_s[start:start + count],
                               self.orientations_xyzw[start:start + count])


def source_digest(path):
    """Identity of the source bytes, or None when the file cannot be read."""
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return None


def read_with_report(path, min_samples=None):
    """Read once and return (inspection record, validated input or None)."""
    record = {'csv_path': str(path), 'source_digest': None, 'problems': []}
    problems = record['problems']
    try:
        source = Path(path).read_bytes()
    except OSError as exc:
        problems.append(f'unreadable: {exc}')
        record['passed'] = False
        return record, None
    record['source_digest'] = hashlib.sha256(source).hexdigest()
    try:
        frame = pd.read_csv(BytesIO(source))
    except (OSError, ValueError, UnicodeError) as exc:
        problems.append(f'unreadable: {exc}')
        record['passed'] = False
        return record, None

    missing = [column for column in REQUIRED_COLUMNS if column not in frame.columns]
    if missing:
        problems.append(f'missing columns {missing}')
        record['passed'] = False
        return record, None
    try:
        times = frame['timestamp'].to_numpy(dtype=float)
        quaternions = frame[list(REQUIRED_COLUMNS[1:])].to_numpy(dtype=float)
    except (TypeError, ValueError) as exc:
        problems.append(f'non-numeric timestamps or quaternions: {exc}')
        record['passed'] = False
        return record, None

    record['samples'] = int(len(frame))
    if min_samples is not None and len(frame) < int(min_samples):
        problems.append(f'{len(frame)} samples, fewer than the {min_samples} '
                        'waypoints asked for')
    if len(frame) < 2:
        problems.append('fewer than two samples')
    if not (np.all(np.isfinite(times)) and np.all(np.isfinite(quaternions))):
        problems.append('non-finite timestamps or quaternions')
    if problems:
        record['passed'] = False
        return record, None

    steps = np.diff(times)
    record['duration_s'] = float(times[-1] - times[0])
    record['step_s'] = float(steps[0])
    record['step_range_s'] = [float(steps.min()), float(steps.max())]
    if np.any(steps <= 0.0):
        problems.append('timestamps do not increase')
    elif not np.allclose(steps, steps[0], atol=STEP_TOLERANCE_S, rtol=0.0):
        problems.append(f'not uniformly sampled: steps span {steps.min():.12g} '
                        f'to {steps.max():.12g} s')
    norm_error = float(np.max(np.abs(np.linalg.norm(quaternions, axis=1) - 1.0)))
    record['max_quaternion_norm_error'] = norm_error
    if norm_error > QUATERNION_NORM_TOLERANCE:
        problems.append(f'quaternion norm off unity by {norm_error:.3g}')

    if not problems:
        rotations = Rotation.from_quat(quaternions)
        rates = np.degrees((rotations[1:] * rotations[:-1].inv()).magnitude() / steps)
        record['angular_rate_deg_s'] = {'min': float(rates.min()),
                                        'median': float(np.median(rates)),
                                        'max': float(rates.max())}
    record['passed'] = not problems
    if problems:
        return record, None
    return record, TrajectoryInput(str(path), record['source_digest'], times, quaternions)


def inspect(path, min_samples=None):
    """Return the input report without raising for invalid trajectory data."""
    return read_with_report(path, min_samples)[0]


def load(path):
    """Return a validated trajectory or explain why the input is invalid."""
    record, trajectory = read_with_report(path)
    if trajectory is None:
        raise ValueError(f'invalid trajectory {path}: {"; ".join(record["problems"])}')
    return trajectory
