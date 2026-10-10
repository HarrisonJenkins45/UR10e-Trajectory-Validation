"""One reader supplies the same validated samples to every consumer."""

import hashlib

import numpy as np
import pandas as pd
import pytest

from ur10e_trajectory_pkg import trajectory_input


def _trajectory(path):
    pd.DataFrame({
        'timestamp': [10.0, 10.1, 10.2, 10.3],
        'q_I_G_x': [0.0] * 4,
        'q_I_G_y': [0.0] * 4,
        'q_I_G_z': [0.0] * 4,
        'q_I_G_w': [1.0] * 4,
        'unused_camera_position': [1.0, 2.0, 3.0, 4.0],
    }).to_csv(path, index=False)
    return path


def test_read_slice_and_identity_share_one_source(tmp_path):
    path = _trajectory(tmp_path / 'motion.csv')
    report, source = trajectory_input.read_with_report(path)
    assert report['passed']
    assert source.sha256 == hashlib.sha256(path.read_bytes()).hexdigest()
    selected = source.slice(1, 3)
    np.testing.assert_allclose(selected.times_s, [10.1, 10.2, 10.3])
    np.testing.assert_allclose(selected.orientations_xyzw[:, 3], [1.0] * 3)
    assert selected.step_s == pytest.approx(0.1)


def test_slice_refuses_missing_samples_and_negative_start(tmp_path):
    source = trajectory_input.load(_trajectory(tmp_path / 'motion.csv'))
    with pytest.raises(ValueError, match='from sample 3'):
        source.slice(3, 2)
    with pytest.raises(ValueError, match='non-negative'):
        source.slice(-1, 2)


def test_inspection_and_load_use_the_same_validation(tmp_path):
    path = _trajectory(tmp_path / 'motion.csv')
    frame = pd.read_csv(path)
    frame.loc[2, 'timestamp'] = 10.25
    frame.to_csv(path, index=False)
    report = trajectory_input.inspect(path)
    assert not report['passed'] and 'uniformly' in report['problems'][0]
    with pytest.raises(ValueError, match='uniformly'):
        trajectory_input.load(path)


def test_source_identity_changes_when_bytes_change(tmp_path):
    path = _trajectory(tmp_path / 'motion.csv')
    first = trajectory_input.load(path).sha256
    with path.open('a', encoding='utf-8') as handle:
        handle.write('\n')
    assert trajectory_input.load(path).sha256 != first
