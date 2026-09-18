#!/usr/bin/env python3
"""Which recording each stage planned, and refusing to mix recordings.

Every stage loads the recording itself, so the only thing tying candidates,
graph, commands and the client's rebuilt targets to one motion is the digest
each records and the next one checks.
"""
import shutil

import numpy as np
import pandas as pd
import pytest

from ur10e_trajectory_pkg import ClientNode as client
from ur10e_trajectory_pkg.failure_census import (
    file_digest,
    recording_mismatch,
    recording_record,
)


@pytest.fixture
def packaged():
    if file_digest(client.DEFAULT_CSV_PATH) is None:
        pytest.skip('packaged trajectory CSV not present')
    return client.DEFAULT_CSV_PATH


def _other(tmp_path, packaged, rows=20):
    """A recording with other bytes: the packaged one, cut short."""
    with open(packaged, encoding='utf-8') as handle:
        lines = handle.read().splitlines()[:rows + 1]
    path = tmp_path / f'first_{rows}.csv'
    path.write_text('\n'.join(lines) + '\n')
    return path


def test_the_same_bytes_at_another_path_agree(tmp_path, packaged):
    """Containers mount the same recording at different paths."""
    copy = tmp_path / 'copy.csv'
    shutil.copyfile(packaged, copy)
    assert recording_mismatch(recording_record(packaged),
                              recording_record(copy)) is None


def test_other_bytes_are_refused_and_both_sides_named(tmp_path, packaged):
    other = _other(tmp_path, packaged)
    reason = recording_mismatch(recording_record(packaged), recording_record(other))
    assert reason is not None
    assert str(other) in reason and packaged in reason


def test_a_record_without_a_digest_is_the_packaged_recording(tmp_path, packaged):
    """Artifacts from before --csv carry no digest. Every stage then loaded
    the packaged recording, so those are the only bytes they agree with."""
    assert recording_mismatch(None, recording_record(None)) is None
    assert recording_mismatch({}, recording_record(_other(tmp_path, packaged))) is not None


def test_an_unreadable_recording_is_refused(tmp_path):
    missing = recording_record(tmp_path / 'absent.csv')
    assert missing['csv_sha256'] is None
    assert 'cannot be read' in recording_mismatch(None, missing)


def test_asking_for_more_samples_than_recorded_is_refused(tmp_path, packaged):
    """num_waypoints takes the first samples; it does not resample, so a
    shorter recording used to yield mismatched arrays rather than an error."""
    short = _other(tmp_path, packaged, rows=20)
    with pytest.raises(ValueError, match='fewer than'):
        client.build_trajectory_targets(short, 21)
    with pytest.raises(ValueError, match='fewer than'):
        client.recorded_start_rate(short, 21)
    x, *_ = client.build_trajectory_targets(short, 20)
    assert len(x) == 20


def test_targets_use_only_q_I_G_and_hold_position_fixed(tmp_path, packaged):
    """The current motion contract is triaxial tumble only.

    SISIFOS positions and the camera attitude must not influence targets, and
    no dormant translation scale may appear in the resulting metadata.
    """
    frame = pd.read_csv(packaged).iloc[:20].copy()
    original = tmp_path / 'original.csv'
    changed = tmp_path / 'changed_non_tumble_columns.csv'
    frame.to_csv(original, index=False)

    altered = frame.copy()
    sample = np.arange(len(altered), dtype=float)
    for index, column in enumerate(
            ('p_G_I_x', 'p_G_I_y', 'p_G_I_z',
             'p_C_I_x', 'p_C_I_y', 'p_C_I_z')):
        altered[column] = 1.0e6 * (index + 1) + sample ** 2
    altered[['q_I_C_x', 'q_I_C_y', 'q_I_C_z']] = 0.0
    altered['q_I_C_w'] = 1.0
    altered.to_csv(changed, index=False)

    first, metadata = client.build_trajectory_targets(
        original, 20, return_metadata=True)
    second = client.build_trajectory_targets(changed, 20)
    for actual, expected in zip(second, first):
        np.testing.assert_allclose(actual, expected, atol=1e-12)

    positions = np.column_stack(first[:3])
    np.testing.assert_allclose(
        positions, np.tile(positions[0], (len(positions), 1)), atol=1e-12)
    assert 'bound_m' not in metadata
    assert 'translation_scale_factor' not in metadata


def _plan(**overrides):
    """recorded_waypoints 2 with a 2 s spin-up rebuilds 12 targets."""
    q_path = [[0.53, 0.68, -1.27, 2.05, -3.86, -2.35, -1.27 + 0.001 * i]
              for i in range(12)]
    plan = {'q_path': q_path, 'recorded_waypoints': 2, 'spin_up_s': 2.0,
            'placement': 'nominal'}
    plan.update(overrides)
    return plan


def test_a_plan_is_rebuilt_from_the_recording_it_names(tmp_path, packaged):
    other = _other(tmp_path, packaged)
    x, *_ = client.plan_targets(_plan(recording=recording_record(other)))
    assert len(x) == 12


def test_a_plan_is_refused_against_other_bytes(tmp_path, packaged):
    """Another recording's targets are a different motion under a validated
    label, even when the caller points at the file explicitly."""
    plan = _plan(recording=recording_record(packaged))
    with pytest.raises(ValueError, match='the plan was built from'):
        client.plan_targets(plan, csv_path=_other(tmp_path, packaged))


def test_a_plan_from_before_recordings_were_named_uses_the_packaged_one(packaged):
    x, *_ = client.plan_targets(_plan())
    assert len(x) == 12
