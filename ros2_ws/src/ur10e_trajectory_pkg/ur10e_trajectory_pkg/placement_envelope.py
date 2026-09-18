"""Provisional placement envelope retained for legacy diagnostics only."""

import numpy as np

from ur10e_trajectory_pkg import frames

# Software robustness envelope, NOT the physical operating envelope.
# Certification later needs the permitted arena volume, attitude range,
# calibration uncertainty and an obstacle survey, none of which this
# repository contains.
# V2 rotates about the layer-0 TARGET, in world (rail-base) axes. V1 rotated
# about the rail-base origin, so a 15 degree attitude change also translated
# the target by up to 0.46 m: 14 of 29 placements moved it further than the
# declared 0.25 m, three put it under the floor or into the wall, and the
# ranking was largely set by those shifts. Artifacts from the two definitions
# must not be compared.
PROVISIONAL_STAGE7_ENVELOPE_V2 = {
    'translation_m': 0.25,
    'rotation_deg': 15.0,
    'coupled_samples': 16,
    'scale': 1.0,          # relative-motion scale does not move layer 0
    'rotation_centre': 'layer-0 target (the placement origin; the trajectory '
                       'holds position constant)',
    'rotation_axes': 'world (rail-base frame), so placement names keep '
                     'their meaning',
    'name': 'PROVISIONAL_STAGE7_ENVELOPE_V2',
}


def placements(envelope=PROVISIONAL_STAGE7_ENVELOPE_V2, seed=0):
    """29 deterministic placements: nominal, 12 axis extrema, 16 coupled."""
    translation = envelope['translation_m']
    rotation = envelope['rotation_deg']
    out = [{'name': 'nominal', 'translation': np.zeros(3), 'rotation_deg': np.zeros(3)}]

    for axis in range(3):
        for sign in (-1.0, 1.0):
            shift = np.zeros(3)
            shift[axis] = sign * translation
            out.append({'name': f'translate_{"xyz"[axis]}{"+" if sign > 0 else "-"}',
                        'translation': shift, 'rotation_deg': np.zeros(3)})
    for axis in range(3):
        for sign in (-1.0, 1.0):
            turn = np.zeros(3)
            turn[axis] = sign * rotation
            out.append({'name': f'rotate_{"rpy"[axis]}{"+" if sign > 0 else "-"}',
                        'translation': np.zeros(3), 'rotation_deg': turn})

    # Deterministic coupled samples. A fixed generator rather than Sobol so
    # the set is reproducible without another dependency.
    rng = np.random.default_rng(seed)
    for index in range(envelope['coupled_samples']):
        out.append({
            'name': f'coupled_{index:02d}',
            'translation': rng.uniform(-translation, translation, 3),
            'rotation_deg': rng.uniform(-rotation, rotation, 3),
        })
    return out


def placement_transform(placement, nominal_RG):
    """Apply a placement offset to the nominal layer-0 placement.

        T = Trans(translation) . Trans(p0) . Rot . Trans(-p0) . nominal

    with p0 the nominal target position and Rot about WORLD axes. Rotation
    therefore turns the target's orientation in place and never moves it; only
    the declared translation does.
    """
    from scipy.spatial.transform import Rotation
    nominal_RG = np.asarray(nominal_RG, dtype=float)
    centre = nominal_RG[:3, 3]
    rotation = frames.make_transform(
        rotation=Rotation.from_euler('xyz', placement['rotation_deg'],
                                     degrees=True).as_matrix(),
        translation=np.zeros(3))
    return (frames.make_transform(rotation=np.eye(3),
                                  translation=np.asarray(placement['translation'])
                                  + centre)
            @ rotation
            @ frames.make_transform(rotation=np.eye(3), translation=-centre)
            @ nominal_RG)
