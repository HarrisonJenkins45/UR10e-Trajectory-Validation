#!/usr/bin/env python3
"""Stage 0 gate: the environment is pinned and validation is reproducible.

A validation verdict depends on solver behaviour, so an unpinned library
upgrade can change which trajectories pass with no source change. Nothing
above this layer means anything until the same commit gives the same answer
in a fresh container, so these run before the kinematics tests are trusted.
"""
import numpy as np
import pytest

from ur10e_trajectory_pkg import environment
from ur10e_trajectory_pkg.validation_core import DEFAULT_IK_SEED, TrajectoryValidator

# Pinned by the base image digest rather than by pip, because they arrive
# through the ROS apt packages. Recorded here so drift is still caught.
BASE_IMAGE_PACKAGES = {
    'scipy': '1.8.0',
    'matplotlib': '3.5.1',
}

# The UR description submodule, at release 2.13.0. Its meshes drive both the
# rendered model and the collision geometry, so a different revision can move
# a collision verdict.
EXPECTED_UR_DESCRIPTION = '18e6f603b3ebc2ec479fecb62d6be544b15755e9'


@pytest.fixture(scope='module')
def pins():
    return environment.parse_requirements()


def test_requirements_file_pins_exact_versions(pins):
    """Every requirement is an exact pin, never a range.

    A range reintroduces the problem the file exists to solve, since two
    checkouts of one commit could resolve differently.
    """
    assert pins, 'requirements.txt contained no pins'
    for name, version in pins.items():
        assert version[0].isdigit(), f'{name} is not pinned to a version'


@pytest.mark.parametrize('package', ['numpy', 'roboticstoolbox-python', 'pybullet'])
def test_installed_versions_match_the_pins(pins, package):
    """The running environment is the one the pins describe."""
    installed = environment.package_versions()[package]
    assert installed == pins[package], (
        f'{package} is {installed} but requirements.txt pins {pins[package]}; '
        'rebuild the image rather than running against a drifted environment'
    )


@pytest.mark.parametrize('package,expected', sorted(BASE_IMAGE_PACKAGES.items()))
def test_base_image_packages_have_not_drifted(package, expected):
    """scipy and matplotlib come from the apt base, not pip.

    They are pinned by the base image digest in the Dockerfile. A mismatch
    means the image was rebuilt against a different base.
    """
    assert environment.package_versions()[package] == expected


def test_numpy_is_below_2(pins):
    """roboticstoolbox's C extensions are built against numpy 1.x.

    Against 2.x they fail at import. The pin is load-bearing, not tidiness.
    """
    assert int(pins['numpy'].split('.')[0]) == 1


def test_numpy_satisfies_the_installed_scipy(pins):
    """scipy states a numpy range; the pin has to sit inside it.

    This was violated for the life of the project: scipy 1.8.0 requires numpy
    below 1.25 and the pin was 1.26.4, so every run emitted a compatibility
    warning. Pinning an unsupported pair makes it reproducible, not supported.
    """
    import scipy

    ceiling = getattr(scipy, 'np_maxversion', None)
    if ceiling is None:            # newer scipy stopped publishing a bound
        pytest.skip('installed scipy declares no numpy ceiling')

    def parts(version):
        return tuple(int(p) for p in version.split('.')[:3])

    assert parts(pins['numpy']) < parts(ceiling), (
        f'numpy {pins["numpy"]} is at or above {ceiling}, the ceiling scipy '
        f'{scipy.__version__} declares'
    )


def test_importing_the_stack_emits_no_compatibility_warning():
    """An incompatible pairing must fail the suite, not print a warning.

    Imports run in a subprocess with warnings promoted to errors, because the
    warning fires at first import and would already have been swallowed by the
    time this test body runs.
    """
    import subprocess
    import sys

    probe = (
        'import scipy, numpy, roboticstoolbox, spatialmath, '
        'spatialgeometry, pandas'
    )
    result = subprocess.run(
        [sys.executable, '-W', 'error::UserWarning', '-c', probe],
        capture_output=True, text=True, timeout=180, check=False,
    )
    assert result.returncode == 0, (
        'importing the stack raised a warning promoted to an error:\n'
        f'{result.stderr.strip()[-600:]}'
    )


def test_ur_description_is_at_the_pinned_revision():
    """The mesh source is pinned, or we are not in a git checkout."""
    revision = environment.ur_description_revision()
    if revision is None:
        pytest.skip('not a git checkout; submodule revision unavailable')
    assert revision == EXPECTED_UR_DESCRIPTION


def test_environment_record_is_complete():
    """describe() must name every tracked package, so a logged result says
    what produced it."""
    record = environment.describe()
    assert record['python']
    missing = [n for n, v in record['packages'].items() if v is None]
    assert not missing, f'no version recorded for {missing}'


# --------------------------------------------------------------------------
# Determinism — the actual stage 0 gate
# --------------------------------------------------------------------------

def _validator(**kwargs):
    from ament_index_python.packages import get_package_share_directory

    from test_geometry_invariants import _urdf_path
    return TrajectoryValidator(
        _urdf_path(),
        mesh_base_path=get_package_share_directory('ur_description'),
        **kwargs,
    )


def test_recovery_generator_is_seeded_by_default():
    """Retries must not draw from numpy's global generator.

    Unseeded, three runs of one 500-waypoint file returned 364, 366 and 365
    feasible waypoints. Seeding makes the answer repeatable; it does not make
    it correct, since the retry still cannot leave its solution basin.
    """
    validator = _validator()
    assert validator._seed == DEFAULT_IK_SEED
    assert isinstance(validator._rng, np.random.Generator)


def test_two_validators_draw_identical_sequences():
    first, second = _validator(), _validator()
    np.testing.assert_allclose(first._rng.random(8), second._rng.random(8))


def test_reset_restores_the_initial_sequence():
    """Repeated validations on one long-lived validator must be independent.

    Seeding at construction alone is not enough: generator state would carry
    from one request into the next, so validating the same trajectory twice
    could give two verdicts. The server holds one validator for its lifetime,
    so this is the realistic case, and it is the same failure shape as the
    start pose the server used to inherit from its own playback.
    """
    validator = _validator()
    before = validator._rng.random(8)
    validator._rng.random(32)          # advance, as a validation would
    validator.reset_rng()
    np.testing.assert_allclose(validator._rng.random(8), before)


def test_explicit_seeds_differ_and_none_is_allowed():
    """A caller can still choose exploration over reproducibility."""
    a = _validator(seed=1)._rng.random(8)
    b = _validator(seed=2)._rng.random(8)
    assert not np.allclose(a, b)
    assert _validator(seed=None)._rng is not None


def test_global_numpy_state_is_untouched():
    """Validating must not disturb random state elsewhere in the process."""
    np.random.seed(12345)
    expected = np.random.rand(4)

    np.random.seed(12345)
    validator = _validator()
    validator._rng.random(64)
    validator.reset_rng()
    np.testing.assert_allclose(np.random.rand(4), expected)
