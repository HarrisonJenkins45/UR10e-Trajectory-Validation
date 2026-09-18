#!/usr/bin/env python3
"""Capture the environment a validation result was produced in.

Validation verdicts depend on solver behaviour, so the same trajectory can
pass under one set of library versions and fail under another with no source
change. Any measurement worth keeping therefore has to record what produced
it. Call `describe()` and log it alongside results.

The pins themselves live in ros2_ws/requirements.txt, which is the single
source of truth; test_environment.py checks the running environment against
that file rather than against a second copy of the numbers.
"""
import importlib.metadata as metadata
import platform
import subprocess
from pathlib import Path

# Packages whose version can change a validation verdict: kinematics, the
# collision backend, and the numerics underneath both.
TRACKED_PACKAGES = (
    'numpy',
    'scipy',
    'roboticstoolbox-python',
    'spatialmath-python',
    'spatialgeometry',
    'pybullet',
    'pandas',
    'matplotlib',
)

# Path of the UR description submodule, relative to the workspace root. Its
# meshes drive both the rendered model and the collision geometry, so its
# revision belongs in the record with the library versions.
UR_DESCRIPTION_PATH = 'src/Universal_Robots_ROS2_Description'


def workspace_root():
    """Return the ros2_ws directory, found by walking up from this file."""
    # .../ros2_ws/src/ur10e_trajectory_pkg/ur10e_trajectory_pkg/this.py
    return Path(__file__).resolve().parents[3]


def package_versions():
    """Installed version of every tracked package, or None if absent."""
    versions = {}
    for name in TRACKED_PACKAGES:
        try:
            versions[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def ur_description_revision():
    """Commit of the UR description submodule, or None outside a checkout.

    Reported rather than asserted: a source tarball has no git metadata, and
    that should degrade the record rather than fail the run.
    """
    path = workspace_root() / UR_DESCRIPTION_PATH
    if not path.is_dir():
        return None
    try:
        # safe.directory is set because the test container runs as root over
        # a bind-mounted checkout owned by the host user, which git otherwise
        # refuses to read. This only ever reads a revision.
        result = subprocess.run(
            ['git', '-c', f'safe.directory={path}',
             '-C', str(path), 'rev-parse', 'HEAD'],
            capture_output=True, text=True, timeout=10, check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() or None


def parse_requirements(path=None):
    """Read requirements.txt into {package: pinned_version}.

    Only `name==version` lines are returned. Comments carry the packages
    pinned by the base image instead, which pip does not manage.
    """
    path = Path(path) if path else workspace_root() / 'requirements.txt'
    pins = {}
    for raw in path.read_text(encoding='utf-8').splitlines():
        line = raw.split('#', 1)[0].strip()
        if '==' not in line:
            continue
        name, _, version = line.partition('==')
        pins[name.strip()] = version.strip()
    return pins


def describe():
    """Full environment record, suitable for logging beside a result."""
    return {
        'python': platform.python_version(),
        'platform': platform.platform(),
        'packages': package_versions(),
        'ur_description': ur_description_revision(),
    }


def format_description(record=None):
    """Render describe() as lines for a log or an experiment header."""
    record = record or describe()
    width = max(len(n) for n in TRACKED_PACKAGES) + 2
    lines = [
        f"{'python':<{width}}{record['python']}",
        f"{'platform':<{width}}{record['platform']}",
    ]
    for name, version in sorted(record['packages'].items()):
        lines.append(f'{name:<{width}}{version or "NOT INSTALLED"}')
    revision = record['ur_description']
    lines.append(
        f'{"ur_description":<{width}}{revision[:12] if revision else "unknown"}'
    )
    return '\n'.join(lines)


if __name__ == '__main__':
    print(format_description())
