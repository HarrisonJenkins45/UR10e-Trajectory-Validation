#!/usr/bin/env bash
# Authoritative test command for this workspace.
#
# Run this rather than invoking pytest or colcon directly, so that results
# quoted in reviews, issues and commit messages all come from the same
# environment. Everything runs inside the pinned container; the host needs
# only docker.
#
#   ./run_tests.sh              build and run the whole suite
#   ./run_tests.sh -k jacobian  pass extra arguments through to pytest
#
# Builds out of tree, under /root inside the container, so the repository
# never accumulates colcon artifacts. Those used to be committed and made a
# fresh checkout fail to build.

set -euo pipefail

IMAGE="${UR10E_IMAGE:-ur10e_sim:latest}"
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Image '$IMAGE' not found. Build it first:" >&2
    echo "    docker build -t ur10e_sim $WORKSPACE" >&2
    exit 1
fi

if [ ! -e "$WORKSPACE/src/Universal_Robots_ROS2_Description/package.xml" ]; then
    echo "The ur_description submodule is empty. Populate it first:" >&2
    echo "    git submodule update --init --recursive" >&2
    exit 1
fi

docker run --rm \
    -v "$WORKSPACE":/root/ros2_ws \
    "$IMAGE" \
    bash -lc '
        # No set -u: the ROS setup scripts reference unbound variables and
        # would abort the run before any test executes.
        set -eo pipefail
        source /opt/ros/humble/setup.bash
        cd /root/ros2_ws
        colcon build --symlink-install \
            --build-base /root/cbuild --install-base /root/cinstall \
            >/dev/null
        source /root/cinstall/setup.bash

        echo "--- environment ---"
        python3 -m ur10e_trajectory_pkg.environment
        echo

        python3 -m pytest src/ur10e_trajectory_pkg/test \
            --ignore=src/ur10e_trajectory_pkg/test/test_flake8.py \
            --ignore=src/ur10e_trajectory_pkg/test/test_pep257.py \
            --ignore=src/ur10e_trajectory_pkg/test/test_copyright.py \
            '"$*"'
    '
