# Dockerfile
#
# Pinned by digest, not by tag. `humble-desktop-full` moves, and it carries
# scipy, matplotlib and the ROS stack, so a moved tag can change a validation
# verdict with no source change. This digest is the image every measurement
# on this branch was taken in.
#
# To upgrade deliberately:
#   docker buildx imagetools inspect osrf/ros:humble-desktop-full
# then replace the digest below, rebuild, and re-run ./run_tests.sh. Expect
# to update the base-image versions asserted in test_environment.py.
#
# BUILT IMAGE PROVENANCE
#
# Pinning the base by digest does not pin what `apt-get update` fetches below,
# so two builds of this file weeks apart can still differ. The reproducibility
# unit is therefore the BUILT image, not this file. Last verified build:
#
#     sha256:b0935ec83c4bc431b3d354ff6e85c68ac6d7940d4484777d6471ff0bc1578bf2
#     built 2026-09-13, suite 50 passed / 5 xfailed,
#     trajectory baseline 370/500 over three consecutive runs
#
# Reuse that image rather than rebuilding when reproducing a result. If you do
# rebuild, re-run ./run_tests.sh and record the new id here.

FROM osrf/ros@sha256:1db1e4e941d4f77fab55bcd479158a75273329335e86ec468bff280859e8178c

RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-ros-gz-image \
    ros-humble-cv-bridge \
    python3-opencv \
    python3-pip \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

# Exact pins, resolved together in one pip invocation. This previously ran as
# four separate installs in a careful order, because roboticstoolbox pulled in
# its own numpy 2.x and broke its C extensions, which are compiled against
# 1.x. Pinning numpy explicitly removes the ordering dependency; see
# requirements.txt for what is pinned here and what the base image supplies.
COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /root/ros2_ws

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc
