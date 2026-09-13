# Dockerfile
FROM osrf/ros:humble-desktop-full

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

# Order matters: let roboticstoolbox pull in its own numpy/matplotlib/spatialma>
# versions first, then pin numpy back down afterward, or its C-extensions
# (compiled against numpy 1.x) will crash exactly like you saw.
RUN pip install --no-cache-dir roboticstoolbox-python && \
    pip install --no-cache-dir "numpy<2" && \
    pip install --no-cache-dir pandas && \
    pip install --no-cache-dir pybullet

WORKDIR /root/ros2_ws

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc






