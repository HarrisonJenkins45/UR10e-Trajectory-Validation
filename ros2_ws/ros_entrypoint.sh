#!/bin/bash
set -e

# Source Virtual Environment
source /opt/ros_venv/bin/activate

# Source ROS 2 Base
source /opt/ros/humble/setup.bash

# Source Local Workspace (if built)
if [ -f "/root/ros2_ws/install/setup.bash" ]; then
    source "/root/ros2_ws/install/setup.bash"
fi

exec "$@"
