import sys
if sys.prefix == '/home/hjenkins33/miniconda3/envs/ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hjenkins33/ros2_ws/src/package1/install/package1'
