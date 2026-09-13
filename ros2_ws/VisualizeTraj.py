
# VisualizeTraj.py
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Path to your generated URDF
    urdf_file_path = os.path.expanduser('~/ros2_ws/ur10e.urdf')
    
    with open(urdf_file_path, 'r') as f:
        robot_desc = f.read()

    # 1b. Locate ur_description's install location via the ROS package index
    #     (robust to wherever it actually got colcon-built), then hand Gazebo
    #     its PARENT dir so 'model://ur_description/...' URIs resolve.
    #     get_package_share_directory returns .../share/ur_description,
    #     so its dirname is .../share -- the directory Gazebo needs to search.
    ur_description_share = get_package_share_directory('ur_description')
    resource_path_parent = os.path.dirname(ur_description_share)

    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    existing_ign_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    new_gz_path = os.pathsep.join(filter(None, [existing_gz_path, resource_path_parent]))
    new_ign_path = os.pathsep.join(filter(None, [existing_ign_path, resource_path_parent]))

    set_gz_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', new_gz_path)
    # Also set the Fortress-era var name, since this container has 'ign' not 'gz'
    set_ign_resource_path = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', new_ign_path)

    # 2. Launch Gazebo server directly via the 'ign' binary, wrapped in xvfb-run.
    #    Bypasses ros_gz_sim's gz_sim.launch.py, which in this container's
    #    package version constructs an invalid 'ign gazebo sim ...' command
    #    (it assumes the newer 'gz sim' CLI style, but only 'ign' is installed).
    #    xvfb-run gives Ogre2 a virtual (software-only) X display so the camera
    #    sensor can render, without needing a GPU, host display, or SSH forwarding.
    #    world_file assumes empty_camera.sdf lives alongside this launch file
    #    at the root of your mounted workspace -- adjust the path if you put it
    #    somewhere else.
    world_file = os.path.expanduser('~/ros2_ws/empty_camera.sdf')
    gz_sim = ExecuteProcess(
        cmd=['xvfb-run', '-a', 'ign', 'gazebo', '-s', '-r', world_file],
        output='screen'
    )

    # 3. Spawn Node to load the UR10e into Gazebo
    spawn_ur10e = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc,
            '-name', 'ur10e',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,
        gz_sim,
        spawn_ur10e
    ])
