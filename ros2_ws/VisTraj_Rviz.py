import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file_path = os.path.expanduser('~/ros2_ws/ur10e.urdf')
    
    with open(urdf_file_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # 1. Publish UR10e URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # 2. Static Transform: map -> world
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
        ),
        
        # 3. Execute MATLAB Trajectory Player Script Directly
        ExecuteProcess(
            cmd=['python3', os.path.expanduser('~/ros2_ws/joint_state_node.py')],
            output='screen'
        ),
        
        # 4. Launch RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
