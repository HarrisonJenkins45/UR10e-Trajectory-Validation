import os
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    # Path to your URDF (rail + arm, same file used everywhere else)
    urdf_file_path = os.path.expanduser('~/ros2_ws/ur10e.urdf')

    rviz_config_path = os.path.expanduser('~/ros2_ws/src/ur10e_trajectory_pkg/rviz/trajectory_view.rviz')

    with open(urdf_file_path, 'r') as f:
        robot_desc = f.read()
 
    return LaunchDescription([
        # Publishes TF from whatever /joint_states says -- no physics,
        # no gravity, no controller lag. Purely kinematic.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
 
        # No joint_state_publisher_gui this time -- Validate_trajServer.py
        # itself publishes /joint_states once a trajectory request succeeds,
        # so RViz will animate exactly what it streams, with no manual input.

        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
        ),

 
        # RViz only draws what's explicitly published to it -- this shows
        # the wall/floor obstacles that Gazebo would otherwise render for
        # free from the SDF world.
        Node(
            package='ur10e_trajectory_pkg',
            executable='obstacle_markers',
            name='obstacle_marker_publisher',
            output='screen'
        )
    ])
 

# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     urdf_file_path = os.path.expanduser('~/ros2_ws/ur10e.urdf')
#     rviz_config_path = os.path.expanduser('~/ros2_ws/src/ur10e_trajectory_pkg/rviz/trajectory_view.rviz')

#     with open(urdf_file_path, 'r') as f:
#         robot_desc = f.read()

#     return LaunchDescription([
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='screen',
#             parameters=[{'robot_description': robot_desc}]
#         ),

#         # Single rviz2 node, loading your saved config (Fixed Frame='world',
#         # RobotModel + TF + MarkerArray displays already set up).
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             output='screen',
#             arguments=['-d', rviz_config_path]
#         ),

#         # Wall/floor obstacle markers
#         Node(
#             package='ur10e_trajectory_pkg',
#             executable='obstacle_markers',
#             name='obstacle_marker_publisher',
#             output='screen'
#         )
#     ])
