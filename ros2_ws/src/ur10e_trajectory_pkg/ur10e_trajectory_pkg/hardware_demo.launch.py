"""Execute a certified plan through ROS, with the plan shown beside the robot.

  hardware:=fake   (default) fake_hardware integrates the commands; nothing
                   leaves this machine. Rehearse here first.
  hardware:=real   ur_bridge (UR10e :30003) and rail_bridge (Parker :5002/5003).
                   Commands are only sent with enable_commands:=true; without
                   it the bridges run in shadow mode (read state, log commands).

RViz shows the measured robot (/joint_states) solid and the plan reference
(/plan/joint_reference, published under the sim/ TF prefix) translucent.
gazebo:=true also drives the headless Gazebo model from the plan reference.

Operate the executor with (see plan_executor for the full ~/start sequence):
  ros2 service call /plan_executor/start std_srvs/srv/Trigger
      straighten arm -> home rail (if needed) -> rail to plan home -> plan
  ros2 service call /plan_executor/move_to_home std_srvs/srv/Trigger
  ros2 service call /plan_executor/stop std_srvs/srv/Trigger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = (
    ('plan', '', 'certified best_plan.json (required)'),
    ('urdf', os.path.expanduser('~/ros2_ws/ur10e.urdf'),
     'robot URDF; the default is the container path, pass <clone>/ros2_ws/ur10e.urdf natively'),
    ('csv', '', "recording the plan was certified on; default: the plan's recorded path"),
    ('hardware', 'fake', 'fake or real'),
    ('enable_commands', 'false', 'real hardware only: actually send commands'),
    ('mode', 'warmup', "warmup (home -> task start) or full (warmup + task)"),
    ('time_scale', '0.1', 'uniform slow-down of the plan, in (0, 1]'),
    ('kp', '0.0', 'position correction gain, 1/s; 0 = feed-forward only'),
    ('rate_hz', '20.0', 'command rate, as in the Simulink model'),
    ('arm_speed_limit', '0.5', 'rad/s, executor and ur_bridge'),
    ('rail_speed_limit', '0.05', 'm/s, executor and rail_bridge'),
    ('arm_abort_tolerance', '0.02', 'rad: halt when any arm joint lags the plan by more'),
    ('rail_abort_tolerance', '0.01', 'm: halt when the rail lags the plan by more'),
    ('speed_scaling_check', 'true', 'refuse/halt unless the UR runs at 100% speed scaling'),
    ('ur_ip', '192.168.7.8', 'UR10e controller'),
    ('ur_stream', 'program', 'program (one persistent URScript program) or lines '
     '(a speedj program per command, as Simulink did)'),
    ('ur_host_ip', '', "this PC's address on the robot network; empty = detect"),
    ('ur_stream_port', '50010', 'TCP port the streaming program connects back to'),
    ('rail_ip', '192.168.7.6', 'Parker rail controller'),
    ('rail_offset_m', '0.0', 'planner rail coordinate of the homed controller zero'),
    ('rail_sign', '1.0', '+1 if JOG FWD moves toward +x in the URDF, else -1'),
    ('rail_units_per_mm', '1.0', 'controller JOG VEL units per mm'),
    ('rail_terminator', 'cr', 'rail line ending: cr, crlf, or lfcr (Simulink used lfcr)'),
    ('rail_soft_min_m', '0.3', 'rail soft limit, planner metres'),
    ('rail_soft_max_m', '2.7', 'rail soft limit, planner metres'),
    ('home_rail', 'if_needed', 'if_needed, always, or never: AXIS0 JOG HOME -1 inside ~/start'),
    ('rail_homing_speed', '0.025', 'm/s, the jog speed set before homing'),
    ('fake_rail_homed', 'true', 'fake hardware only: start with the rail already homed'),
    ('fake_speed_scaling', '1.0', 'fake hardware only: arm speed fraction, like the UR slider'),
    ('fake_initial_offset', '[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
     'fake hardware only: start pose minus plan home, 7 floats'),
    ('rviz', 'true', 'start RViz'),
    ('gazebo', 'false', 'also drive the Gazebo model from the plan reference'),
)


def generate_launch_description():
    config = {name: LaunchConfiguration(name) for name, _, _ in ARGUMENTS}
    fake = LaunchConfigurationEquals('hardware', 'fake')
    real = LaunchConfigurationEquals('hardware', 'real')
    share = get_package_share_directory('ur10e_trajectory_pkg')
    robot_description = ParameterValue(Command(['cat ', config['urdf']]), value_type=str)

    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=text)
          for name, default, text in ARGUMENTS],

        # Measured robot.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        # Plan reference as a second robot under the sim/ prefix, overlaid on
        # the first by an identity world -> sim/world transform.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='sim',
             parameters=[{'robot_description': robot_description, 'frame_prefix': 'sim/'}],
             remappings=[('joint_states', '/plan/joint_reference')], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'world', '--child-frame-id', 'sim/world']),

        Node(package='ur10e_trajectory_pkg', executable='plan_executor', output='screen',
             parameters=[{
                 'plan': config['plan'], 'csv': config['csv'], 'mode': config['mode'],
                 'time_scale': config['time_scale'], 'kp': config['kp'],
                 'rate_hz': config['rate_hz'],
                 'arm_speed_limit': config['arm_speed_limit'],
                 'rail_speed_limit': config['rail_speed_limit'],
                 'home_rail': config['home_rail'],
                 'arm_abort_tolerance': config['arm_abort_tolerance'],
                 'rail_abort_tolerance': config['rail_abort_tolerance'],
                 'speed_scaling_check': config['speed_scaling_check'],
             }]),
        Node(package='ur10e_trajectory_pkg', executable='joint_state_merger', output='screen'),

        Node(package='ur10e_trajectory_pkg', executable='fake_hardware', output='screen',
             condition=fake, parameters=[{
                 'plan': config['plan'],
                 'start_homed': config['fake_rail_homed'],
                 'speed_scaling': config['fake_speed_scaling'],
                 'initial_offset': config['fake_initial_offset'],
                 'rail_switch_m': config['rail_offset_m'],
                 'homing_speed': config['rail_homing_speed'],
             }]),
        Node(package='ur10e_trajectory_pkg', executable='ur_bridge', output='screen',
             condition=real, parameters=[{
                 'robot_ip': config['ur_ip'],
                 'stream': config['ur_stream'],
                 'host_ip': config['ur_host_ip'],
                 'stream_port': config['ur_stream_port'],
                 'enable_commands': config['enable_commands'],
                 'max_joint_speed': config['arm_speed_limit'],
             }]),
        Node(package='ur10e_trajectory_pkg', executable='rail_bridge', output='screen',
             condition=real, parameters=[{
                 'host': config['rail_ip'],
                 'enable_commands': config['enable_commands'],
                 'max_speed': config['rail_speed_limit'],
                 'rail_offset_m': config['rail_offset_m'],
                 'rail_sign': config['rail_sign'],
                 'units_per_mm': config['rail_units_per_mm'],
                 'terminator': config['rail_terminator'],
                 'homing_speed': config['rail_homing_speed'],
                 'soft_min_m': config['rail_soft_min_m'],
                 'soft_max_m': config['rail_soft_max_m'],
             }]),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(share, 'rviz', 'hardware_demo.rviz')],
             condition=IfCondition(config['rviz'])),
        Node(package='ur10e_trajectory_pkg', executable='obstacle_markers', output='screen',
             condition=IfCondition(config['rviz'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'VisualizeTraj.py')),
            launch_arguments={'joint_topic': '/plan/joint_reference'}.items(),
            condition=IfCondition(config['gazebo'])),
    ])
