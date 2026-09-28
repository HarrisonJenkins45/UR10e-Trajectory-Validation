#!/usr/bin/env python3
"""Stand-in for ur_bridge and rail_bridge, for rehearsing without hardware.

Integrates the same command topics the bridges consume and publishes the same
state topics they produce, so the executor, merger and RViz run unchanged:

  /ur/joint_velocity_command, /rail/velocity_command  ->  integrate
  /ur/joint_states, /rail/joint_state, /rail/homed    <-  state
  /rail_bridge/home                                    homing, as the bridge

Commands older than command_timeout are treated as zero, like the real
bridges' watchdogs. initial_offset shifts the starting pose away from the
plan's home; speed_scaling < 1 slows the arm like the UR speed slider;
start_homed:=false starts with the rail unreferenced, and homing
then drives it to rail_switch_m (the planner coordinate of the home switch,
i.e. the real bridge's rail_offset_m).
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray
from std_srvs.srv import Trigger

from ur10e_trajectory_pkg import plan_artifact
from ur10e_trajectory_pkg.configurations import JOINT_NAMES, NUM_JOINTS
from ur10e_trajectory_pkg.ros_params import declare

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FakeHardware(Node):

    def __init__(self):
        super().__init__('fake_hardware')
        plan_path = declare(self, 'plan', '')
        offset = declare(self, 'initial_offset', [0.0] * NUM_JOINTS)
        rate_hz = declare(self, 'rate_hz', 125.0)
        self.command_timeout = declare(self, 'command_timeout', 0.25)
        self.homed = declare(self, 'start_homed', True)
        self.rail_switch = declare(self, 'rail_switch_m', 0.0)
        self.homing_speed = declare(self, 'homing_speed', 0.025)
        # Like the UR pendant speed slider: the arm executes this fraction of
        # every commanded velocity; the rail is unaffected.
        self.speed_scaling = declare(self, 'speed_scaling', 1.0)
        start = (plan_artifact.load_plan(plan_path)['home'] if plan_path
                 else np.zeros(NUM_JOINTS))
        self.q = np.asarray(start, dtype=float) + np.asarray(offset, dtype=float)
        self.qd = np.zeros(NUM_JOINTS)
        self.arm_command = (np.zeros(NUM_JOINTS - 1), 0.0)
        self.rail_command = (0.0, 0.0)
        self.homing = False
        self.create_subscription(Float64MultiArray, '/ur/joint_velocity_command',
                                 self.on_arm, 10)
        self.create_subscription(Float64, '/rail/velocity_command', self.on_rail, 10)
        self.arm_pub = self.create_publisher(JointState, '/ur/joint_states', 10)
        self.rail_pub = self.create_publisher(JointState, '/rail/joint_state', 10)
        self.homed_pub = self.create_publisher(Bool, '/rail/homed', LATCHED)
        self.scaling_pub = self.create_publisher(Float64, '/ur/speed_scaling', 10)
        self.homed_pub.publish(Bool(data=self.homed))
        self.create_service(Trigger, '/rail_bridge/home', self.on_home)
        self.period = 1.0 / rate_hz
        self.last_step = time.monotonic()
        self.create_timer(self.period, self.step)
        self.get_logger().info(f'Fake hardware starting at {np.round(self.q, 4).tolist()}, '
                               f'rail homed: {self.homed}')

    def on_arm(self, message):
        velocities = np.asarray(message.data, dtype=float)
        if velocities.shape == (NUM_JOINTS - 1,) and np.all(np.isfinite(velocities)):
            self.arm_command = (velocities, time.monotonic())

    def on_rail(self, message):
        if self.homing and message.data == 0.0:
            self.get_logger().warn('fake homing aborted')
            self.homing = False
        if np.isfinite(message.data) and not self.homing:
            self.rail_command = (float(message.data), time.monotonic())

    def on_home(self, request, response):
        self.homing = True
        self.homed = False
        self.homed_pub.publish(Bool(data=False))
        response.success, response.message = True, 'fake homing started'
        return response

    def step(self):
        now = time.monotonic()
        dt, self.last_step = now - self.last_step, now
        arm, arm_at = self.arm_command
        rail, rail_at = self.rail_command
        self.qd[1:] = self.speed_scaling * arm if now - arm_at <= self.command_timeout else 0.0
        if self.homing:
            self.qd[0] = -self.homing_speed
            if self.q[0] + self.qd[0] * dt <= self.rail_switch:
                self.q[0], self.qd[0] = self.rail_switch, 0.0
                self.homing, self.homed = False, True
                self.homed_pub.publish(Bool(data=True))
        else:
            self.qd[0] = rail if now - rail_at <= self.command_timeout else 0.0
        self.q += self.qd * dt
        stamp = self.get_clock().now().to_msg()
        arm_state = JointState()
        arm_state.header.stamp = stamp
        arm_state.name = list(JOINT_NAMES[1:])
        arm_state.position = self.q[1:].tolist()
        arm_state.velocity = self.qd[1:].tolist()
        self.arm_pub.publish(arm_state)
        self.scaling_pub.publish(Float64(data=self.speed_scaling))
        rail_state = JointState()
        rail_state.header.stamp = stamp
        rail_state.name = [JOINT_NAMES[0]]
        rail_state.position = [float(self.q[0])]
        rail_state.velocity = [float(self.qd[0])]
        self.rail_pub.publish(rail_state)


def main(args=None):
    rclpy.init(args=args)
    node = FakeHardware()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
