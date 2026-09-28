#!/usr/bin/env python3
"""ROS <-> Parker rail, replacing the Simulink udpPARKERBlock and rail Subsystem.

State:    subscribes to the controller's UDP fast status (port 5003) and
          publishes linear_rail_joint on /rail/joint_state, in planner metres.
          Polls the home-found flag (PRINT BIT 16134, read-only) and publishes
          it on /rail/homed (latched).
Command:  forwards /rail/velocity_command (Float64, m/s along the URDF rail)
          to TCP 5002 as "AXIS0 JOG VEL %05.1f:AXIS0 JOG FWD|REV", the line the
          Simulink model sent at 20 Hz. Zero requests AXIS0 JOG OFF.
Homing:   ~/home sends "AXIS0 JOG VEL <homing_speed>:AXIS0 JOG HOME -1", so the
          controller homes at a bounded jog speed, and supervises it: homing
          completes when the home-found flag is set and the carriage has
          stopped; it is aborted (JOG OFF) on timeout, stale feedback, or any
          zero command such as the executor's ~/stop. Soft limits cannot apply
          while homing, because positions mean nothing until it completes.
          The executor only requests homing with the arm at the plan's home
          posture.

Prerequisites (done by the operator, not here): drive enabled, FSTAT0(48,2),
FSTAT1(48,58) and FSTAT ON configured. See TERMINAL_SOCKET.md on the
hardware_movement branch.

Calibration: after homing, rail_offset_m is the planner coordinate of
controller position 0 and rail_sign is +1 if JOG FWD moves toward +x in the
URDF. The executor's start check compares this reported position with the
plan's home, so a wrong offset refuses to start instead of moving.

Safety behaviour:
  * enable_commands is false by default: shadow mode sends no motion commands
    and only logs them. Read-only PRINT queries are still sent.
  * commands faster than max_speed, or moving further past the soft travel
    limits, send JOG OFF and latch a fault until ~/reset_fault.
  * a stale command stream or stale feedback while moving sends JOG OFF.
  * a controller known not to be homed refuses jog commands.
JOG OFF decelerates at the controller's configured rate; it does not disable
the drive. Keep the physical stop within reach.
"""

import socket
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger

from ur10e_trajectory_pkg import hardware_protocol as wire
from ur10e_trajectory_pkg.configurations import JOINT_NAMES, RAIL_INDEX
from ur10e_trajectory_pkg.ros_params import declare

RAIL_JOINT = JOINT_NAMES[RAIL_INDEX]
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Homing counts as finished only after this long, so a home-found flag left
# over from an earlier homing cannot end a new one before it has started.
HOMING_MIN_S = 1.0
STOPPED_M_S = 0.0005
# Fast-status subscription renewal: periodically, and whenever the stream
# has been quiet this long (it normally arrives at about 100 Hz).
RESUBSCRIBE_S = 5.0
QUIET_S = 0.1


class RailBridge(Node):

    def __init__(self):
        super().__init__('rail_bridge')
        self.host = declare(self, 'host', '192.168.7.6')
        self.command_port = declare(self, 'command_port', wire.RAIL_COMMAND_PORT)
        self.feedback_port = declare(self, 'feedback_port', wire.RAIL_FEEDBACK_PORT)
        self.enable_commands = declare(self, 'enable_commands', False)
        terminator = declare(self, 'terminator', 'cr')
        self.terminator = wire.RAIL_TERMINATORS[terminator]
        self.calibration = wire.RailCalibration(
            offset_m=declare(self, 'rail_offset_m', 0.0),
            sign=declare(self, 'rail_sign', 1.0),
            units_per_mm=declare(self, 'units_per_mm', 1.0))
        self.max_speed = declare(self, 'max_speed', 0.05)
        self.soft_min = declare(self, 'soft_min_m', 0.3)
        self.soft_max = declare(self, 'soft_max_m', 2.7)
        self.command_timeout = declare(self, 'command_timeout', 0.25)
        self.feedback_timeout = declare(self, 'feedback_timeout', 0.25)
        self.homing_speed = declare(self, 'homing_speed', 0.025)
        self.homing_direction = declare(self, 'homing_direction', -1)
        self.homing_timeout = declare(self, 'homing_timeout', 240.0)
        if not self.soft_min < self.soft_max:
            raise ValueError('soft_min_m must be below soft_max_m')
        if not 0 < self.homing_speed <= self.max_speed:
            raise ValueError('homing_speed must be positive and at most max_speed')

        self.state_pub = self.create_publisher(JointState, '/rail/joint_state', 10)
        self.homed_pub = self.create_publisher(Bool, '/rail/homed', LATCHED)
        self.create_subscription(Float64, '/rail/velocity_command', self.on_command, 10)
        self.create_service(Trigger, '~/reset_fault', self.on_reset)
        self.create_service(Trigger, '~/home', self.on_home)
        self.create_timer(0.02, self.watchdog)

        self.lock = threading.Lock()
        self.position = None
        self.measured_velocity = 0.0
        self.received_at = 0.0
        self.tcp = None
        self.moving = False
        self.velocity = 0.0
        self.homing_since = None
        self.homed = None
        self.fault = None
        self.last_command = 0.0
        self.running = True
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.settimeout(0.05)
        # No SO_REUSEADDR: fail loudly if another listener owns port 5003.
        self.udp.bind(('0.0.0.0', self.feedback_port))
        threading.Thread(target=self.read_feedback, daemon=True).start()
        threading.Thread(target=self.poll_homed, daemon=True).start()
        mode = 'COMMANDS ENABLED' if self.enable_commands else 'shadow mode (no motion sent)'
        self.get_logger().info(
            f'Rail bridge to {self.host}, {mode}; offset {self.calibration.offset_m} m, '
            f'sign {self.calibration.sign:+.0f}, soft limits '
            f'[{self.soft_min}, {self.soft_max}] m, max {self.max_speed} m/s')

    # --- state -------------------------------------------------------------

    def subscribe(self):
        try:
            self.udp.sendto(wire.RAIL_SUBSCRIBE, (self.host, self.feedback_port))
        except OSError as error:
            self.get_logger().warn(f'rail subscribe failed: {error}', throttle_duration_sec=5.0)
        return time.monotonic()

    def read_feedback(self):
        # The fast-status stream has been seen to stop on its own (see the
        # rail_protocol captures). Renew the subscription periodically, and at
        # once when the ~100 Hz stream goes quiet, so a lapse costs a few
        # hundred milliseconds rather than a stale-feedback halt.
        last_packet, subscribed = time.monotonic(), 0.0
        while self.running:
            now = time.monotonic()
            if now - subscribed > RESUBSCRIBE_S or (
                    now - last_packet > QUIET_S and now - subscribed > QUIET_S):
                if now - last_packet > QUIET_S:
                    self.get_logger().warn(
                        f'rail feedback quiet for {now - last_packet:.2f} s; resubscribing',
                        throttle_duration_sec=2.0)
                subscribed = self.subscribe()
            try:
                data, peer = self.udp.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                if self.running:
                    time.sleep(0.2)
                continue
            if peer[0] != self.host:
                continue
            try:
                counts, velocity = wire.parse_rail_feedback(data)
            except ValueError as error:
                self.get_logger().warn(f'bad rail packet: {error}', throttle_duration_sec=2.0)
                continue
            last_packet = time.monotonic()
            position = self.calibration.position_m(counts)
            velocity_m_s = self.calibration.velocity_m_s(velocity)
            with self.lock:
                self.position = position
                self.measured_velocity = velocity_m_s
                self.received_at = last_packet
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = [RAIL_JOINT]
            message.position = [position]
            message.velocity = [velocity_m_s]
            self.state_pub.publish(message)

    def feedback(self):
        """(position, velocity) if fresh, else None."""
        with self.lock:
            if self.position is None or time.monotonic() - self.received_at > self.feedback_timeout:
                return None
            return self.position, self.measured_velocity

    def query_homed(self):
        """Read the home-found flag on a short-lived connection of its own."""
        try:
            with socket.create_connection((self.host, self.command_port), timeout=1.0) as sock:
                sock.settimeout(0.3)
                sock.sendall(wire.RAIL_HOMED_QUERY + self.terminator)
                reply = b''
                deadline = time.monotonic() + 1.5
                while time.monotonic() < deadline:
                    try:
                        chunk = sock.recv(4096)
                    except socket.timeout:
                        if reply:
                            break
                        continue
                    if not chunk:
                        break
                    reply += chunk
            homed = wire.parse_rail_bit_reply(reply.decode('ascii', 'replace'))
            if homed is None:
                self.get_logger().warn(f'unreadable home-flag reply {reply!r}',
                                       throttle_duration_sec=30.0)
            return homed
        except OSError as error:
            self.get_logger().warn(f'home-flag query failed: {error}', throttle_duration_sec=30.0)
            return None

    def poll_homed(self):
        while self.running:
            homing = self.homing_since is not None
            # Poll only when idle or homing: the query is a second connection
            # and should not interleave with a live jog stream.
            if homing or not self.moving:
                homed = self.query_homed()
                if homing:
                    self.check_homing_complete(homed)
                elif homed is not None and homed != self.homed:
                    self.set_homed(homed)
            time.sleep(0.5 if homing else 3.0)

    def set_homed(self, homed):
        self.homed = homed
        self.homed_pub.publish(Bool(data=bool(homed)))
        self.get_logger().info(f'rail home-found flag: {homed}')

    # --- commands ----------------------------------------------------------

    def send(self, line):
        """Send one command line; return False, never raise, on failure."""
        if not self.enable_commands:
            self.get_logger().info(f'[shadow] {line.decode()}', throttle_duration_sec=1.0)
            return True
        try:
            if self.tcp is None:
                sock = socket.create_connection((self.host, self.command_port), timeout=1.0)
                threading.Thread(target=self.read_replies, args=(sock,), daemon=True).start()
                self.tcp = sock
            self.tcp.sendall(line + self.terminator)
            return True
        except OSError as error:
            self.close_tcp()
            self.get_logger().error(f'rail command send failed: {error}')
            return False

    def read_replies(self, sock):
        pending = b''
        try:
            while True:
                data = sock.recv(4096)
                if not data:
                    break
                pending = (pending + data)[-4096:]
                if any(word in pending.lower() for word in (b'error', b'invalid', b'unknown')):
                    self.get_logger().error(f'rail controller reply: {pending!r}')
                    pending = b''
        except OSError:
            pass

    def stop(self):
        self.moving = False
        self.homing_since = None
        if not self.send(wire.RAIL_STOP):
            self.get_logger().error('RAIL STOP DELIVERY FAILED: use the physical stop')

    def trip(self, reason):
        if self.fault is None:
            self.fault = reason
            self.get_logger().error(f'rail bridge fault: {reason}')
        self.stop()

    def on_command(self, message):
        velocity = float(message.data)
        if self.homing_since is not None:
            if velocity == 0.0:
                self.get_logger().warn('homing aborted by a stop command')
                self.stop()
            else:
                self.get_logger().warn('ignoring jog command while homing',
                                       throttle_duration_sec=2.0)
            return
        if self.fault:
            self.get_logger().warn(f'ignoring command, fault latched: {self.fault}',
                                   throttle_duration_sec=2.0)
            return
        if not abs(velocity) <= self.max_speed:
            self.trip(f'rail command {velocity:.4f} m/s exceeds {self.max_speed} m/s')
            return
        self.last_command = time.monotonic()
        units = self.calibration.controller_velocity(velocity)
        if abs(units) < wire.RAIL_MIN_SPEED_UNITS:
            if self.moving:
                self.stop()
            return
        if self.homed is False:
            self.trip('rail is not homed; positions are meaningless')
            return
        state = self.feedback()
        if state is None:
            self.trip('no fresh rail feedback; refusing to move')
            return
        if self.beyond_soft_limit(state[0], velocity):
            self.trip(f'rail at {state[0]:.4f} m, outside soft limits in the commanded direction')
            return
        self.moving = True
        self.velocity = velocity
        if not self.send(wire.rail_jog_command(units)):
            self.trip('jog send failed')

    def on_home(self, request, response):
        if not self.enable_commands:
            response.success, response.message = False, 'commands disabled (shadow mode)'
        elif self.fault:
            response.success, response.message = False, f'fault latched: {self.fault}'
        elif self.moving or self.homing_since is not None:
            response.success, response.message = False, 'rail is already moving'
        elif self.feedback() is None:
            response.success, response.message = False, 'no fresh rail feedback'
        else:
            units = abs(self.calibration.controller_velocity(self.homing_speed))
            line = wire.rail_home_command(units, int(self.homing_direction))
            self.set_homed(False)
            self.homing_since = time.monotonic()
            self.moving = True
            if self.send(line):
                response.success = True
                response.message = f'homing at {self.homing_speed} m/s'
                self.get_logger().info(f'{response.message}: {line.decode()}')
            else:
                self.trip('homing command send failed')
                response.success, response.message = False, 'homing command send failed'
        return response

    def check_homing_complete(self, homed):
        started = self.homing_since
        state = self.feedback()
        if started is None or state is None or homed is not True:
            return
        if time.monotonic() - started >= HOMING_MIN_S and abs(state[1]) < STOPPED_M_S:
            self.homing_since = None
            self.moving = False
            self.set_homed(True)
            self.get_logger().info(f'homing complete; rail at {state[0]:.4f} m')

    def watchdog(self):
        if not self.moving:
            return
        state = self.feedback()
        if state is None:
            self.trip('rail feedback stale while moving')
            return
        if self.homing_since is not None:
            if time.monotonic() - self.homing_since > self.homing_timeout:
                self.trip(f'homing did not finish within {self.homing_timeout} s')
            return
        if time.monotonic() - self.last_command > self.command_timeout:
            self.get_logger().warn('rail command stream stopped; sending JOG OFF')
            self.stop()
        elif self.beyond_soft_limit(state[0], self.velocity):
            self.trip(f'rail reached soft limit at {state[0]:.4f} m')

    def beyond_soft_limit(self, position, velocity):
        """Moving further out of the soft range; moving back in is allowed."""
        return ((position <= self.soft_min and velocity < 0) or
                (position >= self.soft_max and velocity > 0))

    def on_reset(self, request, response):
        self.fault = None
        response.success = True
        response.message = 'rail bridge fault cleared'
        return response

    def close_tcp(self):
        if self.tcp is not None:
            try:
                self.tcp.close()
            finally:
                self.tcp = None

    def shutdown(self):
        self.running = False
        try:
            if self.moving or self.tcp is not None:
                self.stop()
            time.sleep(0.1)
        finally:
            self.close_tcp()
            try:
                self.udp.sendto(wire.RAIL_UNSUBSCRIBE, (self.host, self.feedback_port))
            except OSError:
                pass
            self.udp.close()


def main(args=None):
    rclpy.init(args=args)
    node = RailBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # A signal can shut ROS down mid-spin; only real errors propagate.
        if rclpy.ok():
            raise
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
