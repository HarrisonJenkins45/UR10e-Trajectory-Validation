#!/usr/bin/env python3
"""ROS <-> Parker rail, replacing the Simulink udpPARKERBlock and rail Subsystem.

State:    subscribes to the controller's UDP fast status (port 5003) and
          publishes linear_rail_joint on /rail/joint_state, in planner metres,
          with the velocity measured from the encoder counts. Polls the
          home-found flag (PRINT BIT 16134) and publishes it on /rail/homed
          (latched).
Command:  forwards /rail/velocity_command (Float64, m/s along the URDF rail)
          to TCP 5002 as "AXIS0 JOG VEL %05.1f:AXIS0 JOG FWD|REV", the line the
          Simulink model sent at 20 Hz. Zero requests AXIS0 JOG OFF.
Homing:   ~/home sends "AXIS0 JOG VEL <homing_speed>:AXIS0 JOG HOME -1" and
          supervises it: it completes when the home-found flag is set and
          the stop that follows is verified; it is aborted on timeout, stale
          feedback, or any zero command such as the executor's ~/stop.

The rail keeps moving after a jog until the controller has decelerated at
its jog deceleration, and JOG OFF is only a request. So this node never
assumes the rail has stopped. It runs as a small state machine, supervised
from a thread of its own (not the ROS executor, so a stalled callback cannot
hold up a stop):

  stopping  JOG OFF is resent every 0.25 s until the stop is VERIFIED: the
            encoder has been still for 0.3 s and the controller's jog-active
            bit (PRINT BIT 792), read after the stop request, is clear. If
            that takes longer than the deceleration allows, a fault latches
            and, with commands enabled, AXIS0 DRIVE OFF is sent.
  idle      nothing is commanded. Encoder motion (over 1 mm/s, or 1 mm of
            drift) or a set jog-active bit is a fault: something else is
            moving the rail, or a jog survived. The rail is stopped as above.
  jogging   commands flow. Stale commands or feedback, soft limits (with the
            stopping distance), motion against the commanded direction, or
            another node publishing commands, fault and stop. A direction
            reversal first stops, verified, then jogs the other way.
  homing    the controller moves on its own; timeout or stale feedback stop it.

Start-up preflight (again on ~/reset_fault): JOG OFF first, whatever state a
previous process left behind; the fast-status mapping (FSTAT0/1, FSTAT ON);
fresh feedback; a verified stop; version, jog acceleration and deceleration,
home and jog-active flags, all readable. A deceleration that cannot stop the
rail from max_speed within max_stop_distance_m is refused. With commands
enabled the drive is then switched on (AXIS0 DRIVE ON), and on exit it is
switched off again after a verified stop, so an enabled bridge is the only
time the drive is energised (drive_control:=false leaves the drive alone).

Calibration: after homing, rail_offset_m is the planner coordinate of
controller position 0 and rail_sign is +1 if JOG FWD moves toward +x in the
URDF. A wrong sign shows up as motion against the command and faults.

Safety behaviour:
  * enable_commands is false by default: shadow mode sends no motion and
    never switches the drive on. Stops (JOG OFF) and read-only PRINT queries
    are always sent: a stop cannot start motion.
  * faults latch until ~/reset_fault, which re-runs the preflight.
  * only nodes named in allowed_command_nodes may publish commands; a
    leftover `ros2 topic pub` makes the bridge refuse to move.
This node cannot stop the rail if it is killed (SIGKILL), the PC loses
power, or the network fails: keep the physical stop within reach.
"""

import collections
import socket
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

from ur10e_trajectory_pkg import hardware_protocol as wire
from ur10e_trajectory_pkg.configurations import JOINT_NAMES, RAIL_INDEX
from ur10e_trajectory_pkg.ros_params import declare

RAIL_JOINT = JOINT_NAMES[RAIL_INDEX]
COMMAND_TOPIC = '/rail/velocity_command'
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Homing counts as finished only after this long, so a home-found flag left
# over from an earlier homing cannot end a new one before it has started.
HOMING_MIN_S = 1.0
# Fast-status subscription renewal: periodically, and whenever the stream
# has been quiet this long (it normally arrives at about 100 Hz).
RESUBSCRIBE_S = 5.0
QUIET_S = 0.1
# Encoder speed is the position change over this window.
SPEED_WINDOW_S = 0.1
HISTORY_S = 0.5
# Stop verification: still (below STOPPED_M_S) for SETTLE_S, jog bit clear.
STOPPED_M_S = 0.0005
SETTLE_S = 0.3
STOP_RESEND_S = 0.25
# Idle monitor: motion nobody commanded.
IDLE_SPEED_M_S = 0.001
# Motion against the commanded direction, once a jog has had time to start.
OPPOSITE_M_S = 0.001
OPPOSITE_AFTER_S = 0.5
SAFETY_PERIOD_S = 0.02
REPLY_TIMEOUT_S = 1.0
ERROR_WORDS = (b'error', b'invalid', b'unknown')


class PreflightError(Exception):
    pass


class RailBridge(Node):

    def __init__(self):
        super().__init__('rail_bridge')
        self.host = declare(self, 'host', '192.168.7.6')
        self.command_port = declare(self, 'command_port', wire.RAIL_COMMAND_PORT)
        # Local UDP port the feedback arrives on, and the controller's port
        # the subscription is sent to; both 5003 on the rig.
        self.feedback_port = declare(self, 'feedback_port', wire.RAIL_FEEDBACK_PORT)
        self.controller_feedback_port = declare(self, 'controller_feedback_port',
                                                wire.RAIL_FEEDBACK_PORT)
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
        self.drive_control = declare(self, 'drive_control', True)
        self.max_stop_distance = declare(self, 'max_stop_distance_m', 0.15)
        self.idle_drift = declare(self, 'idle_drift_m', 0.001)
        # Used until the deceleration has been read from the controller.
        self.default_stop_timeout = declare(self, 'stop_timeout', 10.0)
        self.require_homed = declare(self, 'require_homed', True)
        self.verify_jog_bit = declare(self, 'verify_jog_bit', True)
        self.configure_fstat = declare(self, 'configure_fstat', True)
        allowed = declare(self, 'allowed_command_nodes', 'plan_executor')
        self.allowed_command_nodes = {n.strip() for n in allowed.split(',') if n.strip()}
        if not self.soft_min < self.soft_max:
            raise ValueError('soft_min_m must be below soft_max_m')
        if not 0 < self.homing_speed <= self.max_speed:
            raise ValueError('homing_speed must be positive and at most max_speed')
        if abs(self.calibration.controller_velocity(self.max_speed)) > wire.RAIL_MAX_SPEED_UNITS:
            raise ValueError('max_speed exceeds what the JOG VEL format can express')

        self.state_pub = self.create_publisher(JointState, '/rail/joint_state', 10)
        self.homed_pub = self.create_publisher(Bool, '/rail/homed', LATCHED)
        # 'ready', or why jog commands would not move the rail; the executor
        # refuses to move anything unless this says 'ready' (or 'homing').
        self.status_pub = self.create_publisher(String, '/rail/status', LATCHED)
        self.published_status = None
        self.create_subscription(Float64, COMMAND_TOPIC, self.on_command, 10)
        self.create_service(Trigger, '~/reset_fault', self.on_reset)
        self.create_service(Trigger, '~/home', self.on_home)
        self.create_timer(0.5, self.check_command_sources)

        # self.lock guards the state below; self.send_lock only the command
        # socket. A thread may take send_lock while holding lock, never the
        # other way round.
        self.lock = threading.RLock()
        self.send_lock = threading.Lock()
        self.state = 'stopping'
        self.fault = None
        self.preflight_done = False
        self.preflight_running = False
        self.preflight_step = 'not started'
        self.position = None
        self.speed = None
        self.received_at = 0.0
        self.history = collections.deque()
        self.stationary_since = None
        self.anchor = None
        self.velocity = 0.0             # commanded m/s while jogging
        self.jog_units = 0.0
        self.jog_since = 0.0
        self.last_command = 0.0
        self.stop_requested_at = time.monotonic()
        self.stop_deadline = self.stop_requested_at + self.default_stop_timeout
        self.last_stop_sent = 0.0
        self.escalated = False
        self.homing_since = None
        self.homing_complete = False
        self.homed = None               # published flag
        self.homed_reading = (None, 0.0)       # latest poll: value, query start
        self.jog_active_reading = (None, 0.0)
        self.decel_m_s2 = None
        self.foreign_publishers = []
        self.tcp = None
        self.running = True
        self.closed = False
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.settimeout(0.05)
        # No SO_REUSEADDR: fail loudly if another listener owns port 5003.
        self.udp.bind(('0.0.0.0', self.feedback_port))

        mode = 'COMMANDS ENABLED' if self.enable_commands else 'shadow mode (no motion sent)'
        self.get_logger().info(
            f'Rail bridge to {self.host}, {mode}; offset {self.calibration.offset_m} m, '
            f'sign {self.calibration.sign:+.0f}, soft limits '
            f'[{self.soft_min}, {self.soft_max}] m, max {self.max_speed} m/s; commands '
            f'accepted from {sorted(self.allowed_command_nodes) or "any node"}')
        # Before anything else: whatever a previous process left jogging.
        self.request_stop('bridge starting')
        self.threads = [threading.Thread(target=target, daemon=True)
                        for target in (self.read_feedback, self.poll_flags, self.safety_loop)]
        for thread in self.threads:
            thread.start()
        self.start_preflight()

    # --- feedback ------------------------------------------------------------

    def subscribe(self):
        try:
            self.udp.sendto(wire.RAIL_SUBSCRIBE, (self.host, self.controller_feedback_port))
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
                counts, _ = wire.parse_rail_feedback(data)
            except ValueError as error:
                self.get_logger().warn(f'bad rail packet: {error}', throttle_duration_sec=2.0)
                continue
            last_packet = time.monotonic()
            position = self.calibration.position_m(counts)
            with self.lock:
                self.position, self.received_at = position, last_packet
                self.history.append((last_packet, position))
                while self.history and last_packet - self.history[0][0] > HISTORY_S:
                    self.history.popleft()
                self.speed = self.encoder_speed(last_packet, position)
                speed = self.speed
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = [RAIL_JOINT]
            message.position = [position]
            message.velocity = [speed or 0.0]
            self.publish(self.state_pub, message)

    def encoder_speed(self, now, position):
        """m/s over the last SPEED_WINDOW_S of counts, or None until there is one.

        The feedback's velocity word is the jog profile's velocity, not a
        measurement, so it cannot show motion the controller did not plan.
        """
        reference = None
        for stamp, earlier in self.history:
            if now - stamp < SPEED_WINDOW_S:
                break
            reference = (stamp, earlier)
        if reference is None:
            return None
        return (position - reference[1]) / (now - reference[0])

    def feedback(self):
        """(position, speed) if fresh, else None. Speed may be None at first."""
        with self.lock:
            age = time.monotonic() - self.received_at
            if self.position is None or age > self.feedback_timeout:
                return None
            return self.position, self.speed

    # --- controller queries (a short-lived connection of their own) -----------

    def session(self, lines, parse=True):
        """Send lines one at a time; return {line: (value or None, raw reply)}.

        With parse, each reply is read until its number arrives; otherwise
        until the controller has been quiet for 0.3 s. Raises OSError.
        """
        results = {}
        with socket.create_connection((self.host, self.command_port), timeout=1.0) as sock:
            sock.settimeout(0.1)
            for line in lines:
                sock.sendall(line + self.terminator)
                reply, text = b'', line.decode('ascii')
                deadline, quiet_since = time.monotonic() + REPLY_TIMEOUT_S, time.monotonic()
                value = None
                while time.monotonic() < deadline:
                    try:
                        chunk = sock.recv(4096)
                    except socket.timeout:
                        if not parse and reply and time.monotonic() - quiet_since > 0.3:
                            break
                        continue
                    if not chunk:
                        break
                    reply += chunk
                    quiet_since = time.monotonic()
                    if parse:
                        value = wire.parse_rail_number_reply(
                            reply.decode('ascii', 'replace'), text)
                        if value is not None:
                            break
                results[line] = (value, reply)
        return results

    def poll_flags(self):
        """Home-found and jog-active flags, whenever the rail is not jogging."""
        while self.running:
            with self.lock:
                state = self.state
            if state == 'jogging':
                # Keep the controller's attention on the jog stream.
                time.sleep(0.1)
                continue
            lines = [wire.RAIL_HOMED_QUERY]
            if self.verify_jog_bit:
                lines.append(wire.RAIL_JOG_ACTIVE_QUERY)
            started = time.monotonic()
            try:
                replies = self.session(lines)
            except OSError as error:
                self.get_logger().warn(f'rail flag query failed: {error}',
                                       throttle_duration_sec=10.0)
                replies = {}
            homed = self.bit(replies.get(wire.RAIL_HOMED_QUERY))
            jog_active = self.bit(replies.get(wire.RAIL_JOG_ACTIVE_QUERY))
            with self.lock:
                self.homed_reading = (homed, started)
                self.jog_active_reading = (jog_active, started)
                if (self.state == 'idle' and self.preflight_done and homed is not None
                        and homed != self.homed):
                    self.set_homed(homed)
            # Poll fast while a stop or homing needs the flags; when idle,
            # slowly, but wake at once if a stop begins.
            pause_until = time.monotonic() + (0.2 if state in ('stopping', 'homing') else 1.0)
            while self.running and time.monotonic() < pause_until:
                with self.lock:
                    if self.state != state:
                        break
                time.sleep(0.02)

    @staticmethod
    def bit(result):
        if result is None or result[0] not in (-1.0, 0.0):
            return None
        return result[0] == -1.0

    def set_homed(self, homed):
        self.homed = homed
        self.publish(self.homed_pub, Bool(data=bool(homed)))
        self.get_logger().info(f'rail home-found flag: {homed}')

    # --- preflight -------------------------------------------------------------

    def start_preflight(self):
        with self.lock:
            if self.preflight_running:
                return False
            self.preflight_running, self.preflight_done = True, False
        threading.Thread(target=self.preflight, daemon=True).start()
        return True

    def step(self, text):
        self.preflight_step = text
        self.get_logger().info(f'preflight: {text}')

    def preflight(self):
        try:
            if self.configure_fstat:
                self.step('configuring the fast-status feedback (FSTAT)')
                for line, (_, reply) in self.session(wire.RAIL_FSTAT_SETUP, parse=False).items():
                    if any(word in reply.lower() for word in ERROR_WORDS):
                        raise PreflightError(f'{line.decode()} answered {reply!r}')
            self.step('waiting for fresh rail feedback (UDP)')
            if not self.wait_for(lambda: self.feedback() is not None, 3.0):
                raise PreflightError(
                    f'no rail feedback on UDP {self.feedback_port}: check FSTAT and that no '
                    'other program (Simulink, rail_protocol listen) owns the port')
            self.step('verifying the rail is stopped (JOG OFF, encoder still, jog bit clear)')
            if not self.wait_for(lambda: self.state == 'idle' or self.fault,
                                 self.stop_deadline - time.monotonic() + 2.0):
                raise PreflightError('the rail could not be verified stopped')
            if self.fault:
                raise PreflightError(self.fault)
            self.step('reading version, jog acceleration/deceleration and flags')
            replies = self.session([wire.RAIL_JOG_ACCEL_QUERY, wire.RAIL_JOG_DECEL_QUERY,
                                    wire.RAIL_HOMED_QUERY, wire.RAIL_JOG_ACTIVE_QUERY])
            version = self.session([wire.RAIL_VERSION_QUERY], parse=False)
            self.get_logger().info(
                'rail controller: '
                + version[wire.RAIL_VERSION_QUERY][1].decode('ascii', 'replace').strip())
            accel, _ = replies[wire.RAIL_JOG_ACCEL_QUERY]
            decel, raw = replies[wire.RAIL_JOG_DECEL_QUERY]
            if decel is None or decel <= 0:
                raise PreflightError(f'jog deceleration (P12350) unreadable: {raw!r}')
            decel_m_s2 = decel / self.calibration.units_per_mm / 1000.0
            stop_distance = self.max_speed ** 2 / (2.0 * decel_m_s2)
            self.get_logger().info(
                f'jog acceleration {accel}, deceleration {decel} units/s^2: a stop from '
                f'{self.max_speed} m/s takes {self.max_speed / decel_m_s2:.1f} s and '
                f'{stop_distance * 1000:.0f} mm')
            if stop_distance > self.max_stop_distance:
                raise PreflightError(
                    f'jog deceleration {decel} units/s^2 needs {stop_distance * 1000:.0f} mm '
                    f'to stop from {self.max_speed} m/s (limit '
                    f'{self.max_stop_distance * 1000:.0f} mm): raise P12350 on the '
                    'controller, or lower max_speed')
            for query, name in ((wire.RAIL_HOMED_QUERY, 'home-found flag'),
                                (wire.RAIL_JOG_ACTIVE_QUERY, 'jog-active flag')):
                value, raw = replies[query]
                if self.bit((value, raw)) is None and (
                        query == wire.RAIL_HOMED_QUERY or self.verify_jog_bit):
                    raise PreflightError(
                        f'{name} ({query.decode()}) unreadable: {raw!r}. The bridge expects '
                        'the echo, then -1 or 0 on its own line, then a prompt')
            with self.lock:
                if self.state != 'idle' or self.fault:
                    raise PreflightError(self.fault or f'rail {self.state} during preflight')
                self.decel_m_s2 = decel_m_s2
                if self.enable_commands and self.drive_control:
                    self.step('switching the drive on (AXIS0 DRIVE ON)')
                    if not self.send(wire.RAIL_DRIVE_ON, motion=True):
                        raise PreflightError('AXIS0 DRIVE ON could not be sent')
                self.set_homed(self.bit(replies[wire.RAIL_HOMED_QUERY]))
                self.preflight_done = True
            self.step('passed')
        except (PreflightError, OSError) as error:
            self.trip(f'preflight failed: {error}')
        finally:
            with self.lock:
                self.preflight_running = False

    def wait_for(self, condition, timeout):
        deadline = time.monotonic() + max(timeout, 0.0)
        while self.running and time.monotonic() < deadline:
            with self.lock:
                if condition():
                    return True
            time.sleep(0.05)
        with self.lock:
            return bool(condition())

    # --- command socket --------------------------------------------------------

    def send(self, line, motion=True):
        """Send one command line; return False, never raise, on failure.

        Motion lines are only logged in shadow mode; stops always go out. A
        broken connection is replaced once before giving up.
        """
        if motion and not self.enable_commands:
            self.get_logger().info(f'[shadow] {line.decode()}', throttle_duration_sec=1.0)
            return True
        error = None
        with self.send_lock:
            for _ in range(2):
                try:
                    if self.tcp is None:
                        sock = socket.create_connection((self.host, self.command_port),
                                                        timeout=1.0)
                        threading.Thread(target=self.read_replies, args=(sock,),
                                         daemon=True).start()
                        self.tcp = sock
                    self.tcp.sendall(line + self.terminator)
                    return True
                except OSError as caught:
                    error = caught
                    self.close_tcp_locked()
        self.get_logger().error(f'rail command {line.decode()!r} not delivered: {error}')
        return False

    def read_replies(self, sock):
        pending = b''
        try:
            while True:
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    # The socket keeps a timeout so a send can never block
                    # forever; for the reader it only means no reply yet.
                    continue
                if not data:
                    break
                pending = (pending + data)[-4096:]
                if any(word in pending.lower() for word in ERROR_WORDS):
                    self.trip(f'rail controller replied {pending!r}')
                    pending = b''
        except OSError:
            pass
        with self.send_lock:
            lost = self.tcp is sock
            if lost:
                self.close_tcp_locked()
        if lost and self.running:
            with self.lock:
                jogging = self.state in ('jogging', 'homing')
            if jogging:
                self.trip('rail command connection closed while the rail was moving')

    def close_tcp_locked(self):
        if self.tcp is not None:
            try:
                self.tcp.close()
            except OSError:
                pass
            self.tcp = None

    # --- stopping --------------------------------------------------------------

    def stop_timeout(self, speed):
        """How long a stop from `speed` may take before it counts as failed."""
        if self.decel_m_s2 is None:
            return self.default_stop_timeout
        speed = self.max_speed if speed is None else abs(speed)
        return 1.5 * speed / self.decel_m_s2 + 1.0 + SETTLE_S

    def request_stop(self, reason, fault=False):
        """JOG OFF now; the safety loop resends it until the stop is verified."""
        with self.lock:
            now = time.monotonic()
            if fault and self.fault is None:
                self.fault = reason
                self.get_logger().error(f'rail bridge fault: {reason}')
            if self.state != 'stopping':
                self.get_logger().info(f'rail stopping ({reason})')
                feedback = self.feedback()
                self.stop_requested_at = now
                self.stop_deadline = now + self.stop_timeout(
                    None if feedback is None else feedback[1])
                self.escalated = False
                self.state = 'stopping'
            self.jog_units, self.velocity = 0.0, 0.0
            self.homing_since = None
            self.stationary_since = None
            self.last_stop_sent = now
            if not self.send(wire.RAIL_STOP, motion=False):
                self.get_logger().error('JOG OFF not delivered; retrying every '
                                        f'{STOP_RESEND_S} s. Keep the physical stop in reach')

    def trip(self, reason):
        self.request_stop(reason, fault=True)

    # --- supervision -------------------------------------------------------------

    def safety_loop(self):
        while self.running:
            try:
                self.supervise(time.monotonic())
            except Exception as error:  # the supervisor must never die
                self.get_logger().error(f'rail supervisor error: {error!r}')
                try:
                    self.trip(f'supervisor error: {error!r}')
                except Exception:
                    pass
            self.publish_status()
            time.sleep(SAFETY_PERIOD_S)

    def supervise(self, now):
        with self.lock:
            feedback = self.feedback()
            speed = None if feedback is None else feedback[1]
            if speed is not None and abs(speed) < STOPPED_M_S:
                self.stationary_since = self.stationary_since or now
            else:
                self.stationary_since = None
            still = self.stationary_since is not None and now - self.stationary_since >= SETTLE_S
            if self.state == 'stopping':
                self.supervise_stop(now, feedback, still)
            elif self.state == 'idle':
                self.supervise_idle(now, feedback)
            elif self.state == 'jogging':
                self.supervise_jog(now, feedback)
            elif self.state == 'homing':
                self.supervise_homing(now, feedback, still)

    def supervise_stop(self, now, feedback, still):
        if now - self.last_stop_sent >= STOP_RESEND_S:
            self.last_stop_sent = now
            self.send(wire.RAIL_STOP, motion=False)
        jog_active, read_at = self.jog_active_reading
        jog_clear = (not self.verify_jog_bit) or (
            jog_active is False and read_at >= self.stop_requested_at)
        if feedback is not None and still and jog_clear:
            self.state = 'idle'
            self.anchor = feedback[0]
            self.get_logger().info(f'rail stop verified at {feedback[0]:.4f} m')
            if self.homing_complete:
                self.homing_complete = False
                self.set_homed(True)
                self.get_logger().info(f'homing complete; rail at {feedback[0]:.4f} m')
            return
        if now > self.stop_deadline and not self.escalated:
            self.escalated = True
            why = ('rail feedback is stale' if feedback is None
                   else f'rail still moving at {abs(feedback[1] or 0.0) * 1000:.1f} mm/s'
                   if not still else 'the controller still reports a jog active')
            reason = (f'stop not verified {now - self.stop_requested_at:.1f} s after JOG OFF: '
                      f'{why}')
            if self.enable_commands and self.drive_control:
                self.send(wire.RAIL_DRIVE_OFF, motion=False)
                reason += '; sent AXIS0 DRIVE OFF'
            if self.fault is None:
                self.fault = reason
            self.get_logger().error(f'{reason}. USE THE PHYSICAL STOP if the rail moves')

    def supervise_idle(self, now, feedback):
        jog_active, read_at = self.jog_active_reading
        if self.verify_jog_bit and jog_active is True and read_at > self.stop_requested_at:
            self.trip('the controller reports a jog active although none is commanded')
        elif feedback is not None:
            position, speed = feedback
            if speed is not None and abs(speed) > IDLE_SPEED_M_S:
                self.trip(f'rail moving at {speed * 1000:+.1f} mm/s without a command')
            elif self.anchor is not None and abs(position - self.anchor) > self.idle_drift:
                self.trip(f'rail drifted {(position - self.anchor) * 1000:+.1f} mm '
                          'without a command')

    def supervise_jog(self, now, feedback):
        if feedback is None:
            self.trip('rail feedback stale while jogging')
            return
        position, speed = feedback
        if now - self.last_command > self.command_timeout:
            self.trip(f'no rail command for {now - self.last_command:.2f} s while jogging')
        elif self.foreign_publishers:
            self.trip(f'other nodes publish {COMMAND_TOPIC}: {self.foreign_publishers}')
        elif self.beyond_soft_limit(position, self.velocity):
            self.trip(f'rail at {position:.4f} m cannot stop inside the soft limits '
                      f'[{self.soft_min}, {self.soft_max}] m')
        elif (speed is not None and now - self.jog_since > OPPOSITE_AFTER_S
              and abs(self.velocity) >= OPPOSITE_M_S and speed * self.velocity < 0
              and abs(speed) > OPPOSITE_M_S):
            self.trip(f'rail moving at {speed * 1000:+.1f} mm/s against the commanded '
                      f'{self.velocity * 1000:+.1f} mm/s: check rail_sign')

    def supervise_homing(self, now, feedback, still):
        if feedback is None:
            self.trip('rail feedback stale while homing')
            return
        if now - self.homing_since > self.homing_timeout:
            self.trip(f'homing did not finish within {self.homing_timeout} s')
            return
        homed, read_at = self.homed_reading
        if homed is True and read_at >= self.homing_since + HOMING_MIN_S and still:
            # Home found. Finish with a verified stop; the flag is published
            # once the stop is verified, so the executor moves on only then.
            self.homing_complete = True
            self.request_stop('homing complete')

    def beyond_soft_limit(self, position, velocity):
        """Moving out of the soft range, allowing for the distance a stop takes."""
        margin = 0.0 if self.decel_m_s2 is None else velocity ** 2 / (2.0 * self.decel_m_s2)
        return ((velocity < 0 and position - margin <= self.soft_min) or
                (velocity > 0 and position + margin >= self.soft_max))

    def check_command_sources(self):
        """Names of nodes other than the allowed ones publishing rail commands."""
        foreign = []
        if self.allowed_command_nodes:
            try:
                infos = self.get_publishers_info_by_topic(COMMAND_TOPIC)
            except Exception as error:
                self.get_logger().warn(f'cannot list {COMMAND_TOPIC} publishers: {error}',
                                       throttle_duration_sec=10.0)
                infos = []
            foreign = sorted({info.node_name for info in infos
                              if info.node_name not in self.allowed_command_nodes})
        with self.lock:
            if foreign != self.foreign_publishers and foreign:
                self.get_logger().error(
                    f'{COMMAND_TOPIC} is also published by {foreign}; the rail will not move '
                    f'until only {sorted(self.allowed_command_nodes)} publish it')
            self.foreign_publishers = foreign
        return foreign

    # --- status ------------------------------------------------------------------

    def status(self):
        with self.lock:
            if not self.enable_commands:
                return 'shadow mode (enable_commands:=false): rail commands are not sent'
            if self.fault:
                return f'fault: {self.fault}'
            if not self.preflight_done:
                return f'starting: {self.preflight_step}'
            if self.foreign_publishers:
                return f'other nodes publish {COMMAND_TOPIC}: {self.foreign_publishers}'
            if self.state == 'homing':
                return 'homing'
            if self.feedback() is None:
                return 'no fresh rail feedback (UDP 5003)'
            return 'ready'

    def publish_status(self):
        status = self.status()
        if status != self.published_status:
            self.published_status = status
            self.publish(self.status_pub, String(data=status))
            self.get_logger().info(f'rail bridge status: {status}')

    def publish(self, publisher, message):
        try:
            publisher.publish(message)
        except Exception:
            # The ROS context is gone during shutdown; supervision carries on.
            pass

    # --- ROS interface -----------------------------------------------------------

    def on_command(self, message):
        velocity = float(message.data)
        foreign = self.check_command_sources()
        with self.lock:
            now = time.monotonic()
            if not abs(velocity) <= self.max_speed:
                self.trip(f'rail command {velocity:.4f} m/s exceeds {self.max_speed} m/s')
                return
            if self.fault:
                self.get_logger().warn(f'ignoring command, fault latched: {self.fault}',
                                       throttle_duration_sec=2.0)
                return
            units = self.calibration.controller_velocity(velocity)
            if abs(units) < wire.RAIL_MIN_SPEED_UNITS:
                self.last_command = now
                if self.state == 'jogging':
                    self.request_stop('zero command')
                elif self.state == 'homing':
                    self.request_stop('homing aborted by a stop command')
                return
            if foreign:
                self.trip(f'other nodes publish {COMMAND_TOPIC}: {foreign}')
                return
            if not self.preflight_done:
                self.get_logger().warn(f'ignoring jog: preflight not passed '
                                       f'({self.preflight_step})', throttle_duration_sec=2.0)
                return
            if self.state == 'stopping':
                self.get_logger().warn('ignoring jog until the stop is verified',
                                       throttle_duration_sec=2.0)
                return
            if self.state == 'homing':
                self.get_logger().warn('ignoring jog command while homing',
                                       throttle_duration_sec=2.0)
                return
            feedback = self.feedback()
            if feedback is None:
                self.trip('no fresh rail feedback; refusing to move')
                return
            if self.homed is not True and (self.require_homed or self.homed is False):
                self.trip(f'rail home-found flag is {self.homed}; positions are meaningless')
                return
            if self.beyond_soft_limit(feedback[0], velocity):
                self.trip(f'rail at {feedback[0]:.4f} m: jog would leave the soft limits')
                return
            if self.state == 'jogging' and units * self.jog_units < 0:
                self.request_stop('direction reversal: verifying a stop before reversing')
                return
            try:
                line = wire.rail_jog_command(units)
            except ValueError as error:
                self.trip(str(error))
                return
            self.last_command = now
            if self.state == 'idle':
                self.state, self.jog_since = 'jogging', now
            self.jog_units, self.velocity = units, velocity
            if not self.send(line):
                self.trip('jog send failed')

    def on_home(self, request, response):
        with self.lock:
            refusal = None
            if not self.enable_commands:
                refusal = 'commands disabled (shadow mode)'
            elif self.fault:
                refusal = f'fault latched: {self.fault}'
            elif not self.preflight_done:
                refusal = f'preflight not passed ({self.preflight_step})'
            elif self.state != 'idle':
                refusal = f'rail is {self.state}; homing needs a verified stop'
            elif self.feedback() is None:
                refusal = 'no fresh rail feedback'
            elif self.foreign_publishers:
                refusal = f'other nodes publish {COMMAND_TOPIC}: {self.foreign_publishers}'
            if refusal:
                response.success, response.message = False, refusal
                return response
            units = abs(self.calibration.controller_velocity(self.homing_speed))
            line = wire.rail_home_command(units, int(self.homing_direction))
            self.set_homed(False)
            self.state, self.homing_since = 'homing', time.monotonic()
            self.homing_complete = False
            if self.send(line):
                response.success = True
                response.message = f'homing at {self.homing_speed} m/s'
                self.get_logger().info(f'{response.message}: {line.decode()}')
            else:
                self.trip('homing command send failed')
                response.success, response.message = False, 'homing command send failed'
        return response

    def on_reset(self, request, response):
        with self.lock:
            if self.preflight_running:
                response.success, response.message = False, 'preflight already running'
                return response
            previous = self.fault
            self.fault = None
            self.preflight_done = False
            self.request_stop('fault reset')
        self.start_preflight()
        response.success = True
        response.message = (f'fault cleared ({previous}); re-running the preflight: watch for '
                            '"rail bridge status: ready"')
        self.get_logger().info(response.message)
        return response

    def shutdown(self):
        """Stop, verified if possible; drive off; then release the sockets."""
        if self.closed:
            return
        self.closed = True
        try:
            self.request_stop('bridge shutting down')
            deadline = time.monotonic() + min(self.stop_timeout(None), 4.0)
            while time.monotonic() < deadline and self.state != 'idle':
                time.sleep(0.05)
            if self.state != 'idle':
                self.get_logger().error('rail stop NOT verified at exit: use the physical stop')
            # Supervision ends here: releasing the drive may nudge the encoder.
            self.running = False
            if self.enable_commands and self.drive_control:
                if self.send(wire.RAIL_DRIVE_OFF, motion=False):
                    self.get_logger().info('rail drive switched off (AXIS0 DRIVE OFF)')
            time.sleep(0.1)
        finally:
            self.running = False
            # Let the threads finish before the node (and its clock) goes away.
            for thread in self.threads:
                thread.join(timeout=2.0)
            with self.send_lock:
                self.close_tcp_locked()
            try:
                self.udp.sendto(wire.RAIL_UNSUBSCRIBE, (self.host, self.controller_feedback_port))
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
