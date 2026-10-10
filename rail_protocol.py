#!/usr/bin/env python3
"""Parker rail feedback, jogging, and stop protocol; Python standard library only."""
import argparse
import json
import math
from pathlib import Path
import signal
import socket
import struct
import threading
import time

HOST = '192.168.7.6'
COUNTS_PER_MM = 26214.4  # Legacy scale, not yet measured here.
SUBSCRIBE = bytes.fromhex('000000010000000a')
UNSUBSCRIBE = bytes(8)
STOP = b'AXIS0 JOG OFF'
TERMINATORS = {'cr': b'\r', 'crlf': b'\r\n', 'lfcr': b'\n\r'}


def parse_packet(data):
    if not 320 <= len(data) <= 2560:
        raise ValueError(f'packet length {len(data)} outside 320..2560')
    words = struct.unpack('>80I', data[:320])
    counts = struct.unpack_from('>i', data)[0]
    velocity = struct.unpack_from('>f', data, 32)[0]
    # Legacy OutSig label only; group 2 is currently unassigned.
    outsig = struct.unpack_from('>f', data, 64)[0]
    if not math.isfinite(velocity):
        raise ValueError('nonfinite velocity field')
    return {'counts': counts, 'position_mm': counts / COUNTS_PER_MM,
            'velocity_raw': velocity, 'outsig_raw': outsig,
            'words': words, 'length': len(data)}


class RailClient:
    """Owns UDP feedback, TCP commands, and an independent command watchdog.

    Motion faults latch until this client is closed. set_velocity renews the
    command lease; retransmission never renews it. Times are monotonic seconds.
    mm conversion must be calibrated before using physical limits for motion.
    """
    def __init__(self, log_path, *, host=HOST, udp_port=5003, tcp_port=5002,
                 bind_port=5003, terminator='cr', command_timeout=0.2,
                 feedback_timeout=0.2):
        if host not in (HOST, '127.0.0.1'):
            raise ValueError('only the rail address or loopback is allowed')
        if not all(math.isfinite(x) and 0 < x <= 0.2
                   for x in (command_timeout, feedback_timeout)):
            raise ValueError('timeouts must be finite, positive, and <= 0.2 s')
        self.host, self.udp_port, self.tcp_port = host, udp_port, tcp_port
        self.bind_port, self.terminator = bind_port, TERMINATORS[terminator]
        self.command_timeout, self.feedback_timeout = command_timeout, feedback_timeout
        self.log_file = open(log_path, 'x', buffering=1)
        self.log_lock = threading.Lock()
        self.lock = threading.RLock()
        self.done = threading.Event()
        self.udp = self.tcp = None
        self.threads = []
        self.latest = None
        self.first_time = None
        self.packet_count = 0
        self.changed_words = set()
        self.previous_words = None
        self.fault = None
        self.active = False
        self.armed = False
        self.last_command = 0.0
        self.deadline = 0.0
        self.bounds = None
        self.start_position = None
        self.velocity = 0.0
        self.units_per_mm = None
        self.closed = False
        self.stop_sent = False
        self.log('session', host=host, terminator=terminator)

    def log(self, event, **fields):
        with self.log_lock:
            self.log_file.write(json.dumps({'wall_time': time.time(),
                'monotonic': time.monotonic(), 'event': event, **fields}) + '\n')

    def _thread(self, fn):
        t = threading.Thread(target=fn, daemon=True)
        self.threads.append(t)
        t.start()

    def subscribe(self):
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.settimeout(0.05)
        # No SO_REUSEADDR: fail if another receiver already owns port 5003.
        self.udp.bind(('0.0.0.0', self.bind_port))
        self.udp.sendto(SUBSCRIBE, (self.host, self.udp_port))
        self.log('udp_tx', hex=SUBSCRIBE.hex())
        self._thread(self._receive_udp)

    def connect(self):
        self.tcp = socket.create_connection((self.host, self.tcp_port), timeout=1.0)
        self.tcp.settimeout(0.05)
        self._thread(self._receive_tcp)
        self._thread(self._watchdog)

    def _receive_udp(self):
        try:
            while not self.done.is_set():
                try:
                    data, peer = self.udp.recvfrom(65535)
                except socket.timeout:
                    continue
                stamp = time.monotonic()
                self.log('udp_rx', peer=list(peer), length=len(data), hex=data.hex())
                if peer != (self.host, self.udp_port):
                    self.log('ignored_peer', peer=list(peer))
                    continue
                try:
                    sample = parse_packet(data)
                except ValueError as e:
                    self.trip(str(e))
                    continue
                sample['received_at'] = stamp
                with self.lock:
                    previous = self.latest
                    if previous and stamp > previous['received_at']:
                        sample['position_derivative_mm_s'] = ((sample['position_mm'] -
                            previous['position_mm']) / (stamp - previous['received_at']))
                    else:
                        sample['position_derivative_mm_s'] = None
                    if self.previous_words is not None:
                        self.changed_words.update(i for i, (a, b) in enumerate(
                            zip(self.previous_words, sample['words'])) if a != b)
                    self.previous_words = sample['words']
                    self.latest = sample
                    self.packet_count += 1
                    if self.first_time is None:
                        self.first_time = stamp
        except Exception as e:
            if not self.done.is_set():
                self.trip(f'UDP receiver failed: {e}')

    def _receive_tcp(self):
        pending = b''
        try:
            while not self.done.is_set():
                try:
                    data = self.tcp.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    self.trip('TCP peer disconnected; stop delivery is not assured')
                    return
                self.log('tcp_rx', hex=data.hex(), text=data.decode('ascii', 'backslashreplace'))
                print('TCP RX:', repr(data), flush=True)
                # Error vocabulary is not established; preserve every byte.
                pending = (pending + data)[-8192:]
                if any(word in pending.lower() for word in (b'error', b'invalid', b'unknown')):
                    self.trip('possible controller error; inspect TCP log')
        except Exception as e:
            if not self.done.is_set():
                self.trip(f'TCP receiver failed: {e}')

    def snapshot(self):
        with self.lock:
            return dict(self.latest) if self.latest else None

    def wait_feedback(self, timeout=3.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                if self.fault:
                    raise RuntimeError(self.fault)
                if self.latest and time.monotonic() - self.latest['received_at'] <= self.feedback_timeout:
                    return dict(self.latest)
            time.sleep(0.01)
        raise TimeoutError('no fresh valid rail feedback')

    def _send(self, line):
        if self.tcp is None:
            raise RuntimeError('TCP is not connected')
        wire = line + self.terminator
        self.tcp.sendall(wire)
        self.log('tcp_tx', hex=wire.hex(), text=wire.decode('ascii'))

    def arm(self, *, units_per_mm, lower_mm, upper_mm, duration):
        values = (units_per_mm, lower_mm, upper_mm, duration)
        if not all(math.isfinite(x) for x in values):
            raise ValueError('motion settings must be finite')
        if units_per_mm <= 0 or lower_mm >= upper_mm or not 0 < duration <= 2:
            raise ValueError('invalid scale, limits, or duration (maximum 2 s)')
        with self.lock:
            sample = self.latest
            if self.tcp is None or self.fault or self.stop_sent or self.armed:
                raise RuntimeError('client cannot be armed')
            if not sample or time.monotonic() - sample['received_at'] > self.feedback_timeout:
                raise RuntimeError('fresh feedback required before arming')
            if not lower_mm < sample['position_mm'] < upper_mm:
                raise RuntimeError('position is outside the operator-defined inner limits')
            self.units_per_mm, self.bounds = units_per_mm, (lower_mm, upper_mm)
            self.start_position = sample['position_mm']
            self.deadline = time.monotonic() + duration
            self.last_command = time.monotonic()
            self.armed = True

    def set_velocity(self, velocity_mm_s):
        if not math.isfinite(velocity_mm_s) or abs(velocity_mm_s) > 5:
            self.trip('velocity outside commissioning cap of 5 mm/s')
            raise ValueError('velocity must be finite and within +/-5 mm/s')
        with self.lock:
            if velocity_mm_s == 0:
                self.stop()
                return
            if not self.armed or self.fault or self.stop_sent:
                raise RuntimeError(self.fault or 'motion not armed, or stop latched')
            reason = self._motion_fault(time.monotonic())
            if reason:
                self.trip(reason)
                raise RuntimeError(reason)
            if self.active and self.velocity * velocity_mm_s < 0:
                self.trip('direction change requires a new confirmed session')
                raise RuntimeError(self.fault)
            speed = abs(velocity_mm_s) * self.units_per_mm
            if not math.isfinite(speed) or not 0.05 <= speed < 100:
                self.trip('controller speed cannot be represented by legacy format')
                raise ValueError(self.fault)
            direction = 'FWD' if velocity_mm_s > 0 else 'REV'
            # Mark active BEFORE sending: a partial send may already cause motion.
            self.active = True
            self.velocity = velocity_mm_s
            self.last_command = time.monotonic()
            try:
                self._send(f'AXIS0 JOG VEL {speed:05.1f}:AXIS0 JOG {direction}'.encode('ascii'))
            except Exception:
                self.trip('jog send failed; stop delivery may also fail')
                raise

    def _motion_fault(self, now):
        if now >= self.deadline:
        if self.closed:
            return
        errors = []
        try:
            if self.tcp is not None:
                try:
                    self.stop()
                    time.sleep(0.1)  # Allow response reader to drain after stop.
                except Exception as e:
                    errors.append(str(e))
        finally:
            if self.udp is not None:
                try:
                    self.udp.sendto(UNSUBSCRIBE, (self.host, self.udp_port))
                    self.log('udp_tx', hex=UNSUBSCRIBE.hex())
                except Exception as e:
                    errors.append(f'unsubscribe failed: {e}')
            self.done.set()
            for thread in self.threa
            return 'motion duration reached'
        if now - self.last_command > self.command_timeout:
            return 'command watchdog expired'
        if not self.latest or now - self.latest['received_at'] > self.feedback_timeout:
            return 'feedback watchdog expired'
        pos = self.latest['position_mm']
        if not self.bounds[0] < pos < self.bounds[1]:
            return 'operator-defined position limit reached'
        displacement = pos - self.start_position
        if self.active and displacement * self.velocity < -abs(self.velocity) * 0.1:
            return 'counts moved opposite the confirmed direction convention'
        if abs(displacement) > 11:
            return 'travel exceeds commissioning displacement cap'
        return None

    def _watchdog(self):
        try:
            while not self.done.wait(0.01):
                with self.lock:
                    if self.armed:
                        reason = self._motion_fault(time.monotonic())
                        if reason:
                            self.trip(reason)
        except Exception as e:
            self.trip(f'watchdog failed: {e}')

    def trip(self, reason):
        with self.lock:
            if self.fault is None:
                self.fault = reason
            try:
                if self.tcp is not None:
                    self.stop()
            finally:
                self.log('fault', reason=reason)

    def stop(self):
        with self.lock:
            self.armed = self.active = False  # Latch before any socket operation.
            if self.stop_sent:
                return
            self.stop_sent = True
            try:
                self._send(STOP)
            except Exception as e:
                self.log('stop_failed', error=str(e))
                self.fault = f'STOP DELIVERY FAILED: {e}; use the physical stop'
                raise RuntimeError(self.fault) from e

    def observe_stop(self, timeout=1.5):
        """Look for 0.3 s of fresh, quiet encoder feedback; not a hardware ACK."""
        end = time.monotonic() + timeout
        quiet_since = None
        anchor = None
        while time.monotonic() < end:
            sample = self.snapshot()
            now = time.monotonic()
            if sample is None or now - sample['received_at'] > self.feedback_timeout:
                raise RuntimeError('cannot assess stop: feedback missing or stale')
            position = sample['position_mm']
            if anchor is None or abs(position - anchor) > 0.01:
                anchor, quiet_since = position, now
            elif now - quiet_since >= 0.3:
                self.log('encoder_quiet_after_stop', tolerance_mm_legacy=0.01,
                         quiet_seconds=now - quiet_since)
                return
            time.sleep(0.02)
        self.log('stop_not_observed')
        raise RuntimeError('encoder did not settle after stop; use physical stop, do not retry')

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def close(self):
        if self.closed:
            return
        errors = []
        try:
            if self.tcp is not None:
                try:
                    self.stop()
                    time.sleep(0.1)  # Allow response reader to drain after stop.
                except Exception as e:
                    errors.append(str(e))
        finally:
            if self.udp is not None:
                try:
                    self.udp.sendto(UNSUBSCRIBE, (self.host, self.udp_port))
                    self.log('udp_tx', hex=UNSUBSCRIBE.hex())
                except Exception as e:
                    errors.append(f'unsubscribe failed: {e}')
            self.done.set()
            for thread in self.threads:
                thread.join(timeout=0.3)
            for sock in (self.tcp, self.udp):
                if sock is not None:
                    sock.close()
            self.log('closed', errors=errors, fault=self.fault)
            self.closed = True
            self.log_file.close()
        if errors:
            raise RuntimeError('; '.join(errors))


def show(client):
    sample = client.snapshot()
    if sample is None:
        print('Waiting for first valid packet (no state yet).', flush=True)
        return
    with client.lock:
        elapsed = sample['received_at'] - client.first_time
        rate = (client.packet_count - 1) / elapsed if elapsed > 0 else 0
        changed = sorted(client.changed_words)
    age = time.monotonic() - sample['received_at']
    derivative = sample['position_derivative_mm_s']
    print(f"counts={sample['counts']} mm(legacy)={sample['position_mm']:.6f} "
          f"velocity_raw={sample['velocity_raw']:.6g} "
          f"dpos_mm_s={derivative} len={sample['length']} "
          f"rate={rate:.1f}Hz age={age:.3f}s changed_words={changed}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', help='new JSONL log path (never overwrite)')
    parser.add_argument('--terminator', choices=TERMINATORS, default='cr')
    sub = parser.add_subparsers(dest='mode', required=True)
    listen = sub.add_parser('listen')
    listen.add_argument('--duration', type=float, default=10)
    jog = sub.add_parser('jog')
    jog.add_argument('--vel', type=float, required=True, help='requested mm/s; +/-5 cap')
    jog.add_argument('--duration', type=float, required=True, help='seconds; 2 s cap')
    jog.add_argument('--units-per-mm', type=float, required=True,
                     help='operator-verified controller user units per mm')
    jog.add_argument('--min-mm', type=float, required=True, help='inner lower travel bound')
    jog.add_argument('--max-mm', type=float, required=True, help='inner upper travel bound')
    jog.add_argument('--watchdog-test-after', type=float,
                     help='stop submitting velocities at this time; watchdog must stop')
    sub.add_parser('stop')
    args = parser.parse_args()
    if args.mode == 'listen' and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error('listen duration must be finite and positive')
    if args.mode == 'jog':
        vals = (args.vel, args.duration, args.units_per_mm, args.min_mm, args.max_mm)
        if not all(math.isfinite(v) for v in vals) or not 0 < abs(args.vel) <= 5 or not 0 < args.duration <= 2:
            parser.error('finite values required; 0 < |vel| <= 5, 0 < duration <= 2')
        if args.units_per_mm <= 0 or args.min_mm >= args.max_mm:
            parser.error('positive units-per-mm and ordered travel limits required')
        if args.watchdog_test_after is not None and (not math.isfinite(args.watchdog_test_after)
                or not 0 < args.watchdog_test_after < args.duration - 0.25):
            parser.error('watchdog test must allow at least 0.25 s before duration ends')
        print(f'RAIL + MOUNTED UR10e: {"FWD" if args.vel > 0 else "REV"}, '
              f'{abs(args.vel)} mm/s for at most {args.duration} s; '
              f'inner travel bounds [{args.min_mm}, {args.max_mm}] mm.\n'
              f'Controller scale: {args.units_per_mm} user units/mm.\n'
              'Confirm measured position scale, speed scale, FWD=increasing counts, '
              'travel bounds with stopping margin, clear path, and operator at physical stop.\n'
              'This host watchdog cannot stop motion after process/host/network failure.')
        if input('Type CONFIRM RAIL MOTION to authorize this exact step: ').strip() != 'CONFIRM RAIL MOTION':
            parser.error('motion not confirmed')
    log_path = args.log or str(Path(__file__).with_name('logs') /
                               f'{time.time_ns()}_{args.mode}.jsonl')
    Path(log_path).parent.mkdir(parents=True, exist_ok=True)
    client = RailClient(log_path, terminator=args.terminator)
    print('Log:', log_path, flush=True)
    interrupted = threading.Event()
    def shutdown(signum, frame):
        interrupted.set()
    old_handlers = {s: signal.signal(s, shutdown) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        if args.mode == 'stop':
            # No feedback prerequisite: manual recovery should always attempt stop.
            client.connect()
            client.stop()
            interrupted.wait(0.5)
            print('Stop bytes sent; physical stop has NOT been verified.', flush=True)
        else:
            client.subscribe()
            client.wait_feedback()
            if args.mode == 'jog' and not interrupted.is_set():
                client.connect()
                client.arm(units_per_mm=args.units_per_mm, lower_mm=args.min_mm,
                           upper_mm=args.max_mm, duration=args.duration)
            start, next_print = time.monotonic(), 0.0
            while not interrupted.is_set() and time.monotonic() - start < args.duration:
                now = time.monotonic()
                if client.fault:
                    raise RuntimeError(client.fault)
                if args.mode == 'jog' and (args.watchdog_test_after is None or
                                           now - start < args.watchdog_test_after):
                    client.set_velocity(args.vel)
                if now >= next_print:
                    show(client)
                    next_print = now + 0.25
                interrupted.wait(0.05)
            if args.mode == 'jog':
                client.stop()
                # Record feedback after stop, do not claim that ACK means stopped.
                end = time.monotonic() + 1.0
                while not interrupted.is_set() and time.monotonic() < end:
                    show(client)
                    interrupted.wait(0.25)
            show(client)
    finally:
        try:
            try:
                if args.mode == 'jog' and client.tcp is not None:
                    client.stop()
                    client.observe_stop()
            finally:
                client.close()
        finally:
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)
    if client.fault:
        raise RuntimeError(client.fault)


if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as exc:
        print(f'EXIT: {exc}', flush=True)
        raise SystemExit(1)
