"""Offline tests: all controller sockets are mocked; no network traffic."""
import math
from pathlib import Path
import struct
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from rail_protocol import RailClient, parse_packet, SUBSCRIBE, UNSUBSCRIBE


def packet(counts=0, velocity=0.0, length=320):
    data = bytearray(length)
    if length >= 68:
        struct.pack_into('>i', data, 0, counts)
        struct.pack_into('>f', data, 32, velocity)
        struct.pack_into('>f', data, 64, 2.5)
    return bytes(data)


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.client = RailClient(Path(self.directory.name) / 'test.jsonl')
        self.tcp = Mock()
        self.client.tcp = self.tcp

    def tearDown(self):
        self.client.close()
        self.directory.cleanup()

    def ready(self, duration=1):
        self.client.latest = {**parse_packet(packet()), 'received_at': time.monotonic()}
        self.client.arm(units_per_mm=1, lower_mm=-20, upper_mm=20, duration=duration)

    def test_signed_position_float_offsets_and_sizes(self):
        result = parse_packet(packet(-262144, 12.5, 2560))
        self.assertEqual(result['counts'], -262144)
        self.assertEqual(result['position_mm'], -10)
        self.assertEqual(result['velocity_raw'], 12.5)
        self.assertEqual(result['outsig_raw'], 2.5)
        for length in (0, 64, 319, 2561):
            with self.assertRaises(ValueError):
                parse_packet(packet(length=length))
        with self.assertRaises(ValueError):
            parse_packet(packet(velocity=math.nan))

    def test_no_state_before_first_packet_and_no_jog(self):
        self.assertIsNone(self.client.snapshot())
        with self.assertRaises(RuntimeError):
            self.client.set_velocity(1)
        self.tcp.sendall.assert_not_called()

    def test_forward_reverse_and_stop_encoding(self):
        self.ready()
        self.client.set_velocity(-1.5)
        self.tcp.sendall.assert_called_with(b'AXIS0 JOG VEL 001.5:AXIS0 JOG REV\r')
        self.client.stop()
        self.tcp.sendall.assert_called_with(b'AXIS0 JOG OFF\r')
        with self.assertRaises(RuntimeError):
            self.client.set_velocity(-1.5)

    def test_stale_feedback_prevents_motion(self):
        self.ready()
        self.client.latest['received_at'] -= 1
        with self.assertRaises(RuntimeError):
            self.client.set_velocity(1)
        self.assertIn('feedback watchdog', self.client.fault)
        self.assertEqual(self.tcp.sendall.call_count, 1)
        self.tcp.sendall.assert_called_with(b'AXIS0 JOG OFF\r')

    def test_command_watchdog_runs_without_command_producer(self):
        self.ready()
        self.client.set_velocity(1)
        self.client.last_command -= 0.21
        self.client._thread(self.client._watchdog)
        end = time.monotonic() + 0.5
        while not self.client.stop_sent and time.monotonic() < end:
            time.sleep(0.005)
        self.assertTrue(self.client.stop_sent)
        self.assertEqual(self.client.fault, 'command watchdog expired')
        self.tcp.sendall.assert_called_with(b'AXIS0 JOG OFF\r')

    def test_duration_stop(self):
        self.ready()
        self.client.deadline = time.monotonic() - 1
        with self.assertRaises(RuntimeError):
            self.client.set_velocity(1)
        self.assertEqual(self.client.fault, 'motion duration reached')

    def test_wrong_direction_and_travel_limits(self):
        self.ready()
        self.client.set_velocity(1)
        self.client.latest['position_mm'] = -0.2
        self.assertIn('opposite', self.client._motion_fault(time.monotonic()))
        self.client.latest['position_mm'] = 21
        self.assertIn('position limit', self.client._motion_fault(time.monotonic()))

    def test_nonfinite_command_stops(self):
        self.ready()
        with self.assertRaises(ValueError):
            self.client.set_velocity(math.nan)
        self.tcp.sendall.assert_called_with(b'AXIS0 JOG OFF\r')

    def test_cleanup_unsubscribes_even_if_stop_fails(self):
        self.client.udp = Mock()
        udp = self.client.udp
        self.tcp.sendall.side_effect = OSError('disconnected')
        with self.assertRaises(RuntimeError):
            self.client.close()
        udp.sendto.assert_called_with(UNSUBSCRIBE, ('192.168.7.6', 5003))
        udp.close.assert_called_once()
        self.tcp.close.assert_called_once()

    def test_jog_send_failure_attempts_stop(self):
        self.ready()
        self.tcp.sendall.side_effect = [OSError('partial write'), None]
        with self.assertRaises(OSError):
            self.client.set_velocity(1)
        self.assertEqual(self.tcp.sendall.call_args_list[-1].args[0], b'AXIS0 JOG OFF\r')
        self.assertFalse(self.client.armed)

    def test_subscribe_same_socket_and_unsubscribe(self):
        udp = Mock()
        with patch('rail_protocol.socket.socket', return_value=udp), patch.object(self.client, '_thread'):
            self.client.subscribe()
        udp.bind.assert_called_once_with(('0.0.0.0', 5003))
        udp.sendto.assert_called_once_with(SUBSCRIBE, ('192.168.7.6', 5003))
        self.client.close()
        udp.sendto.assert_called_with(UNSUBSCRIBE, ('192.168.7.6', 5003))

    def test_stop_observation_rejects_stale_feedback(self):
        self.ready()
        self.client.latest['received_at'] -= 1
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            self.client.observe_stop()

    def test_stop_observation_requires_encoder_to_settle(self):
        count = [0]
        def moving():
            count[0] += 1
            return {'received_at': time.monotonic(), 'position_mm': count[0]}
        with patch.object(self.client, 'snapshot', side_effect=moving):
            with self.assertRaisesRegex(RuntimeError, 'did not settle'):
                self.client.observe_stop(timeout=0.05)

    def test_stop_observation_accepts_fresh_quiet_encoder(self):
        def stationary():
            return {'received_at': time.monotonic(), 'position_mm': 2.0}
        with patch.object(self.client, 'snapshot', side_effect=stationary):
            self.client.observe_stop(timeout=0.5)

    def test_forbidden_ur_address(self):
        with self.assertRaises(ValueError):
            RailClient('/unused', host='192.168.7.8')


if __name__ == '__main__':
    unittest.main()
