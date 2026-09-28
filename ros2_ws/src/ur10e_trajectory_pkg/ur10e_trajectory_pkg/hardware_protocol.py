"""Wire formats for the UR10e and Parker rail, as the Simulink rig used them.

ROS-free so it can be tested without sockets or hardware. The formats are the
ones BaseFrameDetermination.slx sends and parses:

  UR10e  192.168.7.8:30003   realtime state packets in; URScript text out
         (speedj streamed at 20 Hz, one script line per command).
  Parker 192.168.7.6:5002    ASCII jog commands out (JOG VEL + JOG FWD/REV)
         192.168.7.6:5003    UDP fast-status feedback in (FSTAT0 = position
                             counts, FSTAT1 = current jog velocity).

Units at the boundary are converted here once. Inside ROS the rail is metres
along the URDF linear_rail_joint and the arm is radians, as everywhere else in
this package.
"""

import math
import re
import struct

import numpy as np

# --- UR10e realtime interface (port 30003) ---------------------------------

UR_REALTIME_PORT = 30003
# Offsets into the packet's doubles, after the 4-byte length prefix. They are
# the 1-based Simulink selectors [1], [32:37] and [38:43], shifted to 0-based.
UR_TIME_INDEX = 0
UR_Q_ACTUAL = slice(31, 37)
UR_QD_ACTUAL = slice(37, 43)
# Speed scaling: the fraction of programmed speed the controller is applying,
# i.e. the pendant speed slider combined with reduced mode or any safety
# limit. Not read by the Simulink model; offset from the UR client-interface
# realtime layout (time, 15 six-vectors, then scalars up to index 117).
UR_SPEED_SCALING_INDEX = 117
UR_MIN_DOUBLES = 43
UR_ARM_JOINTS = 6


def ur_packet_length(header):
    """Total packet length announced by the first four bytes."""
    if len(header) != 4:
        raise ValueError('UR packet header must be 4 bytes')
    length = struct.unpack('>I', header)[0]
    if length < 4 + 8 * UR_MIN_DOUBLES or length > 1 << 16:
        raise ValueError(f'implausible UR realtime packet length {length}')
    return length


def parse_ur_realtime(packet):
    """Controller time, actual joint positions and velocities from one packet.

    The packet is big-endian: a uint32 total length, then doubles. Its length
    differs between controller versions (1116 bytes on the Simulink rig), but
    the fields used here sit at the same offsets in all of them.
    """
    length = ur_packet_length(packet[:4])
    if len(packet) != length:
        raise ValueError(f'UR packet is {len(packet)} bytes, header says {length}')
    count = (length - 4) // 8
    values = struct.unpack_from(f'>{count}d', packet, 4)
    q = np.asarray(values[UR_Q_ACTUAL], dtype=float)
    qd = np.asarray(values[UR_QD_ACTUAL], dtype=float)
    if not (np.all(np.isfinite(q)) and np.all(np.isfinite(qd))):
        raise ValueError('UR packet has non-finite joint state')
    scaling = (float(values[UR_SPEED_SCALING_INDEX])
               if count > UR_SPEED_SCALING_INDEX else None)
    return {'time': float(values[UR_TIME_INDEX]), 'q': q, 'qd': qd,
            'speed_scaling': scaling, 'length': length}


def _ur_vector(values):
    values = np.asarray(values, dtype=float)
    if values.shape != (UR_ARM_JOINTS,) or not np.all(np.isfinite(values)):
        raise ValueError('UR command needs six finite joint values')
    return '[' + ','.join(f'{value:.6f}' for value in values) + ']'


def speedj_command(velocities, acceleration, duration):
    """One URScript speedj line.

    duration is how long the controller keeps this velocity if no newer
    command replaces it, so it doubles as the stream's dead-man timeout.
    """
    if not (math.isfinite(acceleration) and acceleration > 0):
        raise ValueError('speedj acceleration must be positive and finite')
    if not (math.isfinite(duration) and duration > 0):
        raise ValueError('speedj duration must be positive and finite')
    return (f'speedj({_ur_vector(velocities)},{acceleration:.4f},'
            f'{duration:.4f})\n').encode('ascii')


def stopj_command(deceleration):
    if not (math.isfinite(deceleration) and deceleration > 0):
        raise ValueError('stopj deceleration must be positive and finite')
    return f'stopj({deceleration:.4f})\n'.encode('ascii')


# --- Parker IPA rail ---------------------------------------------------------

RAIL_COMMAND_PORT = 5002
RAIL_FEEDBACK_PORT = 5003
RAIL_SUBSCRIBE = bytes.fromhex('000000010000000a')   # enable, 10 ms interval
RAIL_UNSUBSCRIBE = bytes(8)
RAIL_STOP = b'AXIS0 JOG OFF'
# Legacy encoder scale used by the Simulink model (Gain 1/26214.4/1000).
RAIL_COUNTS_PER_MM = 26214.4
# JOG VEL is formatted %05.1f, so 0.1 unit/s is the smallest nonzero speed
# the legacy command can express and 999.9 the largest.
RAIL_MIN_SPEED_UNITS = 0.05
RAIL_MAX_SPEED_UNITS = 999.9
RAIL_TERMINATORS = {'cr': b'\r', 'crlf': b'\r\n', 'lfcr': b'\n\r'}


def parse_rail_feedback(data):
    """Encoder counts and raw jog velocity from one fast-status datagram."""
    if not 320 <= len(data) <= 2560:
        raise ValueError(f'rail packet length {len(data)} outside 320..2560')
    counts = struct.unpack_from('>i', data, 0)[0]
    velocity = struct.unpack_from('>f', data, 32)[0]
    if not math.isfinite(velocity):
        raise ValueError('rail packet has a non-finite velocity')
    return counts, float(velocity)


class RailCalibration:
    """Map between controller units and the planner's rail coordinate.

    After homing, the controller's zero is repeatable, but it is not the URDF
    zero: offset_m is the planner coordinate of controller position 0, and
    sign is +1 when increasing counts (JOG FWD) move toward +x in the URDF.
    units_per_mm is the controller's user units per mm for JOG VEL; the rig's
    measurements put it near 1, but it is not yet calibrated.
    """

    def __init__(self, offset_m=0.0, sign=1.0, units_per_mm=1.0,
                 counts_per_mm=RAIL_COUNTS_PER_MM):
        if sign not in (1.0, -1.0):
            raise ValueError('rail sign must be +1 or -1')
        for name, value in (('offset_m', offset_m), ('units_per_mm', units_per_mm),
                            ('counts_per_mm', counts_per_mm)):
            if not math.isfinite(value):
                raise ValueError(f'rail {name} must be finite')
        if units_per_mm <= 0 or counts_per_mm <= 0:
            raise ValueError('rail scales must be positive')
        self.offset_m = float(offset_m)
        self.sign = float(sign)
        self.units_per_mm = float(units_per_mm)
        self.counts_per_mm = float(counts_per_mm)

    def position_m(self, counts):
        return self.offset_m + self.sign * counts / self.counts_per_mm / 1000.0

    def velocity_m_s(self, velocity_units):
        return self.sign * velocity_units / self.units_per_mm / 1000.0

    def controller_velocity(self, velocity_m_s):
        """Signed JOG VEL value, in controller units/s, for a planner velocity."""
        return self.sign * velocity_m_s * 1000.0 * self.units_per_mm


def rail_jog_command(velocity_units):
    """The legacy one-line jog: speed magnitude, then direction.

    Returns RAIL_STOP for speeds the format rounds to zero, and refuses speeds
    it cannot represent rather than truncating them.
    """
    if not math.isfinite(velocity_units):
        raise ValueError('rail velocity must be finite')
    speed = abs(velocity_units)
    if speed < RAIL_MIN_SPEED_UNITS:
        return RAIL_STOP
    if speed > RAIL_MAX_SPEED_UNITS:
        raise ValueError(f'rail speed {speed:.1f} units/s exceeds the command format')
    direction = 'FWD' if velocity_units > 0 else 'REV'
    return f'AXIS0 JOG VEL {speed:05.1f}:AXIS0 JOG {direction}'.encode('ascii')


# Home-found flag: -1 when set, 0 when clear. Read-only; it says the axis has
# been referenced since power-up, not that the carriage is at the switch now.
RAIL_HOMED_QUERY = b'PRINT BIT 16134'


def rail_home_command(velocity_units, direction=-1):
    """Set the jog speed, then home: the controller homes at that jog speed."""
    if direction not in (-1, 1):
        raise ValueError('homing direction must be -1 or +1')
    if not RAIL_MIN_SPEED_UNITS <= velocity_units <= RAIL_MAX_SPEED_UNITS:
        raise ValueError(f'homing speed {velocity_units} units/s outside the command format')
    return f'AXIS0 JOG VEL {velocity_units:05.1f}:AXIS0 JOG HOME {direction:d}'.encode('ascii')


def parse_rail_bit_reply(reply, query=RAIL_HOMED_QUERY.decode('ascii')):
    """True/False from a PRINT BIT reply, or None if it cannot be read.

    The controller echoes the command, then prints the value on its own line
    and a prompt. Only a standalone -1 or 0 after the echo is accepted, so a
    prompt such as P00> is never mistaken for the value.
    """
    index = reply.find(query)
    rest = reply[index + len(query):] if index >= 0 else reply
    match = re.search(r'^\s*(-?\d+)\s*$', rest, flags=re.MULTILINE)
    if match is None:
        return None
    return {'-1': True, '0': False}.get(match.group(1))
