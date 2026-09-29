"""Wire formats must match what the Simulink rig sent and parsed."""

import struct

import numpy as np
import pytest

from ur10e_trajectory_pkg import hardware_protocol as wire


def ur_packet(count, q, qd, time_s=12.5, scaling=1.0):
    values = np.zeros(count)
    values[0] = time_s
    if count > 117:
        values[117] = scaling
    values[31:37] = q
    values[37:43] = qd
    length = 4 + 8 * count
    return struct.pack('>I', length) + struct.pack(f'>{count}d', *values)


@pytest.mark.parametrize('count', [139, 151])
def test_ur_realtime_packet_yields_time_and_actual_joints(count):
    q = np.linspace(-1.0, 1.0, 6)
    qd = np.linspace(0.1, 0.6, 6)
    state = wire.parse_ur_realtime(ur_packet(count, q, qd))
    assert state['time'] == 12.5
    np.testing.assert_array_equal(state['q'], q)
    np.testing.assert_array_equal(state['qd'], qd)
    assert state['length'] == 4 + 8 * count


def test_ur_packet_reports_speed_scaling_when_present():
    assert wire.parse_ur_realtime(
        ur_packet(139, np.zeros(6), np.zeros(6), scaling=0.3))['speed_scaling'] == 0.3
    assert wire.parse_ur_realtime(ur_packet(60, np.zeros(6), np.zeros(6)))['speed_scaling'] is None


def test_simulink_rig_packet_is_1116_bytes():
    assert len(ur_packet(139, np.zeros(6), np.zeros(6))) == 1116


def test_ur_packet_with_wrong_length_is_refused():
    packet = ur_packet(139, np.zeros(6), np.zeros(6))
    with pytest.raises(ValueError):
        wire.parse_ur_realtime(packet[:-8])
    with pytest.raises(ValueError):
        wire.ur_packet_length(struct.pack('>I', 20))


def test_speedj_line_is_one_urscript_statement():
    line = wire.speedj_command([0.1, -0.2, 0, 0, 0, 0.05], 1.0, 0.2)
    assert line == (b'speedj([0.100000,-0.200000,0.000000,0.000000,0.000000,'
                    b'0.050000],1.0000,0.2000)\n')
    assert wire.stopj_command(2.0) == b'stopj(2.0000)\n'


@pytest.mark.parametrize('velocities', [[0.1] * 5, [0.1] * 5 + [float('nan')]])
def test_speedj_refuses_malformed_vectors(velocities):
    with pytest.raises(ValueError):
        wire.speedj_command(velocities, 1.0, 0.2)


def rail_packet(counts, velocity, length=320):
    data = bytearray(length)
    struct.pack_into('>i', data, 0, counts)
    struct.pack_into('>f', data, 32, velocity)
    return bytes(data)


def test_rail_feedback_reads_counts_and_jog_velocity():
    assert wire.parse_rail_feedback(rail_packet(-26214400, 12.5)) == (-26214400, 12.5)
    with pytest.raises(ValueError):
        wire.parse_rail_feedback(rail_packet(0, 0.0, length=100))
    with pytest.raises(ValueError):
        wire.parse_rail_feedback(rail_packet(0, float('nan')))


def test_rail_calibration_maps_homed_counts_to_planner_metres():
    calibration = wire.RailCalibration(offset_m=1.5, sign=-1.0)
    assert calibration.position_m(0) == 1.5
    assert calibration.position_m(26214.4 * 100) == pytest.approx(1.4)
    assert calibration.controller_velocity(0.01) == pytest.approx(-10.0)
    assert calibration.velocity_m_s(-10.0) == pytest.approx(0.01)
    with pytest.raises(ValueError):
        wire.RailCalibration(sign=0.5)


def test_rail_jog_line_matches_simulink_format():
    assert wire.rail_jog_command(12.34) == b'AXIS0 JOG VEL 012.3:AXIS0 JOG FWD'
    assert wire.rail_jog_command(-3.0) == b'AXIS0 JOG VEL 003.0:AXIS0 JOG REV'
    assert wire.rail_jog_command(0.01) == wire.RAIL_STOP
    with pytest.raises(ValueError):
        wire.rail_jog_command(1000.0)


def test_homing_line_bounds_the_jog_speed_first():
    assert wire.rail_home_command(25.0) == b'AXIS0 JOG VEL 025.0:AXIS0 JOG HOME -1'
    with pytest.raises(ValueError):
        wire.rail_home_command(25.0, direction=0)
    with pytest.raises(ValueError):
        wire.rail_home_command(0.0)


@pytest.mark.parametrize('reply, expected', [
    ('PRINT BIT 16134\r\n-1\r\nSYS>', True),
    ('P00>PRINT BIT 16134\r\n0\r\nP00>', False),
    ('PRINT BIT 16134\r\nSYS>', None),
    ('garbage', None),
])
def test_home_flag_reply_is_read_after_the_echo(reply, expected):
    assert wire.parse_rail_bit_reply(reply) is expected


def test_controller_modes_and_target_velocity_are_decoded():
    values = np.zeros(139)
    values[7:13] = [0.1, 0, 0, 0, 0, 0.2]
    values[94], values[101], values[131] = 7, 3, 2
    packet = struct.pack('>I', 1116) + struct.pack('>139d', *values)
    state = wire.parse_ur_realtime(packet)
    np.testing.assert_array_equal(state['qd_target'], [0.1, 0, 0, 0, 0, 0.2])
    assert wire.ur_mode_name(wire.UR_ROBOT_MODES, state['robot_mode']) == 'RUNNING (7)'
    assert wire.ur_mode_name(wire.UR_SAFETY_MODES, state['safety_mode']) == 'PROTECTIVE_STOP (3)'
    assert wire.ur_mode_name(wire.UR_PROGRAM_STATES, state['program_state']) == 'PLAYING (2)'


def test_streaming_program_is_one_def_with_its_own_watchdog():
    program = wire.stream_program('192.168.7.50', 50010, 1.0, 2.0, 0.25).decode()
    assert program.startswith('def ros_speedj_stream():') and program.endswith('end\n')
    assert 'socket_open("192.168.7.50", 50010, "ros_stream")' in program
    assert 'if age > 32:' in program                 # 0.25 s at 8 ms per loop
    lines = [line.strip() for line in program.splitlines()]
    openers = [line for line in lines
               if line.endswith(':') and not line.startswith(('else', 'elif'))]
    assert len(openers) == lines.count('end')        # every block is closed
    assert wire.stream_vector([0.1, 0, 0, 0, 0, -0.05]) == \
        b'(0.100000,0.000000,0.000000,0.000000,0.000000,-0.050000)\n'
    with pytest.raises(ValueError):
        wire.stream_program('robot.local', 50010, 1.0, 2.0, 0.25)


def test_rtde_program_and_registers_follow_the_register_base():
    assert wire.rtde_input_names(18) == tuple(
        f'input_double_register_{n}' for n in range(18, 24)) + ('input_int_register_18',)
    program = wire.rtde_program(1.0, 2.0, 0.25, base=18).decode()
    assert 'read_input_integer_register(18)' in program
    assert 'read_input_float_register(23)' in program and '(24)' not in program
