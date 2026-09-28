# Parker rail protocol

Standalone Python 3 module for UDP rail feedback, bounded TCP jogging, and stop.
Standard library only; no ROS node or UR10e communication.

## Files

- [TERMINAL_SOCKET.md](TERMINAL_SOCKET.md): saved interactive Python socket helper and individual-command instructions.
- `rail_protocol.py`: reusable `RailClient` and `listen`, `jog`, `stop` CLI.
- `test_rail_protocol.py`: offline tests with mocked controller sockets.
- `logs/`: raw verification captures and offline test results. Historical captures
  are retained as evidence; the one-off diagnostic and homing scripts are removed.

## Setup

The rail controller is `192.168.7.6`; TCP commands use port **5002**, UDP feedback
uses port **5003**. The host was verified as `192.168.7.50/24` on `enp4s0f1`.
Check the current host address with `ip -4 addr` before connecting. On Windows,
check inbound UDP firewall rules if packets do not arrive. Only one local
listener may bind UDP port 5003.

Configure the shared fast-status fields through the controller terminal:

```text
FSTAT0(48,2)
FSTAT1(48,58)
FSTAT ON
```

Use `FSTAT` without arguments to inspect the configuration. The module subscribes
to the existing stream; it does **not** change FSTAT mappings or enable the drive.
These mappings must match before interpreting feedback. Persistence across a
controller restart has not been verified. Unsubscribing affects this client;
it does not send FSTAT OFF or clear the shared mappings.

## CLI

Run from `~/rail_protocol`. Each invocation creates a timestamped JSONL log in
`logs/`, or use `--log PATH` before the mode to select a new file. Logs are never
overwritten. Times include wall-clock and monotonic timestamps; packet bytes are
stored in hex and TCP replies also have an escaped text representation.

### Listen

```bash
python3 rail_protocol.py listen --duration 60
```

Subscribes from the same socket bound to `0.0.0.0:5003`, displays feedback at
4 Hz, and unsubscribes on exit. Reports signed encoder counts, position using
the legacy mm scale, raw jog velocity, a position derivative, packet length,
mean packet rate, sample age, and word indices that changed. Compare stationary
and moving logs to investigate unknown fields. No state is produced before the
first valid packet. Check sample age: a previous sample is not live feedback.
Ctrl+C ends the capture. Listen sends no drive, jog, or stop commands.

### Jog

```text
python3 rail_protocol.py jog --vel V --duration T \
  --units-per-mm SCALE --min-mm LOWER --max-mm UPPER
```

Replace placeholders with operator-confirmed values: signed velocity V in mm/s,
duration T in seconds, controller user units per mm, and inner travel bounds in
the displayed encoder coordinate system. Include stopping margin in those bounds.
The CLI requires explicit confirmation of direction, speed, duration, clear path,
limits, scaling, and the operator's physical-stop arrangement.

Commissioning limits remain **5 mm/s maximum** and **2 seconds maximum**, as in
the original request. Velocity commands are sent at 20 Hz on one persistent TCP
connection. Positive velocity selects FWD, negative selects REV; zero requests
JOG OFF. These commands move the rail and its mounted UR10e.

The module does not home, enable drives, set acceleration/deceleration, clear
faults, or configure physical limits. Those are controller/operator setup tasks.
To reproduce the original watchdog verification, add
`--watchdog-test-after SECONDS`; the command producer stops submitting commands
while the independent watchdog keeps running. Allow at least 0.25 seconds before
T expires. Expected watchdog expiry is logged as a fault and exits nonzero.
Hardware motion tests require separate operator authorization.

### Stop

```bash
python3 rail_protocol.py stop
```

Connects to TCP 5002 and sends `AXIS0 JOG OFF`, without requiring UDP feedback.
It logs replies and closes. JOG OFF requests deceleration of jog motion; it does
not disable the drive. Closing a TCP connection alone is not a stop command.

CR is the default command terminator. For explicit protocol experiments only,
`--terminator crlf` or `--terminator lfcr` can be placed before the mode. The module
does not automatically retry motion commands with alternate terminators.

## Protocol and verification status

| Item | Format / finding | Evidence status |
|---|---|---|
| Controller identity | IPA Drive, firmware 4.46 Update 4 | Read with VER |
| Subscribe | `00 00 00 01 00 00 00 0a` to UDP 5003 | Received approximately 100 Hz; Parker documents nonzero enable and interval in ms |
| Unsubscribe | Eight zero bytes to UDP 5003 | Transmitted on capture shutdown |
| Packet format | 80 big-endian 32-bit words, 320 bytes in captures | Observed; decoder requires 320..2560 bytes and logs the full packet |
| Word 0, offset 0 | Signed int32 actual position; FSTAT0(48,2), axis 0 P12290 | UDP and TCP reads agree within small stationary variation |
| Position scale | 26214.4 counts/mm | Legacy scale agrees numerically with controller PPU; measured-distance calibration pending |
| Word 8, offset 32 | Float32 **current jog velocity**; FSTAT1(48,58), axis 0 P12346 | About 1 raw unit/s agreed with about 1 mm/s derived from encoder counts; independent physical calibration pending |
| Word 16, offset 64 | Legacy label OutSig | Currently unassigned group 2; meaning unverified, not used for control |
| Other words | Raw uint32 values | Unknown/unassigned; only words 0 and 8 changed in configured motion capture |
| Positive jog | `AXIS0 JOG VEL 001.0:AXIS0 JOG FWD` | Legacy command format; operator confirmed terminal control works |
| Negative jog | Same positive magnitude with JOG REV | Physical direction convention still needs an explicit lab reference |
| Stop | `AXIS0 JOG OFF\r` | Echo/prompt received; later jog-inactive and zero-velocity reads observed |
| Replies | Command echo followed by CRLF and SYS>; errors may be plain text | Log all bytes; echo alone does not prove physical execution |

The velocity field is the jog profile's current velocity, **not an independently
measured carriage velocity**. The separate TCP parameter P12315 is not the same
field; do not interchange their units. Packet configuration matters: before
FSTAT initialization, valid-looking 320-byte packets were entirely zero.

Acceleration/deceleration were initially 0.1 and later read as 10. No module code
changes them. At 0.1 mm/s², 35 mm/s takes 350 seconds to reach from rest. Earlier
10-second homing probes therefore peaked around 1 mm/s and then decelerated
slowly. These historical homing probes are not part of the retained CLI.

## Cleanup and watchdog behavior

`RailClient` owns both sockets, protects shared state with a lock, and provides
`subscribe`, `connect`, `snapshot`, `arm`, `set_velocity`, `stop`, and `close`.
It also supports a context manager. A future ROS wrapper can renew velocity
commands and consume snapshots after hardware validation; no ROS wrapper exists.

During armed motion, command and feedback leases expire after 200 ms, checked
by an independent thread every 10 ms. Faults and stops latch out further motion.
Malformed feedback, TCP failure, recognized error text, expired duration,
operator-defined position limits, excessive travel, or opposite-sign displacement
request stop. Replies are not a complete controller-error parser.

Normal exits, exceptions, SIGINT and SIGTERM attempt stop for a TCP-connected
client, unsubscribe for a UDP client, and close sockets. The CLI observes fresh,
quiet encoder feedback after jogging; this is not proof of physical stopping.
The stop tool has no UDP subscription to cancel. Failed stop delivery is reported.

This host watchdog cannot guarantee stopping after SIGKILL, a frozen process,
blocked host I/O, power loss, or a broken network. Socket/scheduler latency adds
to the timeout. JOG OFF uses configured deceleration; physical limits, a physical
stop, and controller-side supervision remain necessary. The class is a protocol
layer, not a safety-rated motion controller.

## Tests and raw evidence

```bash
python3 -B -m unittest -v test_rail_protocol.py
```

Tests cover decoding, invalid lengths/nonfinite fields, missing or stale feedback,
command expiry, duration, limits, stop latching, partial sends, cleanup failures,
subscription lifecycle, and post-stop observations. They use mocked sockets;
no hardware commands are sent. Physical watchdog testing remains unverified.

Selected captures (all original logs are retained):

- `logs/host_network.txt`: host subnet check.
- `logs/1790357691110797536_stop.jsonl`: stop command and echo.
- `logs/1790359368370308374_read_status.jsonl`: FSTAT configuration/readback.
- `logs/1790359375717067617_listen.jsonl`: stationary configured UDP capture.
- `logs/1790359582315400829_listen.jsonl`: velocity ramp and encoder changes.
- `logs/1790359939282402096_listen.jsonl`: operator's PuTTY run; packets ceased
  before the recording ended, so it does not cover all subsequent activity.
- `logs/1790361754430571090_read_status.jsonl`: later home-found, jog-inactive,
  zero velocity and acceleration/deceleration 10 snapshot. These are historical
  readings, not a statement of the controller's current state.

Remaining verification: measured-distance calibration, physical direction labels,
stop distance under the intended settings, and hardware command/feedback watchdog
experiments. Successful socket writes or matching telemetry alone are insufficient.

References: [Ethernet specification](https://www.parkermotion.com/manuals/Acroloop/Ethernet_Spec_ACR.pdf),
[command reference](https://www.parkermotion.com/manuals/Acroloop/ACR_UG1.pdf),
[parameter reference](https://www.parkermotion.com/manuals/Acroloop/ACR_UG2.pdf).
