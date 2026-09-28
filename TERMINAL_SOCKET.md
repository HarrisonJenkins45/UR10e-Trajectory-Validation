# Send individual rail commands from a terminal

This is the interactive Python socket helper used in the working terminal
session. It connects to the Parker rail at **192.168.7.6, TCP port 5002** and
sends ASCII commands terminated by a carriage return (`\r`). It requires only
Python 3 and its standard library; ROS and PuTTY are not required.

Use one command client at a time. Close the other PuTTY/command session first.

## Open Python

From your shell:

```bash
cd ~/rail_protocol
python3
```

## Connect and define the helper

Paste the following into Python, without the `>>>` or `...` prompt markers.
Press Enter on a blank line after the function definition to finish it.

```python
import socket
s = socket.create_connection(("192.168.7.6", 5002), timeout=2)
s.settimeout(0.5)

def command(text):
    s.sendall((text + "\r").encode("ascii"))
    try:
        while True:
            data = s.recv(4096)
            if not data:
                break
            print(data.decode("ascii", errors="replace"), end="", flush=True)
    except socket.timeout:
        pass


```

The helper prints incoming bytes until no more arrive for 0.5 seconds. That
socket timeout only ends the reply wait; it does **not** stop motion. A delayed
reply may appear during a later call. Echoes and `SYS>` prompts alone do not
confirm physical execution. This helper does not parse errors or save a log.

## Send one command at a time

Start with a read-only command:

```python
command("VER")
```

Read the configured jog velocity, acceleration, and deceleration:

```python
command("PRINT P12348")
command("PRINT P12349")
command("PRINT P12350")
```

To prepare the UDP feedback mappings used by `rail_protocol.py` (these change
shared telemetry configuration, but do not request motion):

```python
command("FSTAT0(48,2)")
command("FSTAT1(48,58)")
command("FSTAT ON")
```

Actual drive/motion commands must be entered individually only when the operator
has confirmed the intended direction, speed, duration, travel limits, and clear
path. The UR10e rides on the rail. This interactive helper has **no automatic
motion timeout or command/feedback watchdog**; use the guarded `jog` CLI for
bounded tests. Keep the physical stop available.

Request jog stop:

```python
command("AXIS0 JOG OFF")
```

JOG OFF decelerates using the controller's configured settings; it is not an
instantaneous stop and does not disable the drive. To check controller state:

```python
command("PRINT BIT 792")    # Jog active: -1 means set, 0 means clear.
command("PRINT P12346")     # Current jog velocity.
command("PRINT BIT 16134")  # Home-found flag; not proof of current home location.
```

Other individual command strings used during commissioning are listed here as
reference, not as a batch to paste or execute:

| Command string | Purpose |
|---|---|
| `AXIS0 DRIVE ON` | Enable axis 0 drive |
| `AXIS0 DRIVE OFF` | Disable axis 0 drive |
| `AXIS0 JOG VEL 35` | Set target jog velocity to 35 controller user units/s |
| `AXIS0 JOG FWD` | Start continuous forward jogging |
| `AXIS0 JOG REV` | Start continuous reverse jogging |
| `AXIS0 JOG HOME -1` | Start homing in the negative direction |
| `AXIS0 JOG ABS 1550`| Centers the arm on the railV


For example, `command("AXIS0 JOG VEL 35")` sets the target; acceleration and
controller configuration determine the resulting speed. Never assume enabling
a drive or changing velocity is harmless when a motion request is pending.

## Finish the session

Request stop and verify that the axis has settled before disconnecting:

```python
command("AXIS0 JOG OFF")
command("PRINT BIT 792")
command("PRINT P12346")
```

Then close the socket and leave Python:

```python
s.close()
exit()
```

Closing Python/the socket, Ctrl+C, or a communication timeout does not provide
an automatic stop in this interactive helper. If communication fails, stop
delivery cannot be assumed; use the physical stop as needed.

See [RAIL_PROTOCOL.md](RAIL_PROTOCOL.md) for the guarded CLI, feedback mappings,
protocol evidence, and remaining verification work.
