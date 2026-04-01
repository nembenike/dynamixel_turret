# Architecture

Dynamix Turret is a two-process runtime with multi-threaded control in the main process.

## High-level components

1. C/C++ main process ([turret.c](../turret.c))
   - Serial communication with AX-12A servos (Dynamixel protocol 1.0)
   - UI rendering and operator input handling (`raylib`)
   - Hand socket client and camera texture updates
2. Python hand process ([hand_server.py](../hand_server.py))
   - Camera capture (OpenCV)
   - Hand landmark extraction (MediaPipe solutions or tasks API)
   - Length-prefixed packet transmission over Unix domain socket

## Process and thread model

### Main process threads

- Main/UI thread
  - Runs event loop, applies control logic, renders dashboard
- `feedback_thread`
  - Polls servo position frequently and telemetry periodically
- `hand_tracking_thread`
  - Forks/execs Python server, connects socket, receives packets

### Python process loop

Single loop handling:

- Camera frame acquisition
- Optional resize to wire dimensions
- Landmark inference
- Packet send (metadata every cycle, frame based on `HAND_FRAME_SEND_EVERY`)

## Shared state and synchronization

In [turret.c](../turret.c):

- `state_mutex`: guards `pan`/`tilt` `ServoState`
- `hand_mutex`: guards hand detection and landmark-derived `HandState`
- `frame_mutex`: guards raw camera frame buffer for texture upload
- `serial_mutex`: serializes SDK reads/writes on one port

This prevents races between UI logic, feedback polling, and socket receiver updates.

## Servo discovery and startup

Startup probes multiple candidate ports and baud rates. If configured IDs are missing, the program scans IDs `1..252` and auto-selects up to two responding servos.

After discovery:

- Torque is enabled for available axes
- Goal positions are centered
- Feedback thread starts
- Hand thread remains off until hand mode is requested

## Control loops

### Manual mode

- Inputs from mouse/keyboard set pan/tilt goals
- Commands are clamped to `0..1023`
- Writes are sent only when changed enough (`SERVO_GOAL_DEADBAND_TICKS`)

### Hand mode

- Palm coordinates are transformed by linear calibration parameters (`m`, `b`) for each axis
- Mapped goals are clamped and sent at max speed (`1023`) while detected
- Calibration can be skipped or repeated

## Rendering pipeline

- Scene is rendered to a fixed virtual target (`1024x680`)
- Final output is scaled to current window size
- Camera frame, when available, is uploaded into a texture and overlaid in the aim pad region

## Failure handling strategy

- Socket disconnect moves hand state to idle loop
- Camera acquisition failures in Python trigger retries and eventual shutdown
- Unknown socket packet sizes are drained and ignored
- Servo read failures mark axis disconnected without crashing UI

## Latency accounting

Python stamps each metadata packet with microsecond timestamp. C computes one-way pipeline latency estimate:

- capture + inference + socket transfer + receiver processing

UI shows current, min, max, and exponential moving average.
