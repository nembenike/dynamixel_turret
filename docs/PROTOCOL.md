# Hand Socket Protocol Reference

This document defines the binary protocol exchanged between [hand_server.py](../hand_server.py) and [turret.c](../turret.c).

## Transport

- Unix domain stream socket
- Path: `/tmp/hand_track.sock`
- Message framing: `uint32` little-endian payload length prefix

Each frame on the stream is:

1. 4 bytes: payload length (`uint32`, little-endian)
2. N bytes: payload body

## Payload types

The receiver distinguishes payloads by exact size.

### 1) Hand metadata packet

Size must equal `sizeof(HandPacket)` in C and `struct.calcsize("=iiifi5i5ifQ")` in Python.

Layout (packed, little-endian, no alignment padding):

1. `int32 detected`
2. `int32 palm_x`
3. `int32 palm_y`
4. `float confidence`
5. `int32 finger_count`
6. `int32 finger_x[5]`
7. `int32 finger_y[5]`
8. `float palm_radius`
9. `uint64 timestamp_us`

Semantics:

- `detected`: `1` for hand present, `0` for idle packet
- `palm_x`, `palm_y`: palm center in camera pixel coordinates
- `confidence`: normalized confidence score (`0.0..1.0`)
- `finger_count`: bounded to max tracked points
- `finger_x/y`: fingertip coordinates for IDs 4/8/12/16/20
- `palm_radius`: geometric scale cue from palm to middle MCP
- `timestamp_us`: source-side monotonic timestamp for latency estimate

### 2) Raw camera frame packet

Size equals:

- `HAND_FRAME_WIDTH * HAND_FRAME_HEIGHT * HAND_FRAME_CHANNELS`
- Current default: `640 * 480 * 3` bytes (RGB)

Data is raw interleaved RGB8 without compression.

## Sender cadence

Per loop in Python:

- Always send one metadata packet
- Send one raw frame when `FRAME_SEND_EVERY` condition is met

`FRAME_SEND_EVERY=1` sends every frame.

## Compatibility requirements

Both files must stay synchronized:

- C struct in [turret.c](../turret.c)
- Python struct format in [hand_server.py](../hand_server.py)

Any field order/type/packing change requires coordinated updates on both ends.

## Error handling behavior

Receiver behavior in C:

- Unknown payload lengths are drained and ignored
- Read failures break receive loop and switch hand state to idle
- Partial frame packet reads are treated as disconnect/error
