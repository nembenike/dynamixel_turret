# Development Notes

## Architecture

The runtime is split into two processes:

1. `turret` (C/C++)
   - Dynamixel control and telemetry
   - GUI rendering via raylib
   - Spawns and connects to `hand_server.py`
2. `hand_server.py` (Python)
   - Camera capture via OpenCV
   - Hand landmark extraction via MediaPipe
   - Sends length-prefixed packets over Unix socket

## Wire Protocol

Each message on `/tmp/hand_track.sock` starts with:

- `uint32` payload length (little-endian)

Payload is either:

- `HandPacket` metadata struct
- raw RGB frame bytes (`640*480*3`)

`HandPacket` fields are documented in both source files and must remain in sync.
