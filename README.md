# Dynamix Turret

Linux-focused pan/tilt turret controller for Dynamixel AX-12A servos with:

- High-rate native control loop and dashboard UI in C/C++ ([turret.c](turret.c))
- Hand landmark tracking in Python using OpenCV + MediaPipe ([hand_server.py](hand_server.py))
- Real-time telemetry, camera preview overlay, and latency measurement

This repository contains both runtime code and developer documentation.

## Documentation map

- Project/developer intro: [README.md](README.md)
- Development workflow: [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)
- Installation and dependencies: [docs/INSTALLATION.md](docs/INSTALLATION.md)
- End-user operation and UI behavior: [docs/USER_GUIDE.md](docs/USER_GUIDE.md)
- Internal architecture and threading model: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- Socket wire protocol reference: [docs/PROTOCOL.md](docs/PROTOCOL.md)
- Troubleshooting cookbook: [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)

## What this project does

At runtime, the system controls two servos (pan + tilt) and renders a full telemetry UI.

- Supports direct aiming with mouse and keyboard
- Supports optional hand-tracking mode with 4-point camera-to-servo calibration
- Streams camera frames into the C UI for overlay debugging
- Displays detected hand, confidence, detection rate, and latency metrics

## Repository layout

- Main controller process: [turret.c](turret.c)
- Hand-tracking process: [hand_server.py](hand_server.py)
- Build/run helper targets: [Makefile](Makefile)
- Python runtime dependencies: [requirements.txt](requirements.txt)
- Developer docs: [docs/](docs/)

## Hardware requirements

- 2x Dynamixel AX-12A servos on a shared bus
- USB Dynamixel adapter (U2D2 / USB2Dynamixel or compatible)
- Stable servo power supply appropriate for load
- Linux machine with a camera accessible via OpenCV

Default IDs expected by the controller:

- PAN: `3`
- TILT: `4`

If these IDs are not detected, the program attempts automatic discovery.

## Software stack

- C/C++ application compiled as GNU++20
- `raylib` for UI rendering
- Dynamixel SDK C library (`libdxl_x64_c`) for protocol 1.0 serial communication
- OpenCV C/C++ libraries linked by the main binary
- Python 3 + `mediapipe` + `opencv-python` for hand tracking

Dependency details and distro-specific install notes are in [docs/INSTALLATION.md](docs/INSTALLATION.md).

## Quick start

1. Install system packages and Python dependencies (see [docs/INSTALLATION.md](docs/INSTALLATION.md)).
2. Build:
  - `make`
3. Run:
  - `make run`
4. In the UI:
  - Move with mouse or arrow keys
  - Press `H` to enable hand mode
  - Complete 4-step calibration with `C`

## Runtime behavior summary

The controller probes likely serial ports and baud rates to locate servos.

- Port candidates: `/dev/ttyACM0`, `/dev/ttyUSB0`, `/dev/ttyACM1`, `/dev/ttyUSB1`
- Baud candidates: `1000000`, `57600`, `115200`, `2000000`

When hand mode is enabled:

- The C process starts [hand_server.py](hand_server.py)
- The Python process opens `/tmp/hand_track.sock`
- Hand metadata and optional RGB frames are sent as length-prefixed packets

Protocol details are documented in [docs/PROTOCOL.md](docs/PROTOCOL.md).

## Keyboard and mouse controls

- Left mouse drag in aim pad: set pan/tilt goal
- Arrow keys: nudge goal
- `Space`: center turret
- `T`: toggle servo torque
- `H`: toggle hand mode
- `C`: capture calibration point (during wizard)
- `Enter`: skip calibration wizard

See [docs/USER_GUIDE.md](docs/USER_GUIDE.md) for full interaction flow.

## Safety notes

- Keep clear of moving parts during calibration and testing.
- Confirm power supply and ground integrity before enabling torque.
- Start with low mechanical load and free movement range.
- Verify servo direction/mounting before unattended operation.

## Build and run targets

The [Makefile](Makefile) provides:

- `make` / `make all` - build `turret`
- `make run` - best-effort serial permission change + launch
- `make install-python-deps` - install Python packages
- `make clean` - remove generated artifacts
- `make help` - print target list

## Environment variables

Consumed by [hand_server.py](hand_server.py):

- `HAND_FRAME_SEND_EVERY`: send every Nth frame (`1` = all frames)
- `HAND_LANDMARKER_MODEL_PATH`: override path for downloaded `.task` model

## Common failure modes

- No servo response: permissions, power, wrong port, wrong baud, bus wiring
- Sometimes a quick scan/test in Dynamixel Wizard 2, then closing Wizard 2, helps the app detect servos again
- No camera frame: inaccessible camera device or OpenCV backend mismatch
- No landmarks: missing/incompatible MediaPipe install or model download issue

See [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) for diagnosis steps.

## License and release notes

No explicit license file is currently present in this repository. Add one before public redistribution.
