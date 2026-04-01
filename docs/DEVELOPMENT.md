# Development Guide

This document describes how to work on the codebase safely and efficiently.

Related docs:

- Installation: [docs/INSTALLATION.md](INSTALLATION.md)
- User behavior and controls: [docs/USER_GUIDE.md](USER_GUIDE.md)
- Internal architecture: [docs/ARCHITECTURE.md](ARCHITECTURE.md)
- Socket protocol: [docs/PROTOCOL.md](PROTOCOL.md)
- Troubleshooting: [docs/TROUBLESHOOTING.md](TROUBLESHOOTING.md)

## Codebase overview

- Main controller/UI: [turret.c](../turret.c)
- Python hand-tracking server: [hand_server.py](../hand_server.py)
- Build targets and linker settings: [Makefile](../Makefile)
- Python dependencies: [requirements.txt](../requirements.txt)

## Build workflow

- Build native app: `make`
- Build + run: `make run`
- Install Python deps: `make install-python-deps`
- Clean artifacts: `make clean`

If dependency paths differ from defaults, override `INCLUDE`, `LIBDIR`, and `LIBS` in the make invocation.

## Runtime architecture summary

Two processes communicate over a Unix socket:

1. `turret` (C/C++)
   - Servo control, GUI, socket client, telemetry
2. `hand_server.py` (Python)
   - Camera capture, landmark inference, packet sender

The main process has separate threads for feedback polling and hand packet reception. Shared state is protected with dedicated mutexes.

## Editing rules and compatibility constraints

### 1) Keep hand packet definitions synchronized

When changing hand metadata schema:

- Update `HandPacket` in [turret.c](../turret.c)
- Update `STRUCT_FMT` and packing order in [hand_server.py](../hand_server.py)
- Update [docs/PROTOCOL.md](PROTOCOL.md)

Any mismatch causes corrupted decoding.

### 2) Keep frame geometry assumptions synchronized

`HAND_FRAME_WIDTH`, `HAND_FRAME_HEIGHT`, and channel count are hard assumptions in both processes.

If changed:

- Update C constants and byte-size checks
- Update Python resize/send behavior
- Re-validate UI overlay mapping and performance

### 3) Respect thread safety boundaries

- Servo state -> `state_mutex`
- Hand state -> `hand_mutex`
- Camera frame buffer -> `frame_mutex`
- SDK serial transactions -> `serial_mutex`

Do not read/write shared fields outside their lock domains.

## Typical feature-change workflows

### Add new hand metadata field

1. Add field to C `HandPacket` and `HandState` as needed.
2. Add value to Python packet construction.
3. Bump docs and UI rendering logic.
4. Validate packet size and runtime behavior.

### Adjust servo behavior

1. Modify command logic in main loop (`goal` computation).
2. Reuse `send_servo_targets_if_needed()` to avoid write spam.
3. Confirm clamping to `0..1023` remains intact.
4. Validate no regression in manual and hand modes.

### Modify calibration model

Current mapping is linear per axis (`pos = m*x + b`).

If changing model complexity:

- Update calibration capture flow and persistent state
- Update reverse mapping used for overlay aim marker
- Document operator steps in [docs/USER_GUIDE.md](USER_GUIDE.md)

## Debugging tips

- Run C app and Python server independently during isolation testing.
- Watch stderr output for startup, socket, and camera errors.
- Use latency metrics in UI to detect regressions.
- Keep a minimal reproducible setup (single camera, no extra consumers).

## Code quality checklist before commit

1. Build succeeds cleanly.
2. Manual aiming works.
3. Hand mode starts and calibrates.
4. Telemetry updates for both axes.
5. No packet mismatch warnings/errors.
6. Documentation updated for any protocol or control changes.

## Future improvement candidates

- Persist calibration coefficients across restarts
- Add configurable port/baud/IDs via CLI flags
- Split `turret.c` into modules for maintainability
- Add automated smoke tests for packet schema compatibility
