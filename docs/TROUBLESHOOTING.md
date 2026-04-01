# Troubleshooting

Use this guide for diagnosing build, runtime, serial, camera, and hand-tracking issues.

## Build problems

### Missing headers or libraries

Symptoms:

- Compile/link errors for `raylib`, `opencv`, or `dynamixel_sdk`

Actions:

1. Install matching development packages.
2. Verify include/library paths in [Makefile](../Makefile) (`INCLUDE`, `LIBDIR`, `LIBS`).
3. Rebuild with `make clean && make`.

### Python dependency install failures

Symptoms:

- `pip` fails for `mediapipe` or `opencv-python`

Actions:

1. Use supported Python version for available wheels.
2. Upgrade pip/setuptools/wheel.
3. Reinstall via `make install-python-deps`.

## Servo communication issues

### No responding servos found

Symptoms:

- Startup exits with no detected pan/tilt

Actions:

1. Verify power to servos and bus wiring.
2. Verify adapter enumerates in `/dev`.
3. Check user permissions for serial node.
4. Confirm expected baud/ID configuration.
5. Disconnect other software that may hold the serial port.
6. If detection still fails, open Dynamixel Wizard 2, scan/test the servos once, close Wizard 2, then start this app again (this can recover communication state on some adapters/firmware).

### One axis missing

Symptoms:

- One panel shows `[NO RESP]`

Actions:

1. Check individual servo ID and bus continuity.
2. Confirm no duplicate IDs on bus.
3. Swap servo connectors to isolate cable/device faults.

## Camera and hand tracking issues

### Hand mode enabled but camera stays offline

Symptoms:

- `CAM:OFF` or waiting-for-frame message

Actions:

1. Run `python3 hand_server.py` directly and inspect stderr.
2. Ensure camera is not busy in another process.
3. Test multiple camera indices if needed.
4. Verify OpenCV can open camera on your backend.

### Hand never detected

Symptoms:

- `Hand: IDLE` continuously, low `DET/s`

Actions:

1. Improve lighting and keep hand fully visible.
2. Ensure frame rate is stable (CPU not overloaded).
3. Confirm MediaPipe initialized (watch Python startup logs).
4. Recalibrate with clear left/right/top/bottom poses.

### Model download or tasks API issues

Symptoms:

- Errors around `.task` model path/download

Actions:

1. Check network access for initial download.
2. Pre-create writable cache directory.
3. Set `HAND_LANDMARKER_MODEL_PATH` to a known writable location.

## Performance and latency issues

### High latency in UI metrics

Symptoms:

- `LAT` frequently high or unstable

Actions:

1. Lower camera load or close competing applications.
2. Increase `HAND_FRAME_SEND_EVERY` to reduce frame bandwidth.
3. Use a faster CPU or lower concurrent system load.

### Jittery tracking

Actions:

1. Re-run calibration.
2. Keep hand movement within camera field center.
3. Ensure servo mechanics are not binding.

## Runtime process/socket issues

### Socket connection failed

Symptoms:

- C logs cannot connect to `/tmp/hand_track.sock`

Actions:

1. Ensure Python process starts successfully.
2. Remove stale socket file if process crashed.
3. Confirm filesystem permissions for `/tmp`.

## Safety escalation checklist

If behavior is unsafe (unexpected fast movement, oscillation, overheating):

1. Disable torque immediately (`T`) or cut servo power.
2. Inspect temperature and load telemetry.
3. Check calibration validity and mechanical stop limits.
4. Re-test in manual mode at lower speed before re-enabling hand mode.
