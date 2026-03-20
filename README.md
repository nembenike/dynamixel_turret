# Dynamix Turret

Pan/tilt turret controller for **AX-12A Dynamixel servos** with:
- Native C/C++ control loop + GUI (`raylib`)
- Live hand tracking via Python + MediaPipe (`hand_server.py`)
- Real-time telemetry (position/speed/load/voltage/temp)

This project is currently Linux-focused.

## Hardware

- 2× Dynamixel AX-12A servos
- USB2Dynamixel/U2D2/compatible interface
- Servos on a shared Dynamixel bus
- Camera supported by OpenCV

Default servo IDs in the app:
- `PAN = 3`
- `TILT = 4`

If those IDs are not found, the app attempts simple auto-discovery.

## Software Requirements

### System libraries (Ubuntu/Debian example)

Install build tools and C/C++ dependencies:

- `g++`, `make`
- `raylib`
- `opencv` (core/videoio/imgproc headers + libs)
- Dynamixel SDK C library (`libdxl_x64_c`)

### Python requirements

Use Python 3.10+ and install:

```bash
python3 -m pip install -r requirements.txt
```

## Build

```bash
make
```

Binary output: `./turret`

## Run

```bash
make run
```

The app tries common serial ports automatically (`/dev/ttyACM0`, `/dev/ttyUSB0`, ...).

If your USB adapter appears on another port, you can still run manually:

```bash
./turret
```

## Controls

- **Mouse drag** in aim pad: set target pan/tilt
- **Arrow keys**: nudge target
- **Space**: center turret
- **T**: torque on/off
- **H**: toggle hand mode
- **C** (during calibration): capture calibration point
- **Enter** (during calibration): skip calibration

## Hand Mode Notes

- `turret` launches `hand_server.py` automatically
- Hand tracking data is streamed over a Unix socket (`/tmp/hand_track.sock`)
- Camera frames are also streamed for in-app overlay
- On some MediaPipe builds, a `.task` model may be downloaded automatically to:
  - `~/.cache/dynamix_turret/hand_landmarker.task`

## Typical First-Time Setup

1. Wire servos and power safely.
2. Confirm your user can access serial devices (`dialout` group, udev rules, or temporary chmod).
3. Install Python dependencies (`make install-python-deps`).
4. Build (`make`).
5. Run (`make run`).
6. Press **H** and complete 4-step hand calibration with **C**.

## Troubleshooting

### No servo response

- Check power and bus wiring
- Verify serial adapter path and permissions
- Ensure baud rate is correct (app probes common rates)
- Sometimes running the DynamixelWizard, using the servos, then closing the Wizard makes the servos cooperate.

### No camera / hand detection

- Confirm camera works with OpenCV
- Verify `mediapipe` and `opencv-python` are installed
- Run `python3 hand_server.py` directly to inspect Python-side errors

### Build fails with missing libraries

Install missing dev packages for `raylib`, OpenCV, and Dynamixel SDK, then rebuild.

## Project Layout

- `turret.c` – main controller + GUI + serial + socket client
- `hand_server.py` – MediaPipe camera tracker + socket server
- `Makefile` – build/run helper targets
- `requirements.txt` – Python dependencies

## Publishing Checklist

Before pushing:

- Ensure local artifacts are removed: `make clean`
- Confirm `.gitignore` excludes virtual envs and cache files
- Optionally add a license file and release notes
