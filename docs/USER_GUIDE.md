# User Guide

This document describes normal operation of the turret application from an operator perspective.

## Startup flow

1. Launch the app (`make run` or `./turret`).
2. The app probes serial ports/baud rates and enables torque on discovered servos.
3. The UI opens with:
   - Aim pad on the left
   - Speed slider and servo telemetry panels on the right
   - Status information in the title bar

## Core controls

- Mouse drag in aim pad: set pan/tilt goal continuously
- Arrow keys: increment/decrement goal in fixed steps
- `Space`: return both axes to center (position 511)
- `T`: torque toggle for discovered servos
- `H`: toggle hand-tracking mode

## UI interpretation

### Aim pad

- Cyan crosshair and ring: commanded goal position
- Red dot: present servo feedback position
- Vector from goal to present shows tracking error

### Speed slider

- Vertical slider range maps to servo speed register values `0..1023`
- Higher values command faster movement response

### Servo panels

Each panel shows:

- Goal and present position (ticks + converted degrees)
- Present speed and load bars
- Voltage and temperature readings
- Connection status (`[NO RESP]` when not responding)

## Hand mode workflow

Press `H` to enable hand mode.

### First-time or recalibration use

A 4-step wizard appears in camera overlay mode:

1. Move hand to LEFT target, align turret, press `C`
2. Move hand to RIGHT target, align turret, press `C`
3. Move hand to TOP target, align turret, press `C`
4. Move hand to BOTTOM target, align turret, press `C`

After capture, linear calibration coefficients are computed. Press `Enter` to skip the wizard and retain previous mapping.

### During active hand control

- Palm center drives pan/tilt goal through the calibrated linear mapping
- Fingertip positions are drawn for visual debugging
- Palm radius is shown as a scale cue

## Status metrics

Top bar fields include:

- `Hand`: detection state and current palm coordinates
- `DET/s`: detection updates per second
- `LAT`: current/min/max/average end-to-end latency
- `CAM`: camera stream status
- `CAL`: calibration state

## Recommended operating procedure

1. Confirm free mechanical movement before enabling hand mode.
2. Start in manual mouse mode and verify direction correctness.
3. Run hand calibration with wide, distinct left/right/top/bottom positions.
4. Observe latency; if high, reduce load and adjust frame-send rate.

## Known behavior notes

- Hand thread starts lazily (first `H` press).
- If hand process disconnects, status falls back to idle and camera goes offline.
- Servo commands use change deadbanding to reduce unnecessary writes.
