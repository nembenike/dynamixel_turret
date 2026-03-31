#!/usr/bin/env python3
"""
hand_server.py  –  MediaPipe hand tracking server for turret.c

Streams hand landmark data + raw camera frames over a Unix domain socket.
Each message = 4-byte uint32 length prefix + HandPacket binary struct.

Wire format (little-endian, packed, matches HandPacket in turret.c):
    int32   detected        (1 = hand visible, 0 = no hand)
    int32   palm_x          (pixels, in camera coordinate space)
    int32   palm_y
    float   confidence      (landmark visibility of wrist, 0.0–1.0)
    int32   finger_count    (always 5 when detected, fingertip IDs 4,8,12,16,20)
    int32   finger_x[5]
    int32   finger_y[5]
    float   palm_radius     (pixels – wrist to middle-MCP distance, useful for scale)

Install:
    pip install mediapipe opencv-python

Run:
    python3 hand_server.py          # turret.c forks this automatically
"""

import os
import socket
import struct
import sys
import time
import math
import urllib.request
from pathlib import Path

import cv2
import mediapipe as mp

# ── Config ───────────────────────────────────────────────────────────
SOCKET_PATH  = "/tmp/hand_track.sock"
FRAME_W      = 640
FRAME_H      = 480
TARGET_FPS   = 60

# MediaPipe Hands settings
MODEL_COMPLEXITY        = 0      # 0 = lite (faster), 1 = full (more accurate)
MIN_DETECTION_CONF      = 0.60
MIN_TRACKING_CONF       = 0.55
MAX_NUM_HANDS           = 1
FRAME_SEND_EVERY        = int(os.environ.get("HAND_FRAME_SEND_EVERY", "1"))

# Tasks model (used when mp.solutions is unavailable, e.g. some Python 3.13 wheels)
HAND_LANDMARKER_MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/"
    "hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
)
HAND_LANDMARKER_MODEL_PATH = os.environ.get(
    "HAND_LANDMARKER_MODEL_PATH",
    str(Path.home() / ".cache" / "dynamix_turret" / "hand_landmarker.task"),
)

# MediaPipe landmark indices
WRIST          = 0
THUMB_TIP      = 4
INDEX_TIP      = 8
MIDDLE_TIP     = 12
RING_TIP       = 16
PINKY_TIP      = 20
FINGERTIP_IDS  = [THUMB_TIP, INDEX_TIP, MIDDLE_TIP, RING_TIP, PINKY_TIP]
MCP_IDS        = [1, 5, 9, 13, 17]   # knuckle bases – used for palm centre

# Struct layout must match HandPacket __attribute__((packed)) in turret.c
# "=iiifi5i5ifQ"  (= → native byte order / no alignment padding)
# Q = uint64_t for timestamp_us (microseconds since epoch)
STRUCT_FMT  = "=iiifi5i5ifQ"
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)

# ── Socket setup ─────────────────────────────────────────────────────
if os.path.exists(SOCKET_PATH):
    os.unlink(SOCKET_PATH)

server_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
server_sock.bind(SOCKET_PATH)
server_sock.listen(1)
print(f"[HAND_SERVER] Listening on {SOCKET_PATH}  (struct size={STRUCT_SIZE})", flush=True)

conn, _ = server_sock.accept()
print("[HAND_SERVER] turret.c connected", flush=True)
# Best-effort low-latency socket tuning.
# TCP_NODELAY is not valid for AF_UNIX on some Linux kernels.
try:
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
except OSError:
    pass

def ensure_tasks_model(path: str) -> str:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    if not p.exists() or p.stat().st_size == 0:
        print(f"[HAND_SERVER] Downloading model to {p}", flush=True)
        urllib.request.urlretrieve(HAND_LANDMARKER_MODEL_URL, str(p))
    return str(p)

# ── Camera ───────────────────────────────────────────────────────────
cap = None
for idx in range(6):
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    if cap.isOpened():
        print(f"[HAND_SERVER] Camera opened at index {idx}", flush=True)
        break
    cap.release()
    cap = cv2.VideoCapture(idx)
    if cap.isOpened():
        print(f"[HAND_SERVER] Camera opened at index {idx} (no V4L2)", flush=True)
        break
    cap.release()
    cap = None

if cap is None or not cap.isOpened():
    print("[HAND_SERVER] ERROR: could not open any camera", flush=True, file=sys.stderr)
    sys.exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
cap.set(cv2.CAP_PROP_FPS,          TARGET_FPS)
cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)          # always grab the freshest frame

actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"[HAND_SERVER] Camera resolution: {actual_w}×{actual_h}", flush=True)
print(f"[HAND_SERVER] Wire frame size: {FRAME_W}×{FRAME_H} RGB", flush=True)

# ── MediaPipe ────────────────────────────────────────────────────────
hands_sol = None
hand_landmarker = None
mp_mode = "none"

if hasattr(mp, "solutions"):
    mp_hands  = mp.solutions.hands
    hands_sol = mp_hands.Hands(
        static_image_mode        = False,
        max_num_hands            = MAX_NUM_HANDS,
        model_complexity         = MODEL_COMPLEXITY,
        min_detection_confidence = MIN_DETECTION_CONF,
        min_tracking_confidence  = MIN_TRACKING_CONF,
    )
    mp_mode = "solutions"
    print("[HAND_SERVER] Using MediaPipe solutions API", flush=True)
else:
    from mediapipe.tasks import python as mp_tasks_python
    from mediapipe.tasks.python import vision as mp_tasks_vision

    model_path = ensure_tasks_model(HAND_LANDMARKER_MODEL_PATH)
    opts = mp_tasks_vision.HandLandmarkerOptions(
        base_options=mp_tasks_python.BaseOptions(model_asset_path=model_path),
        running_mode=mp_tasks_vision.RunningMode.VIDEO,
        num_hands=MAX_NUM_HANDS,
        min_hand_detection_confidence=MIN_DETECTION_CONF,
        min_hand_presence_confidence=MIN_TRACKING_CONF,
        min_tracking_confidence=MIN_TRACKING_CONF,
    )
    hand_landmarker = mp_tasks_vision.HandLandmarker.create_from_options(opts)
    mp_mode = "tasks"
    print("[HAND_SERVER] Using MediaPipe tasks API", flush=True)

# ── Helpers ──────────────────────────────────────────────────────────
def pack_idle():
    """Return a 'no hand' packet."""
    timestamp_us = int(time.monotonic() * 1e6)
    return struct.pack(STRUCT_FMT,
                       0, 0, 0, 0.0, 0,
                       0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0,
                       0.0, timestamp_us)

def pack_hand(lm, w, h, conf=1.0):
    """
    Build HandPacket from a MediaPipe NormalizedLandmarkList.
    w, h are the actual frame pixel dimensions.
    """
    # Palm centre = mean of wrist + 4 MCP knuckles (robust, not affected by
    # fingers extending or curling)
    palm_pts = [lm[WRIST]] + [lm[i] for i in MCP_IDS]
    px = int(sum(p.x for p in palm_pts) / len(palm_pts) * w)
    py = int(sum(p.y for p in palm_pts) / len(palm_pts) * h)

    # Confidence: provided by caller (API dependent)
    conf = float(conf)
    if conf < 0.0:
        conf = 0.0
    if conf > 1.0:
        conf = 1.0

    # Fingertips (always report all 5; caller decides which to use)
    fx = [int(lm[i].x * w) for i in FINGERTIP_IDS]
    fy = [int(lm[i].y * h) for i in FINGERTIP_IDS]

    # Palm radius: Euclidean distance from palm centre to middle-MCP (landmark 9)
    # This gives a good scale estimate independent of hand orientation
    mcx = int(lm[9].x * w)
    mcy = int(lm[9].y * h)
    palm_radius = math.hypot(px - mcx, py - mcy)

    # Timestamp in microseconds for latency measurement
    timestamp_us = int(time.monotonic() * 1e6)

    return struct.pack(STRUCT_FMT,
                       1, px, py, conf, 5,
                       fx[0], fx[1], fx[2], fx[3], fx[4],
                       fy[0], fy[1], fy[2], fy[3], fy[4],
                       palm_radius, timestamp_us)

def send_packet(data: bytes):
    """Send length-prefixed packet; raise on broken pipe."""
    header = struct.pack("=I", len(data))
    conn.sendall(header + data)

# ── Main loop ────────────────────────────────────────────────────────
frame_interval = 1.0 / TARGET_FPS
t_next = time.monotonic()
drop_count = 0
frame_idx = 0

try:
    while True:
        ok, bgr = cap.read()
        if not ok:
            drop_count += 1
            if drop_count > 60:
                print("[HAND_SERVER] Camera stream lost", flush=True, file=sys.stderr)
                break
            time.sleep(0.02)
            continue
        drop_count = 0

        # Normalize to fixed wire size so turret.c can decode raw frame packets.
        if bgr.shape[1] != FRAME_W or bgr.shape[0] != FRAME_H:
            bgr = cv2.resize(bgr, (FRAME_W, FRAME_H), interpolation=cv2.INTER_LINEAR)

        # Convert BGR→RGB (MediaPipe expects RGB)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False          # avoid a copy inside MediaPipe

        if mp_mode == "solutions":
            result = hands_sol.process(rgb)
            if result.multi_hand_landmarks:
                lm_list = result.multi_hand_landmarks[0].landmark
                conf = 1.0
                if result.multi_handedness:
                    conf = float(result.multi_handedness[0].classification[0].score)
                pkt = pack_hand(lm_list, FRAME_W, FRAME_H, conf)
            else:
                pkt = pack_idle()
        else:
            mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            ts_ms = int(time.monotonic() * 1000.0)
            result = hand_landmarker.detect_for_video(mp_img, ts_ms)
            if result.hand_landmarks:
                lm_list = result.hand_landmarks[0]
                conf = 1.0
                if result.handedness and result.handedness[0]:
                    conf = float(result.handedness[0][0].score)
                pkt = pack_hand(lm_list, FRAME_W, FRAME_H, conf)
            else:
                pkt = pack_idle()

        send_packet(pkt)
        frame_idx += 1
        if FRAME_SEND_EVERY <= 1 or (frame_idx % FRAME_SEND_EVERY) == 0:
            send_packet(rgb.tobytes())

        # Throttle to TARGET_FPS (skip sleep if we're already behind)
        now = time.monotonic()
        t_next += frame_interval
        sleep_for = t_next - now
        if sleep_for > 0.0:
            time.sleep(sleep_for)
        else:
            t_next = now   # re-sync if we're lagging

except (BrokenPipeError, ConnectionResetError):
    print("[HAND_SERVER] Client disconnected", flush=True)
except KeyboardInterrupt:
    print("[HAND_SERVER] Interrupted", flush=True)
finally:
    if hands_sol is not None:
        hands_sol.close()
    if hand_landmarker is not None:
        hand_landmarker.close()
    cap.release()
    conn.close()
    server_sock.close()
    if os.path.exists(SOCKET_PATH):
        os.unlink(SOCKET_PATH)
    print("[HAND_SERVER] Shutdown complete", flush=True)