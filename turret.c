/*
 * turret.c
 *
 *  Dynamixel AX-12A Pan/Tilt Turret Controller
 *  Protocol 1.0 implementation with raylib GUI
 *  Hand tracking via MediaPipe (hand_server.py over Unix socket)
 *
 *  Hardware:
 *   - 2× AX-12A Dynamixel servos
 *   - Connected via /dev/ttyACM0 @ 1,000,000 bps
 *   - ID 3 = PAN  (horizontal)
 *   - ID 4 = TILT (vertical)
 *
 *  Build:
 *   g++ turret.c -o turret -lraylib -lm -lpthread -ldl
 *
 *  Runtime deps:
 *   pip install mediapipe opencv-python
 *   (hand_server.py must be in the working directory or on PATH)
 */

#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#define _XOPEN_SOURCE   700

#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <pthread.h>
#include <math.h>
#include <time.h>

#include <raylib.h>
#include <raymath.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <dynamixel_sdk.h>
#ifdef __cplusplus
}
#endif

/* ── Control table addresses (AX-12A) ─────────────────────────────── */
#define ADDR_TORQUE_ENABLE      24
#define ADDR_GOAL_POSITION      30
#define ADDR_MOVING_SPEED       32
#define ADDR_PRESENT_POSITION   36
#define ADDR_PRESENT_SPEED      38
#define ADDR_PRESENT_LOAD       40
#define ADDR_VOLTAGE            42
#define ADDR_TEMPERATURE        43

#define PROTOCOL_VERSION        1.0

#define DEVICENAME              "/dev/ttyACM0"
#define BAUDRATE                1000000

#define DXL_ID_PAN              3
#define DXL_ID_TILT             4
#define BROADCAST_ID            0xFE

#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0

#define DXL_MINIMUM_POSITION_VALUE  0
#define DXL_MAXIMUM_POSITION_VALUE  1023
#define DXL_MIDDLE_POSITION_VALUE   511
#define MOVING_SPEED_DEFAULT        200

/* ── Screen / UI ───────────────────────────────────────────────────── */
#define SW 1024
#define SH 680

/* ── MediaPipe socket ──────────────────────────────────────────────── */
#define HAND_SOCKET_PATH        "/tmp/hand_track.sock"
#define HAND_SERVER_SCRIPT      "hand_server.py"
#define HAND_FRAME_WIDTH        640
#define HAND_FRAME_HEIGHT       480
#define HAND_FRAME_CHANNELS     3
#define HAND_FRAME_BYTES ((uint32_t)(HAND_FRAME_WIDTH * HAND_FRAME_HEIGHT * HAND_FRAME_CHANNELS))

/*
 * Wire format written by hand_server.py (little-endian, packed):
 *   struct HandPacket {
 *       int32_t  detected;
 *       int32_t  palm_x;
 *       int32_t  palm_y;
 *       float    confidence;
 *       int32_t  finger_count;
 *       int32_t  finger_x[5];
 *       int32_t  finger_y[5];
 *       float    palm_radius;
 *       uint64_t timestamp_us;    // microseconds for latency measurement
 *   };
 * Each packet is preceded by a uint32_t length field (= sizeof(HandPacket)).
 */
typedef struct __attribute__((packed)) {
    int32_t  detected;
    int32_t  palm_x;
    int32_t  palm_y;
    float    confidence;
    int32_t  finger_count;
    int32_t  finger_x[5];
    int32_t  finger_y[5];
    float    palm_radius;
    uint64_t timestamp_us;
} HandPacket;

/* ── Shared state ──────────────────────────────────────────────────── */
#define MAX_FINGERS_TRACKED 5

typedef struct {
    int     x, y;
    int     detected;
    float   confidence;
    float   detections_per_sec;
    int     finger_count;
    int     finger_x[MAX_FINGERS_TRACKED];
    int     finger_y[MAX_FINGERS_TRACKED];
    float   palm_radius;
    double  latency_ms;          // latency in milliseconds for last packet
    double  latency_min_ms;      // minimum latency seen
    double  latency_max_ms;      // maximum latency seen
    double  latency_avg_ms;      // running average latency
} HandState;

static HandState hand = { 320, 240, 0, 0.0f, 0.0f, 0, {0}, {0}, 0.0f, 0.0, 1e9, 0.0, 0.0 };
static pthread_mutex_t hand_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Camera frame pushed by Python server (640×480 RGB) */
typedef struct {
    uint8_t *data;
    int      width;
    int      height;
    int      channels;
} CameraFrame;

static CameraFrame camera_frame = { NULL, HAND_FRAME_WIDTH, HAND_FRAME_HEIGHT, HAND_FRAME_CHANNELS };
static pthread_mutex_t frame_mutex = PTHREAD_MUTEX_INITIALIZER;
static int camera_online = 0;

typedef struct {
    int     goal_pos;
    int     present_pos;
    int     present_spd;
    int     present_load;
    float   voltage;
    int     temperature;
    int     error;
    int     connected;
} ServoState;

static ServoState pan  = { DXL_MIDDLE_POSITION_VALUE, DXL_MIDDLE_POSITION_VALUE, 0, 0, 0, 0, 0, 0 };
static ServoState tilt = { DXL_MIDDLE_POSITION_VALUE, DXL_MIDDLE_POSITION_VALUE, 0, 0, 0, 0, 0, 0 };
static pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;

static int port_num = -1;
static pthread_mutex_t serial_mutex = PTHREAD_MUTEX_INITIALIZER;

static uint8_t dxl_id_pan  = DXL_ID_PAN;
static uint8_t dxl_id_tilt = DXL_ID_TILT;
static int dxl_pan_enabled  = 1;
static int dxl_tilt_enabled = 1;

static const char *active_device_name = DEVICENAME;
static int         active_baudrate    = BAUDRATE;

/* ── Helpers ───────────────────────────────────────────────────────── */
static int clampi(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static double monotonic_seconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

/* Full blocking recv */
static int recv_all(int fd, void *buf, size_t len)
{
    size_t got = 0;
    while (got < len) {
        ssize_t n = recv(fd, (char*)buf + got, len - got, 0);
        if (n <= 0) return -1;
        got += (size_t)n;
    }
    return 0;
}

/* ── Dynamixel helpers ─────────────────────────────────────────────── */
static int dxl_ping_servo(uint8_t id)
{
    int v = read1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_VOLTAGE);
    if (getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS) return 0;
    if (getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0) return 0;
    return (v >= 50 && v <= 180);
}

static int dxl_read_reg(uint8_t id, uint8_t reg, int dlen)
{
    int val = -1;
    pthread_mutex_lock(&serial_mutex);
    if (dlen == 1) val = read1ByteTxRx(port_num, PROTOCOL_VERSION, id, reg);
    else           val = read2ByteTxRx(port_num, PROTOCOL_VERSION, id, reg);
    if (getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS) val = -1;
    if (getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0)         val = -1;
    pthread_mutex_unlock(&serial_mutex);
    return val;
}

static void dxl_write_reg(uint8_t id, uint8_t reg, int val, int dlen)
{
    pthread_mutex_lock(&serial_mutex);
    if (dlen == 1) write1ByteTxRx(port_num, PROTOCOL_VERSION, id, reg, (uint8_t)val);
    else           write2ByteTxRx(port_num, PROTOCOL_VERSION, id, reg, (uint16_t)val);
    pthread_mutex_unlock(&serial_mutex);
}

static void torque_enable(uint8_t id, int on)
{
    pthread_mutex_lock(&serial_mutex);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_TORQUE_ENABLE,
                   on ? TORQUE_ENABLE : TORQUE_DISABLE);
    pthread_mutex_unlock(&serial_mutex);
}

/* ── Feedback thread ───────────────────────────────────────────────── */
static void *feedback_thread(void *arg)
{
    (void)arg;
    int div = 0;
    while (1) {
        int pp = dxl_pan_enabled  ? dxl_read_reg(dxl_id_pan,  ADDR_PRESENT_POSITION, 2) : -1;
        int tp = dxl_tilt_enabled ? dxl_read_reg(dxl_id_tilt, ADDR_PRESENT_POSITION, 2) : -1;

        int ps=-1, pl=-1, pv=-1, pt=-1;
        int ts=-1, tl=-1, tv=-1, tt=-1;

        if ((div++ % 5) == 0) {
            if (dxl_pan_enabled) {
                ps = dxl_read_reg(dxl_id_pan, ADDR_PRESENT_SPEED,  2);
                pl = dxl_read_reg(dxl_id_pan, ADDR_PRESENT_LOAD,   2);
                pv = dxl_read_reg(dxl_id_pan, ADDR_VOLTAGE,        1);
                pt = dxl_read_reg(dxl_id_pan, ADDR_TEMPERATURE,    1);
            }
            if (dxl_tilt_enabled) {
                ts = dxl_read_reg(dxl_id_tilt, ADDR_PRESENT_SPEED, 2);
                tl = dxl_read_reg(dxl_id_tilt, ADDR_PRESENT_LOAD,  2);
                tv = dxl_read_reg(dxl_id_tilt, ADDR_VOLTAGE,       1);
                tt = dxl_read_reg(dxl_id_tilt, ADDR_TEMPERATURE,   1);
            }
        }

        pthread_mutex_lock(&state_mutex);
        if (pp >= 0) { pan.present_pos  = pp; pan.connected  = 1; } else { pan.connected  = 0; }
        if (ps >= 0)   pan.present_spd  = ps & 0x3FF;
        if (pl >= 0)   pan.present_load = pl & 0x3FF;
        if (pv >= 0)   pan.voltage      = pv / 10.0f;
        if (pt >= 0)   pan.temperature  = pt;

        if (tp >= 0) { tilt.present_pos  = tp; tilt.connected = 1; } else { tilt.connected = 0; }
        if (ts >= 0)   tilt.present_spd  = ts & 0x3FF;
        if (tl >= 0)   tilt.present_load = tl & 0x3FF;
        if (tv >= 0)   tilt.voltage      = tv / 10.0f;
        if (tt >= 0)   tilt.temperature  = tt;
        pthread_mutex_unlock(&state_mutex);

        usleep(50000);
    }
    return NULL;
}

/* ── MediaPipe hand tracking thread ────────────────────────────────── */
static void *hand_tracking_thread(void *arg)
{
    (void)arg;
    int sock = -1;

    /* ── 1. Launch hand_server.py ──────────────────────────────────── */
    pid_t pid = fork();
    if (pid == 0) {
        /* child: suppress stdout so it doesn't pollute the terminal;
           keep stderr so we see Python tracebacks */
        int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) { dup2(devnull, STDOUT_FILENO); close(devnull); }
        execlp("python3", "python3", HAND_SERVER_SCRIPT, NULL);
        /* exec only returns on error */
        fprintf(stderr, "[HAND] Failed to exec python3 %s: %s\n",
                HAND_SERVER_SCRIPT, strerror(errno));
        _exit(1);
    }
    if (pid < 0) {
        fprintf(stderr, "[HAND] fork() failed: %s\n", strerror(errno));
        goto idle;
    }
    fprintf(stderr, "[HAND] Launched %s (pid %d)\n", HAND_SERVER_SCRIPT, (int)pid);

    /* ── 2. Connect to the Unix socket ────────────────────────────── */
    sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock < 0) { fprintf(stderr, "[HAND] socket(): %s\n", strerror(errno)); goto idle; }

    {
        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, HAND_SOCKET_PATH, sizeof(addr.sun_path) - 1);

        int connected = 0;
        for (int i = 0; i < 60; i++) {          /* up to 6 s */
            if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) == 0) {
                connected = 1;
                break;
            }
            usleep(100000);
        }
        if (!connected) {
            fprintf(stderr, "[HAND] Could not connect to %s after 6 s\n", HAND_SOCKET_PATH);
            close(sock);
            goto idle;
        }
    }

    camera_online = 1;
    fprintf(stderr, "[HAND] Connected to MediaPipe hand server\n");

    /* ── 3. Receive loop ──────────────────────────────────────────── */
    {
        double dps_t0    = monotonic_seconds();
        int    dps_count = 0;
        float  dps_val   = 0.0f;

        while (1) {
            /* 4-byte length prefix */
            uint32_t pkt_len = 0;
            if (recv_all(sock, &pkt_len, 4) < 0) break;

            /* Hand metadata packet */
            if (pkt_len == sizeof(HandPacket)) {
                HandPacket pkt;
                if (recv_all(sock, &pkt, sizeof(HandPacket)) < 0) break;

                /* Calculate latency in milliseconds */
                double now_us = monotonic_seconds() * 1e6;
                double latency_ms = (now_us - (double)pkt.timestamp_us) / 1000.0;
                if (latency_ms < 0.0) latency_ms = 0.0;  /* Handle clock skew */

                if (pkt.detected) dps_count++;
                double now = monotonic_seconds();
                if (now - dps_t0 >= 1.0) {
                    dps_val   = (float)(dps_count / (now - dps_t0));
                    dps_count = 0;
                    dps_t0    = now;
                }

                pthread_mutex_lock(&hand_mutex);
                hand.detected   = pkt.detected;
                hand.x          = pkt.palm_x;
                hand.y          = pkt.palm_y;
                hand.confidence = pkt.confidence;
                hand.palm_radius = pkt.palm_radius;
                hand.detections_per_sec = dps_val;
                hand.latency_ms = latency_ms;
                
                /* Update latency statistics */
                if (latency_ms < hand.latency_min_ms) {
                    hand.latency_min_ms = latency_ms;
                }
                if (latency_ms > hand.latency_max_ms) {
                    hand.latency_max_ms = latency_ms;
                }
                /* Running average: exponential moving average */
                if (hand.latency_avg_ms == 0.0) {
                    hand.latency_avg_ms = latency_ms;
                } else {
                    hand.latency_avg_ms = 0.9 * hand.latency_avg_ms + 0.1 * latency_ms;
                }
                
                int fc = pkt.finger_count;
                if (fc < 0) fc = 0;
                if (fc > MAX_FINGERS_TRACKED) fc = MAX_FINGERS_TRACKED;
                hand.finger_count = fc;
                for (int i = 0; i < fc; i++) {
                    hand.finger_x[i] = pkt.finger_x[i];
                    hand.finger_y[i] = pkt.finger_y[i];
                }
                pthread_mutex_unlock(&hand_mutex);
            } else if (pkt_len == HAND_FRAME_BYTES) {
                pthread_mutex_lock(&frame_mutex);
                if (!camera_frame.data) {
                    camera_frame.data = (uint8_t*)malloc((size_t)HAND_FRAME_BYTES);
                    camera_frame.width = HAND_FRAME_WIDTH;
                    camera_frame.height = HAND_FRAME_HEIGHT;
                    camera_frame.channels = HAND_FRAME_CHANNELS;
                }

                if (camera_frame.data) {
                    if (recv_all(sock, camera_frame.data, HAND_FRAME_BYTES) < 0) {
                        pthread_mutex_unlock(&frame_mutex);
                        break;
                    }
                } else {
                    pthread_mutex_unlock(&frame_mutex);
                    uint8_t drain[256];
                    uint32_t left = pkt_len;
                    while (left > 0) {
                        uint32_t chunk = left < 256 ? left : 256;
                        if (recv_all(sock, drain, chunk) < 0) goto socket_done;
                        left -= chunk;
                    }
                    continue;
                }
                pthread_mutex_unlock(&frame_mutex);
            } else {
                /* Unknown packet size – drain and ignore */
                uint8_t drain[256];
                uint32_t left = pkt_len;
                while (left > 0) {
                    uint32_t chunk = left < 256 ? left : 256;
                    if (recv_all(sock, drain, chunk) < 0) goto socket_done;
                    left -= chunk;
                }
            }
        }
    }

socket_done:
    close(sock);
    camera_online = 0;
    fprintf(stderr, "[HAND] Disconnected from hand server\n");

idle:
    while (1) {
        pthread_mutex_lock(&hand_mutex);
        hand.detected           = 0;
        hand.confidence         = 0.0f;
        hand.detections_per_sec = 0.0f;
        hand.finger_count       = 0;
        hand.palm_radius        = 0.0f;
        pthread_mutex_unlock(&hand_mutex);
        usleep(100000);
    }
    return NULL;
}

/* ── Servo utilities ───────────────────────────────────────────────── */
static float pos_to_deg(int pos)    { return pos * (300.0f / 1023.0f); }
static int   norm_to_pos(float n)
{
    int p = (int)(n * 1023.0f + 0.5f);
    return clampi(p, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE);
}
static int map_linear_to_pos(float x, float m, float b)
{
    int p = (int)(m * x + b + 0.5f);
    return clampi(p, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE);
}

#define SERVO_GOAL_DEADBAND_TICKS 1

static void send_servo_targets_if_needed(int gp, int gt, int speed, int force)
{
    static int last_gp = -1, last_gt = -1, last_speed = -1;

    gp    = clampi(gp,    DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE);
    gt    = clampi(gt,    DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE);
    speed = clampi(speed, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE);

    int goal_changed =
        (last_gp < 0 || last_gt < 0 ||
         abs(gp - last_gp) >= SERVO_GOAL_DEADBAND_TICKS ||
         abs(gt - last_gt) >= SERVO_GOAL_DEADBAND_TICKS);
    int speed_changed = (last_speed != speed);

    if (!force && !goal_changed && !speed_changed) return;

    if (force || goal_changed) {
        if (dxl_pan_enabled)  dxl_write_reg(dxl_id_pan,  ADDR_GOAL_POSITION, gp, 2);
        if (dxl_tilt_enabled) dxl_write_reg(dxl_id_tilt, ADDR_GOAL_POSITION, gt, 2);
        last_gp = gp; last_gt = gt;
    }
    if (force || speed_changed) {
        if (dxl_pan_enabled)  dxl_write_reg(dxl_id_pan,  ADDR_MOVING_SPEED, speed, 2);
        if (dxl_tilt_enabled) dxl_write_reg(dxl_id_tilt, ADDR_MOVING_SPEED, speed, 2);
        last_speed = speed;
    }
}

/* ── GUI helpers ───────────────────────────────────────────────────── */
static void draw_bar(int x, int y, int w, int h,
                     float val, float vmax,
                     Color bar_col, Color bg_col, const char *label)
{
    DrawRectangle(x, y, w, h, bg_col);
    int fw = (int)(val / vmax * w);
    if (fw > 0) DrawRectangle(x, y, fw, h, bar_col);
    DrawRectangleLines(x, y, w, h, ColorAlpha(WHITE, 0.3f));
    DrawText(label, x + 4, y + 2, 12, WHITE);
}

/* ══════════════════════════════════════════════════════════════════════
   main
   ══════════════════════════════════════════════════════════════════════ */
int main(void)
{
    fprintf(stderr, "[DXL] Startup\n");

    /* ── Probe serial ports / baud rates ──────────────────────────── */
    const char *port_candidates[] = {
        DEVICENAME, "/dev/ttyUSB0", "/dev/ttyACM1", "/dev/ttyUSB1"
    };
    const int baud_candidates[] = { BAUDRATE, 57600, 115200, 2000000 };
    const int nports = (int)(sizeof(port_candidates)/sizeof(port_candidates[0]));
    const int nbauds = (int)(sizeof(baud_candidates)/sizeof(baud_candidates[0]));

    int pan_present  = 0;
    int tilt_present = 0;
    int packet_inited = 0;

    for (int i = 0; i < nports && !(pan_present || tilt_present); i++) {
        const char *dev = port_candidates[i];
        fprintf(stderr, "[DXL] Trying %s\n", dev);
        port_num = portHandler(dev);
        if (port_num < 0) continue;
        if (!packet_inited) { packetHandler(); packet_inited = 1; }
        if (!openPort(port_num)) continue;

        for (int b = 0; b < nbauds; b++) {
            int baud = baud_candidates[b];
            if (!setBaudRate(port_num, baud)) continue;

            pan_present  = dxl_ping_servo(dxl_id_pan);
            tilt_present = dxl_ping_servo(dxl_id_tilt);

            /* Auto-discover IDs if defaults not found */
            if (!pan_present && !tilt_present) {
                uint8_t found[2] = {0, 0};
                int fc = 0;
                for (uint8_t id = 1; id <= 252 && fc < 2; id++) {
                    if (id == BROADCAST_ID) continue;
                    if (dxl_ping_servo(id)) found[fc++] = id;
                }
                if (fc >= 1) {
                    dxl_id_pan   = found[0];
                    dxl_id_tilt  = (fc >= 2) ? found[1] : DXL_ID_TILT;
                    pan_present  = 1;
                    tilt_present = (fc >= 2) ? 1 : 0;
                    fprintf(stderr, "[DXL] Auto-selected IDs: PAN=%u TILT=%u\n",
                            (unsigned)dxl_id_pan, (unsigned)dxl_id_tilt);
                }
            }

            fprintf(stderr, "[DXL] Probe %s @ %d: PAN=%s TILT=%s\n",
                    dev, baud,
                    pan_present  ? "OK" : "MISS",
                    tilt_present ? "OK" : "MISS");

            if (pan_present || tilt_present) {
                active_device_name = dev;
                active_baudrate    = baud;
                break;
            }
        }
        if (!(pan_present || tilt_present)) { closePort(port_num); port_num = -1; }
    }

    if (port_num < 0 || (!pan_present && !tilt_present)) {
        fprintf(stderr, "[DXL] No responding servos found\n");
        return 1;
    }
    dxl_pan_enabled  = pan_present  ? 1 : 0;
    dxl_tilt_enabled = tilt_present ? 1 : 0;
    fprintf(stderr, "[DXL] Using %s @ %d bps\n", active_device_name, active_baudrate);

    if (dxl_pan_enabled)  torque_enable(dxl_id_pan,  TORQUE_ENABLE);
    if (dxl_tilt_enabled) torque_enable(dxl_id_tilt, TORQUE_ENABLE);

    pan.goal_pos  = DXL_MIDDLE_POSITION_VALUE;
    tilt.goal_pos = DXL_MIDDLE_POSITION_VALUE;
    send_servo_targets_if_needed(DXL_MIDDLE_POSITION_VALUE,
                                 DXL_MIDDLE_POSITION_VALUE,
                                 MOVING_SPEED_DEFAULT, 1);

    /* ── Start feedback thread ────────────────────────────────────── */
    pthread_t fb_tid;
    pthread_create(&fb_tid, NULL, feedback_thread, NULL);
    pthread_detach(fb_tid);

    /* Hand thread starts lazily when H is first pressed */
    pthread_t ht_tid;
    int hand_thread_started = 0;

    /* ── Window ───────────────────────────────────────────────────── */
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(SW, SH, "AX-12A Turret Controller");
    MaximizeWindow();
    SetTargetFPS(60);

    RenderTexture2D ui_target = LoadRenderTexture(SW, SH);

    /* ── UI state ─────────────────────────────────────────────────── */
    int speed         = MOVING_SPEED_DEFAULT;
    int dragging_speed = 0;
    int hand_mode     = 0;

    /* Camera→servo linear calibration */
    float pan_m  = -1023.0f / 640.0f;
    float pan_b  =  1023.0f;
    float tilt_m = -1023.0f / 480.0f;
    float tilt_b =  1023.0f;
    int calibration_active = 0;
    int calibration_done   = 0;
    int calib_step         = 0;
    int calib_pan_left_x   = 0, calib_pan_right_x  = 0;
    int calib_pan_left_pos = DXL_MIDDLE_POSITION_VALUE;
    int calib_pan_right_pos= DXL_MIDDLE_POSITION_VALUE;
    int calib_tilt_top_y   = 0, calib_tilt_bottom_y = 0;
    int calib_tilt_top_pos = DXL_MIDDLE_POSITION_VALUE;
    int calib_tilt_bottom_pos = DXL_MIDDLE_POSITION_VALUE;

    /* Camera preview texture */
    Texture2D cam_tex = {
        .id = 0,
        .width = 0,
        .height = 0,
        .mipmaps = 0,
        .format = 0
    };
    int       cam_tex_w = 0, cam_tex_h = 0;

    const int PAD_X = 40, PAD_Y = 80;
    const int PAD_W = 560, PAD_H = 440;

    /* ── Main loop ────────────────────────────────────────────────── */
    while (!WindowShouldClose())
    {
        /* Scale mouse from actual window to virtual 1024×680 */
        Vector2 mouse_raw = GetMousePosition();
        int screen_w = GetScreenWidth();
        int screen_h = GetScreenHeight();
        if (screen_w <= 0) screen_w = SW;
        if (screen_h <= 0) screen_h = SH;
        Vector2 mouse = {
            mouse_raw.x * ((float)SW / (float)screen_w),
            mouse_raw.y * ((float)SH / (float)screen_h)
        };

        /* H – toggle hand mode */
        if (IsKeyPressed(KEY_H)) {
            hand_mode = !hand_mode;
            if (hand_mode) {
                if (!hand_thread_started) {
                    pthread_create(&ht_tid, NULL, hand_tracking_thread, NULL);
                    pthread_detach(ht_tid);
                    hand_thread_started = 1;
                }
                calibration_active = 1;
                calib_step = 0;
            }
        }

        /* Aim pad click/drag (disabled while hand mode is running) */
        if ((!hand_mode || calibration_active) && IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
            if (mouse.x >= PAD_X && mouse.x <= PAD_X + PAD_W &&
                mouse.y >= PAD_Y && mouse.y <= PAD_Y + PAD_H)
            {
                float nx = (mouse.x - PAD_X) / (float)PAD_W;
                float ny = 1.0f - (mouse.y - PAD_Y) / (float)PAD_H;
                int gp = norm_to_pos(nx);
                int gt = norm_to_pos(ny);
                pthread_mutex_lock(&state_mutex);
                pan.goal_pos  = gp;
                tilt.goal_pos = gt;
                pthread_mutex_unlock(&state_mutex);
                send_servo_targets_if_needed(gp, gt, speed, 0);
            }
        }

        /* Arrow-key nudge */
        const int NUDGE = 10;
        int gp, gt;
        pthread_mutex_lock(&state_mutex);
        gp = pan.goal_pos; gt = tilt.goal_pos;
        pthread_mutex_unlock(&state_mutex);
        int changed = 0;
        if ((!hand_mode || calibration_active) && IsKeyDown(KEY_LEFT))  { gp -= NUDGE; changed = 1; }
        if ((!hand_mode || calibration_active) && IsKeyDown(KEY_RIGHT)) { gp += NUDGE; changed = 1; }
        if ((!hand_mode || calibration_active) && IsKeyDown(KEY_UP))    { gt -= NUDGE; changed = 1; }
        if ((!hand_mode || calibration_active) && IsKeyDown(KEY_DOWN))  { gt += NUDGE; changed = 1; }
        if (IsKeyPressed(KEY_SPACE)) { gp = DXL_MIDDLE_POSITION_VALUE; gt = DXL_MIDDLE_POSITION_VALUE; changed = 1; }
        if (changed) {
            gp = clampi(gp, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE);
            gt = clampi(gt, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE);
            pthread_mutex_lock(&state_mutex);
            pan.goal_pos  = gp;
            tilt.goal_pos = gt;
            pthread_mutex_unlock(&state_mutex);
            send_servo_targets_if_needed(gp, gt, speed, 0);
        }

        /* Speed slider */
        const int SLX = 610, SLY = 100, SLH = 360;
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) &&
            mouse.x >= SLX-10 && mouse.x <= SLX+30 &&
            mouse.y >= SLY    && mouse.y <= SLY+SLH)
            dragging_speed = 1;
        if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) dragging_speed = 0;
        if (dragging_speed) {
            float t = 1.0f - (mouse.y - SLY) / (float)SLH;
            t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
            speed = (int)(t * 1023);
        }

        /* T – torque toggle */
        if (IsKeyPressed(KEY_T)) {
            static int torque_on = 1;
            torque_on = !torque_on;
            if (dxl_pan_enabled)  torque_enable(dxl_id_pan,  torque_on);
            if (dxl_tilt_enabled) torque_enable(dxl_id_tilt, torque_on);
            if (torque_on) {
                int gnow, tnow;
                pthread_mutex_lock(&state_mutex);
                gnow = pan.goal_pos; tnow = tilt.goal_pos;
                pthread_mutex_unlock(&state_mutex);
                send_servo_targets_if_needed(gnow, tnow, speed, 1);
            }
        }

        /* Calibration wizard */
        if (hand_mode && calibration_active) {
            HandState  hs_now;
            ServoState sp_now, st_now;
            pthread_mutex_lock(&hand_mutex);  hs_now = hand; pthread_mutex_unlock(&hand_mutex);
            pthread_mutex_lock(&state_mutex); sp_now = pan;  st_now = tilt; pthread_mutex_unlock(&state_mutex);

            if (IsKeyPressed(KEY_ENTER)) calibration_active = 0;

            if (IsKeyPressed(KEY_C) && hs_now.detected) {
                switch (calib_step) {
                case 0:
                    calib_pan_left_x   = hs_now.x;
                    calib_pan_left_pos = sp_now.present_pos;
                    calib_step = 1; break;
                case 1:
                    calib_pan_right_x   = hs_now.x;
                    calib_pan_right_pos = sp_now.present_pos;
                    calib_step = 2; break;
                case 2:
                    calib_tilt_top_y   = hs_now.y;
                    calib_tilt_top_pos = st_now.present_pos;
                    calib_step = 3; break;
                case 3:
                    calib_tilt_bottom_y   = hs_now.y;
                    calib_tilt_bottom_pos = st_now.present_pos;
                    {
                        int dx = calib_pan_right_x - calib_pan_left_x;
                        int dy = calib_tilt_bottom_y - calib_tilt_top_y;
                        if (abs(dx) > 8 && abs(dy) > 8) {
                            pan_m  = (float)(calib_pan_right_pos  - calib_pan_left_pos)  / (float)dx;
                            pan_b  = (float)calib_pan_left_pos    - pan_m  * (float)calib_pan_left_x;
                            tilt_m = (float)(calib_tilt_bottom_pos - calib_tilt_top_pos) / (float)dy;
                            tilt_b = (float)calib_tilt_top_pos    - tilt_m * (float)calib_tilt_top_y;
                            calibration_done = 1;
                        }
                    }
                    calibration_active = 0;
                    calib_step = 0;
                    break;
                }
            }
        }

        /* Hand tracking → servo */
        if (hand_mode && !calibration_active) {
            HandState hs_cmd;
            pthread_mutex_lock(&hand_mutex);
            hs_cmd = hand;
            pthread_mutex_unlock(&hand_mutex);

            if (hs_cmd.detected) {
                int hgp = map_linear_to_pos((float)hs_cmd.x, pan_m,  pan_b);
                int hgt = map_linear_to_pos((float)hs_cmd.y, tilt_m, tilt_b);
                hgp = clampi(hgp, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE);
                hgt = clampi(hgt, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE);
                pthread_mutex_lock(&state_mutex);
                pan.goal_pos  = hgp;
                tilt.goal_pos = hgt;
                pthread_mutex_unlock(&state_mutex);
                send_servo_targets_if_needed(hgp, hgt, 1023, 0);
            }
        }

        /* Snapshot for rendering */
        ServoState sp, st;
        HandState  hs;
        pthread_mutex_lock(&state_mutex); sp = pan;  st = tilt; pthread_mutex_unlock(&state_mutex);
        pthread_mutex_lock(&hand_mutex);  hs = hand;            pthread_mutex_unlock(&hand_mutex);

        /* ── Render ───────────────────────────────────────────────── */
        BeginTextureMode(ui_target);
        ClearBackground((Color){14, 18, 28, 255});

        /* Title bar */
        DrawRectangle(0, 0, SW, 40, (Color){20, 28, 48, 255});
        DrawText("AX-12A TURRET CONTROLLER", 14, 10, 20, (Color){0, 200, 255, 255});
        DrawText(TextFormat("Hand: %s (%d,%d)  PORT:%s @ %dbps",
                 hs.detected ? "DETECTED" : "IDLE",
                 hs.x, hs.y, active_device_name, active_baudrate),
                 340, 12, 14, (Color){120, 160, 200, 255});
        DrawText(TextFormat("DET/s: %.1f", hs.detections_per_sec), 705, 28, 11,
                 hs.detections_per_sec > 0.1f ? (Color){120,255,120,255} : (Color){170,190,220,255});
        DrawText(TextFormat("LAT: %.1f ms (min:%.1f max:%.1f avg:%.1f)", 
                 hs.latency_ms, hs.latency_min_ms, hs.latency_max_ms, hs.latency_avg_ms),
                 705, 40, 11,
                 hs.latency_ms < 50.0 ? (Color){120,255,120,255} : 
                 hs.latency_ms < 100.0 ? (Color){255,255,120,255} : (Color){255,120,120,255});
        DrawText(TextFormat("CAM:%s", camera_online ? "LIVE" : "OFF"), 840, 12, 14,
                 camera_online ? (Color){100,255,100,255} : (Color){255,120,120,255});
        DrawText(TextFormat("CAL:%s", calibration_done ? "OK" : "NEEDED"), 910, 28, 11,
                 calibration_done ? (Color){100,255,100,255} : (Color){255,200,120,255});

        /* Aim pad */
        DrawRectangle(PAD_X, PAD_Y, PAD_W, PAD_H, (Color){18, 24, 40, 255});
        DrawRectangleLines(PAD_X, PAD_Y, PAD_W, PAD_H, (Color){0, 150, 255, 80});
        for (int i = 1; i < 4; i++) {
            DrawLine(PAD_X+PAD_W*i/4, PAD_Y, PAD_X+PAD_W*i/4, PAD_Y+PAD_H, (Color){40,60,100,100});
            DrawLine(PAD_X, PAD_Y+PAD_H*i/4, PAD_X+PAD_W, PAD_Y+PAD_H*i/4, (Color){40,60,100,100});
        }
        DrawText("PAN →",  PAD_X+PAD_W-50, PAD_Y+PAD_H+6, 12, (Color){80,120,180,200});
        DrawText("↑ TILT", PAD_X-36,       PAD_Y+2,       12, (Color){80,120,180,200});

        /* Goal crosshair */
        int gx = PAD_X + (int)(sp.goal_pos / 1023.0f * PAD_W);
        int gy = PAD_Y + PAD_H - (int)(st.goal_pos / 1023.0f * PAD_H);
        DrawLine(gx, PAD_Y, gx, PAD_Y+PAD_H, (Color){0,200,255,120});
        DrawLine(PAD_X, gy, PAD_X+PAD_W, gy, (Color){0,200,255,120});
        DrawCircleLines(gx, gy, 14, (Color){0,200,255,200});
        DrawCircle(gx, gy, 5,  (Color){0,200,255,255});

        /* Present dot */
        int px2 = PAD_X + (int)(sp.present_pos / 1023.0f * PAD_W);
        int py2 = PAD_Y + PAD_H - (int)(st.present_pos / 1023.0f * PAD_H);
        DrawCircle(px2, py2, 7, (Color){255,80,60,220});
        DrawLine(gx, gy, px2, py2, (Color){255,150,60,80});

        /* Legend */
        DrawCircle(PAD_X+10, PAD_Y+PAD_H-30, 5, (Color){0,200,255,255});
        DrawText("Goal",    PAD_X+18, PAD_Y+PAD_H-36, 12, WHITE);
        DrawCircle(PAD_X+70, PAD_Y+PAD_H-30, 5, (Color){255,80,60,220});
        DrawText("Present", PAD_X+78, PAD_Y+PAD_H-36, 12, WHITE);

        /* ── Camera frame overlay ─────────────────────────────────── */
        if (hand_mode) {
            int cam_w = 0, cam_h = 0, have_frame = 0;

            pthread_mutex_lock(&frame_mutex);
            if (camera_frame.data && camera_frame.width > 0 && camera_frame.height > 0) {
                if (cam_tex.id == 0 ||
                    cam_tex_w != camera_frame.width ||
                    cam_tex_h != camera_frame.height)
                {
                    if (cam_tex.id != 0) UnloadTexture(cam_tex);
                    Image img = {
                        .data     = camera_frame.data,
                        .width    = camera_frame.width,
                        .height   = camera_frame.height,
                        .mipmaps  = 1,
                        .format   = PIXELFORMAT_UNCOMPRESSED_R8G8B8
                    };
                    cam_tex   = LoadTextureFromImage(img);
                    cam_tex_w = camera_frame.width;
                    cam_tex_h = camera_frame.height;
                } else {
                    UpdateTexture(cam_tex, camera_frame.data);
                }
                cam_w = camera_frame.width;
                cam_h = camera_frame.height;
                have_frame = 1;
            }
            pthread_mutex_unlock(&frame_mutex);

            if (have_frame) {
                DrawTexturePro(
                    cam_tex,
                    (Rectangle){0, 0, (float)cam_tex.width, (float)cam_tex.height},
                    (Rectangle){(float)PAD_X, (float)PAD_Y, (float)PAD_W, (float)PAD_H},
                    (Vector2){0, 0}, 0.0f, WHITE);

                /* Hand overlay on camera */
                if (hs.detected) {
                    int cx = PAD_X + (int)(hs.x / (float)cam_w * PAD_W);
                    int cy = PAD_Y + (int)(hs.y / (float)cam_h * PAD_H);
                    float pr = hs.palm_radius * (PAD_W / (float)cam_w);
                    if (pr < 8.0f) pr = 8.0f;

                    /* Palm circle */
                    DrawCircleLines(cx, cy, pr, (Color){0, 255, 0, 255});
                    DrawCircle(cx, cy, 5, (Color){120, 255, 120, 255});

                    /* Skeleton lines + fingertip dots */
                    for (int i = 0; i < hs.finger_count; i++) {
                        int fx = PAD_X + (int)(hs.finger_x[i] / (float)cam_w * PAD_W);
                        int fy = PAD_Y + (int)(hs.finger_y[i] / (float)cam_h * PAD_H);
                        DrawLine(cx, cy, fx, fy, (Color){255, 210, 80, 240});
                        DrawCircle(fx, fy, 5, (Color){255, 220, 120, 255});
                        DrawCircleLines(fx, fy, 9, (Color){255, 240, 170, 230});
                    }

                    DrawText("PALM", cx - 16, cy - 22, 10, (Color){100, 255, 100, 255});
                    DrawText(TextFormat("F:%d", hs.finger_count),
                             cx - 10, cy + (int)pr + 4, 10, (Color){255, 220, 120, 240});
                }

                /* Turret look-point (where the servo currently points in camera space) */
                if (fabsf(pan_m) > 1e-5f && fabsf(tilt_m) > 1e-5f) {
                    int lx_cam = clampi((int)((sp.present_pos - pan_b)  / pan_m  + 0.5f), 0, cam_w-1);
                    int ly_cam = clampi((int)((st.present_pos - tilt_b) / tilt_m + 0.5f), 0, cam_h-1);
                    int lx = PAD_X + (int)(lx_cam / (float)cam_w * PAD_W);
                    int ly = PAD_Y + (int)(ly_cam / (float)cam_h * PAD_H);
                    DrawCircleLines(lx, ly, 14, (Color){255, 80, 80, 255});
                    DrawLine(lx-8, ly, lx+8, ly, (Color){255, 80, 80, 255});
                    DrawLine(lx, ly-8, lx, ly+8, (Color){255, 80, 80, 255});
                    DrawText("AIM", lx+10, ly-8, 10, (Color){255, 120, 120, 255});
                }

                /* Calibration overlay */
                if (calibration_active) {
                    DrawRectangle(PAD_X+8, PAD_Y+8, PAD_W-16, 86, (Color){0,0,0,160});
                    const char *step_msg = "";
                    if      (calib_step == 0) step_msg = "STEP 1/4: Move hand LEFT.   Aim turret, press C.";
                    else if (calib_step == 1) step_msg = "STEP 2/4: Move hand RIGHT.  Aim turret, press C.";
                    else if (calib_step == 2) step_msg = "STEP 3/4: Move hand TOP.    Aim turret, press C.";
                    else if (calib_step == 3) step_msg = "STEP 4/4: Move hand BOTTOM. Aim turret, press C.";
                    DrawText("HAND CALIBRATION", PAD_X+18, PAD_Y+16, 18, (Color){255,220,120,255});
                    DrawText(step_msg, PAD_X+18, PAD_Y+42, 14, WHITE);
                    DrawText("ENTER = skip (use previous calibration)",
                             PAD_X+18, PAD_Y+64, 12, (Color){180,200,220,255});
                }
            } else {
                DrawRectangle(PAD_X, PAD_Y, PAD_W, PAD_H, (Color){5, 8, 16, 220});
                DrawText("No camera frame – waiting for hand_server.py",
                         PAD_X+18, PAD_Y+16, 18, (Color){255,160,120,255});
                DrawText("Ensure hand_server.py is in the working directory and pip install mediapipe",
                         PAD_X+18, PAD_Y+48, 13, (Color){220,180,160,255});
            }

            /* Hand mode badge */
            DrawText("[HAND MODE]", SW-190, 10, 14,
                     hs.detected ? (Color){100,255,100,255} : (Color){255,150,100,255});
            if (!hs.detected)
                DrawText("(no hand)", SW-190, 28, 11, (Color){255,100,100,200});
        }

        /* Hand dot on aim pad (always, when not in camera mode) */
        if (!hand_mode && hs.detected) {
            int fw = camera_frame.width  > 0 ? camera_frame.width  : 640;
            int fh = camera_frame.height > 0 ? camera_frame.height : 480;
            int hx = PAD_X + (int)(hs.x / (float)fw * PAD_W);
            int hy = PAD_Y + PAD_H - (int)(hs.y / (float)fh * PAD_H);
            float pr = hs.palm_radius * (PAD_W / (float)fw);
            if (pr < 6.0f) pr = 6.0f;
            DrawCircleLines(hx, hy, pr, (Color){100,255,100,220});
            DrawCircle(hx, hy, 5, (Color){100,255,140,255});
            for (int i = 0; i < hs.finger_count; i++) {
                int fx = PAD_X + (int)(hs.finger_x[i] / (float)fw * PAD_W);
                int fy = PAD_Y + PAD_H - (int)(hs.finger_y[i] / (float)fh * PAD_H);
                DrawLine(hx, hy, fx, fy, (Color){255,210,80,220});
                DrawCircle(fx, fy, 4, (Color){255,220,120,255});
            }
        }

        /* Speed slider */
        {
            float spd_frac = speed / 1023.0f;
            int sl_fill = (int)(spd_frac * SLH);
            DrawRectangle(SLX, SLY, 20, SLH, (Color){20,30,50,255});
            DrawRectangle(SLX, SLY+SLH-sl_fill, 20, sl_fill, (Color){0,160,255,200});
            DrawRectangleLines(SLX, SLY, 20, SLH, (Color){0,100,180,150});
            DrawText("SPD", SLX-2, SLY-16, 12, (Color){120,160,200,200});
            DrawText(TextFormat("%d", speed), SLX-4, SLY+SLH+4, 12, WHITE);
        }

        /* Servo panels */
        const int PNX = 650, PNY = 80, PNW = 360;

        /* PAN */
        DrawRectangle(PNX, PNY, PNW, 240, (Color){18,26,44,255});
        DrawRectangleLines(PNX, PNY, PNW, 240,
            sp.connected ? (Color){0,180,255,120} : (Color){200,50,50,120});
        DrawText(TextFormat("PAN  (ID %u)%s", (unsigned)dxl_id_pan,
                 sp.connected ? "" : "  [NO RESP]"),
                 PNX+8, PNY+8, 14,
                 sp.connected ? (Color){0,200,255,255} : (Color){255,80,80,255});
        DrawText(TextFormat("Goal:    %4d  (%.1f°)", sp.goal_pos,    pos_to_deg(sp.goal_pos)),    PNX+8, PNY+30, 13, WHITE);
        DrawText(TextFormat("Present: %4d  (%.1f°)", sp.present_pos, pos_to_deg(sp.present_pos)), PNX+8, PNY+48, 13, WHITE);
        draw_bar(PNX+8, PNY+72,  PNW-16, 14, (float)sp.present_pos,  1023.0f, (Color){0,160,255,200},   (Color){30,40,60,200}, "Pos");
        draw_bar(PNX+8, PNY+92,  PNW-16, 14, (float)sp.present_spd,  1023.0f, (Color){80,220,120,200},  (Color){30,40,60,200}, "Spd");
        draw_bar(PNX+8, PNY+112, PNW-16, 14, (float)sp.present_load, 1023.0f, (Color){255,180,0,200},   (Color){30,40,60,200}, "Load");
        DrawText(TextFormat("Voltage:  %.1f V", sp.voltage),    PNX+8, PNY+136, 13, sp.voltage     > 12.5f ? (Color){255,80,80,255} : WHITE);
        DrawText(TextFormat("Temp:     %d °C",  sp.temperature), PNX+8, PNY+154, 13, sp.temperature > 60   ? (Color){255,80,80,255} : WHITE);
        {
            Vector2 c = {PNX+PNW-55, PNY+200};
            float r = 38;
            DrawCircleSector(c, r, -30, 210, 40, (Color){30,44,70,200});
            float rad_g = (-30.0f + sp.goal_pos    / 1023.0f * 240.0f) * DEG2RAD;
            float rad_p = (-30.0f + sp.present_pos / 1023.0f * 240.0f) * DEG2RAD;
            DrawLineEx(c, (Vector2){c.x+cosf(rad_g)*r, c.y+sinf(rad_g)*r}, 2, (Color){0,200,255,220});
            DrawLineEx(c, (Vector2){c.x+cosf(rad_p)*r, c.y+sinf(rad_p)*r}, 2, (Color){255,80,60,200});
            DrawText("PAN", (int)(c.x-14), (int)(c.y-8), 11, (Color){120,160,200,180});
        }

        /* TILT */
        DrawRectangle(PNX, PNY+250, PNW, 240, (Color){18,26,44,255});
        DrawRectangleLines(PNX, PNY+250, PNW, 240,
            st.connected ? (Color){0,180,255,120} : (Color){200,50,50,120});
        DrawText(TextFormat("TILT (ID %u)%s", (unsigned)dxl_id_tilt,
                 st.connected ? "" : "  [NO RESP]"),
                 PNX+8, PNY+258, 14,
                 st.connected ? (Color){0,200,255,255} : (Color){255,80,80,255});
        DrawText(TextFormat("Goal:    %4d  (%.1f°)", st.goal_pos,    pos_to_deg(st.goal_pos)),    PNX+8, PNY+280, 13, WHITE);
        DrawText(TextFormat("Present: %4d  (%.1f°)", st.present_pos, pos_to_deg(st.present_pos)), PNX+8, PNY+298, 13, WHITE);
        draw_bar(PNX+8, PNY+322, PNW-16, 14, (float)st.present_pos,  1023.0f, (Color){0,160,255,200},  (Color){30,40,60,200}, "Pos");
        draw_bar(PNX+8, PNY+342, PNW-16, 14, (float)st.present_spd,  1023.0f, (Color){80,220,120,200}, (Color){30,40,60,200}, "Spd");
        draw_bar(PNX+8, PNY+362, PNW-16, 14, (float)st.present_load, 1023.0f, (Color){255,180,0,200},  (Color){30,40,60,200}, "Load");
        DrawText(TextFormat("Voltage:  %.1f V", st.voltage),    PNX+8, PNY+386, 13, st.voltage     > 12.5f ? (Color){255,80,80,255} : WHITE);
        DrawText(TextFormat("Temp:     %d °C",  st.temperature), PNX+8, PNY+404, 13, st.temperature > 60   ? (Color){255,80,80,255} : WHITE);
        {
            Vector2 c = {PNX+PNW-55, PNY+450};
            float r = 38;
            DrawCircleSector(c, r, -30, 210, 40, (Color){30,44,70,200});
            float rad_g = (-30.0f + st.goal_pos    / 1023.0f * 240.0f) * DEG2RAD;
            float rad_p = (-30.0f + st.present_pos / 1023.0f * 240.0f) * DEG2RAD;
            DrawLineEx(c, (Vector2){c.x+cosf(rad_g)*r, c.y+sinf(rad_g)*r}, 2, (Color){0,200,255,220});
            DrawLineEx(c, (Vector2){c.x+cosf(rad_p)*r, c.y+sinf(rad_p)*r}, 2, (Color){255,80,60,200});
            DrawText("TILT", (int)(c.x-16), (int)(c.y-8), 11, (Color){120,160,200,180});
        }

        /* Help bar */
        DrawRectangle(0, SH-36, SW, 36, (Color){14,20,36,255});
        DrawText("CLICK pad to aim  |  ARROWS nudge  |  SPACE center  |  T torque  |  H hand mode  |  C capture calib",
                 8, SH-26, 12, (Color){80,120,180,200});

        EndTextureMode();

        BeginDrawing();
        ClearBackground(BLACK);
        DrawTexturePro(
            ui_target.texture,
            (Rectangle){0, 0, (float)SW, (float)-SH},
            (Rectangle){0, 0, (float)screen_w, (float)screen_h},
            (Vector2){0, 0}, 0.0f, WHITE);
        EndDrawing();
    }

    /* Cleanup */
    UnloadRenderTexture(ui_target);
    if (cam_tex.id != 0) UnloadTexture(cam_tex);
    pthread_mutex_lock(&frame_mutex);
    free(camera_frame.data);
    camera_frame.data = NULL;
    pthread_mutex_unlock(&frame_mutex);

    if (dxl_pan_enabled)  torque_enable(dxl_id_pan,  TORQUE_DISABLE);
    if (dxl_tilt_enabled) torque_enable(dxl_id_tilt, TORQUE_DISABLE);
    closePort(port_num);
    CloseWindow();
    return 0;
}