"""
VAT — PRISM VisPy Viewer  (POC)
===============================
Native-OpenGL live viewer for the PRISM map + robot avatar, built on **VisPy**
(GPU point scatter, no shaders to write, 10^5–10^6 points at 60 fps). Replaces the
earlier Rerun (frozen stream) and Open3D (finicky live-update) attempts.

It renders:
  * the **robot block** at the client-predicted pose (netcode dead-reckoning
    between samples), green when VGGT-corrected / amber when coasting;
  * the four **legs** from ``/lowstate`` forward kinematics + the selfie-stick;
  * the camera **trajectory**;
  * the PRISM **point cloud**, STREAMED. In the default ``STREAM_MODE=snapshot`` the
    server pushes a full (compressed, 16-bit-quantised + zlib) snapshot per submap and
    each one *replaces* the local cloud (``snapshot_sync.SnapshotSync``), so it stays
    aligned and never accumulates stale/duplicated blocks — pairs with reset-each-batch
    mapping. ``STREAM_MODE=blocks`` selects the DEPRECATED diff-based block sync
    (``block_sync.BlockSync``). The selected sync is polled off the GL thread.
  * a **latency HUD** (top-left): pose age / rate / capture→display e2e / fix
    quality, and cloud point-count / age / rate.

Metric scale is anchored ONCE at map start (first frame's camera height) and
carried by the overlap chain, so the streamed snapshots share one consistent frame.

In-window keys: ``1`` force re-fetch · ``R`` reset map · ``F`` refit view ·
``,``/``.`` yaw ∓5° · ``/`` yaw 0 · ``N``/``M`` point size ∓.

Usage
-----
  cd client && uv sync
  uv run python prism_viewer.py --snapshot
  ZENOH_ROUTER=tcp/<ip>:7447 uv run python prism_viewer.py --snapshot
"""

from __future__ import annotations

import os
import sys
import json
import time
import logging
import argparse
import glob
import threading
from collections import deque

import numpy as np
from vispy import app, scene

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "common"))
sys.path.insert(0, os.path.join(_ROOT, "robot", "docker"))
import vat_protocol as proto  # noqa: E402
from vat_protocol import (  # noqa: E402
    quat_identity, quat_normalize, quat_mul, quat_slerp, integrate_pose)
from vat_telemetry import ThroughputMeter, ClockOffsetEstimator  # noqa: E402
import zenoh  # noqa: E402
from kinematics import RobotStateTracker, LowStateTracker, LEG_ORDER  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))         # client/ (block_sync)
from block_sync import BlockSync  # noqa: E402
from snapshot_sync import SnapshotSync  # noqa: E402
from vat_cloudbuffer import IncrementalCloud  # noqa: E402
from urdf_robot import URDFRobot  # noqa: E402

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="[%(asctime)s] [%(levelname)s] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("prism-viewer")

ZENOH_ROUTER  = os.environ.get("ZENOH_ROUTER",  "tcp/127.0.0.1:7447")
ROBOT_NAME    = os.environ.get("ROBOT_NAME",    "go2")
SERVER_PREFIX = os.environ.get("SERVER_PREFIX", "server/prism")
# Cloud transport, must match the server: "blocks" (diff-based delta sync, default —
# small per-update payloads so the pose stream isn't starved) or "snapshot" (whole-map
# replace, simplest). See server mapping_config.STREAM_MODE.
STREAM_MODE   = os.environ.get("STREAM_MODE", "blocks").strip().lower()
RENDER_HZ     = float(os.environ.get("RENDER_HZ", "60.0"))
# Cloud rebuild+GPU-upload rate, DECOUPLED from RENDER_HZ. The pose predictor steps and
# the avatar redraws every render tick (RENDER_HZ) — cheap — but the whole-cloud set_data()
# GPU upload is the expensive per-tick op on the single-process/GIL-bound client and, at
# 60 Hz, it starves the render thread AND the pose callbacks (both latencies spiked together
# in telemetry). Coalescing it to a lower rate keeps the pose stream smooth; block deltas
# accumulate losslessly in the store between uploads. 0 = every tick (old behaviour).
CLOUD_RENDER_HZ = float(os.environ.get("CLOUD_RENDER_HZ", "10.0"))
STALE_S       = float(os.environ.get("POSE_STALE_S", "0.5"))
DECAY_S       = float(os.environ.get("POSE_DECAY_S", "1.0"))
SMOOTH_TAU    = float(os.environ.get("POSE_SMOOTH_TAU", "0.08"))
# Extra look-ahead (s) added to the dead-reckoning horizon to hide transport lag:
# the avatar is otherwise always ~transport-latency behind reality. Dial it up to
# match the observed lag (or the HUD's pose age). 0 = predict only from receipt.
POSE_LOOKAHEAD_S = float(os.environ.get("POSE_LOOKAHEAD_S", "0.0"))
# Anti-rubber-band: render the avatar this far in the PAST and INTERPOLATE between
# buffered poses (smooth, no overshoot) instead of extrapolating into the future
# (which overshoots on jitter → snaps back = rubber-banding). Set a touch above the
# pose inter-arrival jitter. 0 = pure extrapolation (old behaviour). Net visual lag
# ≈ POSE_RENDER_DELAY_S − POSE_LOOKAHEAD_S.
POSE_RENDER_DELAY_S = float(os.environ.get("POSE_RENDER_DELAY_S", "0.10"))
# ── Low-latency ADAPTIVE rendering ───────────────────────────────────────────
# The render delay is no longer fixed: it tracks measured inter-arrival JITTER, so on
# a clean link it shrinks toward MIN (extrapolate to ~now → lowest latency) and on a
# bursty link it grows toward MAX (stay inside the buffer → interpolate → smooth).
# POSE_RENDER_DELAY_S above is the fallback used only if ADAPTIVE is off.
POSE_ADAPTIVE        = os.environ.get("POSE_ADAPTIVE", "1") == "1"
POSE_RENDER_DELAY_MIN = float(os.environ.get("POSE_RENDER_DELAY_MIN_S", "0.0"))
POSE_RENDER_DELAY_MAX = float(os.environ.get("POSE_RENDER_DELAY_MAX_S", "0.15"))
POSE_JITTER_K         = float(os.environ.get("POSE_JITTER_K", "3.0"))
# Cap on the constant-ACCELERATION extrapolation horizon (s): beyond this the avatar
# coasts at constant velocity, so a long stall or a bad accel can't fling it away.
POSE_EXTRAP_MAX_S     = float(os.environ.get("POSE_EXTRAP_MAX_S", "0.30"))
CLOUD_MAX_M   = float(os.environ.get("CLOUD_MAX_M", "50.0"))
PT_SIZE       = float(os.environ.get("PCD_POINT_SIZE", "6.0"))
PT_SIZE_MAX   = float(os.environ.get("PCD_POINT_SIZE_MAX", "40.0"))
CUBE_SIZE     = float(os.environ.get("CUBE_SIZE", "1.0"))   # block-sync cube edge (m)
def _find_urdf() -> str:
    """Locate the robot URDF for the mesh avatar. Override with GO2_URDF; otherwise
    auto-detect a .urdf under client/b2w_description (no env var needed)."""
    env = os.environ.get("GO2_URDF")
    if env:
        return env
    cdir = os.path.dirname(os.path.abspath(__file__))
    # any client/<robot>_description folder; prefer one whose name mentions go2
    roots = sorted(glob.glob(os.path.join(cdir, "*_description")),
                   key=lambda p: (0 if "go2" in os.path.basename(p).lower() else 1, p))
    for base in roots:
        for pat in ("urdf/*.urdf", "*.urdf", "**/*.urdf"):
            hits = sorted(glob.glob(os.path.join(base, pat), recursive=True))
            if hits:
                return hits[0]
    return ""


GO2_URDF = _find_urdf()                          # robot mesh avatar (auto-detected)

_KEYS = proto.keys(ROBOT_NAME, SERVER_PREFIX)
RESET_KEY = f"{SERVER_PREFIX}/cmd/reset"
CEILING_KEY = f"{SERVER_PREFIX}/config/ceiling_z"
CEILING_STEP = float(os.environ.get("CEILING_STEP", "0.2"))      # m per keypress
CEILING_START = float(os.environ.get("CEILING_START", "2.2"))    # m when first enabled

ROBOT_HALF = np.array([0.35, 0.16, 0.18], dtype=np.float64)
STICK = np.array([
    float(os.environ.get("STICK_OFFSET_X", "-0.65")),
    float(os.environ.get("STICK_OFFSET_Y", "0.0")),
    float(os.environ.get("STICK_OFFSET_Z", "0.85")),
], dtype=np.float64)
FOOT_COLORS = {"FR": [1.0, 0.3, 0.3, 1], "FL": [0.3, 1.0, 0.3, 1],
               "RR": [0.3, 0.6, 1.0, 1], "RL": [1.0, 0.85, 0.25, 1]}
COL_CORRECTED  = [0.31, 0.90, 0.47, 1.0]
COL_DEADRECKON = [1.0, 0.74, 0.23, 1.0]
COL_HEADING    = [1.0, 0.30, 0.30, 1.0]
COL_STICK      = [0.9, 0.9, 0.25, 1.0]


def quat_to_R(q):
    x, y, z, w = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _yaw_R(deg):
    r = np.deg2rad(deg)
    c, s = np.cos(r), np.sin(r)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


# box edges for the oriented robot block (8 corners → 12 segment endpoint-pairs)
_BOX_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                         for sz in (-1, 1)], dtype=np.float64)
_BOX_EDGES = np.array([(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
                       (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)], dtype=int)


def robot_segments(R, t, body_col):
    """Endpoint pairs (P,3) + per-vertex colours (P,4) for the body box +
    heading + selfie-stick, transformed by pose (R, t)."""
    box = (_BOX_CORNERS * ROBOT_HALF)[_BOX_EDGES].reshape(-1, 3)        # (24,3)
    head = np.array([[0, 0, 0], [ROBOT_HALF[0] * 1.6, 0, 0]])           # (2,3)
    stick = np.array([[0, 0, 0], STICK])                               # (2,3)
    local = np.vstack([box, head, stick])
    world = local @ R.T + t
    cols = np.vstack([np.tile(body_col, (24, 1)),
                      np.tile(COL_HEADING, (2, 1)),
                      np.tile(COL_STICK, (2, 1))])
    return world.astype(np.float32), cols.astype(np.float32)


def leg_segments(leg_data, R, t):
    """Endpoint pairs + colours for all four legs (3 segments each)."""
    pts, cols = [], []
    for leg in LEG_ORDER:
        p = leg_data[leg]
        chain = [p["hip"], p["thigh_root"], p["knee"], p["foot"]]
        for a, b in ((0, 1), (1, 2), (2, 3)):
            pts.append(chain[a]); pts.append(chain[b])
            cols.append(FOOT_COLORS[leg]); cols.append(FOOT_COLORS[leg])
    world = (np.asarray(pts, dtype=np.float64) @ R.T + t).astype(np.float32)
    feet = np.array([leg_data[leg]["foot"] for leg in LEG_ORDER]) @ R.T + t
    foot_cols = np.array([FOOT_COLORS[leg] for leg in LEG_ORDER], dtype=np.float32)
    return world, np.asarray(cols, dtype=np.float32), feet.astype(np.float32), foot_cols


def ground_grid(size=4.0, step=0.5):
    pts = []
    n = int(round(size / step))
    for i in range(-n, n + 1):
        x = i * step
        pts += [[x, -size, 0], [x, size, 0], [-size, x, 0], [size, x, 0]]
    return np.asarray(pts, dtype=np.float32)


# ─────────────────────────────────────────────────────────────────────────────
# Shared state
# ─────────────────────────────────────────────────────────────────────────────


class PosePredictor:
    """Interpolating pose predictor (multiplayer-netcode style).

    Buffers recent poses on a **source-clock timeline** (the fuser's per-sample
    ``timestamp_ns``, robot clock) and renders the avatar at ``source_now −
    render_delay``, INTERPOLATING between the two buffered poses bracketing that
    instant. This is smooth and never overshoots, so it doesn't rubber-band. Only
    when the buffer runs dry (the stream stalled) does it fall back to velocity
    EXTRAPOLATION (with staleness decay) so a disconnected robot coasts to a stop.

    Why source time and not packet receipt time (the previous, jumpy behaviour):
    the client is a single Python process, so a heavy cloud decode / GPU upload
    can stall the pose callback and deliver a *burst* of poses at once. Keyed on
    receipt time, a burst collapses many distinct instants onto ~one timestamp →
    interpolation is ill-conditioned and the avatar teleports/bounces, and the
    receipt-jitter EMA explodes so the adaptive delay swings. Keyed on each pose's
    OWN timestamp, a burst still lands at the correct positions on the timeline, so
    playback stays smooth no matter how bursty the arrival is. A per-stream
    transport-offset (rolling min of recv−source) maps the local monotonic clock
    onto the source timeline without trusting absolute NTP sync."""

    _OFFSET_WIN_S = 5.0        # rolling window for the min-transport-offset estimate
    _CLOCK_RESET_S = 5.0       # source ts older than newest by this → treat as clock jump

    def __init__(self, now_fn=time.monotonic):
        self._now = now_fn                # injectable for deterministic unit tests
        self._lock = threading.Lock()
        # (src_t seconds [robot clock], recv_monotonic, pose), sorted by src_t
        self._buf = deque(maxlen=256)
        self._have = False
        self._off_win = deque()           # (recv_monotonic, transport_delay) window
        self._offset = None               # min transport delay (s): source→local map
        self._src_dt_ema = 0.0            # source-cadence inter-arrival (→ rate + delay)
        self._jitter_ema = 0.0            # EMA of transport delay ABOVE the min → delay
        self._e2e_ms = float("nan")
        self._last_recv_mono = 0.0
        self._disp_pos = np.zeros(3)
        self._disp_quat = quat_identity()

    def _reset_locked(self):
        self._buf.clear()
        self._off_win.clear()
        self._offset = None
        self._src_dt_ema = 0.0
        self._jitter_ema = 0.0
        self._have = False

    def on_pose(self, pose):
        with self._lock:
            now = self._now()
            src_t = int(pose.timestamp_ns) * 1e-9
            if self._buf:
                newest_src = self._buf[-1][0]
                if src_t < newest_src - self._CLOCK_RESET_S:
                    self._reset_locked()          # source clock stepped → start fresh
                elif src_t <= newest_src:
                    return                         # out-of-order / duplicate → drop
            # transport delay for this sample; rolling MIN absorbs the source↔local
            # clock epoch offset (they are NOT NTP-synced) and gives the link baseline.
            delay = now - src_t
            self._off_win.append((now, delay))
            while self._off_win and now - self._off_win[0][0] > self._OFFSET_WIN_S:
                self._off_win.popleft()
            self._offset = min(d for _, d in self._off_win)
            excess = max(0.0, delay - self._offset)     # transport jitter above baseline
            self._jitter_ema = (excess if self._jitter_ema == 0
                                else 0.9 * self._jitter_ema + 0.1 * excess)
            if self._buf:
                d_src = src_t - self._buf[-1][0]
                self._src_dt_ema = (d_src if self._src_dt_ema == 0
                                    else 0.9 * self._src_dt_ema + 0.1 * d_src)
            self._e2e_ms = (time.time_ns() - int(pose.timestamp_ns)) * 1e-6
            self._last_recv_mono = now
            self._buf.append((src_t, now, pose))
            if not self._have:
                self._disp_pos = pose.position.astype(np.float64)
                self._disp_quat = quat_normalize(pose.quaternion)
                self._have = True

    def _target_at(self, t_src, now):
        """Pose at source-timeline instant ``t_src``: interpolate within the buffer, or
        extrapolate from the newest sample if ``t_src`` is past it. Returns
        ``(pos, quat, fix)``. Caller holds the lock."""
        buf = self._buf
        newest_src, newest_recv, newest = buf[-1]
        if t_src >= newest_src or len(buf) == 1:
            # ahead of the buffer → CONSTANT-ACCELERATION extrapolation with decay.
            # p(h) = p₀ + v·h + ½·a·h²  (a = streamed world accel) — tracks accel/braking
            # far better than constant velocity. h (source-domain) is capped so a stall or
            # bad accel can't fling the avatar; vel+accel fade to 0 as the stream goes stale.
            age = now - newest_recv                       # real (wall) staleness
            horizon = max(0.0, min((t_src - newest_src) + POSE_LOOKAHEAD_S, POSE_EXTRAP_MAX_S))
            scale = 1.0 if age <= STALE_S else max(0.0, 1.0 - (age - STALE_S) / max(DECAY_S, 1e-3))
            pos, quat = integrate_pose(newest.position, newest.quaternion,
                                       newest.linear_velocity * scale,
                                       newest.angular_velocity * scale, horizon)
            acc = np.asarray(getattr(newest, "linear_acceleration", np.zeros(3)),
                             dtype=np.float64).reshape(3)
            pos = pos + 0.5 * acc * scale * (horizon * horizon)
            return pos, quat, newest.fix_quality
        # interpolate between the two samples bracketing t_src (scan from the recent end)
        for i in range(len(buf) - 1, 0, -1):
            s0, _, p0 = buf[i - 1]
            s1, _, p1 = buf[i]
            if s0 <= t_src <= s1:
                a = (t_src - s0) / max(s1 - s0, 1e-6)
                pos = (1 - a) * p0.position + a * p1.position
                quat = quat_slerp(quat_normalize(p0.quaternion), quat_normalize(p1.quaternion), a)
                return pos, quat, p1.fix_quality
        s0, _, p0 = buf[0]                 # t_src older than the whole buffer → oldest
        return p0.position, p0.quaternion, p0.fix_quality

    def _render_delay(self):
        """Adaptive render delay (source-timeline): sit ~one source frame behind the
        newest sample plus a jitter margin, so on a clean link we interpolate one frame
        back (low latency, no overshoot) and on a bursty link we grow to stay inside the
        buffer (interpolate → smooth). Fixed value if POSE_ADAPTIVE=0."""
        if not POSE_ADAPTIVE:
            return POSE_RENDER_DELAY_S
        base = self._src_dt_ema if self._src_dt_ema > 0 else 0.0
        return float(np.clip(base + POSE_JITTER_K * self._jitter_ema,
                             POSE_RENDER_DELAY_MIN, POSE_RENDER_DELAY_MAX))

    def step(self, dt_render):
        with self._lock:
            if not self._have:
                return None
            now = self._now()
            # map the local monotonic clock onto the source timeline, then step back
            # by the render delay. offset = min transport delay (source→local).
            t_src = (now - (self._offset or 0.0)) - self._render_delay()
            tgt_pos, tgt_quat, fix = self._target_at(t_src, now)
            # light critically-damped smoothing to absorb segment-boundary kinks
            alpha = 1.0 - np.exp(-dt_render / max(SMOOTH_TAU, 1e-3))
            self._disp_pos = (1 - alpha) * self._disp_pos + alpha * np.asarray(tgt_pos, float)
            self._disp_quat = quat_slerp(self._disp_quat, quat_normalize(tgt_quat), alpha)
            age = now - self._last_recv_mono
            return self._disp_pos.copy(), self._disp_quat.copy(), fix, age

    def telemetry(self):
        with self._lock:
            if not self._have:
                return None
            age = self._now() - self._last_recv_mono
            rate = (1.0 / self._src_dt_ema) if self._src_dt_ema > 1e-6 else float("nan")
            return age, rate, self._e2e_ms, self._buf[-1][2].fix_quality


# ─────────────────────────────────────────────────────────────────────────────
# Viewer
# ─────────────────────────────────────────────────────────────────────────────


class PRISMViewer:
    def __init__(self, request_snapshot=False):
        self._predictor = PosePredictor()
        self._yaw_offset_deg = float(os.environ.get("CLOUD_YAW_OFFSET_DEG", "0"))
        self._traj = None
        self._traj_lock = threading.Lock()
        self._legs_warned = False
        self._cloud_framed = False
        self._cloud_n = 0
        self._pt_size = PT_SIZE
        # latest merged cloud (pre-display-filter) kept so the ceiling clip can be
        # re-applied instantly on a keypress without waiting for the next submap
        self._cloud_xyz_raw = None
        self._cloud_rgb_raw = None
        self._ceiling_z = None               # None = OFF (show whole cloud)
        # ── 3rd-person follow camera ─────────────────────────────────────────
        # When on, the orbit pivot (camera.center) tracks the robot, smoothed so the
        # robot's pose jumps/corrections don't jerk the view. Orbit (arrows/mouse) and
        # zoom (wheel) stay fully manual; pan (w/a/s/d/q/e) shifts a follow OFFSET so
        # you can frame the robot off-centre and it sticks while following.
        self._follow = False
        self._follow_offset = np.zeros(3, dtype=float)   # camera.center - robot, persisted
        self._follow_tau = float(os.environ.get("FOLLOW_SMOOTH_TAU_S", "0.4"))  # bigger = smoother/laggier
        self._last_tick = time.monotonic()
        # throughput: pose recv bytes (cloud bytes come from BlockSync stats)
        self._pose_bytes = 0
        self._status = {}                # latest server-published metrics (JSON)
        self._status_recv_ns = 0
        # per-path throughput meters + receive-time/timestamp captures for latency
        self._tp_cloud = ThroughputMeter()
        self._tp_pose = ThroughputMeter()
        self._tp_traj = ThroughputMeter()
        self._tp_status = ThroughputMeter()
        self._pose_recv_ns = 0
        self._pose_ts_ns = 0
        # Per-path clock-offset filters: server/client/robot clocks are NOT actually
        # synced (the raw deltas go negative), so we run the same windowed-minimum
        # offset filter on EACH arriving stream → latency relative to that link's own
        # baseline (always ≥0, meaningful for stalls/jitter) without trusting NTP.
        self._clk_s2c = ClockOffsetEstimator()     # server → client (status)
        self._clk_pose = ClockOffsetEstimator()    # robot  → client (pose)
        self._clk_cap = ClockOffsetEstimator()     # frame capture → display
        self._render_fps = 0.0
        self._render_stalls = 0
        self._last_metrics_t = 0.0
        self._last_cloud_t = 0.0          # throttles the cloud GPU upload (CLOUD_RENDER_HZ)

        log.info(f"[Viewer] Connecting to Zenoh at {ZENOH_ROUTER}...")
        self._z = self._open(self._conf())
        self._z_fast = self._open(self._conf())          # isolate low-latency pose/legs
        log.info("[Viewer] Connected (2 sessions: bulk + low-latency).")

        # bulk session cloud transport, chosen by STREAM_MODE (must match the server):
        #   snapshot — whole-map replace each submap (default; pairs with reset mode).
        #   blocks   — DEPRECATED diff-based sync (manifest + push + Draco pull).
        # Both expose take_delta()/take_merged()/force_resync() + the HUD telemetry.
        if STREAM_MODE == "snapshot":
            self._blocksync = SnapshotSync(self._z, server_prefix=SERVER_PREFIX,
                                           request_snapshot=request_snapshot)
        else:
            self._blocksync = BlockSync(self._z, cube_m=CUBE_SIZE, server_prefix=SERVER_PREFIX)
        self._pub_reset = self._z.declare_publisher(RESET_KEY)
        self._pub_ceiling = self._z.declare_publisher(CEILING_KEY)
        # CONTROL lane (fast session): the small, latency-critical messages —
        # trajectory + server status — must NOT queue behind the bulk geometry
        # transfer, or the pose-line/HUD lag seconds behind the "green fix" flash.
        self._z_fast.declare_subscriber(_KEYS["trajectory"], self._on_traj)
        self._z_fast.declare_subscriber(_KEYS["status"], self._on_status)
        # low-latency session: authoritative pose + leg FK + body height
        self._z_fast.declare_subscriber(_KEYS["pose"], self._on_pose)
        self._body_tracker = RobotStateTracker(self._z_fast, ROBOT_NAME)
        self._leg_tracker = LowStateTracker(self._z_fast, ROBOT_NAME)
        # Incremental render buffer: update only the cubes that changed each submap
        # instead of re-merging + re-uploading the whole cloud (the render "stalls").
        # VIEWER_INCREMENTAL=0 forces the simple whole-cloud merge path.
        self._incremental = os.environ.get("VIEWER_INCREMENTAL", "1") == "1"
        self._cloudbuf = IncrementalCloud()
        # Robot avatar: real URDF mesh if available (toggle 'U'), else skeleton.
        self._urdf = URDFRobot(GO2_URDF)
        self._robot_mode = "mesh" if self._urdf.available else "skeleton"
        self._mesh_throttle_t = 0.0
        log.info(f"[Viewer] subscribed: [bulk] cloud={STREAM_MODE} | "
                 f"[fast] pose + trajectory + status  (incremental={self._incremental})")

    @staticmethod
    def _conf():
        c = zenoh.Config()
        c.insert_json5("connect/endpoints", f'["{ZENOH_ROUTER}"]')
        c.insert_json5("mode", '"client"')
        return c

    @staticmethod
    def _open(conf):
        while True:
            try:
                return zenoh.open(conf)
            except Exception as e:
                log.warning(f"[Viewer] Zenoh connect failed: {e} — retry in 5s")
                time.sleep(5)

    # ── zenoh callbacks (small payloads) ────────────────────────────────────
    def _on_traj(self, sample):
        buf = bytes(sample.payload)
        self._tp_traj.add(len(buf))
        try:
            pts = proto.unpack_trajectory(buf)
        except proto.ProtocolError:
            return
        if pts.shape[0] >= 2:
            with self._traj_lock:
                self._traj = pts.astype(np.float32)

    def _on_pose(self, sample):
        buf = bytes(sample.payload)
        self._pose_bytes += len(buf)
        self._tp_pose.add(len(buf))
        try:
            pose = proto.unpack_pose(buf)
        except proto.ProtocolError:
            return
        self._pose_recv_ns = time.time_ns()
        self._pose_ts_ns = int(pose.timestamp_ns)
        self._clk_pose.update(self._pose_ts_ns, self._pose_recv_ns)
        self._predictor.on_pose(pose)

    def _on_status(self, sample):
        buf = bytes(sample.payload)
        self._tp_status.add(len(buf))
        try:
            self._status = json.loads(buf.decode())
            self._status_recv_ns = time.time_ns()
            if self._status.get("server_send_ns"):
                self._clk_s2c.update(int(self._status["server_send_ns"]), self._status_recv_ns)
            if self._status.get("newest_frame_robot_ns"):
                self._clk_cap.update(int(self._status["newest_frame_robot_ns"]), self._status_recv_ns)
        except Exception:
            pass

    def _reset_map(self):
        log.info("[Viewer] RESET → wipe server map + local cubes")
        try:
            self._pub_reset.put(b"reset")
        except Exception as e:
            log.warning(f"[Viewer] reset publish failed: {e}")
        self._blocksync.force_resync()
        self._cloud_framed = False

    # ── render setup + loop (main thread, VisPy event loop) ─────────────────
    def run(self):
        canvas = scene.SceneCanvas(title="VAT — PRISM Viewer", keys="interactive",
                                   bgcolor="#0d0d10", size=(1280, 800), show=True)
        view = canvas.central_widget.add_view()
        view.camera = scene.cameras.TurntableCamera(up="+z", fov=45, distance=8.0)
        view.camera.interactive = True       # ensure mouse orbit/zoom is enabled
        self._canvas = canvas

        self._cloud_vis = scene.visuals.Markers(parent=view.scene)
        self._cloud_vis.set_gl_state(depth_test=True)
        try:
            self._cloud_vis.antialias = 1          # soft disc edge, no hard black ring
        except Exception:
            pass
        self._robot_vis = scene.visuals.Line(parent=view.scene, connect="segments",
                                              width=2.0, antialias=True)
        self._legs_vis = scene.visuals.Line(parent=view.scene, connect="segments",
                                             width=3.0, antialias=True)
        self._feet_vis = scene.visuals.Markers(parent=view.scene)
        try:
            self._robot_mesh_vis = scene.visuals.Mesh(parent=view.scene, shading="smooth",
                                                      color=(0.72, 0.76, 0.82, 1.0))
            self._robot_mesh_vis.set_gl_state(depth_test=True, cull_face=False)
            try:        # brighten ambient so back faces are not near-black
                self._robot_mesh_vis.shading_filter.ambient_light = (1, 1, 1, 0.6)
            except Exception:
                pass
            self._robot_mesh_vis.visible = False
        except Exception as _e:
            log.warning(f"[Viewer] mesh visual unavailable ({_e}); skeleton only.")
            self._robot_mesh_vis = None
            self._robot_mode = "skeleton"
        self._traj_vis = scene.visuals.Line(parent=view.scene, connect="strip",
                                             color=(1.0, 0.78, 0.24, 1.0), width=2.0)
        scene.visuals.Line(pos=ground_grid(), parent=view.scene, connect="segments",
                           color=(0.27, 0.27, 0.33, 1.0), width=1.0)
        scene.visuals.XYZAxis(parent=view.scene)
        self._view = view

        # latency HUD — fixed top-left overlay in canvas pixel coords (y-down)
        self._hud = scene.visuals.Text("", color=(0.8, 1.0, 0.85, 1.0), bold=False,
                                       font_size=9, anchor_x="left", anchor_y="top",
                                       parent=canvas.scene)
        # pushed well below the OS title bar so it isn't occluded
        self._hud.transform = scene.transforms.STTransform(translate=(12, 40))

        # ── separate TELEMETRY window (latency / throughput / drops / pose) ──
        # One Text visual PER LINE, stacked vertically. A single multi-line Text
        # renders all lines at the same point on this VisPy/glfw build (they overlap
        # and you only see the last one), so we lay the lines out explicitly.
        self._mcanvas = scene.SceneCanvas(title="VAT — Telemetry", bgcolor="#0c0e12",
                                          size=(470, 600), show=True)
        self._mlines = []
        for i in range(30):
            t = scene.visuals.Text("", color=(0.80, 0.96, 0.86, 1.0), bold=False,
                                   font_size=9, anchor_x="left", anchor_y="top",
                                   parent=self._mcanvas.scene)
            t.transform = scene.transforms.STTransform(translate=(14, 16 + i * 19))
            self._mlines.append(t)

        def _camera_key(ev):
            """Keyboard orbit/tilt/pan/zoom (works regardless of mouse modifiers).
            Returns True if the key was a camera control."""
            cam = self._view.camera
            name = getattr(ev.key, "name", "") or ""
            if name == "Left":    cam.azimuth -= 5
            elif name == "Right": cam.azimuth += 5
            elif name == "Up":    cam.elevation = float(np.clip(cam.elevation + 5, -89, 89))
            elif name == "Down":  cam.elevation = float(np.clip(cam.elevation - 5, -89, 89))
            else:
                return False
            return True

        def _pan_key(k):
            cam = self._view.camera
            step = 0.08 * float(getattr(cam, "scale_factor", cam.distance or 8.0))
            d = np.zeros(3, dtype=float)
            if   k == "a": d[0] -= step
            elif k == "d": d[0] += step
            elif k == "w": d[1] += step
            elif k == "s": d[1] -= step
            elif k == "q": d[2] += step
            elif k == "e": d[2] -= step
            else:
                return False
            cam.center = tuple(np.array(cam.center, dtype=float) + d)
            if self._follow:
                self._follow_offset += d        # keep the pan offset while following
            return True

        @canvas.events.key_press.connect
        def _on_key(ev):
            if _camera_key(ev):                     # arrow keys → orbit/tilt
                return
            k = (ev.text or "").lower()
            if _pan_key(k):                          # w/a/s/d/q/e → pan
                return
            if k == "1":
                self._blocksync.force_resync()      # drop local cubes → full refetch
            elif k == "r":
                self._reset_map()
            elif k == "f":
                self._frame_to(self._cloud_xyz_raw)
            elif k == "t":
                self._toggle_follow()           # 3rd-person follow on/off
            elif k == ",":
                self._set_yaw(self._yaw_offset_deg - 5)
            elif k == ".":
                self._set_yaw(self._yaw_offset_deg + 5)
            elif k == "/":
                self._set_yaw(0.0)
            elif k == "u":
                if self._urdf.available and self._robot_mesh_vis is not None:
                    self._robot_mode = "skeleton" if self._robot_mode == "mesh" else "mesh"
                    log.info(f"[Viewer] robot avatar → {self._robot_mode}")
                else:
                    log.info("[Viewer] URDF mesh unavailable (need client/b2w_description + uv sync)")
            elif k in ("n", "m"):
                self._pt_size = float(np.clip(self._pt_size + (2 if k == "m" else -2), 1, PT_SIZE_MAX))
                log.info(f"[Viewer] point size = {self._pt_size:.0f} (max {PT_SIZE_MAX:.0f})")
                self._render_cloud()        # re-render so the size change shows now
            elif k == "c":                      # toggle ceiling clip on/off
                self._set_ceiling(None if self._ceiling_z is not None else CEILING_START)
            elif k == "[":                      # lower the ceiling
                base = self._ceiling_z if self._ceiling_z is not None else CEILING_START
                self._set_ceiling(base - CEILING_STEP)
            elif k == "]":                      # raise the ceiling
                base = self._ceiling_z if self._ceiling_z is not None else CEILING_START
                self._set_ceiling(base + CEILING_STEP)

        self._timer = app.Timer(interval=1.0 / max(RENDER_HZ, 1.0),
                                connect=self._on_tick, start=True)
        self._print_controls()
        log.info("[Viewer] Rendering. Close the window or Ctrl+C to quit.")
        try:
            app.run()
        except KeyboardInterrupt:
            pass
        finally:
            try:
                self._mcanvas.close()
            except Exception:
                pass
            try:
                self._z.close(); self._z_fast.close()
            except Exception:
                pass
            log.info("[Viewer] Shut down.")

    @staticmethod
    def _print_controls():
        log.info("[Viewer] keys:  ←/→ orbit · ↑/↓ tilt · W/A/S/D/Q/E pan · scroll/F zoom-fit | "
                 "T follow robot (3rd-person) | "
                 "1 re-fetch | R reset | U robot mesh/skeleton | , / . yaw | N/M size | C ceiling | [ / ] ceiling∓  "
                 "(mouse: drag orbit, shift+drag pan, scroll zoom)  +  Telemetry window")

    def _set_yaw(self, deg):
        self._yaw_offset_deg = float(deg)
        log.info(f"[Viewer] cloud↔robot yaw offset = {self._yaw_offset_deg:.0f}°")

    def _set_ceiling(self, z):
        """Set the ceiling-clip height (m, world-Z) or None to disable. Publishes to
        the server (drops points above it at the source → less bandwidth) AND clips
        the already-received cloud locally for instant visual feedback."""
        self._ceiling_z = None if z is None else float(z)
        payload = "off" if self._ceiling_z is None else f"{self._ceiling_z:.2f}"
        try:
            self._pub_ceiling.put(payload.encode())
        except Exception as e:
            log.warning(f"[Viewer] ceiling publish failed: {e}")
        log.info(f"[Viewer] ceiling clip → {'OFF (whole cloud)' if self._ceiling_z is None else f'Z<={self._ceiling_z:.2f}m'}")
        self._render_cloud()                 # instant local feedback

    def _render_cloud(self):
        """(Re)draw the cloud from the last merged snapshot, applying the finite/range
        filter and the optional ceiling clip. Safe to call from the tick or a key."""
        xyz, rgb = self._cloud_xyz_raw, self._cloud_rgb_raw
        if xyz is None or rgb is None or xyz.shape[0] == 0 or xyz.shape[0] != rgb.shape[0]:
            self._cloud_vis.set_data(np.zeros((0, 3), np.float32))
            self._cloud_n = 0
            return
        keep = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).max(axis=1) <= CLOUD_MAX_M)
        if self._ceiling_z is not None:
            keep &= xyz[:, 2] <= self._ceiling_z
        x, c = xyz[keep], rgb[keep]
        rgba = np.ones((x.shape[0], 4), np.float32)
        rgba[:, :3] = c.astype(np.float32) / 255.0
        # edge_color == face_color (width 0) gives a clean, connected Open3D-style
        # point look instead of black-ringed dots.
        self._cloud_vis.set_data(x.astype(np.float32), face_color=rgba, edge_color=rgba,
                                 size=self._pt_size, edge_width=0)
        self._cloud_n = int(x.shape[0])
        if not self._cloud_framed and x.shape[0]:
            self._frame_to(x)
            self._cloud_framed = True

    def _frame_to(self, xyz):
        """Frame the camera to a point set WITHOUT camera.set_range(). set_range walks
        every visual's bounds, and an empty Markers visual (feet before legs arrive)
        has ``_data['a_position'] is None`` → it crashes every tick and freezes mouse
        rotation. Compute centre/distance from this cloud alone instead."""
        try:
            xyz = np.asarray(xyz, np.float64)
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            if xyz.shape[0] == 0:
                return
            lo, hi = xyz.min(axis=0), xyz.max(axis=0)
            cam = self._view.camera
            cam.center = tuple((lo + hi) * 0.5)
            cam.distance = float(max(np.linalg.norm(hi - lo) * 0.9, 1.0))
        except Exception as e:
            log.warning(f"[Viewer] frame failed (non-fatal): {e}")

    def _toggle_follow(self):
        """Toggle 3rd-person follow. On enable, seed the offset from the CURRENT view so
        the pivot doesn't jump — it keeps your framing and just starts tracking the robot.
        Orbit + zoom stay manual; pan adjusts the offset (see _pan_key)."""
        cam = self._view.camera
        self._follow = not self._follow
        if self._follow:
            pos = getattr(self, "_last_pos", None)
            self._follow_offset = (np.array(cam.center, dtype=float) - np.asarray(pos, dtype=float)
                                   if pos is not None else np.zeros(3, dtype=float))
        log.info(f"[Viewer] 3rd-person follow {'ON' if self._follow else 'OFF'} "
                 f"(press 't' to toggle; orbit/zoom/pan still work)")

    def _follow_camera(self, dt):
        """Ease the orbit pivot toward robot + offset with frame-rate-independent
        exponential smoothing (tau seconds), so a jumpy robot pose never jerks the view.
        Only camera.center moves; azimuth/elevation/distance (orbit + zoom) are untouched."""
        pos = getattr(self, "_last_pos", None)
        if pos is None:
            return
        cam = self._view.camera
        target = np.asarray(pos, dtype=float) + self._follow_offset
        cur = np.array(cam.center, dtype=float)
        alpha = 1.0 - float(np.exp(-max(dt, 1e-3) / max(self._follow_tau, 1e-3)))
        cam.center = tuple(cur + alpha * (target - cur))

    def _on_tick(self, event):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        # render FPS (EMA) + stall counter (a tick much slower than the target rate)
        if dt > 1e-6:
            inst = 1.0 / dt
            self._render_fps = inst if self._render_fps == 0 else 0.9 * self._render_fps + 0.1 * inst
            if dt > 3.0 / max(RENDER_HZ, 1.0):
                self._render_stalls += 1

        # cloud: BlockSync hands us the merged map only when a cube changed. Keep the
        # raw snapshot so the ceiling clip can be re-applied instantly on a keypress,
        # and render through one path (which guards against any xyz/rgb mismatch).
        # THROTTLED off the pose/render tick: the heavy whole-cloud GPU upload runs at
        # CLOUD_RENDER_HZ (deltas coalesce losslessly in the store meanwhile) so it can't
        # stall the per-tick pose predictor step + avatar redraw below.
        if CLOUD_RENDER_HZ <= 0 or (now - self._last_cloud_t) >= 1.0 / CLOUD_RENDER_HZ:
            self._last_cloud_t = now
            self._update_cloud()

        # trajectory
        with self._traj_lock:
            traj = self._traj
        if traj is not None and traj.shape[0] >= 2:
            t = traj @ _yaw_R(self._yaw_offset_deg).T if self._yaw_offset_deg else traj
            self._traj_vis.set_data(pos=t.astype(np.float32))

        # robot at the predicted pose — URDF mesh or wireframe skeleton
        pred = self._predictor.step(dt)
        if pred is not None:
            self._draw_robot(pred)

        # 3rd-person follow: ease the orbit pivot toward the robot (jump-proof).
        if self._follow:
            self._follow_camera(dt)

        # Overlays must never kill the render tick (a crash here froze the camera
        # and stopped the robot drawing). Isolate them.
        try:
            self._update_hud()
        except Exception as e:
            if not getattr(self, "_hud_warned", False):
                log.warning(f"[Viewer] HUD update error (suppressed): {e}")
                self._hud_warned = True
        if now - self._last_metrics_t >= 0.25:      # ~4 Hz dashboard refresh
            self._last_metrics_t = now
            try:
                self._update_metrics()
            except Exception as e:
                if not getattr(self, "_metrics_warned", False):
                    log.warning(f"[Viewer] metrics update error (suppressed): {e}")
                    self._metrics_warned = True

    def _draw_robot(self, pred):
        """Draw the robot at the predicted pose. Uses the URDF mesh when in mesh mode
        and everything lines up; otherwise (or on any failure) the wireframe skeleton.
        Mesh/skeleton visibility is mutually exclusive so they never double-draw."""
        pos, quat, fix, age = pred
        R = quat_to_R(quat)
        if self._yaw_offset_deg:
            Y = _yaw_R(self._yaw_offset_deg)
            R, pos = Y @ R, Y @ pos
        self._last_pos = np.asarray(pos, float)

        mesh_ok = False
        if self._robot_mode == "mesh" and self._urdf.available and self._robot_mesh_vis is not None:
            # throttle the (heavier) mesh rebuild to ~20 Hz
            now = time.monotonic()
            if now - self._mesh_throttle_t >= 0.05:
                self._mesh_throttle_t = now
                try:
                    q12, _imu, jvalid = self._leg_tracker.get_joints()
                    geom = self._urdf.world_geometry(R, pos, q12)
                    if geom is not None:
                        v, f = geom
                        self._robot_mesh_vis.set_data(vertices=v, faces=f,
                                                      color=(0.62, 0.67, 0.74, 1.0))
                        mesh_ok = True
                except Exception as e:
                    if not getattr(self, "_mesh_warned", False):
                        log.warning(f"[Viewer] mesh robot failed → skeleton: {e}")
                        self._mesh_warned = True
            else:
                mesh_ok = self._robot_mesh_vis.visible          # keep last frame's choice

        self._robot_mesh_vis.visible = mesh_ok if self._robot_mesh_vis is not None else False
        self._robot_vis.visible = not mesh_ok
        self._legs_vis.visible = not mesh_ok
        self._feet_vis.visible = not mesh_ok
        if mesh_ok:
            return

        body_col = COL_CORRECTED if fix == proto.FIX_CORRECTED else COL_DEADRECKON
        rpts, rcols = robot_segments(R, pos, body_col)
        self._robot_vis.set_data(pos=rpts, color=rcols)
        leg_data, legs_valid = self._leg_tracker.get()
        if legs_valid and leg_data:
            lpts, lcols, feet, fcols = leg_segments(leg_data, R, pos)
            self._legs_vis.set_data(pos=lpts, color=lcols)
            self._feet_vis.set_data(feet, face_color=fcols, edge_color=fcols, size=10, edge_width=0)
        elif not self._legs_warned:
            log.info("[Viewer] (no leg data yet — /lowstate flowing?)")
            self._legs_warned = True

    def _update_cloud(self):
        """Refresh the render buffer from BlockSync. Incremental by default (apply
        only the cubes that changed this submap); falls back to the whole-cloud merge
        on any error or when VIEWER_INCREMENTAL=0. Both paths leave the raw cloud in
        _cloud_xyz_raw/_rgb_raw so the ceiling clip can re-render instantly on a key."""
        if self._incremental:
            try:
                d = self._blocksync.take_delta()
                if d is not None:
                    changed, removed, resync = d
                    self._cloudbuf.apply(changed, removed, resync)
                    self._cloud_xyz_raw, self._cloud_rgb_raw = self._cloudbuf.live()
                    self._render_cloud()
                    self._tp_cloud.add(int(getattr(self._blocksync, "last_push_bytes", 0))
                                       + int(getattr(self._blocksync, "last_bundle_bytes", 0)))
                return
            except Exception as e:
                if not getattr(self, "_inc_warned", False):
                    log.warning(f"[Viewer] incremental cloud failed → merge fallback: {e}")
                    self._inc_warned = True
                self._incremental = False
        m = self._blocksync.take_merged()
        if m is not None:
            self._cloud_xyz_raw, self._cloud_rgb_raw = m
            try:
                self._render_cloud()
            except Exception as e:
                if not getattr(self, "_cloud_warned", False):
                    log.warning(f"[Viewer] cloud render error (suppressed): {e}")
                    self._cloud_warned = True
            self._tp_cloud.add(int(getattr(self._blocksync, "last_bundle_bytes", 0)))

    def _update_metrics(self):
        """Render the separate telemetry window: per-path latency + throughput +
        drops + pose + render health. Latencies use the server-published robot clock
        offset (server/client are NTP-synced); they are RELATIVE to the link baseline
        (the robot clock isn't absolutely synced — see docs)."""
        for m in (self._tp_cloud, self._tp_pose, self._tp_traj, self._tp_status):
            m.decay()
        s = self._status or {}

        def ms(v):
            return f"{v:6.0f} ms" if v == v else "    -- ms"

        def lat(est):
            return est.last_latency_s * 1e3 if est.offset_s is not None else float("nan")

        # latencies — each relative to its own link's baseline (clock-skew immune)
        r2s = float(s.get("robot_to_server_ms", float("nan")))   # server-side filter
        s2c = lat(self._clk_s2c)                                  # client-side filters
        r2c = lat(self._clk_pose)
        cap2disp = lat(self._clk_cap)

        tel = self._predictor.telemetry()
        pose_age, pose_rate, _e2e, fix = tel if tel else (float("nan"),) * 4
        fixs = "VGGT-fix" if fix == proto.FIX_CORRECTED else "dead-reckon"
        pos = getattr(self, "_last_pos", np.zeros(3))
        leg_data, legs_valid = self._leg_tracker.get()
        nfeet = len(leg_data) if (legs_valid and leg_data) else 0

        lines = [
            "── VAT TELEMETRY ───────────────────────────",
            "LATENCY (relative, above link baseline)",
            f"  robot → server     {ms(r2s)}",
            f"  server → client    {ms(s2c)}",
            f"  robot → client(pose){ms(r2c)}",
            f"  capture → display  {ms(cap2disp)}",
            f"  robot→srv offset   {float(s.get('robot_offset_ms', 0.0)):+.0f} ms",
            "",
            "THROUGHPUT",
            f"  robot → server     {float(s.get('robot_kbps',0)):6.0f} KB/s  {float(s.get('robot_fps',0)):4.1f} fps",
            f"  cloud  → client    {self._tp_cloud.kbps:6.0f} KB/s  {self._tp_cloud.mps:4.1f}/s",
            f"  pose   → client    {self._tp_pose.kbps:6.1f} KB/s  {self._tp_pose.mps:4.0f}/s",
            f"  traj/status        {self._tp_traj.kbps:5.1f}/{self._tp_status.kbps:.1f} KB/s",
            "",
            "POSE / ODOMETRY",
            f"  state              {fixs}",
            f"  position   {pos[0]:+.2f} {pos[1]:+.2f} {pos[2]:+.2f} m",
            f"  pose age/rate      {pose_age*1e3:5.0f} ms / {pose_rate:4.1f} Hz",
            f"  legs (feet w/ FK)  {nfeet}/4",
            "",
            "DROPS / RENDER",
            f"  seq gaps (frames)  {int(s.get('seq_gaps',0))}",
            f"  cubes +chg/-rm     +{int(s.get('cubes_changed',0))}/-{int(s.get('cubes_removed',0))}",
            f"  map points         {int(s.get('n_points',0))}",
            f"  render             {self._render_fps:4.0f} fps  stalls {self._render_stalls}",
        ]
        for i, t in enumerate(self._mlines):
            t.text = lines[i] if i < len(lines) else ""
        # also print a compact block to the console every ~3 s (reliable fallback)
        now = time.monotonic()
        if now - getattr(self, "_last_console_t", 0.0) >= 3.0:
            self._last_console_t = now
            log.info("[Telemetry]\n  " + "\n  ".join(lines))

    def _update_hud(self):
        tel = self._predictor.telemetry()
        if tel is None:
            pose_line = "POSE  waiting…"
        else:
            age, rate, e2e_ms, fix = tel
            fixs = "VGGT" if fix == proto.FIX_CORRECTED else "dead-reckon"
            rate_s = f"{rate:4.1f}Hz" if rate == rate else "  – "
            e2e_s = f"{e2e_ms:5.0f}ms" if e2e_ms == e2e_ms else "  – "
            warn = "  ⚠STALE" if age > STALE_S else ""
            pose_line = (f"POSE  age {age*1000:4.0f}ms  rate {rate_s}  "
                         f"e2e {e2e_s}  {self._tp_pose.kbps:5.1f} KB/s  fix {fixs}{warn}")

        # cloud line from BlockSync (diff-based): cubes held, last sync cost
        bs = self._blocksync
        cloud_line = (f"CLOUD  {self._cloud_n} pts  {bs.cubes} cubes  "
                      f"last sync {bs.last_need} cubes / {bs.last_bundle_bytes/1024:5.1f} KB "
                      f"in {bs.last_sync_ms:4.0f}ms  total {bs.bytes_total/1e6:.2f} MB")

        # server-published metrics (cross-network view → find the bottleneck)
        s = self._status
        if s:
            srv = (f"SERVER  submap {s.get('submap_s', 0):.2f}s  "
                   f"cubes {s.get('cubes_changed', '?')}/{s.get('cubes', '?')} changed  "
                   f"manifest {s.get('manifest_kb', '?')} KB  "
                   f"buf {s.get('frames_buffered', '?')}f  map {s.get('n_points', '?')} pts")
        else:
            srv = "SERVER  (no status yet)"

        self._hud.text = (pose_line + "\n" + cloud_line + "\n" + srv
                          + f"\nlook-ahead {POSE_LOOKAHEAD_S*1000:.0f}ms   "
                          f"(1 re-fetch · R reset · F refit · ,/. yaw · N/M size)")


def main():
    parser = argparse.ArgumentParser(description="VAT PRISM VisPy viewer")
    parser.add_argument("--snapshot", action="store_true",
                        help="Fetch the current cloud on start")
    args = parser.parse_args()
    PRISMViewer(request_snapshot=args.snapshot).run()


if __name__ == "__main__":
    main()
