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
  * the PRISM **point cloud**, STREAMED — the server pushes a full (compressed,
    16-bit-quantised + zlib) snapshot per submap and each one *replaces* the local
    cloud, so it stays aligned and never accumulates stale/duplicated blocks. Decode
    runs in a worker thread (off the GL + zenoh-callback threads).
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
from vat_cloudbuffer import IncrementalCloud  # noqa: E402
from urdf_robot import URDFRobot  # noqa: E402

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="[%(asctime)s] [%(levelname)s] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("prism-viewer")

ZENOH_ROUTER  = os.environ.get("ZENOH_ROUTER",  "tcp/127.0.0.1:7447")
ROBOT_NAME    = os.environ.get("ROBOT_NAME",    "go2")
SERVER_PREFIX = os.environ.get("SERVER_PREFIX", "server/prism")
RENDER_HZ     = float(os.environ.get("RENDER_HZ", "60.0"))
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

    Buffers recent poses by receipt time and renders the avatar at ``now -
    POSE_RENDER_DELAY_S``, INTERPOLATING between the two buffered poses bracketing
    that instant. This is smooth and never overshoots, so it doesn't rubber-band.
    Only when the buffer runs dry (the stream stalled) does it fall back to
    velocity EXTRAPOLATION (with staleness decay) so a disconnected robot coasts to
    a stop instead of freezing."""

    def __init__(self):
        self._lock = threading.Lock()
        self._buf = deque(maxlen=128)     # (recv_monotonic, pose), time-ordered
        self._have = False
        self._dt_ema = 0.0
        self._e2e_ms = float("nan")
        self._disp_pos = np.zeros(3)
        self._disp_quat = quat_identity()

    def on_pose(self, pose):
        with self._lock:
            now = time.monotonic()
            if self._buf:
                dt = now - self._buf[-1][0]
                self._dt_ema = dt if self._dt_ema == 0 else 0.9 * self._dt_ema + 0.1 * dt
            self._e2e_ms = (time.time_ns() - pose.timestamp_ns) * 1e-6
            self._buf.append((now, pose))
            if not self._have:
                self._disp_pos = pose.position.astype(np.float64)
                self._disp_quat = quat_normalize(pose.quaternion)
                self._have = True

    def _target_at(self, t):
        """Pose at render time ``t`` (monotonic): interpolate within the buffer, or
        extrapolate from the newest sample if ``t`` is past it. Returns
        ``(pos, quat, fix)``. Caller holds the lock."""
        buf = self._buf
        newest_recv, newest = buf[-1]
        if t >= newest_recv or len(buf) == 1:
            # buffer dry / disconnected → velocity extrapolation with decay
            age = time.monotonic() - newest_recv
            horizon = (t - newest_recv) + POSE_LOOKAHEAD_S
            scale = 1.0 if age <= STALE_S else max(0.0, 1.0 - (age - STALE_S) / max(DECAY_S, 1e-3))
            pos, quat = integrate_pose(newest.position, newest.quaternion,
                                       newest.linear_velocity * scale,
                                       newest.angular_velocity * scale, horizon)
            return pos, quat, newest.fix_quality
        # interpolate between the two samples bracketing t (scan from the recent end)
        for i in range(len(buf) - 1, 0, -1):
            r0, p0 = buf[i - 1]
            r1, p1 = buf[i]
            if r0 <= t <= r1:
                a = (t - r0) / max(r1 - r0, 1e-6)
                pos = (1 - a) * p0.position + a * p1.position
                quat = quat_slerp(quat_normalize(p0.quaternion), quat_normalize(p1.quaternion), a)
                return pos, quat, p1.fix_quality
        r0, p0 = buf[0]                    # t older than the whole buffer → oldest
        return p0.position, p0.quaternion, p0.fix_quality

    def step(self, dt_render):
        with self._lock:
            if not self._have:
                return None
            t = time.monotonic() - POSE_RENDER_DELAY_S
            tgt_pos, tgt_quat, fix = self._target_at(t)
            # light critically-damped smoothing to absorb segment-boundary kinks
            alpha = 1.0 - np.exp(-dt_render / max(SMOOTH_TAU, 1e-3))
            self._disp_pos = (1 - alpha) * self._disp_pos + alpha * np.asarray(tgt_pos, float)
            self._disp_quat = quat_slerp(self._disp_quat, quat_normalize(tgt_quat), alpha)
            age = time.monotonic() - self._buf[-1][0]
            return self._disp_pos.copy(), self._disp_quat.copy(), fix, age

    def telemetry(self):
        with self._lock:
            if not self._have:
                return None
            age = time.monotonic() - self._buf[-1][0]
            rate = (1.0 / self._dt_ema) if self._dt_ema > 1e-6 else float("nan")
            return age, rate, self._e2e_ms, self._buf[-1][1].fix_quality


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

        log.info(f"[Viewer] Connecting to Zenoh at {ZENOH_ROUTER}...")
        self._z = self._open(self._conf())
        self._z_fast = self._open(self._conf())          # isolate low-latency pose/legs
        log.info("[Viewer] Connected (2 sessions: bulk + low-latency).")

        # bulk session: DIFF-BASED block sync (manifest + Draco bundles, only the
        # cubes that changed) + trajectory + server status.
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
        log.info(f"[Viewer] subscribed: [bulk] block-sync(push+manifest) | "
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
            c = np.array(cam.center, dtype=float)
            if   k == "a": c[0] -= step
            elif k == "d": c[0] += step
            elif k == "w": c[1] += step
            elif k == "s": c[1] -= step
            elif k == "q": c[2] += step
            elif k == "e": c[2] -= step
            else:
                return False
            cam.center = tuple(c)
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
