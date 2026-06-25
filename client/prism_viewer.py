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
import threading

import numpy as np
from vispy import app, scene

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "common"))
sys.path.insert(0, os.path.join(_ROOT, "robot", "docker"))
import vat_protocol as proto  # noqa: E402
from vat_protocol import (  # noqa: E402
    quat_identity, quat_normalize, quat_mul, quat_slerp, integrate_pose)
import zenoh  # noqa: E402
from kinematics import RobotStateTracker, LowStateTracker, LEG_ORDER  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))         # client/ (block_sync)
from block_sync import BlockSync  # noqa: E402

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
CLOUD_MAX_M   = float(os.environ.get("CLOUD_MAX_M", "50.0"))
PT_SIZE       = float(os.environ.get("PCD_POINT_SIZE", "3.0"))
CUBE_SIZE     = float(os.environ.get("CUBE_SIZE", "1.0"))   # block-sync cube edge (m)

_KEYS = proto.keys(ROBOT_NAME, SERVER_PREFIX)
RESET_KEY = f"{SERVER_PREFIX}/cmd/reset"

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
    def __init__(self):
        self._lock = threading.Lock()
        self._have = False
        self._sample = None
        self._recv_monotonic = 0.0
        self._dt_ema = 0.0                # inter-arrival time EMA → rate
        self._e2e_ms = float("nan")       # capture→receipt latency (needs clock sync)
        self._disp_pos = np.zeros(3)
        self._disp_quat = quat_identity()

    def on_pose(self, pose):
        with self._lock:
            now = time.monotonic()
            if self._have and self._recv_monotonic:
                dt = now - self._recv_monotonic
                self._dt_ema = dt if self._dt_ema == 0 else 0.9 * self._dt_ema + 0.1 * dt
            # capture→receipt latency (wall clock; meaningful only if robot/client
            # clocks are synced — shown on the HUD so you can judge the lag).
            self._e2e_ms = (time.time_ns() - pose.timestamp_ns) * 1e-6
            self._sample = pose
            self._recv_monotonic = now
            if not self._have:
                self._disp_pos = pose.position.astype(np.float64)
                self._disp_quat = quat_normalize(pose.quaternion)
                self._have = True

    def step(self, dt_render):
        with self._lock:
            if not self._have or self._sample is None:
                return None
            s = self._sample
            age = time.monotonic() - self._recv_monotonic
            # extrapolate forward by age + look-ahead to hide transport latency
            horizon = age + POSE_LOOKAHEAD_S
            scale = 1.0 if age <= STALE_S else max(0.0, 1.0 - (age - STALE_S) / max(DECAY_S, 1e-3))
            tgt_pos, tgt_quat = integrate_pose(
                s.position, s.quaternion, s.linear_velocity * scale,
                s.angular_velocity * scale, horizon)
            alpha = 1.0 - np.exp(-dt_render / max(SMOOTH_TAU, 1e-3))
            self._disp_pos = (1 - alpha) * self._disp_pos + alpha * tgt_pos
            self._disp_quat = quat_slerp(self._disp_quat, tgt_quat, alpha)
            return self._disp_pos.copy(), self._disp_quat.copy(), s.fix_quality, age

    def telemetry(self):
        with self._lock:
            if not self._have:
                return None
            age = time.monotonic() - self._recv_monotonic
            rate = (1.0 / self._dt_ema) if self._dt_ema > 1e-6 else float("nan")
            return age, rate, self._e2e_ms, self._sample.fix_quality


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
        self._last_tick = time.monotonic()
        # throughput: pose recv bytes (cloud bytes come from BlockSync stats)
        self._pose_bytes = 0
        self._status = {}                # latest server-published metrics (JSON)
        self._m_prev_t = time.monotonic()
        self._m_prev_pose = 0
        self._m_pose_bps = 0.0

        log.info(f"[Viewer] Connecting to Zenoh at {ZENOH_ROUTER}...")
        self._z = self._open(self._conf())
        self._z_fast = self._open(self._conf())          # isolate low-latency pose/legs
        log.info("[Viewer] Connected (2 sessions: bulk + low-latency).")

        # bulk session: DIFF-BASED block sync (manifest + Draco bundles, only the
        # cubes that changed) + trajectory + server status.
        self._blocksync = BlockSync(self._z, cube_m=CUBE_SIZE, server_prefix=SERVER_PREFIX)
        self._z.declare_subscriber(_KEYS["trajectory"], self._on_traj)
        self._z.declare_subscriber(_KEYS["status"], self._on_status)
        self._pub_reset = self._z.declare_publisher(RESET_KEY)
        # low-latency session: authoritative pose + leg FK + body height
        self._z_fast.declare_subscriber(_KEYS["pose"], self._on_pose)
        self._body_tracker = RobotStateTracker(self._z_fast, ROBOT_NAME)
        self._leg_tracker = LowStateTracker(self._z_fast, ROBOT_NAME)
        log.info(f"[Viewer] subscribed: [bulk] block-sync + trajectory | [fast] pose, "
                 f"legs←'{ROBOT_NAME}/rt/lowstate'")

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
        try:
            pts = proto.unpack_trajectory(bytes(sample.payload))
        except proto.ProtocolError:
            return
        if pts.shape[0] >= 2:
            with self._traj_lock:
                self._traj = pts.astype(np.float32)

    def _on_pose(self, sample):
        buf = bytes(sample.payload)
        self._pose_bytes += len(buf)
        try:
            self._predictor.on_pose(proto.unpack_pose(buf))
        except proto.ProtocolError:
            pass

    def _on_status(self, sample):
        try:
            self._status = json.loads(bytes(sample.payload).decode())
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

        self._cloud_vis = scene.visuals.Markers(parent=view.scene)
        self._cloud_vis.set_gl_state(depth_test=True)
        self._robot_vis = scene.visuals.Line(parent=view.scene, connect="segments",
                                              width=2.0, antialias=True)
        self._legs_vis = scene.visuals.Line(parent=view.scene, connect="segments",
                                             width=3.0, antialias=True)
        self._feet_vis = scene.visuals.Markers(parent=view.scene)
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

        @canvas.events.key_press.connect
        def _on_key(ev):
            k = (ev.text or "").lower()
            if k == "1":
                self._blocksync.force_resync()      # drop local cubes → full refetch
            elif k == "r":
                self._reset_map()
            elif k == "f":
                self._view.camera.set_range()
            elif k == ",":
                self._set_yaw(self._yaw_offset_deg - 5)
            elif k == ".":
                self._set_yaw(self._yaw_offset_deg + 5)
            elif k == "/":
                self._set_yaw(0.0)
            elif k in ("n", "m"):
                self._pt_size = float(np.clip(self._pt_size + (1 if k == "m" else -1), 1, 12))
                log.info(f"[Viewer] point size = {self._pt_size:.0f}")

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
                self._z.close(); self._z_fast.close()
            except Exception:
                pass
            log.info("[Viewer] Shut down.")

    @staticmethod
    def _print_controls():
        log.info("[Viewer] cloud STREAMS automatically. keys:  1 force re-fetch | "
                 "R reset map | F refit | , / .  yaw ∓5 | / yaw 0 | N / M  point size ∓  "
                 "(mouse: drag rotate, scroll zoom)")

    def _set_yaw(self, deg):
        self._yaw_offset_deg = float(deg)
        log.info(f"[Viewer] cloud↔robot yaw offset = {self._yaw_offset_deg:.0f}°")

    def _on_tick(self, event):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now

        # cloud: BlockSync hands us the merged map only when a cube changed
        m = self._blocksync.take_merged()
        if m is not None:
            xyz, rgb = m
            if xyz.shape[0]:
                keep = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).max(axis=1) <= CLOUD_MAX_M)
                xyz, rgb = xyz[keep], rgb[keep]
                rgba = np.ones((xyz.shape[0], 4), np.float32)
                rgba[:, :3] = rgb.astype(np.float32) / 255.0
                self._cloud_vis.set_data(xyz.astype(np.float32), face_color=rgba,
                                         size=self._pt_size, edge_width=0)
                self._cloud_n = int(xyz.shape[0])
                if not self._cloud_framed:
                    self._view.camera.set_range()
                    self._cloud_framed = True
            else:
                self._cloud_vis.set_data(np.zeros((0, 3), np.float32))
                self._cloud_n = 0

        # trajectory
        with self._traj_lock:
            traj = self._traj
        if traj is not None and traj.shape[0] >= 2:
            t = traj @ _yaw_R(self._yaw_offset_deg).T if self._yaw_offset_deg else traj
            self._traj_vis.set_data(pos=t.astype(np.float32))

        # robot + legs at the predicted pose
        pred = self._predictor.step(dt)
        if pred is not None:
            pos, quat, fix, age = pred
            R = quat_to_R(quat)
            if self._yaw_offset_deg:
                Y = _yaw_R(self._yaw_offset_deg)
                R, pos = Y @ R, Y @ pos
            body_col = COL_CORRECTED if fix == proto.FIX_CORRECTED else COL_DEADRECKON
            rpts, rcols = robot_segments(R, pos, body_col)
            self._robot_vis.set_data(pos=rpts, color=rcols)

            leg_data, legs_valid = self._leg_tracker.get()
            if legs_valid and leg_data:
                lpts, lcols, feet, fcols = leg_segments(leg_data, R, pos)
                self._legs_vis.set_data(pos=lpts, color=lcols)
                self._feet_vis.set_data(feet, face_color=fcols, size=10, edge_width=0)
            elif not self._legs_warned:
                log.info("[Viewer] (no leg data yet — /lowstate flowing?)")
                self._legs_warned = True

        self._update_hud()

    def _update_hud(self):
        # recompute pose receive throughput every ~0.5 s (cumulative byte deltas)
        now = time.monotonic()
        el = now - self._m_prev_t
        if el >= 0.5:
            self._m_pose_bps = (self._pose_bytes - self._m_prev_pose) / el
            self._m_prev_t, self._m_prev_pose = now, self._pose_bytes

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
                         f"e2e {e2e_s}  {self._m_pose_bps/1024:5.1f} KB/s  fix {fixs}{warn}")

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
