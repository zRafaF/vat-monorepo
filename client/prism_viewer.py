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
  * the PRISM **point cloud**, fetched ON DEMAND (press ``1``) — a full freshest
    snapshot that *replaces* the local cloud (no stale/duplicated delta blocks),
    16-bit-quantised + zlib on the wire (see vat_protocol).

Metric scale note: the cloud is anchored to metric scale ONCE at map start (from
the first frame's camera height); subsequent submaps inherit scale through the
overlap chain. So there is no per-frame scale to track here — the viewer just
re-fetches the current map when you ask.

In-window keys: ``1`` fetch cloud · ``R`` reset map · ``F`` refit view ·
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
CLOUD_MAX_M   = float(os.environ.get("CLOUD_MAX_M", "50.0"))
PT_SIZE       = float(os.environ.get("PCD_POINT_SIZE", "3.0"))

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


class LocalCloud:
    def __init__(self):
        self._lock = threading.Lock()
        self._xyz = None
        self._rgb = None
        self._dirty = False
        self._cleared = False

    def apply_snapshot(self, version, xyz, rgb):
        if xyz.shape[0] == 0:
            self.clear()
            log.info(f"[Cloud] empty snapshot v{version} → cleared")
            return
        with self._lock:
            self._xyz, self._rgb, self._dirty = xyz, rgb, True
        log.info(f"[Cloud] snapshot v{version}: {xyz.shape[0]} pts")

    def clear(self):
        with self._lock:
            self._xyz = self._rgb = None
            self._dirty = False
            self._cleared = True

    def pop_cleared(self):
        with self._lock:
            c, self._cleared = self._cleared, False
            return c

    def take_if_dirty(self):
        with self._lock:
            if not self._dirty or self._xyz is None:
                return None
            self._dirty = False
            return self._xyz, self._rgb


class PosePredictor:
    def __init__(self):
        self._lock = threading.Lock()
        self._have = False
        self._sample = None
        self._recv_monotonic = 0.0
        self._disp_pos = np.zeros(3)
        self._disp_quat = quat_identity()

    def on_pose(self, pose):
        with self._lock:
            self._sample = pose
            self._recv_monotonic = time.monotonic()
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
            scale = 1.0 if age <= STALE_S else max(0.0, 1.0 - (age - STALE_S) / max(DECAY_S, 1e-3))
            tgt_pos, tgt_quat = integrate_pose(
                s.position, s.quaternion, s.linear_velocity * scale,
                s.angular_velocity * scale, age)
            alpha = 1.0 - np.exp(-dt_render / max(SMOOTH_TAU, 1e-3))
            self._disp_pos = (1 - alpha) * self._disp_pos + alpha * tgt_pos
            self._disp_quat = quat_slerp(self._disp_quat, tgt_quat, alpha)
            return self._disp_pos.copy(), self._disp_quat.copy(), s.fix_quality, age


# ─────────────────────────────────────────────────────────────────────────────
# Viewer
# ─────────────────────────────────────────────────────────────────────────────


class PRISMViewer:
    def __init__(self, request_snapshot=False):
        self._cloud = LocalCloud()
        self._predictor = PosePredictor()
        self._yaw_offset_deg = float(os.environ.get("CLOUD_YAW_OFFSET_DEG", "0"))
        self._traj = None
        self._traj_lock = threading.Lock()
        self._legs_warned = False
        self._fetching = False
        self._cloud_framed = False
        self._pt_size = PT_SIZE
        self._last_tick = time.monotonic()

        log.info(f"[Viewer] Connecting to Zenoh at {ZENOH_ROUTER}...")
        self._z = self._open(self._conf())
        self._z_fast = self._open(self._conf())          # isolate low-latency pose/legs
        log.info("[Viewer] Connected (2 sessions: bulk + low-latency).")

        self._z.declare_subscriber(_KEYS["trajectory"], self._on_traj)
        self._pub_reset = self._z.declare_publisher(RESET_KEY)
        self._z_fast.declare_subscriber(_KEYS["pose"], self._on_pose)
        self._body_tracker = RobotStateTracker(self._z_fast, ROBOT_NAME)
        self._leg_tracker = LowStateTracker(self._z_fast, ROBOT_NAME)
        log.info(f"[Viewer] subscribed: [bulk] trajectory | [fast] pose, "
                 f"legs←'{ROBOT_NAME}/rt/lowstate'  (cloud is on-demand: press '1')")

        if request_snapshot:
            threading.Thread(target=self._request_snapshot, daemon=True).start()

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
        try:
            self._predictor.on_pose(proto.unpack_pose(bytes(sample.payload)))
        except proto.ProtocolError:
            pass

    # ── on-demand cloud fetch (worker thread; heavy decode off the GL loop) ──
    def _request_snapshot(self):
        if self._fetching:
            return
        self._fetching = True
        try:
            log.info(f"[Viewer] fetching snapshot from '{_KEYS['pcd_snapshot']}'…")
            data = None
            for reply in self._z.get(_KEYS["pcd_snapshot"], timeout=10.0):
                try:
                    if reply.ok:
                        buf = bytes(reply.result.payload)
                        if data is None or len(buf) > len(data):
                            data = buf
                except Exception:
                    pass
            if data and len(data) > 20:
                v, xyz, rgb, *_ = proto.unpack_pcd(data)
                self._cloud.apply_snapshot(v, xyz, rgb)
            else:
                log.warning("[Viewer] no snapshot reply (server mapping yet?)")
        except Exception as e:
            log.warning(f"[Viewer] snapshot fetch failed: {e}")
        finally:
            self._fetching = False

    def _reset_map(self):
        log.info("[Viewer] RESET → clear cloud + wipe server map")
        try:
            self._pub_reset.put(b"reset")
        except Exception as e:
            log.warning(f"[Viewer] reset publish failed: {e}")
        self._cloud.clear()
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

        @canvas.events.key_press.connect
        def _on_key(ev):
            k = (ev.text or "").lower()
            if k == "1":
                threading.Thread(target=self._request_snapshot, daemon=True).start()
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
        log.info("[Viewer] keys:  1 fetch newest cloud | R reset map | F refit | "
                 ", / .  yaw ∓5 | / yaw 0 | N / M  point size ∓  (mouse: drag rotate, scroll zoom)")

    def _set_yaw(self, deg):
        self._yaw_offset_deg = float(deg)
        log.info(f"[Viewer] cloud↔robot yaw offset = {self._yaw_offset_deg:.0f}°")

    def _on_tick(self, event):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now

        if self._cloud.pop_cleared():
            self._cloud_vis.set_data(np.zeros((0, 3), np.float32))

        c = self._cloud.take_if_dirty()
        if c is not None:
            xyz, rgb = c
            keep = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).max(axis=1) <= CLOUD_MAX_M)
            xyz, rgb = xyz[keep], rgb[keep]
            if xyz.shape[0]:
                rgba = np.empty((xyz.shape[0], 4), np.float32)
                rgba[:, :3] = np.clip(rgb, 0, 1)
                rgba[:, 3] = 1.0
                self._cloud_vis.set_data(xyz.astype(np.float32), face_color=rgba,
                                         size=self._pt_size, edge_width=0)
                if not self._cloud_framed:
                    self._view.camera.set_range()
                    self._cloud_framed = True
                log.info(f"[Viewer] cloud rendered: {xyz.shape[0]} pts")

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


def main():
    parser = argparse.ArgumentParser(description="VAT PRISM VisPy viewer")
    parser.add_argument("--snapshot", action="store_true",
                        help="Fetch the current cloud on start")
    args = parser.parse_args()
    PRISMViewer(request_snapshot=args.snapshot).run()


if __name__ == "__main__":
    main()
