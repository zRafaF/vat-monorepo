"""
VAT — PRISM Open3D Viewer  (POC)
================================
A lightweight, **native** live viewer for the PRISM map + robot avatar, built on
Open3D (OpenGL) instead of Rerun. No gRPC/spawn streaming layer, no wgpu — just a
local window updated from a poll loop, so it stays responsive on flaky/VPN links
and adds almost no dependencies beyond what the bring-up tools already use.

It renders, live:
  * the PRISM **point cloud** (full snapshot per submap, versioned);
  * the **robot block** at the client-predicted pose (netcode-style dead reckoning
    between samples), coloured green when VGGT-corrected / amber when coasting;
  * the four **legs** from ``/lowstate`` forward kinematics + the selfie-stick;
  * the camera **trajectory**;
  * the live 360° **camera** image in a separate OpenCV window.

Controls (type in the terminal, then Enter): ``r`` reset map · ``y <deg>`` set the
cloud↔robot yaw align · ``+``/``-`` nudge yaw ±5°. Press ``q`` / close the window
or Ctrl+C to quit.

Usage
-----
  cd client && uv sync
  uv run python prism_viewer.py                 # localhost router
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

import cv2
import numpy as np
import open3d as o3d
import zenoh

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "common"))
sys.path.insert(0, os.path.join(_ROOT, "robot", "docker"))
import vat_protocol as proto  # noqa: E402
from vat_protocol import (  # noqa: E402
    quat_identity, quat_normalize, quat_mul, quat_slerp, integrate_pose)
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
SHOW_CAMERA   = os.environ.get("SHOW_CAMERA", "1") not in ("0", "false", "False")

_KEYS = proto.keys(ROBOT_NAME, SERVER_PREFIX)
RESET_KEY = f"{SERVER_PREFIX}/cmd/reset"

# Go2-W footprint half-sizes (m): length × width × height
ROBOT_HALF = np.array([0.35, 0.16, 0.18], dtype=np.float64)
STICK = np.array([
    float(os.environ.get("STICK_OFFSET_X", "-0.20")),
    float(os.environ.get("STICK_OFFSET_Y", "0.0")),
    float(os.environ.get("STICK_OFFSET_Z", "0.55")),
], dtype=np.float64)
FOOT_COLORS = {"FR": [1.0, 0.3, 0.3], "FL": [0.3, 1.0, 0.3],
               "RR": [0.3, 0.6, 1.0], "RL": [1.0, 0.85, 0.25]}
COL_CORRECTED  = [0.31, 0.90, 0.47]   # green
COL_DEADRECKON = [1.0, 0.74, 0.23]    # amber


# ─────────────────────────────────────────────────────────────────────────────
# Math helpers
# ─────────────────────────────────────────────────────────────────────────────


def quat_to_R(q: np.ndarray) -> np.ndarray:
    """xyzw quaternion → 3×3 rotation matrix."""
    x, y, z, w = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _yaw_R(deg: float) -> np.ndarray:
    r = np.deg2rad(deg)
    c, s = np.cos(r), np.sin(r)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


# ─────────────────────────────────────────────────────────────────────────────
# Shared state holders (written by Zenoh callbacks, read by the render loop)
# ─────────────────────────────────────────────────────────────────────────────


class LocalCloud:
    def __init__(self):
        self._lock = threading.Lock()
        # version-keyed blocks: a snapshot resets to {0: full}, deltas append a
        # new versioned block. The server sends a fresh keyframe snapshot every
        # few submaps to correct any missed/recoloured blocks.
        self._blocks: dict[int, tuple] = {}
        self._dirty = False
        self._cleared = False

    def apply_snapshot(self, version, xyz, rgb):
        if xyz.shape[0] == 0:
            self.clear()
            log.info(f"[Cloud] empty snapshot v{version} → cleared")
            return
        with self._lock:
            self._blocks = {0: (xyz, rgb)}
            self._dirty = True
        log.info(f"[Cloud] snapshot v{version}: {xyz.shape[0]} pts")

    def apply_delta(self, version, xyz, rgb, since_version):
        if xyz.shape[0] == 0:
            return
        with self._lock:
            self._blocks[int(version)] = (xyz, rgb)
            self._dirty = True
        log.info(f"[Cloud] delta v{since_version}→{version}: +{xyz.shape[0]} pts")

    def clear(self):
        with self._lock:
            self._blocks = {}
            self._dirty = False
            self._cleared = True

    def pop_cleared(self) -> bool:
        with self._lock:
            c, self._cleared = self._cleared, False
            return c

    def take_if_dirty(self):
        with self._lock:
            if not self._dirty or not self._blocks:
                return None
            self._dirty = False
            xyz = np.concatenate([b[0] for b in self._blocks.values()])
            rgb = np.concatenate([b[1] for b in self._blocks.values()])
            return xyz, rgb


class PosePredictor:
    """Latest authoritative pose, extrapolated to 'now' (netcode dead reckoning)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._have = False
        self._sample = None
        self._recv_monotonic = 0.0
        self._disp_pos = np.zeros(3)
        self._disp_quat = quat_identity()

    def on_pose(self, pose: proto.PoseState):
        with self._lock:
            self._sample = pose
            self._recv_monotonic = time.monotonic()
            if not self._have:
                self._disp_pos = pose.position.astype(np.float64)
                self._disp_quat = quat_normalize(pose.quaternion)
                self._have = True

    def step(self, dt_render: float):
        with self._lock:
            if not self._have or self._sample is None:
                return None
            s = self._sample
            age = time.monotonic() - self._recv_monotonic
            if age <= STALE_S:
                scale = 1.0
            else:
                scale = max(0.0, 1.0 - (age - STALE_S) / max(DECAY_S, 1e-3))
            lin = s.linear_velocity * scale
            ang = s.angular_velocity * scale
            tgt_pos, tgt_quat = integrate_pose(s.position, s.quaternion, lin, ang, age)
            alpha = 1.0 - np.exp(-dt_render / max(SMOOTH_TAU, 1e-3))
            self._disp_pos = (1 - alpha) * self._disp_pos + alpha * tgt_pos
            self._disp_quat = quat_slerp(self._disp_quat, tgt_quat, alpha)
            return self._disp_pos.copy(), self._disp_quat.copy(), s.fix_quality, age


# ─────────────────────────────────────────────────────────────────────────────
# Geometry builders (Open3D)
# ─────────────────────────────────────────────────────────────────────────────

# unit box corners (±1) and the 12 edges between them
_BOX_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                         for sz in (-1, 1)], dtype=np.float64)
_BOX_EDGES = [(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
              (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)]


def make_robot_lineset() -> o3d.geometry.LineSet:
    """Oriented body box + a forward 'heading' segment + the selfie-stick."""
    corners = _BOX_CORNERS * ROBOT_HALF                       # 8
    heading = np.array([[0, 0, 0], [ROBOT_HALF[0] * 1.6, 0, 0]])  # 8,9
    stick = np.array([[0, 0, 0], STICK])                      # 10,11
    pts = np.vstack([corners, heading, stick])
    lines = list(_BOX_EDGES) + [(8, 9), (10, 11)]
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(pts)
    ls.lines = o3d.utility.Vector2iVector(np.asarray(lines, dtype=np.int32))
    return ls


def update_robot_lineset(ls, R, t, body_col):
    corners = _BOX_CORNERS * ROBOT_HALF
    heading = np.array([[0, 0, 0], [ROBOT_HALF[0] * 1.6, 0, 0]])
    stick = np.array([[0, 0, 0], STICK])
    pts = np.vstack([corners, heading, stick]) @ R.T + t
    ls.points = o3d.utility.Vector3dVector(pts)
    cols = [body_col] * len(_BOX_EDGES) + [[1.0, 0.3, 0.3]] + [[0.9, 0.9, 0.25]]
    ls.colors = o3d.utility.Vector3dVector(np.asarray(cols, dtype=np.float64))


def make_polyline(n_pts) -> o3d.geometry.LineSet:
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(np.zeros((n_pts, 3)))
    ls.lines = o3d.utility.Vector2iVector(
        np.array([[i, i + 1] for i in range(n_pts - 1)], dtype=np.int32))
    return ls


def set_polyline(ls, pts, color):
    pts = np.asarray(pts, dtype=np.float64)
    n = pts.shape[0]
    ls.points = o3d.utility.Vector3dVector(pts)
    ls.lines = o3d.utility.Vector2iVector(
        np.array([[i, i + 1] for i in range(n - 1)], dtype=np.int32))
    ls.colors = o3d.utility.Vector3dVector(np.tile(color, (max(n - 1, 0), 1)))


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
        self._cam_bgr = None
        self._cam_lock = threading.Lock()
        # latest-only RAW payload holders — callbacks just stash bytes (no decode)
        # so a heavy pcd/jpeg can't hog the Zenoh callback executor and starve the
        # low-latency pose/leg streams. Decoding happens in the render thread.
        self._raw_pcd = None
        self._raw_pcd_lock = threading.Lock()
        self._raw_jpeg = None
        self._raw_jpeg_lock = threading.Lock()
        self._legs_warned = False
        self._stop = threading.Event()

        # TWO sessions on purpose: the bulky, reliable point-cloud + camera live on
        # `_z`; the small, latency-critical pose + leg/odom streams live on their
        # OWN session `_z_fast`, so cloud bursts can't serialize ahead of them on a
        # single session's receive/callback path (the starvation you observed when
        # the mapping server was running).
        log.info(f"[Viewer] Connecting to Zenoh at {ZENOH_ROUTER}...")
        self._z = self._open_with_retry(self._client_conf())
        self._z_fast = self._open_with_retry(self._client_conf())
        log.info("[Viewer] Connected (2 sessions: bulk + low-latency).")

        # bulk session: point cloud, trajectory, status, camera, reset
        self._z.declare_subscriber(_KEYS["pcd_delta"],    self._on_pcd)
        self._z.declare_subscriber(_KEYS["pcd_snapshot"], self._on_pcd)
        self._z.declare_subscriber(_KEYS["trajectory"],   self._on_traj)
        self._z.declare_subscriber(_KEYS["status"],       self._on_status)
        self._z.declare_subscriber(_KEYS["camera_frame"], self._on_frame)
        self._pub_reset = self._z.declare_publisher(RESET_KEY)
        # low-latency session: authoritative pose + leg FK + body height
        self._z_fast.declare_subscriber(_KEYS["pose"],    self._on_pose)
        self._body_tracker = RobotStateTracker(self._z_fast, ROBOT_NAME)
        self._leg_tracker = LowStateTracker(self._z_fast, ROBOT_NAME)
        log.info(f"[Viewer] subscribed: [bulk] pcd, trajectory, status, camera | "
                 f"[fast] pose, legs←'{ROBOT_NAME}/rt/lowstate'")

        if request_snapshot:
            self._request_snapshot()

    @staticmethod
    def _client_conf():
        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", f'["{ZENOH_ROUTER}"]')
        conf.insert_json5("mode", '"client"')
        return conf

    @staticmethod
    def _open_with_retry(conf):
        while True:
            try:
                return zenoh.open(conf)
            except Exception as e:
                log.warning(f"[Viewer] Zenoh connect failed: {e} — retry in 5s")
                time.sleep(5)

    # ── subscriber callbacks ────────────────────────────────────────────────
    # HEAVY streams: stash the latest raw bytes only (no decode here) so the
    # Zenoh callback returns instantly and never blocks pose/leg delivery.
    def _on_pcd(self, sample):
        with self._raw_pcd_lock:
            self._raw_pcd = bytes(sample.payload)   # latest wins; stale dropped

    def _on_frame(self, sample):
        if not SHOW_CAMERA:
            return
        with self._raw_jpeg_lock:
            self._raw_jpeg = bytes(sample.payload)

    # LIGHT streams: tiny payloads, safe to decode inline.
    def _on_traj(self, sample):
        try:
            pts = proto.unpack_trajectory(bytes(sample.payload))
        except proto.ProtocolError:
            return
        if pts.shape[0] >= 2:
            with self._traj_lock:
                self._traj = pts.astype(np.float64)

    def _on_pose(self, sample):
        try:
            self._predictor.on_pose(proto.unpack_pose(bytes(sample.payload)))
        except proto.ProtocolError:
            pass

    def _on_status(self, sample):
        pass

    # ── in-window key controls (bound to the Open3D window) ─────────────────
    @staticmethod
    def _print_controls():
        log.info("[Viewer] Window keys:  R reset map | F refit view | "
                 ", / .  yaw -5/+5 | / yaw=0 | "
                 "N / M  cloud point size -/+ | "
                 "(Open3D: drag rotate, scroll zoom, H help)")

    def _key_reset(self, vis):
        self._reset_map();  return False

    def _key_yaw_minus(self, vis):
        self._set_yaw(self._yaw_offset_deg - 5);  return False

    def _key_yaw_plus(self, vis):
        self._set_yaw(self._yaw_offset_deg + 5);  return False

    def _key_yaw_zero(self, vis):
        self._set_yaw(0.0);  return False

    @staticmethod
    def _key_psize(vis, delta):
        o = vis.get_render_option()
        o.point_size = float(np.clip(o.point_size + delta, 1.0, 12.0))
        log.info(f"[Viewer] cloud point size = {o.point_size:.0f}")
        return False

    def _set_yaw(self, deg):
        self._yaw_offset_deg = float(deg)
        log.info(f"[Viewer] cloud↔robot yaw offset = {self._yaw_offset_deg:.0f}°")

    def _reset_map(self):
        log.info("[Viewer] RESET → clear cloud + wipe server map")
        try:
            self._pub_reset.put(b"reset")
        except Exception as e:
            log.warning(f"[Viewer] reset publish failed: {e}")
        self._cloud.clear()

    def _request_snapshot(self):
        log.info(f"[Viewer] requesting snapshot from '{_KEYS['pcd_snapshot']}'...")
        try:
            for reply in self._z.get(_KEYS["pcd_snapshot"], timeout=5.0):
                if reply.ok:
                    data = bytes(reply.result.payload)
                    if len(data) > 20:
                        v, xyz, rgb, *_ = proto.unpack_pcd(data)
                        self._cloud.apply_snapshot(v, xyz, rgb)
        except Exception as e:
            log.warning(f"[Viewer] snapshot request failed: {e}")

    # ── render (Open3D drives a continuous loop via the animation callback) ──
    def run(self):
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="VAT — PRISM Viewer", width=1280, height=800)
        opt = vis.get_render_option()
        opt.background_color = np.array([0.05, 0.05, 0.06])
        opt.point_size = 2.0

        # geometries kept on self so the animation callback can update them
        self._pcd = o3d.geometry.PointCloud()
        self._robot = make_robot_lineset()
        update_robot_lineset(self._robot, np.eye(3), np.zeros(3), COL_DEADRECKON)
        self._traj_ls = make_polyline(2)
        self._legs = {leg: make_polyline(4) for leg in LEG_ORDER}
        self._axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        self._cloud_framed = False
        self._first_cloud_logged = False
        self._last_anim = time.monotonic()

        # Pull any snapshot already received at startup so the cloud is non-empty
        # before we add it (lets the initial view frame the real map).
        self._decode_pending_cloud()

        # CRITICAL: add EVERY geometry BEFORE vis.run(). Open3D does NOT reliably
        # render geometry first add_geometry()'d from inside the animation callback
        # — that's why the cloud/robot/legs never appeared. From here we only
        # mutate points and call update_geometry().
        vis.add_geometry(self._pcd, reset_bounding_box=True)
        for g in (self._robot, self._traj_ls, *self._legs.values(), self._axes):
            vis.add_geometry(g, reset_bounding_box=False)
        self._cloud_framed = len(self._pcd.points) > 0

        # in-window key bindings (replaces the old stdin loop)
        vis.register_key_callback(ord('R'), self._key_reset)
        vis.register_key_callback(ord('F'), self._refit)
        vis.register_key_callback(ord(','), self._key_yaw_minus)
        vis.register_key_callback(ord('.'), self._key_yaw_plus)
        vis.register_key_callback(ord('/'), self._key_yaw_zero)
        vis.register_key_callback(ord('N'), lambda v: self._key_psize(v, -1.0))
        vis.register_key_callback(ord('M'), lambda v: self._key_psize(v, +1.0))
        # Open3D calls this every rendered frame → smooth, continuous updates
        vis.register_animation_callback(self._on_anim)

        self._print_controls()
        log.info("[Viewer] Rendering. Close the window or press the window's X.")
        try:
            vis.run()                         # blocks; pumps events + animation cb
        except KeyboardInterrupt:
            pass
        finally:
            self._stop.set()
            vis.destroy_window()
            if SHOW_CAMERA:
                cv2.destroyAllWindows()
            self._z.close()
            self._z_fast.close()
            log.info("[Viewer] Shut down.")

    def _refit(self, vis):
        """Recompute camera bounds to fit current geometry (bound to 'F')."""
        vis.reset_view_point(True)
        return False

    def _decode_pending_cloud(self) -> bool:
        """Decode the latest raw pcd (render thread) and push it into self._pcd.
        Returns True if the cloud geometry changed (so the caller can
        update_geometry). Heavy decode lives here, never in the Zenoh callback."""
        with self._raw_pcd_lock:
            raw, self._raw_pcd = self._raw_pcd, None
        if raw is not None:
            try:
                v, xyz, rgb, is_snap, since_v = proto.unpack_pcd(raw)
                if is_snap:
                    self._cloud.apply_snapshot(v, xyz, rgb)
                else:
                    self._cloud.apply_delta(v, xyz, rgb, since_v)
            except proto.ProtocolError as e:
                log.warning(f"[Viewer] pcd decode: {e}")
        changed = False
        if self._cloud.pop_cleared():
            self._pcd.clear()
            changed = True
        c = self._cloud.take_if_dirty()
        if c is not None:
            xyz, rgb = c
            n0 = xyz.shape[0]
            keep = np.isfinite(xyz).all(axis=1) & \
                (np.abs(xyz).max(axis=1) <= CLOUD_MAX_M)
            xyz, rgb = xyz[keep], rgb[keep]
            self._pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
            self._pcd.colors = o3d.utility.Vector3dVector(
                np.clip(rgb, 0, 1).astype(np.float64))
            changed = True
            if not self._first_cloud_logged:
                self._first_cloud_logged = True
                log.info(f"[Viewer] first cloud: {xyz.shape[0]}/{n0} pts "
                         f"({n0 - xyz.shape[0]} dropped)")
        return changed

    def _on_anim(self, vis) -> bool:
        """Called by Open3D every frame. Pull the latest stashed data, predict
        the robot pose, refresh geometries + the camera window. Returns True to
        request a redraw so motion stays smooth between data arrivals."""
        now = time.monotonic()
        dt = now - self._last_anim
        self._last_anim = now

        # point cloud (geometry is pre-added; we only mutate + update here)
        if self._decode_pending_cloud():
            vis.update_geometry(self._pcd)
            if not self._cloud_framed and len(self._pcd.points) > 0:
                vis.reset_view_point(True)     # frame the first real cloud
                self._cloud_framed = True

        # trajectory
        with self._traj_lock:
            traj = self._traj
        if traj is not None and traj.shape[0] >= 2:
            t = traj.copy()
            if self._yaw_offset_deg:
                t = t @ _yaw_R(self._yaw_offset_deg).T
            set_polyline(self._traj_ls, t, [1.0, 0.78, 0.24])
            vis.update_geometry(self._traj_ls)

        # robot block + legs at the predicted pose (every frame → smooth)
        pred = self._predictor.step(dt)
        if pred is not None:
            pos, quat, fix, age = pred
            R = quat_to_R(quat)
            if self._yaw_offset_deg:
                Y = _yaw_R(self._yaw_offset_deg)
                R, pos = Y @ R, Y @ pos
            body_col = COL_CORRECTED if fix == proto.FIX_CORRECTED else COL_DEADRECKON
            update_robot_lineset(self._robot, R, pos, body_col)
            vis.update_geometry(self._robot)

            leg_data, legs_valid = self._leg_tracker.get()
            if legs_valid and leg_data:
                for leg in LEG_ORDER:
                    p = leg_data[leg]
                    chain = np.vstack([p["hip"], p["thigh_root"],
                                       p["knee"], p["foot"]])
                    world = chain @ R.T + pos
                    set_polyline(self._legs[leg], world, FOOT_COLORS[leg])
                    vis.update_geometry(self._legs[leg])
            elif not self._legs_warned:
                log.info("[Viewer] (no leg data yet — /lowstate flowing?)")
                self._legs_warned = True

        # camera: decode the latest raw JPEG HERE (render thread), then show it
        if SHOW_CAMERA:
            with self._raw_jpeg_lock:
                raw_jpeg, self._raw_jpeg = self._raw_jpeg, None
            if raw_jpeg is not None:
                try:
                    _, _, _, jpeg = proto.unpack_frame(raw_jpeg)
                    bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                    if bgr is not None:
                        self._cam_bgr = bgr
                except Exception:
                    pass
            if self._cam_bgr is not None:
                cv2.imshow("VAT — 360° camera", self._cam_bgr)
                cv2.waitKey(1)

        return True   # always redraw → predicted motion stays smooth


def main():
    parser = argparse.ArgumentParser(description="VAT PRISM Open3D viewer")
    parser.add_argument("--snapshot", action="store_true",
                        help="Request full snapshot from server on start")
    args = parser.parse_args()
    PRISMViewer(request_snapshot=args.snapshot).run()


if __name__ == "__main__":
    main()
