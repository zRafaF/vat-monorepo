"""
VAT — PRISM Rerun Viewer
========================
Renders the live PRISM map *and* the robot avatar in Rerun.

What's new vs. the first POC
----------------------------
* Subscribes to the **authoritative robot pose** the robot publishes
  (``{robot}/prism/pose``), relayed by the server's Zenoh router.
* Runs a **client-side predictor** (multiplayer-netcode style): between pose
  samples it dead-reckons the avatar using the linear + angular velocity in
  each message, slerps orientation, blends on correction, and decays velocity
  when the stream goes stale.  This keeps the robot block moving smoothly at the
  render rate even though poses arrive intermittently and late.
* Draws a **robot-position block** (an oriented box sized like the Go2-W) at the
  predicted pose, coloured by fix quality (green = VGGT-corrected, amber =
  dead-reckoning on odometry only).

Usage
-----
  uv sync --package vat-client
  python client/prism_rerun_viewer.py                 # localhost router
  ZENOH_ROUTER=tcp/<ip>:7447 python client/prism_rerun_viewer.py
  python client/prism_rerun_viewer.py --snapshot      # request full cloud on start
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
import rerun as rr
import zenoh

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "common"))
import vat_protocol as proto  # noqa: E402
from vat_protocol import quat_identity, quat_normalize, quat_slerp, integrate_pose  # noqa: E402

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="[%(asctime)s] [%(levelname)s] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("prism-viewer")

ZENOH_ROUTER  = os.environ.get("ZENOH_ROUTER",  "tcp/127.0.0.1:7447")
ROBOT_NAME    = os.environ.get("ROBOT_NAME",    "go2")
SERVER_PREFIX = os.environ.get("SERVER_PREFIX", "server/prism")
RENDER_HZ     = float(os.environ.get("RENDER_HZ", "60.0"))
STALE_S       = float(os.environ.get("POSE_STALE_S", "0.5"))    # extrapolate freely until here
DECAY_S       = float(os.environ.get("POSE_DECAY_S", "1.0"))    # then ramp velocity to zero
SMOOTH_TAU    = float(os.environ.get("POSE_SMOOTH_TAU", "0.08"))  # reconciliation time const

_KEYS = proto.keys(ROBOT_NAME, SERVER_PREFIX)

# Go2-W footprint (approx, metres): length × width × height
ROBOT_HALF = np.array([0.35, 0.16, 0.18], dtype=np.float32)

RR_WORLD = "world"
RR_PCD   = f"{RR_WORLD}/point_cloud"
RR_TRAJ  = f"{RR_WORLD}/camera_trajectory"
RR_ROBOT = f"{RR_WORLD}/robot"


# ─────────────────────────────────────────────────────────────────────────────
# Point cloud accumulator (versioned blocks)
# ─────────────────────────────────────────────────────────────────────────────


class LocalCloud:
    def __init__(self):
        self._lock = threading.Lock()
        self._blocks: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        self._version = 0
        self._dirty = False

    def apply_snapshot(self, version, xyz, rgb):
        with self._lock:
            self._blocks = {0: (xyz, rgb)}
            self._version = version
            self._dirty = True
        log.info(f"[Cloud] snapshot v{version}: {xyz.shape[0]} pts")

    def apply_delta(self, version, xyz, rgb, since_version):
        if xyz.shape[0] == 0:
            return
        with self._lock:
            self._blocks[version] = (xyz, rgb)
            self._version = version
            self._dirty = True
        log.info(f"[Cloud] delta v{since_version}→{version}: +{xyz.shape[0]} pts")

    def take_if_dirty(self):
        with self._lock:
            if not self._dirty or not self._blocks:
                return None
            self._dirty = False
            xyz = np.concatenate([b[0] for b in self._blocks.values()])
            rgb = np.concatenate([b[1] for b in self._blocks.values()])
            return xyz, rgb


# ─────────────────────────────────────────────────────────────────────────────
# Client-side pose predictor (netcode-style dead reckoning + reconciliation)
# ─────────────────────────────────────────────────────────────────────────────


class PosePredictor:
    """Holds the latest authoritative pose and extrapolates it to 'now'."""

    def __init__(self):
        self._lock = threading.Lock()
        self._have = False
        self._sample: proto.PoseState | None = None
        self._recv_monotonic = 0.0
        # displayed (smoothed) render state
        self._disp_pos = np.zeros(3, np.float32)
        self._disp_quat = quat_identity().astype(np.float32)

    def on_pose(self, pose: proto.PoseState):
        with self._lock:
            self._sample = pose
            self._recv_monotonic = time.monotonic()
            if not self._have:
                self._disp_pos = pose.position.astype(np.float32)
                self._disp_quat = quat_normalize(pose.quaternion).astype(np.float32)
                self._have = True

    def step(self, dt_render: float):
        """Advance the displayed state toward the extrapolated target.
        Returns (position, quaternion, fix_quality, age_s) or None."""
        with self._lock:
            if not self._have or self._sample is None:
                return None
            s = self._sample
            age = time.monotonic() - self._recv_monotonic

            # velocity decay once the stream is stale → coast to a stop
            if age <= STALE_S:
                scale = 1.0
            else:
                scale = max(0.0, 1.0 - (age - STALE_S) / max(DECAY_S, 1e-3))
            lin = s.linear_velocity * scale
            ang = s.angular_velocity * scale

            tgt_pos, tgt_quat = integrate_pose(s.position, s.quaternion, lin, ang, age)

            # critically-damped blend of displayed → target (reconciliation)
            alpha = 1.0 - np.exp(-dt_render / max(SMOOTH_TAU, 1e-3))
            self._disp_pos = ((1 - alpha) * self._disp_pos + alpha * tgt_pos).astype(np.float32)
            self._disp_quat = quat_slerp(self._disp_quat, tgt_quat, alpha).astype(np.float32)
            return self._disp_pos.copy(), self._disp_quat.copy(), s.fix_quality, age


# ─────────────────────────────────────────────────────────────────────────────
# Viewer
# ─────────────────────────────────────────────────────────────────────────────


class PRISMViewer:
    def __init__(self, request_snapshot=False):
        rr.init("VAT-PRISM-Viewer", spawn=True)
        rr.log(RR_WORLD, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        # static robot body box (sits under the robot transform)
        rr.log(f"{RR_ROBOT}/body",
               rr.Boxes3D(half_sizes=[ROBOT_HALF], colors=[[120, 180, 255]]),
               static=True)
        rr.log(f"{RR_ROBOT}/heading",
               rr.Arrows3D(origins=[[0, 0, 0]], vectors=[[0.5, 0, 0]],
                           colors=[[255, 80, 80]]), static=True)

        self._cloud = LocalCloud()
        self._predictor = PosePredictor()

        log.info(f"[Viewer] Connecting to Zenoh at {ZENOH_ROUTER}...")
        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", f'["{ZENOH_ROUTER}"]')
        conf.insert_json5("mode", '"client"')
        self._z = self._open_with_retry(conf)
        log.info("[Viewer] Connected.")

        self._z.declare_subscriber(_KEYS["pcd_delta"],    self._on_pcd)
        self._z.declare_subscriber(_KEYS["pcd_snapshot"], self._on_pcd)
        self._z.declare_subscriber(_KEYS["trajectory"],   self._on_traj)
        self._z.declare_subscriber(_KEYS["status"],       self._on_status)
        self._z.declare_subscriber(_KEYS["pose"],         self._on_pose)
        log.info(f"[Viewer] subscribed: pcd, trajectory, status, pose "
                 f"('{_KEYS['pose']}')")

        if request_snapshot:
            self._request_snapshot()

        self._stop = threading.Event()
        self._render_thread = threading.Thread(target=self._render_loop, daemon=True)
        self._render_thread.start()

    @staticmethod
    def _open_with_retry(conf):
        while True:
            try:
                return zenoh.open(conf)
            except Exception as e:
                log.warning(f"[Viewer] Zenoh connect failed: {e} — retrying in 5s")
                time.sleep(5)

    # ── subscriber callbacks (keep light; no blocking) ─────────────────────────

    def _on_pcd(self, sample):
        try:
            v, xyz, rgb, is_snap, since_v = proto.unpack_pcd(bytes(sample.payload))
        except proto.ProtocolError as e:
            log.warning(f"[Viewer] pcd decode: {e}")
            return
        if is_snap:
            self._cloud.apply_snapshot(v, xyz, rgb)
        else:
            self._cloud.apply_delta(v, xyz, rgb, since_v)

    def _on_traj(self, sample):
        try:
            pts = proto.unpack_trajectory(bytes(sample.payload))
        except proto.ProtocolError as e:
            log.warning(f"[Viewer] traj decode: {e}")
            return
        if pts.shape[0] >= 2:
            rr.log(RR_TRAJ, rr.LineStrips3D([pts], colors=[[255, 200, 60]], radii=0.01))

    def _on_pose(self, sample):
        try:
            pose = proto.unpack_pose(bytes(sample.payload))
        except proto.ProtocolError as e:
            log.warning(f"[Viewer] pose decode: {e}")
            return
        self._predictor.on_pose(pose)

    def _on_status(self, sample):
        try:
            status = json.loads(bytes(sample.payload).decode())
            rr.log("status", rr.TextLog(json.dumps(status)))
        except Exception:
            pass

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

    # ── render loop ─────────────────────────────────────────────────────────────

    def _render_loop(self):
        period = 1.0 / max(RENDER_HZ, 1.0)
        last = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            dt = now - last
            last = now

            # point cloud (only when changed)
            cloud = self._cloud.take_if_dirty()
            if cloud is not None:
                xyz, rgb = cloud
                colors = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
                rr.log(RR_PCD, rr.Points3D(positions=xyz, colors=colors, radii=0.01))

            # robot avatar (every frame, predicted)
            pred = self._predictor.step(dt)
            if pred is not None:
                pos, quat, fix, age = pred
                rr.log(RR_ROBOT, rr.Transform3D(
                    translation=pos, rotation=rr.Quaternion(xyzw=quat)))
                color = [80, 220, 120] if fix == proto.FIX_CORRECTED else [255, 190, 60]
                rr.log(f"{RR_ROBOT}/body",
                       rr.Boxes3D(half_sizes=[ROBOT_HALF], colors=[color]))

            time.sleep(max(0.0, period - (time.monotonic() - now)))

    def run(self):
        log.info("[Viewer] Rendering. Ctrl+C to quit.")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            log.info("[Viewer] Shutting down.")
        finally:
            self._stop.set()
            self._z.close()


def main():
    parser = argparse.ArgumentParser(description="VAT PRISM Rerun viewer")
    parser.add_argument("--snapshot", action="store_true",
                        help="Request full snapshot from server on start")
    args = parser.parse_args()
    PRISMViewer(request_snapshot=args.snapshot).run()


if __name__ == "__main__":
    main()
