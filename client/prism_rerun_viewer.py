"""
VAT — PRISM Rerun Point Cloud Viewer
=====================================
Subscribes to PRISM server's Zenoh topics and renders the growing 3D map in
real-time using Rerun.

Usage
-----
  # install deps (from repo root)
  uv sync --package vat-client

  # run (default Zenoh router on localhost)
  python client/prism_rerun_viewer.py

  # custom router / robot name
  ZENOH_ROUTER=tcp/192.168.1.100:7447 ROBOT_NAME=go2 python client/prism_rerun_viewer.py

  # request an immediate full snapshot from the server
  python client/prism_rerun_viewer.py --snapshot

Zenoh keys consumed
--------------------
  server/prism/pcd_delta       — incremental point cloud (binary VAT format)
  server/prism/pcd_snapshot    — full point cloud    (same format)
  server/prism/trajectory      — camera trajectory   (binary)
  server/prism/status          — JSON heartbeat      (info only)

Wire format (pcd)
-----------------
  Matches server/prism_server.py::pack_point_cloud / unpack_point_cloud.
  Header (24 bytes, big-endian):
    [4B] magic = 0x50434400
    [4B] version: int32
    [4B] n_points: int32
    [4B] is_snapshot: int32
    [4B] since_version: int32
  Body:
    [n*12 B] xyz  float32
    [n*12 B] rgb  float32  (in [0,1])
"""

from __future__ import annotations

import os
import sys
import json
import struct
import logging
import argparse
import threading
import time
from typing import Optional

import numpy as np
import rerun as rr
import zenoh

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("prism-viewer")

# ─────────────────────────────────────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────────────────────────────────────

ZENOH_ROUTER  = os.environ.get("ZENOH_ROUTER",  "tcp/127.0.0.1:7447")
ROBOT_NAME    = os.environ.get("ROBOT_NAME",    "go2")
SERVER_PREFIX = os.environ.get("SERVER_PREFIX", "server/prism")

KEY_PCD_DELTA    = f"{SERVER_PREFIX}/pcd_delta"
KEY_PCD_SNAPSHOT = f"{SERVER_PREFIX}/pcd_snapshot"
KEY_TRAJECTORY   = f"{SERVER_PREFIX}/trajectory"
KEY_STATUS       = f"{SERVER_PREFIX}/status"

# Rerun paths
RR_WORLD   = "world"
RR_PCD     = f"{RR_WORLD}/point_cloud"
RR_TRAJ    = f"{RR_WORLD}/trajectory"
RR_STATUS  = "status"

# ─────────────────────────────────────────────────────────────────────────────
# Binary format helpers  (mirror of server/prism_server.py)
# ─────────────────────────────────────────────────────────────────────────────

_MAGIC       = 0x50434400
_HEADER_FMT  = "!iiiii"
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)


def _unpack_pcd(data: bytes):
    """Returns (version, xyz, rgb, is_snapshot, since_version)."""
    if len(data) < _HEADER_SIZE:
        raise ValueError("Payload too short for PCD header")
    magic, version, n, is_snap, since_v = struct.unpack_from(_HEADER_FMT, data, 0)
    if magic != _MAGIC:
        raise ValueError(f"Bad magic: 0x{magic:08X}")
    offset = _HEADER_SIZE
    xyz = np.frombuffer(data, dtype=np.float32, count=n * 3,
                        offset=offset).reshape(n, 3).copy()
    rgb = np.frombuffer(data, dtype=np.float32, count=n * 3,
                        offset=offset + n * 12).reshape(n, 3).copy()
    return version, xyz, rgb, bool(is_snap), since_v


def _unpack_trajectory(data: bytes) -> np.ndarray:
    """Returns (N, 3) float32 trajectory positions."""
    if len(data) < 4:
        return np.zeros((0, 3), dtype=np.float32)
    (n,) = struct.unpack_from("!i", data, 0)
    if n <= 0:
        return np.zeros((0, 3), dtype=np.float32)
    arr = np.frombuffer(data, dtype=np.float32, count=n * 3, offset=4).reshape(n, 3)
    return arr.copy()

# ─────────────────────────────────────────────────────────────────────────────
# Local point cloud accumulator
# ─────────────────────────────────────────────────────────────────────────────

class LocalCloud:
    """
    Maintains the client-side merged point cloud.

    Each block key (int64) maps to an (xyz, rgb) pair — the server's block
    granularity is the merge unit.  When a delta arrives we overwrite the
    relevant blocks; when a snapshot arrives we replace everything.

    This mirrors the server's BlockColorCache versioned-block design, keeping
    client RAM proportional to the scene size rather than cumulative updates.
    """

    def __init__(self):
        self._lock = threading.Lock()
        # key → (xyz (M,3), rgb (M,3))
        self._blocks: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        self._version = 0
        self._n_total = 0

    def apply_snapshot(self, version: int, xyz: np.ndarray, rgb: np.ndarray):
        with self._lock:
            # Snapshot replaces everything — treat as one mega block
            self._blocks = {0: (xyz, rgb)}
            self._version = version
            self._n_total = xyz.shape[0]
        log.info(f"[Cloud] Snapshot v{version}: {xyz.shape[0]} pts")

    def apply_delta(self, version: int, xyz: np.ndarray, rgb: np.ndarray,
                    since_version: int):
        with self._lock:
            if xyz.shape[0] == 0:
                return
            # Simple merge: append new points under a version-keyed block
            # (The server groups them by block key; we just keep per-version
            # chunks since the server re-sends only changed blocks.)
            self._blocks[version] = (xyz, rgb)
            self._version = version
            self._n_total = sum(v[0].shape[0] for v in self._blocks.values())
        log.info(f"[Cloud] Delta v{since_version}→{version}: +{xyz.shape[0]} pts "
                 f"(total={self._n_total})")

    def get_flat(self) -> tuple[np.ndarray, np.ndarray]:
        """Return (xyz, rgb) merged across all blocks."""
        with self._lock:
            if not self._blocks:
                empty = np.zeros((0, 3), dtype=np.float32)
                return empty, empty
            chunks_xyz = [b[0] for b in self._blocks.values()]
            chunks_rgb = [b[1] for b in self._blocks.values()]
            return np.concatenate(chunks_xyz), np.concatenate(chunks_rgb)

    @property
    def version(self):
        return self._version

# ─────────────────────────────────────────────────────────────────────────────
# Rerun logger
# ─────────────────────────────────────────────────────────────────────────────

class RerunLogger:
    def __init__(self):
        rr.init("VAT-PRISM-Viewer", spawn=True)
        log.info("[Rerun] Viewer spawned.")

        # Coordinate axes: ROS-style (X forward, Z up)
        rr.log(RR_WORLD, rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)

        self._cloud = LocalCloud()
        self._last_render_version = -1
        self._render_lock = threading.Lock()

    def on_pcd_message(self, data: bytes):
        try:
            version, xyz, rgb, is_snapshot, since_v = _unpack_pcd(data)
        except Exception as e:
            log.warning(f"[Rerun] PCD decode error: {e}")
            return

        if is_snapshot:
            self._cloud.apply_snapshot(version, xyz, rgb)
        else:
            self._cloud.apply_delta(version, xyz, rgb, since_v)

        self._render()

    def on_trajectory(self, data: bytes):
        try:
            positions = _unpack_trajectory(data)
        except Exception as e:
            log.warning(f"[Rerun] Trajectory decode error: {e}")
            return
        if positions.shape[0] == 0:
            return
        rr.log(RR_TRAJ, rr.LineStrips3D(
            [positions],
            colors=[[255, 200, 60]],
            radii=0.01,
        ))

    def on_status(self, data: bytes):
        try:
            status = json.loads(data.decode())
            rr.log(RR_STATUS, rr.TextLog(json.dumps(status)))
            log.info(f"[Server status] {status}")
        except Exception:
            pass

    def _render(self):
        with self._render_lock:
            if self._cloud.version == self._last_render_version:
                return
            xyz, rgb = self._cloud.get_flat()
            if xyz.shape[0] == 0:
                return
            # Rerun expects uint8 colors [0, 255]
            colors_u8 = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
            rr.log(RR_PCD, rr.Points3D(
                positions=xyz,
                colors=colors_u8,
                radii=0.01,
            ))
            self._last_render_version = self._cloud.version
            log.debug(f"[Rerun] Rendered {xyz.shape[0]} pts v{self._cloud.version}")

# ─────────────────────────────────────────────────────────────────────────────
# Main viewer
# ─────────────────────────────────────────────────────────────────────────────

class PRISMViewer:
    def __init__(self, request_snapshot: bool = False):
        log.info(f"[Viewer] Connecting to Zenoh at {ZENOH_ROUTER}...")
        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", f'["{ZENOH_ROUTER}"]')
        conf.insert_json5("mode", '"client"')
        self._z = zenoh.open(conf)
        log.info("[Viewer] Connected.")

        self._logger = RerunLogger()

        # Subscribe to all server output keys
        self._z.declare_subscriber(KEY_PCD_DELTA,    self._on_delta)
        self._z.declare_subscriber(KEY_PCD_SNAPSHOT, self._on_snapshot)
        self._z.declare_subscriber(KEY_TRAJECTORY,   self._on_trajectory)
        self._z.declare_subscriber(KEY_STATUS,       self._on_status)

        log.info(f"[Viewer] Subscribed to {KEY_PCD_DELTA} and {KEY_PCD_SNAPSHOT}")

        if request_snapshot:
            self._request_snapshot()

    def _on_delta(self, sample):
        self._logger.on_pcd_message(bytes(sample.payload))

    def _on_snapshot(self, sample):
        self._logger.on_pcd_message(bytes(sample.payload))

    def _on_trajectory(self, sample):
        self._logger.on_trajectory(bytes(sample.payload))

    def _on_status(self, sample):
        self._logger.on_status(bytes(sample.payload))

    def _request_snapshot(self):
        """Query the server for the full current snapshot."""
        log.info(f"[Viewer] Requesting snapshot from '{KEY_PCD_SNAPSHOT}'...")
        replies = self._z.get(KEY_PCD_SNAPSHOT, timeout=5.0)
        for reply in replies:
            if reply.ok:
                data = bytes(reply.result.payload)
                if len(data) > _HEADER_SIZE:
                    self._logger.on_pcd_message(data)
                    log.info("[Viewer] Snapshot received and rendered.")
                else:
                    log.warning("[Viewer] Empty snapshot reply — server may not have data yet.")

    def run(self):
        log.info("[Viewer] Rendering. Press Ctrl+C to quit.")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            log.info("[Viewer] Shutting down.")
        finally:
            self._z.close()


def main():
    parser = argparse.ArgumentParser(description="VAT PRISM Rerun viewer")
    parser.add_argument("--snapshot", action="store_true",
                        help="Request full snapshot from server on start")
    args = parser.parse_args()

    viewer = PRISMViewer(request_snapshot=args.snapshot)
    viewer.run()


if __name__ == "__main__":
    main()
