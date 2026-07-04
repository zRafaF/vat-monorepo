"""
VAT — Snapshot Sync  (client side of the whole-map streaming path)
==================================================================
Drop-in replacement for :class:`block_sync.BlockSync` when the server streams in
``STREAM_MODE=snapshot``: subscribe ``{server}/pcd_snapshot`` and keep the LATEST
whole-map snapshot. Each snapshot REPLACES the local cloud (no accumulation, no
manifest/diff/pull), which is exactly what pairs with reset-each-batch mapping.

Exposes the same surface the viewer polls — ``take_delta()`` / ``take_merged()`` /
``force_resync()`` and the ``last_push_bytes`` / ``cubes`` telemetry — so the viewer
selects between block-sync and snapshot-sync with a single factory line.

``take_delta`` reports the whole snapshot as a single "block 0" with ``resync=True``,
so the viewer's :class:`IncrementalCloud` clears and re-uploads it wholesale (correct
for a full replace; the incremental slot machinery is a no-op benefit here).
"""

from __future__ import annotations

import logging
import threading
import time

import numpy as np
import zenoh

import vat_protocol as proto

log = logging.getLogger("snapshot-sync")


def _rgb_u8(rgb: np.ndarray) -> np.ndarray:
    """unpack_pcd returns rgb as float in [0,1]; the render buffer wants uint8."""
    rgb = np.asarray(rgb, dtype=np.float32).reshape(-1, 3)
    return np.clip(np.rint(rgb * 255.0), 0, 255).astype(np.uint8)


class SnapshotSync:
    def __init__(self, z: zenoh.Session, cube_m: float = 1.0,
                 server_prefix: str = "server/prism", request_snapshot: bool = True):
        self._z = z
        k = proto.keys(server_prefix=server_prefix)
        self._k_snapshot = k["pcd_snapshot"]
        self._k_delta = k["pcd_delta"]
        self._lock = threading.Lock()
        self._xyz = np.zeros((0, 3), np.float32)
        self._rgb = np.zeros((0, 3), np.uint8)
        self._version = -1
        self._dirty = False
        # telemetry parity with BlockSync (HUD reads these)
        self.cubes = 0
        self.last_need = 0
        self.last_bundle_bytes = 0
        self.last_sync_ms = 0.0
        self.bytes_total = 0
        self.last_push_cubes = 0
        self.last_push_bytes = 0
        self.pushes = 0
        z.declare_subscriber(self._k_snapshot, self._on_pcd)
        z.declare_subscriber(self._k_delta, self._on_pcd)   # server may reuse for coarse deltas
        log.info(f"[SnapshotSync] snapshot←'{self._k_snapshot}'")
        if request_snapshot:
            threading.Thread(target=self.force_resync, daemon=True).start()

    # ── receive ────────────────────────────────────────────────────────────────
    def _apply(self, buf: bytes):
        version, xyz, rgb, is_snap, _since = proto.unpack_pcd(buf)
        rgb = _rgb_u8(rgb)
        with self._lock:
            self._xyz = np.ascontiguousarray(xyz, np.float32)
            self._rgb = rgb
            self._version = int(version)
            self._dirty = True
            self.cubes = int(self._xyz.shape[0])
        self.last_push_bytes = len(buf)
        self.last_push_cubes = int(xyz.shape[0])
        self.bytes_total += len(buf)
        self.pushes += 1
        if xyz.shape[0] == 0:
            log.info(f"[SnapshotSync] empty snapshot v{version} → cleared")
        else:
            log.debug(f"[SnapshotSync] snapshot v{version}: {xyz.shape[0]} pts "
                      f"({len(buf)/1024:.0f} KB)")

    def _on_pcd(self, sample):
        try:
            self._apply(bytes(sample.payload))
        except Exception as e:
            log.debug(f"[SnapshotSync] bad snapshot: {e}")

    # ── render-thread polls ──────────────────────────────────────────────────
    def take_merged(self):
        """→ (xyz f32, rgb u8) of the whole map if it changed since last call, else None."""
        with self._lock:
            if not self._dirty:
                return None
            self._dirty = False
            return self._xyz.copy(), self._rgb.copy()

    def take_delta(self):
        """→ (changed {0:(xyz,rgb)}, removed set(), resync=True) when a new snapshot
        arrived, else None. resync=True makes the viewer replace its cloud wholesale."""
        with self._lock:
            if not self._dirty:
                return None
            self._dirty = False
            if self._xyz.shape[0] == 0:
                return {}, set(), True
            return {0: (self._xyz.copy(), self._rgb.copy())}, set(), True

    def force_resync(self):
        """Pull the current whole-map snapshot on demand (bootstrap / '1' keypress)."""
        try:
            t0 = time.time()
            for reply in self._z.get(self._k_snapshot, timeout=5.0):
                if reply.ok:
                    data = bytes(reply.result.payload)
                    if len(data) >= 4:
                        self._apply(data)
            self.last_sync_ms = (time.time() - t0) * 1000.0
        except Exception as e:
            log.warning(f"[SnapshotSync] snapshot request failed: {e}")
