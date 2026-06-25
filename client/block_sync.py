"""
VAT — Block Sync  (client side of the diff-based cloud sync)
============================================================
Keeps a local cube store in lock-step with the server's map using the manifest /
request / Draco-bundle protocol in :mod:`vat_blockmap`:

  1. subscribe to ``{server}/pcd/manifest`` — the server's current ``{key: crc}``;
  2. a background thread diffs it against the local store → the cube-keys we lack
     or whose CRC changed (and the keys to drop);
  3. it requests the missing cubes in ONE Zenoh query (keys as the query payload)
     to ``{server}/pcd/blocks`` and applies the Draco bundle reply, replacing those
     cubes wholesale;
  4. :meth:`take_merged` hands the viewer the whole cloud when it changed.

Runs entirely off the render thread; the viewer just polls ``take_merged()``.
"""

from __future__ import annotations

import logging
import threading
import time

import zenoh

import vat_protocol as proto
import vat_blockmap as bm

log = logging.getLogger("block-sync")


class BlockSync:
    def __init__(self, z: zenoh.Session, cube_m: float = bm.DEFAULT_CUBE_M,
                 server_prefix: str = "server/prism"):
        self._z = z
        self._store = bm.ClientBlockStore(cube_m)
        k = proto.keys(server_prefix=server_prefix)
        self._k_manifest = k["pcd_manifest"]
        self._k_blocks = k["pcd_blocks"]
        self._remote = {}
        self._lock = threading.Lock()
        self._evt = threading.Event()
        self._stop = False
        # telemetry for the HUD
        self.cubes = 0
        self.last_need = 0
        self.last_bundle_bytes = 0
        self.last_sync_ms = 0.0
        self.bytes_total = 0
        z.declare_subscriber(self._k_manifest, self._on_manifest)
        threading.Thread(target=self._sync_loop, daemon=True).start()
        log.info(f"[BlockSync] manifest←'{self._k_manifest}'  blocks?'{self._k_blocks}'  "
                 f"cube={cube_m}m")

    def _on_manifest(self, sample):
        try:
            man = bm.unpack_manifest(bytes(sample.payload))
        except Exception:
            return
        with self._lock:
            self._remote = man
        self._evt.set()

    def _sync_loop(self):
        while not self._stop:
            if not self._evt.wait(timeout=1.0):
                continue
            self._evt.clear()
            with self._lock:
                remote = dict(self._remote)
            need, drop = bm.diff_manifest(self._store.local_manifest(), remote)
            if not remote and self._store.blocks:        # empty manifest → server reset
                self._store.clear()
            if drop:
                self._store.drop(drop)
            if not need:
                self.cubes = len(self._store.blocks)
                continue
            try:
                t0 = time.time()
                req = bm.pack_request(need)
                applied = 0
                nbytes = 0
                for reply in self._z.get(self._k_blocks, payload=req, timeout=15.0):
                    if reply.ok:
                        buf = bytes(reply.result.payload)
                        nbytes += len(buf)
                        applied += self._store.apply_bundle_bytes(buf)
                self.last_need = len(need)
                self.last_bundle_bytes = nbytes
                self.bytes_total += nbytes
                self.last_sync_ms = (time.time() - t0) * 1000.0
                self.cubes = len(self._store.blocks)
                log.info(f"[BlockSync] synced {applied}/{len(need)} cubes  "
                         f"{nbytes/1024:.0f} KB  {self.last_sync_ms:.0f} ms")
            except Exception as e:
                log.warning(f"[BlockSync] block request failed: {e}")

    def take_merged(self):
        """→ (xyz f32, rgb u8) of the whole map if it changed since last call, else None."""
        return self._store.merged()

    def force_resync(self):
        """Drop local state so the next manifest triggers a full refetch ('1')."""
        self._store.clear()
        self._evt.set()
