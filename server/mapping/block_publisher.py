"""
VAT — Block Publisher  (server side of the diff-based cloud sync)
=================================================================
Wraps :class:`vat_blockmap.BlockGrid` with Zenoh:

  * after every PRISM submap, ``ingest_and_publish(xyz, rgb, version)`` rebuilds the
    canonical cube grid and:
      - publishes the tiny **manifest** (one ``(key, crc)`` per occupied cube) on
        ``{server}/pcd/manifest`` — the bootstrap + repair channel; and
      - PUSHES the cubes that actually changed (+ the keys that were removed) on
        ``{server}/pcd/push`` as ONE Draco bundle — the low-latency steady-state
        path (no client request / round-trip). Huge changesets (first map / global
        re-anchor) skip the push and let the client pull via the manifest diff.
  * a **queryable** on ``{server}/pcd/blocks`` still answers a client's pull
    request (requested cube-keys arrive as the query payload) with a Draco bundle —
    used for bootstrap and to repair any dropped push.

The CRC is computed on a coarse occupancy grid (``crc_quant_m`` ~ ½ voxel) so
sub-voxel nvblox mesh "breathing" doesn't flip cube versions (which used to make
the diff resend ~everything every submap); see ``vat_blockmap`` for the rationale.

The heavy lifting (keying, CRC versioning, Draco) lives in ``vat_blockmap`` so this
file is only the transport glue.
"""

from __future__ import annotations

import logging
import os
import threading
import traceback

import zenoh

import vat_protocol as proto
import vat_blockmap as bm

log = logging.getLogger("block-pub")

# Cap on cubes pushed in one frame. Above this (first map, big re-anchor) we skip
# the push and rely on the manifest → pull path so a single push can't balloon.
PUSH_MAX_CUBES = int(os.environ.get("PUSH_MAX_CUBES", "400"))


class BlockPublisher:
    def __init__(self, z: zenoh.Session, cube_m: float = bm.DEFAULT_CUBE_M,
                 server_prefix: str = "server/prism",
                 quant_bits: int = bm.DRACO_QUANT_BITS, level: int = bm.DRACO_LEVEL,
                 crc_quant_m: float | None = bm.DEFAULT_CRC_QUANT_M):
        self._z = z
        self._grid = bm.BlockGrid(cube_m, crc_quant_m=crc_quant_m)
        self._lock = threading.Lock()
        self._quant_bits = quant_bits
        self._level = level
        k = proto.keys(server_prefix=server_prefix)
        self._k_manifest = k["pcd_manifest"]
        self._k_blocks = k["pcd_blocks"]
        self._k_push = k["pcd_push"]
        self._pub = z.declare_publisher(
            self._k_manifest, congestion_control=zenoh.CongestionControl.DROP)
        # Push rides its own publisher; DROP so a momentarily slow link sheds the
        # delta (the next manifest diff repairs it) instead of head-of-line-blocking.
        self._pub_push = z.declare_publisher(
            self._k_push, congestion_control=zenoh.CongestionControl.DROP)
        self._qbl = z.declare_queryable(self._k_blocks, self._on_request)
        self.last_manifest_bytes = 0
        self.last_push_bytes = 0
        log.info(f"[BlockPub] manifest→'{self._k_manifest}'  push→'{self._k_push}'  "
                 f"blocks?'{self._k_blocks}'  cube={cube_m}m  "
                 f"crc_quant={'%.3fm' % crc_quant_m if crc_quant_m else 'legacy 1mm'}  "
                 f"draco q{quant_bits}/L{level}  push_cap={PUSH_MAX_CUBES}")

    def ingest_and_publish(self, xyz, rgb, map_version: int = 0):
        """Rebuild the grid from the full cloud, publish the manifest, and push the
        changed+removed cubes (unless the changeset is too large).
        Returns (n_changed, n_removed, n_cubes, manifest_bytes, push_bytes)."""
        with self._lock:
            changed, removed = self._grid.ingest(xyz, rgb)
            man = self._grid.manifest()
            # Collect the changed cubes' geometry while we hold the lock + the grid
            # state that produced this manifest (so manifest and push never disagree).
            do_push = 0 < len(changed) <= PUSH_MAX_CUBES
            push_blocks = self._grid.collect(changed) if do_push else None
        buf = bm.pack_manifest(man)
        try:
            self._pub.put(buf)
        except Exception:
            log.error(f"[BlockPub] manifest publish failed:\n{traceback.format_exc()}")
        self.last_manifest_bytes = len(buf)

        self.last_push_bytes = 0
        # Push when there's a bounded change OR only removals (cube_emptied cleanup).
        if do_push or (not changed and removed):
            try:
                pbuf = bm.pack_block_push(
                    push_blocks or [], removed, map_version=map_version,
                    cube_m=self._grid.cube_m, quant_bits=self._quant_bits, level=self._level)
                self._pub_push.put(pbuf)
                self.last_push_bytes = len(pbuf)
            except Exception:
                log.error(f"[BlockPub] push publish failed:\n{traceback.format_exc()}")
        elif len(changed) > PUSH_MAX_CUBES:
            log.info(f"[BlockPub] {len(changed)} cubes changed > cap {PUSH_MAX_CUBES}: "
                     f"manifest-only (client pulls)")
        return len(changed), len(removed), len(man), len(buf), self.last_push_bytes

    def reset(self):
        with self._lock:
            self._grid = bm.BlockGrid(self._grid.cube_m, crc_quant_m=self._grid.crc_quant_m)
        try:
            self._pub.put(bm.pack_manifest({}))      # empty manifest → client clears
        except Exception:
            pass

    def _on_request(self, query):
        try:
            payload = bytes(query.payload) if query.payload is not None else b""
            keys = bm.unpack_request(payload) if payload else []
            with self._lock:
                blocks = self._grid.collect(keys)
            bundle = bm.pack_bundle(blocks, self._grid.cube_m, self._quant_bits, self._level)
            query.reply(query.key_expr, bundle)
            log.debug(f"[BlockPub] served {len(blocks)}/{len(keys)} cubes "
                      f"({len(bundle)/1024:.0f} KB)")
        except Exception:
            log.error(f"[BlockPub] request handler failed:\n{traceback.format_exc()}")
