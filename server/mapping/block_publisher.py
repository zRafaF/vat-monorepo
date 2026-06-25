"""
VAT — Block Publisher  (server side of the diff-based cloud sync)
=================================================================
Wraps :class:`vat_blockmap.BlockGrid` with Zenoh:

  * after every PRISM submap, ``ingest_and_publish(xyz, rgb)`` rebuilds the
    canonical cube grid and publishes the tiny **manifest** (one ``(key, crc)``
    per occupied cube) on ``{server}/pcd/manifest``;
  * a **queryable** on ``{server}/pcd/blocks`` answers a client's request (the
    requested cube-keys arrive as the query payload) with ONE Draco-compressed
    **bundle** of just those cubes.

The heavy lifting (keying, CRC versioning, Draco) lives in ``vat_blockmap`` so this
file is only the transport glue.
"""

from __future__ import annotations

import logging
import threading
import traceback

import zenoh

import vat_protocol as proto
import vat_blockmap as bm

log = logging.getLogger("block-pub")


class BlockPublisher:
    def __init__(self, z: zenoh.Session, cube_m: float = bm.DEFAULT_CUBE_M,
                 server_prefix: str = "server/prism",
                 quant_bits: int = bm.DRACO_QUANT_BITS, level: int = bm.DRACO_LEVEL):
        self._z = z
        self._grid = bm.BlockGrid(cube_m)
        self._lock = threading.Lock()
        self._quant_bits = quant_bits
        self._level = level
        k = proto.keys(server_prefix=server_prefix)
        self._k_manifest = k["pcd_manifest"]
        self._k_blocks = k["pcd_blocks"]
        self._pub = z.declare_publisher(
            self._k_manifest, congestion_control=zenoh.CongestionControl.DROP)
        self._qbl = z.declare_queryable(self._k_blocks, self._on_request)
        self.last_manifest_bytes = 0
        log.info(f"[BlockPub] manifest→'{self._k_manifest}'  blocks?'{self._k_blocks}'  "
                 f"cube={cube_m}m  draco q{quant_bits}/L{level}")

    def ingest_and_publish(self, xyz, rgb):
        """Rebuild the grid from the full cloud and publish the manifest.
        Returns (n_changed, n_removed, n_cubes, manifest_bytes)."""
        with self._lock:
            changed, removed = self._grid.ingest(xyz, rgb)
            man = self._grid.manifest()
        buf = bm.pack_manifest(man)
        try:
            self._pub.put(buf)
        except Exception:
            log.error(f"[BlockPub] manifest publish failed:\n{traceback.format_exc()}")
        self.last_manifest_bytes = len(buf)
        return len(changed), len(removed), len(man), len(buf)

    def reset(self):
        with self._lock:
            self._grid = bm.BlockGrid(self._grid.cube_m)
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
