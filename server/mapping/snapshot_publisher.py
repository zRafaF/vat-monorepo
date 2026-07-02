"""
VAT — Snapshot Publisher  (server side of the whole-map streaming path)
=======================================================================
The simple, low-latency alternative to the diff-based block sync: after every
submap, publish the WHOLE current TSDF surface (already coarse-voxelised to
``STREAM_VOXEL_M`` by the caller) as ONE compressed :func:`vat_protocol.pack_pcd`
snapshot on ``{server}/pcd_snapshot``. The client REPLACES its cloud with each
snapshot, so nothing ever accumulates and there is no manifest / diff / pull
round-trip.

This pairs with reset-each-batch mapping: the map is already rebuilt whole and
kept bounded every batch, so "just send the whole map" is both the simplest and
the lowest-latency transport. Keep the internal map fine (``VOXEL_SIZE``) and
stream coarse (``STREAM_VOXEL_M``) to fit the link.

Drop-in for :class:`block_publisher.BlockPublisher`: same ``ingest_and_publish``
signature + return tuple, same ``reset()``, so ``mapping_server`` can pick either
by ``STREAM_MODE``.
"""

from __future__ import annotations

import logging
import os
import traceback

import numpy as np
import zenoh

import vat_protocol as proto

log = logging.getLogger("snapshot-pub")

# Hard cap on points per streamed snapshot (0 = no cap). Above it we uniformly
# subsample so a single snapshot can't balloon the link. The full-res cloud is
# still available via the pcd_snapshot queryable.
SNAPSHOT_MAX_POINTS = int(os.environ.get("CLOUD_STREAM_MAX_POINTS", "60000"))


class SnapshotPublisher:
    def __init__(self, z: zenoh.Session, server_prefix: str = "server/prism",
                 max_points: int = SNAPSHOT_MAX_POINTS):
        self._z = z
        self._max_points = int(max_points)
        self._rng = np.random.default_rng(0)
        k = proto.keys(server_prefix=server_prefix)
        self._k_snapshot = k["pcd_snapshot"]
        # DROP: a snapshot replaces the whole cloud, so shedding one on a momentarily
        # slow link is harmless — the next submap's snapshot repairs it. This keeps the
        # geometry stream from head-of-line-blocking the session.
        self._pub = z.declare_publisher(
            self._k_snapshot, congestion_control=zenoh.CongestionControl.DROP)
        self.last_snapshot_bytes = 0
        self.last_points = 0
        log.info(f"[SnapshotPub] snapshot→'{self._k_snapshot}'  "
                 f"max_points={self._max_points or 'uncapped'}")

    def _cap(self, xyz, rgb):
        n = xyz.shape[0]
        if self._max_points <= 0 or n <= self._max_points:
            return xyz, rgb
        idx = self._rng.choice(n, size=self._max_points, replace=False)
        return xyz[idx], rgb[idx]

    def ingest_and_publish(self, xyz, rgb, map_version: int = 0, observed_centers=None):
        """Publish the whole cloud as one snapshot. ``observed_centers`` is ignored
        (there is no observation-TTL in snapshot mode — the whole map is resent, so
        anything not in the current surface is simply gone).

        Returns ``(n_changed, n_removed, n_cubes, manifest_bytes, snapshot_bytes)`` to
        match :class:`block_publisher.BlockPublisher` — here n_changed = n_cubes = the
        streamed point count, n_removed = 0, manifest_bytes = 0."""
        xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
        rgb = np.asarray(rgb).reshape(-1, 3)
        xyz, rgb = self._cap(xyz, rgb)
        n = int(xyz.shape[0])
        try:
            buf = proto.pack_pcd(int(map_version), xyz, rgb, is_snapshot=True)
            self._pub.put(buf)
            self.last_snapshot_bytes = len(buf)
        except Exception:
            log.error(f"[SnapshotPub] publish failed:\n{traceback.format_exc()}")
            self.last_snapshot_bytes = 0
        self.last_points = n
        return n, 0, n, 0, self.last_snapshot_bytes

    def reset(self):
        """Tell the client to clear: an empty snapshot is the reset signal the viewer
        already understands (LocalCloud.apply_snapshot → clear on 0 points)."""
        try:
            empty = np.zeros((0, 3), dtype=np.float32)
            self._pub.put(proto.pack_pcd(0, empty, empty, is_snapshot=True))
        except Exception:
            pass
