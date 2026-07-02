"""
VAT mapping server — navigation ESDF (world-frame collision field).
===================================================================

The nvblox ESDF (Euclidean Signed Distance Field) gives, at any point, the signed
distance to the nearest obstacle — the field a local planner queries for collision
costs. This module exposes it to the nav/client in the persistent WORLD frame.

Frame handling (the reset-mode subtlety): in reset-each-batch mode the engine
integrates in the FRESH reconstruction's LOCAL frame, so the nvblox volume — and
therefore ``engine.get_esdf_slice`` / ``tsdf.query_esdf`` — is in that local frame.
The streamed cloud is transformed into the persistent world frame by the rigid
anchor ``T`` (local→world). We apply the SAME ``T`` here so the planner and the
viewer share ONE frame:

* :meth:`publish` — sample a horizontal ESDF slice, transform the sample points by
  ``T``, and publish them as a colored point cloud (distance→color) for viz/nav.
* :meth:`query_world` — map world query points back through ``inv(T)`` into the
  nvblox local frame, query the ESDF, and return signed distances (meters) to the
  planner.

In the (deprecated) online mode ``T`` is identity, so both are pass-throughs.
"""

from __future__ import annotations

import logging

import numpy as np
import zenoh

import mapping_config as cfg
import vat_protocol as proto

log = logging.getLogger("mapping-server")

# Distance (m) at/above which a cell is considered "clear" for the viz color ramp.
_CLEAR_M = 1.0


def _distance_to_rgb(dist_m: np.ndarray) -> np.ndarray:
    """Map signed distance (m) → uint8 RGB: red at/inside an obstacle (≤0),
    through yellow, to green when ≥ _CLEAR_M. For visualization only."""
    t = np.clip(np.asarray(dist_m, np.float32) / _CLEAR_M, 0.0, 1.0)
    r = np.clip((1.0 - t) * 2.0, 0.0, 1.0)
    g = np.clip(t * 2.0, 0.0, 1.0)
    rgb = np.stack([r, g, np.zeros_like(t)], axis=1)
    return np.clip(np.rint(rgb * 255.0), 0, 255).astype(np.uint8)


class NavEsdfPublisher:
    """Publishes a world-frame ESDF slice each submap and answers world-frame queries."""

    def __init__(self, z: zenoh.Session):
        self._z = z
        # DATA_LOW: bulk data that yields to the realtime pose stream but still delivers
        # (BACKGROUND could be starved under steady pose traffic).
        self._pub = z.declare_publisher(
            cfg.NAV_ESDF_KEY, congestion_control=zenoh.CongestionControl.DROP,
            priority=zenoh.Priority.DATA_LOW)
        self._every = max(1, int(cfg.NAV_ESDF_EVERY_N))
        self._height = None
        if cfg.NAV_ESDF_HEIGHT_M not in ("", None):
            try:
                self._height = float(cfg.NAV_ESDF_HEIGHT_M)
            except ValueError:
                self._height = None
        self.last_bytes = 0
        self.last_cells = 0
        log.info(f"[NavESDF] slice→'{cfg.NAV_ESDF_KEY}'  res={cfg.NAV_ESDF_RES_M}m  "
                 f"every={self._every}  height={'median' if self._height is None else self._height}")

    def publish(self, engine, world_anchor, submap_index: int = 0) -> None:
        """Sample the ESDF slice, transform to world, and publish it as a colored cloud.
        No-op if ESDF is off, the slice is empty, or it's not this submap's turn."""
        if not engine.compute_esdf or (submap_index % self._every) != 0:
            return
        try:
            sl = engine.get_esdf_slice(height=self._height, resolution=cfg.NAV_ESDF_RES_M)
        except Exception as e:
            log.debug(f"[NavESDF] slice failed: {e}")
            return
        if not sl:
            return
        xs, ys, dist = sl["xs"], sl["ys"], sl["distance"]      # dist (Hy, Wx), local frame
        gx, gy = np.meshgrid(xs, ys)
        gz = np.full_like(gx, float(sl["z"]))
        pts_local = np.stack([gx, gy, gz], axis=-1).reshape(-1, 3)
        d = np.asarray(dist, np.float32).reshape(-1)
        finite = np.isfinite(d)                                 # drop unobserved cells
        if not finite.any():
            return
        pts_local, d = pts_local[finite], d[finite]

        T = np.asarray(world_anchor if world_anchor is not None else np.eye(4), np.float64)
        pts_world = (pts_local @ T[:3, :3].T + T[:3, 3]).astype(np.float32)
        rgb = _distance_to_rgb(d)
        try:
            buf = proto.pack_pcd(int(submap_index), pts_world, rgb, is_snapshot=True)
            self._pub.put(buf)
            self.last_bytes = len(buf)
            self.last_cells = int(pts_world.shape[0])
        except Exception as e:
            log.debug(f"[NavESDF] publish failed: {e}")

    @staticmethod
    def query_world(engine, world_anchor, points_world: np.ndarray) -> np.ndarray:
        """Signed ESDF distance (m) at Nx3 WORLD points, for a planner. Maps world→local
        via inv(world_anchor), queries the nvblox ESDF, returns (N,) meters (NaN where
        unobserved). Requires torch/CUDA (delegates to engine.tsdf.query_esdf)."""
        import torch
        pts_world = np.asarray(points_world, np.float64).reshape(-1, 3)
        T = np.asarray(world_anchor if world_anchor is not None else np.eye(4), np.float64)
        Tinv = np.linalg.inv(T)
        pts_local = (pts_world @ Tinv[:3, :3].T + Tinv[:3, 3]).astype(np.float32)
        q = torch.from_numpy(pts_local).to(engine.device)
        return engine.tsdf.query_esdf(q).cpu().numpy()
