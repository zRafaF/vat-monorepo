"""
VAT mapping server — persistent world anchor for reset mode.
============================================================

In reset-each-batch mode every batch produces a FRESH reconstruction in its OWN
local frame. To keep the streamed cloud + robot from rotating/jumping between
batches (and to collapse the client-side delta to just the frontier), each fresh
local frame is rigidly re-anchored into ONE persistent world frame, estimated
from the camera poses shared (by capture timestamp) with the previous batch.

This is a rigid SE3 (scale = 1) fit: the map is already metric (scale is locked
inside the engine), so only rotation + translation are solved here. It is NOT a
loop closure — it cannot correct large open-loop drift on a revisit — it only
keeps consecutive fresh batches expressed in a common frame.
"""

from __future__ import annotations

import logging

import numpy as np

log = logging.getLogger("mapping-server")


def orthonormalize(R: np.ndarray) -> np.ndarray:
    """Nearest rotation matrix to R (SVD; fixes any reflection)."""
    U, _, Vt = np.linalg.svd(R)
    Rn = U @ Vt
    if np.linalg.det(Rn) < 0:
        U[:, -1] *= -1.0
        Rn = U @ Vt
    return Rn


def rigid_anchor_from_poses(common) -> np.ndarray:
    """Best-fit RIGID transform (SE3, scale=1) mapping a fresh reconstruction's LOCAL
    frame to the persistent WORLD frame, from corresponding camera poses.

    ``common`` = list of ``(P_local 4x4, P_world 4x4)`` for frames shared with the
    previous batch. Each pair implies a candidate ``T = P_world @ inv(P_local)``; for a
    consistent rigid scene they agree, so we average translation and orthonormalise the
    mean rotation. Using full poses (not just positions) stays well-conditioned even
    when the camera barely moved (a near-static robot)."""
    Rs, ts = [], []
    for P_local, P_world in common:
        T = np.asarray(P_world, np.float64) @ np.linalg.inv(np.asarray(P_local, np.float64))
        Rs.append(T[:3, :3])
        ts.append(T[:3, 3])
    out = np.eye(4)
    out[:3, :3] = orthonormalize(np.mean(np.stack(Rs), axis=0))
    out[:3, 3] = np.mean(np.stack(ts), axis=0)
    return out


def _key(ts) -> int:
    """Capture-time → integer ns key (poses are matched across batches by capture time)."""
    return int(round(float(ts) * 1e9))


class WorldAnchor:
    """Holds the persistent world frame and re-anchors each fresh reset batch into it."""

    def __init__(self, enabled: bool) -> None:
        self.enabled = bool(enabled)
        self._prev_world_poses: dict[int, np.ndarray] = {}   # ts_ns -> 4x4 world pose
        self._anchor = np.eye(4)                             # last good local->world SE3

    def reset(self) -> None:
        self._prev_world_poses = {}
        self._anchor = np.eye(4)

    def compute(self, ts_arr, poses) -> np.ndarray:
        """SE3 mapping the current fresh-reconstruction local frame to the persistent
        world frame, from frames shared (by capture timestamp) with the previous batch.
        First batch / anchoring off → identity (defines the world). Too little overlap →
        reuse the last anchor to avoid a hard jump."""
        if (not self.enabled or not self._prev_world_poses
                or ts_arr is None or poses is None or len(ts_arr) != len(poses)):
            return np.eye(4)
        common = []
        for ti, P in zip(ts_arr, poses):
            w = self._prev_world_poses.get(_key(ti))
            if w is not None:
                common.append((np.asarray(P, np.float64), np.asarray(w, np.float64)))
        if len(common) < 3:
            log.warning(f"[PRISM] reset anchor: only {len(common)} overlapping frames — "
                        f"reusing last anchor (world may shift)")
            return self._anchor.copy()
        self._anchor = rigid_anchor_from_poses(common)
        return self._anchor.copy()

    def remember(self, ts_arr, poses, T: np.ndarray) -> None:
        """Store this batch's WORLD poses (keyed by capture-time ns) for the next anchor."""
        if ts_arr is None or poses is None or len(ts_arr) != len(poses):
            return
        self._prev_world_poses = {_key(ti): (T @ np.asarray(P, np.float64))
                                  for ti, P in zip(ts_arr, poses)}
