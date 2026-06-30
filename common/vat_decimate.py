"""
VAT — Stream decimation (server side)
=====================================
Reduce the POINT COUNT of the streamed cloud, decoupled from the TSDF voxel, so the
mapper can run fine while the wire + the client RENDER stay cheap. Selectable mode
(env ``STREAM_DECIMATE_MODE``), because the right trade-off depends on what hurts:

  * ``none``           — no decimation (stream the full surface).
  * ``voxel_centroid`` — DEFAULT. One point per ``voxel`` cell at the CENTROID of the
                         cell's points (keeps sub-cell placement → crisp edges from a
                         finer TSDF). Deterministic ⇒ stable input → stable output ⇒
                         the occupancy-CRC deltas stay small. Best quality/bandwidth.
  * ``voxel_center``   — One point per ``voxel`` cell SNAPPED to the cell centre. Also
                         deterministic (stable CRC), a touch cheaper, but loses sub-cell
                         placement (looks blockier).
  * ``stride``         — Keep every Nth point (``STREAM_STRIDE``). Cheapest, but NOT
                         deterministic across submaps (marching-cubes vertex order is
                         unstable) → the CRC churns → bandwidth doesn't shrink even
                         though the render does. Use only to probe render cost.

NumPy-only; unit-tested via ``python common/vat_decimate.py``.
"""

from __future__ import annotations

import numpy as np


def _as_out(xyz, rgb):
    rgb = np.asarray(rgb)
    return (np.ascontiguousarray(xyz, np.float32).reshape(-1, 3),
            rgb.astype(np.float32) if rgb.dtype.kind == "f" else rgb.astype(np.uint8))


def _stride(xyz, rgb, stride):
    idx = np.arange(0, xyz.shape[0], max(int(stride), 1))
    return _as_out(np.asarray(xyz)[idx], np.asarray(rgb)[idx])


def _voxel(xyz, rgb, voxel, centroid):
    xyz = np.ascontiguousarray(xyz, np.float64).reshape(-1, 3)
    rgb = np.asarray(rgb).reshape(-1, 3)
    keys = np.floor(xyz / float(voxel)).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    m = uniq.shape[0]
    counts = np.bincount(inv, minlength=m).astype(np.float64)
    if centroid:
        csum = np.zeros((m, 3), np.float64); np.add.at(csum, inv, xyz)
        pts = (csum / counts[:, None]).astype(np.float32)
    else:
        pts = ((uniq.astype(np.float64) + 0.5) * float(voxel)).astype(np.float32)
    rsum = np.zeros((m, 3), np.float64); np.add.at(rsum, inv, rgb.astype(np.float64))
    rmean = rsum / counts[:, None]
    cols = (rmean.astype(np.float32) if rgb.dtype.kind == "f"
            else np.clip(np.rint(rmean), 0, 255).astype(np.uint8))
    return pts, cols


def decimate(xyz, rgb, mode="voxel_centroid", voxel=0.04, stride=3):
    """Decimate ``(xyz, rgb)`` by ``mode``. Returns (xyz f32, rgb same-kind)."""
    mode = (mode or "none").lower()
    xyz = np.asarray(xyz).reshape(-1, 3)
    if xyz.shape[0] == 0 or mode in ("none", "off"):
        return _as_out(xyz, rgb)
    if mode == "stride":
        return _stride(xyz, rgb, stride)
    if not voxel or voxel <= 0:
        return _as_out(xyz, rgb)
    return _voxel(xyz, rgb, voxel, centroid=(mode != "voxel_center"))


# =============================================================================
def _selftest() -> None:
    rng = np.random.default_rng(0)
    dense = (rng.random((6000, 3)) * 0.5).astype(np.float32)
    col = (rng.random((6000, 3)) * 255).astype(np.uint8)

    # none = passthrough
    x, c = decimate(dense, col, mode="none")
    assert x.shape[0] == 6000

    # voxel_centroid: fewer points, deterministic, centroid stays in its cell
    a, ac = decimate(dense, col, mode="voxel_centroid", voxel=0.04)
    b, bc = decimate(dense, col, mode="voxel_centroid", voxel=0.04)
    assert a.shape[0] < 6000 and a.shape == ac.shape
    assert np.array_equal(a, b) and np.array_equal(ac, bc), "centroid must be deterministic"
    assert np.all(np.abs(a - (np.floor(a / 0.04) + 0.5) * 0.04) <= 0.04 + 1e-4)

    # voxel_center: deterministic, snapped exactly to cell centres
    d, _ = decimate(dense, col, mode="voxel_center", voxel=0.04)
    assert np.allclose(d, (np.floor(d / 0.04) + 0.5) * 0.04, atol=1e-4)
    assert d.shape[0] == a.shape[0]      # same cells, different placement

    # stride: cheap, ~1/stride of the points
    s, sc = decimate(dense, col, mode="stride", stride=3)
    assert abs(s.shape[0] - 2000) <= 1 and s.shape == sc.shape
    print(f"vat_decimate self-test OK  (centroid={a.shape[0]} center={d.shape[0]} "
          f"stride={s.shape[0]} of 6000)")


if __name__ == "__main__":
    _selftest()
