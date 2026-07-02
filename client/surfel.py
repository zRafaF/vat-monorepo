"""
VAT — Surfel builder  (client render side)
==========================================
Turn a voxel-snapped point cloud into surface-**normal-oriented**, UNLIT **square**
quads (two triangles each) that tile into a continuous surface — the "looks like a
mesh" legibility win, but as cheap oriented splats rather than a streamed mesh.

Design constraints (from the rig): the point-cloud uplink bandwidth is very tight, so
we transmit NO normals — they're derived on the client. It must be fast, so there is no
kNN tree: the stream is already voxel-snapped, so each point's local surface is fit from
its 3×3×3 GRID neighbours (found by a single vectorised ``searchsorted`` membership test)
via a batched 3×3 PCA. The smallest-variance eigenvector is the surface normal.

Two things make this both correct and cheap:
  * PCA (plane fit) recovers the normal of a THIN shell (marching-cubes surfaces are one
    voxel thick) where an occupancy-gradient normal would cancel to zero on both open
    sides; and
  * we don't care about lighting and a square is two-sided, so the normal SIGN is
    irrelevant — only the plane orientation matters, so no viewpoint/sign disambiguation
    is needed.

Pure NumPy, unit-tested headless:  python client/surfel.py
"""

from __future__ import annotations

import threading

import numpy as np


def estimate_normals(xyz: np.ndarray, voxel_m: float) -> np.ndarray:
    """Fast per-point surface normals for a voxel-snapped cloud.

    3×3×3 grid-neighbour PCA (batched); the smallest-variance eigenvector is the normal.
    Sign is arbitrary (fine for unlit two-sided squares). Points with fewer than 3
    neighbours (isolated) fall back to +Z. Returns unit normals ``(N,3) float32``."""
    xyz = np.asarray(xyz, dtype=np.float64)
    n = xyz.shape[0]
    if n == 0:
        return np.zeros((0, 3), np.float32)
    v = float(voxel_m) if voxel_m and voxel_m > 0 else 0.08

    q = np.round(xyz / v).astype(np.int64)          # grid coordinates
    mn = q.min(axis=0)
    d = q - mn                                       # non-negative grid coords
    span = (d.max(axis=0) + 4).astype(np.int64)      # +pad so neighbours stay in range

    def _enc(a):                                     # (…,3) grid coords → (…,) int64 key
        return (a[..., 0] * span[1] + a[..., 1]) * span[2] + a[..., 2]

    key = _enc(d)
    order = np.argsort(key)
    ks = key[order]                                  # sorted occupancy keys

    # 3×3×3 neighbour offsets (exclude the centre)
    rng = (-1, 0, 1)
    offs = np.array([(i, j, l) for i in rng for j in rng for l in rng
                     if not (i == 0 and j == 0 and l == 0)], np.int64)   # (26,3)

    nb_keys = _enc(d[None, :, :] + offs[:, None, :])         # (26,N)
    idx = np.clip(np.searchsorted(ks, nb_keys), 0, ks.shape[0] - 1)
    present = (ks[idx] == nb_keys).astype(np.float64).T      # (N,26) occupied?

    # Weighted covariance of the present neighbour offsets, WITHOUT the (26,N,3)
    # intermediate: since each offset's position ``rel_m`` is the same for every point,
    # Σ present·(rel-mean)(rel-mean)ᵀ = Σ present·(rel relᵀ) − (Σ present·rel)²/cnt.
    # The two accumulations are matmuls → BLAS (einsum here did not route to BLAS and was
    # ~5× slower). ``eigh`` on the (N,3,3) stack is then the only remaining hot spot.
    rel = (offs.astype(np.float64) * v)                      # (26,3) neighbour offsets (m)
    cnt = present.sum(axis=1)                                # (N,)
    Rm = (rel[:, :, None] * rel[:, None, :]).reshape(offs.shape[0], 9)   # (26,9) rel relᵀ
    Sxx = (present @ Rm).reshape(-1, 3, 3)                   # (N,3,3)  BLAS
    Sx = present @ rel                                       # (N,3)    BLAS
    cov = Sxx - (Sx[:, :, None] * Sx[:, None, :]) / np.maximum(cnt, 1.0)[:, None, None]

    _w, vecs = np.linalg.eigh(cov)                           # ascending eigenvalues
    nrm = vecs[:, :, 0]                                      # smallest-variance dir = normal
    nrm[cnt < 3] = (0.0, 0.0, 1.0)                           # too few neighbours → fallback
    ln = np.linalg.norm(nrm, axis=1)
    ln[ln < 1e-12] = 1.0
    return (nrm / ln[:, None]).astype(np.float32)


def build_quads(xyz: np.ndarray, rgb: np.ndarray, normals: np.ndarray,
                side: float):
    """Oriented square quads (2 triangles) centred at each point, in the plane ⊥ its
    normal, edge length ``side`` (m). Returns ``(vertices (4N,3) f32, faces (2N,3) i32,
    colors (4N,C))`` ready for a VisPy Mesh (unlit → pass colors as vertex_colors)."""
    xyz = np.asarray(xyz, dtype=np.float64)
    N = xyz.shape[0]
    C = rgb.shape[1] if (rgb.ndim == 2 and rgb.shape[1] in (3, 4)) else 3
    if N == 0:
        return (np.zeros((0, 3), np.float32), np.zeros((0, 3), np.int32),
                np.zeros((0, C), np.float32))
    nrm = np.asarray(normals, dtype=np.float64).reshape(N, 3)
    # Two in-plane basis vectors ⊥ the normal. Choose a reference axis that isn't
    # parallel to the normal, then Gram-Schmidt via cross products.
    ref = np.zeros((N, 3)); ref[:, 2] = 1.0
    ref[np.abs(nrm[:, 2]) > 0.9] = (1.0, 0.0, 0.0)
    t1 = np.cross(nrm, ref)
    t1 /= (np.linalg.norm(t1, axis=1, keepdims=True) + 1e-12)
    t2 = np.cross(nrm, t1)                                   # already unit (nrm,t1 ortho)
    h = 0.5 * float(side)
    verts = np.empty((N * 4, 3), np.float64)
    verts[0::4] = xyz + (-t1 - t2) * h
    verts[1::4] = xyz + (t1 - t2) * h
    verts[2::4] = xyz + (t1 + t2) * h
    verts[3::4] = xyz + (-t1 + t2) * h
    base = np.arange(N, dtype=np.int32) * 4
    faces = np.empty((N * 2, 3), np.int32)
    faces[0::2] = np.stack([base, base + 1, base + 2], axis=1)
    faces[1::2] = np.stack([base, base + 2, base + 3], axis=1)
    colors = np.repeat(np.asarray(rgb), 4, axis=0)
    return verts.astype(np.float32), faces, colors


def build_surfels(xyz: np.ndarray, rgb: np.ndarray, voxel_m: float,
                  side: float | None = None):
    """Convenience: normals + quads in one call. ``side`` defaults to ``voxel_m`` so the
    squares tile edge-to-edge. Returns ``(vertices, faces, colors)``."""
    if side is None:
        side = voxel_m
    nrm = estimate_normals(xyz, voxel_m)
    return build_quads(xyz, rgb, nrm, side)


class SurfelWorker:
    """Build surfel MESH ARRAYS off the render thread.

    The PCA-normal + quad build is tens of ms for a room-scale cloud — too much for the
    single GL/render thread (it would hitch the view every submap). This runs it in a
    daemon thread instead (NumPy's matmul/eigh/searchsorted release the GIL, so the
    render + pose callbacks keep flowing) and hands the finished ``(verts, faces,
    colors)`` back; the caller uploads them to its VisPy Mesh on the render thread.

    Latest-only: a newer :meth:`submit` supersedes an unprocessed one, so a burst of
    submaps never backs up a queue — the worker always builds the freshest cloud."""

    def __init__(self, voxel_m: float, side: float | None = None):
        self.voxel_m = float(voxel_m)
        self.side = side
        self._lock = threading.Lock()
        self._evt = threading.Event()
        self._pending = None        # (xyz, rgb) awaiting build
        self._ready = None          # (verts, faces, colors) built, awaiting upload
        self._stop = False
        self._t = threading.Thread(target=self._run, name="surfel-worker", daemon=True)
        self._t.start()

    def submit(self, xyz: np.ndarray, rgb: np.ndarray) -> None:
        with self._lock:
            self._pending = (xyz, rgb)
        self._evt.set()

    def take_ready(self):
        """→ (verts, faces, colors) if a fresh build is waiting, else None. Consumes it."""
        with self._lock:
            out, self._ready = self._ready, None
        return out

    def stop(self) -> None:
        self._stop = True
        self._evt.set()

    def _run(self) -> None:
        while not self._stop:
            self._evt.wait()
            self._evt.clear()
            with self._lock:
                job, self._pending = self._pending, None
            if job is None or self._stop:
                continue
            try:
                built = build_surfels(job[0], job[1], self.voxel_m, self.side)
            except Exception:
                built = None
            if built is not None:
                with self._lock:
                    self._ready = built


# =============================================================================
def _selftest() -> None:
    v = 0.08

    def grid(pts):
        return np.asarray(pts, np.float64) * v

    # 1. flat sheet in the z=0 plane → normals must be ±Z (plane is XY).
    xs, ys = np.meshgrid(np.arange(-4, 5), np.arange(-4, 5))
    sheet = np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], axis=1) * v
    n = estimate_normals(sheet, v)
    interior = (np.abs(sheet[:, 0]) < 3 * v) & (np.abs(sheet[:, 1]) < 3 * v)
    assert np.all(np.abs(n[interior][:, 2]) > 0.9), n[interior][:5]

    # 2. wall in the x=0 plane (spans y,z) → normals must be ±X.
    ys2, zs2 = np.meshgrid(np.arange(-4, 5), np.arange(-4, 5))
    wall = np.stack([np.zeros(ys2.size), ys2.ravel(), zs2.ravel()], axis=1) * v
    nw = estimate_normals(wall, v)
    inw = (np.abs(wall[:, 1]) < 3 * v) & (np.abs(wall[:, 2]) < 3 * v)
    assert np.all(np.abs(nw[inw][:, 0]) > 0.9), nw[inw][:5]

    # 3. quads: planar (⊥ normal), right count, right side length.
    rgb = np.tile([120, 200, 80], (sheet.shape[0], 1)).astype(np.uint8)
    verts, faces, cols = build_surfels(sheet, rgb, v)
    assert verts.shape == (sheet.shape[0] * 4, 3)
    assert faces.shape == (sheet.shape[0] * 2, 3)
    assert cols.shape[0] == sheet.shape[0] * 4
    # first quad lies in a plane whose normal matches the estimated one
    e1 = verts[1] - verts[0]; e2 = verts[3] - verts[0]
    quad_n = np.cross(e1, e2); quad_n /= np.linalg.norm(quad_n)
    assert abs(abs(np.dot(quad_n, n[0])) - 1.0) < 1e-4, quad_n
    # edge length ≈ side (= v)
    assert abs(np.linalg.norm(e1) - v) < 1e-5, np.linalg.norm(e1)
    # faces index within range, no NaNs
    assert faces.max() < verts.shape[0] and np.isfinite(verts).all()

    # 4. empty cloud
    ev, ef, ec = build_surfels(np.zeros((0, 3)), np.zeros((0, 3), np.uint8), v)
    assert ev.shape == (0, 3) and ef.shape == (0, 3)

    # 5. background worker: submit → poll → get the same mesh, latest-only.
    import time as _t
    w = SurfelWorker(v)
    w.submit(sheet, rgb)
    got = None
    for _ in range(200):                 # ≤2 s
        got = w.take_ready()
        if got is not None:
            break
        _t.sleep(0.01)
    assert got is not None and got[0].shape[0] == sheet.shape[0] * 4, "worker produced no mesh"
    assert w.take_ready() is None, "ready slot should be consumed once"
    w.stop()

    print(f"surfel self-test OK  (sheet→±Z, wall→±X, quads planar & tiling @ {v} m; worker OK)")


if __name__ == "__main__":
    _selftest()
