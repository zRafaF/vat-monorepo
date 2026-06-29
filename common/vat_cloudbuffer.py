"""
VAT — Incremental Cloud Buffer  (client render side)
====================================================
A persistent, contiguous (xyz, rgb) buffer the viewer keeps on the GPU-feeding
side and updates PER CUBE from ``ClientBlockStore.take_delta()`` instead of
re-concatenating the whole map every submap (the old ``take_merged`` did an O(N)
rebuild + full re-upload each change — the source of the render "stalls").

Design: a slab over two NumPy arrays with a per-cube slot map.
  * a changed cube whose point count is unchanged is overwritten IN PLACE;
  * otherwise its old slot is freed (filled with NaN so the render-side finite
    filter drops it) and the new points are appended;
  * removed cubes free their slot (NaN);
  * when the dead fraction grows past ``compact_frac`` the buffer is rebuilt
    contiguous from the slot map.

``live()`` returns ``pos[:n], rgb[:n]`` — which may contain NaN holes between
compactions; the viewer already filters non-finite points before drawing, so the
holes never render. NumPy-only and unit-tested (``python common/vat_cloudbuffer.py``).
"""

from __future__ import annotations

from typing import Dict, Tuple

import numpy as np


class IncrementalCloud:
    def __init__(self, init_cap: int = 1 << 16, compact_frac: float = 0.5):
        self._cap = int(init_cap)
        self.pos = np.full((self._cap, 3), np.nan, dtype=np.float32)
        self.rgb = np.zeros((self._cap, 3), dtype=np.uint8)
        self.n = 0                                   # high-water live length
        self._slot: Dict[int, Tuple[int, int]] = {}  # key -> (offset, count)
        self._dead = 0                                # NaN'd rows in [0, n)
        self._compact_frac = float(compact_frac)
        self.dirty = False

    # ── internal ─────────────────────────────────────────────────────────────
    def _grow(self, need: int):
        if self.n + need <= self._cap:
            return
        new_cap = self._cap
        while self.n + need > new_cap:
            new_cap *= 2
        pos = np.full((new_cap, 3), np.nan, dtype=np.float32)
        rgb = np.zeros((new_cap, 3), dtype=np.uint8)
        pos[:self.n] = self.pos[:self.n]
        rgb[:self.n] = self.rgb[:self.n]
        self.pos, self.rgb, self._cap = pos, rgb, new_cap

    def _free(self, off: int, count: int):
        self.pos[off:off + count] = np.nan
        self._dead += count

    def _append(self, key: int, xyz: np.ndarray, rgb: np.ndarray):
        count = xyz.shape[0]
        self._grow(count)
        off = self.n
        self.pos[off:off + count] = xyz
        self.rgb[off:off + count] = rgb
        self.n += count
        self._slot[key] = (off, count)

    def _set_block(self, key: int, xyz: np.ndarray, rgb: np.ndarray):
        xyz = np.ascontiguousarray(xyz, np.float32).reshape(-1, 3)
        rgb = np.ascontiguousarray(rgb, np.uint8).reshape(-1, 3)
        old = self._slot.get(key)
        if old is not None and old[1] == xyz.shape[0]:
            off, count = old
            self.pos[off:off + count] = xyz       # in-place, no fragmentation
            self.rgb[off:off + count] = rgb
            return
        if old is not None:
            self._free(*old)
            del self._slot[key]
        if xyz.shape[0]:
            self._append(key, xyz, rgb)

    def _remove(self, key: int):
        old = self._slot.pop(key, None)
        if old is not None:
            self._free(*old)

    def compact(self):
        """Rebuild contiguous from the slot map (drops all NaN holes)."""
        total = sum(c for _, c in self._slot.values())
        cap = max(self._cap, 1)
        while total > cap:
            cap *= 2
        pos = np.full((cap, 3), np.nan, dtype=np.float32)
        rgb = np.zeros((cap, 3), dtype=np.uint8)
        off = 0
        new_slot: Dict[int, Tuple[int, int]] = {}
        for key, (o, c) in self._slot.items():
            pos[off:off + c] = self.pos[o:o + c]
            rgb[off:off + c] = self.rgb[o:o + c]
            new_slot[key] = (off, c)
            off += c
        self.pos, self.rgb, self._cap = pos, rgb, cap
        self._slot, self.n, self._dead = new_slot, off, 0

    # ── public ───────────────────────────────────────────────────────────────
    def apply(self, changed: Dict[int, Tuple[np.ndarray, np.ndarray]],
              removed, resync: bool = False):
        if resync:
            self.clear()
        for key in removed:
            self._remove(int(key))
        for key, (xyz, rgb) in changed.items():
            self._set_block(int(key), xyz, rgb)
        if self.n and self._dead / max(self.n, 1) > self._compact_frac:
            self.compact()
        self.dirty = True

    def live(self):
        """→ (pos, rgb) views over the live prefix (may contain NaN holes that the
        viewer's finite-filter drops). Cheap; no copy."""
        self.dirty = False
        return self.pos[:self.n], self.rgb[:self.n]

    def clear(self):
        self.n = 0
        self._dead = 0
        self._slot.clear()
        self.dirty = True


# =============================================================================
def _selftest() -> None:
    rng = np.random.default_rng(1)
    buf = IncrementalCloud(init_cap=8)

    def blk(m):
        return (rng.random((m, 3)).astype(np.float32), (rng.random((m, 3)) * 255).astype(np.uint8))

    a, b, c = blk(3), blk(5), blk(2)
    buf.apply({1: a, 2: b, 3: c}, removed=set(), resync=True)
    p, r = buf.live()
    finite = np.isfinite(p).all(axis=1)
    assert finite.sum() == 10, finite.sum()

    # update key 2 with the SAME count → in-place, no growth in live length
    n_before = buf.n
    b2 = blk(5)
    buf.apply({2: b2}, removed=set())
    assert buf.n == n_before
    off, cnt = buf._slot[2]
    assert np.allclose(buf.pos[off:off + cnt], b2[0])

    # update key 1 with a DIFFERENT count → old freed (hole), new appended
    a2 = blk(7)
    buf.apply({1: a2}, removed=set())
    p, r = buf.live()
    finite = np.isfinite(p).all(axis=1)
    assert finite.sum() == 5 + 2 + 7, finite.sum()

    # remove key 3
    buf.apply({}, removed={3})
    p, r = buf.live()
    assert np.isfinite(p).all(axis=1).sum() == 5 + 7

    # force compaction; live point total must be exactly the sum of slot counts
    for _ in range(20):
        k = int(rng.integers(1, 4))
        buf.apply({k: blk(int(rng.integers(1, 6)))}, removed=set())
    buf.compact()
    p, r = buf.live()
    assert np.isfinite(p).all(axis=1).sum() == buf.n == sum(c for _, c in buf._slot.values())
    # after compaction there are no NaN holes
    assert np.isfinite(p).all()
    print("vat_cloudbuffer self-test OK  (in-place / append / remove / compact)")


if __name__ == "__main__":
    _selftest()
