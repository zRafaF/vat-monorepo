"""
VAT — Spatial Block Map  (content-versioned cube grid + Draco block sync)
========================================================================
Shared core for the **diff-based point-cloud sync** that replaces full-snapshot
streaming. The world is partitioned into a sparse grid of fixed-size **cubes**;
each cube carries a **CRC of its contents**, so the server can advertise a tiny
"manifest" (one ``(cube_key, crc)`` per occupied cube) and the client requests
*only* the cubes whose CRC changed. Requested cubes travel back as ONE
**Draco-compressed** blob.

Why this fixes things
---------------------
* **No misalignment / no drift.** Each cube is replaced *wholesale* when its CRC
  changes — there is no incremental delta to accumulate, and a global re-anchor
  (PRISM re-levels/rescales) simply flips every CRC → the client refetches all.
* **Tiny bandwidth.** Steady state ships only the few cubes that changed, Draco-
  compressed (~1.4× over our quant+zlib, ~mm error). The manifest is ~12 B/cube.

Wire pieces (see :func:`vat_protocol.keys` for the Zenoh keys)
-------------------------------------------------------------
* ``manifest``  server→client : ``{cube_key: crc}`` — what the map looks like now.
* ``request``   client→server : the cube_keys whose CRC the client lacks/mismatches.
* ``bundle``    server→client : an index ``(key, crc)`` + one Draco blob of all the
  requested cubes' points; the client re-derives each decoded point's cube from
  its position (Draco reorders points, but its mm-error never crosses a cube edge).

Cubes are created on demand wherever points exist (sparse dict), so the grid grows
with the map automatically — no fixed extent. A coarse-level rollup hash (Merkle)
can be layered on later if the manifest ever gets large; at room scale the flat
manifest is a few KB.

Dependency-light: NumPy + stdlib. ``DracoPy`` is required only for bundle
encode/decode (guarded import) — manifests/requests work without it.
"""

from __future__ import annotations

import os
import struct
import threading
import zlib
from typing import Dict, List, Tuple

import numpy as np

try:
    import DracoPy
    _HAVE_DRACO = True
except Exception:                       # pragma: no cover - optional at import time
    DracoPy = None
    _HAVE_DRACO = False

MAGIC_MANIFEST = 0x424D4E46   # "BMNF"
MAGIC_REQUEST  = 0x42524551   # "BREQ"
MAGIC_BUNDLE   = 0x42424E44   # "BBND"
MAGIC_PUSH     = 0x42505348   # "BPSH" — proactive changed+removed block delta

DEFAULT_CUBE_M   = 1.0
# Position quantisation for the Draco blob. Points are voxel-grid-aligned, so only
# sub-voxel accuracy is needed: 10 bits over a ~10-20 m room ≈ 1-2 cm, plenty for a
# 3 cm voxel map and ~30-40% smaller than the old 12-bit (~1 mm) setting. Level 10
# is Draco's max ratio (decode still ~ms). Both env-tunable without code changes.
DRACO_QUANT_BITS = int(os.environ.get("DRACO_QUANT_BITS", "10"))
DRACO_LEVEL      = int(os.environ.get("DRACO_LEVEL", "10"))

# ── CRC quantisation: the breathing-vs-carving fix ───────────────────────────
# A cube's version is the CRC of its points snapped to a quantisation grid. The OLD
# default hashed at 1 mm (``_HASH_QUANT`` = 1000 pts/m): nvblox marching-cubes
# vertices slide sub-voxel every time a block is re-meshed ("breathing"), so a 1 mm
# CRC flips on EVERY block the camera re-observes even when the geometry is
# unchanged → the diff degenerates to a near-full resend (the "+119/-0 cubes while
# stationary" pathology). Hashing on a COARSE occupancy grid (default ½ voxel via
# the server) makes sub-quantum breathing invisible to the CRC, while a real change
# — a voxel carved away, or a surface that shifts ≥ a grid cell — still flips it.
# Carving therefore propagates correctly (the cube re-sends, or disappears from the
# manifest) but noise does not. ``crc_quant_m`` is set per BlockGrid by the server;
# 0/None keeps the legacy 1 mm behaviour.
DEFAULT_CRC_QUANT_M = (float(os.environ["BLOCKMAP_CRC_QUANT_M"])
                       if os.environ.get("BLOCKMAP_CRC_QUANT_M") else None)
_HASH_QUANT      = 1000.0     # legacy fallback: positions hashed at 1 mm
_KEY_OFF         = 1 << 20    # ±~1e6 cubes per axis fits 21 bits each in an int64


# ─────────────────────────────────────────────────────────────────────────────
# Cube keying  (integer grid coords ⇄ packed int64)
# ─────────────────────────────────────────────────────────────────────────────


def cube_keys(xyz: np.ndarray, cube_m: float) -> np.ndarray:
    """Vectorised: world points (n,3) → packed int64 cube key per point."""
    ijk = np.floor(np.asarray(xyz, dtype=np.float64) / cube_m).astype(np.int64)
    ijk += _KEY_OFF
    return (ijk[:, 0] << 42) | (ijk[:, 1] << 21) | ijk[:, 2]


def unpack_cube_key(key: int) -> Tuple[int, int, int]:
    i = (key >> 42) & 0x1FFFFF
    j = (key >> 21) & 0x1FFFFF
    k = key & 0x1FFFFF
    return i - _KEY_OFF, j - _KEY_OFF, k - _KEY_OFF


# ─────────────────────────────────────────────────────────────────────────────
# Block grid  (canonical, content-versioned)
# ─────────────────────────────────────────────────────────────────────────────


class BlockGrid:
    """A cube_key → (crc, xyz float32, rgb uint8) store, rebuilt from a full cloud.

    Used server-side as the canonical map; the client keeps its own dict and only
    stores the cubes it has been sent."""

    def __init__(self, cube_m: float = DEFAULT_CUBE_M,
                 crc_quant_m: float | None = DEFAULT_CRC_QUANT_M):
        self.cube_m = float(cube_m)
        # CRC position grid (m). None/0 → legacy 1 mm. The server sets this to ≈½
        # voxel so sub-voxel mesh "breathing" doesn't flip cube versions; see
        # DEFAULT_CRC_QUANT_M for the full rationale.
        self.crc_quant_m = float(crc_quant_m) if crc_quant_m else None
        self._hash_quant = (1.0 / self.crc_quant_m) if self.crc_quant_m else _HASH_QUANT
        self.blocks: Dict[int, Tuple[int, np.ndarray, np.ndarray]] = {}

    def ingest(self, xyz: np.ndarray, rgb: np.ndarray) -> Tuple[List[int], List[int]]:
        """Rebuild the grid from a full cloud. ``rgb`` may be float [0,1] or uint8.
        Returns ``(changed_keys, removed_keys)`` vs. the previous state."""
        xyz = np.ascontiguousarray(xyz, dtype=np.float32).reshape(-1, 3)
        rgb = np.asarray(rgb).reshape(-1, 3)
        rgb_u8 = (np.clip(rgb, 0, 1) * 255 + 0.5).astype(np.uint8) \
            if rgb.dtype.kind == "f" else rgb.astype(np.uint8)
        n = xyz.shape[0]
        if n == 0:
            removed = list(self.blocks.keys())
            self.blocks = {}
            return [], removed

        keys = cube_keys(xyz, self.cube_m)
        # Snap to the CRC grid (≈½ voxel by default) so the per-cube CRC is invariant
        # to sub-quantum marching-cubes breathing, but still flips on real carving /
        # surface motion ≥ one grid cell.
        fq = np.rint(xyz * self._hash_quant).astype(np.int64)
        # canonical order: by cube, then by quantised position → order-independent CRC
        order = np.lexsort((fq[:, 2], fq[:, 1], fq[:, 0], keys))
        keys_s, xyz_s, rgb_s, fq_s = keys[order], xyz[order], rgb_u8[order], fq[order]
        uniq, starts = np.unique(keys_s, return_index=True)
        ends = np.append(starts[1:], n)

        new_blocks: Dict[int, Tuple[int, np.ndarray, np.ndarray]] = {}
        changed: List[int] = []
        # A cube's version (CRC) is GEOMETRY-ONLY by default. The colorizer
        # re-projects best-view colour for every vertex each submap, so even a
        # perfectly static surface gets slightly different colours every time — if
        # colour were in the CRC, every cube would flip every submap and the diff
        # would degenerate to a full resend (the "synced N/N cubes" pathology). With
        # geometry-only versioning a cube re-sends only when its shape changes
        # (bandwidth ∝ frontier, not total map). The full 8-bit colour still travels
        # in the bundle; a static cube simply keeps the colour it was last sent with.
        # Set BLOCKMAP_CRC_COLOR=1 to fold a coarse (4-bit) colour back into the CRC.
        include_color = os.environ.get("BLOCKMAP_CRC_COLOR", "0") == "1"
        rgb_crc = (rgb_s >> 4) if include_color else None
        occupancy = self.crc_quant_m is not None
        for kk, s, e in zip(uniq.tolist(), starts.tolist(), ends.tolist()):
            cell = fq_s[s:e]
            if occupancy:
                # OCCUPANCY CRC: hash the SET of occupied grid cells (deduped), so
                # the version depends only on which cells are filled — invariant to
                # how many breathing vertices landed in each, flips only on carving.
                cell = np.unique(cell, axis=0)
            crc = zlib.crc32(np.ascontiguousarray(cell).tobytes()) & 0xFFFFFFFF
            if include_color:
                crc = zlib.crc32(rgb_crc[s:e].tobytes(), crc) & 0xFFFFFFFF
            new_blocks[kk] = (crc, np.ascontiguousarray(xyz_s[s:e]),
                              np.ascontiguousarray(rgb_s[s:e]))
            old = self.blocks.get(kk)
            if old is None or old[0] != crc:
                changed.append(kk)
        removed = [k for k in self.blocks if k not in new_blocks]
        self.blocks = new_blocks
        return changed, removed

    def manifest(self) -> Dict[int, int]:
        return {k: v[0] for k, v in self.blocks.items()}

    def collect(self, keys: List[int]):
        """→ list of (key, crc, xyz, rgb) for the requested keys we actually hold."""
        out = []
        for k in keys:
            b = self.blocks.get(int(k))
            if b is not None:
                out.append((int(k), b[0], b[1], b[2]))
        return out


# ─────────────────────────────────────────────────────────────────────────────
# Manifest  (server → client)
# ─────────────────────────────────────────────────────────────────────────────


def pack_manifest(manifest: Dict[int, int]) -> bytes:
    keys = np.fromiter(manifest.keys(), dtype=">i8", count=len(manifest))
    crcs = np.fromiter(manifest.values(), dtype=">u4", count=len(manifest))
    return struct.pack("!ii", MAGIC_MANIFEST, len(manifest)) + keys.tobytes() + crcs.tobytes()


def unpack_manifest(buf: bytes) -> Dict[int, int]:
    magic, n = struct.unpack_from("!ii", buf, 0)
    if magic != MAGIC_MANIFEST:
        raise ValueError("bad manifest magic")
    off = 8
    keys = np.frombuffer(buf, ">i8", count=n, offset=off)
    crcs = np.frombuffer(buf, ">u4", count=n, offset=off + n * 8)
    return {int(k): int(c) for k, c in zip(keys, crcs)}


def diff_manifest(local: Dict[int, int], remote: Dict[int, int]) -> Tuple[List[int], List[int]]:
    """→ (need, drop): cubes whose CRC differs or is missing locally, and cubes the
    client holds that no longer exist remotely (to be deleted)."""
    need = [k for k, c in remote.items() if local.get(k) != c]
    drop = [k for k in local if k not in remote]
    return need, drop


# ─────────────────────────────────────────────────────────────────────────────
# Request  (client → server)
# ─────────────────────────────────────────────────────────────────────────────


def pack_request(keys: List[int]) -> bytes:
    arr = np.asarray(keys, dtype=">i8")
    return struct.pack("!ii", MAGIC_REQUEST, len(keys)) + arr.tobytes()


def unpack_request(buf: bytes) -> List[int]:
    magic, n = struct.unpack_from("!ii", buf, 0)
    if magic != MAGIC_REQUEST:
        raise ValueError("bad request magic")
    return np.frombuffer(buf, ">i8", count=n, offset=8).astype(np.int64).tolist()


# ─────────────────────────────────────────────────────────────────────────────
# Bundle  (server → client) — index + ONE Draco blob of all requested points
# ─────────────────────────────────────────────────────────────────────────────


def pack_bundle(blocks, cube_m: float = DEFAULT_CUBE_M,
                quant_bits: int = DRACO_QUANT_BITS, level: int = DRACO_LEVEL) -> bytes:
    """``blocks`` = list of (key, crc, xyz float32, rgb uint8). The points of every
    block are concatenated and Draco-encoded once; the index carries (key, crc) so
    the client can update its CRCs (it re-derives point→cube from geometry)."""
    if not _HAVE_DRACO:
        raise RuntimeError("DracoPy not installed (needed for pack_bundle)")
    keys = np.array([b[0] for b in blocks], dtype=">i8")
    crcs = np.array([b[1] for b in blocks], dtype=">u4")
    if blocks:
        allxyz = np.concatenate([np.asarray(b[2], np.float32) for b in blocks])
        allrgb = np.concatenate([np.asarray(b[3], np.uint8) for b in blocks])
        blob = DracoPy.encode(allxyz, colors=allrgb,
                              quantization_bits=quant_bits, compression_level=level)
    else:
        blob = b""
    header = struct.pack("!iii", MAGIC_BUNDLE, len(blocks), len(blob))
    return header + keys.tobytes() + crcs.tobytes() + blob


def unpack_bundle(buf: bytes, cube_m: float = DEFAULT_CUBE_M):
    """→ dict ``key → (crc, xyz float32, rgb uint8)`` for every block in the index.
    Decoded points are re-binned to cubes by position (Draco reorders, but its
    mm-error never crosses a cube edge for voxel-centre clouds)."""
    if not _HAVE_DRACO:
        raise RuntimeError("DracoPy not installed (needed for unpack_bundle)")
    magic, nb, bloblen = struct.unpack_from("!iii", buf, 0)
    if magic != MAGIC_BUNDLE:
        raise ValueError("bad bundle magic")
    off = 12
    keys = np.frombuffer(buf, ">i8", count=nb, offset=off).astype(np.int64)
    crcs = np.frombuffer(buf, ">u4", count=nb, offset=off + nb * 8).astype(np.int64)
    blob = buf[off + nb * 12: off + nb * 12 + bloblen]

    out: Dict[int, Tuple[int, np.ndarray, np.ndarray]] = {
        int(k): (int(c), np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8))
        for k, c in zip(keys, crcs)}
    if bloblen and nb:
        dec = DracoPy.decode(blob)
        xyz = np.asarray(dec.points, dtype=np.float32)
        rgb = (np.asarray(dec.colors, dtype=np.uint8) if getattr(dec, "colors", None) is not None
               and len(np.asarray(dec.colors)) else np.full((len(xyz), 3), 200, np.uint8))
        want = set(int(k) for k in keys)
        pk = cube_keys(xyz, cube_m)
        order = np.argsort(pk)
        pk_s, xyz_s, rgb_s = pk[order], xyz[order], rgb[order]
        uniq, starts = np.unique(pk_s, return_index=True)
        ends = np.append(starts[1:], len(pk_s))
        crc_of = {int(k): int(c) for k, c in zip(keys, crcs)}
        for u, s, e in zip(uniq.tolist(), starts.tolist(), ends.tolist()):
            if u in want:
                out[u] = (crc_of[u], np.ascontiguousarray(xyz_s[s:e]),
                          np.ascontiguousarray(rgb_s[s:e]))
    return out


# -----------------------------------------------------------------------------
# Push delta  (server -> client, pub/sub): proactive changed + removed cubes
# -----------------------------------------------------------------------------
#
# The pull path (manifest -> client diff -> request -> bundle reply) costs a full
# round-trip plus an on-demand server collect EVERY submap. The server already
# knows which cubes changed/were removed the instant it re-ingests, so it PUSHES
# them straight away: no request, no RTT. The manifest/query path stays as the
# repair + bootstrap channel (a late-joining client, or a dropped push, is
# reconciled by the next manifest diff). Layout:
#
#   MAGIC_PUSH (i) | map_version (i) | n_removed (i) | reserved (i)
#   removed_keys[n_removed] (>i8)
#   <bundle bytes>                      (pack_bundle of the changed cubes)
#
_PUSH_HDR = "!iiii"
_PUSH_HDR_SIZE = struct.calcsize(_PUSH_HDR)


def pack_block_push(changed_blocks, removed_keys, map_version: int = 0,
                    cube_m: float = DEFAULT_CUBE_M,
                    quant_bits: int = DRACO_QUANT_BITS, level: int = DRACO_LEVEL) -> bytes:
    """``changed_blocks`` = list of (key, crc, xyz, rgb); ``removed_keys`` = cube
    keys to delete client-side. Returns one self-contained push frame."""
    rem = np.asarray(list(removed_keys), dtype=">i8")
    header = struct.pack(_PUSH_HDR, MAGIC_PUSH, int(map_version), int(rem.size), 0)
    bundle = pack_bundle(changed_blocks, cube_m, quant_bits, level)
    return header + rem.tobytes() + bundle


def unpack_block_push(buf: bytes, cube_m: float = DEFAULT_CUBE_M):
    """-> (map_version, removed_keys list, got dict key->(crc,xyz,rgb))."""
    if len(buf) < _PUSH_HDR_SIZE:
        raise ValueError("push buffer too short")
    magic, map_version, n_removed, _res = struct.unpack_from(_PUSH_HDR, buf, 0)
    if magic != MAGIC_PUSH:
        raise ValueError("bad push magic")
    off = _PUSH_HDR_SIZE
    removed = np.frombuffer(buf, ">i8", count=n_removed, offset=off).astype(np.int64).tolist()
    off += n_removed * 8
    got = unpack_bundle(buf[off:], cube_m)
    return int(map_version), removed, got


# -----------------------------------------------------------------------------
# Client store
# -----------------------------------------------------------------------------


class ClientBlockStore:
    """Client-side cube store: apply bundles/pushes, drop stale cubes, merge for
    render. Tracks per-key changes so the viewer can update only affected GPU
    slots (take_delta) instead of re-uploading the whole cloud (merged)."""

    def __init__(self, cube_m: float = DEFAULT_CUBE_M):
        self.cube_m = float(cube_m)
        self.blocks: Dict[int, Tuple[int, np.ndarray, np.ndarray]] = {}
        self._dirty = True
        # The Zenoh sync thread mutates ``blocks`` while the render thread reads it;
        # the lock keeps merged()/take_delta() consistent under concurrent updates.
        self._lock = threading.Lock()
        # Incremental render bookkeeping: keys touched / removed since the last
        # take_delta(). ``_resync`` forces a full rebuild (after clear / first frame).
        self._delta_changed: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
        self._delta_removed: set = set()
        self._resync = True

    def local_manifest(self) -> Dict[int, int]:
        with self._lock:
            return {k: v[0] for k, v in self.blocks.items()}

    def _note_changed(self, key, xyz, rgb):
        self._delta_changed[int(key)] = (xyz, rgb)
        self._delta_removed.discard(int(key))

    def _note_removed(self, key):
        self._delta_removed.add(int(key))
        self._delta_changed.pop(int(key), None)

    def apply_bundle_bytes(self, buf: bytes) -> int:
        got = unpack_bundle(buf, self.cube_m)
        with self._lock:
            self.blocks.update(got)
            for k, v in got.items():
                self._note_changed(k, v[1], v[2])
            if got:
                self._dirty = True
        return len(got)

    def apply_push_bytes(self, buf: bytes):
        """Apply a proactive push: drop removed cubes, replace changed ones.
        Returns ``(map_version, n_applied, n_removed)``."""
        map_version, removed, got = unpack_block_push(buf, self.cube_m)
        with self._lock:
            for k in removed:
                if self.blocks.pop(int(k), None) is not None:
                    self._note_removed(k)
            self.blocks.update(got)
            for k, v in got.items():
                self._note_changed(k, v[1], v[2])
            if got or removed:
                self._dirty = True
        return map_version, len(got), len(removed)

    def drop(self, keys: List[int]):
        with self._lock:
            for k in keys:
                if self.blocks.pop(int(k), None) is not None:
                    self._note_removed(k)
                    self._dirty = True

    def clear(self):
        with self._lock:
            self.blocks = {}
            self._delta_changed.clear()
            self._delta_removed.clear()
            self._resync = True
            self._dirty = True

    def take_delta(self):
        """-> (changed dict key->(xyz,rgb), removed set, full_resync bool) of the
        cubes touched since the last call, or None if nothing changed. When
        full_resync is True the caller must rebuild from scratch."""
        with self._lock:
            if not self._dirty:
                return None
            self._dirty = False
            changed = self._delta_changed
            removed = self._delta_removed
            resync = self._resync
            self._delta_changed = {}
            self._delta_removed = set()
            self._resync = False
            if resync:                       # hand the caller the whole current map
                changed = {k: (v[1], v[2]) for k, v in self.blocks.items()}
                removed = set()
            return changed, removed, resync

    def merged(self):
        """-> (xyz (N,3) float32, rgb (N,3) uint8) of the whole map, or None if
        unchanged since the last call. ONE atomic snapshot so xyz/rgb lengths match
        even while the sync thread updates the store."""
        with self._lock:
            if not self._dirty:
                return None
            self._dirty = False
            vals = list(self.blocks.values())          # single consistent snapshot
        if not vals:
            return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8)
        xyz = np.concatenate([b[1] for b in vals])
        rgb = np.concatenate([b[2] for b in vals])
        return xyz, rgb


# =============================================================================
# Self-test:  python common/vat_blockmap.py   (NumPy-only parts; Draco optional)
# =============================================================================

def _selftest() -> None:
    rng = np.random.default_rng(0)

    pts = (rng.random((100, 3)) - 0.5) * 20.0
    ks = cube_keys(pts, 1.0)
    for p, k in zip(pts[:10], ks[:10]):
        i, j, l = unpack_cube_key(int(k))
        assert (i, j, l) == tuple(np.floor(p).astype(int)), (p, (i, j, l))

    # -- THE breathing-vs-carving test (the whole point of this refactor) --
    voxel = 0.03
    crc_q = voxel / 2.0
    centers = (np.floor(rng.random((4000, 3)) * 30) + 0.5) * voxel
    centers = np.unique(centers, axis=0)
    col = (rng.random((centers.shape[0], 3)) * 255).astype(np.uint8)
    g = BlockGrid(cube_m=1.0, crc_quant_m=crc_q)
    changed0, removed0 = g.ingest(centers, col)
    assert len(changed0) > 0 and len(removed0) == 0
    n_cubes = len(g.blocks)

    jit = (rng.random(centers.shape) - 0.5) * (crc_q * 0.9)
    col2 = (rng.random(centers.shape) * 255).astype(np.uint8)
    changed1, removed1 = g.ingest(centers + jit, col2)
    assert changed1 == [] and removed1 == [], \
        f"breathing leaked into CRC: {len(changed1)} changed (must be 0)"

    gl = BlockGrid(cube_m=1.0, crc_quant_m=None)
    gl.ingest(centers, col)
    cl, _ = gl.ingest(centers + jit, col2)
    assert len(cl) > 0, "legacy CRC unexpectedly stable"

    a_key = next(iter(g.blocks))
    keep = cube_keys(centers, 1.0) != a_key
    changed2, removed2 = g.ingest((centers + jit)[keep], col2[keep])
    assert a_key in removed2 and changed2 == [], (len(changed2), removed2[:3])
    assert len(g.blocks) == n_cubes - 1

    man = g.manifest()
    assert unpack_manifest(pack_manifest(man)) == man
    need, drop = diff_manifest({}, man)
    assert set(need) == set(man) and drop == []
    need2, drop2 = diff_manifest(man, {})
    assert need2 == [] and set(drop2) == set(man)

    req_keys = list(man)[:5]
    assert unpack_request(pack_request(req_keys)) == req_keys

    removed_keys = [123, 456, 789]
    if _HAVE_DRACO:
        blocks = g.collect(list(g.blocks)[:3])
        buf = pack_block_push(blocks, removed_keys, map_version=42, cube_m=1.0)
        mv, rem, got = unpack_block_push(buf, cube_m=1.0)
        assert mv == 42 and rem == removed_keys and len(got) == len(blocks)
        store = ClientBlockStore(cube_m=1.0)
        v, na, nr = store.apply_push_bytes(buf)
        assert v == 42 and na == len(blocks)
        d = store.take_delta()
        assert d is not None and d[2] is True and len(d[0]) == len(blocks)
        assert store.take_delta() is None
        print("vat_blockmap self-test OK  (occupancy-CRC + push + Draco bundle)")
    else:
        try:
            pack_block_push([], removed_keys, cube_m=1.0)
        except RuntimeError:
            pass
        else:
            raise AssertionError("expected RuntimeError without DracoPy")
        print("vat_blockmap self-test OK  (occupancy-CRC + framing; Draco NOT installed)")


if __name__ == "__main__":
    _selftest()
