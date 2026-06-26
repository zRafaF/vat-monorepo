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

DEFAULT_CUBE_M   = 1.0
# Position quantisation for the Draco blob. Points are voxel-grid-aligned, so only
# sub-voxel accuracy is needed: 10 bits over a ~10-20 m room ≈ 1-2 cm, plenty for a
# 3 cm voxel map and ~30-40% smaller than the old 12-bit (~1 mm) setting. Level 10
# is Draco's max ratio (decode still ~ms). Both env-tunable without code changes.
DRACO_QUANT_BITS = int(os.environ.get("DRACO_QUANT_BITS", "10"))
DRACO_LEVEL      = int(os.environ.get("DRACO_LEVEL", "10"))
_HASH_QUANT      = 1000.0     # positions hashed at 1 mm so the CRC is stable
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

    def __init__(self, cube_m: float = DEFAULT_CUBE_M):
        self.cube_m = float(cube_m)
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
        fq = np.rint(xyz * _HASH_QUANT).astype(np.int64)            # 1 mm grid for CRC
        # canonical order: by cube, then by quantised position → order-independent CRC
        order = np.lexsort((fq[:, 2], fq[:, 1], fq[:, 0], keys))
        keys_s, xyz_s, rgb_s, fq_s = keys[order], xyz[order], rgb_u8[order], fq[order]
        uniq, starts = np.unique(keys_s, return_index=True)
        ends = np.append(starts[1:], n)

        new_blocks: Dict[int, Tuple[int, np.ndarray, np.ndarray]] = {}
        changed: List[int] = []
        for kk, s, e in zip(uniq.tolist(), starts.tolist(), ends.tolist()):
            crc = zlib.crc32(fq_s[s:e].tobytes()) & 0xFFFFFFFF
            crc = zlib.crc32(rgb_s[s:e].tobytes(), crc) & 0xFFFFFFFF
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


# ─────────────────────────────────────────────────────────────────────────────
# Client store
# ─────────────────────────────────────────────────────────────────────────────


class ClientBlockStore:
    """Client-side cube store: apply bundles, drop stale cubes, merge for render."""

    def __init__(self, cube_m: float = DEFAULT_CUBE_M):
        self.cube_m = float(cube_m)
        self.blocks: Dict[int, Tuple[int, np.ndarray, np.ndarray]] = {}
        self._dirty = True

    def local_manifest(self) -> Dict[int, int]:
        return {k: v[0] for k, v in self.blocks.items()}

    def apply_bundle_bytes(self, buf: bytes) -> int:
        got = unpack_bundle(buf, self.cube_m)
        self.blocks.update(got)
        self._dirty = True
        return len(got)

    def drop(self, keys: List[int]):
        for k in keys:
            self.blocks.pop(int(k), None)
        if keys:
            self._dirty = True

    def clear(self):
        self.blocks = {}
        self._dirty = True

    def merged(self):
        """→ (xyz (N,3) float32, rgb (N,3) uint8) of the whole map, or None if
        unchanged since the last call."""
        if not self._dirty:
            return None
        self._dirty = False
        if not self.blocks:
            return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8)
        xyz = np.concatenate([b[1] for b in self.blocks.values()])
        rgb = np.concatenate([b[2] for b in self.blocks.values()])
        return xyz, rgb
