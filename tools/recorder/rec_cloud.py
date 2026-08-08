"""
VAT recorder — the map streams (point cloud, ESDF slice, server status).
=======================================================================
This is where recording is least obvious, because the map transport is a *diff*
and carries no timestamps.

Point cloud
-----------
With ``STREAM_MODE=blocks`` (the default) the server publishes, per submap:

1. a Draco **push** on ``{server}/pcd/push`` — the cubes that changed plus the keys
   that vanished, tagged with ``map_version``; then
2. a **manifest** on ``{server}/pcd/manifest`` — one ``(cube_key, crc)`` per occupied
   cube, no version and no timestamp.

Both channels are ``CongestionControl.DROP``, so *a recorder will lose pushes* on a
busy link, exactly as a client does. :class:`PointCloudRecorder` therefore does what
``client/vat_client/block_sync.py`` does — keeps a mirror of the map in a
``vat_blockmap.ClientBlockStore``, diffs each manifest against it, and pulls the
missing cubes from the ``{server}/pcd/blocks`` queryable. That is also what makes a
**mid-session start** yield a complete map: the first manifest diff pulls everything.

Three artefacts come out of it:

* ``pointcloud/blocks/*.bin`` — every push / manifest / repair bundle, **byte-exact**.
  Offline replay uses the repo's own ``vat_blockmap`` unpackers, so there is no
  second serialisation to drift.
* ``pointcloud/keyframes/*.npz`` — the mirror materialised on a timer. These are
  **free**: the recorder already holds the whole map, so a keyframe costs no query
  and no server work, and it means ``compose.py`` can seek to any time without
  replaying from the start of the session.
* ``pointcloud/index.jsonl`` — the ordered index tying each artefact to a
  ``map_version`` and a session timestamp.

Timestamps
----------
None of the map messages carry one. Each record gets ``ts_src="derived"`` (arrival
mapped onto the session clock) *and*, where a pin exists, ``capture_ts_ns`` — the
real capture time of that ``map_version``, learned from ``pose_correction`` (exact)
or the ``status`` stream. Use ``capture_ts_ns`` when aligning the map against the
panorama and the poses; ``src_ts_ns`` tells you when the client *saw* it, which is
the number the paper's latency claims need.

ESDF slice
----------
``{server}/esdf_slice`` is a ``pack_pcd`` cloud whose ``version`` field is the
**submap index**, not the map version, and whose colours are a lossy
distance→RGB ramp. :class:`EsdfRecorder` records the wire bytes, and additionally
inverts the ramp back to metres so the slice is usable as data, documenting the
saturation honestly.

Server status
-------------
``{server}/status`` is the only stream that ties ``map_version`` to a timestamp, and
it carries the measured uplink (``robot_kbps``, ``robot_fps``,
``robot_to_server_ms``). :class:`StatusRecorder` records it verbatim, feeds the
version↔time index, and summarises the uplink for §3.2's "real uplink" figure.
"""

from __future__ import annotations

import hashlib
import io
import json
import logging
import threading
import time
from typing import Optional

import numpy as np

import rec_config as rcfg          # noqa: F401 — also puts repo/common on sys.path

import vat_blockmap as bm          # noqa: E402  (needs rec_config's path insert)
import vat_protocol as proto       # noqa: E402

from rec_base import StreamRecorder
from rec_clock import SessionClock
from rec_sinks import Budget, SessionWriter

log = logging.getLogger("vat-record")

# Mirrors nav_esdf._CLEAR_M — the distance (m) the viz ramp saturates green at.
# Kept as a literal rather than importing nav_esdf, which pulls in zenoh + the
# mapping config at module scope.
ESDF_CLEAR_M = 1.0


def esdf_rgb_to_distance_m(rgb) -> np.ndarray:
    """Invert ``nav_esdf._distance_to_rgb``: uint8 RGB → signed distance (metres).

    The forward map is ``t = clip(d/CLEAR, 0, 1)``, ``r = clip((1-t)*2, 0, 1)``,
    ``g = clip(t*2, 0, 1)``. It is one-way lossy at both ends:

    * every distance **≥ CLEAR (1 m)** saturates to ``t = 1`` → recovered as exactly
      1.0 m, and
    * every distance **≤ 0** (at or inside an obstacle) collapses to ``t = 0`` → 0.0 m,
      so how *deep* inside an obstacle a cell is cannot be recovered.

    In between, the 8-bit channel gives ~2 mm resolution. Good enough for the video
    and for a qualitative figure; not a substitute for ``engine.get_esdf_slice`` if
    you need true signed distances.
    """
    a = np.asarray(rgb)
    if a.dtype.kind == "f":                    # unpack_pcd hands back [0,1] floats
        a = np.clip(np.rint(a * 255.0), 0, 255).astype(np.uint8)
    r = a[:, 0].astype(np.float64)
    g = a[:, 1].astype(np.float64)
    t = np.where(g < 255.0, (g / 255.0) / 2.0, 1.0 - (r / 255.0) / 2.0)
    return (t * ESDF_CLEAR_M).astype(np.float32)


def _every(seconds: float) -> str:
    return f"{seconds:g}s" if seconds and seconds > 0 else "off"


def _npz_bytes(**arrays) -> bytes:
    buf = io.BytesIO()
    np.savez_compressed(buf, **arrays)
    return buf.getvalue()


# ═════════════════════════════════════════════════════════════════════════════
# Point cloud
# ═════════════════════════════════════════════════════════════════════════════


class PointCloudRecorder(StreamRecorder):
    """Record the versioned point-cloud transport and materialise map keyframes."""

    name = "pointcloud"

    def __init__(self, sw: SessionWriter, clock: SessionClock,
                 budget: Optional[Budget] = None, *,
                 cube_m: float = rcfg.CUBE_SIZE,
                 keyframe_s: float = 10.0,
                 repair: bool = True,
                 snapshot_query_s: float = 0.0,
                 push_grace_s: float = rcfg.PUSH_GRACE_S,
                 pull_timeout_s: float = 15.0):
        super().__init__(sw, clock, budget)
        self.cube_m = float(cube_m)
        self.keyframe_s = float(keyframe_s)
        self.repair = bool(repair)
        self.snapshot_query_s = float(snapshot_query_s)
        self.push_grace_s = float(push_grace_s)
        self.pull_timeout_s = float(pull_timeout_s)

        self._k = rcfg.KEYS
        self._blocks_dir = sw.subdir(self.name, "blocks")
        self._kf_dir = sw.subdir(self.name, "keyframes")
        self._snap_dir = sw.subdir(self.name, "snapshots")
        self._idx = sw.jsonl_index(self.name, "index.jsonl")

        # DracoPy is needed to *decode* pushes/bundles. Without it we still record
        # every byte (nothing is lost, replay is just deferred) but the mirror,
        # repair and keyframes are impossible — say so loudly instead of pretending.
        self.draco = bool(getattr(bm, "_HAVE_DRACO", False))
        self._store = bm.ClientBlockStore(self.cube_m) if self.draco else None
        if not self.draco:
            log.warning(f"[{self.name}] DracoPy NOT installed — recording raw wire "
                        f"bytes only: no map mirror, no manifest repair, no "
                        f"materialised keyframes. Install DracoPy (it is already a "
                        f"client dependency) for a self-contained recording.")

        self._n = {"push": 0, "manifest": 0, "repair": 0, "snapshot": 0, "keyframe": 0}
        self._nlock = threading.Lock()
        self.last_map_version = -1
        self.n_cubes = 0
        self.n_pushes_undecodable = 0
        self.n_repaired_cubes = 0
        self.n_repair_pulls = 0
        self.versions_seen = set()
        self._last_kf_mono = 0.0
        self._last_snapq_mono = 0.0

        self._remote_manifest = {}
        self._man_lock = threading.Lock()
        self._man_evt = threading.Event()
        self._stop = threading.Event()
        self._repair_thread: Optional[threading.Thread] = None
        self.stats.key = f"{rcfg.SERVER_PREFIX}/pcd/{{push,manifest,blocks}} + pcd_snapshot"

    # ── wiring ───────────────────────────────────────────────────────────────
    def attach(self, z) -> None:
        super().attach(z)
        self.subscribe(self._k["pcd_push"], self._on_push)
        self.subscribe(self._k["pcd_manifest"], self._on_manifest)
        # Also subscribed even in blocks mode: costs nothing, and it means a server
        # running STREAM_MODE=snapshot is recorded correctly without a flag.
        self.subscribe(self._k["pcd_snapshot"], self._on_snapshot)
        if self.repair and self.draco:
            self.note_query(self._k["pcd_blocks"])
            self._repair_thread = threading.Thread(
                target=self._repair_loop, name="pcd-repair", daemon=True)
            self._repair_thread.start()
            log.info(f"[{self.name}] ? '{self._k['pcd_blocks']}'  "
                     f"(manifest-diff repair, grace={self.push_grace_s}s)")
        if self.snapshot_query_s > 0:
            self.note_query(self._k["pcd_snapshot"])
        log.info(f"[{self.name}] cube={self.cube_m}m  "
                 f"keyframe_every={_every(self.keyframe_s)}  "
                 f"snapshot_query_every={_every(self.snapshot_query_s)}")

    # ── index helper ─────────────────────────────────────────────────────────
    def _record(self, kind: str, data: bytes, subdir: str, name_fn,
                map_version=None, extra: Optional[dict] = None, stamp=None,
                force: bool = False) -> Optional[dict]:
        """Write one artefact + its index row.

        ``name_fn(n)`` builds the filename from this artefact's sequence number, which
        is allocated *inside the lock* — ``_ingest_snapshot`` can be reached from both
        the subscriber thread and the main thread's snapshot query, and computing the
        number outside would let two writers pick the same filename.

        ``force`` bypasses the budget check (not the accounting) for the one artefact
        that must always be written: the final map keyframe. Without it, a run ended by
        ``--duration`` or ``--max-size`` would end on a stale map.
        """
        if self._closed:
            self.stats.skip()
            return None
        if not force and (self.budget.expired() or not self.budget.claim(len(data))):
            self.stats.skip()
            return None
        if force:
            self.budget.claim(len(data))          # count it even past the cap
        st = stamp or self.clock.stamp(None)
        with self._nlock:
            n_seq = self._n.get(kind, 0) + 1
            self._n[kind] = n_seq
        try:
            path, n = self.sw.write_blob(data, self.name, subdir, name_fn(n_seq))
        except OSError as e:
            self.budget.release(len(data))
            self.stats.error(f"write: {e}")
            return None
        rec = {
            "kind": kind,
            "src_ts_ns": st.src_ts_ns, "ts_src": st.ts_src,
            "wall_ns": st.wall_ns, "mono_ns": st.mono_ns,
            "map_version": (None if map_version is None else int(map_version)),
            "bytes": n, "file": self.sw.rel(path),
        }
        if map_version is not None:
            pin = self.clock.version_pin(int(map_version))
            rec["capture_ts_ns"] = pin["capture_ns"] if pin else None
            rec["capture_ts_src"] = pin["source"] if pin else None
        rec.update(extra or {})
        self._idx.append(rec)
        self.stats.sample(nbytes=n, src_ts_ns=st.src_ts_ns, wall_ns=st.wall_ns)
        return rec

    # ── blocks mode: proactive push ──────────────────────────────────────────
    def _on_push(self, sample) -> None:
        raw = bytes(sample.payload)
        st = self.clock.stamp(None)
        applied = removed = None
        version = None
        if self.draco:
            try:
                version, n_app, n_rem = self._store.apply_push_bytes(raw)
                applied, removed = n_app, n_rem
                self.last_map_version = int(version)
                self.versions_seen.add(int(version))
                self.n_cubes = len(self._store.blocks)
            except Exception as e:                          # noqa: BLE001
                # Record the bytes regardless — a decode failure here must not lose
                # the sample, it just defers understanding it to offline replay.
                self.n_pushes_undecodable += 1
                self.stats.error(f"push decode: {e}")
        else:
            try:                                            # header parse is Draco-free
                version = int(np.frombuffer(raw[4:8], dtype=">i4")[0])
                self.last_map_version = version
                self.versions_seen.add(version)
            except Exception:
                pass
        self._record("push", raw, "blocks", lambda n: f"push_{n:06d}.bin",
                     map_version=version, stamp=st,
                     extra={"cubes_applied": applied, "cubes_removed": removed,
                            "cubes_total_after": self.n_cubes or None,
                            "decoded": applied is not None})

    # ── blocks mode: manifest (bootstrap + repair channel) ───────────────────
    def _on_manifest(self, sample) -> None:
        raw = bytes(sample.payload)
        st = self.clock.stamp(None)
        # Parse AFTER the bytes are safe on disk. A truncated/corrupt manifest must not
        # cost us the sample — _on_push takes the same stance, and the module docstring
        # promises every push/manifest/repair bundle byte-exact.
        try:
            man = bm.unpack_manifest(raw)
        except Exception as e:                                  # noqa: BLE001
            self.stats.error(f"manifest decode: {e}")
            self._record("manifest", raw, "blocks",
                         lambda n: f"manifest_{n:06d}.bin", stamp=st,
                         extra={"decoded": False, "decode_error": str(e)[:120]})
            return
        with self._man_lock:
            self._remote_manifest = man
        self._man_evt.set()
        # An empty manifest is the server's reset signal (see BlockPublisher.reset).
        if not man and self._store is not None and self._store.blocks:
            self._store.clear()
            log.info(f"[{self.name}] empty manifest → server reset; mirror cleared")
        crc_digest = hashlib.blake2b(
            b"".join(int(k).to_bytes(8, "big", signed=True)
                     + int(c).to_bytes(4, "big") for k, c in sorted(man.items())),
            digest_size=8).hexdigest() if man else ""
        self._record("manifest", raw, "blocks", lambda n: f"manifest_{n:06d}.bin",
                     map_version=(self.last_map_version
                                  if self.last_map_version >= 0 else None),
                     stamp=st,
                     extra={"cubes": len(man), "manifest_digest": crc_digest,
                            "reset_signal": not man, "decoded": True,
                            "map_version_is_last_push": True})

    def _repair_loop(self) -> None:
        """Mirror of ``BlockSync._sync_loop``: diff the manifest, pull what's missing."""
        while not self._stop.is_set():
            if not self._man_evt.wait(timeout=1.0):
                continue
            self._man_evt.clear()
            if self.push_grace_s > 0:      # let the just-sent push land first
                if self._stop.wait(self.push_grace_s):
                    return
            with self._man_lock:
                remote = dict(self._remote_manifest)
            try:
                need, drop = bm.diff_manifest(self._store.local_manifest(), remote)
                if drop:
                    self._store.drop(drop)
                if not need:
                    continue
                self.n_repair_pulls += 1
                req = bm.pack_request(need)
                applied = 0
                for reply in self._z.get(self._k["pcd_blocks"], payload=req,
                                         timeout=self.pull_timeout_s):
                    if not reply.ok:
                        continue
                    buf = bytes(reply.result.payload)
                    if self._stop.is_set():
                        # Stopping: do NOT apply cubes we can no longer index, or the
                        # mirror (and therefore the final keyframe) would contain
                        # geometry that offline replay of the index cannot reproduce.
                        self.stats.skip()
                        break
                    applied += self._store.apply_bundle_bytes(buf)
                    self._record("repair", buf, "blocks",
                                 lambda n: f"repair_{n:06d}.bin",
                                 map_version=(self.last_map_version
                                              if self.last_map_version >= 0 else None),
                                 extra={"cubes_requested": len(need),
                                        "cubes_dropped": len(drop)})
                self.n_repaired_cubes += applied
                self.n_cubes = len(self._store.blocks)
                log.info(f"[{self.name}] repaired {applied}/{len(need)} cubes "
                         f"(mirror now {self.n_cubes} cubes)")
            except Exception as e:                          # noqa: BLE001
                self.stats.error(f"repair: {e}")
                log.debug(f"[{self.name}] repair pull failed", exc_info=True)

    # ── snapshot mode (and on-demand snapshot queries) ───────────────────────
    def _on_snapshot(self, sample) -> None:
        self._ingest_snapshot(bytes(sample.payload), origin="stream")

    def _ingest_snapshot(self, raw: bytes, origin: str) -> None:
        if len(raw) <= 24:                                  # empty/short = clear signal
            self.stats.skip()
            return
        version, xyz, rgb, is_snap, since_v = proto.unpack_pcd(raw)
        self.last_map_version = max(self.last_map_version, int(version))
        self.versions_seen.add(int(version))
        self._record("snapshot", raw, "snapshots",
                     lambda n, v=int(version): f"snapshot_{n:06d}_v{v}.bin",
                     map_version=version,
                     extra={"n_points": int(xyz.shape[0]), "is_snapshot": bool(is_snap),
                            "since_version": int(since_v), "origin": origin,
                            "wire_format": "vat_protocol.pack_pcd"})

    # ── materialised keyframes (free — no server involvement) ────────────────
    def write_keyframe(self, reason: str = "timer", *,
                       force: bool = False) -> Optional[dict]:
        """Dump the current mirror to a self-contained ``.npz`` map keyframe."""
        if self._store is None:
            return None
        # Read the blocks directly under the store's lock rather than via merged():
        # merged() consumes the store's dirty flag, and a keyframe dump must be a pure
        # observation of the mirror, not something that mutates its state.
        with self._store._lock:                              # noqa: SLF001
            vals = list(self._store.blocks.values())          # one consistent snapshot
        if not vals:
            return None
        xyz = np.concatenate([b[1] for b in vals])
        rgb = np.concatenate([b[2] for b in vals])
        if xyz.shape[0] == 0:
            return None
        mv = self.last_map_version
        pin = self.clock.version_pin(mv) if mv >= 0 else None
        data = _npz_bytes(
            points=np.ascontiguousarray(xyz, np.float32),
            colors=np.ascontiguousarray(rgb, np.uint8),
            map_version=np.int64(mv),
            capture_ts_ns=np.int64(pin["capture_ns"] if pin else 0),
            cube_m=np.float32(self.cube_m))
        rec = self._record(
            "keyframe", data, "keyframes",
            lambda n, v=mv: f"kf_{n:06d}_v{v}.npz", map_version=mv, force=force,
            extra={"n_points": int(xyz.shape[0]), "n_cubes": self.n_cubes,
                   "reason": reason, "npz_keys": ["points", "colors", "map_version",
                                                  "capture_ts_ns", "cube_m"]})
        if rec:
            self._last_kf_mono = time.monotonic()
        return rec

    def _query_snapshot(self) -> None:
        """Optional cross-check: ask the server for its canonical current surface."""
        try:
            best = None
            for reply in self._z.get(self._k["pcd_snapshot"],
                                     timeout=self.pull_timeout_s):
                if reply.ok:
                    buf = bytes(reply.result.payload)
                    if best is None or len(buf) > len(best):
                        best = buf
            if best:
                self._ingest_snapshot(best, origin="query")
        except Exception as e:                              # noqa: BLE001
            self.stats.error(f"snapshot query: {e}")

    def tick(self, now_mono: float) -> None:
        if self.budget.expired():
            return
        if self.keyframe_s > 0 and self.draco \
                and now_mono - self._last_kf_mono >= self.keyframe_s:
            self._last_kf_mono = now_mono          # set first: a slow dump must not spin
            self.write_keyframe("timer")
        if self.snapshot_query_s > 0 \
                and now_mono - self._last_snapq_mono >= self.snapshot_query_s:
            self._last_snapq_mono = now_mono
            self._query_snapshot()

    def close(self) -> None:
        self._stop.set()
        self._man_evt.set()
        if self._repair_thread is not None:
            self._repair_thread.join(timeout=3.0)
            if self._repair_thread.is_alive():
                # A pull can be blocked in a 15 s Zenoh query. It will find _closed set
                # and discard its reply rather than write past the closed index.
                log.info(f"[{self.name}] a repair pull is still in flight; its reply "
                         f"will be discarded (not applied, not indexed)")
        # A final keyframe means the recording always ends on a complete map state —
        # force=True so a --duration / --max-size stop still gets one.
        try:
            self.write_keyframe("final", force=True)
        except Exception as e:                              # noqa: BLE001
            self.stats.error(f"final keyframe: {e}")
        # Only now refuse further writes, so the final keyframe above still lands.
        super().close()

    def extra_summary(self) -> dict:
        vs = sorted(self.versions_seen)
        return {
            "index": f"{self.name}/index.jsonl",
            "counts": dict(self._n),
            "cube_m": self.cube_m,
            "draco_available": self.draco,
            "mirror_cubes_final": self.n_cubes,
            "map_versions_seen": len(vs),
            "map_version_first": vs[0] if vs else None,
            "map_version_last": vs[-1] if vs else None,
            "repair_pulls": self.n_repair_pulls,
            "repaired_cubes": self.n_repaired_cubes,
            "pushes_undecodable": self.n_pushes_undecodable,
            "keyframe_every_s": self.keyframe_s or None,
            "snapshot_query_every_s": self.snapshot_query_s or None,
            "note": ("Raw wire bytes are byte-exact; replay them with "
                     "vat_blockmap.unpack_block_push / unpack_bundle / "
                     "unpack_manifest. Keyframes are materialised from the local "
                     "mirror, so they cost the server nothing. pcd/push and "
                     "pcd/manifest are CongestionControl.DROP: losses are expected "
                     "and are healed by the manifest-diff repair pull."),
        }

    def status_line(self) -> str:
        return (f"map=v{self.last_map_version}/{self.n_cubes}cubes"
                f"(p{self._n['push']} k{self._n['keyframe']})")


# ═════════════════════════════════════════════════════════════════════════════
# ESDF slice
# ═════════════════════════════════════════════════════════════════════════════


class EsdfRecorder(StreamRecorder):
    """Record ESDF slices, keeping the wire bytes and the inverted distances."""

    name = "esdf"

    def __init__(self, sw: SessionWriter, clock: SessionClock,
                 budget: Optional[Budget] = None, *, decode: bool = True):
        super().__init__(sw, clock, budget)
        self.decode = bool(decode)
        sw.subdir(self.name, "slices")
        self._idx = sw.jsonl_index(self.name, "index.jsonl")
        self._i = 0
        self.stats.key = rcfg.KEYS["esdf_slice"]

    def attach(self, z) -> None:
        super().attach(z)
        self.subscribe(rcfg.KEYS["esdf_slice"], self._on_slice)

    def _on_slice(self, sample) -> None:
        raw = bytes(sample.payload)
        st = self.clock.stamp(None)
        # NOTE: nav_esdf packs the SUBMAP INDEX into pack_pcd's `version` field —
        # it is NOT the map_version. Recorded under its real name.
        submap_index, xyz, rgb, _snap, _since = proto.unpack_pcd(raw)
        if xyz.shape[0] == 0:
            self.stats.skip()
            return
        dist = esdf_rgb_to_distance_m(rgb) if self.decode else None
        arrays = {
            "points": np.ascontiguousarray(xyz, np.float32),
            "colors": np.ascontiguousarray(
                np.clip(np.rint(rgb * 255.0), 0, 255).astype(np.uint8)
                if rgb.dtype.kind == "f" else rgb, np.uint8),
            "submap_index": np.int64(submap_index),
        }
        if dist is not None:
            arrays["distance_m"] = dist
        data = _npz_bytes(**arrays)
        if self.budget.expired() or not self.budget.claim(len(data) + len(raw)):
            self.stats.skip()
            return
        self._i += 1
        try:
            npz_path, _ = self.sw.write_blob(
                data, self.name, "slices", f"esdf_{self._i:06d}.npz")
            wire_path, _ = self.sw.write_blob(
                raw, self.name, "slices", f"esdf_{self._i:06d}.bin")
        except OSError as e:
            self.budget.release(len(data) + len(raw))
            self.stats.error(f"write: {e}")
            return
        z_vals = np.unique(np.round(xyz[:, 2], 3))
        self._idx.append({
            "src_ts_ns": st.src_ts_ns, "ts_src": st.ts_src,
            "wall_ns": st.wall_ns, "mono_ns": st.mono_ns,
            "submap_index": int(submap_index),
            "n_cells": int(xyz.shape[0]),
            "slice_z_m": float(z_vals[0]) if z_vals.size == 1 else None,
            "distance_m_range": ([float(dist.min()), float(dist.max())]
                                 if dist is not None else None),
            "npz": self.sw.rel(npz_path), "wire": self.sw.rel(wire_path),
            "bytes": len(raw),
        })
        self.stats.sample(nbytes=len(raw), src_ts_ns=st.src_ts_ns, wall_ns=st.wall_ns)

    def extra_summary(self) -> dict:
        return {
            "index": f"{self.name}/index.jsonl",
            "decoded_distances": self.decode,
            "clear_m": ESDF_CLEAR_M,
            "note": ("pack_pcd's `version` field on this key is the SUBMAP INDEX, "
                     "not map_version. distance_m is inverted from the viz colour "
                     f"ramp: saturating at 0 m and {ESDF_CLEAR_M} m, ~2 mm resolution "
                     "in between. No source timestamp on the wire → ts_src=derived."),
        }

    def status_line(self) -> str:
        return f"esdf={self.stats.n}"


# ═════════════════════════════════════════════════════════════════════════════
# Server status  (the version ↔ time index, and the measured uplink)
# ═════════════════════════════════════════════════════════════════════════════


class StatusRecorder(StreamRecorder):
    """Record ``{server}/status`` verbatim and mine it for pins + uplink stats."""

    name = "status"

    def __init__(self, sw: SessionWriter, clock: SessionClock,
                 budget: Optional[Budget] = None):
        super().__init__(sw, clock, budget)
        sw.subdir(self.name)
        self._jsonl = sw.jsonl_index(self.name, "status.jsonl")
        self.submap_versions = set()      # distinct map_versions the server reported
        self.states = {}
        self._num = {}                    # field → [n, sum, min, max]
        self.stats.key = rcfg.KEYS["status"]

    def attach(self, z) -> None:
        super().attach(z)
        self.subscribe(rcfg.KEYS["status"], self._on_status)

    _TRACK = ("robot_kbps", "robot_fps", "robot_to_server_ms", "robot_offset_ms",
              "cloud_mbps", "submap_s", "n_points", "cubes", "cubes_changed",
              "cubes_removed", "manifest_kb", "push_kb", "frames_buffered")

    def _on_status(self, sample) -> None:
        raw = bytes(sample.payload)
        payload = json.loads(raw.decode("utf-8", "replace"))
        st = self.clock.stamp(None)
        mv = payload.get("map_version")
        # The server stamps `newest_frame_robot_ns` on the ROBOT clock — an
        # approximate but real pin from map_version to capture time. It never
        # overwrites an exact pose_correction pin (see SessionClock.pin_version).
        newest = payload.get("newest_frame_robot_ns")
        if mv is not None and newest:
            self.clock.pin_version(int(mv), int(newest), "status", st.wall_ns)
        state = str(payload.get("state", ""))
        self.states[state] = self.states.get(state, 0) + 1
        if mv is not None and state == "processing":
            self.submap_versions.add(int(mv))
        for f in self._TRACK:
            v = payload.get(f)
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                acc = self._num.setdefault(f, [0, 0.0, float(v), float(v)])
                acc[0] += 1
                acc[1] += float(v)
                acc[2] = min(acc[2], float(v))
                acc[3] = max(acc[3], float(v))
        self._jsonl.append({
            "src_ts_ns": st.src_ts_ns, "ts_src": st.ts_src,
            "wall_ns": st.wall_ns, "mono_ns": st.mono_ns,
            "status": payload,
        })
        self.stats.sample(nbytes=len(raw), src_ts_ns=st.src_ts_ns, wall_ns=st.wall_ns)

    def extra_summary(self) -> dict:
        agg = {f: {"n": a[0], "mean": round(a[1] / a[0], 4) if a[0] else None,
                   "min": round(a[2], 4), "max": round(a[3], 4)}
               for f, a in sorted(self._num.items())}
        return {
            "jsonl": f"{self.name}/status.jsonl",
            "states": dict(sorted(self.states.items())),
            "submaps_seen": len(self.submap_versions),
            "server_metrics": agg,
            "note": ("The only stream tying map_version to a timestamp "
                     "(newest_frame_robot_ns, robot clock) and the source of the "
                     "measured uplink (robot_kbps / robot_fps) for §3.2."),
        }

    def status_line(self) -> str:
        return f"status={self.stats.n}/{len(self.submap_versions)}submaps"


# ═════════════════════════════════════════════════════════════════════════════
# Self-test:  python tools/recorder/rec_cloud.py
# ═════════════════════════════════════════════════════════════════════════════

def _selftest() -> None:
    import shutil
    import tempfile

    class _S:
        def __init__(self, payload):
            self.payload = payload

    # ── ESDF colour-ramp inversion is exact where the ramp is invertible ──
    def fwd(d):                                    # nav_esdf._distance_to_rgb
        t = np.clip(np.asarray(d, np.float32) / ESDF_CLEAR_M, 0.0, 1.0)
        r = np.clip((1.0 - t) * 2.0, 0.0, 1.0)
        g = np.clip(t * 2.0, 0.0, 1.0)
        rgb = np.stack([r, g, np.zeros_like(t)], axis=1)
        return np.clip(np.rint(rgb * 255.0), 0, 255).astype(np.uint8)

    d_in = np.array([0.0, 0.05, 0.25, 0.5, 0.75, 0.999, 1.0], np.float32)
    d_out = esdf_rgb_to_distance_m(fwd(d_in))
    assert np.allclose(d_in, d_out, atol=2.5e-3), (d_in, d_out)
    # ...and saturates honestly outside [0, CLEAR]
    sat = esdf_rgb_to_distance_m(fwd(np.array([-2.0, 3.0], np.float32)))
    assert abs(sat[0] - 0.0) < 1e-6 and abs(sat[1] - ESDF_CLEAR_M) < 1e-6

    tmp = tempfile.mkdtemp(prefix="vatrec-cloud-")
    try:
        sw = SessionWriter(tmp, "s")
        clock = SessionClock()
        base = 1_700_000_000_000_000_000
        clock.stamp(base)                          # teach the clock an offset

        # ── status: pins + uplink aggregation + submap counting ──
        stat = StatusRecorder(sw, clock)
        for i in range(3):
            stat._on_status(_S(json.dumps({
                "state": "processing", "ts": 1.0, "map_version": 10 + i,
                "newest_frame_robot_ns": base + i * 10**9,
                "robot_kbps": 300.0 + i * 10, "robot_fps": 2.5,
                "n_points": 50000, "cubes": 120,
            }).encode()))
        ss = stat.summary()
        assert ss["samples"] == 3 and ss["submaps_seen"] == 3
        assert ss["server_metrics"]["robot_kbps"]["mean"] == 310.0
        assert ss["server_metrics"]["robot_kbps"]["min"] == 300.0
        assert clock.version_pin(11)["source"] == "status"
        assert clock.version_pin(11)["capture_ns"] == base + 10**9

        # ── point cloud, blocks mode ──
        pc = PointCloudRecorder(sw, clock, keyframe_s=0.0, repair=False)
        if pc.draco:
            grid = bm.BlockGrid(cube_m=1.0, crc_quant_m=0.015)
            pts = (np.floor(np.random.default_rng(0).random((3000, 3)) * 40) + 0.5) * 0.03
            pts = np.unique(pts, axis=0).astype(np.float32)
            col = np.full((pts.shape[0], 3), 128, np.uint8)
            changed, _removed = grid.ingest(pts, col)
            push = bm.pack_block_push(grid.collect(changed), [], map_version=11,
                                      cube_m=1.0)
            pc._on_push(_S(push))
            assert pc.last_map_version == 11
            assert pc.n_cubes == len(grid.blocks) and pc.n_cubes > 0
            pc._on_manifest(_S(bm.pack_manifest(grid.manifest())))

            kf = pc.write_keyframe("test")
            assert kf is not None and kf["kind"] == "keyframe"
            # the keyframe pins to the version's capture time via the status pin
            assert kf["map_version"] == 11
            assert kf["capture_ts_ns"] == base + 10**9
            with np.load(sw.path(*kf["file"].split("/"))) as z:
                assert int(z["map_version"]) == 11
                assert z["points"].shape[0] == pts.shape[0]
                assert z["points"].dtype == np.float32 and z["colors"].dtype == np.uint8
                assert int(z["capture_ts_ns"]) == base + 10**9

            # raw push bytes are byte-exact and replay through the repo's unpacker
            recs = [json.loads(l) for l in
                    open(sw.path("pointcloud", "index.jsonl")).read().splitlines()]
            pr = next(r for r in recs if r["kind"] == "push")
            assert pr["decoded"] is True and pr["map_version"] == 11
            blob = open(sw.path(*pr["file"].split("/")), "rb").read()
            assert blob == push
            mv2, rem2, got2 = bm.unpack_block_push(blob, cube_m=1.0)
            assert mv2 == 11 and rem2 == [] and len(got2) == pc.n_cubes

            mr = next(r for r in recs if r["kind"] == "manifest")
            assert mr["cubes"] == len(grid.manifest()) and not mr["reset_signal"]

            # an empty manifest is the reset signal and clears the mirror
            pc._on_manifest(_S(bm.pack_manifest({})))
            assert len(pc._store.blocks) == 0

            # A corrupt manifest must still cost us nothing: bytes on disk, error
            # counted, recording continues (same stance as a bad push).
            n_before = pc._n["manifest"]
            pc._on_manifest(_S(b"\x00\x00\x00\x01truncated"))
            assert pc._n["manifest"] == n_before + 1
            bad = [json.loads(l) for l in
                   open(sw.path("pointcloud", "index.jsonl")).read().splitlines()
                   if json.loads(l)["kind"] == "manifest"][-1]
            assert bad["decoded"] is False and bad["decode_error"]
            assert open(sw.path(*bad["file"].split("/")), "rb").read() \
                == b"\x00\x00\x00\x01truncated"
            assert pc.summary()["errors"] == 1

            # The FINAL keyframe must land even when a cap already ended the session,
            # or a --duration run would finish on a stale map.
            capped = PointCloudRecorder(SessionWriter(tmp, "capped"), clock,
                                        Budget(max_bytes=1, name="tiny"),
                                        cube_m=1.0, keyframe_s=0.0, repair=False)
            capped._store.apply_push_bytes(push)
            capped.last_map_version = 11
            assert capped.budget.bytes_exhausted() or capped.write_keyframe("t") is None
            assert capped.write_keyframe("timer") is None      # refused, as designed
            assert capped.write_keyframe("final", force=True) is not None
            capped.close()
            assert capped.extra_summary()["counts"]["keyframe"] >= 1

            # Two snapshots of the same map_version must not collide on a filename
            # (_ingest_snapshot is reachable from both the subscriber and the query).
            sw3 = SessionWriter(tmp, "snapnames")
            pc3 = PointCloudRecorder(sw3, clock, cube_m=1.0, keyframe_s=0.0,
                                     repair=False)
            wire_a = proto.pack_pcd(5, np.zeros((2, 3), np.float32),
                                    np.zeros((2, 3), np.float32), is_snapshot=True)
            pc3._ingest_snapshot(wire_a, "stream")
            pc3._ingest_snapshot(wire_a, "query")
            snaps = [json.loads(l)["file"] for l in
                     open(sw3.path("pointcloud", "index.jsonl")).read().splitlines()]
            assert len(set(snaps)) == 2, snaps
            pc3.close()
        else:
            log.warning("DracoPy missing — block-path assertions skipped")

        # ── point cloud, snapshot mode ──
        xyz = np.array([[0, 0, 0], [1, 1, 1], [2, 0, 1]], np.float32)
        rgb = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], np.float32)
        wire = proto.pack_pcd(42, xyz, rgb, is_snapshot=True)
        pc._on_snapshot(_S(wire))
        srec = [json.loads(l) for l in
                open(sw.path("pointcloud", "index.jsonl")).read().splitlines()
                if json.loads(l)["kind"] == "snapshot"]
        assert len(srec) == 1 and srec[0]["n_points"] == 3
        assert srec[0]["map_version"] == 42 and srec[0]["origin"] == "stream"
        assert open(sw.path(*srec[0]["file"].split("/")), "rb").read() == wire
        errs_before = pc.summary()["errors"]
        pc._on_snapshot(_S(b"\x00" * 12))          # too short → skipped, not an error
        assert pc.summary()["errors"] == errs_before

        # ── ESDF ──
        es = EsdfRecorder(sw, clock)
        cells = np.array([[0, 0, 0.5], [1, 0, 0.5], [2, 0, 0.5]], np.float32)
        es._on_slice(_S(proto.pack_pcd(7, cells, fwd(np.array([0.0, 0.5, 1.0])),
                                       is_snapshot=True)))
        er = json.loads(open(sw.path("esdf", "index.jsonl")).readline())
        assert er["submap_index"] == 7 and er["n_cells"] == 3
        assert abs(er["slice_z_m"] - 0.5) < 1e-6
        with np.load(sw.path(*er["npz"].split("/"))) as z:
            assert np.allclose(z["distance_m"], [0.0, 0.5, 1.0], atol=2.5e-3)

        pc.close()
        sw.close()
        print(f"rec_cloud self-test OK  (draco={pc.draco}, byte-exact pushes replay, "
              f"free keyframes, status pins, ESDF ramp inverted)")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    _selftest()
