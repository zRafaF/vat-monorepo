#!/usr/bin/env python3
"""
VAT — ``compose.py``: align and compose a recorded session
=========================================================
Companion to ``vat_record.py``. Takes a ``recordings/<session_id>/`` and puts every
stream on **one timeline**, so the final video's layout is a decision you make in an
editor later rather than one baked into the recorder.

Four commands
-------------
``info``
    What is in the recording, how healthy it is, and the window in which every
    stream has data.

``export`` — *the main one*
    Build a timeline (uniform ``--fps``, or one tick per panorama frame) and, for
    each tick, resolve every stream to the sample that belongs at that instant:

    * panorama frame (transmit and/or full-res) — nearest, with the error reported;
    * robot pose — **interpolated** (LERP position, SLERP orientation) to the exact
      tick, because the pose stream is ~30 Hz and interpolating is strictly better
      than snapping;
    * map state — either the nearest materialised keyframe, or (``--map replay``) the
      exact state produced by replaying the recorded Draco pushes up to that instant;
    * periscope frame — its ``(segment, byte_offset, byte_len)`` so you can slice the
      exact encoded frame out of the elementary stream;
    * ESDF slice and camera trail — nearest.

    Writes ``timeline.csv`` + ``timeline.jsonl`` + a ``README.md``, and optionally a
    numbered ``frames/`` directory of hard links so ``ffmpeg -i frames/%06d.jpg``
    just works. **No rendering, no heavy dependencies** — this is the path into any
    editor.

``periscope``
    Slice the encoded periscope frames back out of the elementary stream (and
    optionally decode them to PNG with the viewer's own decoder).

``render`` — *optional*
    Offscreen-render the point cloud + trajectory with Open3D, composite the
    panorama and periscope panels, and mux to mp4 with ffmpeg. Modular on purpose:
    ``--layout`` picks the arrangement, and everything degrades to a clear message
    if Open3D / PyAV / ffmpeg is missing.

Every timestamp here is the session clock (robot capture ns) written by the
recorder; see ``docs/recording.md`` for the clock contract.

Examples
--------
::

    python tools/recorder/compose.py info    recordings/20260808-201500_lab_loop_p1
    python tools/recorder/compose.py export  recordings/<id> --fps 10 --link hard
    python tools/recorder/compose.py export  recordings/<id> --at panorama --map replay
    python tools/recorder/compose.py render  recordings/<id> --out demo.mp4 --fps 15
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import logging
import os
import shutil
import subprocess
import sys
import tempfile
from typing import Dict, List, Optional, Tuple

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import rec_config as rcfg          # noqa: E402  (also puts repo/common on sys.path)

import vat_blockmap as bm          # noqa: E402
import vat_protocol as proto       # noqa: E402

log = logging.getLogger("vat-compose")

#: every stream list a Recording holds, by attribute name (used for merging + tagging)
_STREAM_ATTRS = ("panorama_transmit", "panorama_fullres", "periscope", "pointcloud",
                 "esdf", "status", "fused", "corrections", "trajectory")

#: identity of a sample, for de-duplicating a stream recorded on BOTH sides of a paired
#: capture. In a robot+cloud pair the transmit panorama and the fused pose are seen by
#: both recorders, and simply concatenating them would double every rate in the report
#: and every asset in the timeline.
_MERGE_KEY = {
    "panorama_transmit": ("seq",),
    "panorama_fullres": ("seq",),
    "periscope": ("seq", "src_ts_ns"),
    "fused": ("seq", "src_ts_ns"),
    "corrections": ("src_ts_ns", "map_version"),
}
#: streams with no reliable per-sample identity. The map transport in particular must
#: come from ONE session: replaying two interleaved copies of pushes and manifest
#: removals would corrupt the reconstructed map.
_MERGE_SINGLE = ("pointcloud", "esdf", "status", "trajectory")

MAP_MODES = ("keyframe", "replay", "none")
LINK_MODES = ("none", "hard", "copy")
LAYOUTS = ("cloud", "cloud+panorama", "quad")


# ═════════════════════════════════════════════════════════════════════════════
# Loading
# ═════════════════════════════════════════════════════════════════════════════


class Recording:
    """A loaded session: parsed metadata plus every stream index, in memory.

    Indexes are small (a few thousand rows for a multi-minute capture) — the bulk
    of a recording is blobs, which stay on disk and are referenced by path.
    """

    def __init__(self, root: str, extra_roots=()):
        self.root = os.path.abspath(root)
        self.roots = [self.root]
        if not os.path.isdir(self.root):
            raise SystemExit(f"not a directory: {self.root}")
        self.meta = _read_json(os.path.join(self.root, "meta.json")) or {}
        self.manifest = _read_json(os.path.join(self.root, "MANIFEST.json")) or {}
        if not self.meta and not self.manifest:
            raise SystemExit(f"{self.root} has no meta.json / MANIFEST.json — "
                             f"is it a vat-record session?")

        self.panorama_transmit = _read_csv(self.p("panorama_transmit",
                                                  "frame_index.csv"))
        self.panorama_fullres = _read_csv(self.p("panorama_fullres",
                                                 "frame_index.csv"))
        self.periscope = _read_csv(self.p("periscope_timestamps.csv"),
                                   required=("segment", "byte_offset", "byte_len",
                                             "src_ts_ns"))
        self.pointcloud = _read_jsonl(self.p("pointcloud", "index.jsonl"))
        self.esdf = _read_jsonl(self.p("esdf", "index.jsonl"))
        self.status = _read_jsonl(self.p("status", "status.jsonl"))
        self.fused = _read_jsonl(self.p("poses", "robot_fused.jsonl"))
        self.corrections = _read_jsonl(self.p("poses", "cloud_correction.jsonl"))
        self.trajectory = _read_jsonl(self.p("poses", "trajectory.jsonl"))

        for rows in (self.panorama_transmit, self.panorama_fullres, self.periscope):
            _coerce_numeric(rows)
        # Tag every row with the session it came from, so a merged recording can still
        # resolve each blob against the right root (see `fp`).
        for attr in _STREAM_ATTRS:
            for r in getattr(self, attr):
                r["_root"] = self.root

        # A paired capture: the robot-side recorder holds the full-res panorama, the
        # cloud-side one holds the map. They share the session clock (both stamp from the
        # robot capture clock), so absorbing one into the other is just a per-stream
        # union — no resampling, no clock translation.
        for extra in extra_roots or ():
            self.absorb(Recording(extra))

        # map_version → capture time, complete for the whole session. The index rows
        # carry the pin they knew about *at write time*, but the first artefacts of a
        # submap are published before its pose_correction/status arrive, so those rows
        # have no pin. MANIFEST.json holds the final table — use it to backfill.
        self.version_pins = dict(getattr(self, "version_pins_extra", {}) or {})
        for k, v in (self.manifest.get("version_pins") or {}).items():
            try:
                self.version_pins[int(k)] = v
            except (TypeError, ValueError):
                pass

        # Map records: the *effective* time of a map state is its capture pin when we
        # have one (the real keyframe capture time), and its arrival otherwise.
        #
        # The running-max clamp exists because pins can go BACKWARDS across a reset
        # batch boundary (a rebuilt batch tiles oldest-window-first), and the map must
        # only ever move forward. It is applied ONLY across pinned records: a capture
        # time and a derived arrival time are different quantities — an arrival (which
        # is capture + pipeline latency, ~1 s later) would otherwise poison the running
        # max and drag every subsequent pinned record forward with it, collapsing the
        # first second of map evolution onto one timestamp.
        hi = 0
        self.n_unpinned = 0
        for r in self.pointcloud:
            if not r.get("capture_ts_ns") and r.get("map_version") is not None:
                pin = self.version_pins.get(int(r["map_version"]))
                if pin:
                    r["capture_ts_ns"] = pin.get("capture_ns")
                    r["capture_ts_src"] = pin.get("source")
            cap = r.get("capture_ts_ns")
            if cap:
                hi = max(hi, int(cap))
                r["_t"] = hi
                r["_t_src"] = "capture"
            else:
                # No version, or a version nothing ever pinned (e.g. a manifest that
                # arrived before this session's first push). Its own arrival time is
                # the best available and must not affect the pinned chain.
                r["_t"] = int(r.get("src_ts_ns") or 0)
                r["_t_src"] = "arrival"
                self.n_unpinned += 1
        self.keyframes = [r for r in self.pointcloud if r.get("kind") == "keyframe"]
        self.snapshots = [r for r in self.pointcloud if r.get("kind") == "snapshot"]

        # Blobs the recorder's ring budget may have evicted after indexing them: drop
        # those rows so `nearest()` can never hand back a dead path (see finding on
        # RingBudget in tools/recorder/README.md → "Known limits").
        self.missing_blobs = {}
        for attr in ("panorama_transmit", "panorama_fullres"):
            rows = getattr(self, attr)
            keep = [r for r in rows if not r.get("file") or os.path.exists(self.fp(r))]
            if len(keep) != len(rows):
                self.missing_blobs[attr] = len(rows) - len(keep)
                log.warning(f"[{attr}] {len(rows) - len(keep)} of {len(rows)} indexed "
                            f"frames are missing on disk (ring eviction or a truncated "
                            f"copy) — ignoring those rows")
                setattr(self, attr, keep)

    # ── merging a paired capture ─────────────────────────────────────────────
    def absorb(self, other: "Recording") -> None:
        """Union another session's streams into this one.

        Built for the two-recorder pattern: `--where robot` deliberately does not record
        the map (the router is on the server, so it would cross the field link inbound),
        so a full capture is a robot-side session plus a cloud-side one. Both stamp on
        the same session clock, so each stream is simply taken from whichever session
        actually has it; when BOTH have rows they are concatenated and re-sorted by
        timestamp, which is the right answer for a stream that was recorded on both
        sides. Every row keeps its own `_root`, so blobs still resolve.
        """
        self.roots.append(other.root)
        for attr in _STREAM_ATTRS:
            mine, theirs = getattr(self, attr), getattr(other, attr)
            if not theirs:
                continue
            if not mine:
                setattr(self, attr, list(theirs))
                log.info(f"[merge] {attr}: {len(theirs)} row(s) from "
                         f"{os.path.basename(other.root)}")
            elif attr in _MERGE_SINGLE:
                log.warning(
                    f"[merge] {attr}: both sessions recorded it; keeping "
                    f"{os.path.basename(self.root)}'s {len(mine)} row(s) and ignoring "
                    f"{os.path.basename(other.root)}'s {len(theirs)}. This stream has no "
                    f"per-sample identity, and interleaving two copies of the map "
                    f"transport would corrupt a replay.")
            else:
                keys = _MERGE_KEY[attr]
                seen, merged_rows, dupes = {}, [], 0
                for r in list(mine) + list(theirs):
                    k = tuple(r.get(c) for c in keys)
                    prev = seen.get(k)
                    if prev is None:
                        seen[k] = r
                        merged_rows.append(r)
                        continue
                    dupes += 1
                    # Keep whichever copy actually has its blob on disk.
                    if not prev.get("file") and r.get("file"):
                        merged_rows[merged_rows.index(prev)] = r
                        seen[k] = r
                    elif prev.get("file") and not os.path.exists(self.fp(prev)) \
                            and r.get("file") and os.path.exists(other.fp(r)):
                        merged_rows[merged_rows.index(prev)] = r
                        seen[k] = r
                setattr(self, attr, merged_rows)
                log.info(f"[merge] {attr}: {len(mine)} + {len(theirs)} → "
                         f"{len(merged_rows)} rows ({dupes} duplicate(s) collapsed on "
                         f"{'+'.join(keys)})")
        self.version_pins_extra = dict(getattr(other, "version_pins", {}) or {})
        for k, v in (other.manifest.get("version_pins") or {}).items():
            try:
                self.version_pins_extra[int(k)] = v
            except (TypeError, ValueError):
                pass
        self._ts_cache = {}

    @property
    def merged(self) -> bool:
        return len(self.roots) > 1

    # ── paths ────────────────────────────────────────────────────────────────
    def p(self, *parts: str) -> str:
        return os.path.join(self.root, *parts)

    def fp(self, row: dict, key: str = "file") -> str:
        """Absolute path of a row's blob, resolved against ITS OWN session root."""
        rel = row.get(key)
        if not rel:
            return ""
        return os.path.join(row.get("_root") or self.root, *str(rel).split("/"))

    def rel(self, *parts: str) -> str:
        return "/".join(parts)

    @property
    def session_id(self) -> str:
        return (self.meta.get("session_id") or self.manifest.get("session_id")
                or os.path.basename(self.root))

    # ── the composable window ────────────────────────────────────────────────
    #: streams whose coverage is continuous enough to bound the composable window.
    #: `panorama_fullres` is deliberately absent — it is decimated, ring-evicted and
    #: can start late, so it must not truncate the timeline. Nor is the gated
    #: `corrections` stream, which legitimately goes quiet.
    DENSE = ("panorama_transmit", "periscope", "fused", "pointcloud")

    def spans(self) -> Dict[str, Tuple[int, int]]:
        """``{stream: (first_ns, last_ns)}`` for every stream that has samples."""
        out = {}
        for name, attr, _path in _STREAM_INDEXES:
            ts = self.ts(attr)
            if ts.size:
                out[name] = (int(ts[0]), int(ts[-1]))
        return out

    def window(self, mode: str = "aligned") -> Tuple[Optional[int], Optional[int]]:
        """``(start_ns, end_ns)`` for the timeline.

        ``aligned`` (default) is the intersection over the dense streams — the span in
        which everything is available. ``full`` is the union, for when you would rather
        have the whole capture and accept that some streams are missing at the edges.

        Prefers the recorder's own ``derived.aligned_window``: it saw the streams live,
        including any that produced no index file at all.
        """
        dense = [attr for attr in self.DENSE if self.ts(attr).size]
        if not dense and self.ts("panorama_fullres").size:
            dense = ["panorama_fullres"]          # a robot-side full-res-only recording
        if mode == "full":
            firsts = [int(self.ts(a)[0]) for a in dense]
            lasts = [int(self.ts(a)[-1]) for a in dense]
            return (min(firsts), max(lasts)) if firsts else (None, None)
        aw = (self.manifest.get("derived") or {}).get("aligned_window") or {}
        if aw.get("start_src_ts_ns") and aw.get("end_src_ts_ns"):
            return int(aw["start_src_ts_ns"]), int(aw["end_src_ts_ns"])
        if not dense:
            return None, None
        return (max(int(self.ts(a)[0]) for a in dense),
                min(int(self.ts(a)[-1]) for a in dense))

    # ── timestamp arrays (sorted, for searchsorted) ──────────────────────────
    def ts(self, stream: str) -> np.ndarray:
        cache = getattr(self, "_ts_cache", None)
        if cache is None:
            cache = self._ts_cache = {}
        if stream in cache:
            return cache[stream]
        rows = self.rows(stream)
        key = "_t" if stream in ("pointcloud", "keyframes", "snapshots") else "src_ts_ns"
        vals = np.array([int(r.get(key) or 0) for r in rows], dtype=np.int64)
        order = np.argsort(vals, kind="stable")
        cache[stream] = vals[order]
        cache[stream + "__order"] = order
        return cache[stream]

    def order(self, stream: str) -> np.ndarray:
        self.ts(stream)
        return self._ts_cache[stream + "__order"]

    def rows(self, stream: str) -> List[dict]:
        return getattr(self, stream, []) or []

    def nearest(self, stream: str, t_ns: int) -> Optional[dict]:
        """The row whose timestamp is closest to ``t_ns`` (None if the stream is empty)."""
        ts = self.ts(stream)
        if ts.size == 0:
            return None
        i = int(np.searchsorted(ts, t_ns))
        if i == 0:
            j = 0
        elif i >= ts.size:
            j = ts.size - 1
        else:
            j = i if abs(int(ts[i]) - t_ns) < abs(t_ns - int(ts[i - 1])) else i - 1
        row = dict(self.rows(stream)[int(self.order(stream)[j])])
        row["_dt_ms"] = (int(ts[j]) - t_ns) / 1e6
        return row

    def at_or_before(self, stream: str, t_ns: int) -> Optional[dict]:
        """The newest row at or before ``t_ns`` — the right semantics for map state."""
        ts = self.ts(stream)
        if ts.size == 0:
            return None
        i = int(np.searchsorted(ts, t_ns, side="right")) - 1
        if i < 0:
            return None
        row = dict(self.rows(stream)[int(self.order(stream)[i])])
        row["_dt_ms"] = (int(ts[i]) - t_ns) / 1e6
        return row


def load(root: str, extra_roots=()) -> Recording:
    """Load one session, optionally merging partner sessions from the same capture."""
    return Recording(root, extra_roots=extra_roots)


def _read_json(path: str) -> Optional[dict]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _read_jsonl(path: str) -> List[dict]:
    out = []
    try:
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        out.append(json.loads(line))
                    except ValueError:
                        pass                    # a torn last line from a hard kill
    except OSError:
        pass
    return out


def _read_csv(path: str, required=()) -> List[dict]:
    """Read a CSV index, dropping any row torn by a hard kill.

    The recorder flushes every row, but a SIGKILL or power loss mid-``writerow`` can
    still leave a short final line. ``DictReader`` fills the missing fields with None,
    which then blows up in whatever converts them — so drop such rows here, once, and
    say so, exactly as ``_read_jsonl`` does for a torn last line.
    """
    try:
        with open(path, "r", newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
    except OSError:
        return []
    keep = [r for r in rows
            if None not in r.values() and None not in r.keys()
            and all(r.get(k) not in (None, "") for k in required)]
    if len(keep) != len(rows):
        log.warning(f"[{os.path.basename(path)}] dropped {len(rows) - len(keep)} "
                    f"incomplete row(s) — the recording was probably killed hard")
    return keep


_NUMERIC = ("seq", "src_ts_ns", "wall_ns", "mono_ns", "wire_bytes", "image_bytes",
            "width", "height", "native_w", "bytes", "byte_offset", "byte_len",
            "keyframe", "optical")
_FLOAT = ("camera_height_m", "latency_ms", "yaw_deg", "pitch_deg", "hfov_deg",
          "vfov_deg")


def _coerce_numeric(rows: List[dict]) -> None:
    for r in rows:
        for k in _NUMERIC:
            if k in r and r[k] not in ("", None):
                try:
                    r[k] = int(r[k])
                except (TypeError, ValueError):
                    pass
        for k in _FLOAT:
            if k in r and r[k] not in ("", None):
                try:
                    r[k] = float(r[k])
                except (TypeError, ValueError):
                    pass


# ═════════════════════════════════════════════════════════════════════════════
# info
# ═════════════════════════════════════════════════════════════════════════════


# (reported name, Recording attribute, index path). The reported names match the
# stream names in the recorder's MANIFEST.json so the two can be read side by side.
_STREAM_INDEXES = (
    ("panorama_transmit", "panorama_transmit", "panorama_transmit/frame_index.csv"),
    ("panorama_fullres", "panorama_fullres", "panorama_fullres/frame_index.csv"),
    ("periscope", "periscope", "periscope_timestamps.csv"),
    ("pointcloud", "pointcloud", "pointcloud/index.jsonl"),
    ("esdf", "esdf", "esdf/index.jsonl"),
    ("status", "status", "status/status.jsonl"),
    ("poses_robot_fused", "fused", "poses/robot_fused.jsonl"),
    ("poses_cloud_correction", "corrections", "poses/cloud_correction.jsonl"),
    ("poses_trajectory", "trajectory", "poses/trajectory.jsonl"),
)


def info(rep: Recording) -> dict:
    lo, hi = rep.window()
    streams = {}
    for name, attr, path in _STREAM_INDEXES:
        rows = rep.rows(attr)
        ts = rep.ts(attr)
        entry = {
            "index": path, "rows": len(rows),
            "first_src_ts_ns": int(ts[0]) if ts.size else None,
            "last_src_ts_ns": int(ts[-1]) if ts.size else None,
            "span_s": (round((int(ts[-1]) - int(ts[0])) / 1e9, 3)
                       if ts.size > 1 else None),
        }
        if entry["span_s"] and len(rows) > 1:
            # (n-1)/span, matching rec_sinks.StreamStats — N samples span N-1 intervals.
            entry["mean_hz"] = round((len(rows) - 1) / entry["span_s"], 3)
        if name == "pointcloud":
            entry["keyframes"] = len(rep.keyframes)
            entry["snapshots"] = len(rep.snapshots)
            entry["pushes"] = sum(1 for r in rows if r.get("kind") == "push")
            entry["manifests"] = sum(1 for r in rows if r.get("kind") == "manifest")
            entry["repairs"] = sum(1 for r in rows if r.get("kind") == "repair")
        streams[name] = entry
    # per the CSV/JSONL, how many samples carry a REAL capture timestamp
    derived_ts = {}
    for name, attr in (("pointcloud", "pointcloud"), ("esdf", "esdf"),
                       ("status", "status"), ("poses_trajectory", "trajectory")):
        rows = rep.rows(attr)
        if rows:
            derived_ts[name] = sum(1 for r in rows
                                   if r.get("ts_src") != "source") / len(rows)
    return {
        "session_id": rep.session_id,
        "root": rep.root,
        "merged_roots": rep.roots if rep.merged else None,
        "status": rep.manifest.get("status") or rep.meta.get("status"),
        "stop_reason": rep.manifest.get("stop_reason"),
        "capture": rep.meta.get("capture", {}),
        "config_hash": (rep.meta.get("config", {}) or {}).get("mapping_config_hash"),
        "window": {"start_src_ts_ns": lo, "end_src_ts_ns": hi,
                   "duration_s": round((hi - lo) / 1e9, 3) if lo and hi else None},
        "streams": streams,
        "fraction_derived_timestamps": derived_ts,
        "derived": rep.manifest.get("derived", {}),
    }


def print_info(rep: Recording) -> None:
    i = info(rep)
    cap = i["capture"] or {}
    print(f"\nVAT recording — {i['session_id']}")
    print("=" * 78)
    print(f"  root          {i['root']}")
    if i.get("merged_roots"):
        for extra in i["merged_roots"][1:]:
            print(f"  merged with   {extra}")
    print(f"  status        {i['status']}  ({i['stop_reason']})")
    print(f"  scene         {cap.get('scene')}   family={cap.get('trajectory_family')}"
          f"   pass={cap.get('pass_index')}   seed={cap.get('seed')}")
    print(f"  camera height {cap.get('camera_height_m')} m "
          f"({cap.get('camera_height_source') or 'source not recorded'})")
    print(f"  flat floor    {cap.get('start_over_clear_flat_floor')}"
          f"   operator={cap.get('operator')}")
    print(f"  config hash   {i['config_hash']}")
    w = i["window"]
    print(f"  window        {w['duration_s']} s  "
          f"[{w['start_src_ts_ns']} .. {w['end_src_ts_ns']}] (session clock)")
    print(f"\n  {'stream':<24}{'rows':>8}{'span s':>10}{'Hz':>9}   extra")
    print("  " + "-" * 74)
    for name, e in i["streams"].items():
        if not e["rows"]:
            continue
        extra = ""
        if name == "pointcloud":
            extra = (f"kf={e['keyframes']} push={e['pushes']} "
                     f"man={e['manifests']} rep={e['repairs']} snap={e['snapshots']}")
        print(f"  {name:<24}{e['rows']:>8}{(e['span_s'] or 0):>10.1f}"
              f"{(e.get('mean_hz') or 0):>9.2f}   {extra}")
    empty = [n for n, e in i["streams"].items() if not e["rows"]]
    if empty:
        print(f"\n  not recorded: {', '.join(empty)}")
    d = i["derived"] or {}
    if d.get("uplink"):
        u = d["uplink"]
        print(f"\n  uplink        {u.get('mean_wire_bytes_per_frame')} B/frame  "
              f"{u.get('mean_hz')} Hz  {u.get('mean_kbps')} kB/s"
              f"   (server said {(u.get('server_reported_robot_kbps') or {}).get('mean')} kB/s)")
    if d.get("pose_correction_gating"):
        g = d["pose_correction_gating"]
        print(f"  corrections   {g['corrections_published']} published / "
              f"{g['submaps_seen_in_status']} submaps  "
              f"({g['suppressed_or_rejected']} suppressed or rejected)")
    fd = i["fraction_derived_timestamps"]
    if fd:
        print("\n  derived (non-source) timestamps: "
              + ", ".join(f"{k} {v*100:.0f}%" for k, v in fd.items())
              + "\n  (map/ESDF/status/trajectory carry no wire timestamp — align maps "
                "on capture_ts_ns)")
    print()


# ═════════════════════════════════════════════════════════════════════════════
# Map replay
# ═════════════════════════════════════════════════════════════════════════════


def _map_apply(store: bm.ClientBlockStore, rep: Recording, rec: dict) -> bool:
    """Apply one recorded map artefact to ``store``. True if it changed anything."""
    kind = rec.get("kind")
    path = rec.get("file")
    if not path:
        return False
    full = rep.fp(rec)
    try:
        if kind == "push":
            with open(full, "rb") as f:
                store.apply_push_bytes(f.read())
            return True
        if kind == "repair":
            with open(full, "rb") as f:
                store.apply_bundle_bytes(f.read())
            return True
        if kind == "manifest":
            # A manifest cannot ADD geometry, but an EMPTY one is the server's reset
            # signal and must clear the map or the replay accumulates a stale world.
            with open(full, "rb") as f:
                man = bm.unpack_manifest(f.read())
            if not man:
                store.clear()
                return True
            drop = [k for k in store.local_manifest() if k not in man]
            if drop:
                store.drop(drop)
                return True
            return False
    except Exception as e:                                      # noqa: BLE001
        log.warning(f"[replay] {kind} {path}: {e}")
    return False


def replay_map(rep: Recording, until_ts_ns: Optional[int] = None
               ) -> Tuple[np.ndarray, np.ndarray, int]:
    """Rebuild the exact map state at ``until_ts_ns`` from the recorded wire bytes.

    Applies pushes, repair bundles and manifest removals in recorded order using
    ``vat_blockmap.ClientBlockStore`` — the same code the live client runs, so the
    replayed map is what the operator saw, not a re-derivation.

    Returns ``(xyz float32, rgb uint8, map_version)``.
    """
    if not getattr(bm, "_HAVE_DRACO", False):
        raise SystemExit("replaying the map needs DracoPy (a client dependency):\n"
                         "  cd client && uv sync      # or: uv pip install DracoPy")
    cube_m = float((rep.meta.get("zenoh") or {}).get("cube_size_m") or rcfg.CUBE_SIZE)
    store = bm.ClientBlockStore(cube_m)
    version = -1
    for rec in rep.pointcloud:
        if until_ts_ns is not None and rec["_t"] > until_ts_ns:
            break
        if rec.get("kind") == "snapshot":
            # A whole-map snapshot supersedes the block state entirely.
            try:
                with open(rep.fp(rec), "rb") as f:
                    v, xyz, rgb, _snap, _since = proto.unpack_pcd(f.read())
                store.clear()
                store.apply_bundle_bytes(bm.pack_bundle(
                    _blocks_from_cloud(xyz, rgb, cube_m), cube_m))
                version = int(v)
            except Exception as e:                              # noqa: BLE001
                log.warning(f"[replay] snapshot {rec.get('file')}: {e}")
            continue
        if _map_apply(store, rep, rec) and rec.get("map_version") is not None:
            version = int(rec["map_version"])
    merged = store.merged()
    if merged is None:
        return (np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8), version)
    xyz, rgb = merged
    return xyz, rgb, version


def _blocks_from_cloud(xyz, rgb, cube_m: float):
    """Bin a whole cloud into ``(key, crc, xyz, rgb)`` blocks for the client store."""
    xyz = np.ascontiguousarray(xyz, np.float32).reshape(-1, 3)
    rgb = np.asarray(rgb).reshape(-1, 3)
    rgb8 = ((np.clip(rgb, 0, 1) * 255 + 0.5).astype(np.uint8)
            if rgb.dtype.kind == "f" else rgb.astype(np.uint8))
    keys = bm.cube_keys(xyz, cube_m)
    order = np.argsort(keys, kind="stable")
    ks, xs, cs = keys[order], xyz[order], rgb8[order]
    uniq, starts = np.unique(ks, return_index=True)
    ends = np.append(starts[1:], len(ks))
    return [(int(k), 0, np.ascontiguousarray(xs[s:e]), np.ascontiguousarray(cs[s:e]))
            for k, s, e in zip(uniq.tolist(), starts.tolist(), ends.tolist())]


def load_map_npz(rep: Recording, rec: dict) -> Tuple[np.ndarray, np.ndarray, int]:
    """Load a materialised keyframe ``.npz`` → ``(xyz, rgb uint8, map_version)``."""
    with np.load(rep.fp(rec)) as z:
        return (np.asarray(z["points"], np.float32),
                np.asarray(z["colors"], np.uint8), int(z["map_version"]))


# ═════════════════════════════════════════════════════════════════════════════
# Pose interpolation
# ═════════════════════════════════════════════════════════════════════════════


def interp_pose(rep: Recording, t_ns: int, max_gap_s: float = 0.5) -> Optional[dict]:
    """Interpolate the fused pose to exactly ``t_ns``.

    LERP on position, SLERP on orientation (``vat_protocol.quat_slerp`` — the same
    routine the live client's predictor uses). Falls back to the nearest sample when
    the tick is outside the recorded span, and returns ``None`` if the nearest sample
    is further than ``max_gap_s`` away (a real dropout, which must not be papered
    over with a stale pose).
    """
    ts = rep.ts("fused")
    if ts.size == 0:
        return None
    rows, order = rep.rows("fused"), rep.order("fused")
    i = int(np.searchsorted(ts, t_ns))
    if i <= 0 or i >= ts.size:
        j = 0 if i <= 0 else ts.size - 1
        if abs(int(ts[j]) - t_ns) > max_gap_s * 1e9:
            return None
        r = dict(rows[int(order[j])])
        r["_dt_ms"] = (int(ts[j]) - t_ns) / 1e6
        r["_interpolated"] = False
        return r
    a, b = rows[int(order[i - 1])], rows[int(order[i])]
    ta, tb = int(ts[i - 1]), int(ts[i])
    if (tb - ta) > max_gap_s * 1e9:
        return None
    u = 0.0 if tb == ta else (t_ns - ta) / (tb - ta)
    pa = np.asarray(a["position"], np.float64)
    pb = np.asarray(b["position"], np.float64)
    qa = np.asarray(a["quaternion"], np.float64)
    qb = np.asarray(b["quaternion"], np.float64)
    va = np.asarray(a.get("linear_velocity", [0, 0, 0]), np.float64)
    vb = np.asarray(b.get("linear_velocity", [0, 0, 0]), np.float64)
    return {
        "position": (pa + (pb - pa) * u).tolist(),
        "quaternion": proto.quat_slerp(qa, qb, float(u)).tolist(),
        "linear_velocity": (va + (vb - va) * u).tolist(),
        "fix_quality": int(b.get("fix_quality", 0)),
        "fix": b.get("fix"),
        "seq": b.get("seq"),
        "_dt_ms": 0.0,
        "_interpolated": True,
        "_u": round(float(u), 6),
        "_bracket_ns": [ta, tb],
    }


# ═════════════════════════════════════════════════════════════════════════════
# export
# ═════════════════════════════════════════════════════════════════════════════

TIMELINE_COLUMNS = [
    "tick", "src_ts_ns", "t_rel_s",
    "panorama_transmit_file", "panorama_transmit_seq", "panorama_transmit_dt_ms",
    "panorama_transmit_bytes", "camera_height_m",
    "panorama_fullres_file", "panorama_fullres_seq", "panorama_fullres_dt_ms",
    "pose_position_x", "pose_position_y", "pose_position_z",
    "pose_quat_x", "pose_quat_y", "pose_quat_z", "pose_quat_w",
    "pose_fix", "pose_dt_ms", "pose_interpolated",
    "map_version", "map_capture_ts_ns", "map_keyframe_file", "map_points",
    "map_dt_ms", "map_replay_file",
    "periscope_root", "periscope_segment", "periscope_byte_offset", "periscope_byte_len",
    "periscope_seq", "periscope_keyframe", "periscope_codec", "periscope_dt_ms",
    "periscope_yaw_deg", "periscope_pitch_deg", "periscope_hfov_deg",
    "esdf_npz", "esdf_submap_index", "esdf_dt_ms",
    "trajectory_file", "trajectory_dt_ms",
    "linked_frame", "linked_frame_repeat",
]


def build_timeline(rep: Recording, *, fps: float = 10.0, at: str = "uniform",
                   start_s: float = 0.0, end_s: Optional[float] = None,
                   window: str = "aligned") -> List[int]:
    """The tick timestamps (session clock ns) the export will resolve."""
    lo, hi = rep.window(window)
    if lo is None:
        raise SystemExit("no stream has any samples — nothing to compose")
    # Be loud when the window throws away a lot of a stream: one late-starting stream
    # silently truncating the export is a nasty surprise on a 5-minute capture.
    for name, (a, b) in rep.spans().items():
        outside = max(0, lo - a) + max(0, b - hi)
        total = max(1, b - a)
        if outside / total > 0.25:
            log.warning(
                f"[timeline] '{name}' spans {(b - a) / 1e9:.1f}s but "
                f"{outside / 1e9:.1f}s of it lies outside the '{window}' window "
                f"({(hi - lo) / 1e9:.1f}s) — that data will not be composed. "
                f"Use --window full to keep the whole capture.")
    lo = int(lo + start_s * 1e9)
    if end_s is not None:
        hi = min(int(hi), int(lo + end_s * 1e9))
    if hi <= lo:
        raise SystemExit(f"empty window after --start/--end (lo={lo} hi={hi})")
    if at == "panorama":
        ts = rep.ts("panorama_transmit")
        if ts.size == 0:
            ts = rep.ts("panorama_fullres")
        if ts.size == 0:
            raise SystemExit("--at panorama needs a recorded panorama stream")
        return [int(t) for t in ts if lo <= int(t) <= hi]
    if fps <= 0:
        raise SystemExit("--fps must be > 0 for a uniform timeline")
    step = int(round(1e9 / fps))
    return list(range(lo, hi + 1, step))


def export(rep: Recording, out_dir: str, *, fps: float = 10.0, at: str = "uniform",
           start_s: float = 0.0, end_s: Optional[float] = None,
           map_mode: str = "keyframe", link: str | bool = "hard",
           max_pose_gap_s: float = 0.5, window: str = "aligned",
           panorama: str = "auto") -> List[dict]:
    """Resolve every stream at every tick and write the aligned asset timeline."""
    if map_mode not in MAP_MODES:
        raise SystemExit(f"--map must be one of {MAP_MODES}")
    if link is True:
        link = "hard"
    elif link is False:
        link = "none"
    if link not in LINK_MODES:
        raise SystemExit(f"--link must be one of {LINK_MODES}")

    os.makedirs(out_dir, exist_ok=True)
    ticks = build_timeline(rep, fps=fps, at=at, start_s=start_s, end_s=end_s,
                           window=window)
    t0 = ticks[0]
    log.info(f"[export] {len(ticks)} ticks over "
             f"{(ticks[-1] - t0) / 1e9:.2f}s  map={map_mode}  link={link}")

    prefer_fullres = panorama == "fullres" or (
        panorama == "auto" and not rep.panorama_transmit and rep.panorama_fullres)

    # Map replay is a single forward pass over the recorded artefacts, dumping the
    # store whenever the timeline crosses a tick — O(records + ticks), not O(both).
    replay_files: Dict[int, str] = {}
    if map_mode == "replay":
        replay_files = _replay_to_ticks(rep, ticks, os.path.join(out_dir, "map"))

    frames_dir = os.path.join(out_dir, "frames") if link != "none" else None
    if frames_dir:
        os.makedirs(frames_dir, exist_ok=True)

    rows: List[dict] = []
    last_linked_src = None            # (rel, abs) of the last frame placed
    n_linked = n_link_holes = 0
    for n, t in enumerate(ticks):
        row = {c: None for c in TIMELINE_COLUMNS}
        row["tick"] = n
        row["src_ts_ns"] = t
        row["t_rel_s"] = round((t - t0) / 1e9, 6)

        ptx = rep.nearest("panorama_transmit", t)
        if ptx:
            row.update(panorama_transmit_file=ptx.get("file") or None,
                       panorama_transmit_seq=ptx.get("seq"),
                       panorama_transmit_dt_ms=round(ptx["_dt_ms"], 3),
                       panorama_transmit_bytes=ptx.get("wire_bytes"),
                       camera_height_m=ptx.get("camera_height_m"))
        pfr = rep.nearest("panorama_fullres", t)
        if pfr:
            row.update(panorama_fullres_file=pfr.get("file") or None,
                       panorama_fullres_seq=pfr.get("seq"),
                       panorama_fullres_dt_ms=round(pfr["_dt_ms"], 3))

        pose = interp_pose(rep, t, max_gap_s=max_pose_gap_s)
        if pose:
            p, q = pose["position"], pose["quaternion"]
            row.update(pose_position_x=round(p[0], 6), pose_position_y=round(p[1], 6),
                       pose_position_z=round(p[2], 6),
                       pose_quat_x=round(q[0], 9), pose_quat_y=round(q[1], 9),
                       pose_quat_z=round(q[2], 9), pose_quat_w=round(q[3], 9),
                       pose_fix=pose.get("fix"),
                       pose_dt_ms=round(pose["_dt_ms"], 3),
                       pose_interpolated=int(bool(pose["_interpolated"])))

        if map_mode != "none":
            kf = rep.at_or_before("keyframes", t) or rep.nearest("keyframes", t)
            if kf:
                row.update(map_version=kf.get("map_version"),
                           map_capture_ts_ns=kf.get("capture_ts_ns"),
                           map_keyframe_file=kf.get("file"),
                           map_points=kf.get("n_points"),
                           map_dt_ms=round(kf["_dt_ms"], 3))
            elif rep.snapshots:
                sn = rep.at_or_before("snapshots", t) or rep.nearest("snapshots", t)
                if sn:
                    row.update(map_version=sn.get("map_version"),
                               map_capture_ts_ns=sn.get("capture_ts_ns"),
                               map_keyframe_file=sn.get("file"),
                               map_points=sn.get("n_points"),
                               map_dt_ms=round(sn["_dt_ms"], 3))
            if t in replay_files:
                row["map_replay_file"] = replay_files[t]

        per = rep.nearest("periscope", t)
        if per:
            row.update(periscope_root=per.get("_root"),
                       periscope_segment=per.get("segment"),
                       periscope_byte_offset=per.get("byte_offset"),
                       periscope_byte_len=per.get("byte_len"),
                       periscope_seq=per.get("seq"),
                       periscope_keyframe=per.get("keyframe"),
                       periscope_codec=per.get("codec"),
                       periscope_dt_ms=round(per["_dt_ms"], 3),
                       periscope_yaw_deg=per.get("yaw_deg"),
                       periscope_pitch_deg=per.get("pitch_deg"),
                       periscope_hfov_deg=per.get("hfov_deg"))

        es = rep.nearest("esdf", t)
        if es:
            row.update(esdf_npz=es.get("npz"),
                       esdf_submap_index=es.get("submap_index"),
                       esdf_dt_ms=round(es["_dt_ms"], 3))
        tj = rep.at_or_before("trajectory", t) or rep.nearest("trajectory", t)
        if tj:
            row.update(trajectory_file=tj.get("file"),
                       trajectory_dt_ms=round(tj["_dt_ms"], 3))

        if frames_dir:
            src_rel = (row["panorama_fullres_file"] if prefer_fullres
                       else row["panorama_transmit_file"]) or \
                      row["panorama_transmit_file"] or row["panorama_fullres_file"]
            # `ffmpeg -i frames/%06d.jpg` stops at the FIRST gap and still exits 0, so a
            # single missing blob would silently truncate the video. Hold the previous
            # frame instead — that is what a video should do across a dropout anyway —
            # and flag the repeat so the timeline stays honest about it.
            src_abs = ""
            if src_rel:
                src_row = (pfr if src_rel == row["panorama_fullres_file"] else ptx)
                src_abs = rep.fp(src_row) if src_row else ""
            repeated = 0
            if not src_abs and last_linked_src:
                src_rel, src_abs, repeated = last_linked_src[0], last_linked_src[1], 1
            if src_abs:
                dst = os.path.join(frames_dir, f"{n:06d}.jpg")
                if _place(src_abs, dst, link):
                    row["linked_frame"] = f"frames/{n:06d}.jpg"
                    row["linked_frame_repeat"] = repeated
                    last_linked_src = (src_rel, src_abs)
                    n_linked += 1
            if row["linked_frame"] is None:
                n_link_holes += 1
        rows.append(row)

    if frames_dir and n_link_holes:
        log.warning(
            f"[export] {n_link_holes} of {len(rows)} ticks have no frames/ image "
            f"(the first ticks precede any panorama frame). `ffmpeg -i frames/%06d.jpg` "
            f"stops at the first gap and still exits 0 — start from tick "
            f"{n_link_holes:06d}, or use --window full / --start to move the timeline "
            f"onto the panorama stream.")
    _write_timeline(rep, out_dir, rows, ticks, fps, at, map_mode, link,
                    prefer_fullres, window, n_linked, n_link_holes)
    log.info(f"[export] wrote {out_dir}")
    return rows


def _place(src: str, dst: str, mode: str) -> bool:
    """Hard-link (cheap, works on NTFS) or copy ``src`` → ``dst``."""
    if not os.path.exists(src):
        return False
    if os.path.exists(dst):
        return True
    try:
        if mode == "hard":
            os.link(src, dst)
            return True
    except (OSError, NotImplementedError):
        pass                                     # cross-device / unsupported → copy
    try:
        shutil.copyfile(src, dst)
        return True
    except OSError as e:
        log.warning(f"[export] could not place {src}: {e}")
        return False


def _replay_to_ticks(rep: Recording, ticks: List[int], out_dir: str) -> Dict[int, str]:
    """One forward pass: dump the replayed map whenever we cross a tick."""
    if not getattr(bm, "_HAVE_DRACO", False):
        raise SystemExit("--map replay needs DracoPy (a client dependency):\n"
                         "  cd client && uv sync      # or: uv pip install DracoPy")
    os.makedirs(out_dir, exist_ok=True)
    cube_m = float((rep.meta.get("zenoh") or {}).get("cube_size_m") or rcfg.CUBE_SIZE)
    store = bm.ClientBlockStore(cube_m)
    out: Dict[int, str] = {}
    ti = 0
    version = -1
    # The map only changes once per submap (~1 Hz) but ticks run at --fps, so writing a
    # file per tick would emit thousands of near-identical full-map .npz files — tens of
    # GB for a five-minute capture. Write one per distinct state and point the
    # intervening ticks at it.
    last_written = {"version": None, "rel": None}
    n_reused = 0

    def dump(tick_i: int, t_ns: int) -> None:
        nonlocal n_reused
        if last_written["rel"] is not None and last_written["version"] == version:
            out[t_ns] = last_written["rel"]
            n_reused += 1
            return
        merged = store.merged()
        if merged is None:
            with store._lock:                                    # noqa: SLF001
                vals = list(store.blocks.values())
            if not vals:
                return
            xyz = np.concatenate([b[1] for b in vals])
            rgb = np.concatenate([b[2] for b in vals])
        else:
            xyz, rgb = merged
        if xyz.shape[0] == 0:
            return
        rel = f"map/map_{tick_i:06d}.npz"
        buf = io.BytesIO()
        np.savez_compressed(buf, points=xyz.astype(np.float32),
                            colors=rgb.astype(np.uint8),
                            map_version=np.int64(version),
                            src_ts_ns=np.int64(t_ns))
        with open(os.path.join(os.path.dirname(out_dir), rel), "wb") as f:
            f.write(buf.getvalue())
        out[t_ns] = rel
        last_written["version"] = version
        last_written["rel"] = rel

    for rec in rep.pointcloud:
        t = rec["_t"]
        while ti < len(ticks) and ticks[ti] < t:
            dump(ti, ticks[ti])
            ti += 1
        if rec.get("kind") == "snapshot":
            try:
                with open(rep.fp(rec), "rb") as f:
                    v, xyz, rgb, _s, _sv = proto.unpack_pcd(f.read())
                store.clear()
                store.apply_bundle_bytes(bm.pack_bundle(
                    _blocks_from_cloud(xyz, rgb, cube_m), cube_m))
                version = int(v)
            except Exception as e:                               # noqa: BLE001
                log.warning(f"[replay] snapshot {rec.get('file')}: {e}")
            continue
        if _map_apply(store, rep, rec) and rec.get("map_version") is not None:
            version = int(rec["map_version"])
    while ti < len(ticks):                        # remaining ticks see the final map
        dump(ti, ticks[ti])
        ti += 1
    n_files = len(set(out.values()))
    log.info(f"[replay] {n_files} distinct map state(s) written for {len(ticks)} ticks "
             f"({n_reused} ticks reuse an unchanged map)")
    return out


def _write_timeline(rep, out_dir, rows, ticks, fps, at, map_mode, link,
                    prefer_fullres, window="aligned", n_linked=0,
                    n_link_holes=0) -> None:
    with open(os.path.join(out_dir, "timeline.csv"), "w", newline="",
              encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=TIMELINE_COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    with open(os.path.join(out_dir, "timeline.jsonl"), "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, separators=(",", ":")) + "\n")

    filled = {c: sum(1 for r in rows if r[c] is not None) for c in TIMELINE_COLUMNS}
    with open(os.path.join(out_dir, "README.md"), "w", encoding="utf-8") as f:
        f.write(_EXPORT_README.format(
            session_id=rep.session_id, root=rep.root, n=len(rows),
            fps=("per panorama frame" if at == "panorama" else f"{fps} Hz"),
            span=f"{(ticks[-1] - ticks[0]) / 1e9:.3f}",
            t0=ticks[0], t1=ticks[-1], map_mode=map_mode, link=link,
            window=window,
            frames=(f"{n_linked} linked"
                    + (f", {n_link_holes} tick(s) with NO image — `ffmpeg -i "
                       f"frames/%06d.jpg` will stop at the first gap"
                       if n_link_holes else "")),
            panorama=("full-res" if prefer_fullres else "transmit"),
            coverage="\n".join(
                f"| `{c}` | {filled[c]} / {len(rows)} |"
                for c in TIMELINE_COLUMNS if filled[c])))


_EXPORT_README = """\
# Aligned export — `{session_id}`

Source recording: `{root}`

| | |
|---|---|
| ticks | {n} |
| rate | {fps} |
| span | {span} s (session clock `{t0}` … `{t1}`) |
| window | `{window}` |
| map mode | `{map_mode}` |
| frame links | `{link}` (from the {panorama} panorama) — {frames} |

Every row of `timeline.csv` / `timeline.jsonl` is **one instant on the session
clock** (`src_ts_ns`, the robot capture clock in nanoseconds) with each stream
resolved to what belonged at that instant. `*_dt_ms` is how far the chosen sample
sits from the tick — check it before trusting an alignment; a large value means that
stream had a gap there, not that the tick is wrong.

* **Pose** is *interpolated* to the exact tick (LERP position, SLERP orientation),
  so `pose_dt_ms` is 0 and `pose_interpolated` is 1. Empty pose columns mean a real
  dropout longer than the allowed gap — the export refuses to reuse a stale pose.
* **Map** columns point at a materialised keyframe (`map_keyframe_file`, an `.npz`
  with `points` / `colors` / `map_version` / `capture_ts_ns`) and, with
  `--map replay`, at the exact replayed state for that tick (`map_replay_file`).
  Align maps on `map_capture_ts_ns` when you need the true capture time.
* **Periscope** gives `(periscope_segment, periscope_byte_offset,
  periscope_byte_len)`: slice exactly those bytes out of the elementary stream to
  get that encoded frame. `compose.py periscope` does it for you.
* **`linked_frame_repeat` = 1** means this tick had no panorama frame of its own and
  `frames/` holds a repeat of the previous one, so the `%06d` sequence stays
  gap-free (`ffmpeg` stops at the first gap and still exits 0).
* The **window** is the intersection of the continuously-published streams. Pass
  `--window full` to keep the whole capture instead, accepting that some streams are
  absent at the edges; the export warns whenever a stream is substantially clipped.

## Making a video from this

```sh
# panorama at the export rate, straight from the numbered links
ffmpeg -framerate 10 -i frames/%06d.jpg -c:v libx264 -pix_fmt yuv420p panorama.mp4

# or let compose.py render the cloud + panels (needs Open3D)
python tools/recorder/compose.py render {root} --out demo.mp4 --fps 10
```

## Column coverage

| column | rows filled |
|---|---|
{coverage}
"""


# ═════════════════════════════════════════════════════════════════════════════
# periscope extraction
# ═════════════════════════════════════════════════════════════════════════════


def periscope_extract(rep: Recording, out_dir: str, *, limit: int = 0,
                      decode: bool = False) -> int:
    """Slice each periscope frame out of the elementary stream into its own file.

    With ``decode=True`` also decodes to PNG using the *viewer's own* decoder
    (``vat_client.periscope_view._Decoder``), so "does it decode here?" gets the same
    answer it would in the live client.
    """
    rows = rep.periscope
    if not rows:
        raise SystemExit("no periscope frames in this recording")
    os.makedirs(out_dir, exist_ok=True)
    decoder = None
    if decode:
        decoder = _load_periscope_decoder()
    handles: Dict[str, io.BufferedReader] = {}
    n = 0
    n_decoded = 0
    try:
        for r in rows:
            seg = r.get("segment")
            if not seg:
                continue
            if seg not in handles:
                handles[seg] = open(rep.fp(r, "segment"), "rb")
            f = handles[seg]
            f.seek(int(r["byte_offset"]))
            payload = f.read(int(r["byte_len"]))
            codec = r.get("codec", "bin")
            ext = "jpg" if codec == "mjpeg" else codec
            base = f"{int(r['src_ts_ns'])}_seq{int(r['seq']):06d}"
            with open(os.path.join(out_dir, f"{base}.{ext}"), "wb") as o:
                o.write(payload)
            if decoder is not None:
                rgb = _decode_periscope(decoder, codec, payload)
                if rgb is not None:
                    _write_png(os.path.join(out_dir, f"{base}.png"), rgb)
                    n_decoded += 1
            n += 1
            if limit and n >= limit:
                break
    finally:
        for f in handles.values():
            f.close()
    log.info(f"[periscope] extracted {n} frame(s) → {out_dir}")
    if decoder is not None:
        if n_decoded == 0:
            log.warning(
                f"[periscope] 0 of {n} frames DECODED — the encoded frames are on "
                f"disk but this machine cannot decode them. H.264/H.265 needs PyAV "
                f"(`cd client && uv sync`); a run that starts mid-GOP also cannot "
                f"decode until the first keyframe (see MANIFEST.json → periscope → "
                f"frames_before_first_keyframe).")
        else:
            log.info(f"[periscope] decoded {n_decoded}/{n} to PNG")
    return n


def _load_periscope_decoder():
    """The viewer's exact decoder, as ``tools/periscope_probe.py`` loads it."""
    client_dir = os.path.join(rcfg.REPO_ROOT, "client")
    if client_dir not in sys.path:
        sys.path.insert(0, client_dir)
    try:
        from vat_client.periscope_view import _Decoder
        return _Decoder()
    except Exception as e:                                       # noqa: BLE001
        raise SystemExit(
            f"cannot load the periscope decoder: {e}\n"
            f"H.264/H.265 needs PyAV and MJPEG needs OpenCV — both are client "
            f"dependencies:\n  cd client && uv sync")


def _decode_periscope(decoder, codec: str, payload: bytes):
    ids = {"mjpeg": proto.PSCOPE_CODEC_MJPEG, "h264": proto.PSCOPE_CODEC_H264,
           "hevc": proto.PSCOPE_CODEC_HEVC}
    try:
        return decoder.decode(ids.get(codec, proto.PSCOPE_CODEC_H264), payload)
    except Exception as e:                                       # noqa: BLE001
        log.debug(f"[periscope] decode failed: {e}")
        return None


def _write_png(path: str, rgb) -> None:
    try:
        import cv2
        cv2.imwrite(path, np.asarray(rgb)[:, :, ::-1])           # RGB → BGR
    except ImportError:
        log.warning("[periscope] OpenCV missing — skipping PNG output")


# ═════════════════════════════════════════════════════════════════════════════
# render  (optional: Open3D offscreen + ffmpeg)
# ═════════════════════════════════════════════════════════════════════════════


class CloudRenderer:
    """Offscreen point-cloud + trajectory renderer, in a 'toy box' oblique view.

    Deliberately thin: it draws the map, the camera trail and a robot marker, and
    hands back an RGB array. Layout and panels are the caller's problem, so the
    final composition stays a choice rather than a hard-coded design.
    """

    def __init__(self, width: int = 1280, height: int = 720, point_size: float = 2.5,
                 bg=(0.05, 0.06, 0.08, 1.0)):
        try:
            import open3d as o3d
        except ImportError:
            raise SystemExit(
                "rendering needs Open3D, which is NOT a default client dependency "
                "(the live viewer uses VisPy):\n"
                "  cd client && uv pip install open3d\n"
                "Or skip rendering: `compose.py export` gives you aligned per-frame "
                "assets for any video editor.")
        self.o3d = o3d
        self.width, self.height = int(width), int(height)
        self.point_size = float(point_size)
        self.renderer = o3d.visualization.rendering.OffscreenRenderer(
            self.width, self.height)
        self.renderer.scene.set_background(list(bg))
        self._mat = o3d.visualization.rendering.MaterialRecord()
        self._mat.shader = "defaultUnlit"
        self._mat.point_size = self.point_size
        self._line = o3d.visualization.rendering.MaterialRecord()
        self._line.shader = "unlitLine"
        self._line.line_width = 3.0

    def frame(self, xyz, rgb, trail=None, robot_xyz=None, eye_offset=(-4.0, -4.0, 3.0),
              up=(0.0, 0.0, 1.0)):
        o3d = self.o3d
        scene = self.renderer.scene
        scene.clear_geometry()
        if xyz is not None and len(xyz):
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(np.asarray(xyz, np.float64))
            col = np.asarray(rgb)
            col = col.astype(np.float64) / 255.0 if col.dtype != np.float64 else col
            pc.colors = o3d.utility.Vector3dVector(np.clip(col, 0.0, 1.0))
            scene.add_geometry("map", pc, self._mat)
        if trail is not None and len(trail) >= 2:
            t = np.asarray(trail, np.float64)
            ls = o3d.geometry.LineSet()
            ls.points = o3d.utility.Vector3dVector(t)
            ls.lines = o3d.utility.Vector2iVector(
                np.stack([np.arange(len(t) - 1), np.arange(1, len(t))], axis=1))
            ls.colors = o3d.utility.Vector3dVector(
                np.tile([[1.0, 0.75, 0.1]], (len(t) - 1, 1)))
            scene.add_geometry("trail", ls, self._line)
        centre = (np.asarray(robot_xyz, np.float64) if robot_xyz is not None
                  else (np.asarray(xyz, np.float64).mean(axis=0)
                        if xyz is not None and len(xyz) else np.zeros(3)))
        if robot_xyz is not None:
            marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.12)
            marker.translate(centre)
            marker.paint_uniform_color([0.95, 0.25, 0.2])
            marker.compute_vertex_normals()
            scene.add_geometry("robot", marker, self._mat)
        eye = centre + np.asarray(eye_offset, np.float64)
        self.renderer.setup_camera(60.0, centre, eye, np.asarray(up, np.float64))
        img = self.renderer.render_to_image()
        return np.asarray(img)


def render(rep: Recording, out_path: str, *, fps: float = 10.0, at: str = "uniform",
           start_s: float = 0.0, end_s: Optional[float] = None,
           layout: str = "quad", width: int = 1280, height: int = 720,
           map_mode: str = "keyframe", trail_len: int = 400,
           window: str = "aligned", keep_frames: bool = False) -> str:
    """Render a synchronized demo video: map + trail + panorama + periscope panels."""
    if layout not in LAYOUTS:
        raise SystemExit(f"--layout must be one of {LAYOUTS}")
    if shutil.which("ffmpeg") is None:
        raise SystemExit("rendering needs ffmpeg on PATH to mux the frames")
    try:
        import cv2
    except ImportError:
        raise SystemExit("rendering needs OpenCV (a client dependency): "
                         "cd client && uv sync")

    # Build the renderer FIRST: a missing Open3D should fail immediately, before we
    # spend time exporting a timeline into a scratch directory.
    cr = CloudRenderer(width, height)

    work = tempfile.mkdtemp(prefix="vat-render-") if not keep_frames else \
        os.path.join(os.path.dirname(os.path.abspath(out_path)), "render_frames")
    os.makedirs(work, exist_ok=True)
    rows = export(rep, os.path.join(work, "aligned"), fps=fps, at=at,
                  start_s=start_s, end_s=end_s, map_mode=map_mode, link="none",
                  window=window)
    decoder = None
    if layout == "quad" and any(r["periscope_byte_len"] for r in rows):
        try:
            decoder = _load_periscope_decoder()
        except SystemExit as e:
            log.warning(f"[render] periscope panel disabled: {e}")

    trail: List[list] = []
    seg_handles: Dict[str, io.BufferedReader] = {}
    map_cache: Tuple[Optional[str], Optional[tuple]] = (None, None)
    n_written = 0
    try:
        for row in rows:
            if row.get("pose_position_x") is not None:
                trail.append([row["pose_position_x"], row["pose_position_y"],
                              row["pose_position_z"]])
                trail = trail[-max(2, trail_len):]
            map_file = row.get("map_replay_file") or row.get("map_keyframe_file")
            if map_file and map_file != map_cache[0]:
                map_cache = (map_file, _load_cloud(rep, work, map_file))
            xyz, rgb = map_cache[1] if map_cache[1] else (None, None)
            robot = (np.array([row["pose_position_x"], row["pose_position_y"],
                               row["pose_position_z"]], np.float64)
                     if row.get("pose_position_x") is not None else None)
            canvas = cr.frame(xyz, rgb, trail=trail, robot_xyz=robot)
            canvas = np.ascontiguousarray(canvas[:, :, :3])

            if layout in ("cloud+panorama", "quad") and row["panorama_transmit_file"]:
                pano = cv2.imread(rep.p(*row["panorama_transmit_file"].split("/")))
                if pano is not None:
                    canvas = _overlay(canvas, pano[:, :, ::-1], "bottom", 0.34)
            if layout == "quad" and decoder is not None and row["periscope_byte_len"]:
                pf = _read_periscope(rep, seg_handles, row)
                if pf is not None:
                    rgbf = _decode_periscope(decoder, row["periscope_codec"], pf)
                    if rgbf is not None:
                        canvas = _overlay(canvas, np.asarray(rgbf), "topright", 0.28)

            _label(cv2, canvas, f"t={row['t_rel_s']:7.2f}s  "
                                f"map v{row.get('map_version')}  "
                                f"{row.get('map_points') or 0} pts")
            cv2.imwrite(os.path.join(work, f"f{row['tick']:06d}.png"),
                        canvas[:, :, ::-1])
            n_written += 1
    finally:
        for f in seg_handles.values():
            f.close()

    if n_written == 0:
        raise SystemExit("nothing was rendered — check `compose.py info`")
    cmd = ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
           "-framerate", f"{fps:g}", "-i", os.path.join(work, "f%06d.png"),
           "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", out_path]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f"ffmpeg failed: {(r.stderr or '').strip()[:400]}")
    log.info(f"[render] {out_path}  ({n_written} frames @ {fps:g} fps)")
    if not keep_frames:
        shutil.rmtree(work, ignore_errors=True)
    return out_path


def _load_cloud(rep: Recording, work: str, rel: str):
    path = rep.p(*rel.split("/"))
    if not os.path.exists(path):
        path = os.path.join(work, "aligned", *rel.split("/"))
    try:
        if path.endswith(".npz"):
            with np.load(path) as z:
                return np.asarray(z["points"]), np.asarray(z["colors"])
        with open(path, "rb") as f:                 # a raw pack_pcd snapshot
            _v, xyz, rgb, _s, _sv = proto.unpack_pcd(f.read())
        return xyz, (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
    except Exception as e:                                       # noqa: BLE001
        log.warning(f"[render] could not load map {rel}: {e}")
        return None


def _seg_abs(rep: Recording, row: dict) -> str:
    """Absolute path of a timeline row's periscope segment (merge-aware)."""
    seg = row.get("periscope_segment")
    if not seg:
        return ""
    root = row.get("periscope_root") or rep.root
    return os.path.join(root, *str(seg).split("/"))


def _read_periscope(rep: Recording, handles: dict, row: dict):
    seg = row.get("periscope_segment")
    if not seg:
        return None
    if seg not in handles:
        try:
            handles[seg] = open(_seg_abs(rep, row), "rb")
        except OSError:
            return None
    f = handles[seg]
    f.seek(int(row["periscope_byte_offset"]))
    return f.read(int(row["periscope_byte_len"]))


def _overlay(canvas: np.ndarray, panel: np.ndarray, where: str,
             scale: float) -> np.ndarray:
    """Composite ``panel`` onto ``canvas`` (both RGB uint8) at a relative size."""
    import cv2
    H, W = canvas.shape[:2]
    ph, pw = panel.shape[:2]
    if ph == 0 or pw == 0:
        return canvas
    tw = max(2, int(W * scale))
    th = max(2, int(round(tw * ph / pw)))
    if th > H // 2:
        th = H // 2
        tw = max(2, int(round(th * pw / ph)))
    small = cv2.resize(panel, (tw, th), interpolation=cv2.INTER_AREA)
    m = 12
    if where == "bottom":
        x, y = (W - tw) // 2, H - th - m
    elif where == "topright":
        x, y = W - tw - m, m
    else:
        x, y = m, m
    canvas[max(0, y - 2):y + th + 2, max(0, x - 2):x + tw + 2] = 30
    canvas[y:y + th, x:x + tw] = small[:, :, :3]
    return canvas


def _label(cv2, canvas: np.ndarray, text: str) -> None:
    cv2.putText(canvas, text, (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (235, 235, 235), 1, cv2.LINE_AA)


# ═════════════════════════════════════════════════════════════════════════════
# CLI
# ═════════════════════════════════════════════════════════════════════════════


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="compose.py",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="Align and compose a vat-record session.",
        epilog="Start with `info`, then `export`. See docs/recording.md.")
    sub = p.add_subparsers(dest="cmd", required=True)

    def _with(sp):
        sp.add_argument("--with", dest="partners", action="append", default=[],
                        metavar="SESSION",
                        help="merge a partner session from the same capture "
                             "(repeatable) — e.g. the robot-side full-res recording "
                             "alongside the cloud-side map recording. Both are on the "
                             "same session clock, so streams are simply unioned.")

    i = sub.add_parser("info", help="summarise a recording and its health")
    i.add_argument("session")
    _with(i)
    i.add_argument("--json", action="store_true", help="machine-readable output")

    e = sub.add_parser("export", help="write aligned per-frame assets on one timeline")
    e.add_argument("session")
    _with(e)
    e.add_argument("--out", default=None,
                   help="output dir (default <session>/aligned)")
    e.add_argument("--fps", type=float, default=10.0, help="uniform tick rate")
    e.add_argument("--at", choices=("uniform", "panorama"), default="uniform",
                   help="tick at a fixed rate, or once per panorama frame")
    e.add_argument("--start", type=float, default=0.0, metavar="S",
                   help="skip the first S seconds of the window")
    e.add_argument("--end", type=float, default=None, metavar="S",
                   help="stop S seconds after --start")
    e.add_argument("--map", dest="map_mode", choices=MAP_MODES, default="keyframe",
                   help="'keyframe' references the nearest materialised map; "
                        "'replay' rebuilds the exact state per tick from the recorded "
                        "pushes; 'none' skips the map (default keyframe)")
    e.add_argument("--link", choices=LINK_MODES, default="hard",
                   help="numbered frames/ dir for ffmpeg: hard link, copy, or none")
    e.add_argument("--panorama", choices=("auto", "transmit", "fullres"),
                   default="auto", help="which panorama feeds frames/")
    e.add_argument("--window", choices=("aligned", "full"), default="aligned",
                   help="'aligned' = the span where every continuously-published "
                        "stream has data (default); 'full' = the whole capture, "
                        "accepting missing streams at the edges")
    e.add_argument("--max-pose-gap", type=float, default=0.5, metavar="S",
                   help="refuse to interpolate a pose across a gap longer than S")

    q = sub.add_parser("periscope", help="slice periscope frames out of the stream")
    q.add_argument("session")
    _with(q)
    q.add_argument("--out", default=None, help="output dir (default <session>/periscope_frames)")
    q.add_argument("--limit", type=int, default=0, help="stop after N frames")
    q.add_argument("--decode", action="store_true",
                   help="also decode to PNG with the viewer's own decoder")

    r = sub.add_parser("render", help="render a synchronized demo video (needs Open3D)")
    r.add_argument("session")
    _with(r)
    r.add_argument("--out", default="demo.mp4")
    r.add_argument("--fps", type=float, default=10.0)
    r.add_argument("--at", choices=("uniform", "panorama"), default="uniform")
    r.add_argument("--start", type=float, default=0.0)
    r.add_argument("--end", type=float, default=None)
    r.add_argument("--layout", choices=LAYOUTS, default="quad")
    r.add_argument("--width", type=int, default=1280)
    r.add_argument("--height", type=int, default=720)
    r.add_argument("--map", dest="map_mode", choices=MAP_MODES, default="keyframe")
    r.add_argument("--window", choices=("aligned", "full"), default="aligned")
    r.add_argument("--trail", type=int, default=400, help="trail length in ticks")
    r.add_argument("--keep-frames", action="store_true",
                   help="keep the intermediate PNGs next to --out")
    return p


def main(argv=None) -> int:
    logging.basicConfig(level=os.environ.get("LOG_LEVEL", "INFO").upper(),
                        format="[%(levelname)s] %(message)s")
    args = build_parser().parse_args(argv)
    rep = load(args.session, extra_roots=getattr(args, "partners", []) or [])

    if args.cmd == "info":
        if args.json:
            print(json.dumps(info(rep), indent=2, default=str))
        else:
            print_info(rep)
        return 0
    if args.cmd == "export":
        out = args.out or rep.p("aligned")
        export(rep, out, fps=args.fps, at=args.at, start_s=args.start,
               end_s=args.end, map_mode=args.map_mode, link=args.link,
               panorama=args.panorama, max_pose_gap_s=args.max_pose_gap,
               window=args.window)
        print(f"\naligned export → {out}\n  timeline.csv / timeline.jsonl / README.md")
        return 0
    if args.cmd == "periscope":
        out = args.out or rep.p("periscope_frames")
        n = periscope_extract(rep, out, limit=args.limit, decode=args.decode)
        print(f"\n{n} periscope frame(s) → {out}")
        return 0
    if args.cmd == "render":
        path = render(rep, args.out, fps=args.fps, at=args.at, start_s=args.start,
                      end_s=args.end, layout=args.layout, width=args.width,
                      height=args.height, map_mode=args.map_mode,
                      trail_len=args.trail, window=args.window,
                      keep_frames=args.keep_frames)
        print(f"\nrendered → {path}")
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
