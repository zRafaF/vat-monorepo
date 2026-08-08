"""
VAT recorder — on-disk sinks, indexes and storage discipline.
============================================================
Everything that touches the filesystem lives here so the stream recorders stay
about *what* a stream means, not about durability.

Design rules
------------
* **Byte-exact blobs.** Payloads are written as they came off the wire (JPEG
  bodies, Annex-B NAL units, Draco push frames). Offline tooling then re-reads
  them with the repo's own ``vat_protocol`` / ``vat_blockmap`` unpackers — no
  second serialisation to get out of sync.
* **Atomic publish.** Blobs go to ``<name>.tmp`` then ``os.replace`` — a recording
  killed mid-write never leaves a half file that looks complete.
* **Indexes flush per record.** Index files (CSV / JSONL) are small and are the
  thing that makes a partial recording *usable*, so they are flushed on every
  append. Ctrl-C therefore costs at most the sample in flight.
* **Budgets, not surprises.** :class:`Budget` enforces the wall-clock and byte
  caps; :class:`RingBudget` additionally evicts oldest-first so a long run can
  keep a bounded *newest* window (used for the full-res panorama).
* **Nothing is dropped silently.** :class:`StreamStats` tracks counts, bytes,
  timestamp span, sequence gaps and refusals, and every stream reports them into
  ``MANIFEST.json`` at close.
"""

from __future__ import annotations

import csv
import json
import os
import threading
import time
from collections import deque
from typing import Deque, Iterable, Optional, Tuple


# ═════════════════════════════════════════════════════════════════════════════
# Budgets
# ═════════════════════════════════════════════════════════════════════════════


class Budget:
    """A wall-clock + byte allowance. ``0`` on either cap means *uncapped*."""

    def __init__(self, max_bytes: int = 0, duration_s: float = 0.0,
                 name: str = "budget"):
        self.name = name
        self.max_bytes = int(max_bytes or 0)
        self.duration_s = float(duration_s or 0.0)
        self._t0 = time.monotonic()
        self._bytes = 0
        self._refused = 0
        self._lock = threading.Lock()

    @property
    def bytes_written(self) -> int:
        with self._lock:
            return self._bytes

    @property
    def refused(self) -> int:
        with self._lock:
            return self._refused

    @property
    def elapsed_s(self) -> float:
        return time.monotonic() - self._t0

    def time_expired(self) -> bool:
        return self.duration_s > 0.0 and self.elapsed_s >= self.duration_s

    def bytes_exhausted(self) -> bool:
        with self._lock:
            return self.max_bytes > 0 and self._bytes >= self.max_bytes

    def expired(self) -> bool:
        return self.time_expired() or self.bytes_exhausted()

    def claim(self, nbytes: int) -> bool:
        """Reserve ``nbytes``. False (and a refusal counted) if that busts the cap."""
        with self._lock:
            if self.max_bytes > 0 and self._bytes + int(nbytes) > self.max_bytes:
                self._refused += 1
                return False
            self._bytes += int(nbytes)
            return True

    def release(self, nbytes: int) -> None:
        """Give bytes back (a ring evicted a file, or a write failed)."""
        with self._lock:
            self._bytes = max(0, self._bytes - int(nbytes))

    def summary(self) -> dict:
        return {
            "bytes_written": self.bytes_written,
            "max_bytes": self.max_bytes or None,
            "duration_s": round(self.elapsed_s, 3),
            "max_duration_s": self.duration_s or None,
            "refused_samples": self.refused,
            "time_expired": self.time_expired(),
            "bytes_exhausted": self.bytes_exhausted(),
        }


class RingBudget(Budget):
    """A :class:`Budget` that evicts oldest files instead of refusing new ones.

    Used for the full-resolution panorama: on a long walk you usually want the
    newest N gigabytes rather than the first N gigabytes, and a hard refusal would
    silently truncate the tail of the capture.
    """

    def __init__(self, max_bytes: int = 0, duration_s: float = 0.0,
                 name: str = "ring"):
        super().__init__(max_bytes, duration_s, name)
        self._files: Deque[Tuple[str, int]] = deque()
        self._evicted = 0
        self._evicted_bytes = 0

    def track(self, path: str, nbytes: int) -> Iterable[str]:
        """Register a written file; return the paths evicted to stay under the cap."""
        with self._lock:
            self._files.append((path, int(nbytes)))
        gone = []
        while self.max_bytes > 0 and self.bytes_written > self.max_bytes:
            with self._lock:
                if not self._files:
                    break
                path, nb = self._files.popleft()
            try:
                os.remove(path)
            except OSError:
                pass
            self.release(nb)
            with self._lock:
                self._evicted += 1
                self._evicted_bytes += nb
            gone.append(path)
        return gone

    def claim(self, nbytes: int) -> bool:            # ring never refuses on bytes
        with self._lock:
            self._bytes += int(nbytes)
        return True

    def summary(self) -> dict:
        s = super().summary()
        with self._lock:
            s.update({"ring": True, "evicted_files": self._evicted,
                      "evicted_bytes": self._evicted_bytes,
                      "retained_files": len(self._files)})
        return s


# ═════════════════════════════════════════════════════════════════════════════
# Per-stream health
# ═════════════════════════════════════════════════════════════════════════════


class StreamStats:
    """Counts, byte totals, timestamp span, seq gaps and errors for one stream."""

    def __init__(self, name: str, key: str = ""):
        self.name = name
        self.key = key
        self._lock = threading.Lock()
        self.n = 0
        self.n_bytes = 0
        self.n_errors = 0
        self.n_skipped = 0            # deliberately not written (decimation / caps)
        self.first_src_ts_ns: Optional[int] = None
        self.last_src_ts_ns: Optional[int] = None
        self.first_wall_ns: Optional[int] = None
        self.last_wall_ns: Optional[int] = None
        self._prev_seq: Optional[int] = None
        self.seq_gaps = 0             # number of gap events
        self.seq_missing = 0          # total samples missing across all gaps
        self.seq_backwards = 0
        self.last_error = ""

    def sample(self, nbytes: int = 0, src_ts_ns: Optional[int] = None,
               wall_ns: Optional[int] = None, seq: Optional[int] = None,
               seq_mask: int = 0xFFFFFFFF) -> None:
        with self._lock:
            self.n += 1
            self.n_bytes += int(nbytes)
            if src_ts_ns:
                if self.first_src_ts_ns is None:
                    self.first_src_ts_ns = int(src_ts_ns)
                self.last_src_ts_ns = int(src_ts_ns)
            if wall_ns:
                if self.first_wall_ns is None:
                    self.first_wall_ns = int(wall_ns)
                self.last_wall_ns = int(wall_ns)
            if seq is not None:
                if self._prev_seq is not None:
                    expected = (self._prev_seq + 1) & seq_mask
                    if seq != expected:
                        gap = (seq - expected) & seq_mask
                        # A huge masked delta means the seq went backwards (retransmit
                        # or a restarted publisher), not that millions were lost.
                        if gap > (seq_mask >> 1):
                            self.seq_backwards += 1
                        else:
                            self.seq_gaps += 1
                            self.seq_missing += gap
                self._prev_seq = int(seq)

    def error(self, msg: str) -> None:
        with self._lock:
            self.n_errors += 1
            self.last_error = str(msg)[:200]

    def skip(self, n: int = 1) -> None:
        with self._lock:
            self.n_skipped += int(n)

    def summary(self) -> dict:
        with self._lock:
            span_s = None
            if self.first_src_ts_ns and self.last_src_ts_ns:
                span_s = round((self.last_src_ts_ns - self.first_src_ts_ns) / 1e9, 3)
            # (n-1)/span, not n/span: N samples span N-1 intervals, so this recovers
            # the true nominal rate (25 frames over 9.6 s is 2.5 Hz, not 2.6 Hz).
            hz = (round((self.n - 1) / span_s, 3)
                  if span_s and span_s > 0 and self.n > 1 else None)
            return {
                "stream": self.name, "key": self.key,
                "samples": self.n, "bytes": self.n_bytes,
                "mean_bytes": int(self.n_bytes / self.n) if self.n else 0,
                "src_ts_span_s": span_s, "mean_hz": hz,
                "first_src_ts_ns": self.first_src_ts_ns,
                "last_src_ts_ns": self.last_src_ts_ns,
                "first_wall_ns": self.first_wall_ns,
                "last_wall_ns": self.last_wall_ns,
                "seq_gap_events": self.seq_gaps, "seq_samples_missing": self.seq_missing,
                "seq_backwards": self.seq_backwards,
                "skipped": self.n_skipped,
                "errors": self.n_errors, "last_error": self.last_error,
            }


# ═════════════════════════════════════════════════════════════════════════════
# Index writers
# ═════════════════════════════════════════════════════════════════════════════


class CsvIndex:
    """Append-only CSV with a fixed header, flushed per row.

    ``append`` after ``close`` is a counted no-op, not an exception: a straggler
    worker thread finishing after shutdown must never raise inside a Zenoh callback
    (where the traceback would be swallowed) or abort the session's finalisation.
    """

    def __init__(self, path: str, columns):
        self.path = path
        self.columns = list(columns)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self._f = open(path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(self.columns)
        self._f.flush()
        self._lock = threading.Lock()
        self._closed = False
        self.rows = 0
        self.dropped_after_close = 0

    def append(self, row) -> None:
        with self._lock:
            if self._closed:
                self.dropped_after_close += 1
                return
            self._w.writerow(row)
            self._f.flush()
            self.rows += 1

    def close(self) -> None:
        with self._lock:
            self._closed = True
            try:
                self._f.flush()
                self._f.close()
            except Exception:
                pass


class JsonlIndex:
    """Append-only JSON-lines index, flushed per record (no-op after close)."""

    def __init__(self, path: str):
        self.path = path
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self._f = open(path, "w", encoding="utf-8")
        self._lock = threading.Lock()
        self._closed = False
        self.rows = 0
        self.dropped_after_close = 0

    def append(self, record: dict) -> None:
        line = json.dumps(record, separators=(",", ":"), default=_json_default)
        with self._lock:
            if self._closed:
                self.dropped_after_close += 1
                return
            self._f.write(line + "\n")
            self._f.flush()
            self.rows += 1

    def close(self) -> None:
        with self._lock:
            self._closed = True
            try:
                self._f.flush()
                self._f.close()
            except Exception:
                pass


class TumTrajectory:
    """TUM-format trajectory: ``timestamp tx ty tz qx qy qz qw`` (seconds, space-sep).

    The de-facto interchange format for evo / TUM-RGBD tooling, so the recorded
    pose stream drops straight into an ATE evaluation.
    """

    HEADER = "# timestamp tx ty tz qx qy qz qw"

    def __init__(self, path: str, comment: str = ""):
        self.path = path
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self._f = open(path, "w", encoding="utf-8")
        if comment:
            for line in str(comment).splitlines():
                self._f.write(f"# {line}\n")
        self._f.write(self.HEADER + "\n")
        self._f.flush()
        self._lock = threading.Lock()
        self._closed = False
        self.rows = 0
        self.dropped_after_close = 0

    def append(self, ts_ns: int, pos, quat) -> None:
        line = (f"{ts_ns / 1e9:.9f} "
                f"{float(pos[0]):.6f} {float(pos[1]):.6f} {float(pos[2]):.6f} "
                f"{float(quat[0]):.9f} {float(quat[1]):.9f} "
                f"{float(quat[2]):.9f} {float(quat[3]):.9f}")
        with self._lock:
            if self._closed:
                self.dropped_after_close += 1
                return
            self._f.write(line + "\n")
            self._f.flush()
            self.rows += 1

    def close(self) -> None:
        with self._lock:
            self._closed = True
            try:
                self._f.flush()
                self._f.close()
            except Exception:
                pass


def _json_default(o):
    """Make numpy scalars/arrays JSON-serialisable without importing numpy here."""
    tolist = getattr(o, "tolist", None)
    if callable(tolist):
        return tolist()
    item = getattr(o, "item", None)
    if callable(item):
        return item()
    return str(o)


# ═════════════════════════════════════════════════════════════════════════════
# Session directory
# ═════════════════════════════════════════════════════════════════════════════


class SessionWriter:
    """Owns ``recordings/<session_id>/`` and every file under it."""

    def __init__(self, root: str, session_id: str):
        self.session_id = session_id
        self.root = os.path.abspath(os.path.join(root, session_id))
        os.makedirs(self.root, exist_ok=True)
        self._closables = []
        self._lock = threading.Lock()

    # ── paths ────────────────────────────────────────────────────────────────
    def path(self, *parts: str) -> str:
        return os.path.join(self.root, *parts)

    def subdir(self, *parts: str) -> str:
        d = self.path(*parts)
        os.makedirs(d, exist_ok=True)
        return d

    def rel(self, abs_path: str) -> str:
        """Repo-independent, POSIX-style path relative to the session root."""
        return os.path.relpath(abs_path, self.root).replace(os.sep, "/")

    # ── factories (registered for close) ─────────────────────────────────────
    def csv_index(self, *parts, columns) -> CsvIndex:
        idx = CsvIndex(self.path(*parts), columns)
        with self._lock:
            self._closables.append(idx)
        return idx

    def jsonl_index(self, *parts) -> JsonlIndex:
        idx = JsonlIndex(self.path(*parts))
        with self._lock:
            self._closables.append(idx)
        return idx

    def tum(self, *parts, comment: str = "") -> TumTrajectory:
        t = TumTrajectory(self.path(*parts), comment)
        with self._lock:
            self._closables.append(t)
        return t

    # ── blobs ────────────────────────────────────────────────────────────────
    def write_blob(self, data: bytes, *parts: str) -> Tuple[str, int]:
        """Atomically write ``data``; return ``(abs_path, nbytes)``."""
        dst = self.path(*parts)
        os.makedirs(os.path.dirname(dst) or ".", exist_ok=True)
        tmp = dst + ".tmp"
        with open(tmp, "wb") as f:
            f.write(data)
        os.replace(tmp, dst)
        return dst, len(data)

    def write_json(self, obj: dict, *parts: str) -> str:
        dst = self.path(*parts)
        os.makedirs(os.path.dirname(dst) or ".", exist_ok=True)
        tmp = dst + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(obj, f, indent=2, sort_keys=False, default=_json_default)
            f.write("\n")
        os.replace(tmp, dst)
        return dst

    def read_json(self, *parts: str) -> dict:
        with open(self.path(*parts), "r", encoding="utf-8") as f:
            return json.load(f)

    # ── lifecycle ────────────────────────────────────────────────────────────
    def close(self) -> None:
        with self._lock:
            closables, self._closables = self._closables, []
        for c in closables:
            try:
                c.close()
            except Exception:
                pass


# ═════════════════════════════════════════════════════════════════════════════
# Self-test:  python tools/recorder/rec_sinks.py
# ═════════════════════════════════════════════════════════════════════════════

def _selftest() -> None:
    import shutil
    import tempfile

    tmp = tempfile.mkdtemp(prefix="vatrec-")
    try:
        sw = SessionWriter(tmp, "sess-001")
        assert os.path.isdir(sw.root)

        # blobs are atomic and leave no .tmp behind
        p, n = sw.write_blob(b"\xff\xd8jpeg", "panorama_transmit", "frames", "000000001.jpg")
        assert n == 6 and os.path.exists(p) and not os.path.exists(p + ".tmp")
        assert sw.rel(p) == "panorama_transmit/frames/000000001.jpg"

        # indexes flush per row → readable while still open
        idx = sw.csv_index("panorama_transmit", "frame_index.csv",
                           columns=["seq", "src_ts_ns"])
        idx.append([1, 123])
        with open(idx.path) as f:
            assert f.read().splitlines() == ["seq,src_ts_ns", "1,123"]

        jl = sw.jsonl_index("pointcloud", "index.jsonl")
        jl.append({"map_version": 3, "n": 10})
        with open(jl.path) as f:
            assert json.loads(f.readline())["map_version"] == 3

        tum = sw.tum("poses", "robot_fused.tum", comment="unit test")
        tum.append(1_500_000_000_000_000_000, (1, 2, 3), (0, 0, 0, 1))
        lines = open(tum.path).read().splitlines()
        assert lines[0] == "# unit test" and lines[1] == TumTrajectory.HEADER
        assert lines[2].split()[0] == "1500000000.000000000"
        assert lines[2].split()[1:4] == ["1.000000", "2.000000", "3.000000"]

        # ── Budget: refuses past the cap, counts the refusal ──
        b = Budget(max_bytes=100, duration_s=0.0, name="t")
        assert b.claim(60) and not b.bytes_exhausted()
        assert not b.claim(60) and b.refused == 1        # 60+60 > 100 → refused
        assert b.claim(40) and b.bytes_exhausted()
        assert b.expired()
        b2 = Budget(duration_s=-1.0)                     # 0/negative → uncapped
        assert not b2.expired()

        # ── RingBudget: never refuses, evicts oldest ──
        rd = os.path.join(tmp, "ring")
        os.makedirs(rd, exist_ok=True)
        r = RingBudget(max_bytes=100)
        paths = []
        for i in range(4):
            fp = os.path.join(rd, f"{i}.bin")
            with open(fp, "wb") as f:
                f.write(b"x" * 40)
            assert r.claim(40)
            gone = list(r.track(fp, 40))
            paths.append(fp)
            if i >= 2:
                assert gone, f"expected eviction at i={i}"
        assert not os.path.exists(paths[0])              # oldest gone
        assert os.path.exists(paths[-1])                 # newest kept
        assert r.summary()["evicted_files"] >= 1
        assert r.bytes_written <= 100

        # ── StreamStats: gap accounting, and backwards seq is not a 4-billion gap ──
        st = StreamStats("frames", "go2/prism/camera/frame")
        for i, seq in enumerate([10, 11, 14, 15, 12]):
            st.sample(nbytes=1000, src_ts_ns=1_000_000_000 * (i + 1),
                      wall_ns=1, seq=seq)
        s = st.summary()
        assert s["samples"] == 5 and s["bytes"] == 5000
        assert s["seq_gap_events"] == 1 and s["seq_samples_missing"] == 2   # 12,13
        assert s["seq_backwards"] == 1                                     # 15 → 12
        assert s["src_ts_span_s"] == 4.0 and s["mean_hz"] == 1.0   # 4 intervals / 4 s
        st.error("boom")
        assert st.summary()["errors"] == 1

        # ── writing after close is a counted no-op, never an exception ──
        # A straggler worker finishing after shutdown must not raise inside a Zenoh
        # callback (where the traceback is swallowed) nor abort finalisation.
        for w, call in ((idx, lambda: idx.append([2, 456])),
                        (jl, lambda: jl.append({"map_version": 9})),
                        (tum, lambda: tum.append(1, (0, 0, 0), (0, 0, 0, 1)))):
            before = w.rows
            w.close()
            call()
            call()
            assert w.rows == before, type(w).__name__
            assert w.dropped_after_close == 2, type(w).__name__

        sw.close()
        print("rec_sinks self-test OK  (atomic blobs, per-row flush, ring eviction, "
              "gap accounting, no-op after close)")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    _selftest()
