"""
VAT robot — full-res frame archive (SQLite index + JPEG files on disk)
======================================================================
Each frame the decimator transmits (downscaled) to the mapping server also has
its **full-resolution twin** archived here, tagged with the SAME
``seq`` / ``ts_ns`` / ``camera_height`` — a 1:1 mapping. This is the source for
offline, non-real-time reconstruction (Gaussian splats, NeRF, photogrammetry):
fetch frames by ``seq``, join them against the recorded pose trajectory by
timestamp, and run the heavy algorithm later.

Design
------
* **SQLite is the index, JPEGs are files on disk** (``<dir>/frames/<seq>.jpg``).
  Random access by ``seq``, trivial FIFO eviction (delete row + unlink file),
  and the DB file stays tiny. Chosen over blobs-in-SQLite so a 10 GB rolling
  archive never needs a painful ``VACUUM``.
* **Decoupled writer thread + bounded queue.** ``submit()`` is non-blocking and
  *drops* the frame if the queue is full, so disk I/O or a 4K JPEG encode can
  never stall the real-time Zenoh publish path. Realtime is sacred; the archive
  is best-effort.
* **Rolling size cap** (``ARCHIVE_MAX_BYTES``): when total bytes exceed the cap,
  the oldest frames are evicted first.

On-demand fetch: ``get(seq)`` returns a ``vat_protocol.pack_frame()`` payload
(full-res JPEG + the original ts/seq/cam_h), so a Zenoh queryable can reply with
it and the requester decodes it with the usual ``unpack_frame()``.
"""

from __future__ import annotations

import os
import queue
import sqlite3
import threading
import time
import logging
from typing import Optional

import cv2
import numpy as np

import vat_protocol as proto

log = logging.getLogger("frame-archive")

_SUFFIXES = (
    ("TIB", 1024**4), ("TB", 1024**4), ("T", 1024**4),
    ("GIB", 1024**3), ("GB", 1024**3), ("G", 1024**3),
    ("MIB", 1024**2), ("MB", 1024**2), ("M", 1024**2),
    ("KIB", 1024),    ("KB", 1024),    ("K", 1024),
)


def parse_size(value) -> int:
    """Parse '10GB' / '500MB' / '10737418240' → bytes (binary multiples)."""
    s = str(value).strip().upper().replace(" ", "")
    if not s:
        return 0
    for suf, mult in _SUFFIXES:
        if s.endswith(suf):
            return int(float(s[: -len(suf)]) * mult)
    return int(float(s))


class FrameArchive:
    """Thread-safe, size-capped full-res frame archive."""

    def __init__(self, directory: str, max_bytes, *, jpeg_quality: int = 92,
                 queue_size: int = 8):
        self._dir = directory
        self._frames_dir = os.path.join(directory, "frames")
        self._db_path = os.path.join(directory, "index.sqlite")
        self._max_bytes = int(parse_size(max_bytes))
        self._jpeg_quality = int(jpeg_quality)

        self._queue: "queue.Queue" = queue.Queue(maxsize=queue_size)
        self._stop = threading.Event()
        self._lock = threading.Lock()        # serialises all DB access
        self._total = 0
        self._written = 0
        self._dropped = 0

        os.makedirs(self._frames_dir, exist_ok=True)
        self._db = sqlite3.connect(self._db_path, check_same_thread=False)
        self._init_db()
        self._total = self._recompute_total()
        self._evict_if_needed()              # in case the cap shrank since last run

        self._thread = threading.Thread(target=self._run, name="frame-archive",
                                        daemon=True)
        self._thread.start()
        log.info(f"[archive] {directory}  cap={self._max_bytes/1e9:.1f}GB  "
                 f"q={self._jpeg_quality}  start_total={self._total/1e6:.0f}MB")

    # ── setup ────────────────────────────────────────────────────────────────
    def _init_db(self):
        with self._lock:
            self._db.execute("PRAGMA journal_mode=WAL")
            self._db.execute("PRAGMA synchronous=NORMAL")
            self._db.execute(
                """CREATE TABLE IF NOT EXISTS frames(
                       seq        INTEGER PRIMARY KEY,
                       ts_ns      INTEGER NOT NULL,
                       cam_h      REAL    NOT NULL,
                       width      INTEGER NOT NULL,
                       height     INTEGER NOT NULL,
                       bytes      INTEGER NOT NULL,
                       path       TEXT    NOT NULL,
                       created_ns INTEGER NOT NULL)""")
            self._db.execute(
                "CREATE INDEX IF NOT EXISTS idx_created ON frames(created_ns)")
            self._db.commit()

    def _recompute_total(self) -> int:
        with self._lock:
            row = self._db.execute(
                "SELECT COALESCE(SUM(bytes), 0) FROM frames").fetchone()
        return int(row[0] or 0)

    # ── producer side (real-time thread) ──────────────────────────────────────
    def submit(self, seq: int, ts_ns: int, cam_h: float, bgr: np.ndarray,
               *, copy: bool = False) -> None:
        """Queue a full-res frame for archival. Non-blocking; drops under
        back-pressure so the caller's real-time path is never stalled.

        ``bgr`` is referenced, not copied, by default — safe because OpenCV's
        ``VideoCapture.read()`` hands out a fresh array per frame. Pass
        ``copy=True`` if your source reuses its buffer.
        """
        if self._stop.is_set():
            return
        try:
            self._queue.put_nowait(
                (int(seq), int(ts_ns), float(cam_h),
                 bgr.copy() if copy else bgr))
        except queue.Full:
            self._dropped += 1
            if self._dropped % 30 == 1:
                log.warning(f"[archive] queue full — dropping full-res frame "
                            f"(dropped={self._dropped}); disk/CPU can't keep up")

    # ── consumer side (writer thread) ─────────────────────────────────────────
    def _run(self):
        while not self._stop.is_set():
            try:
                item = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._write(*item)
            except Exception as e:            # never let the writer die
                log.warning(f"[archive] write failed: {e}")
            finally:
                self._queue.task_done()

    def _write(self, seq: int, ts_ns: int, cam_h: float, bgr: np.ndarray):
        h, w = bgr.shape[:2]
        ok, jb = cv2.imencode(".jpg", bgr,
                              [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        if not ok or jb is None:
            log.warning("[archive] encode failed — skipping frame")
            return
        data = jb.tobytes()
        rel = os.path.join("frames", f"{seq:09d}.jpg")
        dst = os.path.join(self._dir, rel)
        tmp = dst + ".tmp"
        with open(tmp, "wb") as f:
            f.write(data)
        os.replace(tmp, dst)                  # atomic publish of the file
        nbytes = len(data)

        with self._lock:
            old = self._db.execute(
                "SELECT bytes FROM frames WHERE seq=?", (seq,)).fetchone()
            if old is not None:
                self._total -= int(old[0])
            self._db.execute(
                "INSERT OR REPLACE INTO frames"
                "(seq, ts_ns, cam_h, width, height, bytes, path, created_ns)"
                " VALUES(?,?,?,?,?,?,?,?)",
                (seq, ts_ns, cam_h, w, h, nbytes, rel, time.time_ns()))
            self._db.commit()
            self._total += nbytes
            self._written += 1

        self._evict_if_needed()

    def _evict_if_needed(self):
        while self._total > self._max_bytes:
            with self._lock:
                row = self._db.execute(
                    "SELECT seq, bytes, path FROM frames "
                    "ORDER BY created_ns ASC, seq ASC LIMIT 1").fetchone()
                if row is None:
                    self._total = 0
                    break
                seq, b, rel = row
                self._db.execute("DELETE FROM frames WHERE seq=?", (seq,))
                self._db.commit()
                self._total -= int(b)
            try:
                os.remove(os.path.join(self._dir, rel))
            except OSError:
                pass

    # ── on-demand fetch (queryable thread) ────────────────────────────────────
    def get(self, seq: int) -> Optional[bytes]:
        """Return a ``pack_frame()`` payload (full-res JPEG + original
        ts/seq/cam_h) for ``seq``, or ``None`` if not archived."""
        with self._lock:
            row = self._db.execute(
                "SELECT ts_ns, cam_h, path FROM frames WHERE seq=?",
                (int(seq),)).fetchone()
        if row is None:
            return None
        ts_ns, cam_h, rel = row
        try:
            with open(os.path.join(self._dir, rel), "rb") as f:
                jpeg = f.read()
        except OSError:
            return None
        return proto.pack_frame(int(ts_ns), int(seq) & 0xFFFFFFFF,
                                float(cam_h), jpeg)

    def latest_seq(self) -> Optional[int]:
        with self._lock:
            row = self._db.execute("SELECT MAX(seq) FROM frames").fetchone()
        return int(row[0]) if row and row[0] is not None else None

    def count(self) -> int:
        with self._lock:
            return int(self._db.execute(
                "SELECT COUNT(*) FROM frames").fetchone()[0])

    def stats(self) -> str:
        return (f"frames={self.count()} "
                f"size={self._total/1e9:.2f}/{self._max_bytes/1e9:.0f}GB "
                f"written={self._written} dropped={self._dropped} "
                f"q={self._queue.qsize()}")

    def close(self):
        self._stop.set()
        try:
            self._thread.join(timeout=2.0)
        except Exception:
            pass
        with self._lock:
            try:
                self._db.commit()
                self._db.close()
            except Exception:
                pass
