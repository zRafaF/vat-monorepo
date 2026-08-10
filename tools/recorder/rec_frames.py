"""
VAT recorder — the 360° panorama streams.
========================================
Two recorders, both keyed on the **same ``seq``** the live pipeline uses, so a
transmit frame and its full-resolution twin are trivially joinable (see
``robot/docker/frame_archive.py``: "tagged with the SAME seq / ts_ns /
camera_height — a 1:1 mapping").

:class:`PanoramaTransmitRecorder`
    Subscribes ``{robot}/prism/camera/frame`` and stores each frame's encoded body
    **byte-exact**, plus the full wire size. That gives both the decodable frames
    and the real uplink characterisation (bytes/frame, effective Hz) the
    publication roadmap §3.2 asks for. Pure subscriber: zero effect on the link.

:class:`PanoramaFullresRecorder`
    The robot archives a full-resolution twin of every transmitted frame locally
    (``ARCHIVE_ENABLE=true``). This recorder learns the live ``seq`` numbers off the
    transmit stream and pulls the twins by ``seq`` from that **local** archive over
    ``{robot}/prism/camera/archive/get``. Run it on the robot (``--where robot``) so
    those bytes never touch the field link — that is the whole point of pulling
    from the archive instead of raising the transmit resolution.

    The archive's writer thread is asynchronous and *drops* frames under
    back-pressure, so pulls are deliberately lagged (``--fullres-lag``) and a miss
    is recorded as a skip rather than treated as an error.

Both write a ``frame_index.csv`` whose ``src_ts_ns`` column is the robot capture
timestamp straight off the ``FRME`` header — never an arrival time.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from collections import deque
from typing import Optional, Tuple

import rec_config as rcfg          # noqa: F401 — also puts repo/common on sys.path

import vat_protocol as proto       # noqa: E402  (needs rec_config's path insert)

from rec_base import StreamRecorder
from rec_clock import SessionClock
from rec_sinks import Budget, SessionWriter

log = logging.getLogger("vat-record")

_FRAME_COLUMNS = [
    "seq", "src_ts_ns", "ts_src", "wall_ns", "mono_ns",
    "wire_bytes", "image_bytes", "camera_height_m", "width", "height",
    "latency_ms", "file",
]

# JPEG Start-Of-Frame markers that carry the frame dimensions.
_JPEG_SOF = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
             0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}


def image_dims(buf: bytes) -> Tuple[int, int]:
    """``(width, height)`` from an encoded image header, or ``(0, 0)`` if unknown.

    Header-only parse for JPEG / WebP / PNG. Deliberately dependency-free: the
    recorder must not need OpenCV just to log a resolution, and it must not decode
    every frame on the capture path. The wire format is JPEG today
    (``vat_protocol.pack_frame`` → ``cv2.imencode('.jpg', …)``), but WebP is handled
    too because the roadmap describes the transmit stream as WebP — so a future
    codec swap keeps working instead of silently logging zeros.
    """
    n = len(buf)
    if n >= 4 and buf[0] == 0xFF and buf[1] == 0xD8:                 # JPEG
        i = 2
        while i + 9 < n:
            if buf[i] != 0xFF:
                i += 1
                continue
            m = buf[i + 1]
            if m == 0xFF:                                            # fill byte
                i += 1
                continue
            if m == 0x01 or 0xD0 <= m <= 0xD9:                       # standalone
                i += 2
                continue
            seglen = int.from_bytes(buf[i + 2:i + 4], "big")
            if m in _JPEG_SOF:
                return (int.from_bytes(buf[i + 7:i + 9], "big"),
                        int.from_bytes(buf[i + 5:i + 7], "big"))
            if seglen < 2:
                break
            i += 2 + seglen
        return 0, 0
    if n >= 16 and buf[:4] == b"RIFF" and buf[8:12] == b"WEBP":      # WebP
        fmt = buf[12:16]
        if fmt == b"VP8 " and n >= 30 and buf[23:26] == b"\x9d\x01\x2a":
            return (int.from_bytes(buf[26:28], "little") & 0x3FFF,
                    int.from_bytes(buf[28:30], "little") & 0x3FFF)
        if fmt == b"VP8L" and n >= 25:
            bits = int.from_bytes(buf[21:25], "little")
            return (bits & 0x3FFF) + 1, ((bits >> 14) & 0x3FFF) + 1
        if fmt == b"VP8X" and n >= 30:
            return (int.from_bytes(buf[24:27], "little") + 1,
                    int.from_bytes(buf[27:30], "little") + 1)
        return 0, 0
    if n >= 24 and buf[:8] == b"\x89PNG\r\n\x1a\n":                  # PNG
        return (int.from_bytes(buf[16:20], "big"),
                int.from_bytes(buf[20:24], "big"))
    return 0, 0


class _HeightTracker:
    """Running min/mean/max of the per-frame ``camera_height`` seen on the wire.

    The measured camera height is the metric-scale anchor (§3.2), and it travels on
    every ``FRME`` header. Recording the observed distribution means the anchor is
    captured even if the operator forgets to pass ``--camera-height``, and it
    cross-checks the value they did pass.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.n = 0
        self.n_unknown = 0          # wire value < 0 → robot said "unknown"
        self._sum = 0.0
        self.lo: Optional[float] = None
        self.hi: Optional[float] = None

    def add(self, h: float) -> None:
        with self._lock:
            if h is None or h <= 0.0:
                self.n_unknown += 1
                return
            self.n += 1
            self._sum += float(h)
            self.lo = h if self.lo is None else min(self.lo, h)
            self.hi = h if self.hi is None else max(self.hi, h)

    def summary(self) -> dict:
        with self._lock:
            return {
                "n": self.n, "n_unknown": self.n_unknown,
                "min_m": None if self.lo is None else round(self.lo, 4),
                "mean_m": None if not self.n else round(self._sum / self.n, 4),
                "max_m": None if self.hi is None else round(self.hi, 4),
            }


# ═════════════════════════════════════════════════════════════════════════════
# Transmit-resolution panorama
# ═════════════════════════════════════════════════════════════════════════════


class PanoramaTransmitRecorder(StreamRecorder):
    """The 360° stream exactly as the robot sends it to the cloud."""

    name = "panorama_transmit"

    def __init__(self, sw: SessionWriter, clock: SessionClock,
                 budget: Optional[Budget] = None, *, every: int = 1,
                 index_only: bool = False, seq_sink=None):
        super().__init__(sw, clock, budget)
        self.every = max(1, int(every))
        self.index_only = bool(index_only)
        #: optional callback ``(seq, ts_ns, cam_h, wall_ns)`` — how the full-res
        #: puller learns which seqs exist without a second subscription.
        self.seq_sink = seq_sink
        self.height = _HeightTracker()
        self._frames_dir = None if index_only else sw.subdir(self.name, "frames")
        self._idx = sw.csv_index(self.name, "frame_index.csv", columns=_FRAME_COLUMNS)
        self._n_seen = 0
        self.stats.key = rcfg.KEYS["camera_frame"]

    def attach(self, z) -> None:
        super().attach(z)
        self.subscribe(rcfg.KEYS["camera_frame"], self._on_frame)

    def _on_frame(self, sample) -> None:
        raw = bytes(sample.payload)
        ts_ns, seq, cam_h, body = proto.unpack_frame(raw)
        st = self.clock.stamp(ts_ns)
        self.height.add(cam_h)
        if self.seq_sink is not None:
            try:
                self.seq_sink(int(seq), int(ts_ns), float(cam_h), st.wall_ns)
            except Exception as e:                       # never let a sink break us
                self.stats.error(f"seq_sink: {e}")

        self._n_seen += 1
        if (self._n_seen - 1) % self.every:              # keep the first, then every Nth
            self.stats.skip()
            return
        if self.budget.expired():
            self.stats.skip()
            return

        w, h = image_dims(body)
        rel = ""
        if not self.index_only:
            if not self.budget.claim(len(body)):
                self.stats.skip()
                return
            try:
                path, _ = self.sw.write_blob(body, self.name, "frames", f"{seq:09d}.jpg")
            except OSError as e:
                self.budget.release(len(body))
                self.stats.error(f"write: {e}")
                return
            rel = self.sw.rel(path)

        self._idx.append([seq, ts_ns, st.ts_src, st.wall_ns, st.mono_ns,
                          len(raw), len(body), f"{cam_h:.4f}", w, h,
                          f"{st.latency_ms:.1f}", rel])
        # Only track seq gaps when keeping every frame: with --transmit-every N the seq
        # sequence skips by design, and reporting that as loss would be a lie.
        self.stats.sample(nbytes=len(raw), src_ts_ns=ts_ns, wall_ns=st.wall_ns,
                          seq=(seq if self.every == 1 else None))

    def extra_summary(self) -> dict:
        return {
            "index": self.sw.rel(self._idx.path),
            "frames_dir": None if self.index_only else f"{self.name}/frames",
            "index_only": self.index_only,
            "decimate_every": self.every,
            "seq_gaps_tracked": self.every == 1,
            "camera_height_wire": self.height.summary(),
            "note": ("wire_bytes = full Zenoh payload (20-byte FRME header + image); "
                     "image_bytes = encoded image only. Use wire_bytes for uplink."),
        }

    def status_line(self) -> str:
        s = self.stats.summary()
        hz = s["mean_hz"] or 0.0
        return f"pano={s['samples']}@{hz:.1f}Hz"


# ═════════════════════════════════════════════════════════════════════════════
# Full-resolution panorama (pulled from the robot's local archive)
# ═════════════════════════════════════════════════════════════════════════════


class PanoramaFullresRecorder(StreamRecorder):
    """Pull full-res twins by ``seq`` from the robot's on-board frame archive."""

    name = "panorama_fullres"
    # Coverage is inherently partial — --fullres-every decimates it, the ring budget
    # evicts the oldest, and early pulls can miss while the archive warms up — so it
    # must NOT define the window in which everything is available.
    dense = False

    def __init__(self, sw: SessionWriter, clock: SessionClock,
                 budget: Optional[Budget] = None, *, every: int = 1,
                 lag_s: float = 2.0, timeout_s: float = 5.0,
                 queue_max: int = 512, own_subscription: bool = True):
        super().__init__(sw, clock, budget)
        self.every = max(1, int(every))
        self.lag_s = float(lag_s)
        self.timeout_s = float(timeout_s)
        self.own_subscription = bool(own_subscription)
        self._key = rcfg.KEYS["camera_archive_get"]
        self.stats.key = self._key
        self.note_query(self._key)
        self.height = _HeightTracker()

        self.sw.subdir(self.name, "frames")
        self._idx = sw.csv_index(self.name, "frame_index.csv", columns=_FRAME_COLUMNS)
        # (seq, ts_ns, enqueued_mono) — bounded so a stalled archive can't grow RAM.
        self._q: deque = deque(maxlen=int(queue_max))
        self._q_lock = threading.Lock()
        self._stop = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._n_seen = 0
        self.n_missing = 0            # archive replied "not found"
        self.n_timeout = 0
        self.n_dropped_queue = 0
        self._evicted = []            # blobs the ring budget removed after indexing

    # ── seq intake ───────────────────────────────────────────────────────────
    def offer(self, seq: int, ts_ns: int, cam_h: float, wall_ns: int) -> None:
        """Note that ``seq`` exists and should be pulled (respecting decimation)."""
        self._n_seen += 1
        if (self._n_seen - 1) % self.every:
            self.stats.skip()
            return
        dropped = None
        with self._q_lock:
            if len(self._q) == self._q.maxlen:
                # A bounded deque discards the OLDEST on append; do it explicitly so
                # we can name the seq we actually lost and count it as a skip. Dropping
                # the oldest is the right choice — the newest frames are the ones still
                # inside the robot's rolling archive.
                dropped = self._q.popleft()[0]
                self.n_dropped_queue += 1
            self._q.append((int(seq), int(ts_ns), time.monotonic()))
        if dropped is not None:
            self.stats.skip()
            if self.n_dropped_queue % 25 == 1:
                log.warning(f"[{self.name}] pull queue full — discarded seq={dropped} "
                            f"unpulled (archive/link can't keep up; dropped="
                            f"{self.n_dropped_queue}). Raise --fullres-every or lower "
                            f"the capture rate.")

    def _on_frame(self, sample) -> None:
        """Only used when this recorder owns its own subscription to the live feed."""
        raw = bytes(sample.payload)
        ts_ns, seq, cam_h, _body = proto.unpack_frame(raw)
        self.offer(int(seq), int(ts_ns), float(cam_h), self.clock.stamp(ts_ns).wall_ns)

    def attach(self, z) -> None:
        super().attach(z)
        if self.own_subscription:
            self.subscribe(rcfg.KEYS["camera_frame"], self._on_frame)
        log.info(f"[{self.name}] ? '{self._key}'  (query/reply; lag={self.lag_s}s "
                 f"every={self.every})")
        self._worker = threading.Thread(target=self._run, name="fullres-pull",
                                        daemon=True)
        self._worker.start()

    # ── worker ───────────────────────────────────────────────────────────────
    def _run(self) -> None:
        while not self._stop.is_set():
            item = None
            with self._q_lock:
                if self._q:
                    seq, ts_ns, enq = self._q[0]
                    # Wait out the archive's async writer before asking for the frame.
                    if time.monotonic() - enq >= self.lag_s:
                        item = self._q.popleft()
            if item is None:
                self._stop.wait(0.1)
                continue
            if self.budget.expired():
                self.stats.skip()
                continue
            try:
                self._pull(item[0])
            except Exception as e:                       # noqa: BLE001
                self.stats.error(f"{type(e).__name__}: {e}")
                log.debug(f"[{self.name}] pull failed", exc_info=True)

    def _pull(self, seq: int) -> None:
        sel = f"{self._key}?seq={seq}"
        payload = None
        err = None
        for reply in self._z.get(sel, timeout=self.timeout_s):
            if reply.ok:
                payload = bytes(reply.result.payload)
                break
            try:
                err = bytes(reply.err.payload).decode(errors="replace")
            except Exception:
                err = "error reply"
        if self._stop.is_set():
            # We were stopping while this query was in flight (it can block for
            # --fullres-timeout). Discard the reply rather than write a blob the
            # already-finalised index will never mention.
            self.stats.skip()
            return
        if payload is None:
            if err:
                self.n_missing += 1
                if self.n_missing % 25 == 1:
                    log.warning(f"[{self.name}] archive miss seq={seq}: {err} "
                                f"(misses={self.n_missing}) — ARCHIVE_ENABLE=true? "
                                f"still inside the rolling window?")
            else:
                self.n_timeout += 1
                if self.n_timeout % 25 == 1:
                    log.warning(f"[{self.name}] no reply for seq={seq} within "
                                f"{self.timeout_s}s (timeouts={self.n_timeout})")
            self.stats.skip()
            return

        # The archive replies with a pack_frame() payload carrying the ORIGINAL
        # ts/seq/camera_height, so the full-res twin keeps the transmit frame's
        # identity on the session clock — we never re-stamp it.
        a_ts_ns, a_seq, a_cam_h, body = proto.unpack_frame(payload)
        # observe=False: this frame is pulled --fullres-lag seconds after capture, so
        # `arrival - capture` is dominated by our own deliberate lag. Feeding it to the
        # clock's running-minimum baseline would inflate the offset by seconds.
        st = self.clock.stamp(a_ts_ns, observe=False)
        self.height.add(a_cam_h)
        if not self.budget.claim(len(body)):
            self.stats.skip()
            return
        try:
            path, _ = self.sw.write_blob(body, self.name, "frames", f"{a_seq:09d}.jpg")
        except OSError as e:
            self.budget.release(len(body))
            self.stats.error(f"write: {e}")
            return
        track = getattr(self.budget, "track", None)
        if callable(track):                              # RingBudget: evict oldest
            for gone in track(path, len(body)):
                # The index row for an evicted frame stays (it is still a true record
                # of what the robot captured and what it cost), but the blob is gone —
                # so log the eviction explicitly and report the count. compose.py drops
                # rows whose blob is missing when it loads a recording.
                self._evicted.append(self.sw.rel(gone))
                if len(self._evicted) % 100 == 1:
                    log.info(f"[{self.name}] ring cap reached — evicted "
                             f"{len(self._evicted)} oldest frame(s); index rows are "
                             f"kept and compose.py ignores rows with no blob")
        w, h = image_dims(body)
        self._idx.append([a_seq, a_ts_ns, st.ts_src, st.wall_ns, st.mono_ns,
                          len(payload), len(body), f"{a_cam_h:.4f}", w, h,
                          f"{st.latency_ms:.1f}", self.sw.rel(path)])
        self.stats.sample(nbytes=len(payload), src_ts_ns=a_ts_ns,
                          wall_ns=st.wall_ns,
                          seq=(a_seq if self.every == 1 else None))

    def close(self) -> None:
        self._stop.set()
        if self._worker is not None:
            self._worker.join(timeout=3.0)
        with self._q_lock:
            pending = len(self._q)
        if pending:
            log.info(f"[{self.name}] {pending} queued seq(s) not pulled at stop "
                     f"(recorded as skipped)")
            self.stats.skip(pending)
        super().close()

    def extra_summary(self) -> dict:
        with self._q_lock:
            pending = len(self._q)
        return {
            "index": self.sw.rel(self._idx.path),
            "frames_dir": f"{self.name}/frames",
            "decimate_every": self.every,
            "seq_gaps_tracked": self.every == 1,
            "pull_lag_s": self.lag_s,
            "archive_misses": self.n_missing,
            "query_timeouts": self.n_timeout,
            "queue_overflow_drops": self.n_dropped_queue,
            "unpulled_at_stop": pending,
            "evicted_by_ring": len(self._evicted),
            "evicted_files_sample": self._evicted[:20],
            "camera_height_wire": self.height.summary(),
            "note": ("latency_ms on this stream is the archive-pull age (≈ "
                     "--fullres-lag plus the query round trip), NOT a transport "
                     "latency — the frame itself was captured at src_ts_ns. Rows whose "
                     "blob the ring evicted are kept in the index; compose.py ignores "
                     "them when loading."),
        }

    def status_line(self) -> str:
        return f"fullres={self.stats.n}(-{self.n_missing})"


# ═════════════════════════════════════════════════════════════════════════════
# Self-test:  python tools/recorder/rec_frames.py
# ═════════════════════════════════════════════════════════════════════════════

def _selftest() -> None:
    import shutil
    import struct
    import tempfile

    # ── image_dims: real JPEG (via cv2 when present) + hand-built headers ──
    try:
        import cv2
        import numpy as np
        ok, jb = cv2.imencode(".jpg", np.zeros((518, 1036, 3), np.uint8))
        assert ok
        assert image_dims(jb.tobytes()) == (1036, 518)
        ok, pb = cv2.imencode(".png", np.zeros((7, 13, 3), np.uint8))
        assert ok and image_dims(pb.tobytes()) == (13, 7)
        real_jpeg = jb.tobytes()
    except ImportError:                        # cv2 optional for the self-test
        real_jpeg = b"\xff\xd8\xff\xc0\x00\x11\x08\x02\x06\x04\x0c" + b"\x00" * 8
        assert image_dims(real_jpeg) == (1036, 518)

    webp_vp8 = (b"RIFF" + b"\x00" * 4 + b"WEBPVP8 " + b"\x00" * 4
                + b"\x00\x00\x00" + b"\x9d\x01\x2a"
                + (1036).to_bytes(2, "little") + (518).to_bytes(2, "little"))
    assert image_dims(webp_vp8) == (1036, 518)
    assert image_dims(b"nonsense") == (0, 0)
    assert image_dims(b"") == (0, 0)

    tmp = tempfile.mkdtemp(prefix="vatrec-frames-")
    try:
        sw = SessionWriter(tmp, "s")
        clock = SessionClock()
        seen = []
        rec = PanoramaTransmitRecorder(sw, clock, Budget(name="t"), every=1,
                                       seq_sink=lambda *a: seen.append(a))

        class _S:                                     # stand-in for zenoh.Sample
            def __init__(self, payload):
                self.payload = payload

        base_ns = 1_700_000_000_000_000_000
        for i in range(5):
            wire = proto.pack_frame(base_ns + i * 400_000_000, 100 + i, 1.15, real_jpeg)
            rec._on_frame(_S(wire))

        s = rec.summary()
        assert s["samples"] == 5, s
        assert s["seq_gap_events"] == 0
        # capture timestamps recorded verbatim on the session clock
        assert s["first_src_ts_ns"] == base_ns
        assert s["last_src_ts_ns"] == base_ns + 4 * 400_000_000
        assert abs(s["mean_hz"] - 2.5) < 1e-6           # 4 intervals over 1.6 s
        assert s["camera_height_wire"]["mean_m"] == 1.15
        assert len(seen) == 5 and seen[0][0] == 100
        # blobs are byte-exact
        with open(sw.path("panorama_transmit", "frames", "000000100.jpg"), "rb") as f:
            assert f.read() == real_jpeg
        # index rows carry the source timestamp and the full wire size
        rows = open(rec._idx.path).read().splitlines()
        assert rows[0].split(",")[:3] == ["seq", "src_ts_ns", "ts_src"]
        first = rows[1].split(",")
        assert first[0] == "100" and first[1] == str(base_ns) and first[2] == "source"
        assert int(first[5]) == len(real_jpeg) + struct.calcsize("!iqIf")
        assert first[8] == "1036" and first[9] == "518"

        # decimation keeps the first frame then every Nth
        rec2 = PanoramaTransmitRecorder(SessionWriter(tmp, "s2"), SessionClock(),
                                        Budget(name="t2"), every=2)
        for i in range(5):
            rec2._on_frame(_S(proto.pack_frame(base_ns + i, i, 1.0, real_jpeg)))
        assert rec2.summary()["samples"] == 3 and rec2.summary()["skipped"] == 2

        # index_only mode writes rows but no image files
        rec3 = PanoramaTransmitRecorder(SessionWriter(tmp, "s3"), SessionClock(),
                                        index_only=True)
        rec3._on_frame(_S(proto.pack_frame(base_ns, 1, 1.0, real_jpeg)))
        assert rec3.summary()["samples"] == 1
        assert not os.path.exists(os.path.join(tmp, "s3", "panorama_transmit", "frames"))

        # a corrupt sample is counted, not fatal (subscribe() wrapper does this live)
        try:
            rec._on_frame(_S(b"\x00\x00\x00\x01short"))
        except proto.ProtocolError:
            pass
        else:
            raise AssertionError("expected ProtocolError from a bad frame")

        # full-res queue decimation + bounded queue
        fr = PanoramaFullresRecorder(sw, clock, Budget(name="f"), every=3,
                                     queue_max=4, own_subscription=False)
        for i in range(9):
            fr.offer(i, base_ns + i, 1.15, 0)
        with fr._q_lock:
            assert len(fr._q) == 3, len(fr._q)          # 0, 3, 6
            assert [q[0] for q in fr._q] == [0, 3, 6]
        assert fr.summary()["skipped"] == 6
        # full-res is NOT dense: it is decimated / ring-evicted, so it must never
        # bound the composable window
        assert fr.dense is False and rec.dense is True
        fr.close()

        # a full queue must drop the OLDEST, count it as a skip, and name what it lost
        fr2 = PanoramaFullresRecorder(SessionWriter(tmp, "s4"), SessionClock(),
                                      Budget(name="f2"), queue_max=3,
                                      own_subscription=False)
        for i in range(7):
            fr2.offer(i, base_ns + i, 1.15, 0)
        with fr2._q_lock:
            assert [q[0] for q in fr2._q] == [4, 5, 6], [q[0] for q in fr2._q]
        assert fr2.n_dropped_queue == 4
        assert fr2.summary()["skipped"] == 4, fr2.summary()["skipped"]
        fr2.close()

        sw.close()
        print("rec_frames self-test OK  (dims parse, verbatim capture ts, byte-exact "
              "blobs, decimation, bounded pull queue with accounted drops)")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    _selftest()
