"""
VAT recorder — the remote periscope video slice.
===============================================
The periscope is the one stream the recorder **cannot conjure**: the robot only
encodes and publishes while an operator client keeps a ``ViewRequest`` alive
(``PERISCOPE_VIEWER_TIMEOUT_S``, 5 s by default — see ``docs/periscope.md``). A
passive recorder therefore captures the periscope *as the operator actually used
it*, which is exactly what a real-world demo video wants. The recorder never
publishes a ``ViewRequest`` and never asks for a keyframe: doing either would
change what the robot encodes and what the live client sees.

Consequences, all recorded rather than hidden:

* Recording may start mid-GOP. H.264/HEVC cannot be decoded before the first
  ``is_keyframe`` frame, so ``frames_before_first_keyframe`` is reported and the
  index flags every frame's keyframe bit.
* The codec is a per-frame header field (``PSCOPE_CODEC_MJPEG`` / ``H264`` /
  ``HEVC``), not an assumption. If it changes mid-session the recorder rolls to a
  new elementary-stream segment instead of writing an undecodable mix.

What lands on disk
------------------
``periscope/periscope.h264`` (or ``.hevc`` / ``.mjpeg``)
    The elementary stream, byte-exact: every frame's payload concatenated in
    arrival order. Annex-B NAL units for H.26x; whole JPEGs for MJPEG.
``periscope_timestamps.csv``
    One row per frame with the **capture** timestamp from the ``PSCF`` header plus
    the frame's ``(segment, byte_offset, byte_len)`` — so any frame can be sliced
    out of the elementary stream exactly, and the video re-syncs with the poses
    and the map on the session clock. This CSV, not the mp4's internal PTS, is
    authoritative for timing.
``periscope.mp4``
    Convenience remux via ``ffmpeg -c copy`` at the measured mean rate. Nominal
    timing only (the stream is variable-rate); use the CSV for alignment.
"""

from __future__ import annotations

import logging
import os
import shutil
import subprocess
from typing import Optional

import rec_config as rcfg          # noqa: F401 — also puts repo/common on sys.path

import vat_protocol as proto       # noqa: E402  (needs rec_config's path insert)

from rec_base import StreamRecorder
from rec_clock import SessionClock
from rec_sinks import Budget, SessionWriter

log = logging.getLogger("vat-record")

CODEC_NAME = {
    proto.PSCOPE_CODEC_MJPEG: "mjpeg",
    proto.PSCOPE_CODEC_H264: "h264",
    proto.PSCOPE_CODEC_HEVC: "hevc",
}
#: container extension + the ffmpeg demuxer name for each elementary stream
CODEC_EXT = {"mjpeg": "mjpeg", "h264": "h264", "hevc": "hevc"}

_COLUMNS = [
    "seq", "src_ts_ns", "ts_src", "wall_ns", "mono_ns", "latency_ms",
    "codec", "keyframe", "width", "height", "native_w", "optical",
    "yaw_deg", "pitch_deg", "hfov_deg", "vfov_deg", "aspect",
    "bytes", "segment", "byte_offset", "byte_len",
]


class PeriscopeRecorder(StreamRecorder):
    """Record the encoded periscope slice + a frame→timestamp sidecar."""

    name = "periscope"

    def __init__(self, sw: SessionWriter, clock: SessionClock,
                 budget: Optional[Budget] = None, *, mux_mp4: bool = True):
        super().__init__(sw, clock, budget)
        self.mux_mp4 = bool(mux_mp4)
        self._dir = sw.subdir(self.name)
        # The timestamps sidecar sits at the session root (roadmap layout) because
        # it is the artefact a video editor reaches for; the elementary streams live
        # in periscope/ next to the muxed mp4.
        self._idx = sw.csv_index("periscope_timestamps.csv", columns=_COLUMNS)
        self._seg_i = 0
        self._seg_codec: Optional[str] = None
        self._seg_path: Optional[str] = None
        self._seg_f = None
        self._seg_offset = 0
        self._segments = []           # [{file, codec, frames, bytes, first/last ts}]
        self.n_keyframes = 0
        self.frames_before_first_keyframe = 0
        self._have_keyframe = False
        self.stats.key = rcfg.KEYS["periscope_frame"]

    def attach(self, z) -> None:
        super().attach(z)
        self.subscribe(rcfg.KEYS["periscope_frame"], self._on_frame)

    # ── segment handling ─────────────────────────────────────────────────────
    def _open_segment(self, codec_name: str) -> None:
        self._close_segment()
        self._seg_i += 1
        ext = CODEC_EXT.get(codec_name, "bin")
        suffix = "" if self._seg_i == 1 else f"_{self._seg_i}"
        name = f"periscope{suffix}.{ext}"
        self._seg_path = os.path.join(self._dir, name)
        self._seg_f = open(self._seg_path, "wb")
        self._seg_codec = codec_name
        self._seg_offset = 0
        self._segments.append({
            "file": self.sw.rel(self._seg_path), "codec": codec_name,
            "frames": 0, "bytes": 0, "keyframes": 0,
            "first_src_ts_ns": None, "last_src_ts_ns": None,
        })
        log.info(f"[{self.name}] segment {self._seg_i}: {name} (codec={codec_name})")

    def _close_segment(self) -> None:
        if self._seg_f is not None:
            try:
                self._seg_f.flush()
                self._seg_f.close()
            except Exception:
                pass
        self._seg_f = None

    # ── sample handler ───────────────────────────────────────────────────────
    def _on_frame(self, sample) -> None:
        raw = bytes(sample.payload)
        f = proto.unpack_periscope_frame(raw)
        st = self.clock.stamp(f.timestamp_ns)
        codec = CODEC_NAME.get(f.codec, f"codec{f.codec}")
        payload = bytes(f.payload)

        if f.is_keyframe:
            self.n_keyframes += 1
            self._have_keyframe = True
        elif not self._have_keyframe:
            # Undecodable until the first IDR. Counted, never silently dropped —
            # and we do NOT request a keyframe, which would perturb the session.
            self.frames_before_first_keyframe += 1

        if self._seg_f is None or codec != self._seg_codec:
            if self._seg_codec is not None:
                log.warning(f"[{self.name}] codec changed {self._seg_codec}→{codec} "
                            f"— rolling to a new segment")
            self._open_segment(codec)

        if self.budget.expired() or not self.budget.claim(len(payload)):
            self.stats.skip()
            return
        # Take the offset from the file itself, never from a counter we maintain: a
        # buffered flush can fail (ENOSPC) *after* some bytes reached the file, and an
        # offset counter that skipped that frame would then be wrong for every
        # subsequent row — silently, in the one file the docstring calls authoritative.
        # tell() is self-correcting, so a failed write costs exactly one frame.
        try:
            offset = self._seg_f.tell()
            self._seg_f.write(payload)
            self._seg_f.flush()          # a killed recording keeps every whole frame
            end = self._seg_f.tell()
        except OSError as e:
            self.budget.release(len(payload))
            self.stats.error(f"write: {e}")
            try:
                self._seg_offset = self._seg_f.tell()
            except OSError:
                pass
            return
        if end - offset != len(payload):     # short write: do not index a partial frame
            self.stats.error(f"short write: {end - offset} of {len(payload)} bytes")
            self._seg_offset = end
            return
        self._seg_offset = end

        seg = self._segments[-1]
        seg["frames"] += 1
        seg["bytes"] += len(payload)
        seg["keyframes"] += 1 if f.is_keyframe else 0
        if seg["first_src_ts_ns"] is None:
            seg["first_src_ts_ns"] = int(f.timestamp_ns)
        seg["last_src_ts_ns"] = int(f.timestamp_ns)

        self._idx.append([
            f.seq, f.timestamp_ns, st.ts_src, st.wall_ns, st.mono_ns,
            f"{st.latency_ms:.1f}", codec, int(bool(f.is_keyframe)),
            f.width, f.height, f.native_w, int(bool(f.optical)),
            f"{f.yaw_deg:.3f}", f"{f.pitch_deg:.3f}",
            f"{f.hfov_deg:.3f}", f"{f.vfov_deg:.3f}",
            f"{f.aspect_w}:{f.aspect_h}",
            len(raw), seg["file"], offset, len(payload),
        ])
        self.stats.sample(nbytes=len(raw), src_ts_ns=f.timestamp_ns,
                          wall_ns=st.wall_ns, seq=f.seq)

    # ── finalisation ─────────────────────────────────────────────────────────
    def close(self) -> None:
        self._close_segment()
        if self.mux_mp4:
            for i, seg in enumerate(self._segments):
                if seg["frames"] > 0:
                    seg["mp4"] = self._mux(seg, primary=(i == 0))
        super().close()

    def _mean_fps(self, seg: dict) -> float:
        a, b = seg.get("first_src_ts_ns"), seg.get("last_src_ts_ns")
        if not a or not b or b <= a or seg["frames"] < 2:
            return 15.0                                  # PERISCOPE_FPS default
        span_s = (b - a) / 1e9
        return max(1.0, min(120.0, (seg["frames"] - 1) / span_s))

    def _mux(self, seg: dict, primary: bool) -> Optional[str]:
        """Remux one elementary stream to mp4 with ``ffmpeg -c copy``.

        Best-effort by design: the elementary stream plus
        ``periscope_timestamps.csv`` is the authoritative record, and the mp4 is a
        convenience for eyeballing / dropping into an editor. A missing ffmpeg is a
        log line, never a failed recording.
        """
        if shutil.which("ffmpeg") is None:
            log.info(f"[{self.name}] ffmpeg not found — kept the elementary stream "
                     f"({seg['file']}); mux it later with:  ffmpeg -r <fps> -f "
                     f"{seg['codec']} -i {seg['file']} -c copy periscope.mp4")
            return None
        src = self.sw.path(seg["file"])
        dst = (self.sw.path("periscope.mp4") if primary
               else os.path.splitext(src)[0] + ".mp4")
        fps = self._mean_fps(seg)
        cmd = ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
               "-fflags", "+genpts", "-r", f"{fps:.4f}",
               "-f", seg["codec"], "-i", src, "-c", "copy", dst]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        except Exception as e:                            # noqa: BLE001
            log.warning(f"[{self.name}] mux failed to launch: {e}")
            return None
        if r.returncode != 0 or not os.path.exists(dst):
            log.warning(f"[{self.name}] ffmpeg mux failed (rc={r.returncode}): "
                        f"{(r.stderr or '').strip()[:200]}")
            return None
        log.info(f"[{self.name}] muxed {self.sw.rel(dst)} at nominal {fps:.2f} fps "
                 f"(exact times: periscope_timestamps.csv)")
        return self.sw.rel(dst)

    def extra_summary(self) -> dict:
        return {
            "index": "periscope_timestamps.csv",
            "segments": self._segments,
            "keyframes": self.n_keyframes,
            "frames_before_first_keyframe": self.frames_before_first_keyframe,
            "passive": True,
            "note": ("The robot streams the periscope only while an operator client "
                     "keeps a ViewRequest alive; the recorder publishes nothing, so "
                     "an empty capture means no operator was aiming the periscope. "
                     "periscope_timestamps.csv is authoritative for timing; the mp4 "
                     "is a nominal-rate convenience remux."),
        }

    def status_line(self) -> str:
        return f"peri={self.stats.n}/kf{self.n_keyframes}"


# ═════════════════════════════════════════════════════════════════════════════
# Self-test:  python tools/recorder/rec_periscope.py
# ═════════════════════════════════════════════════════════════════════════════

def _selftest() -> None:
    import shutil as _sh
    import tempfile

    class _S:
        def __init__(self, payload):
            self.payload = payload

    tmp = tempfile.mkdtemp(prefix="vatrec-peri-")
    try:
        sw = SessionWriter(tmp, "s")
        rec = PeriscopeRecorder(sw, SessionClock(), Budget(name="p"), mux_mp4=False)
        base = 1_700_000_000_000_000_000

        # Start mid-GOP: two delta frames arrive before the first keyframe.
        payloads = []
        for i in range(6):
            body = bytes([i]) * (100 + i)
            payloads.append(body)
            rec._on_frame(_S(proto.pack_periscope_frame(proto.PeriscopeFrame(
                seq=i, timestamp_ns=base + i * 66_666_666,
                codec=proto.PSCOPE_CODEC_H264, is_keyframe=(i >= 2),
                width=640, height=480, native_w=640, yaw_deg=10.0, pitch_deg=-5.0,
                hfov_deg=60.0, vfov_deg=45.0, aspect_w=4, aspect_h=3,
                optical=True, payload=body))))

        s = rec.summary()
        assert s["samples"] == 6
        assert s["frames_before_first_keyframe"] == 2, s
        assert s["keyframes"] == 4
        assert len(s["segments"]) == 1 and s["segments"][0]["codec"] == "h264"

        # The elementary stream is exactly the concatenated payloads...
        es = sw.path("periscope", "periscope.h264")
        blob = open(es, "rb").read()
        assert blob == b"".join(payloads)
        # ...and every CSV row slices its own frame back out byte-exactly.
        rows = [r.split(",") for r in open(rec._idx.path).read().splitlines()]
        assert rows[0] == _COLUMNS
        cols = {c: i for i, c in enumerate(_COLUMNS)}
        for i, row in enumerate(rows[1:]):
            off, ln = int(row[cols["byte_offset"]]), int(row[cols["byte_len"]])
            assert blob[off:off + ln] == payloads[i]
            assert int(row[cols["src_ts_ns"]]) == base + i * 66_666_666
            assert row[cols["ts_src"]] == "source"
            assert row[cols["aspect"]] == "4:3" and row[cols["codec"]] == "h264"
        assert int(rows[1][cols["keyframe"]]) == 0
        assert int(rows[3][cols["keyframe"]]) == 1

        # ~15 fps from a 66.67 ms spacing
        assert abs(rec._mean_fps(rec._segments[0]) - 15.0) < 0.1

        # A mid-session codec change rolls to a new segment rather than mixing.
        rec._on_frame(_S(proto.pack_periscope_frame(proto.PeriscopeFrame(
            seq=99, timestamp_ns=base + 10**9, codec=proto.PSCOPE_CODEC_MJPEG,
            is_keyframe=True, width=320, height=240, payload=b"\xff\xd8jpg"))))
        s2 = rec.summary()
        assert len(s2["segments"]) == 2
        assert s2["segments"][1]["codec"] == "mjpeg"
        assert s2["segments"][1]["file"].endswith("periscope_2.mjpeg")
        assert open(sw.path("periscope", "periscope_2.mjpeg"), "rb").read() == b"\xff\xd8jpg"

        rec.close()

        # ── a failed flush must cost exactly ONE frame, not desynchronise every
        #    subsequent byte_offset (offsets come from tell(), not a counter) ──
        sw2 = SessionWriter(tmp, "s2")
        rec2 = PeriscopeRecorder(sw2, SessionClock(), Budget(name="p2"),
                                 mux_mp4=False)

        class _FlakyFile:
            """Writes reach the file; the flush after frame #2 fails (ENOSPC-style)."""

            def __init__(self, f):
                self._f = f
                self.fail_next_flush = False

            def write(self, b):
                return self._f.write(b)

            def flush(self):
                self._f.flush()
                if self.fail_next_flush:
                    self.fail_next_flush = False
                    raise OSError(28, "No space left on device")

            def tell(self):
                return self._f.tell()

            def close(self):
                self._f.close()

        bodies = [bytes([i + 1]) * (50 + i) for i in range(5)]
        for i, body in enumerate(bodies):
            rec2._on_frame(_S(proto.pack_periscope_frame(proto.PeriscopeFrame(
                seq=i, timestamp_ns=base + i, codec=proto.PSCOPE_CODEC_H264,
                is_keyframe=True, width=64, height=64, payload=body))))
            if i == 0:                       # wrap the handle once the segment exists
                rec2._seg_f = _FlakyFile(rec2._seg_f)
            if i == 1:
                rec2._seg_f.fail_next_flush = True
        assert rec2.summary()["errors"] == 1, rec2.summary()
        assert rec2.summary()["samples"] == 4                  # frame #2 not indexed
        rec2.close()
        blob2 = open(sw2.path("periscope", "periscope.h264"), "rb").read()
        rows2 = [r.split(",") for r in open(rec2._idx.path).read().splitlines()[1:]]
        for row in rows2:                    # every surviving row still slices exactly
            off, ln = int(row[cols["byte_offset"]]), int(row[cols["byte_len"]])
            want = bodies[int(row[cols["seq"]])]
            assert blob2[off:off + ln] == want, row[cols["seq"]]

        sw.close()
        sw2.close()
        print("rec_periscope self-test OK  (byte-exact ES, tell()-based offsets survive "
              "a failed flush, mid-GOP start counted, codec-change segmentation)")
    finally:
        _sh.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    _selftest()
