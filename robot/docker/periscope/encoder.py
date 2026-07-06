"""
VAT - Remote Periscope: video encoder abstraction.

Tries hardware/efficient video first (PyAV -> NVENC HEVC -> NVENC H.264 ->
software libx265/libx264) and falls back to per-frame MJPEG (cv2.imencode) if
PyAV is unavailable. The MJPEG path means the whole periscope pipeline works
end-to-end with no extra dependency (every frame is a keyframe); the H.26x paths
kick in automatically when PyAV + a suitable encoder are present.

All encoders share one interface:
    enc.codec_id            -> PSCOPE_CODEC_* (for the wire header)
    enc.encode(bgr)         -> list[(payload_bytes, is_keyframe)]
    enc.request_keyframe()  -> force an IDR on the next frame
    enc.close()
"""

from __future__ import annotations

import logging
from typing import List, Tuple

import numpy as np

import vat_protocol as proto

log = logging.getLogger("periscope.encoder")


class MJPEGEncoder:
    """Fallback: encode each frame as an independent JPEG (always a keyframe).
    No inter-frame compression, but zero extra deps and trivially robust to loss."""

    codec_id = proto.PSCOPE_CODEC_MJPEG

    def __init__(self, quality: int = 80):
        import cv2
        self._cv2 = cv2
        self._params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]

    def encode(self, bgr: np.ndarray) -> List[Tuple[bytes, bool]]:
        ok, buf = self._cv2.imencode(".jpg", bgr, self._params)
        if not ok:
            return []
        return [(buf.tobytes(), True)]

    def request_keyframe(self):
        pass                      # every MJPEG frame is already a keyframe

    def close(self):
        pass


class PyAVEncoder:
    """H.264/H.265 via PyAV. ``codec_name`` is a libav encoder name, e.g.
    'hevc_nvenc', 'h264_nvenc', 'libx265', 'libx264'."""

    def __init__(self, codec_name: str, codec_id: int, width: int, height: int,
                 fps: float, bitrate: int, gop: int):
        import av
        self._av = av
        self.codec_id = codec_id
        cc = av.CodecContext.create(codec_name, "w")
        cc.width = int(width)
        cc.height = int(height)
        cc.pix_fmt = "yuv420p"
        cc.framerate = max(1, int(round(fps)))
        cc.time_base = self._av.time_base if hasattr(self._av, "time_base") else None
        cc.bit_rate = int(bitrate)
        cc.gop_size = max(1, int(gop))
        # Low-latency options (best-effort; unknown keys are ignored per-encoder).
        opts = {}
        if "nvenc" in codec_name:
            opts = {"preset": "p1", "tune": "ll", "zerolatency": "1",
                    "delay": "0", "rc": "cbr"}
        elif codec_name.startswith("libx26"):
            opts = {"preset": "ultrafast", "tune": "zerolatency"}
        try:
            cc.options = opts
        except Exception:
            pass
        self._cc = cc
        self._name = codec_name
        self._force_kf = False

    def encode(self, bgr: np.ndarray) -> List[Tuple[bytes, bool]]:
        av = self._av
        rgb = bgr[:, :, ::-1]                        # cv2 BGR -> RGB
        frame = av.VideoFrame.from_ndarray(np.ascontiguousarray(rgb), format="rgb24")
        frame = frame.reformat(format="yuv420p")
        if self._force_kf:
            try:
                frame.pict_type = av.video.frame.PictureType.I
            except Exception:
                pass
            self._force_kf = False
        out = []
        for pkt in self._cc.encode(frame):
            out.append((bytes(pkt), bool(pkt.is_keyframe)))
        return out

    def request_keyframe(self):
        self._force_kf = True

    def close(self):
        try:
            for _ in self._cc.encode(None):          # flush
                pass
        except Exception:
            pass


# Preference -> ordered list of (libav encoder name, PSCOPE codec id) to try.
_PREF = {
    "h265": [("hevc_nvenc", proto.PSCOPE_CODEC_HEVC), ("libx265", proto.PSCOPE_CODEC_HEVC),
             ("h264_nvenc", proto.PSCOPE_CODEC_H264), ("libx264", proto.PSCOPE_CODEC_H264)],
    "hevc": [("hevc_nvenc", proto.PSCOPE_CODEC_HEVC), ("libx265", proto.PSCOPE_CODEC_HEVC)],
    "h264": [("h264_nvenc", proto.PSCOPE_CODEC_H264), ("libx264", proto.PSCOPE_CODEC_H264)],
    "mjpeg": [],
}


def _validate_encoder(name: str, width: int, height: int, fps: float,
                      bitrate: int, gop: int) -> None:
    """Force the codec to actually open by encoding one throwaway frame.

    PyAV opens the underlying libav codec LAZILY on the first encode() call, NOT
    in CodecContext.create -- so merely constructing a PyAVEncoder does not prove
    the encoder can run. On a host where e.g. hevc_nvenc is registered but cannot
    be opened (no GPU / no driver / no /dev/nvidia in the container / NVENC session
    limit hit), construction succeeds and every real encode() then raises. Without
    this trial-open the fallback ladder (nvenc -> libx26x -> mjpeg) is defeated:
    make_encoder returns a dead encoder and the service publishes zero frames
    (the aim frustum shows, but the video never does).

    Raises if the encoder cannot actually be opened; returns cleanly otherwise.
    """
    probe = PyAVEncoder(name, 0, width, height, fps, bitrate, gop)
    try:
        probe.encode(np.zeros((int(height), int(width), 3), np.uint8))  # forces open
    finally:
        probe.close()


def make_encoder(pref: str, width: int, height: int, fps: float,
                 bitrate: int, gop: int, jpeg_quality: int = 80):
    """Return the best available encoder for ``pref`` (h265|h264|hevc|mjpeg),
    falling back to MJPEG. Logs which one it picked."""
    pref = (pref or "h265").lower()
    for name, cid in _PREF.get(pref, []):
        try:
            _validate_encoder(name, width, height, fps, bitrate, gop)
            enc = PyAVEncoder(name, cid, width, height, fps, bitrate, gop)
            log.info(f"[periscope] encoder: {name} ({width}x{height}@{fps:.0f})")
            return enc
        except Exception as e:
            # WARNING (not debug): a silently-skipped hardware encoder is exactly
            # what makes "frustum but no video" hard to diagnose.
            log.warning(f"[periscope] encoder {name} unavailable, trying next: {e}")
    log.warning("[periscope] no H.26x encoder available (install PyAV + NVENC "
                "for HEVC/H.264) - falling back to per-frame MJPEG")
    return MJPEGEncoder(jpeg_quality)
