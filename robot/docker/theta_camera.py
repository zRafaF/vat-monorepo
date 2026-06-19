"""
VAT — Theta Camera (capture + decimate)
=======================================
Captures the **RICOH THETA X** 360° stream over **UVC** with OpenCV, picks the
sharpest frame in a small window, stamps the camera height, and publishes the
result to Zenoh — all in one process. No ROS camera node, no host-side
stitching: the Theta X does dynamic stitching + zenith correction *in-camera*
during live streaming, so the UVC stream is already a clean equirectangular.

    Theta X --UVC--> [OpenCV capture] --> [best-of-window + camera_height + JPEG]
                                      --> {robot}/prism/camera/frame

This replaces both the old Insta360 ROS driver and the separate frame decimator
(see docs/archive/insta360.md for why we moved away).

Capture source (pick one; checked in this order)
------------------------------------------------
  THETA_GST_PIPELINE  full GStreamer pipeline → cv2.CAP_GSTREAMER
                      (needs OpenCV built with GStreamer + the gstthetauvc plugin)
  THETA_DEVICE        v4l2 device path/index → cv2.CAP_V4L2
                      (e.g. /dev/video0 fed by libuvc-theta-sample `gst_loopback`;
                       works with the pip OpenCV in the container)
  (neither set)       a default gstthetauvc pipeline built from THETA_MODE

  THETA_MODE          2K (1920×960) or 4K (3840×1920)   default 2K

See docs/setup/robot.md for the host-side libuvc-theta / gstthetauvc setup.

Best-of-window, camera height, retransmit, live config: identical to the old
decimator — only the *input* changed (UVC capture instead of a bridged ROS topic).

Environment
-----------
  ROBOT_NAME, ZENOH_CONNECT, THROTTLE_FPS, WINDOW_SIZE, JPEG_QUALITY, LOSSLESS,
  CAMERA_FPS, SHARPNESS_DOWNSCALE, FALLBACK_BODY_HEIGHT, STICK_OFFSET_{X,Y,Z},
  THETA_GST_PIPELINE, THETA_DEVICE, THETA_MODE, CAPTURE_RETRY_S
"""

from __future__ import annotations

import os
import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import zenoh

import vat_protocol as proto
from kinematics import build_robot_model, RobotStateTracker

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="[%(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("theta-camera")

# ─────────────────────────────────────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────────────────────────────────────

ROBOT_NAME      = os.environ.get("ROBOT_NAME",      "go2")
ZENOH_CONNECT   = os.environ.get("ZENOH_CONNECT",   "tcp/127.0.0.1:7447")
JPEG_QUALITY    = int(os.environ.get("JPEG_QUALITY", "85"))
LOSSLESS        = os.environ.get("LOSSLESS", "").lower() in ("1", "true", "yes")
CAMERA_FPS      = float(os.environ.get("CAMERA_FPS", "30.0"))
SHARP_DOWNSCALE = float(os.environ.get("SHARPNESS_DOWNSCALE", "0.5"))
FALLBACK_BODY_H = float(os.environ.get("FALLBACK_BODY_HEIGHT", "0.30"))
RETX_BUFFER     = int(os.environ.get("RETX_BUFFER", "256"))
CAPTURE_RETRY_S = float(os.environ.get("CAPTURE_RETRY_S", "3.0"))

# Capture source
THETA_GST_PIPELINE = os.environ.get("THETA_GST_PIPELINE", "").strip()
THETA_DEVICE       = os.environ.get("THETA_DEVICE", "").strip()
THETA_MODE         = os.environ.get("THETA_MODE", "2K").strip()

_KEYS = proto.keys(ROBOT_NAME)
KEY_OUTPUT       = _KEYS["camera_frame"]
KEY_FRAME_GET    = _KEYS["camera_frame_get"]
KEY_THROTTLE_FPS = _KEYS["cfg_throttle_fps"]
KEY_WINDOW_SIZE  = _KEYS["cfg_window_size"]

# Mutable, live-tunable config
_throttle_fps: float = float(os.environ.get("THROTTLE_FPS", "3.0"))
_window_size:  int   = int(os.environ.get("WINDOW_SIZE", "5"))
_config_lock = threading.Lock()


def _get_config():
    with _config_lock:
        return _throttle_fps, _window_size


def sharpness(bgr: np.ndarray) -> float:
    """Variance of Laplacian (higher = sharper).  Downscaled for speed."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    if 0.0 < SHARP_DOWNSCALE < 1.0:
        gray = cv2.resize(gray, None, fx=SHARP_DOWNSCALE, fy=SHARP_DOWNSCALE,
                          interpolation=cv2.INTER_AREA)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


# ─────────────────────────────────────────────────────────────────────────────
# UVC capture (the Theta-specific part)
# ─────────────────────────────────────────────────────────────────────────────

def _default_gst_pipeline(mode: str) -> str:
    """A portable (software-decode) gstthetauvc pipeline. For Jetson HW decode,
    set THETA_GST_PIPELINE with `nvv4l2decoder`/`nvvidconv` instead."""
    return (f"thetauvcsrc mode={mode} ! queue ! h264parse ! decodebin ! "
            f"videoconvert ! video/x-raw,format=BGR ! "
            f"appsink drop=true max-buffers=2 sync=false")


def open_capture() -> cv2.VideoCapture:
    """Open the Theta UVC stream. Tries the configured source; raises on failure."""
    if THETA_GST_PIPELINE:
        log.info(f"[Capture] GStreamer pipeline: {THETA_GST_PIPELINE}")
        return cv2.VideoCapture(THETA_GST_PIPELINE, cv2.CAP_GSTREAMER)
    if THETA_DEVICE:
        dev = int(THETA_DEVICE) if THETA_DEVICE.isdigit() else THETA_DEVICE
        log.info(f"[Capture] v4l2 device: {THETA_DEVICE}")
        return cv2.VideoCapture(dev, cv2.CAP_V4L2)
    pipeline = _default_gst_pipeline(THETA_MODE)
    log.info(f"[Capture] default gstthetauvc pipeline (mode={THETA_MODE}): {pipeline}")
    return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)


class ThetaCapture(threading.Thread):
    """Reads frames from the Theta and pushes them into the decimator."""

    def __init__(self, decimator: "FrameDecimator"):
        super().__init__(daemon=True)
        self._decimator = decimator
        self._stop = threading.Event()
        self._frames = 0

    def stop(self):
        self._stop.set()

    @property
    def frames(self):
        return self._frames

    def run(self):
        while not self._stop.is_set():
            cap = None
            try:
                cap = open_capture()
                if not cap or not cap.isOpened():
                    log.warning("[Capture] could not open Theta stream — retrying "
                                f"in {CAPTURE_RETRY_S}s. Is the camera in LIVE mode "
                                "and the UVC source available? (see docs/setup/robot.md)")
                    time.sleep(CAPTURE_RETRY_S)
                    continue
                log.info("[Capture] Theta stream open. Streaming…")
                while not self._stop.is_set():
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        log.warning("[Capture] read failed — reopening stream")
                        break
                    self._frames += 1
                    self._decimator.push(time.time_ns(), frame)
            except Exception as e:
                log.warning(f"[Capture] error: {e}")
            finally:
                if cap is not None:
                    cap.release()
            if not self._stop.is_set():
                time.sleep(CAPTURE_RETRY_S)


# ─────────────────────────────────────────────────────────────────────────────
# Best-of-window decimator  (unchanged logic — input is now UVC, not a ROS topic)
# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class FrameEntry:
    ts_ns: int
    bgr: np.ndarray


class FrameDecimator:
    """Best-of-N-frame-window decimator with live camera-height tagging."""

    def __init__(self, z: zenoh.Session, state: RobotStateTracker, model):
        self._z = z
        self._state = state
        self._model = model
        self._pub = self._declare_reliable_publisher(z, KEY_OUTPUT)
        self._buf: deque[FrameEntry] = deque()
        self._lock = threading.Lock()
        self._next_tick_ns: Optional[int] = None
        self._published = 0
        self._skipped = 0
        self._seq = 0
        self._retx: dict[int, bytes] = {}
        self._retx_order: deque[int] = deque()
        self._retx_lock = threading.Lock()
        try:
            z.declare_queryable(KEY_FRAME_GET, self._on_frame_get)
            log.info(f"[Decimator] retransmit queryable on '{KEY_FRAME_GET}'")
        except Exception as e:
            log.warning(f"[Decimator] could not declare retransmit queryable: {e}")

    @staticmethod
    def _declare_reliable_publisher(z, key):
        for kwargs in (
            dict(congestion_control=zenoh.CongestionControl.BLOCK,
                 reliability=zenoh.Reliability.RELIABLE,
                 priority=zenoh.Priority.DATA_HIGH),
            dict(congestion_control=zenoh.CongestionControl.BLOCK,
                 priority=zenoh.Priority.DATA_HIGH),
            dict(congestion_control=zenoh.CongestionControl.BLOCK),
        ):
            try:
                return z.declare_publisher(key, **kwargs)
            except TypeError:
                continue
        return z.declare_publisher(key)

    def _on_frame_get(self, query):
        try:
            params = query.parameters if hasattr(query, "parameters") else \
                query.selector.parameters
            seq = int(params["seq"]) if "seq" in params else -1
            with self._retx_lock:
                payload = self._retx.get(seq)
            if payload is not None:
                query.reply(KEY_FRAME_GET, payload)
                log.debug(f"[Decimator] retransmit seq={seq} ({len(payload)//1024}kB)")
            else:
                query.reply_err(f"seq {seq} not buffered".encode())
        except Exception as e:
            try:
                query.reply_err(str(e).encode())
            except Exception:
                pass

    def push(self, ts_ns: int, bgr: np.ndarray):
        fps, ws = _get_config()
        ws = max(1, ws | 1)
        half = ws // 2
        interval_ns = int(1e9 / max(fps, 0.1))
        lookahead_ns = int((half + 0.5) / max(CAMERA_FPS, 1.0) * 1e9)

        with self._lock:
            self._buf.append(FrameEntry(ts_ns, bgr))
            if self._next_tick_ns is None:
                self._next_tick_ns = ts_ns
            while self._next_tick_ns is not None:
                if not self._try_emit_tick(half, interval_ns, lookahead_ns, ts_ns):
                    break
            max_keep = max(ws + 4, int(2 * CAMERA_FPS / max(fps, 0.1)))
            while len(self._buf) > max_keep:
                self._buf.popleft()

    def _try_emit_tick(self, half, interval_ns, lookahead_ns, newest_ns) -> bool:
        tick = self._next_tick_ns
        buf = self._buf
        if not buf:
            return False
        center = min(range(len(buf)), key=lambda i: abs(buf[i].ts_ns - tick))
        frames_after = len(buf) - 1 - center
        have_lookahead = frames_after >= half
        timed_out = newest_ns >= tick + interval_ns + lookahead_ns
        if not have_lookahead and not timed_out:
            return False
        lo = max(0, center - half)
        hi = min(len(buf), center + half + 1)
        candidates = list(buf)[lo:hi]
        best = max(candidates, key=lambda e: sharpness(e.bgr))
        self._emit(best, n_candidates=len(candidates))
        self._next_tick_ns = tick + interval_ns
        return True

    def _emit(self, entry: FrameEntry, n_candidates: int):
        if LOSSLESS:
            ok, jbuf = cv2.imencode(".png", entry.bgr)
        else:
            ok, jbuf = cv2.imencode(".jpg", entry.bgr,
                                    [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok or jbuf is None:
            log.warning("encode failed — skipping tick")
            self._skipped += 1
            return

        body = self._state.get()
        cam_h = self._model.camera_height(body.body_height, body.rotation)

        seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        payload = proto.pack_frame(entry.ts_ns, seq, cam_h, jbuf.tobytes())

        with self._retx_lock:
            self._retx[seq] = payload
            self._retx_order.append(seq)
            while len(self._retx_order) > RETX_BUFFER:
                self._retx.pop(self._retx_order.popleft(), None)

        try:
            self._pub.put(payload, encoding=proto.ENC_FRAME)
        except TypeError:
            self._pub.put(payload)
        self._published += 1
        log.debug(f"emit seq={seq} ts={entry.ts_ns//1_000_000}ms cam_h={cam_h:.2f}m "
                  f"cands={n_candidates} size={len(payload)//1024}kB total={self._published}")

    def stats(self) -> str:
        fps, ws = _get_config()
        return (f"published={self._published} skipped={self._skipped} "
                f"buf={len(self._buf)} fps={fps:.1f} win={ws}")


# ─────────────────────────────────────────────────────────────────────────────
# Live-config callbacks
# ─────────────────────────────────────────────────────────────────────────────

def _on_throttle_fps(sample):
    global _throttle_fps
    try:
        val = float(bytes(sample.payload).decode().strip())
        if 0.1 <= val <= 30.0:
            with _config_lock:
                _throttle_fps = val
            log.info(f"[Config] throttle_fps → {val:.2f} Hz")
        else:
            log.warning(f"[Config] throttle_fps {val} out of range [0.1, 30]")
    except Exception as e:
        log.warning(f"[Config] bad throttle_fps payload: {e}")


def _on_window_size(sample):
    global _window_size
    try:
        val = int(float(bytes(sample.payload).decode().strip()))
        if 1 <= val <= 31:
            with _config_lock:
                _window_size = val | 1
            log.info(f"[Config] window_size → {_window_size}")
        else:
            log.warning(f"[Config] window_size {val} out of range [1, 31]")
    except Exception as e:
        log.warning(f"[Config] bad window_size payload: {e}")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def _open_session() -> zenoh.Session:
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ZENOH_CONNECT}"]')
    conf.insert_json5("mode", '"peer"')
    while True:
        try:
            return zenoh.open(conf)
        except Exception as e:
            log.warning(f"Zenoh connect failed: {e} — retrying in 5s")
            time.sleep(5)


def main():
    log.info(f"Connecting to Zenoh at {ZENOH_CONNECT}...")
    z = _open_session()
    fps, ws = _get_config()
    enc_label = "PNG (lossless)" if LOSSLESS else f"JPEG q={JPEG_QUALITY}"
    log.info(f"Connected. Theta UVC → '{KEY_OUTPUT}'  @ {fps}Hz  "
             f"window={ws}  encode={enc_label}")

    model = build_robot_model()
    state = RobotStateTracker(z, ROBOT_NAME, fallback_body_height=FALLBACK_BODY_H)
    decimator = FrameDecimator(z, state, model)

    capture = ThetaCapture(decimator)
    capture.start()

    z.declare_subscriber(KEY_THROTTLE_FPS, _on_throttle_fps)
    z.declare_subscriber(KEY_WINDOW_SIZE, _on_window_size)

    try:
        while True:
            time.sleep(10)
            log.info(f"{decimator.stats()} captured={capture.frames}")
    except KeyboardInterrupt:
        pass
    finally:
        capture.stop()
        z.close()


if __name__ == "__main__":
    main()
