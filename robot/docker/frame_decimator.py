"""
VAT — Frame Decimator
======================
Runs inside the Docker container alongside dynamic_bridge.py.

Subscribes (Zenoh, CDR sensor_msgs/Image from the bridge):
  {ROBOT_NAME}/rt/equirectangular/image

Live config (Zenoh, plain float/int string):
  {ROBOT_NAME}/rt/prism/config/throttle_fps   → output rate Hz   (default 3.0)
  {ROBOT_NAME}/rt/prism/config/window_size    → sharpness window  (default 5, odd)

Publishes (Zenoh, raw bytes — no ROS/CDR wrapper):
  {ROBOT_NAME}/prism/camera/frame

Wire format
-----------
  bytes 0–7   int64 little-endian — nanoseconds (from ROS header.stamp,
                                    falls back to time.time_ns() if stamp is zero)
  bytes 8–N   JPEG-encoded image

Best-of-window algorithm
------------------------
Camera publishes at ~30 Hz; we want to emit at throttle_fps (e.g. 3 Hz).
A naive approach grabs every Nth frame, which may be motion-blurred.

Instead we use a temporally-centred sharpness window:
  - Incoming frames are buffered with their timestamps.
  - At each target tick T (spaced 1/throttle_fps apart), we collect all frames
    in the range [T - half_window_dt, T + half_window_dt] where
        half_window_dt = (window_size // 2) / CAMERA_FPS  seconds.
  - From those candidates we pick the sharpest (max Laplacian variance).
  - We delay emitting until at least one frame past T + half_window_dt arrives,
    guaranteeing the lookahead frames are actually in the buffer.

Sharpness metric: variance of the Laplacian of the grayscale image.
High variance → lots of edges → sharp.  Low → blurry.

Environment variables
---------------------
  ROBOT_NAME      Zenoh key prefix           (default: go2)
  ZENOH_CONNECT   Zenoh router endpoint       (default: tcp/127.0.0.1:7447)
  THROTTLE_FPS    Initial output rate Hz      (default: 3.0)
  WINDOW_SIZE     Initial sharpness window    (default: 5)
  JPEG_QUALITY    JPEG compression quality    (default: 85)
  CAMERA_FPS      Expected camera input rate  (default: 30.0)
                  Only used to compute half_window_dt; does not need to be exact.
"""

from __future__ import annotations

import os
import struct
import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import zenoh
from rosbags.typesys import Stores, get_typestore

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("frame-decimator")

# ─────────────────────────────────────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────────────────────────────────────

ROBOT_NAME    = os.environ.get("ROBOT_NAME",    "go2")
ZENOH_CONNECT = os.environ.get("ZENOH_CONNECT", "tcp/127.0.0.1:7447")
JPEG_QUALITY  = int(os.environ.get("JPEG_QUALITY",  "85"))
CAMERA_FPS    = float(os.environ.get("CAMERA_FPS",  "30.0"))

# Mutable config (overridable live via Zenoh)
_throttle_fps: float = float(os.environ.get("THROTTLE_FPS",  "3.0"))
_window_size:  int   = int(os.environ.get("WINDOW_SIZE",     "5"))
_config_lock = threading.Lock()

KEY_IMAGE        = f"{ROBOT_NAME}/rt/equirectangular/image"
KEY_OUTPUT       = f"{ROBOT_NAME}/prism/camera/frame"
KEY_THROTTLE_FPS = f"{ROBOT_NAME}/rt/prism/config/throttle_fps"
KEY_WINDOW_SIZE  = f"{ROBOT_NAME}/rt/prism/config/window_size"

# ─────────────────────────────────────────────────────────────────────────────
# ROS2 CDR deserialisation
# ─────────────────────────────────────────────────────────────────────────────

_typestore = get_typestore(Stores.ROS2_HUMBLE)


def decode_ros_image(cdr_bytes: bytes) -> Optional[tuple[int, np.ndarray]]:
    """
    Deserialise a CDR sensor_msgs/Image payload.
    Returns (timestamp_ns, bgr_array) or None on failure.
    Timestamp from msg.header.stamp; falls back to time.time_ns().
    """
    try:
        msg = _typestore.deserialize_cdr(cdr_bytes, "sensor_msgs/msg/Image")
    except Exception as e:
        log.warning(f"CDR decode failed: {e}")
        return None

    sec     = getattr(msg.header.stamp, "sec",     0)
    nanosec = getattr(msg.header.stamp, "nanosec", 0)
    ts_ns   = int(sec) * 1_000_000_000 + int(nanosec)
    if ts_ns == 0:
        ts_ns = time.time_ns()

    h, w = int(msg.height), int(msg.width)
    enc  = str(msg.encoding).lower().strip()
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    try:
        if enc in ("rgb8", "rgb"):
            bgr = cv2.cvtColor(data.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
        elif enc in ("bgr8", "bgr"):
            bgr = data.reshape(h, w, 3).copy()
        elif enc in ("mono8", "8uc1"):
            bgr = cv2.cvtColor(data.reshape(h, w), cv2.COLOR_GRAY2BGR)
        elif enc in ("rgba8",):
            bgr = cv2.cvtColor(data.reshape(h, w, 4), cv2.COLOR_RGBA2BGR)
        else:
            log.warning(f"Unsupported encoding '{enc}' — skipping frame")
            return None
    except Exception as e:
        log.warning(f"Image reshape failed (enc={enc} h={h} w={w}): {e}")
        return None

    return ts_ns, bgr

# ─────────────────────────────────────────────────────────────────────────────
# Sharpness
# ─────────────────────────────────────────────────────────────────────────────

def sharpness(bgr: np.ndarray) -> float:
    """Variance of Laplacian on grayscale image. Higher = sharper."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())

# ─────────────────────────────────────────────────────────────────────────────
# Frame buffer
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class FrameEntry:
    ts_ns:     int
    sharpness: float
    bgr:       np.ndarray


def _max_buffer_frames() -> int:
    with _config_lock:
        fps = _throttle_fps
        ws  = _window_size
    return max(ws + 4, int(2.0 * CAMERA_FPS / fps))

# ─────────────────────────────────────────────────────────────────────────────
# Decimator
# ─────────────────────────────────────────────────────────────────────────────

class FrameDecimator:
    def __init__(self, zenoh_session: zenoh.Session):
        self._z   = zenoh_session
        self._pub = self._z.declare_publisher(
            KEY_OUTPUT,
            congestion_control=zenoh.CongestionControl.DROP,
        )
        self._buf: deque[FrameEntry] = deque()
        self._lock = threading.Lock()
        self._next_tick_ns: Optional[int] = None
        self._published_count = 0

    def push(self, ts_ns: int, bgr: np.ndarray):
        entry = FrameEntry(ts_ns=ts_ns, sharpness=sharpness(bgr), bgr=bgr)

        with _config_lock:
            fps = _throttle_fps
            ws  = _window_size

        max_buf = _max_buffer_frames()
        half_win_ns = int((ws // 2) / CAMERA_FPS * 1_000_000_000)
        tick_interval_ns = int(1_000_000_000 / fps)

        with self._lock:
            self._buf.append(entry)

            if self._next_tick_ns is None:
                self._next_tick_ns = ts_ns

            # Drain frames too old to be in any future window
            cutoff = self._next_tick_ns - half_win_ns * 2
            while self._buf and self._buf[0].ts_ns < cutoff:
                self._buf.popleft()

            while len(self._buf) > max_buf:
                self._buf.popleft()

            # Emit as many ticks as we have full lookahead for
            while (self._next_tick_ns is not None and
                   ts_ns >= self._next_tick_ns + half_win_ns):
                self._emit_tick(self._next_tick_ns, half_win_ns, ws)
                self._next_tick_ns += tick_interval_ns

    def _emit_tick(self, tick_ns: int, half_win_ns: int, ws: int):
        lo = tick_ns - half_win_ns
        hi = tick_ns + half_win_ns
        candidates = [e for e in self._buf if lo <= e.ts_ns <= hi]

        if not candidates:
            closest = min(self._buf, key=lambda e: abs(e.ts_ns - tick_ns), default=None)
            if closest is None:
                return
            candidates = [closest]

        best = max(candidates, key=lambda e: e.sharpness)

        ok, buf = cv2.imencode(".jpg", best.bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok or buf is None:
            log.warning("JPEG encode failed — skipping tick")
            return

        payload = struct.pack("<q", best.ts_ns) + buf.tobytes()
        self._pub.put(payload)
        self._published_count += 1

        log.debug(
            f"tick={tick_ns//1_000_000}ms  best={best.ts_ns//1_000_000}ms  "
            f"sharp={best.sharpness:.0f}  candidates={len(candidates)}/{ws}  "
            f"size={len(payload)//1024}kB  total={self._published_count}"
        )

# ─────────────────────────────────────────────────────────────────────────────
# Zenoh callbacks
# ─────────────────────────────────────────────────────────────────────────────

_decimator: Optional[FrameDecimator] = None


def _on_image(sample):
    result = decode_ros_image(bytes(sample.payload))
    if result is None or _decimator is None:
        return
    _decimator.push(*result)


def _on_throttle_fps(sample):
    global _throttle_fps
    try:
        val = float(bytes(sample.payload).decode().strip())
        if 0.1 <= val <= 30.0:
            with _config_lock:
                _throttle_fps = val
            log.info(f"[Config] throttle_fps → {val:.2f} Hz")
        else:
            log.warning(f"[Config] throttle_fps {val} out of range [0.1, 30.0]")
    except Exception as e:
        log.warning(f"[Config] bad throttle_fps payload: {e}")


def _on_window_size(sample):
    global _window_size
    try:
        val = int(bytes(sample.payload).decode().strip())
        if 1 <= val <= 31:
            val = val if val % 2 == 1 else val + 1  # force odd
            with _config_lock:
                _window_size = val
            log.info(f"[Config] window_size → {val}")
        else:
            log.warning(f"[Config] window_size {val} out of range [1, 31]")
    except Exception as e:
        log.warning(f"[Config] bad window_size payload: {e}")

# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    global _decimator

    log.info(f"Connecting to Zenoh at {ZENOH_CONNECT}...")
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ZENOH_CONNECT}"]')
    conf.insert_json5("mode", '"client"')

    z = None
    while z is None:
        try:
            z = zenoh.open(conf)
        except Exception as e:
            log.warning(f"Zenoh connect failed: {e} — retrying in 5s")
            time.sleep(5)

    log.info(f"Connected. Listening on '{KEY_IMAGE}' → publishing '{KEY_OUTPUT}' "
             f"@ {_throttle_fps}Hz  window={_window_size}  quality={JPEG_QUALITY}")

    _decimator = FrameDecimator(z)
    z.declare_subscriber(KEY_IMAGE,        _on_image)
    z.declare_subscriber(KEY_THROTTLE_FPS, _on_throttle_fps)
    z.declare_subscriber(KEY_WINDOW_SIZE,  _on_window_size)

    try:
        while True:
            time.sleep(10)
            if _decimator:
                with _config_lock:
                    fps, ws = _throttle_fps, _window_size
                log.info(f"published={_decimator._published_count}  "
                         f"buf={len(_decimator._buf)}  fps={fps:.1f}  win={ws}")
    except KeyboardInterrupt:
        pass
    finally:
        z.close()


if __name__ == "__main__":
    main()
