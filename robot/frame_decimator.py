"""
VAT — Frame Decimator
======================
Runs inside the bridge Docker container alongside dynamic_bridge.py.

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
import sys
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
    The timestamp comes from msg.header.stamp; falls back to time.time_ns().
    """
    try:
        msg = _typestore.deserialize_cdr(cdr_bytes, "sensor_msgs/msg/Image")
    except Exception as e:
        log.warning(f"CDR decode failed: {e}")
        return None

    # Timestamp — prefer the ROS header stamp (set by the camera driver)
    sec     = getattr(msg.header.stamp, "sec",     0)
    nanosec = getattr(msg.header.stamp, "nanosec", 0)
    ts_ns   = int(sec) * 1_000_000_000 + int(nanosec)
    if ts_ns == 0:
        ts_ns = time.time_ns()

    # Image data
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
    """Variance of the Laplacian on the grayscale image.  Higher = sharper."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())

# ─────────────────────────────────────────────────────────────────────────────
# Frame buffer entry
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class FrameEntry:
    ts_ns:     int
    sharpness: float
    bgr:       np.ndarray   # kept until JPEG is needed to avoid repeated encodes


# Maximum buffer duration: keep at most 2× the throttle period worth of frames
# (recomputed whenever throttle_fps changes)
def _max_buffer_frames() -> int:
    with _config_lock:
        fps = _throttle_fps
        ws  = _window_size
    # Keep enough frames to fill 2 full throttle periods + the window
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

        # Publish-tick scheduling
        self._next_tick_ns: Optional[int] = None   # nanoseconds
        self._published_count = 0

    def push(self, ts_ns: int, bgr: np.ndarray):
        """Add a new decoded frame.  Called from the Zenoh subscriber thread."""
        entry = FrameEntry(ts_ns=ts_ns, sharpness=sharpness(bgr), bgr=bgr)

        with _config_lock:
            fps = _throttle_fps
            ws  = _window_size

        max_buf = _max_buffer_frames()
        half_win_ns = int((ws // 2) / CAMERA_FPS * 1_000_000_000)
        tick_interval_ns = int(1_000_000_000 / fps)

        with self._lock:
            self._buf.append(entry)

            # Initialise first tick at the first frame's timestamp
            if self._next_tick_ns is None:
                self._next_tick_ns = ts_ns

            # Drain stale frames (older than current tick − half window)
            cutoff = self._next_tick_ns - half_win_ns
            while self._buf and self._buf[0].ts_ns < cutoff - half_win_ns:
                self._buf.popleft()

            # Limit buffer size
            while len(self._buf) > max_buf:
                self._buf.popleft()

            # Try to emit: wait until we have lookahead coverage past the tick
            # i.e., at least one frame at ts > next_tick + half_win
            while (self._next_tick_ns is not None and
                   ts_ns >= self._next_tick_ns + half_win_ns):
                self._emit_tick(self._next_tick_ns, half_win_ns, ws)
                self._next_tick_ns += tick_interval_ns

    def _emit_tick(self, tick_ns: int, half_win_ns: int, ws: int):
        """Pick the sharpest frame in [tick - half_win, tick + half_win] and publish."""
        lo = tick_ns - half_win_ns
        hi = tick_ns + half_win_ns

        candidates = [e for e in self._buf if lo <= e.ts_ns <= hi]

        if not candidates:
            # Fallback: use the single closest frame
            closest = min(self._buf, key=lambda e: abs(e.ts_ns - tick_ns), default=None)
            if closest is None:
                return
            candidates = [closest]

        best = max(candidates, key=lambda e: e.sharpness)

        # JPEG encode
        ok, buf = cv2.imencode(".jpg", best.bgr,
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok or buf is None:
            log.warning("[Decimator] JPEG encode failed — skipping tick")
            return

        # Wire format: [8B ts_ns int64 LE] + JPEG bytes
        payload = struct.pack("<q", best.ts_ns) + buf.tobytes()
        self._pub.put(payload)

        self._published_count += 1
        log.debug(
            f"[Decimator] tick={tick_ns//1_000_000}ms "
            f"best_ts={best.ts_ns//1_000_000}ms "
            f"sharpness={best.sharpness:.1f} "
            f"candidates={len(candidates)}/{ws} "
            f"size={len(payload)//1024}kB "
            f"published={self._published_count}"
        )

# ─────────────────────────────────────────────────────────────────────────────
# Zenoh callbacks
# ─────────────────────────────────────────────────────────────────────────────

_decimator: Optional[FrameDecimator] = None


def _on_image(sample):
    global _decimator
    result = decode_ros_image(bytes(sample.payload))
    if result is None or _decimator is None:
        return
    ts_ns, bgr = result
    _decimator.push(ts_ns, bgr)


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
            # Force odd for symmetric window
            val = val if val % 2 == 1 else val + 1
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

    log.info(f"[Decimator] Connecting to Zenoh at {ZENOH_CONNECT}...")
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ZENOH_CONNECT}"]')
    conf.insert_json5("mode", '"client"')

    # Retry until router is reachable
    z = None
    while z is None:
        try:
            z = zenoh.open(conf)
        except Exception as e:
            log.warning(f"Zenoh connect failed: {e} — retrying in 5s")
            time.sleep(5)

    log.info("[Decimator] Connected.")
    _decimator = FrameDecimator(z)

    # Subscribers
    z.declare_subscriber(KEY_IMAGE,        _on_image)
    z.declare_subscriber(KEY_THROTTLE_FPS, _on_throttle_fps)
    z.declare_subscriber(KEY_WINDOW_SIZE,  _on_window_size)

    log.info(
        f"[Decimator] Listening on '{KEY_IMAGE}'\n"
        f"            Publishing to '{KEY_OUTPUT}'\n"
        f"            throttle_fps={_throttle_fps} Hz  window_size={_window_size}  "
        f"jpeg_quality={JPEG_QUALITY}"
    )

    try:
        while True:
            time.sleep(10)
            with _config_lock:
                fps = _throttle_fps
                ws  = _window_size
            if _decimator:
                log.info(
                    f"[Decimator] ♥ published={_decimator._published_count} "
                    f"buf_len={len(_decimator._buf)} "
                    f"fps={fps:.1f} win={ws}"
                )
    except KeyboardInterrupt:
        log.info("[Decimator] Shutting down.")
    finally:
        z.close()


if __name__ == "__main__":
    main()
