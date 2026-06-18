"""
VAT — Frame Decimator
======================
Runs inside the robot Docker container alongside ``dynamic_bridge.py`` and
``pose_fuser.py``.  It is a plain Zenoh client (not a ROS node): the bridge
already exposes the equirectangular image stream on Zenoh as CDR, so this
process only needs to subscribe, pick the best frame, and re-publish.

Pipeline
--------
  bridge → {robot}/rt/equirectangular/image  (CDR sensor_msgs/Image, ~30 Hz)
        → [decimate to throttle_fps, pick sharpest in an N-frame window]
        → {robot}/prism/camera/frame          (VAT frame: ts + camera_height + JPEG)

Best-of-window (configurable)
-----------------------------
The camera runs at ~30 Hz but PRISM only wants a few Hz.  Naively grabbing every
Nth frame risks picking a motion-blurred one.  Instead, for each output tick we
look at a **window of N consecutive frames centred on the target frame** (e.g.
3 or 5 frames — the target frame and its immediate neighbours) and emit the
*sharpest* one.  N is ``window_size`` and is tunable live over Zenoh.

  window_size = 1  → no neighbours, emit the frame nearest the tick
  window_size = 5  → the target frame ± 2 neighbours, sharpest wins

Sharpness is the variance of the Laplacian (computed only for the N candidates,
not every incoming frame, to save CPU on the Jetson).

Camera height (metric scale for PRISM)
--------------------------------------
Each emitted frame carries the camera's height above the floor at capture time,
computed by :mod:`kinematics` from the live body state (``SportModeState`` via
the bridge) + the selfie-stick geometry.  The Go2-W can lie down / stand up, so
this is not constant.  The server reads it straight from the frame message.

Live config (Zenoh, plain string payloads)
-------------------------------------------
  {robot}/rt/prism/config/throttle_fps   float, output rate Hz   (default 3.0)
  {robot}/rt/prism/config/window_size    int, sharpness window   (default 5, odd)

Environment
-----------
  ROBOT_NAME, ZENOH_CONNECT, THROTTLE_FPS, WINDOW_SIZE, JPEG_QUALITY,
  CAMERA_FPS, IMAGE_TOPIC, SHARPNESS_DOWNSCALE, STICK_OFFSET_{X,Y,Z},
  FALLBACK_BODY_HEIGHT
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
from rosbags.typesys import Stores, get_typestore

import vat_protocol as proto
from kinematics import build_robot_model, RobotStateTracker

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="[%(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("frame-decimator")

# ─────────────────────────────────────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────────────────────────────────────

ROBOT_NAME     = os.environ.get("ROBOT_NAME",     "go2")
ZENOH_CONNECT  = os.environ.get("ZENOH_CONNECT",  "tcp/127.0.0.1:7447")
JPEG_QUALITY   = int(os.environ.get("JPEG_QUALITY",   "85"))
CAMERA_FPS     = float(os.environ.get("CAMERA_FPS",   "30.0"))
IMAGE_TOPIC    = os.environ.get("IMAGE_TOPIC",    "equirectangular/image")
SHARP_DOWNSCALE = float(os.environ.get("SHARPNESS_DOWNSCALE", "0.5"))  # 0<..<=1
FALLBACK_BODY_H = float(os.environ.get("FALLBACK_BODY_HEIGHT", "0.30"))

_KEYS = proto.keys(ROBOT_NAME)
KEY_IMAGE        = f"{ROBOT_NAME}/rt/{IMAGE_TOPIC}"
KEY_OUTPUT       = _KEYS["camera_frame"]
KEY_FRAME_GET    = _KEYS["camera_frame_get"]
KEY_THROTTLE_FPS = _KEYS["cfg_throttle_fps"]
KEY_WINDOW_SIZE  = _KEYS["cfg_window_size"]

# How many recently-emitted frames to keep buffered for on-demand retransmit.
RETX_BUFFER = int(os.environ.get("RETX_BUFFER", "256"))

# Mutable, live-tunable config
_throttle_fps: float = float(os.environ.get("THROTTLE_FPS", "3.0"))
_window_size:  int   = int(os.environ.get("WINDOW_SIZE", "5"))
_config_lock = threading.Lock()


def _get_config():
    with _config_lock:
        return _throttle_fps, _window_size


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 image CDR decode
# ─────────────────────────────────────────────────────────────────────────────

_typestore = get_typestore(Stores.ROS2_HUMBLE)


def decode_ros_image(cdr_bytes: bytes) -> Optional[tuple[int, np.ndarray]]:
    """Deserialise a CDR sensor_msgs/Image → (timestamp_ns, bgr).  None on failure."""
    try:
        msg = _typestore.deserialize_cdr(cdr_bytes, "sensor_msgs/msg/Image")
    except Exception as e:
        log.warning(f"CDR decode failed: {e}")
        return None

    sec     = int(getattr(msg.header.stamp, "sec", 0))
    nanosec = int(getattr(msg.header.stamp, "nanosec", 0))
    ts_ns   = sec * 1_000_000_000 + nanosec or time.time_ns()

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
        log.warning(f"Image reshape failed (enc={enc} {w}x{h}): {e}")
        return None
    return ts_ns, bgr


def sharpness(bgr: np.ndarray) -> float:
    """Variance of Laplacian (higher = sharper).  Downscaled for speed."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    if 0.0 < SHARP_DOWNSCALE < 1.0:
        gray = cv2.resize(gray, None, fx=SHARP_DOWNSCALE, fy=SHARP_DOWNSCALE,
                          interpolation=cv2.INTER_AREA)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


# ─────────────────────────────────────────────────────────────────────────────
# Decimator
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
        # Reliable transport: every frame matters for pose estimation, so we
        # prefer back-pressure (BLOCK) over silent drops.
        self._pub = self._declare_reliable_publisher(z, KEY_OUTPUT)
        self._buf: deque[FrameEntry] = deque()
        self._lock = threading.Lock()
        self._next_tick_ns: Optional[int] = None
        self._published = 0
        self._skipped = 0
        self._seq = 0
        # Ring buffer of recently emitted payloads, keyed by seq, for retransmit.
        self._retx: dict[int, bytes] = {}
        self._retx_order: deque[int] = deque()
        self._retx_lock = threading.Lock()
        # Queryable so the server can re-request a dropped frame by seq.
        try:
            z.declare_queryable(KEY_FRAME_GET, self._on_frame_get)
            log.info(f"[Decimator] retransmit queryable on '{KEY_FRAME_GET}'")
        except Exception as e:
            log.warning(f"[Decimator] could not declare retransmit queryable: {e}")

    @staticmethod
    def _declare_reliable_publisher(z, key):
        # Newer zenoh supports a reliability kwarg; fall back gracefully.
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
        """Reply with a buffered frame payload for the requested ?seq=N."""
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
        ws = max(1, ws | 1)                 # force odd, >=1
        half = ws // 2
        interval_ns = int(1e9 / max(fps, 0.1))
        # generous bound on how long to wait for the lookahead half-window
        lookahead_ns = int((half + 0.5) / max(CAMERA_FPS, 1.0) * 1e9)

        with self._lock:
            self._buf.append(FrameEntry(ts_ns, bgr))
            if self._next_tick_ns is None:
                self._next_tick_ns = ts_ns

            # Emit every tick for which we have a complete forward half-window.
            while self._next_tick_ns is not None:
                emitted = self._try_emit_tick(half, interval_ns, lookahead_ns, ts_ns)
                if not emitted:
                    break

            # Bound memory: keep a little more than two windows + one interval.
            max_keep = max(ws + 4, int(2 * CAMERA_FPS / max(fps, 0.1)))
            while len(self._buf) > max_keep:
                self._buf.popleft()

    def _try_emit_tick(self, half: int, interval_ns: int,
                       lookahead_ns: int, newest_ns: int) -> bool:
        tick = self._next_tick_ns
        buf = self._buf
        if not buf:
            return False

        # Index of the frame closest in time to this tick.
        center = min(range(len(buf)), key=lambda i: abs(buf[i].ts_ns - tick))
        frames_after = len(buf) - 1 - center

        # Wait for a full forward half-window UNLESS the stream has already moved
        # well past the tick (avoid stalling if the camera slows/stops).
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
        ok, jbuf = cv2.imencode(".jpg", entry.bgr,
                                [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok or jbuf is None:
            log.warning("JPEG encode failed — skipping tick")
            self._skipped += 1
            return

        # Camera height at capture time (kinematics + live body state).
        body = self._state.get()
        cam_h = self._model.camera_height(body.body_height, body.rotation)

        seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        payload = proto.pack_frame(entry.ts_ns, seq, cam_h, jbuf.tobytes())

        # Buffer for retransmit before publishing.
        with self._retx_lock:
            self._retx[seq] = payload
            self._retx_order.append(seq)
            while len(self._retx_order) > RETX_BUFFER:
                self._retx.pop(self._retx_order.popleft(), None)

        try:
            self._pub.put(payload, encoding=proto.ENC_FRAME)
        except TypeError:
            self._pub.put(payload)           # older zenoh without encoding kwarg
        self._published += 1
        log.debug(f"emit seq={seq} ts={entry.ts_ns//1_000_000}ms cam_h={cam_h:.2f}m "
                  f"cands={n_candidates} size={len(payload)//1024}kB "
                  f"total={self._published}")

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
                _window_size = val | 1       # force odd
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
    conf.insert_json5("mode", '"client"')
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
    log.info(f"Connected. '{KEY_IMAGE}' → '{KEY_OUTPUT}'  @ {fps}Hz  "
             f"window={ws}  jpeg_q={JPEG_QUALITY}")

    model = build_robot_model()
    state = RobotStateTracker(z, ROBOT_NAME, fallback_body_height=FALLBACK_BODY_H)
    decimator = FrameDecimator(z, state, model)

    def on_image(sample):
        try:
            result = decode_ros_image(bytes(sample.payload))
            if result is not None:
                decimator.push(*result)
        except Exception as e:
            log.warning(f"frame handling error: {e}")

    z.declare_subscriber(KEY_IMAGE, on_image)
    z.declare_subscriber(KEY_THROTTLE_FPS, _on_throttle_fps)
    z.declare_subscriber(KEY_WINDOW_SIZE, _on_window_size)

    try:
        while True:
            time.sleep(10)
            log.info(decimator.stats())
    except KeyboardInterrupt:
        pass
    finally:
        z.close()


if __name__ == "__main__":
    main()
# end of frame_decimator.py
