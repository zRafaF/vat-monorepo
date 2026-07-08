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
                      (e.g. /dev/video10 fed by `make theta-uvc` (gstthetauvc);
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
from kinematics import (
    build_robot_model, RobotStateTracker, LowStateTracker,
    camera_height_above_ground)
from frame_archive import FrameArchive

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
# Cap the TCP send buffer (bytes) on THIS process's uplink to the router. The
# bulk frame stream is the only thing that can bloat the kernel socket buffer;
# a large buffer lets a burst of frame bytes queue up ahead of the realtime pose
# packets (which ride a separate session/link but share the physical uplink and
# the bottleneck queue) → the pose "chug". Bounding so_sndbuf limits how many
# frame bytes can sit un-preemptable in flight, trading a little frame throughput
# for lower pose latency under load. Empty/0 = leave the OS default (off).
# ~256 KiB ≈ a couple of frames; lower toward 131072 for more aggressive latency,
# raise if frame throughput on a fast link suffers. Tuned in vat.env.
FRAME_SO_SNDBUF = os.environ.get("FRAME_SO_SNDBUF", "").strip()
JPEG_QUALITY    = int(os.environ.get("JPEG_QUALITY", "85"))
# Transmit codec for the live frame: "webp" (≈25-35% smaller than JPEG at equal quality
# → less robot-uplink bandwidth, which is what starves the pose stream) or "jpeg". The
# server decodes transparently (cv2.imdecode sniffs the format from magic bytes), so only
# this encoder changes. WEBP_QUALITY is 0-100 (visually ~= JPEG quality).
FRAME_CODEC     = os.environ.get("FRAME_CODEC", "webp").strip().lower()
WEBP_QUALITY    = int(os.environ.get("WEBP_QUALITY", "82"))
LOSSLESS        = os.environ.get("LOSSLESS", "").lower() in ("1", "true", "yes")
CAMERA_FPS      = float(os.environ.get("CAMERA_FPS", "30.0"))
SHARP_DOWNSCALE = float(os.environ.get("SHARPNESS_DOWNSCALE", "0.5"))
FALLBACK_BODY_H = float(os.environ.get("FALLBACK_BODY_HEIGHT", "0.30"))

# Camera height stamped into every frame — drives PRISM's per-submap METRIC SCALE,
# so it MUST be consistent (a varying/wrong height makes each submap a different
# scale → the online map misaligns). gradio uses one constant height and aligns
# perfectly, so 'const' is the default. 'legs' derives a stable, stance-aware
# height from the leg FK ground plane (correct when the dog crouches/stands).
CAMERA_HEIGHT_MODE  = os.environ.get("CAMERA_HEIGHT_MODE", "const").strip().lower()
CAMERA_HEIGHT_CONST = float(os.environ.get("CAMERA_HEIGHT_M", "1.15"))   # measured ground→camera
STICK = np.array([
    float(os.environ.get("STICK_OFFSET_X", "-0.65")),
    float(os.environ.get("STICK_OFFSET_Y", "0.0")),
    float(os.environ.get("STICK_OFFSET_Z", "0.85")),
], dtype=np.float64)
RETX_BUFFER     = int(os.environ.get("RETX_BUFFER", "256"))
CAPTURE_RETRY_S = float(os.environ.get("CAPTURE_RETRY_S", "3.0"))
# Frozen-stream watchdog. When the Theta drops offline (or the host `theta-uvc`
# gstthetauvc feed dies), the v4l2 loopback keeps handing back the SAME cached
# frame instead of failing — so the container would publish that one stale frame
# forever and need a manual restart. Real frames always differ by sensor noise,
# so if the captured frame is byte-identical for this many seconds we treat the
# stream as dead: stop publishing and reopen the capture, which transparently
# picks up a restarted `theta-uvc` with NO container restart. 0 disables.
STALE_TIMEOUT_S = float(os.environ.get("STALE_TIMEOUT_S", "2.5"))

# Capture source
THETA_GST_PIPELINE = os.environ.get("THETA_GST_PIPELINE", "").strip()
THETA_DEVICE       = os.environ.get("THETA_DEVICE", "").strip()
THETA_MODE         = os.environ.get("THETA_MODE", "2K").strip()

# Real-time transmit downscale (0/unset → send the capture resolution as-is)
TRANSMIT_WIDTH  = int(os.environ.get("TRANSMIT_WIDTH", "0"))
TRANSMIT_HEIGHT = int(os.environ.get("TRANSMIT_HEIGHT", "0"))

# Full-res frame archive (written off the real-time path; see frame_archive.py)
ARCHIVE_ENABLE       = os.environ.get("ARCHIVE_ENABLE", "").lower() in ("1", "true", "yes")
ARCHIVE_DIR          = os.environ.get("ARCHIVE_DIR", "/archive")
ARCHIVE_MAX_BYTES    = os.environ.get("ARCHIVE_MAX_BYTES", "10GB")
ARCHIVE_JPEG_QUALITY = int(os.environ.get("ARCHIVE_JPEG_QUALITY", "92"))

_KEYS = proto.keys(ROBOT_NAME)
KEY_OUTPUT       = _KEYS["camera_frame"]
KEY_FRAME_GET    = _KEYS["camera_frame_get"]
KEY_ARCHIVE_GET  = _KEYS["camera_archive_get"]
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

    def __init__(self, decimator: "FrameDecimator", periscope=None, vo_runner=None):
        super().__init__(daemon=True)
        self._decimator = decimator
        self._vo_runner = vo_runner        # optional visual-odometry (VO_ENABLE)
        # Optional remote-periscope service: fed the SAME live full-res frame (no
        # second capture, no fps-capped archive). submit_frame is non-blocking.
        self._periscope = periscope
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
                last_sig = None
                last_change = time.time()
                warned_frozen = False
                while not self._stop.is_set():
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        log.warning("[Capture] read failed — reopening stream")
                        break
                    now = time.time()
                    # Cheap content signature over a sparse grid; live frames differ
                    # by sensor noise every read, a frozen stream is byte-identical.
                    sig = hash(frame[::32, ::32].tobytes())
                    if sig != last_sig:
                        last_sig = sig
                        last_change = now
                        warned_frozen = False
                    else:
                        # Duplicate frame: do NOT publish it (don't feed the map a
                        # stale frame). If it stays frozen past the timeout, drop the
                        # handle and reopen so a restarted theta-uvc is picked up.
                        if not warned_frozen:
                            log.warning("[Capture] identical frames — camera may be "
                                        "frozen/offline; pausing publish")
                            warned_frozen = True
                        if STALE_TIMEOUT_S > 0 and (now - last_change) > STALE_TIMEOUT_S:
                            log.warning(f"[Capture] stream FROZEN for "
                                        f"{now - last_change:.1f}s — reopening capture "
                                        "(reboot the camera + re-run 'make theta-uvc'; "
                                        "recovery is automatic, no container restart)")
                            break
                        continue
                    self._frames += 1
                    ts_ns = time.time_ns()
                    self._decimator.push(ts_ns, frame)
                    if self._periscope is not None:
                        self._periscope.submit_frame(frame, ts_ns)
                    if self._vo_runner is not None:
                        self._vo_runner.on_frame(frame, ts_ns)
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

    def __init__(self, z: zenoh.Session, state: RobotStateTracker, model,
                 archive=None, leg_tracker=None):
        self._z = z
        self._state = state
        self._model = model
        self._legs = leg_tracker          # only set in CAMERA_HEIGHT_MODE='legs'
        self._archive = archive
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
        if self._archive is not None:
            try:
                z.declare_queryable(KEY_ARCHIVE_GET, self._on_archive_get)
                log.info(f"[Decimator] archive queryable on '{KEY_ARCHIVE_GET}'")
            except Exception as e:
                log.warning(f"[Decimator] could not declare archive queryable: {e}")

    @staticmethod
    def _declare_reliable_publisher(z, key):
        # Priority DATA (not DATA_HIGH): the bulk camera stream must sit BELOW the
        # realtime pose (DATA_HIGH), so on the shared robot WiFi uplink the pose is
        # scheduled ahead of the big JPEG frames instead of competing at equal priority
        # (the cause of the pose "chug" during frame bursts). Kept RELIABLE+BLOCK so
        # mapping frames still arrive; if the uplink is genuinely saturated (confirm with
        # tools/latency_probe.py), switching this to DROP lets it shed frames under
        # congestion and frees more airtime for pose (frames are recoverable via the
        # camera_frame_get retransmit query).
        for kwargs in (
            dict(congestion_control=zenoh.CongestionControl.BLOCK,
                 reliability=zenoh.Reliability.RELIABLE,
                 priority=zenoh.Priority.DATA),
            dict(congestion_control=zenoh.CongestionControl.BLOCK,
                 priority=zenoh.Priority.DATA),
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

    def _on_archive_get(self, query):
        try:
            params = query.parameters if hasattr(query, "parameters") else \
                query.selector.parameters
            seq = int(params["seq"]) if "seq" in params else -1
            payload = self._archive.get(seq) if self._archive is not None else None
            if payload is not None:
                query.reply(KEY_ARCHIVE_GET, payload)
                log.debug(f"[Archive] served seq={seq} ({len(payload)//1024}kB)")
            else:
                query.reply_err(f"archive seq {seq} not found".encode())
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
        # Real-time TRANSMIT copy: downscale (if configured) then encode small.
        tx = entry.bgr
        if TRANSMIT_WIDTH > 0 and TRANSMIT_HEIGHT > 0:
            tx = cv2.resize(entry.bgr, (TRANSMIT_WIDTH, TRANSMIT_HEIGHT),
                            interpolation=cv2.INTER_AREA)
        if LOSSLESS:
            ok, jbuf = cv2.imencode(".png", tx)
        elif FRAME_CODEC == "webp":
            ok, jbuf = cv2.imencode(".webp", tx,
                                    [cv2.IMWRITE_WEBP_QUALITY, WEBP_QUALITY])
            if not ok:      # OpenCV without WebP support → fall back to JPEG
                ok, jbuf = cv2.imencode(".jpg", tx,
                                        [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        else:
            ok, jbuf = cv2.imencode(".jpg", tx,
                                    [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok or jbuf is None:
            log.warning("encode failed — skipping tick")
            self._skipped += 1
            return

        body = self._state.get()
        # Camera height for PRISM's metric scale. Keep it CONSISTENT across frames
        # (the cause of the misaligned online map was a varying/wrong height).
        if CAMERA_HEIGHT_MODE == "legs" and self._legs is not None:
            legs, ok = self._legs.get()
            cam_h, _bh, _co = camera_height_above_ground(
                legs if ok else {}, body.rotation, STICK,
                fallback_base_height=FALLBACK_BODY_H)
        else:
            cam_h = CAMERA_HEIGHT_CONST

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

        # Full-res TWIN → archive (same seq/ts/cam_h). The heavy full-res encode
        # and disk I/O run on the archive's own thread; submit() never blocks us.
        if self._archive is not None:
            self._archive.submit(seq, entry.ts_ns, cam_h, entry.bgr)

        log.debug(f"emit seq={seq} ts={entry.ts_ns//1_000_000}ms cam_h={cam_h:.2f}m "
                  f"cands={n_candidates} tx={len(payload)//1024}kB total={self._published}")

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
    # Optionally bound the send buffer on the frame uplink (see FRAME_SO_SNDBUF).
    # so_sndbuf is a per-endpoint locator option honoured for TCP/TLS links.
    endpoint = ZENOH_CONNECT
    if FRAME_SO_SNDBUF and FRAME_SO_SNDBUF != "0":
        sep = ";" if "#" in endpoint else "#"
        endpoint = f"{endpoint}{sep}so_sndbuf={FRAME_SO_SNDBUF}"
        log.info(f"[camera] uplink send buffer capped: so_sndbuf={FRAME_SO_SNDBUF}")
    conf.insert_json5("connect/endpoints", f'["{endpoint}"]')
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
    if LOSSLESS:
        enc_label = "PNG (lossless)"
    elif FRAME_CODEC == "webp":
        enc_label = f"WebP q={WEBP_QUALITY} (JPEG q={JPEG_QUALITY} fallback)"
    else:
        enc_label = f"JPEG q={JPEG_QUALITY}"
    log.info(f"Connected. Theta UVC → '{KEY_OUTPUT}'  @ {fps}Hz  "
             f"window={ws}  encode={enc_label}")
    _tx = f"{TRANSMIT_WIDTH}x{TRANSMIT_HEIGHT}" if TRANSMIT_WIDTH > 0 else "capture-res"
    log.info(f"Transmit={_tx}  archive="
             f"{('on → ' + ARCHIVE_DIR) if ARCHIVE_ENABLE else 'off'}")

    model = build_robot_model()
    state = RobotStateTracker(z, ROBOT_NAME, fallback_body_height=FALLBACK_BODY_H)
    leg_tracker = None
    if CAMERA_HEIGHT_MODE == "legs":
        leg_tracker = LowStateTracker(z, ROBOT_NAME)          # leg FK → ground plane
        log.info(f"[cam-height] mode=legs (stance-aware, stick={tuple(STICK)})")
    else:
        log.info(f"[cam-height] mode=const → {CAMERA_HEIGHT_CONST:.2f} m "
                 f"(consistent scale for the online map)")

    archive = None
    if ARCHIVE_ENABLE:
        try:
            archive = FrameArchive(ARCHIVE_DIR, ARCHIVE_MAX_BYTES,
                                   jpeg_quality=ARCHIVE_JPEG_QUALITY)
        except Exception as e:
            log.warning(f"[archive] disabled — init failed: {e}")
            archive = None
    decimator = FrameDecimator(z, state, model, archive=archive, leg_tracker=leg_tracker)

    # Remote periscope (optional): shares this process's live full-res frame. Its
    # own Zenoh session isolates its QoS from the mapping stream. A failure here
    # must never take down the camera, so it is fully guarded.
    periscope = None
    try:
        from periscope import config as psc_cfg, PeriscopeService
        if psc_cfg.ENABLE:
            periscope = PeriscopeService()
    except Exception as e:
        log.warning(f"[periscope] disabled (init failed): {e}")
        periscope = None

    # Visual odometry (optional, VO_ENABLE): front-pinhole flow -> body motion
    # direction for the fuser (strafe/rotation foolproofing). Guarded; never blocks.
    vo_runner = None
    try:
        from visual_odometry import VoRunner
        vo_runner = VoRunner(z, ROBOT_NAME, lambda: leg_tracker.get_imu_odom()[1][2])
        if not vo_runner.enabled:
            vo_runner = None
    except Exception as e:
        log.warning(f"[vo] disabled (init failed): {e}")
        vo_runner = None

    capture = ThetaCapture(decimator, periscope=periscope, vo_runner=vo_runner)
    capture.start()

    z.declare_subscriber(KEY_THROTTLE_FPS, _on_throttle_fps)
    z.declare_subscriber(KEY_WINDOW_SIZE, _on_window_size)

    try:
        while True:
            time.sleep(10)
            extra = f"  {archive.stats()}" if archive is not None else ""
            log.info(f"{decimator.stats()} captured={capture.frames}{extra}")
    except KeyboardInterrupt:
        pass
    finally:
        capture.stop()
        if periscope is not None:
            periscope.close()
        if archive is not None:
            archive.close()
        z.close()


if __name__ == "__main__":
    main()
