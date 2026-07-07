"""
VAT - RGBD (RealSense D435i) client: receive a robot-deprojected voxel cloud.

The robot deprojects + voxel-downsamples + colorizes (near=warm) the latest depth
frame and ships it as a compact pack_pcd cloud (zlib + quant). This client just
unpacks it and hands the CAMERA-OPTICAL points to the viewer, which transforms
them by the live pose + D435i mount and renders them. No client-side deprojection.

Single frame, no accumulation: each cloud replaces the previous one. Config
(kind/fps/max_range/range-gate) is published to the robot as an RgbdRequest;
keepalive runs on a dedicated thread so the stream never stalls with the render loop.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque

import numpy as np

import vat_protocol as proto

log = logging.getLogger("rgbd.client")

_KIND_NAME = {proto.RGBD_KIND_OFF: "off", proto.RGBD_KIND_DEPTH: "depth",
              proto.RGBD_KIND_COLOR: "color"}
_CYCLE = [proto.RGBD_KIND_OFF, proto.RGBD_KIND_DEPTH]   # depth voxels or off


class RgbdClient:
    def __init__(self, z_session, robot_name: str, kind: int = proto.RGBD_KIND_DEPTH,
                 fps: int = 10, max_range_m: float = 2.0, range_gate: bool = False):
        self._z = z_session
        self._K = proto.keys(robot_name)
        self.kind = int(kind)
        self.req_fps = int(fps)
        self.max_range_mm = int(max_range_m * 1000)
        self.range_gate = bool(range_gate)
        self._seq = 0

        self._lock = threading.Lock()
        self._pts = None                 # latest (N,3) optical-frame points (float32)
        self._cols = None                # latest (N,4) rgba float32
        self._npts = 0
        self._last_frame_t = 0.0
        self._last_pub_t = 0.0
        self._last_log_t = 0.0
        # telemetry
        self._recv_t = deque(maxlen=60)
        self._fps = 0.0
        self._n_recv = 0
        self._n_dec = 0
        self._last_bytes = 0
        self._decode_err = None
        self._decode_warned = False

        self._pub = self._z.declare_publisher(self._K["rgbd_request"])
        self._sub = self._z.declare_subscriber(self._K["rgbd_frame"], self._on_frame)
        self.publish()
        self._ka_stop = threading.Event()
        self._ka_thread = threading.Thread(target=self._ka_loop, name="rgbd-keepalive", daemon=True)
        self._ka_thread.start()
        log.info(f"[rgbd] cloud client on '{self._K['rgbd_frame']}' "
                 f"req->'{self._K['rgbd_request']}' kind={_KIND_NAME.get(self.kind)}")

    # -- controls -------------------------------------------------------------
    def cycle_kind(self):
        i = _CYCLE.index(self.kind) if self.kind in _CYCLE else 0
        self.kind = _CYCLE[(i + 1) % len(_CYCLE)]
        self.publish()
        return _KIND_NAME.get(self.kind)

    def set_range(self, meters: float):
        self.max_range_mm = int(max(0.2, meters) * 1000)
        self.publish()

    def toggle_range_gate(self):
        self.range_gate = not self.range_gate
        self.publish()
        return self.range_gate

    @property
    def enabled(self) -> bool:
        return self.kind != proto.RGBD_KIND_OFF

    def publish(self):
        self._seq += 1
        flags = proto.RGBD_FLAG_RANGE_GATE if self.range_gate else 0
        r = proto.RgbdRequest(kind=self.kind, fps=self.req_fps,
                              max_range_mm=self.max_range_mm, flags=flags, seq=self._seq)
        try:
            self._pub.put(proto.pack_rgbd_request(r), encoding=proto.ENC_RGBR)
        except TypeError:
            self._pub.put(proto.pack_rgbd_request(r))
        except Exception as e:
            log.debug(f"[rgbd] request publish failed: {e}")
        self._last_pub_t = time.time()

    def _ka_loop(self):
        while not self._ka_stop.wait(0.5):
            if self.enabled:
                try:
                    self.publish()
                except Exception:
                    pass

    def keepalive(self, interval_s: float = 0.5):
        """Called from the render tick for the periodic diagnostic only; keepalive
        publishing happens on the dedicated _ka_loop thread."""
        now = time.time()
        if now - self._last_log_t >= 3.0:
            self._last_log_t = now
            age = (now - self._last_frame_t) if self._last_frame_t else -1.0
            log.info(f"[rgbd] rx={self._n_recv} dec={self._n_dec} fps={self.fps():.1f} "
                     f"pts={self._npts} last={age:.1f}s kind={self.kind} stale={self.stale()}")

    # -- incoming clouds (Zenoh callback thread) -----------------------------
    def _on_frame(self, sample):
        raw = bytes(sample.payload)
        self._n_recv += 1
        self._last_bytes = len(raw)
        try:
            _ver, xyz, rgb, _snap, _since = proto.unpack_pcd(raw)
        except Exception as e:
            self._decode_err = str(e)[:60]
            if not self._decode_warned:
                log.warning(f"[rgbd] unpack_pcd failed ({e})")
                self._decode_warned = True
            return
        self._n_dec += 1
        tnow = time.monotonic()
        self._recv_t.append(tnow)
        if len(self._recv_t) >= 2:
            span = self._recv_t[-1] - self._recv_t[0]
            if span > 1e-6:
                self._fps = (len(self._recv_t) - 1) / span
        rgba = np.ones((rgb.shape[0], 4), np.float32)
        rgba[:, :3] = rgb.astype(np.float32)          # unpack_pcd gives rgb in [0,1]
        with self._lock:
            self._pts = xyz.astype(np.float32)        # camera-optical frame
            self._cols = rgba
            self._npts = int(xyz.shape[0])
            self._last_frame_t = time.time()

    # -- accessors (GL thread) -----------------------------------------------
    def latest_points(self):
        """(optical-frame points (N,3) f32, rgba (N,4) f32) or (None, None)."""
        with self._lock:
            if self._pts is None:
                return None, None
            return self._pts, self._cols

    def frame_id(self) -> int:
        return self._n_dec

    def fps(self) -> float:
        if not self._recv_t or (time.monotonic() - self._recv_t[-1]) > 1.0:
            return 0.0
        return self._fps

    def stale(self, timeout_s: float = 1.5) -> bool:
        with self._lock:
            return (time.time() - self._last_frame_t) > timeout_s

    def status_text(self) -> str:
        if not self.enabled:
            return "rgbd: off (x)"
        if self._n_recv == 0:
            return f"rgbd: {_KIND_NAME.get(self.kind)} - no cloud yet (relay/realsense up?)"
        if self._n_dec == 0:
            return f"rgbd: {self._n_recv} rx but 0 unpacked ({self._decode_err or '?'})"
        return (f"rgbd: {_KIND_NAME.get(self.kind)} {self._npts}pts {self.fps():.0f}fps "
                f"{self._last_bytes//1024}KB rng{self.max_range_mm/1000:.1f}m"
                f"{'*gate' if self.range_gate else ''} rx{self._n_recv}")
