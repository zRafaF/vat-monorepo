"""
VAT - RGBD (RealSense D435i) single-frame panel: client side.

Short-term goal (NOT a point cloud, NO accumulation): show the LATEST single frame
from the robot's forward depth camera as a panel floating in front of the robot, so
the operator can spot dynamic objects crossing the view. The robot sends only the
ACTIVE stream the client asked for (depth OR color OR nothing) so we never waste
uplink shipping color when depth is on screen.

  * depth : robot sends an 8-bit image = depth scaled to [0, max_range]; the client
            applies a near=warm / far=cool colormap (single channel on the wire).
  * color : robot sends a JPEG RGB image.

This module owns the request/keepalive + decode + telemetry. The 3D placement (panel
quad + D435i frustum, anchored to the live predicted pose at the camera's mount) lives
in prism_viewer, mirroring how the periscope panel/frustum are drawn.
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
_CYCLE = [proto.RGBD_KIND_OFF, proto.RGBD_KIND_DEPTH, proto.RGBD_KIND_COLOR]


class RgbdClient:
    """Publishes an RgbdRequest (kind/fps/range) and decodes the returned single
    frames. Construct with the viewer's low-latency Zenoh session."""

    def __init__(self, z_session, robot_name: str, kind: int = proto.RGBD_KIND_DEPTH,
                 fps: int = 20, max_range_m: float = 4.0, range_gate: bool = False):
        self._z = z_session
        self._K = proto.keys(robot_name)
        self.kind = int(kind)
        self.fps = int(fps)
        self.max_range_mm = int(max_range_m * 1000)
        self.range_gate = bool(range_gate)
        self._seq = 0

        self._lock = threading.Lock()
        self._img = None                 # latest decoded RGB uint8 (H,W,3) (depth colorized / color)
        self._depth8 = None              # latest raw 8-bit depth (H,W) for 3D deprojection; None for color
        self._meta = None                # dict: kind,hfov,vfov,width,height,max_range_mm,min_depth_mm
        self._last_frame_t = 0.0
        self._last_pub_t = 0.0
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
        self._last_log_t = 0.0
        self.publish()
        log.info(f"[rgbd] client on '{self._K['rgbd_frame']}' "
                 f"req->'{self._K['rgbd_request']}' kind={_KIND_NAME.get(self.kind)}")

    # -- controls -------------------------------------------------------------
    def cycle_kind(self):
        """off -> depth -> color -> off (the toggle key)."""
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
        r = proto.RgbdRequest(kind=self.kind, fps=self.fps,
                              max_range_mm=self.max_range_mm, flags=flags, seq=self._seq)
        try:
            self._pub.put(proto.pack_rgbd_request(r), encoding=proto.ENC_RGBR)
        except TypeError:
            self._pub.put(proto.pack_rgbd_request(r))
        except Exception as e:
            log.debug(f"[rgbd] request publish failed: {e}")
        self._last_pub_t = time.time()

    def keepalive(self, interval_s: float = 0.5):
        """Re-send the current request so the robot keeps streaming (viewer-timeout).
        Also emits a periodic diagnostic so we can see if the stream keeps flowing."""
        now = time.time()
        if self.enabled and (now - self._last_pub_t) >= interval_s:
            self.publish()
        if now - self._last_log_t >= 3.0:
            self._last_log_t = now
            age = (now - self._last_frame_t) if self._last_frame_t else -1.0
            log.info(f"[rgbd] rx={self._n_recv} dec={self._n_dec} fps={self.fps():.1f} "
                     f"last_frame={age:.1f}s kind={self.kind} enabled={self.enabled} "
                     f"stale={self.stale()}")

    # -- incoming frames (Zenoh callback thread) ------------------------------
    def _decode(self, f):
        """Return (rgb, depth8): rgb is the display image (colorized depth / color);
        depth8 is the raw 8-bit depth (near..far over max_range) for 3D deprojection,
        or None for the color stream."""
        import cv2
        arr = np.frombuffer(f.payload, np.uint8)
        if f.kind == proto.RGBD_KIND_COLOR:
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            return (None, None) if bgr is None else (bgr[:, :, ::-1].copy(), None)
        # depth: single-channel 8-bit, 0 = invalid, 1..255 = near..far over max_range
        d = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
        if d is None:
            return None, None
        if d.ndim == 3:
            d = d[:, :, 0]
        d = d.astype(np.uint8)
        inv = (255 - d).astype(np.uint8)                 # near -> high -> warm
        try:
            cm = cv2.applyColorMap(inv, cv2.COLORMAP_TURBO)
        except Exception:
            cm = cv2.applyColorMap(inv, cv2.COLORMAP_JET)
        cm[d == 0] = 0                                    # invalid -> black
        return cm[:, :, ::-1].copy(), d                  # (RGB, raw depth8)

    def _on_frame(self, sample):
        try:
            f = proto.unpack_rgbd_frame(bytes(sample.payload))
        except proto.ProtocolError:
            return
        self._n_recv += 1
        self._last_bytes = len(f.payload)
        try:
            rgb, depth8 = self._decode(f)
        except Exception as e:
            self._decode_err = str(e)[:60]
            if not self._decode_warned:
                log.warning(f"[rgbd] decode failed ({e}); need opencv-python")
                self._decode_warned = True
            return
        if rgb is None:
            return
        self._n_dec += 1
        tnow = time.monotonic()
        self._recv_t.append(tnow)
        if len(self._recv_t) >= 2:
            span = self._recv_t[-1] - self._recv_t[0]
            if span > 1e-6:
                self._fps = (len(self._recv_t) - 1) / span
        with self._lock:
            self._img = rgb
            self._depth8 = depth8
            self._meta = dict(kind=f.kind, hfov=f.hfov_deg, vfov=f.vfov_deg,
                              width=f.width, height=f.height,
                              max_range_mm=f.max_range_mm, min_depth_mm=f.min_depth_mm)
            self._last_frame_t = time.time()

    # -- accessors (GL thread) ------------------------------------------------
    def latest_image(self):
        with self._lock:
            return self._img

    def latest_meta(self):
        with self._lock:
            return None if self._meta is None else dict(self._meta)

    def latest_depth(self):
        """(raw 8-bit depth (H,W), meta) for 3D deprojection; (None, None) for color."""
        with self._lock:
            if self._depth8 is None or self._meta is None:
                return None, None
            return self._depth8, dict(self._meta)

    def frame_id(self) -> int:
        """Monotonic count of decoded frames (viewer uses it to redraw only on new data)."""
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
            return f"rgbd: {_KIND_NAME.get(self.kind)} - no frames yet (relay up? realsense node?)"
        if self._n_dec == 0:
            return f"rgbd: {self._n_recv} rx but 0 decoded ({self._decode_err or 'opencv?'})"
        m = self.latest_meta() or {}
        near = m.get("min_depth_mm", 0)
        near_s = f" near {near/1000:.2f}m" if near else ""
        return (f"rgbd: {_KIND_NAME.get(self.kind)} {m.get('width','?')}x{m.get('height','?')} "
                f"{self.fps():.0f}fps {self._last_bytes//1024}KB rng{self.max_range_mm/1000:.1f}m"
                f"{'*gate' if self.range_gate else ''}{near_s} rx{self._n_recv}")
