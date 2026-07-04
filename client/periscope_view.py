"""
VAT - Remote Periscope: client side.

Owns the operator's aim state, publishes ``ViewRequest`` to the robot, subscribes
to the encoded ``PeriscopeFrame`` stream and decodes it (PyAV for H.264/HEVC,
OpenCV for the MJPEG fallback), and computes the two camera-anchored frustum
wireframes (requested vs. actual) for the 3D viewer.

Decoding runs in the Zenoh callback thread; the latest RGB image + actual view
params are stored under a lock and pulled by the viewer on the GL thread.
"""

from __future__ import annotations

import logging
import threading
import time

import numpy as np

import vat_protocol as proto
import vat_periscope as psc

log = logging.getLogger("periscope.client")


class _Decoder:
    """Lazily-created decoder keyed by codec id. PyAV for H.26x, cv2 for MJPEG."""

    def __init__(self):
        self._codec = None
        self._dec = None

    def _make(self, codec_id):
        if codec_id == proto.PSCOPE_CODEC_MJPEG:
            import cv2                      # noqa: F401 (probe availability)
            return ("mjpeg", None)
        name = "hevc" if codec_id == proto.PSCOPE_CODEC_HEVC else "h264"
        import av
        return (name, av.CodecContext.create(name, "r"))

    def decode(self, codec_id, payload: bytes):
        """Return the newest decoded RGB uint8 (H,W,3), or None."""
        if codec_id != self._codec:
            self._codec, self._dec = None, None
            self._kind, self._dec = self._make(codec_id)
            self._codec = codec_id
        if self._kind == "mjpeg":
            import cv2
            arr = np.frombuffer(payload, np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            return None if bgr is None else bgr[:, :, ::-1].copy()   # BGR->RGB
        import av
        pkt = av.Packet(payload)
        frames = self._dec.decode(pkt)
        if not frames:
            return None
        return frames[-1].to_ndarray(format="rgb24")


class PeriscopeClient:
    """Aim state + publisher + decoded-frame buffer. Construct with the viewer's
    low-latency Zenoh session so it shares the transport."""

    def __init__(self, z_session, robot_name: str, min_fov: float = 20.0,
                 max_fov: float = 130.0, default_fov: float = 90.0,
                 default_tier: int = 480, default_aspect: str = "1:1"):
        self._z = z_session
        self._K = proto.keys(robot_name)
        self._min_fov, self._max_fov = min_fov, max_fov
        self.enabled = True

        # requested aim
        self.yaw = 0.0
        self.pitch = 0.0
        self.hfov = float(np.clip(default_fov, min_fov, max_fov))
        self.aspect = default_aspect
        self.tier = int(default_tier)
        self._seq = 0

        self._decoder = _Decoder()
        self._lock = threading.Lock()
        self._img = None                      # latest decoded RGB
        self._actual = None                   # latest actual view meta (dict)
        self._last_frame_t = 0.0
        self._decode_warned = False

        self._pub = self._z.declare_publisher(self._K["periscope_request"])
        self._pub_kf = self._z.declare_publisher(self._K["periscope_keyframe"])
        self._z.declare_subscriber(self._K["periscope_frame"], self._on_frame)
        self.publish()                        # tell the robot our initial view
        log.info(f"[periscope] client on '{self._K['periscope_frame']}' "
                 f"req->'{self._K['periscope_request']}'")

    # -- aim controls (call from the viewer's input handler) ------------------
    def nudge(self, dyaw: float, dpitch: float):
        self.yaw, self.pitch, self.hfov = psc.clamp_view(
            self.yaw + dyaw, self.pitch + dpitch, self.hfov, self._min_fov, self._max_fov)
        self.publish()

    def zoom(self, factor: float):
        """factor <1 zooms in (narrower FOV), >1 zooms out."""
        _, _, self.hfov = psc.clamp_view(self.yaw, self.pitch, self.hfov * factor,
                                         self._min_fov, self._max_fov)
        self.publish()

    def set_tier(self, tier: int):
        self.tier = int(tier)
        self.publish()

    def cycle_aspect(self):
        order = ["1:1", "4:3", "16:9"]
        i = order.index(self.aspect) if self.aspect in order else -1
        self.aspect = order[(i + 1) % len(order)]
        self.publish()

    def request_keyframe(self):
        try:
            self._pub_kf.put(b"1")
        except Exception:
            pass

    def publish(self):
        aw, ah = psc.parse_aspect(self.aspect)
        self._seq += 1
        v = proto.ViewRequest(yaw_deg=self.yaw, pitch_deg=self.pitch,
                              hfov_deg=self.hfov, res_tier=self.tier,
                              aspect_w=int(round(aw)), aspect_h=int(round(ah)),
                              seq=self._seq, timestamp_ns=time.time_ns())
        try:
            self._pub.put(proto.pack_view_request(v), encoding=proto.ENC_VREQ)
        except TypeError:
            self._pub.put(proto.pack_view_request(v))
        except Exception as e:
            log.debug(f"[periscope] view request publish failed: {e}")

    # -- incoming frames (Zenoh callback thread) ------------------------------
    def _on_frame(self, sample):
        try:
            f = proto.unpack_periscope_frame(bytes(sample.payload))
        except proto.ProtocolError:
            return
        try:
            rgb = self._decoder.decode(f.codec, f.payload)
        except Exception as e:
            if not self._decode_warned:
                log.warning(f"[periscope] decode unavailable for codec {f.codec} "
                            f"({e}); install PyAV for H.26x. MJPEG works without it.")
                self._decode_warned = True
            return
        if rgb is None:
            return
        with self._lock:
            self._img = rgb
            self._actual = dict(yaw=f.yaw_deg, pitch=f.pitch_deg, hfov=f.hfov_deg,
                                vfov=f.vfov_deg, aspect_w=f.aspect_w,
                                aspect_h=f.aspect_h, optical=f.optical,
                                width=f.width, height=f.height, native_w=f.native_w)
            self._last_frame_t = time.time()

    # -- accessors (GL thread) ------------------------------------------------
    def latest_image(self):
        with self._lock:
            return self._img

    def latest_actual(self):
        with self._lock:
            return None if self._actual is None else dict(self._actual)

    def stale(self, timeout_s: float = 2.0) -> bool:
        with self._lock:
            return (time.time() - self._last_frame_t) > timeout_s

    # -- frustum geometry -----------------------------------------------------
    def req_vfov(self) -> float:
        """Vertical FOV (deg) of the currently requested view, from its aspect."""
        aw, ah = psc.parse_aspect(self.aspect)
        return psc.vfov_from_hfov(self.hfov, aw, ah)

    @staticmethod
    def frustum_world(cam_pos, base_R, yaw_deg, pitch_deg, hfov_deg,
                      vfov_deg, far_m=3.0):
        """Return an (2N, 3) float32 array of world-space line segment endpoints for
        a VisPy Line(connect='segments'), for a frustum anchored at ``cam_pos`` (the
        CAMERA world position, not the base) and aimed ``(yaw, pitch)`` relative to
        the robot's heading. ``base_R`` is the robot's DISPLAYED 3x3 body->world
        rotation (so the frustum tracks the drawn avatar). Camera-frame edges come
        from vat_periscope.frustum_edges."""
        cam_pos = np.asarray(cam_pos, dtype=np.float64).reshape(3)
        base_R = np.asarray(base_R, dtype=np.float64).reshape(3, 3)
        # robot heading = yaw of base +x forward (first column of R) in world
        fwd0 = base_R[:, 0]
        heading = np.arctan2(fwd0[1], fwd0[0])
        tot_yaw = heading + np.radians(yaw_deg)
        pit = np.radians(pitch_deg)
        # aimed forward direction in world
        fwd = np.array([np.cos(pit) * np.cos(tot_yaw),
                        np.cos(pit) * np.sin(tot_yaw),
                        np.sin(pit)])
        world_up = np.array([0.0, 0.0, 1.0])
        right = np.cross(fwd, world_up)
        rn = np.linalg.norm(right)
        right = right / rn if rn > 1e-6 else np.array([1.0, 0, 0])
        up = np.cross(right, fwd)
        R = np.column_stack([right, up, fwd])          # camera(+x,+y,+z) -> world
        edges = psc.frustum_edges(hfov_deg, vfov_deg, far_m=far_m)   # (N,2,3) camera
        pts = edges.reshape(-1, 3) @ R.T + cam_pos
        return pts.astype(np.float32)
