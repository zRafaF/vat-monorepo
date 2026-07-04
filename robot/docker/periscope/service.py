"""
VAT — Remote Periscope service (robot side).

Runs inside the camera process and is fed the live full-resolution equirectangular
frame via :meth:`submit_frame` (no second capture, no fps-capped archive). A worker
thread renders the operator's requested slice, encodes it, and publishes it on the
periscope's OWN Zenoh session (best-effort / DROP / DATA priority, bounded send
buffer) so it never starves the pose stream.

Control flows the other way over the same bus: the client publishes a
``ViewRequest`` (yaw/pitch/fov/aspect/tier) which the robot subscribes to, plus a
``keyframe`` request to force an IDR after a drop.
"""

from __future__ import annotations

import json
import logging
import threading
import time

import zenoh

import vat_protocol as proto
import vat_periscope as psc

from . import config as cfg
from .reproject import SliceRenderer
from .encoder import make_encoder

log = logging.getLogger("periscope.service")


class PeriscopeService:
    def __init__(self):
        self._renderer = SliceRenderer()
        self._lock = threading.Lock()
        self._frame = None                 # latest equirect BGR
        self._frame_ts = 0
        # current requested view (start centred, default tier/aspect/fov)
        self._yaw = 0.0
        self._pitch = 0.0
        self._hfov = float(min(90.0, cfg.MAX_FOV))
        self._aspect = cfg.ASPECT
        self._tier = cfg.RES_TIER
        self._last_req_t = 0.0             # wall time of last view request
        self._out_seq = 0

        self._enc = None
        self._enc_dims = None              # (w, h) the current encoder was built for
        self._last_idr = 0.0
        self._pub_count = 0                # frames published (for the stats log)
        self._last_stat_t = 0.0
        self._stop = threading.Event()

        self._z = self._open_session()
        self._pub = self._declare_frame_pub()
        K = proto.keys(cfg.ROBOT_NAME)
        self._z.declare_subscriber(K["periscope_request"], self._on_request)
        self._z.declare_subscriber(K["periscope_keyframe"], self._on_keyframe)
        log.info(f"[periscope] request←'{K['periscope_request']}'  "
                 f"frame→'{K['periscope_frame']}'  codec_pref={cfg.CODEC} "
                 f"tier={cfg.RES_TIER} aspect={cfg.ASPECT} "
                 f"fov[{cfg.MIN_FOV:.0f},{cfg.MAX_FOV:.0f}] dyn_fps={cfg.FPS_DYNAMIC}")

        self._thread = threading.Thread(target=self._run, name="periscope",
                                        daemon=True)
        self._thread.start()

    # -- Zenoh setup ----------------------------------------------------------
    def _open_session(self) -> zenoh.Session:
        conf = zenoh.Config()
        endpoint = cfg.ZENOH_CONNECT
        if cfg.SO_SNDBUF and cfg.SO_SNDBUF != "0":
            sep = ";" if "#" in endpoint else "#"
            endpoint = f"{endpoint}{sep}so_sndbuf={cfg.SO_SNDBUF}"
        conf.insert_json5("connect/endpoints", f'["{endpoint}"]')
        conf.insert_json5("mode", '"peer"')
        while not self._stop.is_set():
            try:
                return zenoh.open(conf)
            except Exception as e:
                log.warning(f"[periscope] Zenoh connect failed: {e} — retry in 5s")
                time.sleep(5)

    def _declare_frame_pub(self):
        key = proto.keys(cfg.ROBOT_NAME)["periscope_frame"]
        # best-effort + DROP + DATA priority: video yields to pose (DATA_HIGH) and
        # sheds frames under congestion rather than blocking (same discipline as the
        # mapping uplink). Fall back gracefully across binding versions.
        for kwargs in (
            dict(congestion_control=zenoh.CongestionControl.DROP,
                 reliability=zenoh.Reliability.BEST_EFFORT,
                 priority=zenoh.Priority.DATA),
            dict(congestion_control=zenoh.CongestionControl.DROP,
                 priority=zenoh.Priority.DATA),
            dict(congestion_control=zenoh.CongestionControl.DROP),
        ):
            try:
                return self._z.declare_publisher(key, **kwargs)
            except TypeError:
                continue
        return self._z.declare_publisher(key)

    # -- control callbacks ----------------------------------------------------
    def _on_request(self, sample):
        try:
            v = proto.unpack_view_request(bytes(sample.payload))
        except proto.ProtocolError as e:
            log.warning(f"[periscope] bad view request: {e}")
            return
        yaw, pitch, hfov = psc.clamp_view(v.yaw_deg, v.pitch_deg, v.hfov_deg,
                                          cfg.MIN_FOV, cfg.MAX_FOV)
        with self._lock:
            self._yaw, self._pitch, self._hfov = yaw, pitch, hfov
            if v.aspect_w > 0 and v.aspect_h > 0:
                self._aspect = f"{v.aspect_w}:{v.aspect_h}"
            if v.res_tier > 0:
                self._tier = int(v.res_tier)
            self._last_req_t = time.time()

    def _on_keyframe(self, _sample):
        if self._enc is not None:
            self._enc.request_keyframe()

    # -- frame intake (called by the camera thread) ---------------------------
    def submit_frame(self, equirect_bgr, timestamp_ns: int):
        """Hand the periscope the latest decoded full-res frame. Cheap + non-blocking
        (just stores the reference); the worker thread does the heavy lifting."""
        with self._lock:
            self._frame = equirect_bgr
            self._frame_ts = int(timestamp_ns)

    # -- worker ---------------------------------------------------------------
    def _target_fps(self, now: float, last_req: float) -> float:
        if not cfg.FPS_DYNAMIC:
            return cfg.FPS
        active = (now - last_req) < cfg.ACTIVE_WINDOW_S
        return cfg.FPS_MAX if active else cfg.FPS_MIN

    def _viewer_active(self, now: float, last_req: float) -> bool:
        if cfg.VIEWER_TIMEOUT_S <= 0:
            return last_req > 0.0            # stream once any viewer has connected
        return (now - last_req) < cfg.VIEWER_TIMEOUT_S

    def _run(self):
        log.info("[periscope] worker started.")
        while not self._stop.is_set():
            now = time.time()
            with self._lock:
                frame = self._frame
                ts = self._frame_ts
                yaw, pitch, hfov = self._yaw, self._pitch, self._hfov
                aspect, tier, last_req = self._aspect, self._tier, self._last_req_t
            fps = self._target_fps(now, last_req)
            period = 1.0 / max(1.0, fps)
            active = self._viewer_active(now, last_req)
            if now - self._last_stat_t >= 10.0:      # heartbeat every 10 s
                self._last_stat_t = now
                enc = type(self._enc).__name__ if self._enc else "none"
                age = (now - last_req) if last_req > 0 else -1.0
                log.info(f"[periscope] published={self._pub_count} viewer_active={active} "
                         f"last_req_age={age:.1f}s fps={fps:.0f} enc={enc} "
                         f"dims={self._enc_dims} frame={'yes' if frame is not None else 'NONE'}")
            if frame is None or not active:
                time.sleep(min(0.2, max(period, 0.05)))
                continue
            try:
                self._render_encode_publish(frame, ts, yaw, pitch, hfov, aspect, tier)
            except Exception:
                log.exception("[periscope] frame pipeline error")
                time.sleep(0.1)
            # simple rate limit (encode time is already spent)
            time.sleep(period)

    def _render_encode_publish(self, frame, ts, yaw, pitch, hfov, aspect, tier):
        # Render a slightly WIDER slice (overscan) so the client can micro-pan; the
        # header reports the actual rendered geometry.
        render_hfov = min(hfov + cfg.OVERSCAN_DEG, cfg.MAX_FOV)
        slice_bgr, meta = self._renderer.render(frame, yaw, pitch, render_hfov,
                                                aspect, tier)
        h, w = slice_bgr.shape[:2]

        # (Re)build the encoder if the output size changed (zoom crossed the optical
        # floor, or aspect/tier changed). A fresh encoder starts on a keyframe.
        if self._enc is None or self._enc_dims != (w, h):
            if self._enc is not None:
                self._enc.close()
            gop = max(1, int(round(cfg.FPS_MAX * cfg.IDR_INTERVAL_S)))
            self._enc = make_encoder(cfg.CODEC, w, h, cfg.FPS_MAX, cfg.BITRATE,
                                     gop, cfg.JPEG_QUALITY)
            self._enc_dims = (w, h)
            self._last_idr = time.time()

        # Periodic IDR for drop recovery on the best-effort link.
        if (time.time() - self._last_idr) >= cfg.IDR_INTERVAL_S:
            self._enc.request_keyframe()
            self._last_idr = time.time()

        for payload, is_kf in self._enc.encode(slice_bgr):
            self._out_seq += 1
            pf = proto.PeriscopeFrame(
                seq=self._out_seq, timestamp_ns=int(ts), codec=self._enc.codec_id,
                is_keyframe=is_kf, width=meta["width"], height=meta["height"],
                native_w=meta["native_w"], yaw_deg=meta["yaw_deg"],
                pitch_deg=meta["pitch_deg"], hfov_deg=meta["hfov_deg"],
                vfov_deg=meta["vfov_deg"], aspect_w=meta["aspect_w"],
                aspect_h=meta["aspect_h"], optical=meta["optical"], payload=payload)
            buf = proto.pack_periscope_frame(pf)
            try:
                self._pub.put(buf, encoding=proto.ENC_PSCF)
            except TypeError:
                self._pub.put(buf)
            self._pub_count += 1

    def close(self):
        self._stop.set()
        try:
            if self._enc is not None:
                self._enc.close()
        finally:
            try:
                self._z.close()
            except Exception:
                pass
