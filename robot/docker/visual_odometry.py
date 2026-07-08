"""
VAT - lightweight monocular visual odometry (360 -> front pinhole).

Purpose: foolproof the wheel+IMU dead-reckoning by observing the one thing the
wheels can't — the horizontal MOTION DIRECTION (forward vs strafe) — independent of
leg slip. The wheels give the speed MAGNITUDE; VO gives the direction; the fuser
combines them, so a rotate-with-strafe stops reading as pure rotation.

How (kept deliberately simple + fast for the Jetson):
  1. Reproject a rectilinear FRONT pinhole from the equirect frame (reuse the
     periscope geometry) and track sparse KLT features between consecutive frames.
  2. DE-ROTATE the flow with the gyro yaw-rate (subtract the uniform yaw-induced
     shift), so turning doesn't create a false strafe signal.
  3. From the residual (translational) flow: mean horizontal flow -> lateral motion,
     radial divergence -> forward motion. Output a unit body-frame direction + a
     confidence. Yaw-only motion de-rotates to ~0 residual -> low confidence -> ignored.

Runs in the camera process (fed the live full-res frame like the periscope) and
publishes a VoDelta the fuser consumes. Fully OPTIONAL and gated: if OpenCV/geometry
isn't available, or a frame yields too few tracks, it emits nothing and the estimator
runs on wheels+IMU exactly as before. Monocular => no metric scale here (that comes
from the wheels); VO only supplies direction + a yaw cross-check.
"""

from __future__ import annotations

import logging
import os

import numpy as np

log = logging.getLogger("vo")


class VisualOdometry:
    def __init__(self, hfov_deg: float = 90.0, out_w: int = 320, out_h: int = 200,
                 min_features: int = 40, yaw_flow_sign: float = 1.0):
        self._hfov = float(hfov_deg)
        self._ow, self._oh = int(out_w), int(out_h)
        self._min_features = int(min_features)
        self._yaw_sign = float(yaw_flow_sign)
        self._map = None                 # cached equirect->pinhole remap
        self._map_key = None
        self._prev_gray = None
        self._prev_ts = 0
        self._f = self._ow / (2.0 * np.tan(np.radians(self._hfov) / 2.0))   # focal px
        self._cx, self._cy = self._ow / 2.0, self._oh / 2.0
        try:
            import cv2  # noqa: F401
            self._ok = True
        except Exception as e:
            log.warning(f"[vo] OpenCV unavailable ({e}); visual odometry disabled")
            self._ok = False

    def _pinhole(self, equirect_bgr):
        import cv2
        import vat_periscope as psc
        h, w = equirect_bgr.shape[:2]
        key = (h, w)
        if key != self._map_key:
            vf = psc.vfov_from_hfov(self._hfov, self._ow, self._oh)
            self._mx, self._my = psc.build_remap(h, w, 0.0, 0.0, self._hfov, vf,
                                                 self._ow, self._oh)
            self._map_key = key
        rect = cv2.remap(equirect_bgr, self._mx, self._my, interpolation=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_WRAP)
        return cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)

    def process(self, equirect_bgr, ts_ns: int, gyro_wz: float, dt: float):
        """Return (dir_x, dir_y, yaw_rate, confidence) in the body horizontal plane
        (x fwd, y left), or None if unusable. Direction is de-rotated; magnitude is
        the wheel's job. Pure rotation -> confidence ~0."""
        if not self._ok or equirect_bgr is None or dt <= 1e-4:
            return None
        import cv2
        try:
            gray = self._pinhole(equirect_bgr)
        except Exception:
            return None
        prev, self._prev_gray = self._prev_gray, gray
        if prev is None or prev.shape != gray.shape:
            return None
        p0 = cv2.goodFeaturesToTrack(prev, maxCorners=200, qualityLevel=0.01,
                                     minDistance=8, blockSize=7)
        if p0 is None or len(p0) < self._min_features:
            return None
        p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, gray, p0, None,
                                             winSize=(21, 21), maxLevel=3)
        if p1 is None or st is None:
            return None
        good = st.reshape(-1) == 1
        if int(good.sum()) < self._min_features:
            return None
        a = p0.reshape(-1, 2)[good]
        b = p1.reshape(-1, 2)[good]
        flow = b - a                                   # (N,2) du,dv
        # DE-ROTATE: remove the uniform horizontal shift caused by yaw.
        du_yaw = self._yaw_sign * self._f * float(gyro_wz) * float(dt)
        res = flow.copy()
        res[:, 0] -= du_yaw
        # robust central tendency (median) of the residual flow
        med = np.median(res, axis=0)
        mean_du, mean_dv = float(med[0]), float(med[1])
        # radial divergence (expansion) about the image centre -> forward motion
        r = a - np.array([self._cx, self._cy])
        rn = np.linalg.norm(r, axis=1) + 1e-6
        radial = np.sum(res * (r / rn[:, None]), axis=1)
        divergence = float(np.median(radial))
        # Body-frame motion: forward ~ +divergence; lateral(+y left) ~ -mean_du
        # (camera moving right -> world flows left -> du<0 -> +y? sign tuned by VO_LAT_SIGN).
        fwd = divergence
        lat = -mean_du
        v = np.array([fwd, lat], dtype=np.float64)
        n = float(np.linalg.norm(v))
        if n < 1e-3:
            return None                                # (near-)pure rotation -> ignore
        v /= n
        # confidence: feature count + flow agreement (low residual scatter = trustworthy)
        scatter = float(np.median(np.linalg.norm(res - med, axis=1)))
        conf = float(np.clip(good.sum() / 200.0, 0.0, 1.0)) * float(np.clip(1.0 - scatter / 6.0, 0.0, 1.0))
        yaw_rate = -du_yaw / (self._f * max(dt, 1e-3)) if False else float(gyro_wz)
        return float(v[0]), float(v[1]), yaw_rate, conf


class VoRunner:
    """Glue for the camera process: throttle frames, pull the gyro yaw-rate, run VO,
    and publish a VoDelta. Fully OPTIONAL (VO_ENABLE=0 => inert). ``gyro_getter`` is a
    callable returning the body yaw-rate (rad/s)."""

    def __init__(self, z_session, robot_name: str, gyro_getter):
        import vat_protocol as proto
        self._proto = proto
        self._gyro = gyro_getter
        self.enabled = os.environ.get("VO_ENABLE", "0").strip().lower() in ("1", "true", "yes", "on")
        self._fps = float(os.environ.get("VO_FPS", "12"))
        self._min_conf = float(os.environ.get("VO_MIN_CONF", "0.3"))
        self._lat_sign = float(os.environ.get("VO_LAT_SIGN", "1"))
        self._last_ts = 0
        self._n = 0
        self._vo = None
        self._pub = None
        if self.enabled:
            try:
                self._vo = VisualOdometry(
                    hfov_deg=float(os.environ.get("VO_HFOV", "90")),
                    yaw_flow_sign=float(os.environ.get("VO_YAW_FLOW_SIGN", "1")))
                self._pub = z_session.declare_publisher(proto.keys(robot_name)["vo"])
                log.info(f"[vo] ENABLED fps={self._fps} hfov={os.environ.get('VO_HFOV','90')} "
                         f"min_conf={self._min_conf} -> '{proto.keys(robot_name)['vo']}'")
            except Exception as e:
                log.warning(f"[vo] init failed ({e}); disabled")
                self.enabled = False

    def on_frame(self, frame_bgr, ts_ns: int):
        if not self.enabled or self._pub is None:
            return
        if self._last_ts and (ts_ns - self._last_ts) < 1e9 / max(self._fps, 1.0):
            return
        dt = (ts_ns - self._last_ts) * 1e-9 if self._last_ts else 0.0
        self._last_ts = ts_ns
        if dt <= 0:
            return
        try:
            wz = float(self._gyro())
        except Exception:
            wz = 0.0
        try:
            r = self._vo.process(frame_bgr, ts_ns, wz, dt)
        except Exception:
            return
        if r is None:
            return
        dx, dy, yaw, conf = r
        if conf < self._min_conf:
            return
        v = self._proto.VoDelta(timestamp_ns=ts_ns, dir_x=dx, dir_y=dy * self._lat_sign,
                                yaw_rate=yaw, confidence=conf)
        try:
            self._pub.put(self._proto.pack_vo(v), encoding=self._proto.ENC_VODO)
        except TypeError:
            self._pub.put(self._proto.pack_vo(v))
        except Exception:
            pass
        self._n += 1
