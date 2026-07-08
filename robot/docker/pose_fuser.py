"""
VAT — Pose Fuser  (PLACEHOLDER)
===============================
The robot — not the server — is authoritative for its global pose.  This
process realises the ``server (pose) → dog → server (router) → client`` path:

  * subscribes to the **VGGT camera-pose correction** the server sends DOWN
        server/prism/pose_correction           (slow, drift-free, laggy)
  * runs our own **dead-reckoning estimator** over the always-on ``/lowstate``
    (IMU attitude + wheel odometry — see :mod:`estimator`), because the Go2-W
    publishes no ``robot_odom`` and the ``lf/sportmodestate`` velocity is zeroed
  * converts the camera correction to a **base** pose with :mod:`kinematics`
        T_world_base = T_world_camera ∘ inverse(T_base_camera)
  * publishes the **authoritative** pose UP, which the server's router relays
    to the client
        {robot}/prism/pose                     (pos + quat + lin/ang vel + ts)

⚠️  PLACEHOLDER FUSION
---------------------
The estimator is a wheel-odometry + IMU dead-reckoner, re-anchored by the VGGT
correction.  Between corrections the pose is pure odometry and drifts — the
expected Stage-2.5 signal.  Swap :class:`estimator.WheelInertialEstimator` for a
full EKF / factor-graph later behind the same interface; the fuser, wire format
and Zenoh keys are unchanged.

Environment
-----------
  ROBOT_NAME, ZENOH_CONNECT, PUBLISH_HZ, FIX_HOLD_S, ATTITUDE_GAIN,
  CORRECTION_POS_GAIN, CORRECTION_ROT_GAIN, WHEEL_RADIUS (kinematics),
  plus the STICK_* vars (for the camera→base correction transform).
"""

from __future__ import annotations

import os
import logging
import threading
import time

import numpy as np
import zenoh

import vat_protocol as proto
from vat_protocol import PoseState, FIX_CORRECTED, FIX_DEADRECKON
from kinematics import (build_robot_model, LowStateTracker, RobotStateTracker,
                        Transform, base_height_above_ground, WHEEL_RADIUS)
# The estimator backend is chosen by POSE_BACKEND via the make_estimator factory:
#   * gtsam -> IMU-preintegration fixed-lag smoother that FUSES the accelerometer +
#     gyro + wheel-odometry velocity + the delayed VGGT correction (the default);
#   * graph / blend -> the pure-NumPy wheel+IMU estimator, used automatically as a
#     fallback if the gtsam wheel isn't in the image so the pose path never stops.
from estimator import make_estimator

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="[%(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("pose-fuser")

ROBOT_NAME    = os.environ.get("ROBOT_NAME",    "go2")
ZENOH_CONNECT = os.environ.get("ZENOH_CONNECT", "tcp/127.0.0.1:7447")
PUBLISH_HZ    = float(os.environ.get("PUBLISH_HZ",          "50.0"))
ATT_GAIN      = float(os.environ.get("ATTITUDE_GAIN",       "0.08"))
# Correction gains: fraction of each VGGT anchor folded into the correction TARGET.
# 1.0 = converge the avatar ALL the way to VGGT (no residual "half-way" offset). The
# ESKF no longer steps the *live* pose by this fraction — it slews smoothly to the
# target over CORRECTION_SLEW_TAU seconds — so full gain is smooth, not a teleport.
# The server-side deadband/outlier gate already suppresses still-scene jitter. Lower
# these below 1.0 only to add extra low-pass smoothing of VGGT noise at the target.
POS_GAIN      = float(os.environ.get("CORRECTION_POS_GAIN", "1.0"))
ROT_GAIN      = float(os.environ.get("CORRECTION_ROT_GAIN", "1.0"))
FIX_HOLD_S    = float(os.environ.get("FIX_HOLD_S",          "1.0"))
# Vertical channel: derive base height above the floor from LEG-FK stance (tracks
# stand<->prone + body tilt) instead of the flat SportModeState.body_height (often a
# fallback constant). Fixes the "prone robot flies" artifact + gives a true height.
VERTICAL_FROM_LEGS = os.environ.get("VERTICAL_FROM_LEGS", "1").strip().lower() in ("1","true","yes","on")
# Fine-tune the published base Z (m, +up) if wheels still float/sink after the leg-FK fix.
VERTICAL_Z_OFFSET = float(os.environ.get("VERTICAL_Z_OFFSET", "0.0"))
# Strafe: take lateral body velocity from the Go2 onboard estimate (SportModeState
# velocity) when available. Wheels give only forward speed, so without this a
# rotate+strafe reads as pure rotation and the avatar swings. Degrades to the
# non-holonomic prior (vy=0) when the velocity field is zeroed (lf/ topic).
STRAFE_FROM_SPORT = os.environ.get("STRAFE_FROM_SPORT", "1").strip().lower() in ("1","true","yes","on")
# Visual odometry (optional): use the VO body-motion DIRECTION to recover strafe from
# the wheel forward speed (body_vy = body_vx * dir_y/dir_x). Inert until VO frames
# arrive (VO_ENABLE=1 in the camera process), so it is safe to leave on.
VO_FUSE = os.environ.get("VO_FUSE", "1").strip().lower() in ("1","true","yes","on")
VO_MIN_CONF = float(os.environ.get("VO_MIN_CONF", "0.3"))
VO_MAX_AGE_S = float(os.environ.get("VO_MAX_AGE_S", "0.5"))

_KEYS = proto.keys(ROBOT_NAME)


# ─────────────────────────────────────────────────────────────────────────────
# Fuser node
# ─────────────────────────────────────────────────────────────────────────────


class PoseFuser:
    def __init__(self, z: zenoh.Session):
        self._z = z
        self._model = build_robot_model()
        self._low = LowStateTracker(z, ROBOT_NAME)            # IMU + wheel odom
        # Body height (stand↔prone) from SportModeState — the vertical channel the
        # planar wheel/IMU dead-reckoner can't provide. Drives the published Z.
        self._body = RobotStateTracker(
            z, ROBOT_NAME,
            fallback_body_height=float(os.environ.get("FALLBACK_BODY_HEIGHT", "0.30")))
        self._est, backend_note = make_estimator(att_gain=ATT_GAIN,
                                                 pos_gain=POS_GAIN, rot_gain=ROT_GAIN)
        self._backend_note = backend_note
        log.info(f"[Fuser] estimator backend: {backend_note}")
        self._lock = threading.Lock()
        self._last_pub_ns = time.time_ns()
        self._corrections = 0          # fixes handed to the estimator (received)
        self._last_corr_lag_s = float("nan")   # capture -> apply latency of the last fix
        self._seq = 0

        # express=True: flush each (tiny) pose sample immediately instead of
        # letting it wait up to the transport's adaptive-batching window
        # (transport/link/tx/queue/batching.time_limit, 1 ms by default) behind
        # bulk frame data. This is the latency lever for the pose "chug" during
        # camera bursts. Fall back gracefully if the installed zenoh binding
        # predates the `express` kwarg.
        _pose_qos = dict(
            congestion_control=zenoh.CongestionControl.DROP,   # realtime
            priority=zenoh.Priority.DATA_HIGH,
        )
        try:
            self._pub = z.declare_publisher(_KEYS["pose"], express=True, **_pose_qos)
        except TypeError:
            self._pub = z.declare_publisher(_KEYS["pose"], **_pose_qos)
        try:
            self._live = z.liveliness().declare_token(_KEYS["live_pose"])
        except Exception:
            self._live = None

        self._vo = None            # latest VoDelta (visual-odometry direction)
        self._vo_ts = 0
        z.declare_subscriber(_KEYS["pose_correction"], self._on_correction)
        if VO_FUSE:
            z.declare_subscriber(_KEYS["vo"], self._on_vo)
        log.info(f"[Fuser] odom←'{ROBOT_NAME}/rt/lowstate' (IMU+wheels)  "
                 f"correction←'{_KEYS['pose_correction']}'  "
                 f"pose→'{_KEYS['pose']}'  @ {PUBLISH_HZ}Hz")

    def _on_correction(self, sample):
        try:
            c = proto.unpack_pose_correction(bytes(sample.payload))
        except proto.ProtocolError as e:
            log.warning(f"[Fuser] bad correction: {e}")
            return
        # camera pose (map) → base pose (map) via kinematics
        cam_world = Transform.from_xyz_quat(c.position, c.quaternion)
        base_world = self._model.base_from_camera_world(cam_world)
        now = time.time_ns()
        # capture -> apply latency of THIS fix (the real VGGT correction lag: how old the
        # keyframe this fix describes is by the time we fold it in). Surfaced in the 10 s log.
        self._last_corr_lag_s = (now - c.timestamp_ns) * 1e-9
        # c.timestamp_ns is the keyframe CAPTURE time (robot clock) — the fix is
        # stale by (now - capture). Hand both to the estimator so it re-anchors at
        # the right point in its history instead of teleporting the live pose.
        with self._lock:
            try:
                self._est.correct(base_world, capture_ts_ns=c.timestamp_ns, now_ns=now)
                self._corrections += 1
            except Exception as e:
                # A failed fuse (e.g. a fix older than the smoother lag) must not stop
                # the high-rate predict/publish loop — drop this correction and continue.
                if not getattr(self, "_corr_err_logged", False):
                    log.warning(f"[Fuser] correction fuse failed ({e}); "
                                f"dropping this fix, pose path continues")
                    self._corr_err_logged = True
                return
        log.debug(f"[Fuser] correction v{c.map_version} lag="
                  f"{(now - c.timestamp_ns) * 1e-9:.2f}s → base "
                  f"{np.round(base_world.translation, 3)}")

    def _on_vo(self, sample):
        try:
            v = proto.unpack_vo(bytes(sample.payload))
        except proto.ProtocolError:
            return
        with self._lock:
            self._vo = v
            self._vo_ts = time.time_ns()

    def _publish_once(self):
        now = time.time_ns()
        # Richer odometry for the fusion backend: IMU attitude + gyro + ACCELEROMETER
        # (drives the IMU preintegration) + contact-selected wheel speed + wheel-diff
        # yaw rate. The NumPy backends ignore accel/body_wz, so this is safe for all.
        imu_quat, gyro, accel, body_vx, body_wz, n_contact, valid = self._low.get_imu_odom()
        body = self._body.get()
        legs, legs_valid = self._low.get()          # leg-FK points for the vertical channel
        body_vy = 0.0
        if STRAFE_FROM_SPORT and body.valid:
            _vy = float(body.linear_velocity[1])
            if np.isfinite(_vy) and abs(_vy) < 2.0:
                body_vy = _vy
        # VO strafe: if a recent, confident VO direction exists, recover the lateral
        # component from the wheel forward speed via the direction ratio (dir_y/dir_x).
        # Independent of leg slip, so it captures the strafe during rotate+forward.
        if VO_FUSE and self._vo is not None and self._vo.confidence >= VO_MIN_CONF \
                and (now - self._vo_ts) < VO_MAX_AGE_S * 1e9 and abs(self._vo.dir_x) > 0.25:
            _ratio = float(np.clip(self._vo.dir_y / self._vo.dir_x, -2.0, 2.0))
            body_vy = float(body_vx) * _ratio
        world_acc = np.zeros(3, dtype=np.float64)
        with self._lock:
            dt = (now - self._last_pub_ns) * 1e-9
            self._last_pub_ns = now
            self._est.predict(imu_quat, gyro, body_vx, valid, dt, body_vy=body_vy,
                              now_ns=now, accel=accel, body_wz=body_wz)
            self._seq += 1
            st = self._est.state()
            # gtsam backend returns (pos, quat, world_vel, world_accel); the NumPy
            # backends return the 3-tuple (no accel) -- support both.
            pos, quat, world_vel = st[0], st[1], st[2]
            if len(st) >= 4:
                world_acc = np.asarray(st[3], dtype=np.float64)
            # Vertical (map floor = Z 0). Prefer leg-FK stance height (drops when the
            # dog goes prone, accounts for tilt) over the flat SportModeState value.
            base_h = None
            if VERTICAL_FROM_LEGS and legs_valid and legs:
                base_h = base_height_above_ground(legs, imu_quat)
            if base_h is not None:
                # base_height_above_ground() is base-above-CONTACT (already +wheel radius).
                # The client lifts the avatar by GROUND_CLEARANCE (=wheel radius) so the
                # wheels rest on Z=0 — publish the hub-plane height (subtract the radius)
                # so we don't double-count and the wheels touch instead of flying.
                pos[2] = base_h - WHEEL_RADIUS + VERTICAL_Z_OFFSET
                self._z_src = 'legs'
            else:
                pos[2] = body.body_height
                self._z_src = 'body'
            corrected = (now - self._est.last_correction_ns) < FIX_HOLD_S * 1e9
            fix = FIX_CORRECTED if (self._est.have_vggt and corrected) else FIX_DEADRECKON

        # World-frame linear_acceleration from the IMU-fusion backend lets the client
        # extrapolate at constant ACCELERATION between samples (was constant-velocity when
        # left zero). Zero for the NumPy backends -> identical to prior behaviour.
        pose = PoseState(
            timestamp_ns=now,
            seq=self._seq,
            position=pos.astype(np.float32),
            quaternion=quat.astype(np.float32),
            linear_velocity=world_vel.astype(np.float32),
            angular_velocity=np.asarray(gyro, dtype=np.float32),
            linear_acceleration=world_acc.astype(np.float32),
            fix_quality=fix,
        )
        try:
            self._pub.put(proto.pack_pose(pose), encoding=proto.ENC_POSE)
        except TypeError:
            self._pub.put(proto.pack_pose(pose))

    def run(self):
        period = 1.0 / max(PUBLISH_HZ, 1.0)
        last_log = time.time()
        while True:
            t0 = time.time()
            try:
                self._publish_once()
            except Exception as e:
                log.warning(f"[Fuser] publish error: {e}")
            if time.time() - last_log > 10:
                st = self._est.state()          # 3-tuple (NumPy) or 4-tuple (gtsam: +accel)
                pos, vel = st[0], st[2]
                _, _, body_vx, valid = self._low.get_odom()
                stale = getattr(self._est, "n_stale", 0)
                # |vel| is the PUBLISHED world speed the client extrapolates from. If the
                # robot is visibly moving but |vel|~0, dead-reckoning has no motion (wheel
                # odom is forward-only and reads ~0 when walking/strafing/turning) -> the
                # avatar freezes between VGGT corrections. That is the signal to add a
                # velocity source that works off the wheels (e.g. leg/contact odometry).
                log.info(f"[Fuser] seq={self._seq} odom_valid={valid} backend={self._backend_note} "
                         f"vggt={self._est.have_vggt} corrections={self._corrections} "
                         f"corr_lag={self._last_corr_lag_s:.2f}s stale_dropped={stale} "
                         f"body_vx={body_vx:+.2f}m/s |vel|={float(np.linalg.norm(vel)):.2f}m/s "
                         f"z={pos[2]:+.2f}({getattr(self,'_z_src','?')}) pos={np.round(pos, 2)}")
                last_log = time.time()
            time.sleep(max(0.0, period - (time.time() - t0)))


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
    log.info("Connected.")
    fuser = PoseFuser(z)
    try:
        fuser.run()
    except KeyboardInterrupt:
        pass
    finally:
        z.close()


if __name__ == "__main__":
    main()
