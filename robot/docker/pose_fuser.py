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
from kinematics import build_robot_model, LowStateTracker, RobotStateTracker, Transform
from estimator import WheelInertialEstimator

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
POS_GAIN      = float(os.environ.get("CORRECTION_POS_GAIN", "0.5"))
ROT_GAIN      = float(os.environ.get("CORRECTION_ROT_GAIN", "0.5"))
FIX_HOLD_S    = float(os.environ.get("FIX_HOLD_S",          "1.0"))

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
        self._est = WheelInertialEstimator(att_gain=ATT_GAIN,
                                           pos_gain=POS_GAIN, rot_gain=ROT_GAIN)
        self._lock = threading.Lock()
        self._last_pub_ns = time.time_ns()
        self._corrections = 0
        self._seq = 0

        self._pub = z.declare_publisher(
            _KEYS["pose"],
            congestion_control=zenoh.CongestionControl.DROP,   # realtime
            priority=zenoh.Priority.DATA_HIGH,
        )
        try:
            self._live = z.liveliness().declare_token(_KEYS["live_pose"])
        except Exception:
            self._live = None

        z.declare_subscriber(_KEYS["pose_correction"], self._on_correction)
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
        # c.timestamp_ns is the keyframe CAPTURE time (robot clock) — the fix is
        # stale by (now - capture). Hand both to the estimator so it re-anchors at
        # the right point in its history instead of teleporting the live pose.
        with self._lock:
            self._est.correct(base_world, capture_ts_ns=c.timestamp_ns, now_ns=now)
            self._corrections += 1
        log.debug(f"[Fuser] correction v{c.map_version} lag="
                  f"{(now - c.timestamp_ns) * 1e-9:.2f}s → base "
                  f"{np.round(base_world.translation, 3)}")

    def _publish_once(self):
        now = time.time_ns()
        imu_quat, gyro, body_vx, valid = self._low.get_odom()
        body = self._body.get()
        with self._lock:
            dt = (now - self._last_pub_ns) * 1e-9
            self._last_pub_ns = now
            self._est.predict(imu_quat, gyro, body_vx, valid, dt, now_ns=now)
            self._seq += 1
            pos, quat, world_vel = self._est.state()
            # Vertical comes from body height (map floor = Z 0), so the avatar
            # rises/lowers with the dog's stance instead of staying pinned.
            pos[2] = body.body_height
            corrected = (now - self._est.last_correction_ns) < FIX_HOLD_S * 1e9
            fix = FIX_CORRECTED if (self._est.have_vggt and corrected) else FIX_DEADRECKON

        pose = PoseState(
            timestamp_ns=now,
            seq=self._seq,
            position=pos.astype(np.float32),
            quaternion=quat.astype(np.float32),
            linear_velocity=world_vel.astype(np.float32),
            angular_velocity=np.asarray(gyro, dtype=np.float32),
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
                pos, _, vel = self._est.state()
                _, _, body_vx, valid = self._low.get_odom()
                log.info(f"[Fuser] seq={self._seq} odom_valid={valid} "
                         f"vggt={self._est.have_vggt} corrections={self._corrections} "
                         f"body_vx={body_vx:+.2f}m/s pos={np.round(pos, 2)}")
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
