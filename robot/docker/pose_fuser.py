"""
VAT — Pose Fuser  (PLACEHOLDER)
===============================
The robot — not the server — is authoritative for its global pose.  This
process realises the ``server (pose) → dog → server (router) → client`` path:

  * subscribes to the **VGGT camera-pose correction** the server sends DOWN
        server/prism/pose_correction           (slow, drift-free, laggy)
  * subscribes (via the bridge) to the robot's **onboard odometry**
        {robot}/rt/utlidar/robot_odom          (nav_msgs/Odometry, always-on)
  * converts the camera pose to a **base** pose with :mod:`kinematics`
        T_world_base = T_world_camera ∘ inverse(T_base_camera)
  * publishes the **authoritative** pose UP, which the server's router relays
    to the client
        {robot}/prism/pose                     (pos + quat + lin/ang vel + ts)

⚠️  PLACEHOLDER FUSION
---------------------
This is **not** a real EKF.  It is the robot's onboard odometry re-expressed in
the map frame: the published pose is ``T_world_odom ∘ T_odom_base``, where the
**anchor** ``T_world_odom`` is set so the pose starts at the origin and is
re-anchored each time a VGGT correction lands.  Between corrections the pose is
pure odometry — it drifts, which is exactly the Stage-2.5 signal.  A production
estimator (NumPy/`filterpy` EKF) drops in by replacing the anchor snap with a
proper fusion of odom + correction, keeping the same inputs, outputs, Zenoh key
and wire format.

Why odometry and not ``SportModeState.velocity``?  The low-frequency
``lf/sportmodestate`` relay zeros the ``velocity`` field (just as it zeros
``foot_position_body``), so integrating it never moves.  ``robot_odom`` carries a
real, already-integrated position.

Environment
-----------
  ROBOT_NAME, ZENOH_CONNECT, PUBLISH_HZ, ODOM_TOPIC, FIX_HOLD_S,
  plus the kinematics STICK_* vars (for the camera→base correction transform).
"""

from __future__ import annotations

import os
import logging
import threading
import time

import numpy as np
import zenoh

import vat_protocol as proto
from vat_protocol import (
    quat_rotate, PoseState, FIX_CORRECTED, FIX_DEADRECKON,
)
from kinematics import build_robot_model, OdometryTracker, Transform

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="[%(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("pose-fuser")

ROBOT_NAME    = os.environ.get("ROBOT_NAME",    "go2")
ZENOH_CONNECT = os.environ.get("ZENOH_CONNECT", "tcp/127.0.0.1:7447")
PUBLISH_HZ    = float(os.environ.get("PUBLISH_HZ",  "50.0"))
ODOM_TOPIC    = os.environ.get("ODOM_TOPIC",        "utlidar/robot_odom")
FIX_HOLD_S    = float(os.environ.get("FIX_HOLD_S",  "1.0"))

_KEYS = proto.keys(ROBOT_NAME)


# ─────────────────────────────────────────────────────────────────────────────
# Fuser node  (odometry → map frame, re-anchored by VGGT corrections)
# ─────────────────────────────────────────────────────────────────────────────


class PoseFuser:
    def __init__(self, z: zenoh.Session):
        self._z = z
        self._model = build_robot_model()
        self._odom = OdometryTracker(z, ROBOT_NAME, ODOM_TOPIC)
        self._lock = threading.Lock()

        # T_world_odom — maps the robot's odom frame into the map frame.
        # None until the first odom sample (then set to zero the start to origin);
        # overwritten whenever a VGGT correction re-anchors the pose.
        self._anchor: Transform | None = None
        self._have_vggt = False
        self._last_correction_ns = 0
        self._corrections = 0
        self._seq = 0
        self._last_pos = np.zeros(3)

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
        log.info(f"[Fuser] odom←'{ROBOT_NAME}/rt/{ODOM_TOPIC}'  "
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
        odom_pose, _, _, valid = self._odom.get()
        if not valid:
            return
        # Re-anchor so the published pose matches VGGT now and keeps tracking odom:
        #   T_world_odom = T_world_base ∘ inverse(T_odom_base)
        with self._lock:
            self._anchor = base_world.compose(odom_pose.inverse())
            self._have_vggt = True
            self._last_correction_ns = time.time_ns()
            self._corrections += 1
        log.debug(f"[Fuser] correction v{c.map_version} → base "
                  f"{np.round(base_world.translation, 3)}")

    def _publish_once(self):
        now = time.time_ns()
        odom_pose, lin_body, ang_body, valid = self._odom.get()
        if not valid:
            return    # no odometry yet — nothing to publish

        with self._lock:
            if self._anchor is None:
                # first sample: zero the odom origin so the pose starts at [0,0,0]
                self._anchor = odom_pose.inverse()
            world = self._anchor.compose(odom_pose)        # T_world_base
            self._seq += 1
            corrected = (now - self._last_correction_ns) < FIX_HOLD_S * 1e9
            fix = FIX_CORRECTED if (self._have_vggt and corrected) else FIX_DEADRECKON
            pos = world.translation.astype(np.float32)
            quat = world.rotation.astype(np.float32)
            self._last_pos = pos

        world_lin_vel = quat_rotate(quat, lin_body).astype(np.float32)
        pose = PoseState(
            timestamp_ns=now,
            seq=self._seq,
            position=pos,
            quaternion=quat,
            linear_velocity=world_lin_vel,
            angular_velocity=np.asarray(ang_body, dtype=np.float32),
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
                anchored = self._anchor is not None
                log.info(f"[Fuser] seq={self._seq} odom_anchored={anchored} "
                         f"vggt={self._have_vggt} corrections={self._corrections} "
                         f"pos={np.round(self._last_pos, 2)}")
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
