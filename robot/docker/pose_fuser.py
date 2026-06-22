"""
VAT — Pose Fuser  (PLACEHOLDER)
===============================
The robot — not the server — is authoritative for its global pose.  This
process realises the ``server (pose) → dog → server (router) → client`` path:

  * subscribes to the **VGGT camera-pose correction** the server sends DOWN
        server/prism/pose_correction           (slow, drift-free, laggy)
  * subscribes (via the bridge) to the robot's **fast onboard odometry**
        {robot}/rt/sportmodestate              (~50–500 Hz, drifts)
  * converts the camera pose to a **base** pose with :mod:`kinematics`
        T_world_base = T_world_camera ∘ inverse(T_base_camera)
  * fuses the slow correction with the fast odometry and publishes the
    **authoritative** pose UP, which the server's router relays to the client
        {robot}/prism/pose                     (pos + quat + lin/ang vel + ts)

⚠️  PLACEHOLDER FUSION
---------------------
This is **not** a real EKF.  It is a constant-velocity dead-reckoner anchored
by the VGGT correction — just enough to prove the data path and message
contract.  A production estimator (NumPy/`filterpy` EKF, or a `fuse` /
`robot_localization` ROS node) drops in by replacing :meth:`FuseState.predict`
and :meth:`FuseState.correct` while keeping the same inputs, outputs, Zenoh key
and wire format.  See ``docs/streaming_poc.md`` → *Pose & state estimation*.

Environment
-----------
  ROBOT_NAME, ZENOH_CONNECT, PUBLISH_HZ, CORRECTION_POS_GAIN,
  CORRECTION_ROT_GAIN, FIX_HOLD_S, plus the kinematics STICK_* vars.
"""

from __future__ import annotations

import os
import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import zenoh

import vat_protocol as proto
from vat_protocol import (
    quat_identity, quat_normalize, quat_rotate, quat_slerp, integrate_pose,
    PoseState, FIX_CORRECTED, FIX_DEADRECKON,
)
from kinematics import build_robot_model, RobotStateTracker, Transform

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="[%(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("pose-fuser")

ROBOT_NAME    = os.environ.get("ROBOT_NAME",    "go2")
ZENOH_CONNECT = os.environ.get("ZENOH_CONNECT", "tcp/127.0.0.1:7447")
PUBLISH_HZ    = float(os.environ.get("PUBLISH_HZ",          "50.0"))
POS_GAIN      = float(os.environ.get("CORRECTION_POS_GAIN", "0.5"))   # 0..1 blend
ROT_GAIN      = float(os.environ.get("CORRECTION_ROT_GAIN", "0.5"))   # 0..1 slerp
FIX_HOLD_S    = float(os.environ.get("FIX_HOLD_S",          "1.0"))

_KEYS = proto.keys(ROBOT_NAME)


# ─────────────────────────────────────────────────────────────────────────────
# Fusion state  (placeholder: constant-velocity dead-reckon + correction blend)
# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class FuseState:
    position: np.ndarray            # world frame
    orientation: np.ndarray         # world frame, xyzw
    seq: int = 0
    have_anchor: bool = False       # has a VGGT correction ever arrived?
    last_correction_ns: int = 0

    @staticmethod
    def initial() -> "FuseState":
        return FuseState(np.zeros(3), quat_identity())

    def predict(self, world_lin_vel: np.ndarray, body_ang_vel: np.ndarray,
                dt: float):
        """Dead-reckon forward by dt (the part a real EKF would do better)."""
        if dt <= 0:
            return
        self.position, self.orientation = integrate_pose(
            self.position, self.orientation, world_lin_vel, body_ang_vel, dt)

    def correct(self, base_world: Transform, now_ns: int):
        """Anchor the dead-reckoned pose to the VGGT-derived base pose."""
        if not self.have_anchor:
            # first fix: snap exactly
            self.position = base_world.translation.astype(np.float32)
            self.orientation = quat_normalize(base_world.rotation).astype(np.float32)
            self.have_anchor = True
        else:
            self.position = ((1 - POS_GAIN) * self.position
                             + POS_GAIN * base_world.translation).astype(np.float32)
            self.orientation = quat_slerp(self.orientation, base_world.rotation,
                                          ROT_GAIN).astype(np.float32)
        self.last_correction_ns = now_ns


# ─────────────────────────────────────────────────────────────────────────────
# Fuser node
# ─────────────────────────────────────────────────────────────────────────────


class PoseFuser:
    def __init__(self, z: zenoh.Session):
        self._z = z
        self._model = build_robot_model()
        self._state_tracker = RobotStateTracker(z, ROBOT_NAME)
        self._fuse = FuseState.initial()
        self._lock = threading.Lock()
        self._last_pub_ns = time.time_ns()
        self._corrections = 0

        self._pub = z.declare_publisher(
            _KEYS["pose"],
            congestion_control=zenoh.CongestionControl.DROP,   # realtime
            priority=zenoh.Priority.DATA_HIGH,
        )
        # liveliness so the client/system can detect the pose stream's presence
        try:
            self._live = z.liveliness().declare_token(_KEYS["live_pose"])
        except Exception:
            self._live = None

        z.declare_subscriber(_KEYS["pose_correction"], self._on_correction)
        log.info(f"[Fuser] correction←'{_KEYS['pose_correction']}'  "
                 f"pose→'{_KEYS['pose']}'  @ {PUBLISH_HZ}Hz")

    def _on_correction(self, sample):
        try:
            c = proto.unpack_pose_correction(bytes(sample.payload))
        except proto.ProtocolError as e:
            log.warning(f"[Fuser] bad correction: {e}")
            return
        # camera pose (world) → base pose (world) via kinematics
        cam_world = Transform.from_xyz_quat(c.position, c.quaternion)
        base_world = self._model.base_from_camera_world(cam_world)
        with self._lock:
            self._fuse.correct(base_world, time.time_ns())
            self._corrections += 1
        log.debug(f"[Fuser] correction v{c.map_version} → base "
                  f"{np.round(base_world.translation, 3)}")

    def _publish_once(self):
        now = time.time_ns()
        body = self._state_tracker.get()
        with self._lock:
            dt = (now - self._last_pub_ns) * 1e-9
            self._last_pub_ns = now

            # body-frame linear velocity → world frame using current orientation
            world_lin_vel = quat_rotate(self._fuse.orientation, body.linear_velocity)
            body_ang_vel = body.angular_velocity

            self._fuse.predict(world_lin_vel, body_ang_vel, dt)
            self._fuse.seq += 1

            corrected = (now - self._fuse.last_correction_ns) < FIX_HOLD_S * 1e9
            fix = FIX_CORRECTED if (self._fuse.have_anchor and corrected) else FIX_DEADRECKON

            pose = PoseState(
                timestamp_ns=now,
                seq=self._fuse.seq,
                position=self._fuse.position.copy(),
                quaternion=self._fuse.orientation.copy(),
                linear_velocity=world_lin_vel.astype(np.float32),
                angular_velocity=np.asarray(body_ang_vel, dtype=np.float32),
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
                s = self._fuse
                log.info(f"[Fuser] seq={s.seq} anchored={s.have_anchor} "
                         f"corrections={self._corrections} pos={np.round(s.position,2)}")
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
