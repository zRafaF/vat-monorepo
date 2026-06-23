"""
VAT — State Estimator  (dead-reckoning, placeholder for a full EKF)
==================================================================
We own the robot's global pose, and on the Go2-W we have to dead-reckon it
ourselves: ``robot_odom`` isn't published, and the low-frequency
``SportModeState.velocity`` is zeroed by the ``lf/`` relay. So this estimator
derives motion from the always-on ``/lowstate`` (500 Hz):

  * **attitude** from the IMU quaternion (gravity-referenced, drift-free in
    roll/pitch), smoothed between samples by integrating the gyro — a
    complementary filter;
  * **body velocity** from **wheel odometry**: the four wheel motors' angular
    velocity ``dq`` × wheel radius give the ground speed; the wheels are
    non-holonomic so lateral velocity ≈ 0;
  * **position** by integrating that body velocity through the current attitude
    into the world frame.

With no global correction the position drifts — that is the expected, useful
signal. When the cloud's VGGT pose lands, :meth:`correct` re-anchors the pose.

This is deliberately a **placeholder estimator**, not a full filter. The
interface — :meth:`predict` (high-rate motion) + :meth:`correct` (low-rate
global fix) + :meth:`state` — is exactly the shape a real EKF or a factor-graph
smoother needs, so the upgrade path is to swap the body of these two methods
while the fuser, wire format and Zenoh keys stay put.

Pure NumPy: no ROS, no Zenoh — so it unit-tests in isolation.
"""

from __future__ import annotations

import numpy as np

from vat_protocol import (
    quat_identity, quat_normalize, quat_rotate, quat_slerp, integrate_pose,
)
from kinematics import Transform


class WheelInertialEstimator:
    """Wheel-odometry + IMU dead-reckoner with a VGGT re-anchor.

    Parameters
    ----------
    att_gain : float
        Complementary-filter gain pulling the gyro-integrated attitude toward the
        IMU's absolute attitude each step (0 = gyro only, 1 = IMU only).
    pos_gain, rot_gain : float
        How hard a VGGT correction pulls position / orientation (0..1).
    """

    def __init__(self, att_gain: float = 0.08,
                 pos_gain: float = 0.5, rot_gain: float = 0.5):
        self.position = np.zeros(3, dtype=np.float64)
        self.orientation = quat_identity()          # xyzw, world
        self.world_vel = np.zeros(3, dtype=np.float64)
        self.have_vggt = False
        self.last_correction_ns = 0
        self._att_gain = att_gain
        self._pos_gain = pos_gain
        self._rot_gain = rot_gain
        self._inited = False

    # -- high-rate prediction -------------------------------------------------
    def predict(self, imu_quat: np.ndarray, gyro: np.ndarray,
                body_vx: float, valid: bool, dt: float,
                body_vy: float = 0.0):
        """Advance the state by ``dt`` from one /lowstate sample.

        ``imu_quat`` (xyzw) and ``gyro`` (rad/s, body) are the IMU attitude and
        rate; ``body_vx``/``body_vy`` are the wheel-odometry body velocity (m/s).
        Does nothing if ``dt<=0`` or the sample is invalid."""
        if dt <= 0 or not valid:
            return
        imu_quat = quat_normalize(imu_quat)
        if not self._inited:
            self.orientation = imu_quat
            self._inited = True
        # attitude: gyro-integrate (smooth) then blend toward the IMU absolute
        _, q_gyro = integrate_pose(self.position, self.orientation,
                                   np.zeros(3), gyro, dt)
        self.orientation = quat_normalize(
            quat_slerp(q_gyro, imu_quat, self._att_gain))
        # body velocity → world; wheels are non-holonomic so vy defaults to 0
        v_body = np.array([body_vx, body_vy, 0.0], dtype=np.float64)
        self.world_vel = quat_rotate(self.orientation, v_body)
        self.position = self.position + self.world_vel * dt

    # -- low-rate global correction ------------------------------------------
    def correct(self, base_world: Transform, now_ns: int):
        """Re-anchor to the VGGT-derived base pose in the map frame."""
        if not self.have_vggt:
            self.position = np.asarray(base_world.translation, dtype=np.float64).copy()
            self.orientation = quat_normalize(base_world.rotation)
            self.have_vggt = True
        else:
            self.position = ((1 - self._pos_gain) * self.position
                             + self._pos_gain * base_world.translation)
            self.orientation = quat_normalize(
                quat_slerp(self.orientation, base_world.rotation, self._rot_gain))
        self.last_correction_ns = now_ns

    # -- accessor -------------------------------------------------------------
    def state(self):
        """Returns (position(3), orientation_xyzw(4), world_velocity(3))."""
        return (self.position.copy(), self.orientation.copy(), self.world_vel.copy())


# ─────────────────────────────────────────────────────────────────────────────
# Self-test:  python estimator.py
# ─────────────────────────────────────────────────────────────────────────────

def _selftest():
    from vat_protocol import quat_from_rotvec

    # 1) Drive straight forward (level, no rotation) → position grows in +x.
    est = WheelInertialEstimator()
    for _ in range(100):                      # 2 s at 50 Hz, 0.3 m/s
        est.predict(quat_identity(), np.zeros(3), 0.3, True, 0.02)
    pos, qz, vel = est.state()
    assert abs(pos[0] - 0.6) < 1e-3 and abs(pos[1]) < 1e-6, pos
    assert abs(vel[0] - 0.3) < 1e-6, vel

    # 2) Facing +y (yaw 90°) and driving forward → motion goes +y.
    est = WheelInertialEstimator(att_gain=1.0)   # trust IMU fully for the test
    yaw90 = quat_from_rotvec([0, 0, np.pi / 2])
    for _ in range(100):
        est.predict(yaw90, np.zeros(3), 0.3, True, 0.02)
    pos, _, _ = est.state()
    assert abs(pos[1] - 0.6) < 1e-3 and abs(pos[0]) < 1e-3, pos

    # 3) VGGT correction snaps then holds the anchor.
    est = WheelInertialEstimator()
    est.predict(quat_identity(), np.zeros(3), 0.3, True, 0.02)
    target = Transform.from_xyz_quat([5.0, 2.0, 0.0], quat_identity())
    est.correct(target, now_ns=1)
    pos, _, _ = est.state()
    assert np.allclose(pos, [5.0, 2.0, 0.0], atol=1e-6), pos
    assert est.have_vggt

    print("estimator self-test OK  "
          "(straight→x=0.6, yaw→y=0.6, VGGT snap OK)")


if __name__ == "__main__":
    _selftest()
