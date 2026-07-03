"""
VAT - Error-State Kalman Filter pose estimator (pure NumPy, no GTSAM)
====================================================================
A self-contained wheel + IMU fusion for the robot's authoritative pose. Drop-in for
the older estimators behind the same ``predict / correct / state`` interface, so the
fuser, wire format and Zenoh keys are unchanged. No exotic dependencies -> it always
runs on the Jetson (the gtsam wheel never loaded there; this replaces it).

Why an ESKF fixes the "avatar freezes between VGGT updates" symptom
-------------------------------------------------------------------
The previous NumPy estimator set the published velocity to the *instantaneous* wheel
speed every tick, so a single noisy 0 reading -> published velocity 0 -> the client
(which dead-reckons ``p += v*dt``) freezes until the next VGGT correction jumps it.
Here **velocity is a filter STATE**: it is propagated by the IMU accelerometer and
only *nudged* toward the wheel reading (with tunable trust), so it stays continuous
through wheel dropouts and short gaps. That is what makes the between-correction
motion smooth and, in turn, hides the multi-second VGGT latency (we dead-reckon to
*now* and let VGGT only correct slow drift).

State (odometry frame, Z-up / REP-103)
--------------------------------------
Filtered:  p (position, 3), v (velocity, 3)   -- a 6-state linear KF.
External :  attitude q comes straight from the IMU quaternion (Unitree's fused,
            gravity-referenced attitude; roll/pitch are drift-free, yaw drift is
            corrected by the VGGT re-anchor). Not a filter state -> simpler + robust.
Accel bias: a slow scalar-per-axis estimate updated ONLY when detected stationary
            (ZUPT gate), so a constant accelerometer offset can't ramp the velocity.

Measurements
------------
* Wheel velocity (every tick): body-frame velocity ``[body_vx, body_vy, 0]`` with
  ANISOTROPIC noise -- tight on the forward (wheel) axis, looser on lateral so the
  IMU can express a strafe transient instead of it being pinned to zero, loose on
  vertical. Bounds velocity drift and supplies the motion signal.
* VGGT global pose (slow, delayed): handled OUT of the filter as a rigid
  ``world <- odom`` re-anchor matched at the fix's capture time (delayed-measurement
  correction), exactly like the proven estimator. The filter improves the odometry
  it re-anchors; the re-anchor keeps the published pose globally consistent without
  teleporting.

Pure NumPy: no ROS, no Zenoh, no GTSAM -> unit-tests in isolation (see __main__).
"""

from __future__ import annotations

import os
from collections import deque

import numpy as np

from vat_protocol import (
    quat_identity, quat_normalize, quat_rotate, quat_slerp, quat_mul, quat_conj,
)
from kinematics import Transform

_G = 9.81


def _quat_to_R(q) -> np.ndarray:
    """xyzw quaternion -> 3x3 rotation matrix (world <- body)."""
    x, y, z, w = np.asarray(q, dtype=np.float64) / (np.linalg.norm(q) + 1e-12)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


class ESKFEstimator:
    """Wheel + IMU error-state KF with a delayed VGGT re-anchor. Interface matches
    ``WheelInertialEstimator``: ``predict`` (high rate) / ``correct`` (VGGT) / ``state``."""

    def __init__(self, att_gain: float = 0.08, pos_gain: float = 0.7,
                 rot_gain: float = 0.7, history_s: float = 15.0, **_ignored):
        # --- filtered odometry state (odom frame) ---
        self._p = np.zeros(3, dtype=np.float64)
        self._v = np.zeros(3, dtype=np.float64)
        self._P = np.diag([1e-4, 1e-4, 1e-4, 1.0, 1.0, 1.0]).astype(np.float64)
        self._q = quat_identity()                 # attitude from the IMU
        self._a_world = np.zeros(3, dtype=np.float64)   # last world accel (published)
        self._a_bias = np.zeros(3, dtype=np.float64)    # slow accel bias (ZUPT-learned)

        # --- world <- odom correction (from VGGT), applied in state() ---
        self._corr_R = quat_identity()
        self._corr_p = np.zeros(3, dtype=np.float64)

        # --- time-indexed odometry history for delayed VGGT matching ---
        self._history: deque = deque()            # (t_ns, p, q)
        self._history_ns = int(history_s * 1e9)
        self._last_t_ns = 0
        self._inited = False

        # --- tunables (env) ---
        self._acc_sigma = float(os.environ.get("ESKF_ACCEL_SIGMA", "0.5"))   # m/s^2 process
        self._wheel_sigma = float(os.environ.get("ESKF_WHEEL_SIGMA", "0.05"))  # m/s fwd meas
        self._lat_sigma = float(os.environ.get("ESKF_LAT_SIGMA", "0.10"))   # m/s lateral (soft)
        self._vert_sigma = float(os.environ.get("ESKF_VERT_SIGMA", "0.10"))  # m/s vertical
        self._pos_gain = pos_gain
        self._rot_gain = rot_gain
        self._zupt_vx = float(os.environ.get("ESKF_ZUPT_VX", "0.03"))       # m/s
        self._zupt_gyro = float(os.environ.get("ESKF_ZUPT_GYRO", "0.05"))   # rad/s
        self._bias_beta = float(os.environ.get("ESKF_BIAS_BETA", "0.01"))   # bias EMA rate

        self.have_vggt = False
        self.last_correction_ns = 0
        self.n_stale = 0                          # kept for log compatibility
        self.backend_note = "eskf (numpy wheel+IMU, delayed-VGGT re-anchor)"

    # -- high-rate prediction + wheel update ----------------------------------
    def predict(self, imu_quat, gyro, body_vx, valid, dt,
                body_vy: float = 0.0, now_ns: int | None = None,
                accel=None, body_wz: float | None = None):
        """One /lowstate sample. ``imu_quat`` (xyzw) is the attitude; ``accel`` the
        IMU specific force (body, m/s^2); ``gyro`` the body rate; ``body_vx``/``body_vy``
        the wheel-odometry body velocity. ``now_ns`` stamps the history (robot clock)."""
        if dt <= 0 or not valid:
            return
        imu_quat = np.asarray(imu_quat, dtype=np.float64).reshape(4)
        gyro = np.asarray(gyro, dtype=np.float64).reshape(3)
        f = (np.asarray(accel, dtype=np.float64).reshape(3) if accel is not None
             else np.array([0.0, 0.0, _G]))
        # guard placeholder/garbage samples
        if not (np.all(np.isfinite(imu_quat)) and np.all(np.isfinite(gyro))
                and np.isfinite(body_vx) and np.all(np.isfinite(f))):
            return
        an = float(np.linalg.norm(f))
        if an < 3.0 or an > 40.0:                 # implausible specific force -> rest reaction
            f = np.array([0.0, 0.0, _G])
        q = quat_normalize(imu_quat)
        self._q = q
        R = _quat_to_R(q)

        # world linear acceleration (gravity removed), minus the slow bias estimate
        a_world = R @ f + np.array([0.0, 0.0, -_G]) - self._a_bias
        self._a_world = a_world

        # ZUPT-gated accel-bias learning: when stationary the true world accel is 0,
        # so the residual IS the bias -> pull the estimate toward it (kills drift).
        stationary = (abs(float(body_vx)) < self._zupt_vx
                      and float(np.linalg.norm(gyro)) < self._zupt_gyro)
        if stationary:
            self._a_bias = self._a_bias + self._bias_beta * a_world

        self._last_t_ns = int(now_ns) if now_ns is not None \
            else self._last_t_ns + int(dt * 1e9)
        if not self._inited:
            self._inited = True

        # --- KF predict (constant-accel model, 6-state [p, v]) ---
        self._p = self._p + self._v * dt + 0.5 * a_world * dt * dt
        self._v = self._v + a_world * dt
        F = np.eye(6)
        F[0:3, 3:6] = np.eye(3) * dt
        qpp = (0.5 * self._acc_sigma * dt * dt) ** 2
        qvv = (self._acc_sigma * dt) ** 2
        Q = np.diag([qpp, qpp, qpp, qvv, qvv, qvv])
        self._P = F @ self._P @ F.T + Q

        # --- wheel velocity measurement (body frame, anisotropic) ---
        z = np.array([float(body_vx), float(body_vy), 0.0], dtype=np.float64)
        H = np.zeros((3, 6))
        H[:, 3:6] = R.T                            # body vel = R^T v
        y = z - R.T @ self._v
        Rm = np.diag([self._wheel_sigma ** 2, self._lat_sigma ** 2, self._vert_sigma ** 2])
        S = H @ self._P @ H.T + Rm
        try:
            K = self._P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = None
        if K is not None:
            dx = K @ y
            self._p = self._p + dx[0:3]
            self._v = self._v + dx[3:6]
            self._P = (np.eye(6) - K @ H) @ self._P
            self._P = 0.5 * (self._P + self._P.T)  # keep symmetric

        # --- record history on the robot clock (for delayed VGGT matching) ---
        self._history.append((self._last_t_ns, self._p.copy(), self._q.copy()))
        while len(self._history) > 2 and \
                (self._last_t_ns - self._history[0][0]) > self._history_ns:
            self._history.popleft()

    def _odom_at(self, t_ns: int):
        """Interpolate the odometry (p, q) at a past time from the history buffer."""
        h = self._history
        if not h:
            return self._p.copy(), self._q.copy()
        if t_ns <= h[0][0]:
            return h[0][1].copy(), h[0][2].copy()
        if t_ns >= h[-1][0]:
            return h[-1][1].copy(), h[-1][2].copy()
        for i in range(len(h) - 1):
            t0, p0, q0 = h[i]
            t1, p1, q1 = h[i + 1]
            if t0 <= t_ns <= t1:
                a = (t_ns - t0) / max(t1 - t0, 1)
                return ((1.0 - a) * p0 + a * p1), quat_slerp(q0, q1, a)
        return h[-1][1].copy(), h[-1][2].copy()

    # -- low-rate, delayed global correction ----------------------------------
    def correct(self, base_world: Transform, capture_ts_ns: int | None = None,
                now_ns: int | None = None):
        """Re-anchor on a VGGT base pose captured at ``capture_ts_ns``: compute the
        rigid world<-odom offset that makes odom(t_c) == VGGT(t_c), then blend it into
        the running correction. The published pose is ``T_corr o odom(now)`` -- so all
        motion since the (stale) keyframe is preserved; no teleport."""
        now_ns = now_ns if now_ns is not None else (capture_ts_ns or self._last_t_ns)
        self.last_correction_ns = now_ns
        if capture_ts_ns is None:
            op, oq = self._p.copy(), self._q.copy()
        else:
            op, oq = self._odom_at(int(capture_ts_ns))

        R_v = quat_normalize(base_world.rotation)
        p_v = np.asarray(base_world.translation, dtype=np.float64).reshape(3)
        R_corr_new = quat_normalize(quat_mul(R_v, quat_conj(quat_normalize(oq))))
        p_corr_new = p_v - quat_rotate(R_corr_new, op)

        if not self.have_vggt:
            self._corr_R = R_corr_new
            self._corr_p = p_corr_new
            self.have_vggt = True
        else:
            self._corr_p = (1.0 - self._pos_gain) * self._corr_p \
                + self._pos_gain * p_corr_new
            self._corr_R = quat_normalize(
                quat_slerp(self._corr_R, R_corr_new, self._rot_gain))

    # -- accessor -------------------------------------------------------------
    def state(self):
        """(position(3), orientation_xyzw(4), world_velocity(3), world_accel(3)) -- the
        correction transform applied to the live filtered odometry."""
        pos = quat_rotate(self._corr_R, self._p) + self._corr_p
        quat = quat_normalize(quat_mul(self._corr_R, self._q))
        vel = quat_rotate(self._corr_R, self._v)
        acc = quat_rotate(self._corr_R, self._a_world)
        return pos, quat, vel, acc

    @property
    def position(self):
        return self.state()[0]

    @property
    def orientation(self):
        return self.state()[1]

    @property
    def world_vel(self):
        return self.state()[2]


# ─────────────────────────────────────────────────────────────────────────────
# Self-test:  python eskf_estimator.py
# ─────────────────────────────────────────────────────────────────────────────
def _selftest():
    G = _G
    rest = np.array([0.0, 0.0, G])            # specific force at rest (level)

    # 1) Rest: velocity and position stay ~0.
    est = ESKFEstimator()
    t = 0
    for _ in range(150):                      # 5 s @ 30 Hz
        t += 33_000_000
        est.predict(quat_identity(), np.zeros(3), 0.0, True, 1 / 30, now_ns=t, accel=rest)
    p, _, v, _ = est.state()
    assert np.linalg.norm(p) < 0.05 and np.linalg.norm(v) < 0.05, (p, v)

    # 2) Constant-velocity forward via wheels (accel = gravity only) -> v -> ~0.3, p grows.
    est = ESKFEstimator()
    t = 0
    for _ in range(150):
        t += 33_000_000
        est.predict(quat_identity(), np.zeros(3), 0.3, True, 1 / 30, now_ns=t, accel=rest)
    p, _, v, _ = est.state()
    assert abs(v[0] - 0.3) < 0.05, v
    assert 1.0 < p[0] < 1.6 and abs(p[1]) < 0.05, p     # ~0.3 m/s * ~4.5 s

    # 3) FREEZE-FIX: a one-sample wheel dropout must NOT collapse the velocity.
    est = ESKFEstimator()
    t = 0
    for _ in range(90):
        t += 33_000_000
        est.predict(quat_identity(), np.zeros(3), 0.3, True, 1 / 30, now_ns=t, accel=rest)
    v_before = est.state()[2][0]
    t += 33_000_000
    est.predict(quat_identity(), np.zeros(3), 0.0, True, 1 / 30, now_ns=t, accel=rest)  # dropout
    v_after = est.state()[2][0]
    assert v_after > 0.5 * v_before, (v_before, v_after)   # velocity persists (no freeze)

    # 4) Delayed VGGT: keep driving, land a stale fix stamped in the past -> re-anchor
    #    without teleporting (pose = truth_at_capture + motion since capture).
    est = ESKFEstimator()
    t = 0
    cap_ns = None
    for k in range(120):
        t += 33_000_000
        est.predict(quat_identity(), np.zeros(3), 0.5, True, 1 / 30, now_ns=t, accel=rest)
        if k == 40:
            cap_ns = t
            cap_x = est.state()[0][0]
    est.correct(Transform.from_xyz_quat([10.0, 0.0, 0.0], quat_identity()),
                capture_ts_ns=cap_ns, now_ns=t)
    x_now = est.state()[0][0]
    assert x_now > 10.0 + 0.3, x_now             # ahead of the stale anchor (no teleport back)
    assert est.have_vggt

    print("eskf_estimator self-test OK  "
          "(rest, cruise, wheel-dropout continuity, delayed-VGGT back-prop)")


if __name__ == "__main__":
    _selftest()
