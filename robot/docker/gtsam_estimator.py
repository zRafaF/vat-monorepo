"""
VAT — GTSAM pose estimator  (IMU preintegration + odometry + delayed VGGT)
==========================================================================
A factor-graph state estimator for the robot's authoritative global pose, built on
GTSAM's IMU preintegration (`CombinedImuFactor`) inside a fixed-lag smoother. It is a
drop-in for :class:`estimator.WheelInertialEstimator` behind the same
``predict / correct / state`` interface, so the fuser, wire format and Zenoh keys are
unchanged — only the estimation quality changes.

Why this is better through accelerations and rotations
------------------------------------------------------
* The **accelerometer is actually used**: high-rate accel+gyro are preintegrated, so the
  motion model is constant-*acceleration*, not the old constant-velocity-from-wheels. Hard
  accel/braking and wheel slip no longer fool it.
* **Odometry** (fused wheel + leg/contact body velocity) enters as a soft world-velocity
  factor that bounds IMU drift and rejects slip — a ZUPT-like correction.
* **Gyro + gravity** give a drift-free attitude inside the same optimisation.
* The **delayed, drift-free VGGT** global pose is added as a `Pose3` prior at the keyframe
  matching its capture time; the fixed-lag smoother fuses it without teleporting (it
  re-optimises the recent window jointly — the principled form of the old back-prop).

The class deliberately RAISES on import if GTSAM is unavailable, and the factory
(:func:`make_estimator` in ``estimator.py``) catches that and any runtime solve failure
to fall back to the proven NumPy estimator — the pose path must never stop publishing.

Frames: world is Z-up, right-handed (ROS REP-103 / nvblox). IMU specific force ``f`` and
true accel ``a`` relate by ``a = R·(f − bias_a) + g`` with ``g = (0,0,−9.81)``; we publish
that world ``a`` so the client can extrapolate at constant acceleration.

⚠️  Noise sigmas below are reasonable starting points — they MUST be tuned on the rig
(IMU datasheet + a static Allan-variance run). This module cannot be validated without
hardware; the unit self-test only checks the API wiring and the rest-state invariants.
"""

from __future__ import annotations

import os
from collections import deque

import numpy as np

import gtsam                                  # raises ImportError if absent → factory falls back
from gtsam import (Pose3, Rot3, NavState, imuBias,
                   PreintegrationCombinedParams, PreintegratedCombinedMeasurements,
                   CombinedImuFactor, PriorFactorPose3, PriorFactorVector,
                   PriorFactorConstantBias, NonlinearFactorGraph, Values)
from gtsam.symbol_shorthand import X, V, B
# The fixed-lag smoother and its key->timestamp map live in gtsam_unstable in current
# builds (older builds exposed them on the top-level gtsam module). Import defensively:
# if neither is present the ImportError propagates and the make_estimator factory falls
# back to the NumPy estimator, so the pose path still runs.
try:
    from gtsam_unstable import BatchFixedLagSmoother, FixedLagSmootherKeyTimestampMap
except Exception:                                  # pragma: no cover
    from gtsam import BatchFixedLagSmoother, FixedLagSmootherKeyTimestampMap

from kinematics import Transform

_G = 9.81


def _ts_map(pairs):
    """Build a FixedLagSmootherKeyTimestampMap from (key, seconds) pairs. The smoother's
    update() requires this native map type - a plain Python dict is NOT accepted."""
    m = FixedLagSmootherKeyTimestampMap()
    for k, t in pairs:
        m.insert((k, float(t)))
    return m


def _rot3_from_xyzw(q) -> Rot3:
    q = np.asarray(q, dtype=np.float64).reshape(4)
    n = np.linalg.norm(q)
    q = q / n if n > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])
    return Rot3.Quaternion(q[3], q[0], q[1], q[2])      # (w, x, y, z)


def _xyzw_from_rot3(R: Rot3) -> np.ndarray:
    q = R.toQuaternion()
    return np.array([q.x(), q.y(), q.z(), q.w()], dtype=np.float64)


class GTSAMImuEstimator:
    """IMU-preintegration fixed-lag smoother. Same interface as WheelInertialEstimator,
    with an extra accelerometer input on ``predict`` and a world-accel in ``state``."""

    def __init__(self, pos_gain: float = 0.5, rot_gain: float = 0.5,
                 history_s: float = 15.0, **_ignored):
        # --- noise (TUNE ON HARDWARE) ---------------------------------------
        sa = float(os.environ.get("IMU_ACCEL_SIGMA", "0.08"))     # m/s²/√Hz
        sg = float(os.environ.get("IMU_GYRO_SIGMA", "0.004"))     # rad/s/√Hz
        sba = float(os.environ.get("IMU_BIAS_ACCEL_SIGMA", "0.004"))
        sbg = float(os.environ.get("IMU_BIAS_GYRO_SIGMA", "0.001"))
        self._odom_sigma = float(os.environ.get("ODOM_VEL_SIGMA", "0.10"))   # m/s
        self._vggt_pos_sigma = float(os.environ.get("VGGT_POS_SIGMA", "0.10"))
        self._vggt_rot_sigma = float(os.environ.get("VGGT_ROT_SIGMA", "0.05"))
        self._kf_dt = float(os.environ.get("KF_DT_S", "0.2"))     # keyframe cadence
        lag = float(os.environ.get("POSE_LAG_S", "6.0"))          # fixed-lag window
        # Fixed-lag window in ns. A keyframe older than this has been marginalised out of
        # the smoother, so a (delayed) VGGT fix must land on a keyframe newer than this -
        # lag MUST exceed the worst-case VGGT correction latency. Node count = POSE_LAG_S /
        # KF_DT_S (~30 keyframes here; small so the batch solve stays under the 30 Hz budget).
        self._lag_ns = int(lag * 1e9)

        p = PreintegrationCombinedParams.MakeSharedU(_G)          # z-up world gravity
        I3 = np.eye(3)
        p.setAccelerometerCovariance(I3 * sa ** 2)
        p.setGyroscopeCovariance(I3 * sg ** 2)
        p.setIntegrationCovariance(I3 * 1e-8)
        p.setBiasAccCovariance(I3 * sba ** 2)
        p.setBiasOmegaCovariance(I3 * sbg ** 2)
        p.setBiasAccOmegaInit(np.eye(6) * 1e-5)
        self._params = p

        self._bias = imuBias.ConstantBias()
        self._pim = PreintegratedCombinedMeasurements(p, self._bias)
        self._smoother = BatchFixedLagSmoother(lag)

        # keyframe state (latest OPTIMISED) + the live forward-predicted nav
        self._i = -1
        self._kf_nav = None                       # NavState at last keyframe (optimised)
        self._kf_bias = self._bias
        self._kf_ns = None
        self._nav = None                          # live forward-predicted NavState
        self._last_accel_body = np.zeros(3)       # last specific force (for world accel)
        self._kf_hist: deque = deque()            # (ns, kf_index) for delayed-VGGT matching
        self._history_ns = int(history_s * 1e9)
        self._inited = False
        self._last_t_ns = 0

        self.have_vggt = False
        self.last_correction_ns = 0
        self.backend_note = f"gtsam (CombinedImuFactor, lag={lag}s, kf_dt={self._kf_dt}s)"

    # ── high-rate prediction ────────────────────────────────────────────────
    def predict(self, imu_quat, gyro, body_vx, valid, dt,
                body_vy: float = 0.0, now_ns: int | None = None,
                accel=None, body_wz: float | None = None):
        """Integrate one /lowstate sample. ``accel`` (body specific force, m/s²) drives
        the IMU preintegration; ``imu_quat`` seeds the initial attitude; ``body_vx/vy``
        are the fused wheel+leg body velocity used as a soft world-velocity factor."""
        if dt <= 0 or not valid:
            return
        gyro = np.asarray(gyro, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(gyro)):
            gyro = np.zeros(3)
        acc = (np.asarray(accel, dtype=np.float64).reshape(3) if accel is not None
               else np.array([0.0, 0.0, _G]))          # rest reaction if no accel wired
        # Reject a placeholder/garbage accelerometer (some topics are filled with junk):
        # without a trustworthy specific force, substitute the rest reaction (+g up) so
        # preintegration can't diverge into freefall — the filter then leans on the gyro
        # and the wheel/contact odometry. A real |a| stays within a few g of gravity.
        an = float(np.linalg.norm(acc))
        if not np.all(np.isfinite(acc)) or an < 3.0 or an > 40.0:
            acc = np.array([0.0, 0.0, _G])
        self._last_accel_body = acc
        self._last_t_ns = int(now_ns) if now_ns is not None else self._last_t_ns + int(dt * 1e9)

        if not self._inited:
            self._bootstrap(imu_quat, self._last_t_ns)

        # preintegrate; advance the live nav by predicting from the last keyframe
        self._pim.integrateMeasurement(acc, gyro, float(dt))
        try:
            self._nav = self._pim.predict(self._kf_nav, self._kf_bias)
        except Exception:
            pass

        # close a keyframe at the configured cadence
        if self._kf_ns is None or (self._last_t_ns - self._kf_ns) >= self._kf_dt * 1e9:
            self._close_keyframe(body_vx, body_vy, self._last_t_ns)

    def _bootstrap(self, imu_quat, t_ns):
        R0 = _rot3_from_xyzw(imu_quat)
        pose0 = Pose3(R0, np.zeros(3))
        self._kf_nav = NavState(pose0, np.zeros(3))
        self._nav = self._kf_nav
        self._i = 0
        self._kf_ns = t_ns
        g, vals = NonlinearFactorGraph(), Values()
        vals.insert(X(0), pose0)
        vals.insert(V(0), np.zeros(3))
        vals.insert(B(0), self._bias)
        # loose priors so the graph is well-constrained before any VGGT lands
        g.add(PriorFactorPose3(X(0), pose0,
              gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.2, 1.0, 1.0, 1.0]))))
        g.add(PriorFactorVector(V(0), np.zeros(3), gtsam.noiseModel.Isotropic.Sigma(3, 0.2)))
        g.add(PriorFactorConstantBias(B(0), self._bias,
              gtsam.noiseModel.Isotropic.Sigma(6, 0.1)))
        t0 = t_ns * 1e-9                     # ABSOLUTE seconds (fixed origin -> monotonic)
        self._smoother.update(g, vals, _ts_map([(X(0), t0), (V(0), t0), (B(0), t0)]))
        self._kf_hist.append((t_ns, 0))
        self._inited = True

    def _close_keyframe(self, body_vx, body_vy, t_ns):
        i, j = self._i, self._i + 1
        # ABSOLUTE seconds with a FIXED origin keeps the fixed-lag timestamps monotonic even
        # after old keyframes are trimmed from _kf_hist. A moving origin (time since the
        # oldest kept keyframe) shifts every stamp on each trim and desyncs the lag window,
        # so the smoother would then marginalise the wrong nodes.
        tj = t_ns * 1e-9
        pred = self._pim.predict(self._kf_nav, self._kf_bias)
        g, vals = NonlinearFactorGraph(), Values()
        vals.insert(X(j), pred.pose())
        vals.insert(V(j), pred.velocity())
        vals.insert(B(j), self._kf_bias)
        g.add(CombinedImuFactor(X(i), V(i), X(j), V(j), B(i), B(j), self._pim))
        # odometry as a soft world-velocity factor: body vel rotated by the predicted
        # attitude (wheels non-holonomic → vy≈0). Bounds IMU drift / rejects slip.
        v_world = pred.pose().rotation().rotate(np.array([body_vx, body_vy, 0.0]))
        g.add(PriorFactorVector(V(j), v_world,
              gtsam.noiseModel.Isotropic.Sigma(3, self._odom_sigma)))
        try:
            self._smoother.update(g, vals, _ts_map([(X(j), tj), (V(j), tj), (B(j), tj)]))
            self._refresh_from_estimate(j)
        except Exception:
            # keep the predicted nav; a transient solve failure must not stop output
            self._kf_nav, self._kf_ns, self._i = pred, t_ns, j
        # reset preintegration onto the (possibly updated) bias for the next interval
        self._pim.resetIntegrationAndSetBias(self._kf_bias)
        self._kf_ns = t_ns
        self._kf_hist.append((t_ns, j))
        while len(self._kf_hist) > 2 and (t_ns - self._kf_hist[0][0]) > self._history_ns:
            self._kf_hist.popleft()

    def _refresh_from_estimate(self, j):
        est = self._smoother.calculateEstimate()
        pose = est.atPose3(X(j))
        vel = est.atVector(V(j))
        self._kf_bias = est.atConstantBias(B(j))
        self._kf_nav = NavState(pose, vel)
        self._nav = self._kf_nav
        self._i = j

    # ── low-rate, possibly-delayed global correction ─────────────────────────
    def correct(self, base_world: Transform, capture_ts_ns: int | None = None,
                now_ns: int | None = None):
        """Add the (stale, drift-free) VGGT base pose as a Pose3 prior at the keyframe
        nearest its capture time. The fixed-lag smoother re-optimises the window, fusing
        it without teleporting. Out-of-window fixes (older than the lag) are dropped."""
        if not self._inited or not self._kf_hist:
            return
        self.last_correction_ns = now_ns if now_ns is not None else self._last_t_ns
        idx = self._nearest_kf(int(capture_ts_ns) if capture_ts_ns is not None else self._last_t_ns)
        if idx is None:
            return
        R = _rot3_from_xyzw(base_world.rotation)
        t = np.asarray(base_world.translation, dtype=np.float64).reshape(3)
        g = NonlinearFactorGraph()
        g.add(PriorFactorPose3(X(idx), Pose3(R, t),
              gtsam.noiseModel.Diagonal.Sigmas(np.array(
                  [self._vggt_rot_sigma] * 3 + [self._vggt_pos_sigma] * 3))))
        try:
            self._smoother.update(g, Values(), _ts_map([]))
            self._refresh_from_estimate(self._i)   # re-read the latest after the fuse
            self.have_vggt = True
        except Exception:
            raise   # let the factory log + fall back to the NumPy estimator

    def _nearest_kf(self, t_ns):
        best, bestd = None, None
        for ns, idx in self._kf_hist:
            if self._last_t_ns - ns > self._lag_ns:
                continue                    # marginalised out of the fixed lag -> unusable
            d = abs(ns - t_ns)
            if bestd is None or d < bestd:
                best, bestd = idx, d
        return best

    # ── accessor ─────────────────────────────────────────────────────────────
    def state(self):
        """(position(3), orientation_xyzw(4), world_velocity(3), world_accel(3))."""
        if self._nav is None:
            return (np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]),
                    np.zeros(3), np.zeros(3))
        pose = self._nav.pose()
        pos = np.asarray(pose.translation(), dtype=np.float64).reshape(3)
        quat = _xyzw_from_rot3(pose.rotation())
        vel = np.asarray(self._nav.velocity(), dtype=np.float64).reshape(3)
        # world linear acceleration = R·(f − bias_a) + g_world  (≈0 at rest)
        f = self._last_accel_body - np.asarray(self._kf_bias.accelerometer(), dtype=np.float64)
        a_world = pose.rotation().rotate(f) + np.array([0.0, 0.0, -_G])
        return pos, quat, vel, a_world

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
# Self-test:  python gtsam_estimator.py   (only runs if gtsam imports)
# ─────────────────────────────────────────────────────────────────────────────
def _selftest():
    from vat_protocol import quat_identity
    est = GTSAMImuEstimator()
    # Rest: feed gravity reaction (0,0,+g), level attitude → pose/vel stay ~0, accel ~0.
    t = 0
    for _ in range(60):                          # ~0.6 s at 100 Hz
        t += 10_000_000
        est.predict(quat_identity(), np.zeros(3), 0.0, True, 0.01,
                    now_ns=t, accel=np.array([0.0, 0.0, _G]))
    pos, quat, vel, acc = est.state()
    assert np.linalg.norm(pos) < 0.2, pos
    assert np.linalg.norm(acc) < 0.5, acc           # gravity correctly cancelled
    # A VGGT prior snaps the anchor near the requested pose.
    tgt = Transform.from_xyz_quat([2.0, 1.0, 0.0], quat_identity())
    est.correct(tgt, capture_ts_ns=t, now_ns=t)
    pos2, _, _, _ = est.state()
    assert np.linalg.norm(pos2[:2] - np.array([2.0, 1.0])) < 0.5, pos2
    print("gtsam_estimator self-test OK  (rest invariants + VGGT prior pull)")


if __name__ == "__main__":
    _selftest()
