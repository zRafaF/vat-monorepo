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

import os
from collections import deque

import numpy as np

from vat_protocol import (
    quat_identity, quat_normalize, quat_rotate, quat_slerp, quat_mul, quat_conj,
    integrate_pose,
)
from kinematics import Transform
from pose_graph import (
    SlidingWindowPoseGraph, se3, se3_inv, quat_to_R, R_to_quat)


class WheelInertialEstimator:
    """Wheel-odometry + IMU dead-reckoner with a *delayed* VGGT re-anchor.

    The VGGT global pose the cloud sends down is **stale**: it describes where the
    camera was at a keyframe captured ~2-4 s ago. Snapping the *current* pose to
    it would teleport the robot backwards and throw away every metre it travelled
    since the keyframe. Instead we run an **out-of-sequence (delayed-measurement)
    correction**, the lightweight form of a fixed-lag pose-graph:

      * the odometry integrator runs in its own slowly-**drifting** frame and we
        keep a short time-indexed **history** of it;
      * a VGGT fix stamped at capture time ``t_c`` is matched against the history
        pose at ``t_c`` to compute the rigid **world←odom** correction transform
        ``T_corr`` (so ``T_corr ∘ odom(t_c) == vggt(t_c)``);
      * the *published* pose is always ``T_corr ∘ odom(now)`` — the global anchor
        is re-based at the right instant while all motion since ``t_c`` is kept.

    This keeps the published pose consistent across many corrections (the raw
    odometry frame is never mutated) and is a drop-in for a real sliding-window
    factor-graph smoother behind the same predict/correct/state interface.

    Parameters
    ----------
    att_gain : float
        Complementary-filter gain pulling the gyro-integrated attitude toward the
        IMU's absolute attitude each step (0 = gyro only, 1 = IMU only).
    pos_gain, rot_gain : float
        How hard each VGGT correction pulls ``T_corr`` toward the freshly computed
        offset (0..1). First fix is always applied in full.
    history_s : float
        Seconds of odometry history to retain — must exceed the worst-case VGGT
        correction latency so ``t_c`` is still in the buffer when the fix lands.
    """

    def __init__(self, att_gain: float = 0.08,
                 pos_gain: float = 0.5, rot_gain: float = 0.5,
                 history_s: float = 15.0):
        # raw odometry-integrated pose, in its own drifting frame
        self._odom_pos = np.zeros(3, dtype=np.float64)
        self._odom_quat = quat_identity()           # xyzw
        self._odom_vel = np.zeros(3, dtype=np.float64)   # world (odom-frame) vel
        # world←odom correction transform (rotation + translation)
        self._corr_R = quat_identity()
        self._corr_p = np.zeros(3, dtype=np.float64)
        # time-indexed odometry history: (t_ns, odom_pos, odom_quat)
        self._history: deque = deque()
        self._history_ns = int(history_s * 1e9)
        self._last_t_ns = 0
        self.have_vggt = False
        self.last_correction_ns = 0
        self._att_gain = att_gain
        self._pos_gain = pos_gain
        self._rot_gain = rot_gain
        self._inited = False
        # Correction backend: "graph" = sliding-window SE(3) pose graph (jointly
        # fits the recent VGGT fixes against the odometry chain → converges in one
        # solve, less jitter, drift-corrected); "blend" = the legacy single-anchor
        # complementary blend. POSE_BACKEND=blend to fall back.
        self._backend = os.environ.get("POSE_BACKEND", "graph").strip().lower()
        self._graph = SlidingWindowPoseGraph(
            window=int(os.environ.get("POSE_GRAPH_WINDOW", "12"))) \
            if self._backend == "graph" else None
        self._prev_odom_kf_T = None

    # -- high-rate prediction -------------------------------------------------
    def predict(self, imu_quat: np.ndarray, gyro: np.ndarray,
                body_vx: float, valid: bool, dt: float,
                body_vy: float = 0.0, now_ns: int | None = None):
        """Advance the state by ``dt`` from one /lowstate sample.

        ``imu_quat`` (xyzw) and ``gyro`` (rad/s, body) are the IMU attitude and
        rate; ``body_vx``/``body_vy`` are the wheel-odometry body velocity (m/s).
        ``now_ns`` stamps the history entry (robot wall clock, same clock the VGGT
        capture timestamp is on); if omitted it is synthesised from ``dt``.
        Does nothing if ``dt<=0`` or the sample is invalid."""
        if dt <= 0 or not valid:
            return
        imu_quat = quat_normalize(imu_quat)
        if not self._inited:
            self._odom_quat = imu_quat
            self._inited = True
        # attitude: gyro-integrate (smooth) then blend toward the IMU absolute
        _, q_gyro = integrate_pose(self._odom_pos, self._odom_quat,
                                   np.zeros(3), gyro, dt)
        self._odom_quat = quat_normalize(
            quat_slerp(q_gyro, imu_quat, self._att_gain))
        # body velocity → odom-world; wheels are non-holonomic so vy defaults to 0
        v_body = np.array([body_vx, body_vy, 0.0], dtype=np.float64)
        self._odom_vel = quat_rotate(self._odom_quat, v_body)
        self._odom_pos = self._odom_pos + self._odom_vel * dt
        # advance + record history on the robot clock
        self._last_t_ns = int(now_ns) if now_ns is not None \
            else self._last_t_ns + int(dt * 1e9)
        self._history.append(
            (self._last_t_ns, self._odom_pos.copy(), self._odom_quat.copy()))
        while len(self._history) > 2 and \
                (self._last_t_ns - self._history[0][0]) > self._history_ns:
            self._history.popleft()

    def _odom_at(self, t_ns: int):
        """Interpolate the drifting odometry pose at past time ``t_ns`` from the
        history buffer. Clamps to the buffer ends; returns ``(pos, quat)`` or the
        live odometry pose if there is no history yet."""
        h = self._history
        if not h:
            return self._odom_pos.copy(), self._odom_quat.copy()
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

    # -- low-rate, possibly-delayed global correction -------------------------
    def correct(self, base_world: Transform, capture_ts_ns: int | None = None,
                now_ns: int | None = None):
        """Re-anchor on a VGGT base pose captured at ``capture_ts_ns``.

        Computes the world←odom offset that makes the odometry pose *at capture
        time* equal the VGGT pose, then blends it into the running ``T_corr`` —
        so the live pose is re-based without teleporting and without discarding
        the motion accumulated since the keyframe."""
        now_ns = now_ns if now_ns is not None else (capture_ts_ns or self._last_t_ns)
        self.last_correction_ns = now_ns

        # where dead-reckoning thought we were at the keyframe's capture time
        if capture_ts_ns is None:
            odom_pos_t, odom_quat_t = self._odom_pos.copy(), self._odom_quat.copy()
        else:
            odom_pos_t, odom_quat_t = self._odom_at(int(capture_ts_ns))

        R_v = quat_normalize(base_world.rotation)
        p_v = np.asarray(base_world.translation, dtype=np.float64).reshape(3)

        # Pose-graph backend: jointly fit this fix against the odometry chain.
        if self._backend == "graph" and self._graph is not None:
            self._correct_graph(odom_pos_t, odom_quat_t, R_v, p_v,
                                 int(capture_ts_ns) if capture_ts_ns is not None else now_ns)
            self.have_vggt = True
            return

        # offset s.t.  R_corr * odom_quat_t == R_v  and
        #              R_corr * odom_pos_t + p_corr == p_v
        R_corr_new = quat_normalize(quat_mul(R_v, quat_conj(quat_normalize(odom_quat_t))))
        p_corr_new = p_v - quat_rotate(R_corr_new, odom_pos_t)

        if not self.have_vggt:
            self._corr_R = R_corr_new
            self._corr_p = p_corr_new
            self.have_vggt = True
        else:
            self._corr_p = (1.0 - self._pos_gain) * self._corr_p \
                + self._pos_gain * p_corr_new
            self._corr_R = quat_normalize(
                quat_slerp(self._corr_R, R_corr_new, self._rot_gain))

    def _correct_graph(self, odom_pos_t, odom_quat_t, R_v, p_v, ts_ns):
        """Pose-graph correction: add this keyframe (seeded from the current
        world←odom anchor and linked to the previous keyframe by the odometry
        relative pose), attach the VGGT absolute factor, optimise the window, and
        recompute the world←odom anchor from the optimised latest keyframe.
        ``state()`` then applies that anchor to live odometry exactly as before."""
        T_odom_kf = se3(quat_to_R(odom_quat_t), np.asarray(odom_pos_t, np.float64))
        T_corr = se3(quat_to_R(self._corr_R), self._corr_p)
        world_init = T_corr @ T_odom_kf
        odom_rel = (se3_inv(self._prev_odom_kf_T) @ T_odom_kf
                    if self._prev_odom_kf_T is not None else None)
        idx = self._graph.add_keyframe(float(ts_ns) * 1e-9, world_init, odom_rel=odom_rel)
        self._graph.add_absolute(idx, se3(quat_to_R(R_v), np.asarray(p_v, np.float64)))
        self._graph.optimize()
        T_kf = self._graph.latest_T()
        if T_kf is not None:
            T_corr_new = T_kf @ se3_inv(T_odom_kf)
            self._corr_R = R_to_quat(T_corr_new[:3, :3])
            self._corr_p = T_corr_new[:3, 3].copy()
        self._prev_odom_kf_T = T_odom_kf

    # -- accessor -------------------------------------------------------------
    def state(self):
        """Returns the *published* world pose (position(3), orientation_xyzw(4),
        world_velocity(3)) = the correction transform applied to live odometry."""
        pos = quat_rotate(self._corr_R, self._odom_pos) + self._corr_p
        quat = quat_normalize(quat_mul(self._corr_R, self._odom_quat))
        vel = quat_rotate(self._corr_R, self._odom_vel)
        return pos, quat, vel

    # back-compat: some callers/tests read these directly
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

    # 3) VGGT correction snaps then holds the anchor (capture == current time).
    est = WheelInertialEstimator()
    est.predict(quat_identity(), np.zeros(3), 0.3, True, 0.02)
    target = Transform.from_xyz_quat([5.0, 2.0, 0.0], quat_identity())
    est.correct(target, capture_ts_ns=est._last_t_ns, now_ns=est._last_t_ns)
    pos, _, _ = est.state()
    assert np.allclose(pos, [5.0, 2.0, 0.0], atol=1e-6), pos
    assert est.have_vggt

    # 4) DELAYED correction must NOT teleport: the robot keeps driving after the
    #    keyframe, the (stale) fix lands later, and the corrected pose must equal
    #    truth_at_capture + motion_since_capture — no metres discarded.
    est = WheelInertialEstimator()
    t = 0
    # drive +x at 0.5 m/s for 1.0 s (50 steps @ 20 ms); remember the capture time
    # and odom pose after 0.4 s, as if a keyframe were captured there.
    cap_ns, cap_odom_x = None, None
    for k in range(50):
        t += 20_000_000                       # +20 ms
        est.predict(quat_identity(), np.zeros(3), 0.5, True, 0.02, now_ns=t)
        if k == 19:                            # 0.40 s in
            cap_ns = t
            cap_odom_x = est.state()[0][0]
    # VGGT says that at capture time the TRUE base was at x=10 (we had drifted to
    # ~0.2). Fix arrives "now" (end of the run) but is stamped at capture time.
    truth_at_capture = Transform.from_xyz_quat([10.0, 0.0, 0.0], quat_identity())
    est.correct(truth_at_capture, capture_ts_ns=cap_ns, now_ns=t)
    pos_now, _, _ = est.state()
    # expected x = truth_at_capture.x + (odom_now - odom_at_capture)
    odom_now_x = est._odom_pos[0]
    expected_x = 10.0 + (odom_now_x - cap_odom_x)
    assert abs(pos_now[0] - expected_x) < 1e-6, (pos_now[0], expected_x)
    # and it must be well ahead of the stale anchor (no teleport back to x=10)
    assert pos_now[0] > 10.0 + 0.25, pos_now[0]

    print("estimator self-test OK  "
          "(straight→x=0.6, yaw→y=0.6, VGGT snap OK, delayed back-prop OK)")


if __name__ == "__main__":
    _selftest()
