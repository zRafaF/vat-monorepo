"""
VAT mapping server — pose-correction extraction & stabilisation.

The server derives the latest VGGT *camera* pose and sends it DOWN to the robot
fuser as a drift correction. This module owns:

  * quaternion helpers,
  * extracting a (position, quaternion) from the VGGT extrinsics (with a
    heading-from-trajectory fallback), and
  * :class:`PoseCorrectionGate` — the commit/outlier/deadband logic that keeps a
    noisy per-submap pose from making the avatar twitch (especially when still).

All logic here is pure NumPy and unit-testable without Zenoh or a GPU.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

import vat_protocol as proto


def rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix → quaternion (x, y, z, w)."""
    R = np.asarray(R, dtype=np.float64)
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w, x, y, z = (0.25 * s, (R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x, y, z = ((R[2, 1] - R[1, 2]) / s, 0.25 * s,
                      (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x, y, z = ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                      0.25 * s, (R[1, 2] + R[2, 1]) / s)
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x, y, z = ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                      (R[1, 2] + R[2, 1]) / s, 0.25 * s)
    q = np.array([x, y, z, w], dtype=np.float64)
    return q / (np.linalg.norm(q) + 1e-12)


def quat_angle_deg(q1, q2) -> float:
    """Smallest rotation angle (deg) between two xyzw quaternions."""
    d = float(np.clip(abs(np.dot(np.asarray(q1), np.asarray(q2))), 0.0, 1.0))
    return float(np.degrees(2.0 * np.arccos(d)))


def camera_pose_from_matrix(M) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """(position, quaternion) from a 4x4 camera-to-world matrix — the true VGGT
    extrinsics in the leveled world frame. Returns ``None`` if degenerate."""
    if M is None:
        return None
    M = np.asarray(M, dtype=np.float64)
    if M.shape != (4, 4) or not np.all(np.isfinite(M)):
        return None
    pos = M[:3, 3]
    quat = rotmat_to_quat(M[:3, :3])
    if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(quat))):
        return None
    return pos.astype(np.float32), quat.astype(np.float32)


def camera_pose_from_trajectory(traj: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Fallback: position + heading-from-tangent orientation, used when the engine
    hasn't produced full extrinsics yet."""
    traj = np.asarray(traj, dtype=np.float64)
    if traj.ndim != 2 or traj.shape[0] == 0:
        return None
    pos = traj[-1]
    quat = proto.quat_identity()
    if traj.shape[0] >= 2:
        fwd = traj[-1] - traj[-2]
        if np.linalg.norm(fwd) > 1e-4:
            yaw = np.arctan2(fwd[1], fwd[0])
            quat = np.array([0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)])
    return pos.astype(np.float32), quat.astype(np.float32)


class PoseCorrectionGate:
    """Decide whether/what to publish as the VGGT pose correction.

    Gates, in order:
      1. commit   — suppress everything until the engine has committed the metric
                    scale (during warm-up the frame is still shifting);
      2. outlier  — reject a pose implying travel faster than ``max_speed`` m/s
                    since the last published one (a drifted/degenerate submap);
      3. deadband — within ``deadband_m`` / ``deadband_deg`` of the last published
                    pose it's still-scene noise → don't republish, so a stationary
                    robot's avatar holds instead of twitching.

    Accepted poses are returned RAW (no smoothing): each is stamped with its
    keyframe capture time and the robot estimator re-anchors odometry at that
    instant, so smoothing the position here would desync it from its timestamp.
    """

    def __init__(self, max_speed: float, jump_margin: float,
                 deadband_m: float, deadband_deg: float):
        self.max_speed = max_speed
        self.jump_margin = jump_margin
        self.deadband_m = deadband_m
        self.deadband_deg = deadband_deg
        self.reset()

    def reset(self):
        self._pos = None
        self._quat = None
        self._ts = None
        # lightweight counters for diagnostics
        self.n_published = 0
        self.n_suppressed = 0
        self.n_rejected = 0

    def evaluate(self, pos, quat, cam_ts_s: Optional[float],
                 scale_committed: bool):
        """Return ``(pos, quat)`` to publish, or ``None`` to suppress.

        The second element of the suppression is conveyed by a status string for
        logging via :attr:`last_reason`.
        """
        self.last_reason = "published"
        if not scale_committed:
            self.last_reason = "warmup (scale not committed)"
            return None
        pos = np.asarray(pos, dtype=np.float64)
        quat = np.asarray(quat, dtype=np.float64)

        if self._pos is not None:
            dt = abs((cam_ts_s or 0.0) - (self._ts or 0.0))
            jump = float(np.linalg.norm(pos - self._pos))
            if dt > 0 and jump > self.max_speed * dt + self.jump_margin:
                self.n_rejected += 1
                self.last_reason = (f"outlier {jump:.2f}m > "
                                    f"{self.max_speed*dt + self.jump_margin:.2f}m/{dt:.2f}s")
                return None
            if jump < self.deadband_m and quat_angle_deg(quat, self._quat) < self.deadband_deg:
                self.n_suppressed += 1
                self.last_reason = "deadband (still)"
                return None

        self._pos = pos.copy()
        self._quat = quat.copy()
        self._ts = cam_ts_s
        self.n_published += 1
        return pos.astype(np.float32), quat.astype(np.float32)
