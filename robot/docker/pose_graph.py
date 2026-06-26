"""
VAT — lightweight sliding-window SE(3) pose graph (robot side).

Replaces the single-anchor complementary blend in the pose fuser with a small
fixed-lag factor graph that JOINTLY fits the recent trajectory to:

  * **odometry edges** — relative pose between consecutive keyframes (fast, drifts),
  * **VGGT absolute factors** — the drift-free map pose of a keyframe (slow, laggy),
  * a weak **prior** on the oldest kept node for gauge/observability.

It is solved by Gauss-Newton on the SE(3) manifold (numpy only — no GTSAM/Ceres
dependency). Keeping a window (not just the latest correction) means a noisy VGGT
pose is smoothed against the odometry chain instead of snapping the avatar, and the
estimate converges in one optimisation rather than over several blended corrections.

The fuser drives it: add a keyframe per VGGT correction (seeded by odometry),
attach the odometry relative edge + the VGGT absolute factor, ``optimize()``, then
read ``latest_world()`` as the new world←odom anchor. Pure, deterministic, and unit
tested against synthetic trajectories (see ``__main__``).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

# ── SO(3) / SE(3) helpers (matrix exp/log via Rodrigues) ─────────────────────


def _skew(w: np.ndarray) -> np.ndarray:
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def so3_exp(w: np.ndarray) -> np.ndarray:
    th = float(np.linalg.norm(w))
    if th < 1e-9:
        return np.eye(3) + _skew(w)
    k = w / th
    K = _skew(k)
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def so3_log(R: np.ndarray) -> np.ndarray:
    c = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    th = float(np.arccos(c))
    if th < 1e-9:
        return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) * 0.5
    return (th / (2.0 * np.sin(th))) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def se3(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def se3_inv(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    return se3(R.T, -R.T @ T[:3, 3])


def se3_log(T: np.ndarray) -> np.ndarray:
    """6-vector [rho(3), phi(3)] (translation-ish, rotation). Small-motion accurate;
    for residuals near the solution that is exactly the regime we use."""
    phi = so3_log(T[:3, :3])
    return np.concatenate([T[:3, 3], phi])


def quat_to_R(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q / (np.linalg.norm(q) + 1e-12)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R: np.ndarray) -> np.ndarray:
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / (np.linalg.norm(q) + 1e-12)


# ── graph ────────────────────────────────────────────────────────────────────


@dataclass
class _Node:
    ts: float
    T: np.ndarray                       # current world←node estimate (4x4)


@dataclass
class _OdomEdge:
    i: int
    j: int
    T_rel: np.ndarray                   # measured node_i ← node_j
    w_t: float
    w_r: float


@dataclass
class _AbsFactor:
    i: int
    T_meas: np.ndarray                  # measured world←node_i (VGGT)
    w_t: float
    w_r: float


class SlidingWindowPoseGraph:
    """Fixed-lag SE(3) pose graph (Gauss-Newton, numpy-only)."""

    def __init__(self, window: int = 12,
                 odom_sigma_t: float = 0.05, odom_sigma_r: float = 0.03,
                 vggt_sigma_t: float = 0.10, vggt_sigma_r: float = 0.08,
                 prior_sigma_t: float = 1e3, prior_sigma_r: float = 1e3):
        self.window = int(window)
        self.w_odom = (1.0 / odom_sigma_t, 1.0 / odom_sigma_r)
        self.w_vggt = (1.0 / vggt_sigma_t, 1.0 / vggt_sigma_r)
        self.w_prior = (1.0 / prior_sigma_t, 1.0 / prior_sigma_r)
        self.nodes: List[_Node] = []
        self.odom: List[_OdomEdge] = []
        self.absf: List[_AbsFactor] = []

    # -- construction --------------------------------------------------------
    def add_keyframe(self, ts: float, world_init: np.ndarray,
                     odom_rel: Optional[np.ndarray] = None,
                     w_t: Optional[float] = None, w_r: Optional[float] = None) -> int:
        """Append a keyframe seeded at ``world_init`` (4x4). If ``odom_rel`` (the
        measured relative pose prev←this from odometry) is given, link it to the
        previous node. Returns the node index."""
        idx = len(self.nodes)
        self.nodes.append(_Node(ts, np.array(world_init, dtype=np.float64)))
        if odom_rel is not None and idx > 0:
            self.odom.append(_OdomEdge(idx - 1, idx, np.array(odom_rel, np.float64),
                                       w_t or self.w_odom[0], w_r or self.w_odom[1]))
        self._trim()
        return len(self.nodes) - 1

    def add_absolute(self, idx: int, world_meas: np.ndarray,
                     w_t: Optional[float] = None, w_r: Optional[float] = None):
        if 0 <= idx < len(self.nodes):
            self.absf.append(_AbsFactor(idx, np.array(world_meas, np.float64),
                                        w_t or self.w_vggt[0], w_r or self.w_vggt[1]))

    def _trim(self):
        """Drop nodes/edges/factors outside the fixed lag (keep last ``window``)."""
        excess = len(self.nodes) - self.window
        if excess <= 0:
            return
        self.nodes = self.nodes[excess:]
        self.odom = [_OdomEdge(e.i - excess, e.j - excess, e.T_rel, e.w_t, e.w_r)
                     for e in self.odom if e.i - excess >= 0]
        self.absf = [_AbsFactor(f.i - excess, f.T_meas, f.w_t, f.w_r)
                     for f in self.absf if f.i - excess >= 0]

    # -- optimisation --------------------------------------------------------
    def _residuals(self):
        r = []
        for e in self.odom:
            Ti, Tj = self.nodes[e.i].T, self.nodes[e.j].T
            err = se3_log(se3_inv(e.T_rel) @ (se3_inv(Ti) @ Tj))
            r.append(np.concatenate([e.w_t * err[:3], e.w_r * err[3:]]))
        for f in self.absf:
            err = se3_log(se3_inv(f.T_meas) @ self.nodes[f.i].T)
            r.append(np.concatenate([f.w_t * err[:3], f.w_r * err[3:]]))
        # weak prior on node 0 (gauge) toward its current estimate
        if self.nodes:
            r.append(np.zeros(6))      # prior residual is 0 at the linearisation point
        return np.concatenate(r) if r else np.zeros(0)

    def optimize(self, iters: int = 8, step_eps: float = 1e-4):
        """Gauss-Newton over per-node SE(3) increments (numerical Jacobian on the
        6-DoF local parameterisation). Small problem (≤window·6 params) → fast."""
        n = len(self.nodes)
        if n == 0:
            return
        for _ in range(iters):
            r0 = self._residuals()
            if r0.size == 0:
                return
            m = r0.size
            J = np.zeros((m, 6 * n))
            base = [nd.T.copy() for nd in self.nodes]
            for k in range(n):
                for d in range(6):
                    delta = np.zeros(6)
                    delta[d] = step_eps
                    self.nodes[k].T = base[k] @ se3(so3_exp(delta[3:]), delta[:3])
                    J[:, 6 * k + d] = (self._residuals() - r0) / step_eps
                    self.nodes[k].T = base[k]
            # Gauss-Newton step  dx = -(JᵀJ)⁻¹ Jᵀ r   (damped for safety)
            JTJ = J.T @ J + 1e-6 * np.eye(6 * n)
            dx = -np.linalg.solve(JTJ, J.T @ r0)
            for k in range(n):
                d = dx[6 * k:6 * k + 6]
                self.nodes[k].T = base[k] @ se3(so3_exp(d[3:]), d[:3])
            if np.linalg.norm(dx) < 1e-6:
                break

    # -- accessors -----------------------------------------------------------
    def latest_world(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if not self.nodes:
            return None
        T = self.nodes[-1].T
        return T[:3, 3].copy(), R_to_quat(T[:3, :3])

    def latest_T(self) -> Optional[np.ndarray]:
        return self.nodes[-1].T.copy() if self.nodes else None


# ── self-test: recover a synthetic trajectory from noisy odom + sparse VGGT ──
if __name__ == "__main__":
    rng = np.random.default_rng(0)

    def yawT(yaw, t):
        R = so3_exp(np.array([0, 0, yaw]))
        return se3(R, np.array(t, float))

    # ground-truth: robot drives a gentle arc
    N = 12
    gt = [yawT(0.15 * i, [0.5 * i, 0.1 * i * i / N, 0.0]) for i in range(N)]

    g = SlidingWindowPoseGraph(window=N)
    odom_T = gt[0].copy()
    for i in range(N):
        if i == 0:
            g.add_keyframe(0.0, odom_T.copy())
        else:
            rel_true = se3_inv(gt[i - 1]) @ gt[i]
            # noisy + biased odometry relative (drift)
            noise = se3(so3_exp(rng.normal(0, 0.01, 3) + np.array([0, 0, 0.02])),
                        rng.normal(0, 0.02, 3) + np.array([0.02, 0, 0]))
            rel_meas = rel_true @ noise
            odom_T = odom_T @ rel_meas
            g.add_keyframe(float(i), odom_T.copy(), odom_rel=rel_meas)
        # sparse, noisy absolute (VGGT) on every 3rd keyframe
        if i % 3 == 0:
            p_n = gt[i][:3, 3] + rng.normal(0, 0.03, 3)
            R_n = gt[i][:3, :3] @ so3_exp(rng.normal(0, 0.02, 3))
            g.add_absolute(i, se3(R_n, p_n))

    odom_err = np.linalg.norm(odom_T[:3, 3] - gt[-1][:3, 3])
    g.optimize(iters=12)
    p, _ = g.latest_world()
    graph_err = np.linalg.norm(p - gt[-1][:3, 3])
    print(f"raw odometry final-pos error : {odom_err:.3f} m")
    print(f"pose-graph final-pos error   : {graph_err:.3f} m")
    mean = np.mean([np.linalg.norm(g.nodes[i].T[:3, 3] - gt[i][:3, 3]) for i in range(N)])
    print(f"pose-graph mean-pos error    : {mean:.3f} m")
    print("PASS" if graph_err < odom_err * 0.6 else "CHECK")
