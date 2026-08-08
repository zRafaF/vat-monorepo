"""
VAT recorder — the pose streams.
================================
Three separate streams, kept separate on purpose: they are three different
estimates with three different rates, latencies and authorities, and the paper
needs to be able to talk about each one.

:class:`FusedPoseRecorder` — ``{robot}/prism/pose``
    The robot's **authoritative** fused pose (ESKF over wheel odometry + IMU +
    leg-FK, re-anchored by the cloud correction), ~30 Hz. Written as TUM
    (``timestamp tx ty tz qx qy qz qw``) so it drops straight into ``evo`` for ATE,
    **and** as JSONL keeping everything TUM cannot hold: linear/angular velocity,
    linear acceleration, ``fix_quality`` and ``seq``. Accepts both the 72-byte v1
    and 84-byte v2 layouts (``unpack_pose`` zero-fills v1 acceleration).

:class:`CorrectionRecorder` — ``{server}/prism/pose_correction``
    The cloud's slow, drift-free **camera** pose sent DOWN to the robot. The only
    wire message carrying *both* a capture timestamp and a ``map_version``, which
    makes it the exact pin between the session timeline and map state — so every
    sample also calls :meth:`SessionClock.pin_version`.

    §3.2 asks for "cloud pose corrections **+ gating stats**". The gate
    (``PoseCorrectionGate``) runs *before* publishing and its counters never reach
    the wire, so the recorder does the two observable things instead: it recomputes
    each correction's gate-relevant quantities (Δt, jump, implied speed, rotation
    delta, the outlier threshold that applied) against the configured thresholds,
    and it lets the session manifest derive how many submaps were suppressed by
    comparing published corrections against submaps seen in the ``status`` stream.

:class:`TrajectoryRecorder` — ``{server}/prism/trajectory``
    Camera positions for the viewer's trail: **positions only, no orientations, no
    per-pose timestamps**, and truncated to the newest ``TRAJ_MAX_POSES`` (300).
    Recorded for completeness and for the video's trail overlay, with ``ts_src``
    honestly marked ``derived`` — it is *not* a substitute for a timestamped
    trajectory. Use the fused pose TUM file for that.
"""

from __future__ import annotations

import io
import logging
import os
from typing import Optional

import numpy as np

import rec_config as rcfg          # noqa: F401 — also puts repo/common on sys.path

import vat_protocol as proto       # noqa: E402  (needs rec_config's path insert)

from rec_base import StreamRecorder
from rec_clock import SessionClock
from rec_sinks import Budget, SessionWriter

log = logging.getLogger("vat-record")

# Reuse the server's own rotation-delta helper so "3 degrees" means the same thing
# here as it does inside PoseCorrectionGate. pose_estimation imports only numpy +
# vat_protocol, so it loads fine in the client env; the fallback keeps the recorder
# working if server/mapping is ever unavailable (e.g. a client-only checkout).
try:
    import sys as _sys
    if rcfg.MAPPING_DIR not in _sys.path:
        _sys.path.insert(0, rcfg.MAPPING_DIR)
    from pose_estimation import quat_angle_deg
    _GATE_HELPERS = "server/mapping/pose_estimation.py"
except Exception:                                          # pragma: no cover
    def quat_angle_deg(q1, q2) -> float:
        d = float(np.clip(abs(np.dot(np.asarray(q1), np.asarray(q2))), 0.0, 1.0))
        return float(np.degrees(2.0 * np.arccos(d)))
    _GATE_HELPERS = "local fallback"

_FIX = {proto.FIX_DEADRECKON: "deadreckon", proto.FIX_CORRECTED: "corrected"}


# ═════════════════════════════════════════════════════════════════════════════
# Robot fused pose (authoritative)
# ═════════════════════════════════════════════════════════════════════════════


class FusedPoseRecorder(StreamRecorder):
    """Record the robot's authoritative fused pose as TUM + JSONL."""

    name = "poses_robot_fused"

    def __init__(self, sw: SessionWriter, clock: SessionClock,
                 budget: Optional[Budget] = None):
        super().__init__(sw, clock, budget)
        sw.subdir("poses")
        self._tum = sw.tum(
            "poses", "robot_fused.tum",
            comment=("VAT robot fused pose (authoritative), map frame, TUM format.\n"
                     f"key: {rcfg.KEYS['pose']}\n"
                     "timestamp = robot capture clock (seconds); quaternion is xyzw.\n"
                     "Full record incl. velocity/accel/fix_quality: robot_fused.jsonl"))
        self._jsonl = sw.jsonl_index("poses", "robot_fused.jsonl")
        self.n_corrected = 0
        self.n_v1 = 0                 # legacy 72-byte poses (no acceleration)
        self.stats.key = rcfg.KEYS["pose"]

    def attach(self, z) -> None:
        super().attach(z)
        self.subscribe(rcfg.KEYS["pose"], self._on_pose)

    def _on_pose(self, sample) -> None:
        raw = bytes(sample.payload)
        p = proto.unpack_pose(raw)
        st = self.clock.stamp(p.timestamp_ns)
        if len(raw) < 84:
            self.n_v1 += 1
        if p.fix_quality == proto.FIX_CORRECTED:
            self.n_corrected += 1
        self._tum.append(p.timestamp_ns, p.position, p.quaternion)
        self._jsonl.append({
            "src_ts_ns": int(p.timestamp_ns), "ts_src": st.ts_src,
            "wall_ns": st.wall_ns, "mono_ns": st.mono_ns,
            "latency_ms": round(st.latency_ms, 2),
            "seq": int(p.seq),
            "position": [round(float(v), 6) for v in p.position],
            "quaternion": [round(float(v), 9) for v in p.quaternion],
            "linear_velocity": [round(float(v), 6) for v in p.linear_velocity],
            "angular_velocity": [round(float(v), 6) for v in p.angular_velocity],
            "linear_acceleration": [round(float(v), 6) for v in p.linear_acceleration],
            "fix_quality": int(p.fix_quality),
            "fix": _FIX.get(int(p.fix_quality), str(p.fix_quality)),
            "wire_bytes": len(raw),
        })
        self.stats.sample(nbytes=len(raw), src_ts_ns=p.timestamp_ns,
                          wall_ns=st.wall_ns, seq=int(p.seq) & 0x7FFFFFFF,
                          seq_mask=0x7FFFFFFF)

    def extra_summary(self) -> dict:
        return {
            "tum": "poses/robot_fused.tum",
            "jsonl": "poses/robot_fused.jsonl",
            "samples_fix_corrected": self.n_corrected,
            "samples_legacy_v1_72B": self.n_v1,
            "frame": "map (world), Z-up; quaternion xyzw",
        }

    def status_line(self) -> str:
        s = self.stats.summary()
        return f"pose={s['samples']}@{(s['mean_hz'] or 0.0):.0f}Hz"


# ═════════════════════════════════════════════════════════════════════════════
# Cloud pose correction (+ derived gate metrics)
# ═════════════════════════════════════════════════════════════════════════════


class CorrectionRecorder(StreamRecorder):
    """Record cloud pose corrections and pin ``map_version`` → capture time."""

    name = "poses_cloud_correction"
    # Gated and therefore genuinely sparse — a still robot publishes none at all, so
    # this stream must not define the composable window.
    dense = False

    def __init__(self, sw: SessionWriter, clock: SessionClock,
                 budget: Optional[Budget] = None):
        super().__init__(sw, clock, budget)
        sw.subdir("poses")
        self._jsonl = sw.jsonl_index("poses", "cloud_correction.jsonl")
        self._prev = None                # (ts_s, pos, quat) of the previous sample
        self.max_speed = _cfg_float("CORRECTION_MAX_SPEED", 2.5)
        self.jump_margin = _cfg_float("CORRECTION_JUMP_MARGIN", 0.75)
        self.deadband_m = _cfg_float("CORRECTION_DEADBAND_M", 0.06)
        self.deadband_deg = _cfg_float("CORRECTION_DEADBAND_DEG", 3.0)
        self.n_would_deadband = 0
        self.n_nonmonotonic = 0
        self.versions = set()
        self.stats.key = rcfg.KEYS["pose_correction"]

    def attach(self, z) -> None:
        super().attach(z)
        self.subscribe(rcfg.KEYS["pose_correction"], self._on_correction)

    def _on_correction(self, sample) -> None:
        raw = bytes(sample.payload)
        c = proto.unpack_pose_correction(raw)
        st = self.clock.stamp(c.timestamp_ns)
        # The exact version↔capture-time pin: this keyframe time is a genuine
        # FrameInput.timestamp threaded all the way through the engine.
        self.clock.pin_version(c.map_version, c.timestamp_ns, "pose_correction",
                               st.wall_ns)
        self.versions.add(int(c.map_version))

        ts_s = c.timestamp_ns / 1e9
        pos = np.asarray(c.position, dtype=np.float64)
        quat = np.asarray(c.quaternion, dtype=np.float64)
        gate = {"first": self._prev is None}
        if self._prev is not None:
            p_ts, p_pos, p_quat = self._prev
            dt = abs(ts_s - p_ts)
            jump = float(np.linalg.norm(pos - p_pos))
            rot = quat_angle_deg(quat, p_quat)
            thresh = self.max_speed * dt + self.jump_margin
            gate = {
                "first": False,
                "dt_s": round(dt, 6),
                "jump_m": round(jump, 6),
                "implied_speed_mps": round(jump / dt, 4) if dt > 0 else None,
                "rot_delta_deg": round(rot, 4),
                "outlier_threshold_m": round(thresh, 4),
                # Both of these should be False for every published correction; a
                # True is a red flag that the recording and the server disagree on
                # the configured thresholds (check the config hash).
                "would_reject_outlier": bool(dt > 0 and jump > thresh),
                "would_deadband": bool(jump < self.deadband_m
                                       and rot < self.deadband_deg),
                "monotonic_ok": bool(ts_s > p_ts),
            }
            if gate["would_deadband"]:
                self.n_would_deadband += 1
            if not gate["monotonic_ok"]:
                self.n_nonmonotonic += 1
        self._prev = (ts_s, pos.copy(), quat.copy())

        self._jsonl.append({
            "src_ts_ns": int(c.timestamp_ns), "ts_src": st.ts_src,
            "wall_ns": st.wall_ns, "mono_ns": st.mono_ns,
            "latency_ms": round(st.latency_ms, 2),
            "map_version": int(c.map_version),
            "position": [round(float(v), 6) for v in c.position],
            "quaternion": [round(float(v), 9) for v in c.quaternion],
            "frame": "camera pose in map frame",
            "gate": gate,
            "wire_bytes": len(raw),
        })
        self.stats.sample(nbytes=len(raw), src_ts_ns=c.timestamp_ns,
                          wall_ns=st.wall_ns)

    def extra_summary(self) -> dict:
        return {
            "jsonl": "poses/cloud_correction.jsonl",
            "map_versions_corrected": len(self.versions),
            "gate_thresholds": {
                "source": _GATE_HELPERS,
                "CORRECTION_MAX_SPEED": self.max_speed,
                "CORRECTION_JUMP_MARGIN": self.jump_margin,
                "CORRECTION_DEADBAND_M": self.deadband_m,
                "CORRECTION_DEADBAND_DEG": self.deadband_deg,
            },
            "anomalies": {
                "would_deadband": self.n_would_deadband,
                "non_monotonic_capture_ts": self.n_nonmonotonic,
            },
            "note": ("PoseCorrectionGate counters (published/suppressed/rejected/"
                     "stale) are not published on Zenoh. The per-sample `gate` block "
                     "is recomputed by the recorder from the observable stream; the "
                     "suppressed COUNT is derived in MANIFEST.json as "
                     "submaps_seen - corrections_published (needs --status)."),
        }

    def status_line(self) -> str:
        return f"corr={self.stats.n}"


def _cfg_float(name: str, default: float) -> float:
    """Read a gate threshold the same way ``mapping_config`` does."""
    try:
        return float(os.environ.get(name, str(default)))
    except ValueError:
        return default


# ═════════════════════════════════════════════════════════════════════════════
# Camera trajectory (positions only)
# ═════════════════════════════════════════════════════════════════════════════


class TrajectoryRecorder(StreamRecorder):
    """Record the streamed camera-position trail (no orientations, no timestamps)."""

    name = "poses_trajectory"

    def __init__(self, sw: SessionWriter, clock: SessionClock,
                 budget: Optional[Budget] = None):
        super().__init__(sw, clock, budget)
        sw.subdir("poses", "trajectory")
        self._jsonl = sw.jsonl_index("poses", "trajectory.jsonl")
        self._i = 0
        self.stats.key = rcfg.KEYS["trajectory"]

    def attach(self, z) -> None:
        super().attach(z)
        self.subscribe(rcfg.KEYS["trajectory"], self._on_traj)

    def _on_traj(self, sample) -> None:
        raw = bytes(sample.payload)
        pos = proto.unpack_trajectory(raw)
        # No source timestamp on the wire → derived (see rec_clock).
        st = self.clock.stamp(None)
        if pos.shape[0] == 0:
            self.stats.skip()
            return
        if self.budget.expired() or not self.budget.claim(pos.nbytes):
            self.stats.skip()
            return
        self._i += 1
        rel = f"poses/trajectory/traj_{self._i:06d}.npy"
        # Serialise in memory, then let SessionWriter publish it atomically (np.save
        # would append a second '.npy' to a '.tmp' path, defeating tmp+replace).
        buf = io.BytesIO()
        np.save(buf, pos.astype(np.float32), allow_pickle=False)
        self.sw.write_blob(buf.getvalue(), *rel.split("/"))
        self._jsonl.append({
            "src_ts_ns": st.src_ts_ns, "ts_src": st.ts_src,
            "wall_ns": st.wall_ns, "mono_ns": st.mono_ns,
            "n_poses": int(pos.shape[0]), "file": rel,
            "wire_bytes": len(raw),
        })
        self.stats.sample(nbytes=len(raw), src_ts_ns=st.src_ts_ns, wall_ns=st.wall_ns)

    def extra_summary(self) -> dict:
        return {
            "jsonl": "poses/trajectory.jsonl",
            "dir": "poses/trajectory",
            "note": ("pack_trajectory carries camera POSITIONS only — no "
                     "orientations, no per-pose timestamps — and the server truncates "
                     "it to the newest TRAJ_MAX_POSES. ts_src=derived. For a "
                     "timestamped trajectory use poses/robot_fused.tum."),
        }

    def status_line(self) -> str:
        return f"traj={self.stats.n}"


# ═════════════════════════════════════════════════════════════════════════════
# Self-test:  python tools/recorder/rec_poses.py
# ═════════════════════════════════════════════════════════════════════════════

def _selftest() -> None:
    import json
    import shutil
    import tempfile

    class _S:
        def __init__(self, payload):
            self.payload = payload

    tmp = tempfile.mkdtemp(prefix="vatrec-poses-")
    try:
        sw = SessionWriter(tmp, "s")
        clock = SessionClock()
        base = 1_700_000_000_000_000_000

        # ── fused pose: TUM + JSONL, v1 and v2 layouts ──
        fp = FusedPoseRecorder(sw, clock)
        for i in range(4):
            p = proto.PoseState(
                timestamp_ns=base + i * 33_333_333, seq=i,
                position=np.array([i * 0.1, 0.2, 0.3], np.float32),
                quaternion=np.array([0, 0, 0, 1], np.float32),
                linear_velocity=np.array([0.5, 0, 0], np.float32),
                angular_velocity=np.array([0, 0, 0.1], np.float32),
                linear_acceleration=np.array([0.01, 0, 0], np.float32),
                fix_quality=proto.FIX_CORRECTED if i == 2 else proto.FIX_DEADRECKON)
            fp._on_pose(_S(proto.pack_pose(p)))
        # a legacy 72-byte pose must still parse (accel zero-filled)
        fp._on_pose(_S(proto.pack_pose(proto.PoseState(
            timestamp_ns=base + 4 * 33_333_333, seq=4,
            position=np.zeros(3, np.float32), quaternion=np.array([0, 0, 0, 1], np.float32),
            linear_velocity=np.zeros(3, np.float32),
            angular_velocity=np.zeros(3, np.float32)))[:72]))

        s = fp.summary()
        assert s["samples"] == 5, s
        assert s["samples_fix_corrected"] == 1
        assert s["samples_legacy_v1_72B"] == 1
        tum_lines = [l for l in open(sw.path("poses", "robot_fused.tum"))
                     if not l.startswith("#")]
        assert len(tum_lines) == 5
        f0 = tum_lines[0].split()
        assert f0[0] == f"{base / 1e9:.9f}" and len(f0) == 8
        assert f0[1] == "0.000000" and f0[4:] == ["0.000000000"] * 3 + ["1.000000000"]
        j0 = json.loads(open(sw.path("poses", "robot_fused.jsonl")).readline())
        assert j0["src_ts_ns"] == base and j0["ts_src"] == "source"
        assert j0["fix"] == "deadreckon" and j0["wire_bytes"] == 84
        j4 = json.loads(open(sw.path("poses", "robot_fused.jsonl")).read()
                        .splitlines()[4])
        assert j4["wire_bytes"] == 72 and j4["linear_acceleration"] == [0.0, 0.0, 0.0]

        # ── corrections: version pins + derived gate metrics ──
        cr = CorrectionRecorder(sw, clock)
        # a 0.5 m move over 1 s: inside the outlier threshold, outside the deadband
        for i, (dx, mv) in enumerate([(0.0, 11), (0.5, 12), (0.5, 13)]):
            cr._on_correction(_S(proto.pack_pose_correction(proto.PoseCorrection(
                timestamp_ns=base + i * 1_000_000_000, map_version=mv,
                position=np.array([i * dx, 0, 0], np.float32),
                quaternion=np.array([0, 0, 0, 1], np.float32)))))
        recs = [json.loads(l) for l in
                open(sw.path("poses", "cloud_correction.jsonl")).read().splitlines()]
        assert len(recs) == 3
        assert recs[0]["gate"]["first"] is True
        g = recs[2]["gate"]
        assert g["dt_s"] == 1.0 and abs(g["jump_m"] - 0.5) < 1e-6
        assert abs(g["implied_speed_mps"] - 0.5) < 1e-6
        assert abs(g["outlier_threshold_m"] - (2.5 + 0.75)) < 1e-9
        assert g["would_reject_outlier"] is False and g["would_deadband"] is False
        assert g["monotonic_ok"] is True
        # the exact pin lands on the clock, and beats a status approximation
        assert clock.version_pin(13)["capture_ns"] == base + 2_000_000_000
        assert clock.version_pin(13)["source"] == "pose_correction"
        clock.pin_version(13, 1, "status")
        assert clock.version_pin(13)["capture_ns"] == base + 2_000_000_000
        assert cr.summary()["map_versions_corrected"] == 3

        # a still robot inside the deadband is flagged (server should have suppressed it)
        cr._on_correction(_S(proto.pack_pose_correction(proto.PoseCorrection(
            timestamp_ns=base + 3_000_000_000, map_version=14,
            position=np.array([1.0, 0, 0], np.float32),
            quaternion=np.array([0, 0, 0, 1], np.float32)))))
        assert cr.summary()["anomalies"]["would_deadband"] == 1

        # ── trajectory: derived timestamps, .npy round-trip ──
        tr = TrajectoryRecorder(sw, clock)
        traj = np.array([[0, 0, 1.1], [0.5, 0, 1.1], [1.0, 0.2, 1.1]], np.float32)
        tr._on_traj(_S(proto.pack_trajectory(traj)))
        tr._on_traj(_S(proto.pack_trajectory(np.zeros((0, 3), np.float32))))  # skipped
        t = tr.summary()
        assert t["samples"] == 1 and t["skipped"] == 1
        rec0 = json.loads(open(sw.path("poses", "trajectory.jsonl")).readline())
        assert rec0["n_poses"] == 3 and rec0["ts_src"] in ("derived", "wall")
        got = np.load(sw.path(*rec0["file"].split("/")))
        assert np.allclose(got, traj, atol=1e-6)

        sw.close()
        print(f"rec_poses self-test OK  (TUM+JSONL, v1/v2 poses, gate metrics via "
              f"{_GATE_HELPERS}, version pins)")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    _selftest()
