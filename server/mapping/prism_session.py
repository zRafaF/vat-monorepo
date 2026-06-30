"""
VAT mapping server — online PRISM session.

Owns the PRISM-VGGT engine and the seq-keyed frame buffer, and drives the engine
in ONLINE mode (``reset=False``) over the contiguous, gap-free prefix of received
frames. Each processed submap yields the CURRENT TSDF surface (the thin cloud the
offline/gradio path shows) plus the trajectory and the latest camera pose.

Why the current surface and not ``get_point_cloud_snapshot()``: the BlockColorCache
behind that snapshot only ever adds/updates blocks (never removes), so it
ACCUMULATES every block ever seen → thick/fuzzy/duplicated walls. The current
nvblox surface is re-derived each submap, so a shifted surface replaces the old
geometry (thin 1-voxel walls), matching gradio.
"""

from __future__ import annotations

import threading
import traceback
import logging
from dataclasses import dataclass
from typing import Optional

import numpy as np

import mapping_config as cfg
import vat_salt
from frame_io import IncomingFrame
from prism_vggt import FrameInput
from prism_vggt.backends.panovggt import PanoVGGTBackend
from prism_vggt.engine import StreamingWindowEngine

log = logging.getLogger("mapping-server")


def _orthonormalize(R: np.ndarray) -> np.ndarray:
    """Nearest rotation matrix to R (SVD; fixes any reflection)."""
    U, _, Vt = np.linalg.svd(R)
    Rn = U @ Vt
    if np.linalg.det(Rn) < 0:
        U[:, -1] *= -1.0
        Rn = U @ Vt
    return Rn


def rigid_anchor_from_poses(common):
    """Best-fit RIGID transform (SE3, scale=1) mapping a fresh reconstruction's LOCAL
    frame to the persistent WORLD frame, from corresponding camera poses.

    ``common`` = list of ``(P_local 4x4, P_world 4x4)`` for frames shared with the
    previous batch. Each pair implies a candidate ``T = P_world @ inv(P_local)``; for a
    consistent rigid scene they agree, so we average translation and orthonormalise the
    mean rotation. Using full poses (not just positions) stays well-conditioned even
    when the camera barely moved (a near-static robot)."""
    Rs, ts = [], []
    for P_local, P_world in common:
        T = np.asarray(P_world, np.float64) @ np.linalg.inv(np.asarray(P_local, np.float64))
        Rs.append(T[:3, :3]); ts.append(T[:3, 3])
    out = np.eye(4)
    out[:3, :3] = _orthonormalize(np.mean(np.stack(Rs), axis=0))
    out[:3, 3] = np.mean(np.stack(ts), axis=0)
    return out


@dataclass
class SubmapResult:
    version: int
    points: np.ndarray          # (N,3) float32 — CURRENT TSDF surface
    colors: np.ndarray          # (N,3) uint8
    trajectory: Optional[np.ndarray]
    cam_pose: Optional[np.ndarray]   # 4x4 camera→world of the newest keyframe
    cam_ts: Optional[float]          # capture time (s) of that keyframe


class OnlinePRISMSession:
    """PRISM engine + seq-keyed frame buffer, driven online."""

    def __init__(self, weights_path: str):
        log.info("[PRISM] Loading PanoVGGT perception backend...")
        import os
        self.perception = PanoVGGTBackend(
            config_path=os.path.join(os.path.dirname(weights_path),
                                     "..", "third_party", "PanoVGGT",
                                     "training", "config", "default.yaml"),
            weights_path=weights_path,
        )
        self.engine = StreamingWindowEngine(
            perception=self.perception, voxel_size=cfg.VOXEL_SIZE,
            max_depth=cfg.MAX_DEPTH, face_size=cfg.FACE_SIZE)
        self.engine.compute_esdf = False
        self.engine.point_cloud_only = True
        self.engine.processing_mode = cfg.PROCESSING_MODE
        # Streaming-stability config (online ghost / breathing / bandwidth fixes):
        #  * voxel-snap → byte-identical unchanged geometry → stable block CRCs;
        #  * keyframe gating → don't re-integrate a static view (breathing/ghosts);
        #  * tsdf_decay → optional active carving of stale voxels (off unless enabled).
        self.engine.cloud_voxel_snap = cfg.CLOUD_VOXEL_SNAP
        self.engine.keyframe_min_trans_m = cfg.KEYFRAME_MIN_TRANS_M
        self.engine.keyframe_min_rot_deg = cfg.KEYFRAME_MIN_ROT_DEG
        self.engine.keyframe_max_interval_s = cfg.KEYFRAME_MAX_INTERVAL_S
        self.engine.tsdf_decay = cfg.TSDF_DECAY
        self.engine.decay_every_n = cfg.DECAY_EVERY_N
        log.info(f"[PRISM] Engine ready ({cfg.summary()}).")

        self._frames: dict[int, FrameInput] = {}
        self._lock = threading.Lock()
        self._last_processed_seq = -1
        # reset-mode world anchor: keep successive fresh reconstructions in ONE
        # world frame so the cloud doesn't rotate/jump and static geometry lands
        # in the same cubes (delta collapses to the frontier).
        self._prev_world_poses = {}     # {ts_ns:int -> 4x4 world camera pose}
        self._world_anchor = np.eye(4)
        self._salt_db = []      # [{id,ts,desc,pos}] keyframes for revisit detection (v0)

    # ── frame buffer ─────────────────────────────────────────────────────────
    def reset_map(self):
        with self._lock:
            self._frames.clear()
            self._last_processed_seq = -1
            self._prev_world_poses = {}
            self._world_anchor = np.eye(4)
        self.engine.reset()
        log.info("[PRISM] Map reset — TSDF + colorizer + frame buffer cleared.")

    def add_frame(self, seq: int, frame: IncomingFrame) -> bool:
        fi = FrameInput(image=frame.image, mask=frame.mask,
                        camera_height=frame.camera_height, timestamp=frame.timestamp)
        with self._lock:
            new = seq not in self._frames
            self._frames[seq] = fi
            return new

    def has_seq(self, seq: int) -> bool:
        with self._lock:
            return seq in self._frames

    def stats(self):
        with self._lock:
            if not self._frames:
                return 0, None, None, 0
            seqs = sorted(self._frames)
            new = sum(1 for s in seqs if s > self._last_processed_seq)
            return len(self._frames), seqs[0], seqs[-1], new

    def missing_seqs(self, lo: int, hi: int) -> list[int]:
        with self._lock:
            return [s for s in range(lo, hi + 1) if s not in self._frames]

    # ── online processing ──────────────────────────────────────────────────
    def _drop_backlog_if_behind(self) -> bool:
        """If the server has fallen more than BACKLOG_MAX_FRAMES behind real time
        (processing slower than capture → growing latency), drop the stale backlog and
        resync to the most recent frames. Online mode tracks windows by frame INDEX, so
        shifting the frame base requires an engine reset — this snaps latency back to
        real time at the cost of a brief map rebuild (the right trade for live teleop)."""
        cap = cfg.BACKLOG_MAX_FRAMES
        if cap <= 0:
            return False
        with self._lock:
            if not self._frames:
                return False
            seqs = sorted(self._frames)
            newest = seqs[-1]
            behind = newest - self._last_processed_seq
            if behind <= cap:
                return False
            keep_from = newest - max(cfg.BACKLOG_KEEP_FRAMES, cfg.WINDOW_SIZE)
            dropped = [sq for sq in seqs if sq < keep_from]
            for sq in dropped:
                del self._frames[sq]
            self._last_processed_seq = -1
            n_drop, n_keep = len(dropped), len(self._frames)
        self.engine.reset()
        self._prev_world_poses = {}
        self._world_anchor = np.eye(4)
        log.warning(f"[PRISM] backlog guard: {behind} frames behind (cap {cap}) → "
                    f"dropped {n_drop} stale, kept {n_keep}, resync to recent")
        return True

    def _contiguous_prefix(self):
        """Return ``(frames_list, lo, max_seq)`` for the gap-free prefix from the
        smallest buffered seq, so a frame's list index never shifts between calls
        (the engine tracks windows by index; a shifted index breaks the overlap
        correspondence and causes the progressive misalignment we chased)."""
        with self._lock:
            if not self._frames:
                return [], None, None
            seqs = sorted(self._frames)
            lo = seqs[0]
            frames_list, s = [], lo
            while s in self._frames:
                frames_list.append(self._frames[s])
                s += 1
            max_seq = s - 1
            if s <= seqs[-1]:
                log.warning(f"[PRISM] frame gap at seq={s} (have up to {seqs[-1]}); "
                            f"processing contiguous prefix [{lo}..{max_seq}] only")
            return frames_list, lo, max_seq

    def process_new(self):
        """Generator yielding a :class:`SubmapResult` per submap.

        Online (reset=False): keep the accumulated map, stream each submap.
        Reset (reset=True): rebuild a FRESH map from the recent window, then stream ONE
        result re-anchored into the persistent world frame (so it doesn't rotate/jump and
        the delta stays small). See cfg.PRISM_RESET_EACH_BATCH / RESET_WORLD_ANCHOR."""
        self._drop_backlog_if_behind()       # latency self-correct if we fell behind
        frames_list, lo, max_seq = self._contiguous_prefix()
        if len(frames_list) < cfg.WINDOW_SIZE:
            return
        reset = bool(cfg.PRISM_RESET_EACH_BATCH)
        if not getattr(self, "_mode_logged", False):
            log.info(f"[PRISM] >>> MODE = {'RESET (fresh rebuild each batch)' if reset else 'ONLINE (accumulating map)'}"
                     f"  (PRISM_RESET_EACH_BATCH={'1' if reset else '0'})")
            self._mode_logged = True
        if reset:
            yield from self._process_reset(frames_list, max_seq)
            return
        try:
            for _mesh, _pcd, traj, _plane in self.engine.process_sequence(
                    frames_list, window_size=cfg.WINDOW_SIZE, overlap=cfg.OVERLAP,
                    reset=False, finalize=False):
                # publish the CURRENT TSDF surface (thin, gradio-style)
                cloud = self.engine.get_current_cloud()
                xyz, rgb = cloud["points"], cloud["colors"]
                if xyz.shape[0] == 0:
                    continue
                cam_pose, cam_ts = self._newest_camera_pose()
                yield SubmapResult(
                    version=int(cloud["version"]), points=xyz, colors=rgb,
                    trajectory=np.asarray(traj, np.float32) if traj is not None else None,
                    cam_pose=cam_pose, cam_ts=cam_ts)
        except Exception:
            log.error(f"[PRISM] Engine error:\n{traceback.format_exc()}")
        if cfg.SALT_ENABLE:
            self._salt_step(max_seq)
        with self._lock:
            self._last_processed_seq = max_seq if max_seq is not None else self._last_processed_seq

    def _process_reset(self, frames_list, max_seq):
        """Reset mode: rebuild from only the most recent frames, re-anchor the fresh
        cloud + poses into the persistent world frame, and yield ONE result. No global
        map is kept; geometry stays clean (no accumulation), the world frame stays
        consistent (rigid anchor), so the cube diff only ships what actually changed."""
        if cfg.RESET_WINDOW_FRAMES > 0 and len(frames_list) > cfg.RESET_WINDOW_FRAMES:
            frames_list = frames_list[-cfg.RESET_WINDOW_FRAMES:]
        _h0 = getattr(self.engine, "_perc_hits", 0)
        _m0 = getattr(self.engine, "_perc_misses", 0)
        last_traj = None
        try:
            for _m, _p, traj, _pl in self.engine.process_sequence(
                    frames_list, window_size=cfg.WINDOW_SIZE, overlap=cfg.OVERLAP,
                    reset=True, finalize=False):
                last_traj = traj
        except Exception:
            log.error(f"[PRISM] Engine error (reset):\n{traceback.format_exc()}")
            with self._lock:
                self._last_processed_seq = max_seq if max_seq is not None else self._last_processed_seq
            return

        cloud = self.engine.get_current_cloud()
        xyz, rgb = cloud["points"], cloud["colors"]
        ts_arr, poses = self.engine.get_poses()
        T = self._reset_world_anchor(ts_arr, poses)          # local -> world (SE3)
        R, t = T[:3, :3], T[:3, 3]
        if xyz.shape[0]:
            xyz = (np.asarray(xyz, np.float64) @ R.T + t).astype(np.float32)
        traj_w = None
        if last_traj is not None and len(last_traj):
            traj_w = (np.asarray(last_traj, np.float64) @ R.T + t).astype(np.float32)
        cam_local, cam_ts = self._newest_camera_pose()
        cam_w = (T @ np.asarray(cam_local, np.float64)) if cam_local is not None else None
        # remember this batch's WORLD poses (keyed by capture-time ns) for the next anchor
        if ts_arr is not None and poses is not None and len(ts_arr) == len(poses):
            self._prev_world_poses = {int(round(float(ti) * 1e9)): (T @ np.asarray(P, np.float64))
                                      for ti, P in zip(ts_arr, poses)}
        ran = getattr(self.engine, "_perc_misses", 0) - _m0
        cached = getattr(self.engine, "_perc_hits", 0) - _h0
        log.info(f"[PRISM] RESET rebuild: {len(frames_list)} frames | perception {ran} ran / "
                 f"{cached} cached | {int(xyz.shape[0])} surface pts")
        if xyz.shape[0]:
            yield SubmapResult(version=int(cloud["version"]), points=xyz, colors=rgb,
                               trajectory=traj_w, cam_pose=cam_w, cam_ts=cam_ts)
        with self._lock:
            self._last_processed_seq = max_seq if max_seq is not None else self._last_processed_seq

    def _reset_world_anchor(self, ts_arr, poses):
        """SE3 mapping the current fresh-reconstruction local frame to the persistent
        world frame, from frames shared (by capture timestamp) with the previous batch.
        First batch / anchoring off → identity (defines the world). Too little overlap →
        reuse the last anchor to avoid a hard jump."""
        if (not cfg.RESET_WORLD_ANCHOR or not self._prev_world_poses
                or ts_arr is None or poses is None or len(ts_arr) != len(poses)):
            return np.eye(4)
        common = []
        for ti, P in zip(ts_arr, poses):
            w = self._prev_world_poses.get(int(round(float(ti) * 1e9)))
            if w is not None:
                common.append((np.asarray(P, np.float64), np.asarray(w, np.float64)))
        if len(common) < 3:
            log.warning(f"[PRISM] reset anchor: only {len(common)} overlapping frames — "
                        f"reusing last anchor (world may shift)")
            return self._world_anchor.copy()
        self._world_anchor = rigid_anchor_from_poses(common)
        return self._world_anchor.copy()

    def _salt_step(self, max_seq):
        """SALT v0 (read-only): build a keyframe DB and LOG revisits + the pose drift
        between the current camera and the matched older keyframe. The drift is the
        accumulated open-loop error — large values here confirm that alignment is what
        stops the TSDF carving (and is what loop-frame injection in v1 would fix)."""
        try:
            fr = self._frames.get(max_seq)
            if fr is None:
                return
            ts = float(fr.timestamp)
            cam_pose, _cam_ts = self._newest_camera_pose()
            if cam_pose is None:
                return
            pos = np.asarray(cam_pose[:3, 3], np.float64)
            desc = vat_salt.descriptor(fr.image)
            best = None                                    # (sim, kf)
            for kf in self._salt_db:
                if ts - kf["ts"] < cfg.SALT_MIN_GAP_S:
                    continue
                sim, _shift = vat_salt.similarity(desc, kf["desc"])
                if best is None or sim > best[0]:
                    best = (sim, kf)
            if best is not None and best[0] >= cfg.SALT_SIM_THRESH:
                drift = float(np.linalg.norm(pos - best[1]["pos"]))
                log.info(f"[SALT] revisit: sim={best[0]:.3f}  Δt={ts - best[1]['ts']:.1f}s  "
                         f"pos_gap(drift)={drift:.2f}m  (kf #{best[1]['id']}, db={len(self._salt_db)})")
            if not self._salt_db or (ts - self._salt_db[-1]["ts"]) >= cfg.SALT_KF_EVERY_S:
                self._salt_db.append({"id": len(self._salt_db), "ts": ts,
                                      "desc": desc, "pos": pos.copy()})
                if len(self._salt_db) > cfg.SALT_DB_MAX:
                    self._salt_db.pop(0)
        except Exception:
            log.debug("[SALT] step failed", exc_info=True)

    def _newest_camera_pose(self):
        """Newest camera pose (by capture timestamp) + its capture time, for the
        pose correction. Append order isn't time order across windows/overlap."""
        try:
            ts, poses = self.engine.get_poses()
            if poses is None or len(poses) == 0:
                return None, None
            if ts is not None and len(ts) == len(poses):
                newest = int(np.argmax(ts))
                return np.asarray(poses[newest], np.float64), float(ts[newest])
            return np.asarray(poses[-1], np.float64), None
        except Exception:
            return None, None

    # ── full-res on-demand snapshot (the press-1 / make fetch_pcd path) ───────
    def current_cloud(self):
        """The current TSDF surface as ``(xyz float32, rgb uint8, version)`` — same
        source as the streamed cloud, so the on-demand fetch can't disagree with it."""
        c = self.engine.get_current_cloud()
        return c["points"], c["colors"], int(c["version"])
