"""
VAT mapping server — PRISM session (engine driver).
===================================================

Owns the PRISM-VGGT engine and drives it over the buffered frames. Two modes:

* RESET (default, ``PRISM_RESET_EACH_BATCH=1``): rebuild a FRESH map from only the
  most recent ``RESET_WINDOW_FRAMES`` frames each batch (a "rolling mini-gradio"),
  then re-anchor that fresh reconstruction into ONE persistent world frame
  (:class:`world_anchor.WorldAnchor`) and stream ONE result. No global map is kept,
  so cross-batch pose drift and revisit ghosts never accumulate and the map stays
  bounded → constant live latency. The reset is done IN PLACE via ``mapper.clear()``
  (engine ``SOFT_RESET``), so it is cheap.

* ONLINE (``PRISM_RESET_EACH_BATCH=0``) — DEPRECATED: keep the accumulating map and
  stream each submap's current TSDF surface. Retained behind the flag for A/B on the
  rig; accumulates drift/ghosts and grows latency (the reason reset mode is default).

Frame bookkeeping lives in :class:`frame_buffer.FrameBuffer`; the persistent world
frame in :class:`world_anchor.WorldAnchor`. This module is just the glue that turns
buffered frames into :class:`SubmapResult`s.
"""

from __future__ import annotations

import logging
import os
import traceback
from dataclasses import dataclass
from typing import Optional

import numpy as np

import mapping_config as cfg
from frame_io import IncomingFrame
from frame_buffer import FrameBuffer
from world_anchor import WorldAnchor
from prism_vggt import FrameInput
from prism_vggt.backends.panovggt import PanoVGGTBackend
from prism_vggt.engine import StreamingWindowEngine

log = logging.getLogger("mapping-server")


@dataclass
class SubmapResult:
    version: int
    points: np.ndarray               # (N,3) float32 — CURRENT TSDF surface (world frame)
    colors: np.ndarray               # (N,3) uint8
    trajectory: Optional[np.ndarray]
    cam_pose: Optional[np.ndarray]   # 4x4 camera→world of the newest keyframe
    cam_ts: Optional[float]          # capture time (s) of that keyframe
    world_anchor: Optional[np.ndarray] = None   # 4x4 local→world SE3 used this batch
    #                                             (identity in online mode; drives nav ESDF)
    obs_centers: Optional[np.ndarray] = None    # (M,3) camera positions added THIS submap
    #                                             (blocks-mode observation-TTL only)


class OnlinePRISMSession:
    """PRISM engine + seq-keyed frame buffer, driven in reset (default) or online mode."""

    def __init__(self, weights_path: str):
        log.info("[PRISM] Loading PanoVGGT perception backend...")
        self.perception = PanoVGGTBackend(
            config_path=os.path.join(os.path.dirname(weights_path),
                                     "..", "third_party", "PanoVGGT",
                                     "training", "config", "default.yaml"),
            weights_path=weights_path,
        )
        self.engine = StreamingWindowEngine(
            perception=self.perception, voxel_size=cfg.VOXEL_SIZE,
            max_depth=cfg.MAX_DEPTH, face_size=cfg.FACE_SIZE)
        # ESDF for navigation (world-frame slice published by nav_esdf). Bounded + cheap
        # in reset mode because the volume is a small recent-window local map.
        self.engine.compute_esdf = cfg.COMPUTE_ESDF
        self.engine.esdf_slice_resolution = cfg.NAV_ESDF_RES_M
        self.engine.point_cloud_only = True
        self.engine.processing_mode = cfg.PROCESSING_MODE
        self.engine.soft_reset = cfg.SOFT_RESET
        # Streaming-stability config (voxel-snap for byte-identical unchanged geometry;
        # keyframe gating so a static view isn't re-integrated; decay only matters for
        # the deprecated online path — reset mode never accumulates, so nothing to carve).
        self.engine.cloud_voxel_snap = cfg.CLOUD_VOXEL_SNAP
        self.engine.keyframe_min_trans_m = cfg.KEYFRAME_MIN_TRANS_M
        self.engine.keyframe_min_rot_deg = cfg.KEYFRAME_MIN_ROT_DEG
        self.engine.keyframe_max_interval_s = cfg.KEYFRAME_MAX_INTERVAL_S
        self.engine.tsdf_decay = cfg.TSDF_DECAY
        self.engine.decay_every_n = cfg.DECAY_EVERY_N
        self.engine.tsdf_prune_radius = cfg.TSDF_PRUNE_RADIUS_M
        log.info(f"[PRISM] Engine ready ({cfg.summary()}).")

        self._buffer = FrameBuffer()
        self._anchor = WorldAnchor(enabled=cfg.RESET_WORLD_ANCHOR)
        self.last_world_anchor = np.eye(4)   # local→world SE3 of the last batch (nav ESDF)
        self._mode_logged = False

    # ── frame buffer (delegated) ───────────────────────────────────────────────
    def reset_map(self):
        self._buffer.clear()
        self._anchor.reset()
        self.last_world_anchor = np.eye(4)
        # Explicit user reset → full reconstruct (infrequent; guarantees a clean Mapper).
        self.engine.reset(soft=False)
        log.info("[PRISM] Map reset — TSDF + colorizer + frame buffer cleared.")

    def add_frame(self, seq: int, frame: IncomingFrame) -> bool:
        fi = FrameInput(image=frame.image, mask=frame.mask,
                        camera_height=frame.camera_height, timestamp=frame.timestamp)
        return self._buffer.add(seq, fi)

    def has_seq(self, seq: int) -> bool:
        return self._buffer.has(seq)

    def stats(self):
        return self._buffer.stats()

    def missing_seqs(self, lo: int, hi: int) -> list[int]:
        return self._buffer.missing(lo, hi)

    # ── processing ─────────────────────────────────────────────────────────────
    def process_new(self):
        """Generator yielding a :class:`SubmapResult` per submap."""
        reset = bool(cfg.PRISM_RESET_EACH_BATCH)
        if not self._mode_logged:
            log.info(f"[PRISM] >>> MODE = "
                     f"{'RESET (fresh rebuild each batch)' if reset else 'ONLINE (accumulating map) [DEPRECATED]'}"
                     f"  (PRISM_RESET_EACH_BATCH={'1' if reset else '0'})")
            self._mode_logged = True

        if reset:
            # Bound the buffer: reset mode only ever uses the recent window, so keep a
            # little more than one window and drop the rest (else the dict grows forever
            # and latency creeps back up — the exact failure mode we're fixing).
            keep = max(cfg.RESET_WINDOW_FRAMES, cfg.WINDOW_SIZE) + cfg.WINDOW_SIZE
            dropped = self._buffer.trim_to_recent(keep)
            if dropped:
                log.debug(f"[PRISM] trimmed {dropped} old frames (reset window={keep})")
        else:
            self._drop_backlog_if_behind()

        frames_list, lo, max_seq, gap = self._buffer.contiguous_prefix()
        if gap is not None:
            log.warning(f"[PRISM] frame gap at seq={gap}; processing contiguous prefix "
                        f"[{lo}..{max_seq}] only")
        if len(frames_list) < cfg.WINDOW_SIZE:
            return

        if reset:
            yield from self._process_reset(frames_list, max_seq)
        else:
            yield from self._process_online(frames_list, max_seq)

    def _process_reset(self, frames_list, max_seq):
        """Reset mode: rebuild from only the most recent frames, re-anchor the fresh
        cloud + poses into the persistent world frame, and yield ONE result."""
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
            self._buffer.mark_processed(max_seq)
            return

        cloud = self.engine.get_current_cloud()
        xyz, rgb = cloud["points"], cloud["colors"]
        ts_arr, poses = self.engine.get_poses()
        T = self._anchor.compute(ts_arr, poses)          # local -> world (SE3)
        self.last_world_anchor = T
        R, t = T[:3, :3], T[:3, 3]
        if xyz.shape[0]:
            xyz = (np.asarray(xyz, np.float64) @ R.T + t).astype(np.float32)
        traj_w = None
        if last_traj is not None and len(last_traj):
            traj_w = (np.asarray(last_traj, np.float64) @ R.T + t).astype(np.float32)
        cam_local, cam_ts = self._newest_camera_pose()
        cam_w = (T @ np.asarray(cam_local, np.float64)) if cam_local is not None else None
        self._anchor.remember(ts_arr, poses, T)
        ran = getattr(self.engine, "_perc_misses", 0) - _m0
        cached = getattr(self.engine, "_perc_hits", 0) - _h0
        log.info(f"[PRISM] RESET rebuild: {len(frames_list)} frames | perception {ran} ran / "
                 f"{cached} cached | {int(xyz.shape[0])} surface pts")
        if xyz.shape[0]:
            yield SubmapResult(version=int(cloud["version"]), points=xyz, colors=rgb,
                               trajectory=traj_w, cam_pose=cam_w, cam_ts=cam_ts,
                               world_anchor=T)
        self._buffer.mark_processed(max_seq)

    def _process_online(self, frames_list, max_seq):
        """DEPRECATED online-accumulate path. Streams each submap's current TSDF surface
        from ONE growing map. Kept behind PRISM_RESET_EACH_BATCH=0 for A/B only."""
        try:
            prev_traj_len = len(self.engine.trajectory)
            for _mesh, _pcd, traj, _plane in self.engine.process_sequence(
                    frames_list, window_size=cfg.WINDOW_SIZE, overlap=cfg.OVERLAP,
                    reset=False, finalize=False):
                cloud = self.engine.get_current_cloud()
                xyz, rgb = cloud["points"], cloud["colors"]
                if xyz.shape[0] == 0:
                    continue
                full_traj = np.asarray(traj, np.float32) if traj is not None else None
                obs_centers = None
                if full_traj is not None and len(full_traj):
                    obs_centers = (full_traj[prev_traj_len:] if len(full_traj) > prev_traj_len
                                   else full_traj[-cfg.WINDOW_SIZE:])
                    prev_traj_len = len(full_traj)
                cam_pose, cam_ts = self._newest_camera_pose()
                yield SubmapResult(
                    version=int(cloud["version"]), points=xyz, colors=rgb,
                    trajectory=full_traj, cam_pose=cam_pose, cam_ts=cam_ts,
                    world_anchor=np.eye(4), obs_centers=obs_centers)
        except Exception:
            log.error(f"[PRISM] Engine error:\n{traceback.format_exc()}")
        self._buffer.mark_processed(max_seq)

    def _drop_backlog_if_behind(self) -> bool:
        """(Online mode) If the server fell more than BACKLOG_MAX_FRAMES behind real
        time, drop the stale backlog and resync to recent frames so latency
        self-corrects. Online tracks windows by index, so shifting the base requires an
        engine reset. The viewer briefly blanks while the map rebuilds (reset mode, the
        default, does not have this problem)."""
        cap = cfg.BACKLOG_MAX_FRAMES
        if cap <= 0 or self._buffer.behind() <= cap:
            return False
        keep = max(cfg.BACKLOG_KEEP_FRAMES, cfg.WINDOW_SIZE)
        dropped = self._buffer.trim_to_recent(keep)
        self.engine.reset(soft=self.engine.soft_reset)
        self._anchor.reset()
        log.warning(f"[PRISM] backlog guard: dropped {dropped} stale frames, resync to recent")
        return True

    def _newest_camera_pose(self):
        """Newest camera pose (by capture timestamp) + its capture time. Append order
        isn't time order across windows/overlap."""
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
        """The current TSDF surface as ``(xyz float32, rgb uint8, version)`` in the
        WORLD frame — same source as the streamed cloud, so the on-demand fetch can't
        disagree with it."""
        c = self.engine.get_current_cloud()
        xyz, rgb = c["points"], c["colors"]
        T = self.last_world_anchor
        if xyz.shape[0] and not np.allclose(T, np.eye(4)):
            xyz = (np.asarray(xyz, np.float64) @ T[:3, :3].T + T[:3, 3]).astype(np.float32)
        return xyz, rgb, int(c["version"])
