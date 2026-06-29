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
from frame_io import IncomingFrame
from prism_vggt import FrameInput
from prism_vggt.backends.panovggt import PanoVGGTBackend
from prism_vggt.engine import StreamingWindowEngine

log = logging.getLogger("mapping-server")


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
        self.engine.tsdf_decay = cfg.TSDF_DECAY
        log.info(f"[PRISM] Engine ready ({cfg.summary()}).")

        self._frames: dict[int, FrameInput] = {}
        self._lock = threading.Lock()
        self._last_processed_seq = -1

    # ── frame buffer ─────────────────────────────────────────────────────────
    def reset_map(self):
        with self._lock:
            self._frames.clear()
            self._last_processed_seq = -1
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
        """Generator: process every not-yet-seen window on the contiguous prefix,
        KEEPING the accumulated map, yielding a :class:`SubmapResult` per submap."""
        frames_list, lo, max_seq = self._contiguous_prefix()
        if len(frames_list) < cfg.WINDOW_SIZE:
            return
        # Experimental: rebuild a fresh map from only the most recent frames each
        # batch (reset=True), eliminating cross-batch accumulation/drift at the cost
        # of reprocessing the window and keeping no global map. Default off (online).
        reset = bool(cfg.PRISM_RESET_EACH_BATCH)
        if reset and cfg.RESET_WINDOW_FRAMES > 0 and len(frames_list) > cfg.RESET_WINDOW_FRAMES:
            frames_list = frames_list[-cfg.RESET_WINDOW_FRAMES:]
        try:
            for _mesh, _pcd, traj, _plane in self.engine.process_sequence(
                    frames_list, window_size=cfg.WINDOW_SIZE, overlap=cfg.OVERLAP,
                    reset=reset, finalize=False):
                # THE FIX: publish the CURRENT TSDF surface (thin, gradio-style),
                # not the accumulating BlockColorCache snapshot.
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
        with self._lock:
            self._last_processed_seq = max_seq if max_seq is not None else self._last_processed_seq

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
