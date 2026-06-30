"""
VAT — Mapping Server (orchestrator)
===================================
The cloud-side mapping & routing process. PRISM-VGGT is the perception backend it
drives. This file is intentionally thin: it wires Zenoh I/O to the focused modules

  * mapping_config   — all tunables + Zenoh keys
  * frame_io         — VAT frame decode + spherical mask
  * prism_session    — PRISM engine + seq-keyed frame buffer (online driving)
  * pose_estimation  — VGGT camera-pose extraction + correction stabilisation
  * block_publisher  — diff-based cube-grid cloud sync

and runs the batch loop. The map is built from the CURRENT TSDF surface each
submap (thin, gradio-equivalent) — NOT the accumulating BlockColorCache, which was
the cause of the thick/fuzzy/duplicated walls.

Zenoh keys + wire formats: see common/vat_protocol.py.
"""

from __future__ import annotations

import os
import json
import time
import logging
import threading
import traceback

import numpy as np
import zenoh

os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

import mapping_config as cfg
import vat_protocol as proto
import vat_blockmap as bm
from block_publisher import BlockPublisher
from frame_io import build_mask, decode_frame
from prism_session import OnlinePRISMSession
from pose_estimation import (
    PoseCorrectionGate, camera_pose_from_matrix, camera_pose_from_trajectory)
from telemetry import ClockOffsetEstimator, ThroughputMeter

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="[%(asctime)s] [%(levelname)s] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("mapping-server")

_KEYS = cfg.KEYS


class MappingServer:
    def __init__(self):
        self._processing = False
        self._last_window_t = time.time()
        self._max_seq_seen = -1
        self._gap_count = 0
        self._frame_rx = 0
        self._last_hb = time.time()
        self._reset_requested = False
        self._last_submap_t = 0.0
        self._cloud_mbps = 0.0
        self._mask = build_mask()
        self._ceiling_z = cfg.CEILING_Z      # None = send whole cloud (no slicing)
        # telemetry: robot→server clock offset + frame throughput (see telemetry.py)
        self._clock = ClockOffsetEstimator()
        self._rx_meter = ThroughputMeter()
        self._last_frame_robot_ns = 0
        self._corr_gate = PoseCorrectionGate(
            cfg.CORRECTION_MAX_SPEED, cfg.CORRECTION_JUMP_MARGIN,
            cfg.CORRECTION_DEADBAND_M, cfg.CORRECTION_DEADBAND_DEG)

        log.info(f"[Server] Connecting to Zenoh at {cfg.ZENOH_ROUTER}...")
        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", f'["{cfg.ZENOH_ROUTER}"]')
        conf.insert_json5("mode", '"client"')
        self._z = self._open_with_retry(conf)
        log.info("[Server] Zenoh connected.")

        self._pub_delta    = self._z.declare_publisher(
            _KEYS["pcd_delta"], congestion_control=zenoh.CongestionControl.BLOCK)
        self._pub_snapshot = self._z.declare_publisher(
            _KEYS["pcd_snapshot"], congestion_control=zenoh.CongestionControl.BLOCK)
        self._pub_traj     = self._z.declare_publisher(
            _KEYS["trajectory"], congestion_control=zenoh.CongestionControl.DROP)
        self._pub_status   = self._z.declare_publisher(
            _KEYS["status"], congestion_control=zenoh.CongestionControl.DROP)
        self._pub_pose_cor = self._z.declare_publisher(
            _KEYS["pose_correction"], congestion_control=zenoh.CongestionControl.DROP)

        self._blockpub = BlockPublisher(self._z, cube_m=cfg.CUBE_SIZE,
                                        server_prefix=cfg.SERVER_PREFIX,
                                        crc_quant_m=cfg.CRC_QUANT_M)
        try:
            self._live = self._z.liveliness().declare_token(_KEYS["live_server"])
        except Exception:
            self._live = None

        self._prism = None
        threading.Thread(target=self._init_prism, daemon=True).start()

        self._z.declare_subscriber(_KEYS["camera_frame"], self._on_camera_frame)
        self._z.declare_queryable(_KEYS["pcd_snapshot"], self._on_snapshot_query)
        self._z.declare_subscriber(cfg.RESET_KEY, self._on_reset)
        self._z.declare_subscriber(cfg.CEILING_KEY, self._on_ceiling)
        log.info(f"[Server] ceiling clip: "
                 f"{'OFF (whole cloud)' if self._ceiling_z is None else f'Z<={self._ceiling_z:.2f}m'}"
                 f"  (set live on '{cfg.CEILING_KEY}')")

        threading.Thread(target=self._batch_loop, daemon=True).start()
        log.info(f"[Server] frames←'{_KEYS['camera_frame']}'  "
                 f"pcd→'{_KEYS['pcd_delta']}'  correction→'{_KEYS['pose_correction']}'  "
                 f"batch: {cfg.WINDOW_SIZE - cfg.OVERLAP} new frames OR {cfg.WINDOW_TIMEOUT_S}s")

    @staticmethod
    def _open_with_retry(conf):
        while True:
            try:
                return zenoh.open(conf)
            except Exception as e:
                log.warning(f"[Server] Zenoh connect failed: {e} — retrying in 5s")
                time.sleep(5)

    def _init_prism(self):
        try:
            self._prism = OnlinePRISMSession(cfg.WEIGHTS_PATH)
            self._publish_status("ready")
        except Exception:
            log.error(f"[Server] PRISM init failed:\n{traceback.format_exc()}")
            self._publish_status("error")

    def _publish_status(self, state, extra=None):
        payload = {"state": state, "ts": time.time()}
        if extra:
            payload.update(extra)
        try:
            self._pub_status.put(json.dumps(payload).encode())
        except Exception:
            pass

    # ── frame intake ──────────────────────────────────────────────────────────
    def _on_camera_frame(self, sample):
        try:
            decoded = decode_frame(bytes(sample.payload), self._mask)
        except proto.ProtocolError as e:
            log.warning(f"[Server] bad frame: {e}")
            return
        except Exception:
            log.error(f"[Server] frame decode error:\n{traceback.format_exc()}")
            return
        if decoded is None:
            log.warning("[Server] JPEG decode failed")
            return
        seq, frame = decoded
        if self._prism is None:
            return
        # telemetry: robot→server clock offset + throughput (payload = wire bytes)
        robot_ns = int(frame.timestamp * 1e9)
        self._clock.update(robot_ns)
        self._rx_meter.add(len(bytes(sample.payload)))
        self._last_frame_robot_ns = robot_ns
        self._prism.add_frame(seq, frame)
        self._frame_rx += 1
        total, _lo, _hi, new = self._prism.stats()
        if total < cfg.WINDOW_SIZE:
            log.info(f"[Server] frame seq={seq} — accumulating "
                     f"{total}/{cfg.WINDOW_SIZE} (first map needs {cfg.WINDOW_SIZE})")
        else:
            log.info(f"[Server] frame seq={seq} — buffer {total} | new since last map: {new}")
        if self._max_seq_seen >= 0 and seq > self._max_seq_seen + 1:
            self._gap_count += seq - self._max_seq_seen - 1
            log.warning(f"[Server] seq gap: jumped {self._max_seq_seen}→{seq} "
                        f"(will attempt retransmit before next window)")
        self._max_seq_seen = max(self._max_seq_seen, seq)

    def _on_snapshot_query(self, query):
        """On-demand full snapshot (press 1 / make fetch_pcd). Serves the CURRENT
        TSDF surface — the SAME source as the stream — so the fetch can't show a
        different (accumulated/duplicated) cloud than what is being streamed."""
        try:
            if self._prism is None:
                query.reply(query.key_expr, b""); return
            xyz, rgb, version = self._prism.current_cloud()
            xyz, rgb = self._clip_ceiling(xyz, rgb)
            if xyz.shape[0] == 0:
                query.reply(query.key_expr, b""); return
            query.reply(query.key_expr, proto.pack_pcd(version, xyz, rgb, is_snapshot=True))
        except Exception:
            log.error(f"[Server] snapshot query error:\n{traceback.format_exc()}")

    # ── reset ─────────────────────────────────────────────────────────────────
    def _on_reset(self, sample):
        log.info("[Server] reset command received.")
        self._reset_requested = True

    def _on_ceiling(self, sample):
        """Live ceiling-height config from the client. Empty/'off'/'none'/non-finite
        disables clipping (whole cloud sent)."""
        try:
            raw = bytes(sample.payload).decode("utf-8", "ignore")
        except Exception:
            return
        self._ceiling_z = cfg._parse_ceiling(raw)
        log.info(f"[Server] ceiling clip → "
                 f"{'OFF (whole cloud)' if self._ceiling_z is None else f'Z<={self._ceiling_z:.2f}m'}"
                 f" (from '{raw.strip()}')")

    def _clip_ceiling(self, xyz, rgb):
        """Drop points above the ceiling height (world Z-up). No-op if disabled."""
        if self._ceiling_z is None or xyz.shape[0] == 0:
            return xyz, rgb
        keep = xyz[:, 2] <= self._ceiling_z
        return xyz[keep], rgb[keep]

    def _do_reset(self):
        self._reset_requested = False
        log.info("[Server] >> RESET: clearing TSDF map + frame buffer…")
        try:
            self._prism.reset_map()
        except Exception:
            log.error(f"[Server] reset failed:\n{traceback.format_exc()}")
            return
        self._max_seq_seen = -1
        self._gap_count = 0
        self._last_window_t = time.time()
        self._corr_gate.reset()
        self._rx_meter = ThroughputMeter()
        self._blockpub.reset()
        try:
            empty = np.zeros((0, 3), dtype=np.float32)
            self._pub_snapshot.put(proto.pack_pcd(0, empty, empty, is_snapshot=True))
        except Exception:
            log.error(f"[Server] reset clear-publish failed:\n{traceback.format_exc()}")
        self._publish_status("reset")
        log.info("[Server] >> RESET done. Rebuilding from new frames.")

    # ── frame-drop recovery ─────────────────────────────────────────────────
    def _recover_gaps(self):
        total, lo_seq, hi_seq, _new = self._prism.stats()
        if total == 0 or hi_seq is None:
            return
        lo = max(lo_seq, hi_seq - cfg.WINDOW_SIZE * 2)
        missing = self._prism.missing_seqs(lo, hi_seq)[:cfg.MAX_RETRIES_CYCLE]
        for seq in missing:
            try:
                sel = f"{_KEYS['camera_frame_get']}?seq={seq}"
                got = False
                for reply in self._z.get(sel, timeout=cfg.RETRY_TIMEOUT_S):
                    if reply.ok:
                        data = bytes(reply.result.payload)
                        if len(data) > 20:
                            decoded = decode_frame(data, self._mask)
                            if decoded is not None:
                                self._prism.add_frame(*decoded)
                                got = True
                log.info(f"[Server] recovered dropped frame seq={seq}" if got else
                         f"[Server] could not recover seq={seq} (no longer buffered)")
            except Exception as e:
                log.warning(f"[Server] retransmit query failed seq={seq}: {e}")

    # ── batching driver ───────────────────────────────────────────────────────
    def _batch_loop(self):
        while True:
            time.sleep(0.1)
            if self._prism is None or self._processing:
                continue
            if self._reset_requested:
                self._do_reset()
                continue
            total, _lo, _hi, new = self._prism.stats()
            now = time.time()
            if now - self._last_hb >= 5.0:
                self._last_hb = now
                if total == 0:
                    log.info("[Server] waiting for first frame… "
                             "(is the robot streaming go2/prism/camera/frame?)")
                elif total < cfg.WINDOW_SIZE:
                    log.info(f"[Server] accumulating {total}/{cfg.WINDOW_SIZE} frames…")
            if total < cfg.WINDOW_SIZE:
                continue
            by_frames = new >= (cfg.WINDOW_SIZE - cfg.OVERLAP)
            by_time = (new >= cfg.MIN_NEW_FRAMES and
                       (time.time() - self._last_window_t) >= cfg.WINDOW_TIMEOUT_S)
            if by_frames or by_time:
                trigger = "frames" if by_frames else "timeout"
                threading.Thread(target=self._run_prism, args=(trigger,),
                                 daemon=True).start()

    def _run_prism(self, trigger: str):
        if self._processing:
            return
        self._processing = True
        t0 = time.time()
        total, lo_seq, hi_seq, _new = self._prism.stats()
        log.info(f"[Server] >> mapping ({trigger}): {total} frames buffered "
                 f"[seq {lo_seq}..{hi_seq}], window={cfg.WINDOW_SIZE} overlap={cfg.OVERLAP}")
        try:
            self._recover_gaps()
            n_sub, version, last_pts = 0, -1, 0
            for r in self._prism.process_new():
                t_iter = time.time()
                version = r.version
                n_full = int(r.points.shape[0])
                xyz, rgb = self._clip_ceiling(r.points, r.colors)
                if xyz.shape[0] == 0:
                    continue
                # DECOUPLE stream density from the TSDF voxel: the mapper stays fine
                # (VOXEL_SIZE) for internal quality, but we voxel-downsample the STREAMED
                # cloud to STREAM_VOXEL_M (centroid → keeps placement) to fit the link.
                if cfg.STREAM_VOXEL_M > cfg.VOXEL_SIZE * 1.001:
                    xyz, rgb = bm.voxel_downsample(xyz, rgb, cfg.STREAM_VOXEL_M)
                # Diff-based block sync from the CURRENT surface → cubes that no
                # longer have points are REMOVED (no accumulation/ghosting).
                n_changed, n_removed, n_cubes, man_bytes, push_bytes = \
                    self._blockpub.ingest_and_publish(xyz, rgb, map_version=version)
                _now = time.time()
                if self._last_submap_t:
                    dt = max(_now - self._last_submap_t, 1e-3)
                    # bytes actually put on the wire this submap = manifest + push
                    self._cloud_mbps = ((man_bytes + push_bytes) / dt) / 1e6
                self._last_submap_t = _now

                if r.trajectory is not None and len(r.trajectory) > 0:
                    traj_send = (r.trajectory[-cfg.TRAJ_MAX_POSES:]
                                 if cfg.TRAJ_MAX_POSES > 0 else r.trajectory)
                    self._pub_traj.put(proto.pack_trajectory(traj_send))
                    self._publish_pose_correction(version, r.cam_pose, r.trajectory, r.cam_ts)

                n_sub += 1
                last_pts = int(xyz.shape[0])
                self._last_window_t = time.time()

                # ── diagnostics: surface size, cube churn, trajectory extent ──
                extent = "—"
                if r.trajectory is not None and len(r.trajectory) > 0:
                    span = r.trajectory.max(axis=0) - r.trajectory.min(axis=0)
                    extent = f"{span[0]:.1f}x{span[1]:.1f}x{span[2]:.1f}m"
                clipped = n_full - last_pts
                self._rx_meter.decay()
                self._publish_status("processing", {
                    "map_version": int(version), "n_points": last_pts,
                    "n_points_full": n_full, "ceiling_clipped": clipped,
                    "ceiling_z": self._ceiling_z,
                    "cubes": n_cubes, "cubes_changed": n_changed, "cubes_removed": n_removed,
                    "manifest_kb": round(man_bytes / 1024, 1),
                    "push_kb": round(push_bytes / 1024, 1), "submap": n_sub,
                    "submap_s": round(time.time() - t_iter, 2),
                    "cloud_mbps": round(self._cloud_mbps, 3),
                    "frames_buffered": total, "trigger": trigger,
                    "seq_gaps": self._gap_count,
                    # ── telemetry for the client metrics window ──
                    "server_send_ns": time.time_ns(),
                    "robot_offset_ms": round((self._clock.offset_s or 0.0) * 1e3, 1),
                    "robot_to_server_ms": round(self._clock.last_latency_s * 1e3, 1),
                    "robot_kbps": round(self._rx_meter.kbps, 1),
                    "robot_fps": round(self._rx_meter.mps, 2),
                    "newest_frame_robot_ns": int(self._last_frame_robot_ns)})
                clip_note = (f" (clipped {clipped} above {self._ceiling_z:.2f}m)"
                             if self._ceiling_z is not None else "")
                log.info(f"[Server] ✓ submap v{version}: surface {last_pts} pts{clip_note} | "
                         f"cubes {n_cubes} (+{n_changed}/-{n_removed}) | traj {extent} | "
                         f"corr={self._corr_gate.last_reason}")
            if n_sub == 0:
                log.info(f"[Server] no new window yet ({total} frames buffered)")
            else:
                log.info(f"[Server] ▣ batch: {n_sub} submap(s) in {time.time()-t0:.2f}s "
                         f"→ v{version}, {last_pts} surface pts | corr published "
                         f"{self._corr_gate.n_published} suppressed {self._corr_gate.n_suppressed} "
                         f"rejected {self._corr_gate.n_rejected}")
        except Exception:
            log.error(f"[Server] PRISM run failed:\n{traceback.format_exc()}")
        finally:
            self._processing = False

    def _publish_pose_correction(self, version, cam_pose, traj_np, cam_ts_s=None):
        pose = camera_pose_from_matrix(cam_pose)
        if pose is None:
            pose = camera_pose_from_trajectory(traj_np)
        if pose is None:
            return
        committed = bool(getattr(self._prism.engine, "_scale_committed", True))
        accepted = self._corr_gate.evaluate(pose[0], pose[1], cam_ts_s, committed)
        if accepted is None:
            return
        pos, quat = accepted
        ts_ns = int(round(cam_ts_s * 1e9)) if cam_ts_s else time.time_ns()
        c = proto.PoseCorrection(timestamp_ns=ts_ns, map_version=int(version),
                                 position=pos, quaternion=quat)
        try:
            self._pub_pose_cor.put(proto.pack_pose_correction(c), encoding=proto.ENC_PCOR)
        except TypeError:
            self._pub_pose_cor.put(proto.pack_pose_correction(c))
        except Exception:
            log.error(f"[Server] pose correction publish failed:\n{traceback.format_exc()}")

    def run(self):
        log.info("[Server] Running. Waiting for PRISM model + frames...")
        self._publish_status("starting")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            log.info("[Server] Shutting down.")
        finally:
            self._z.close()


if __name__ == "__main__":
    MappingServer().run()
