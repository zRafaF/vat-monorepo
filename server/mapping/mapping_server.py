"""
VAT — Mapping Server
====================
The cloud-side mapping & routing process.  (PRISM-VGGT is the *perception
backend* it drives — this process is the server, not "PRISM".)

It subscribes to decimated camera frames from the robot, batches them into
sliding windows, runs the PRISM-VGGT pipeline to build a coloured point cloud,
and publishes:

  * point-cloud snapshots/deltas to the client, and
  * the low-frequency **VGGT camera-pose correction** DOWN to the robot.

The robot — not this server — is authoritative for the robot pose: the server
emits a camera-pose correction; the robot fuses it with onboard odometry and
the router relays the result to the client.

Frame reliability (every frame matters for pose quality)
--------------------------------------------------------
Frames arrive reliably (the decimator publishes with RELIABLE/BLOCK) and carry
a monotonic ``seq``.  The server:
  * detects ``seq`` gaps and **re-requests** the missing frames from the
    decimator's queryable (``{robot}/prism/camera/frame/get?seq=N``) before
    processing, and
  * batches a window when **either** enough new frames have arrived **or** a
    timeout elapses (whichever comes first) — so a stalled/sparse stream never
    leaves the client hanging.

Zenoh keys + wire formats: see common/vat_protocol.py.
"""

from __future__ import annotations

import os
import sys
import json
import time
import logging
import threading
import traceback
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import zenoh

# repo root is three levels up: server/mapping/mapping_server.py → repo/common
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_REPO_ROOT, "common"))
import vat_protocol as proto  # noqa: E402

os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

from prism_vggt import FrameInput  # noqa: E402
from prism_vggt.backends.panovggt import PanoVGGTBackend  # noqa: E402
from prism_vggt.engine import StreamingWindowEngine  # noqa: E402
from prism_vggt.utils.masking import get_spherical_valid_mask  # noqa: E402

logging.basicConfig(
    level=os.environ.get("LOG_LEVEL", "INFO").upper(),
    format="[%(asctime)s] [%(levelname)s] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("mapping-server")

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────

ZENOH_ROUTER  = os.environ.get("ZENOH_ROUTER",  "tcp/127.0.0.1:7447")
ROBOT_NAME    = os.environ.get("ROBOT_NAME",    "go2")
SERVER_PREFIX = os.environ.get("SERVER_PREFIX", "server/prism")

# Default relative to THIS file (the submodule sits next to it), so the server
# runs correctly no matter the working directory (e.g. `cd server/mapping`).
_HERE = os.path.dirname(os.path.abspath(__file__))
WEIGHTS_PATH  = os.environ.get("WEIGHTS_PATH",
                               os.path.join(_HERE, "PRISM-VGGT/checkpoints/model.pt"))
VOXEL_SIZE    = float(os.environ.get("VOXEL_SIZE", "0.02"))
MAX_DEPTH     = float(os.environ.get("MAX_DEPTH",  "4.5"))
FACE_SIZE     = int(os.environ.get("FACE_SIZE",    "512"))
WINDOW_SIZE   = int(os.environ.get("WINDOW_SIZE",  "10"))
OVERLAP       = int(os.environ.get("OVERLAP",      "3"))

# Batching: process when N new frames arrive OR this many seconds elapse.
WINDOW_TIMEOUT_S = float(os.environ.get("WINDOW_TIMEOUT_S", "2.0"))
MIN_NEW_FRAMES   = int(os.environ.get("MIN_NEW_FRAMES", "1"))

# Frame-drop recovery.
RETRY_TIMEOUT_S    = float(os.environ.get("RETRY_TIMEOUT_S", "0.3"))
MAX_RETRIES_CYCLE  = int(os.environ.get("MAX_RETRIES_CYCLE", str(WINDOW_SIZE)))

CAMERA_HEIGHT = float(os.environ.get("CAMERA_HEIGHT", "0.50"))  # fallback only

TARGET_WIDTH  = int(os.environ.get("TARGET_WIDTH",  "1036"))
TARGET_HEIGHT = int(os.environ.get("TARGET_HEIGHT", "518"))
ZENITH_LIMIT  = float(os.environ.get("ZENITH_LIMIT", "75"))
NADIR_LIMIT   = float(os.environ.get("NADIR_LIMIT", "-70"))

_KEYS = proto.keys(ROBOT_NAME, SERVER_PREFIX)


@dataclass
class IncomingFrame:
    image: np.ndarray
    mask: np.ndarray
    camera_height: float
    timestamp: float


# ─────────────────────────────────────────────────────────────────────────────
# Online PRISM session  (frames keyed by seq so gaps can be filled in order)
# ─────────────────────────────────────────────────────────────────────────────


class OnlinePRISMSession:
    """Wraps StreamingWindowEngine for online operation.

    StreamingWindowEngine.process_sequence() resets state per call, so for the
    POC we replay the accumulated (seq-ordered) frame list each time a window is
    ready.  Keying by seq lets us insert re-requested frames in the right place.
    """

    def __init__(self, weights_path: str):
        log.info("[PRISM] Loading PanoVGGT perception backend...")
        self.perception = PanoVGGTBackend(
            config_path=os.path.join(os.path.dirname(weights_path),
                                     "..", "third_party", "PanoVGGT",
                                     "training", "config", "default.yaml"),
            weights_path=weights_path,
        )
        self.engine = StreamingWindowEngine(
            perception=self.perception, voxel_size=VOXEL_SIZE,
            max_depth=MAX_DEPTH, face_size=FACE_SIZE)
        self.engine.compute_esdf = False
        self.engine.point_cloud_only = True
        log.info("[PRISM] Engine ready.")

        self._frames: dict[int, FrameInput] = {}
        self._lock = threading.Lock()
        self._last_processed_count = 0

    def add_frame(self, seq: int, frame: IncomingFrame) -> bool:
        """Store a frame by seq.  Returns True if it was new."""
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
            seqs = self._frames.keys()
            return (len(self._frames), min(seqs), max(seqs),
                    len(self._frames) - self._last_processed_count)

    def missing_seqs(self, lo: int, hi: int) -> list[int]:
        with self._lock:
            return [s for s in range(lo, hi + 1) if s not in self._frames]

    def run_until_latest(self):
        with self._lock:
            n = len(self._frames)
            frames = [self._frames[s] for s in sorted(self._frames)]
        if n < WINDOW_SIZE:
            return None

        last_pcd_dict, last_traj = None, None
        try:
            for _mesh, pcd, traj, _plane in self.engine.process_sequence(
                    frames, window_size=WINDOW_SIZE, overlap=OVERLAP):
                if pcd is not None and len(pcd.points) > 0:
                    last_pcd_dict = {"snapshot": self.engine.get_point_cloud_snapshot(),
                                     "version": self.engine.get_map_version()}
                    last_traj = traj.copy() if traj is not None else None
        except Exception:
            log.error(f"[PRISM] Engine error:\n{traceback.format_exc()}")
            return None

        with self._lock:
            self._last_processed_count = n
        return last_pcd_dict, last_traj


def _camera_pose_from_trajectory(traj: np.ndarray):
    """Best-effort camera pose for the correction (position + heading-from-tangent
    orientation placeholder).  TODO: use true VGGT per-keyframe extrinsics."""
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


# ─────────────────────────────────────────────────────────────────────────────
# Mapping server
# ─────────────────────────────────────────────────────────────────────────────


class MappingServer:
    def __init__(self):
        self._processing = False
        self._last_window_t = time.time()
        self._max_seq_seen = -1
        self._gap_count = 0
        self._mask = get_spherical_valid_mask(
            TARGET_HEIGHT, TARGET_WIDTH, zenith_deg=ZENITH_LIMIT, nadir_deg=NADIR_LIMIT)

        log.info(f"[Server] Connecting to Zenoh at {ZENOH_ROUTER}...")
        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", f'["{ZENOH_ROUTER}"]')
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

        try:
            self._live = self._z.liveliness().declare_token(_KEYS["live_server"])
        except Exception:
            self._live = None

        self._prism: Optional[OnlinePRISMSession] = None
        threading.Thread(target=self._init_prism, daemon=True).start()

        self._z.declare_subscriber(_KEYS["camera_frame"], self._on_camera_frame)
        self._z.declare_queryable(_KEYS["pcd_snapshot"], self._on_snapshot_query)

        # Batching driver: N-frames-or-timeout, whichever first.
        threading.Thread(target=self._batch_loop, daemon=True).start()

        log.info(f"[Server] frames←'{_KEYS['camera_frame']}'  "
                 f"pcd→'{_KEYS['pcd_delta']}'  correction→'{_KEYS['pose_correction']}'  "
                 f"batch: {WINDOW_SIZE-OVERLAP} new frames OR {WINDOW_TIMEOUT_S}s")

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
            self._prism = OnlinePRISMSession(WEIGHTS_PATH)
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

    # ── frame decode ────────────────────────────────────────────────────────

    def _frame_from_payload(self, payload: bytes):
        """Decode a VAT frame payload → (seq, IncomingFrame) or None."""
        ts_ns, seq, cam_h, jpeg = proto.unpack_frame(payload)
        arr = np.frombuffer(jpeg, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        if rgb.shape[1] != TARGET_WIDTH or rgb.shape[0] != TARGET_HEIGHT:
            rgb = cv2.resize(rgb, (TARGET_WIDTH, TARGET_HEIGHT), interpolation=cv2.INTER_AREA)
        height = cam_h if cam_h > 0 else CAMERA_HEIGHT
        return seq, IncomingFrame(image=rgb, mask=self._mask.copy(),
                                  camera_height=height, timestamp=ts_ns * 1e-9)

    def _on_camera_frame(self, sample):
        try:
            decoded = self._frame_from_payload(bytes(sample.payload))
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
        self._prism.add_frame(seq, frame)
        # gap detection (recovery happens in the batch loop before processing)
        if self._max_seq_seen >= 0 and seq > self._max_seq_seen + 1:
            self._gap_count += seq - self._max_seq_seen - 1
            log.warning(f"[Server] seq gap: jumped {self._max_seq_seen}→{seq} "
                        f"(will attempt retransmit before next window)")
        self._max_seq_seen = max(self._max_seq_seen, seq)

    def _on_snapshot_query(self, query):
        try:
            if self._prism is None:
                query.reply(query.key_expr, b""); return
            snap = self._prism.engine.get_point_cloud_snapshot()
            xyz, rgb, version = snap["points"], snap["colors"], snap["version"]
            if xyz.shape[0] == 0:
                query.reply(query.key_expr, b""); return
            query.reply(query.key_expr, proto.pack_pcd(version, xyz, rgb, is_snapshot=True))
        except Exception:
            log.error(f"[Server] snapshot query error:\n{traceback.format_exc()}")

    # ── frame-drop recovery ───────────────────────────────────────────────────

    def _recover_gaps(self):
        """Re-request any missing seqs in the recent window region from the
        decimator's queryable, so the window we're about to process is complete."""
        total, lo_seq, hi_seq, _new = self._prism.stats()
        if total == 0 or hi_seq is None:
            return
        lo = max(lo_seq, hi_seq - WINDOW_SIZE * 2)
        missing = self._prism.missing_seqs(lo, hi_seq)[:MAX_RETRIES_CYCLE]
        for seq in missing:
            try:
                sel = f"{_KEYS['camera_frame_get']}?seq={seq}"
                got = False
                for reply in self._z.get(sel, timeout=RETRY_TIMEOUT_S):
                    if reply.ok:
                        data = bytes(reply.result.payload)
                        if len(data) > 20:
                            decoded = self._frame_from_payload(data)
                            if decoded is not None:
                                self._prism.add_frame(*decoded)
                                got = True
                if got:
                    log.info(f"[Server] recovered dropped frame seq={seq}")
                else:
                    log.warning(f"[Server] could not recover seq={seq} (no longer buffered)")
            except Exception as e:
                log.warning(f"[Server] retransmit query failed seq={seq}: {e}")

    # ── batching driver ───────────────────────────────────────────────────────

    def _batch_loop(self):
        while True:
            time.sleep(0.1)
            if self._prism is None or self._processing:
                continue
            total, _lo, _hi, new = self._prism.stats()
            if total < WINDOW_SIZE:
                continue
            by_frames = new >= (WINDOW_SIZE - OVERLAP)
            by_time = (new >= MIN_NEW_FRAMES and
                       (time.time() - self._last_window_t) >= WINDOW_TIMEOUT_S)
            if by_frames or by_time:
                trigger = "frames" if by_frames else "timeout"
                threading.Thread(target=self._run_prism, args=(trigger,),
                                 daemon=True).start()

    def _run_prism(self, trigger: str):
        if self._processing:
            return
        self._processing = True
        t0 = time.time()
        try:
            self._recover_gaps()           # fill dropped frames before processing
            result = self._prism.run_until_latest()
            self._last_window_t = time.time()
            if not result:
                return
            pcd_dict, traj = result
            if pcd_dict is None:
                return
            snap = pcd_dict["snapshot"]
            version = pcd_dict["version"]
            xyz, rgb = snap["points"], snap["colors"]
            if xyz.shape[0] == 0:
                return

            self._pub_snapshot.put(proto.pack_pcd(version, xyz, rgb, is_snapshot=True))

            delta = self._prism.engine.get_point_cloud_delta(version - 1)
            d_xyz, d_rgb = delta["points"], delta["colors"]
            if d_xyz.shape[0] > 0:
                self._pub_delta.put(proto.pack_pcd(
                    version, d_xyz, d_rgb, is_snapshot=False, since_version=version - 1))

            if traj is not None and len(traj) > 0:
                traj_np = np.asarray(traj, dtype=np.float32)
                self._pub_traj.put(proto.pack_trajectory(traj_np))
                self._publish_pose_correction(version, traj_np)

            elapsed = time.time() - t0
            self._publish_status("processing", {
                "map_version": version, "n_points": int(xyz.shape[0]),
                "elapsed_s": round(elapsed, 2), "trigger": trigger,
                "seq_gaps": self._gap_count})
            log.info(f"[Server] ✓ submap v{version} ({trigger}): {xyz.shape[0]} pts | "
                     f"{elapsed:.2f}s | delta={d_xyz.shape[0]} | gaps={self._gap_count}")
        except Exception:
            log.error(f"[Server] PRISM run failed:\n{traceback.format_exc()}")
        finally:
            self._processing = False

    def _publish_pose_correction(self, version, traj_np):
        pose = _camera_pose_from_trajectory(traj_np)
        if pose is None:
            return
        pos, quat = pose
        c = proto.PoseCorrection(timestamp_ns=time.time_ns(), map_version=int(version),
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
