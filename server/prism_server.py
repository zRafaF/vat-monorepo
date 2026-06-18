"""
VAT — PRISM Streaming Server
=============================
Subscribes to equirectangular camera frames published by the robot over Zenoh,
accumulates them into sliding windows, runs the PRISM-VGGT mapping pipeline,
and publishes coloured point-cloud deltas back over Zenoh for any connected client.

Architecture
------------
  robot (Docker: bridge + decimator)  ──Zenoh──►  [this process]  ──Zenoh──►  client
                                                        │
                                                   PRISM-VGGT engine
                                                   (nvblox + PanoVGGT)

Zenoh key schema
----------------
  {ROBOT_NAME}/prism/camera/frame      ← [8B ts_ns int64 LE] + JPEG (from frame_decimator)
  {ROBOT_NAME}/rt/sport_mode_state     ← SportModeState (CDR via bridge) — body height
  server/prism/pcd_snapshot            → full cloud (binary, on queryable)
  server/prism/pcd_delta               → incremental delta per submap
  server/prism/trajectory              → camera trajectory (binary)
  server/prism/status                  → JSON heartbeat

Wire format  (pcd_delta / pcd_snapshot)
----------------------------------------
  Offset  Bytes  Type    Field
  ──────  ─────  ──────  ─────────────────────────────────────────────────
  0       4      int32   magic  = 0x50434400  ("PCD\x00")
  4       4      int32   version        (engine map_version)
  8       4      int32   n_points
  12      4      int32   is_snapshot    (1 = full snapshot, 0 = delta)
  16      4      int32   since_version  (only meaningful for deltas)
  20      n*12   float32 xyz  [n, 3]
  20+n*12 n*12   float32 rgb  [n, 3]  in [0, 1]

Usage
-----
  # minimal
  python server/prism_server.py

  # override config via env vars
  ZENOH_ROUTER=tcp/192.168.1.100:7447 \\
  ROBOT_NAME=go2 \\
  CAMERA_HEIGHT=1.05 \\
  WINDOW_SIZE=10 \\
  python server/prism_server.py
"""

from __future__ import annotations

import os
import json
import time
import struct
import logging
import threading
import traceback
from collections import deque
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import zenoh

# ── rosbags for CDR deserialisation — only used for sport_mode_state ──────────
from rosbags.typesys import Stores, get_typestore

# ── PRISM-VGGT pipeline ───────────────────────────────────────────────────────
# Requires the server/PRISM-VGGT submodule to be checked out and installed:
#   git submodule update --init server/PRISM-VGGT
#   uv sync --package vat-server
os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

from prism_vggt import FrameInput
from prism_vggt.backends.panovggt import PanoVGGTBackend
from prism_vggt.engine import StreamingWindowEngine
from prism_vggt.utils.masking import get_spherical_valid_mask

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("prism-server")

# ─────────────────────────────────────────────────────────────────────────────
# Configuration  (overridable via environment variables)
# ─────────────────────────────────────────────────────────────────────────────

ZENOH_ROUTER   = os.environ.get("ZENOH_ROUTER",   "tcp/127.0.0.1:7447")
ROBOT_NAME     = os.environ.get("ROBOT_NAME",     "go2")
SERVER_PREFIX  = os.environ.get("SERVER_PREFIX",  "server/prism")

# PRISM engine parameters
WEIGHTS_PATH   = os.environ.get("WEIGHTS_PATH",   "server/PRISM-VGGT/checkpoints/model.pt")
VOXEL_SIZE     = float(os.environ.get("VOXEL_SIZE",    "0.02"))
MAX_DEPTH      = float(os.environ.get("MAX_DEPTH",     "4.5"))
FACE_SIZE      = int(os.environ.get("FACE_SIZE",       "512"))
WINDOW_SIZE    = int(os.environ.get("WINDOW_SIZE",     "10"))
OVERLAP        = int(os.environ.get("OVERLAP",         "3"))

# Camera / scene parameters
# TODO (Phase 2): replace with live body_height from SportModeState odometry.
# Camera height = dog body height + mount offset above body centre.
# Body height varies ~0.27–0.35m; mount offset is ~0.18m → ~0.45–0.53m total.
# For the POC we use a fixed value, configurable via env var.
CAMERA_HEIGHT  = float(os.environ.get("CAMERA_HEIGHT",  "0.50"))

# Image pre-processing
TARGET_WIDTH   = int(os.environ.get("TARGET_WIDTH",   "1036"))
TARGET_HEIGHT  = int(os.environ.get("TARGET_HEIGHT",  "518"))
ZENITH_LIMIT   = float(os.environ.get("ZENITH_LIMIT",  "75"))
NADIR_LIMIT    = float(os.environ.get("NADIR_LIMIT",  "-70"))

# ─────────────────────────────────────────────────────────────────────────────
# Zenoh keys
# ─────────────────────────────────────────────────────────────────────────────

KEY_CAMERA_FRAME    = f"{ROBOT_NAME}/prism/camera/frame"   # raw bytes from decimator (no /rt/)
KEY_SPORT_STATE     = f"{ROBOT_NAME}/rt/sport_mode_state"

KEY_PCD_DELTA       = f"{SERVER_PREFIX}/pcd_delta"
KEY_PCD_SNAPSHOT    = f"{SERVER_PREFIX}/pcd_snapshot"
KEY_TRAJECTORY      = f"{SERVER_PREFIX}/trajectory"
KEY_STATUS          = f"{SERVER_PREFIX}/status"

# ─────────────────────────────────────────────────────────────────────────────
# Binary wire format helpers
# ─────────────────────────────────────────────────────────────────────────────

_MAGIC = 0x50434400  # "PCD\x00"
_HEADER_FMT = "!iiiii"   # big-endian: magic, version, n_pts, is_snapshot, since_version
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)


def pack_point_cloud(version: int, xyz: np.ndarray, rgb: np.ndarray,
                     is_snapshot: bool, since_version: int = 0) -> bytes:
    """Serialise a point cloud to the VAT wire format."""
    n = xyz.shape[0]
    header = struct.pack(_HEADER_FMT,
                         _MAGIC, version, n,
                         1 if is_snapshot else 0,
                         since_version)
    xyz_bytes = np.ascontiguousarray(xyz, dtype=np.float32).tobytes()
    rgb_bytes = np.ascontiguousarray(rgb, dtype=np.float32).tobytes()
    return header + xyz_bytes + rgb_bytes


def unpack_point_cloud(data: bytes):
    """Deserialise a VAT wire-format point cloud. Returns (version, xyz, rgb, is_snapshot, since_version)."""
    magic, version, n, is_snap, since_v = struct.unpack_from(_HEADER_FMT, data, 0)
    if magic != _MAGIC:
        raise ValueError(f"Bad magic: 0x{magic:08X}")
    offset = _HEADER_SIZE
    xyz = np.frombuffer(data, dtype=np.float32, count=n * 3, offset=offset).reshape(n, 3)
    rgb = np.frombuffer(data, dtype=np.float32, count=n * 3, offset=offset + n * 12).reshape(n, 3)
    return version, xyz, rgb, bool(is_snap), since_v


def pack_trajectory(positions: np.ndarray) -> bytes:
    """Serialise camera trajectory (N, 3) float32."""
    n = positions.shape[0]
    header = struct.pack("!i", n)
    return header + np.ascontiguousarray(positions, dtype=np.float32).tobytes()

# ─────────────────────────────────────────────────────────────────────────────
# Frame wire format  (produced by robot/frame_decimator.py)
# ─────────────────────────────────────────────────────────────────────────────
# bytes 0–7   int64 little-endian — capture timestamp in nanoseconds
# bytes 8–N   JPEG image data

_FRAME_TS_SIZE = 8  # bytes reserved for the timestamp


def decode_camera_frame(payload: bytes) -> Optional[tuple[int, np.ndarray]]:
    """
    Decode the raw frame payload from frame_decimator.py.

    Returns (timestamp_ns, rgb_array) or None on failure.
    """
    if len(payload) <= _FRAME_TS_SIZE:
        log.warning("Camera frame payload too short")
        return None
    try:
        (ts_ns,) = struct.unpack_from("<q", payload, 0)
        jpeg_bytes = payload[_FRAME_TS_SIZE:]
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            log.warning("JPEG decode returned None")
            return None
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return ts_ns, rgb
    except Exception as e:
        log.warning(f"Camera frame decode failed: {e}")
        return None


# ─────────────────────────────────────────────────────────────────────────────
# SportModeState CDR deserialisation (body height for Phase 2 camera height)
# ─────────────────────────────────────────────────────────────────────────────

_typestore = get_typestore(Stores.ROS2_HUMBLE)


def decode_sport_mode_state(cdr_bytes: bytes) -> Optional[float]:
    """
    Extract body_height from a SportModeState_ CDR payload bridged by
    DynamicZenohBridge.  Returns height in metres or None on failure.

    SportModeState is a Unitree Go2 custom message type; it may or may not be
    present in the standard ROS2 Humble typestore depending on the bridge's
    installed message packages.  If CDR fails, we return None rather than
    crashing — the server falls back to the fixed CAMERA_HEIGHT constant.
    """
    try:
        msg = _typestore.deserialize_cdr(cdr_bytes, "unitree_go/msg/SportModeState")
        return float(msg.body_height)
    except Exception:
        return None

# ─────────────────────────────────────────────────────────────────────────────
# Frame accumulator
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class IncomingFrame:
    image: np.ndarray       # (H, W, 3) uint8 RGB
    mask: np.ndarray        # (H, W) bool
    camera_height: float
    timestamp: float        # unix timestamp


class FrameAccumulator:
    """Thread-safe sliding window of incoming frames."""

    def __init__(self, window_size: int, overlap: int):
        self.window_size = window_size
        self.overlap = overlap
        self._lock = threading.Lock()
        # all frames since last reset; we keep enough for one full window
        self._buffer: deque[IncomingFrame] = deque()
        self._total_received = 0
        # how many frames have already been submitted to a window
        self._submitted = 0

    def push(self, frame: IncomingFrame):
        with self._lock:
            self._buffer.append(frame)
            self._total_received += 1
            # Keep buffer bounded: we only ever need window_size frames at a time
            max_keep = self.window_size + self.overlap
            while len(self._buffer) > max_keep:
                self._buffer.popleft()

    def ready(self) -> bool:
        """True when we have a complete first window, or enough new frames for the next."""
        with self._lock:
            return len(self._buffer) >= self.window_size

    def get_window(self) -> list[IncomingFrame]:
        """Return the latest WINDOW_SIZE frames as a list (oldest first)."""
        with self._lock:
            buf = list(self._buffer)
        return buf[-self.window_size:]

    def drop_oldest(self, n: int):
        """Remove the N oldest frames (call after consuming overlap frames)."""
        with self._lock:
            for _ in range(min(n, len(self._buffer))):
                self._buffer.popleft()

# ─────────────────────────────────────────────────────────────────────────────
# Online PRISM session
# ─────────────────────────────────────────────────────────────────────────────

class OnlinePRISMSession:
    """
    Wraps the StreamingWindowEngine for online frame-by-frame operation.

    Design note (POC limitation)
    ----------------------------
    StreamingWindowEngine.process_sequence() resets all state at the start and
    yields one result per sub-window over the ENTIRE sequence.  True online
    streaming (where each call continues from the previous map state) requires
    a refactor of the engine to expose a `step()` API — this is tracked as
    TODO in docs/streaming_poc.md.

    For the POC we maintain an ever-growing list of FrameInputs and replay the
    full sequence each time a new window is ready.  This is correct but O(N)
    per call.  In practice sessions are bounded (a few hundred frames), so the
    overhead stays manageable.  The A/B parallel inference pipeline still
    overlaps work across sub-windows, so the LAST submap (which is the new
    one) comes out quickly.
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
            perception=self.perception,
            voxel_size=VOXEL_SIZE,
            max_depth=MAX_DEPTH,
            face_size=FACE_SIZE,
        )
        self.engine.compute_esdf = False   # skip ESDF in streaming mode (save ~80ms)
        self.engine.point_cloud_only = True
        log.info("[PRISM] Engine ready.")

        self._all_frames: list[FrameInput] = []
        self._last_processed_count = 0
        self._lock = threading.Lock()

    def add_frame(self, frame: IncomingFrame):
        fi = FrameInput(
            image=frame.image,
            mask=frame.mask,
            camera_height=frame.camera_height,
            timestamp=frame.timestamp,
        )
        with self._lock:
            self._all_frames.append(fi)

    def has_new_window(self) -> bool:
        """True when we have enough new frames to form the next sub-window."""
        with self._lock:
            n = len(self._all_frames)
        new_frames_needed = WINDOW_SIZE - OVERLAP  # frames that advance the window
        if self._last_processed_count == 0:
            return n >= WINDOW_SIZE
        return (n - self._last_processed_count) >= new_frames_needed

    def run_until_latest(self):
        """
        Run process_sequence over all accumulated frames and return the LAST
        submap's point cloud — the one that includes the newest observations.

        Returns (pcd_snapshot_dict, trajectory_np) or None if not enough frames.
        """
        with self._lock:
            frames = list(self._all_frames)

        if len(frames) < WINDOW_SIZE:
            return None

        last_pcd_dict = None
        last_traj = None

        try:
            for _mesh, pcd, traj, _plane in self.engine.process_sequence(
                frames, window_size=WINDOW_SIZE, overlap=OVERLAP
            ):
                # Collect every submap but only publish the last one below
                if pcd is not None and len(pcd.points) > 0:
                    last_pcd_dict = {
                        "snapshot": self.engine.get_point_cloud_snapshot(),
                        "version":  self.engine.get_map_version(),
                    }
                    last_traj = traj.copy() if traj is not None else None
        except Exception:
            log.error(f"[PRISM] Engine error:\n{traceback.format_exc()}")
            return None

        self._last_processed_count = len(frames)
        return last_pcd_dict, last_traj

# ─────────────────────────────────────────────────────────────────────────────
# Main server
# ─────────────────────────────────────────────────────────────────────────────

class PRISMServer:
    def __init__(self):
        self._camera_height = CAMERA_HEIGHT
        self._accumulator = FrameAccumulator(WINDOW_SIZE, OVERLAP)
        self._session_lock = threading.Lock()
        self._processing = False

        # Mask is constant for fixed resolution
        self._mask = get_spherical_valid_mask(
            TARGET_HEIGHT, TARGET_WIDTH,
            zenith_deg=ZENITH_LIMIT,
            nadir_deg=NADIR_LIMIT,
        )

        log.info(f"[Server] Connecting to Zenoh at {ZENOH_ROUTER}...")
        conf = zenoh.Config()
        conf.insert_json5("connect/endpoints", f'["{ZENOH_ROUTER}"]')
        conf.insert_json5("mode", '"client"')
        self._z = zenoh.open(conf)
        log.info("[Server] Zenoh connected.")

        # Publishers
        self._pub_delta    = self._z.declare_publisher(KEY_PCD_DELTA,
            congestion_control=zenoh.CongestionControl.BLOCK)
        self._pub_snapshot = self._z.declare_publisher(KEY_PCD_SNAPSHOT,
            congestion_control=zenoh.CongestionControl.BLOCK)
        self._pub_traj     = self._z.declare_publisher(KEY_TRAJECTORY,
            congestion_control=zenoh.CongestionControl.DROP)
        self._pub_status   = self._z.declare_publisher(KEY_STATUS,
            congestion_control=zenoh.CongestionControl.DROP)

        # Lazy-init PRISM session (heavy: loads model weights)
        self._prism: Optional[OnlinePRISMSession] = None
        self._prism_init_thread = threading.Thread(
            target=self._init_prism, daemon=True)
        self._prism_init_thread.start()

        # Subscribers
        self._z.declare_subscriber(KEY_CAMERA_FRAME, self._on_camera_frame)
        self._z.declare_subscriber(KEY_SPORT_STATE,  self._on_sport_state)

        # Queryable: clients can request the full snapshot at any time
        self._z.declare_queryable(KEY_PCD_SNAPSHOT, self._on_snapshot_query)

        log.info(f"[Server] Subscribed to camera frames on '{KEY_CAMERA_FRAME}'")
        log.info(f"[Server] Streaming deltas on '{KEY_PCD_DELTA}'")

    def _init_prism(self):
        try:
            self._prism = OnlinePRISMSession(WEIGHTS_PATH)
            self._publish_status("ready")
        except Exception:
            log.error(f"[Server] Failed to init PRISM session:\n{traceback.format_exc()}")
            self._publish_status("error")

    def _publish_status(self, state: str, extra: dict | None = None):
        payload = {"state": state, "ts": time.time()}
        if extra:
            payload.update(extra)
        try:
            self._pub_status.put(json.dumps(payload).encode())
        except Exception:
            pass

    # ── Zenoh callbacks ──────────────────────────────────────────────────────

    def _on_camera_frame(self, sample):
        """
        Receive a frame from the robot's frame_decimator.

        Payload format (produced by robot/frame_decimator.py):
          bytes 0–7:  int64 LE — capture timestamp in nanoseconds
          bytes 8–N:  JPEG image data
        """
        try:
            result = decode_camera_frame(bytes(sample.payload))
            if result is None:
                return
            ts_ns, rgb = result

            # Resize to canonical PRISM resolution if needed
            if rgb.shape[1] != TARGET_WIDTH or rgb.shape[0] != TARGET_HEIGHT:
                rgb = cv2.resize(rgb, (TARGET_WIDTH, TARGET_HEIGHT),
                                 interpolation=cv2.INTER_AREA)

            # Convert nanoseconds to float seconds for FrameInput
            frame = IncomingFrame(
                image=rgb,
                mask=self._mask.copy(),
                camera_height=self._camera_height,
                timestamp=ts_ns * 1e-9,
            )
            self._accumulator.push(frame)
            log.debug(f"[Server] Frame received ts={ts_ns//1_000_000}ms "
                      f"buf={len(self._accumulator._buffer)}")

            # Kick off processing in a background thread if PRISM is ready
            if self._prism is not None:
                self._prism.add_frame(frame)
                if not self._processing and self._prism.has_new_window():
                    threading.Thread(target=self._run_prism, daemon=True).start()

        except Exception:
            log.error(f"[Server] Error in frame callback:\n{traceback.format_exc()}")

    def _on_sport_state(self, sample):
        """
        Update camera height from the robot's body odometry.

        SportModeState.body_height is the measured height of the body centre
        above the ground.  Camera height = body_height + CAMERA_MOUNT_OFFSET.

        The camera mount offset is the physical distance from the dog's body
        centre to the Insta360 optical centre.  Measure this from your CAD
        model or physically.  Default 0.18m is a rough estimate for the Go2.
        """
        CAMERA_MOUNT_OFFSET = float(os.environ.get("CAMERA_MOUNT_OFFSET", "0.18"))
        try:
            body_height = decode_sport_mode_state(bytes(sample.payload))
            if body_height is not None and body_height > 0.1:
                self._camera_height = body_height + CAMERA_MOUNT_OFFSET
                log.debug(f"[Server] Camera height updated: {self._camera_height:.3f}m "
                          f"(body={body_height:.3f}m + offset={CAMERA_MOUNT_OFFSET:.3f}m)")
        except Exception:
            pass

    def _on_snapshot_query(self, query):
        """Reply to an explicit snapshot request from a client."""
        if self._prism is None:
            query.reply(query.key_expr, b"")
            return
        snap = self._prism.engine.get_point_cloud_snapshot()
        xyz = snap["points"]
        rgb = snap["colors"]
        version = snap["version"]
        if xyz.shape[0] == 0:
            query.reply(query.key_expr, b"")
            return
        payload = pack_point_cloud(version, xyz, rgb, is_snapshot=True)
        query.reply(query.key_expr, payload)

    # ── PRISM processing ─────────────────────────────────────────────────────

    def _run_prism(self):
        if self._processing:
            return
        self._processing = True
        t0 = time.time()
        try:
            result = self._prism.run_until_latest()
            if result is None:
                return
            pcd_dict, traj = result
            if pcd_dict is None:
                return

            snap = pcd_dict["snapshot"]
            version = pcd_dict["version"]
            xyz = snap["points"]
            rgb = snap["colors"]

            if xyz.shape[0] == 0:
                return

            # Publish full snapshot (clients use this for initial sync or resync)
            snap_payload = pack_point_cloud(version, xyz, rgb, is_snapshot=True)
            self._pub_snapshot.put(snap_payload)

            # Also publish a delta for incremental clients
            # (For the POC we just re-send the snapshot on the delta key too;
            # proper delta publishing requires engine.get_point_cloud_delta())
            delta = self._prism.engine.get_point_cloud_delta(version - 1)
            d_xyz = delta["points"]
            d_rgb = delta["colors"]
            if d_xyz.shape[0] > 0:
                delta_payload = pack_point_cloud(
                    version, d_xyz, d_rgb,
                    is_snapshot=False,
                    since_version=version - 1,
                )
                self._pub_delta.put(delta_payload)

            # Trajectory
            if traj is not None and len(traj) > 0:
                traj_np = np.asarray(traj, dtype=np.float32)
                self._pub_traj.put(pack_trajectory(traj_np))

            elapsed = time.time() - t0
            self._publish_status("processing", {
                "map_version": version,
                "n_points": int(xyz.shape[0]),
                "elapsed_s": round(elapsed, 2),
            })
            log.info(f"[Server] ✓ Submap v{version}: {xyz.shape[0]} pts "
                     f"| {elapsed:.2f}s | delta={d_xyz.shape[0]} pts")

        except Exception:
            log.error(f"[Server] PRISM run failed:\n{traceback.format_exc()}")
        finally:
            self._processing = False

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def run(self):
        log.info("[Server] Running. Waiting for PRISM model to load and frames to arrive...")
        self._publish_status("starting")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            log.info("[Server] Shutting down.")
        finally:
            self._z.close()


# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    server = PRISMServer()
    server.run()
