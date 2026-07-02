"""
VAT mapping server — configuration & Zenoh keys (single source of truth).

Importing this module also puts the repo's ``common/`` on ``sys.path`` so every
mapping-server module can ``import vat_protocol``. Keep ALL tunables here so the
behaviour of the server is configured in one obvious place rather than scattered
through a 700-line script.
"""

from __future__ import annotations

import os
import sys

# repo root is two levels up from server/mapping/ → repo/common
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_HERE))
_COMMON = os.path.join(_REPO_ROOT, "common")
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)

import vat_protocol as proto  # noqa: E402  (needs the path insert above)

# ── Zenoh / identity ─────────────────────────────────────────────────────────
ZENOH_ROUTER  = os.environ.get("ZENOH_ROUTER",  "tcp/127.0.0.1:7447")
ROBOT_NAME    = os.environ.get("ROBOT_NAME",    "go2")
SERVER_PREFIX = os.environ.get("SERVER_PREFIX", "server/prism")

KEYS = proto.keys(ROBOT_NAME, SERVER_PREFIX)
RESET_KEY = f"{SERVER_PREFIX}/cmd/reset"

# ── PRISM engine ─────────────────────────────────────────────────────────────
WEIGHTS_PATH = os.environ.get(
    "WEIGHTS_PATH", os.path.join(_HERE, "PRISM-VGGT/checkpoints/model.pt"))
VOXEL_SIZE  = float(os.environ.get("VOXEL_SIZE", "0.03"))
MAX_DEPTH   = float(os.environ.get("MAX_DEPTH",  "4.5"))
FACE_SIZE   = int(os.environ.get("FACE_SIZE",    "512"))
WINDOW_SIZE = int(os.environ.get("WINDOW_SIZE",  "12"))
OVERLAP     = int(os.environ.get("OVERLAP",      "4"))
PROCESSING_MODE = os.environ.get("PROCESSING_MODE", "parallel").strip().lower()


def _parse_ceiling(raw: str):
    """Ceiling height (world Z, m): points above it are dropped before sending.
    Empty / 'off' / 'none' / non-finite ⇒ disabled (whole cloud sent)."""
    s = (raw or "").strip().lower()
    if s in ("", "off", "none", "disable", "disabled", "inf", "+inf"):
        return None
    try:
        v = float(s)
        return v if v == v and abs(v) != float("inf") else None
    except ValueError:
        return None


# Ceiling-plane clip (toy-box view + bandwidth). Live-settable over Zenoh on
# CEILING_KEY; startup default from CEILING_Z. None = send the whole cloud.
CEILING_KEY = f"{SERVER_PREFIX}/config/ceiling_z"
CEILING_Z = _parse_ceiling(os.environ.get("CEILING_Z", ""))

# ── Cloud delivery ───────────────────────────────────────────────────────────
# Streaming transport for the map point cloud:
#   "snapshot" (default) — publish the WHOLE current TSDF surface, coarse-voxelised to
#       STREAM_VOXEL_M, as one compressed pack_pcd snapshot per submap (pcd_snapshot).
#       The client replaces its cloud wholesale, so nothing accumulates and there is no
#       manifest/diff/pull round-trip. Pairs with reset-each-batch (the map is already
#       rebuilt whole + bounded each batch, so "just send the whole map" is the simplest
#       and lowest-latency path). Keep the map fine (VOXEL_SIZE) and stream coarse.
#   "blocks" — DEPRECATED diff-based cube sync (manifest + push + Draco pull). Kept for
#       A/B on the rig; slated for removal once snapshot mode is validated.
STREAM_MODE = os.environ.get("STREAM_MODE", "snapshot").strip().lower()
# Cap points in the STREAMED snapshot (0 = no cap); full-res still via the query.
CLOUD_STREAM_MAX_POINTS = int(os.environ.get("CLOUD_STREAM_MAX_POINTS", "60000"))
CUBE_SIZE = float(os.environ.get("CUBE_SIZE", "1.0"))

# ── Streaming stability (online ghost / breathing / bandwidth fixes) ─────────
# Occupancy-CRC grid for block versioning (m). ~½ voxel so sub-voxel nvblox mesh
# "breathing" doesn't churn the diff (which used to resend ~every cube each submap).
# Stream decimation: the LIVE cloud is voxel-downsampled to this (m) before it is
# sent, DECOUPLED from the TSDF voxel — keep the mapper fine (VOXEL_SIZE, default 3cm)
# for reconstruction quality while streaming COARSER to fit the link and keep the
# per-submap snapshot small. Default 5cm ≈ ⅓ fewer points than the 3cm map for a
# barely-visible quality drop. Set = VOXEL_SIZE for no decoupling (finest stream).
STREAM_VOXEL_M = float(os.environ.get("STREAM_VOXEL_M", "0.05"))
# How to decimate the streamed cloud (see common/vat_decimate.py):
#   none | voxel_centroid (default, deterministic, keeps placement) |
#   voxel_center (deterministic, snapped) | stride (fast, NOT deterministic → CRC churn)
STREAM_DECIMATE_MODE = os.environ.get("STREAM_DECIMATE_MODE", "voxel_centroid").strip().lower()
STREAM_STRIDE = int(os.environ.get("STREAM_STRIDE", "3"))   # for stride mode
# Occupancy-CRC grid (m), tied to the STREAMED density (~½ the stream voxel) so it
# matches what actually goes on the wire.
CRC_QUANT_M = float(os.environ.get("CRC_QUANT_M", str(max(STREAM_VOXEL_M, VOXEL_SIZE) * 0.5)))
# Emit the streamed surface as one point per voxel CENTRE (byte-identical across
# submaps for unchanged geometry → stable CRCs). 1 = on (recommended online).
CLOUD_VOXEL_SNAP = os.environ.get("CLOUD_VOXEL_SNAP", "1") == "1"
# Keyframe gating: only integrate a frame into the TSDF if the camera moved enough
# since the last integrated frame. Stops re-integrating a static scene (the cause of
# breathing + ghost thickening + the "+119 cubes while stationary" churn). 0 = off.
KEYFRAME_MIN_TRANS_M = float(os.environ.get("KEYFRAME_MIN_TRANS_M", "0.05"))
KEYFRAME_MIN_ROT_DEG = float(os.environ.get("KEYFRAME_MIN_ROT_DEG", "8.0"))
# Time escape: integrate at least this often (s) even when the robot is still, so a
# 360 camera keeps re-observing and DYNAMIC changes (moved/new objects) are caught and
# decayed in instead of the map freezing on first sight. 0 disables the escape.
KEYFRAME_MAX_INTERVAL_S = float(os.environ.get("KEYFRAME_MAX_INTERVAL_S", "1.0"))
# nvblox TSDF decay (active carving of stale/unsupported voxels). OFF by default —
# enable + tune on the rig once the decay API is confirmed for your nvblox build.
TSDF_DECAY = os.environ.get("TSDF_DECAY", "1") == "1"
# Apply decay every N integrated submaps. Higher N = longer point "lifetime" (gentler
# sliding window, geometry persists longer); N=1 decays every submap. Lets you trade
# drift-resistance vs map completeness without touching the nvblox decay rate.
# For SoTA benchmarks set TSDF_DECAY=0 → full accumulated scene, no sliding window.
DECAY_EVERY_N = int(os.environ.get("DECAY_EVERY_N", "1"))
# Observation-TTL on the streamed cube map (nav sliding window, client-side clearing).
# Cubes the camera leaves behind and does NOT re-observe within this many submaps are
# dropped from the streamed grid → they leave the manifest/push as `removed` → the
# client clears them (no whole-map resend). A revisit re-observes and re-adds them.
# This is what makes stale voxels actually disappear on the client; it complements the
# TSDF decay (which carves the volume) by bounding the STREAMED map. 0 = off (keep all).
MAP_TTL_SUBMAPS = int(os.environ.get("MAP_TTL_SUBMAPS", "0"))
# A cube counts as "observed" this submap if its centre is within this radius (m) of any
# of the submap's camera positions. Defaults to the sensor range (MAX_DEPTH): you can
# only refresh what the 360° camera could actually see. Empty/0 → MAX_DEPTH.
MAP_TTL_RADIUS_M = float(os.environ.get("MAP_TTL_RADIUS_M", "0") or 0.0) or MAX_DEPTH
# nvblox TSDF prune radius (m): clear the nvblox volume outside this sphere around the
# robot each submap, so the ESDF used for NAVIGATION only reflects a bounded, recent
# local map — and the mesh-pull cost (the live-latency driver) stays constant as the
# robot travels. Note this bounds the TSDF itself, so the streamed/viewer surface is
# also limited to this radius (the viewer shows a moving local bubble). Uses nvblox's
# native radius-clear if available, else decay+deallocate. Supersedes TSDF_DECAY.
# 0 = OFF → full accumulation (use 0 for SoTA benchmarks against other methods).
TSDF_PRUNE_RADIUS_M = float(os.environ.get("TSDF_PRUNE_RADIUS_M", "0") or 0.0)
# PRIMARY MODE (default ON): rebuild a FRESH map each batch (engine reset=True) over
# only the most recent RESET_WINDOW_FRAMES frames, instead of accumulating online.
# This is the "rolling mini-gradio": each batch is one internally-consistent
# reconstruction, so cross-batch pose drift and revisit ghosts never accumulate (the
# thick/duplicated walls) and the map stays bounded → constant live latency. The
# fresh clouds are re-anchored into ONE persistent world frame (RESET_WORLD_ANCHOR).
# Reset is done IN PLACE via mapper.clear() (see SOFT_RESET) so it is cheap.
# 0 = DEPRECATED online-accumulate path (kept behind the flag for A/B on the rig).
PRISM_RESET_EACH_BATCH = os.environ.get("PRISM_RESET_EACH_BATCH", "1") == "1"
RESET_WINDOW_FRAMES = int(os.environ.get("RESET_WINDOW_FRAMES", "60"))
# Soft reset: wipe the nvblox volume + color cache IN PLACE (mapper.clear()) on each
# per-batch reset instead of reconstructing the Mapper (CUDA re-alloc + block-hash
# regrow — the reset-latency cost). 1 = on (recommended). 0 = full reconstruct.
SOFT_RESET = os.environ.get("SOFT_RESET", "1") == "1"
# Recompute the ESDF each submap so the navigation stack can query collision distances
# (nav_esdf publishes a world-frame slice; see NAV_ESDF_* below). Bounded + cheap in
# reset mode because the map is a small local window. 0 = skip ESDF (viewer-only).
COMPUTE_ESDF = os.environ.get("COMPUTE_ESDF", "1") == "1"
# Re-anchor each fresh reset reconstruction into ONE persistent world frame (rigid
# SE3, from frames shared with the previous batch). Stops the cloud/robot rotating &
# jumping between batches and collapses the delta to the frontier. 1 = on (recommended
# whenever PRISM_RESET_EACH_BATCH=1).
RESET_WORLD_ANCHOR = os.environ.get("RESET_WORLD_ANCHOR", "1") == "1"
# Cap the camera trajectory streamed to the viewer to the last N poses (the full
# trajectory grows without bound and is re-sent each submap). 0 = send all.
TRAJ_MAX_POSES = int(os.environ.get("TRAJ_MAX_POSES", "300"))
# 1 = full snapshot every submap (viewer replaces wholesale → always aligned).
PCD_KEYFRAME_EVERY = int(os.environ.get("PCD_KEYFRAME_EVERY", "1"))

# ── Batching / frame-drop recovery ───────────────────────────────────────────
WINDOW_TIMEOUT_S  = float(os.environ.get("WINDOW_TIMEOUT_S", "5.0"))
# Backlog guard: if the server falls more than this many frames behind real time
# (processing slower than capture), drop the stale backlog and resync to the most
# recent frames so latency self-corrects instead of running away. 0 disables.
BACKLOG_MAX_FRAMES  = int(os.environ.get("BACKLOG_MAX_FRAMES", "45"))
BACKLOG_KEEP_FRAMES = int(os.environ.get("BACKLOG_KEEP_FRAMES", "36"))
# Backlog-resync display continuity: on a resync the live (nav) TSDF rebuilds clean,
# but the viewer keeps the last colored surface as a DISPLAY-ONLY base so it never
# blanks out. The base never feeds the live TSDF/ESDF (no stale geometry in nav) and
# ages out after RESYNC_BASE_HOLD_SUBMAPS submaps to avoid a lingering seam.
RESYNC_PRESERVE_DISPLAY  = os.environ.get("RESYNC_PRESERVE_DISPLAY", "1") == "1"
RESYNC_BASE_HOLD_SUBMAPS = int(os.environ.get("RESYNC_BASE_HOLD_SUBMAPS", "8"))
MIN_NEW_FRAMES    = int(os.environ.get("MIN_NEW_FRAMES", "1"))
RETRY_TIMEOUT_S   = float(os.environ.get("RETRY_TIMEOUT_S", "0.3"))
MAX_RETRIES_CYCLE = int(os.environ.get("MAX_RETRIES_CYCLE", str(WINDOW_SIZE)))

# ── Camera / image ───────────────────────────────────────────────────────────
CAMERA_HEIGHT = float(os.environ.get("CAMERA_HEIGHT", "0.50"))  # fallback only
TARGET_WIDTH  = int(os.environ.get("TARGET_WIDTH",  "1036"))
TARGET_HEIGHT = int(os.environ.get("TARGET_HEIGHT", "518"))
ZENITH_LIMIT  = float(os.environ.get("ZENITH_LIMIT", "75"))
NADIR_LIMIT   = float(os.environ.get("NADIR_LIMIT", "-70"))

# ── Pose-correction safety (see pose_estimation.PoseCorrectionGate) ──────────
CORRECTION_MAX_SPEED    = float(os.environ.get("CORRECTION_MAX_SPEED", "2.5"))
CORRECTION_JUMP_MARGIN  = float(os.environ.get("CORRECTION_JUMP_MARGIN", "0.75"))
CORRECTION_DEADBAND_M   = float(os.environ.get("CORRECTION_DEADBAND_M", "0.06"))
CORRECTION_DEADBAND_DEG = float(os.environ.get("CORRECTION_DEADBAND_DEG", "3.0"))


# ── Navigation ESDF (world-frame collision field) ────────────────────────────
# When COMPUTE_ESDF=1, publish a horizontal ESDF slice (signed distance to the
# nearest obstacle, meters) for the nav stack. In reset mode the nvblox volume lives
# in the fresh reconstruction's LOCAL frame; nav_esdf transforms the slice into the
# persistent WORLD frame using the same rigid anchor as the streamed cloud, so the
# planner and the viewer share one frame.
NAV_ESDF_PUBLISH  = os.environ.get("NAV_ESDF_PUBLISH", "1") == "1"
NAV_ESDF_KEY      = f"{SERVER_PREFIX}/esdf_slice"
NAV_ESDF_RES_M    = float(os.environ.get("NAV_ESDF_RES_M", "0.05"))  # slice sample spacing
# Slice height above the floor (world Z, m). Empty ⇒ median trajectory height.
NAV_ESDF_HEIGHT_M = os.environ.get("NAV_ESDF_HEIGHT_M", "").strip()
NAV_ESDF_EVERY_N  = int(os.environ.get("NAV_ESDF_EVERY_N", "1"))     # publish every N submaps


def summary() -> str:
    """One-line config echo for the startup log."""
    _mode = (f"RESET(win={RESET_WINDOW_FRAMES},soft={'on' if SOFT_RESET else 'off'},"
             f"anchor={'on' if RESET_WORLD_ANCHOR else 'off'})"
             if PRISM_RESET_EACH_BATCH else f"ONLINE[deprecated](decay={'on' if TSDF_DECAY else 'off'})")
    return (f"MODE={_mode} stream={STREAM_MODE} esdf={'on' if COMPUTE_ESDF else 'off'} "
            f"window={WINDOW_SIZE} overlap={OVERLAP} voxel={VOXEL_SIZE} "
            f"face={FACE_SIZE} proc={PROCESSING_MODE} cube={CUBE_SIZE} "
            f"stream_voxel={STREAM_VOXEL_M} voxel_snap={CLOUD_VOXEL_SNAP} "
            f"kf_gate={KEYFRAME_MIN_TRANS_M}m/{KEYFRAME_MIN_ROT_DEG}deg")
