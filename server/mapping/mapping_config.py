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
# Cap points in the STREAMED snapshot (0 = no cap); full-res still via the query.
CLOUD_STREAM_MAX_POINTS = int(os.environ.get("CLOUD_STREAM_MAX_POINTS", "60000"))
CUBE_SIZE = float(os.environ.get("CUBE_SIZE", "1.0"))

# ── Streaming stability (online ghost / breathing / bandwidth fixes) ─────────
# Occupancy-CRC grid for block versioning (m). ~½ voxel so sub-voxel nvblox mesh
# "breathing" doesn't churn the diff (which used to resend ~every cube each submap).
CRC_QUANT_M = float(os.environ.get("CRC_QUANT_M", str(VOXEL_SIZE * 0.5)))
# Emit the streamed surface as one point per voxel CENTRE (byte-identical across
# submaps for unchanged geometry → stable CRCs). 1 = on (recommended online).
CLOUD_VOXEL_SNAP = os.environ.get("CLOUD_VOXEL_SNAP", "1") == "1"
# Keyframe gating: only integrate a frame into the TSDF if the camera moved enough
# since the last integrated frame. Stops re-integrating a static scene (the cause of
# breathing + ghost thickening + the "+119 cubes while stationary" churn). 0 = off.
KEYFRAME_MIN_TRANS_M = float(os.environ.get("KEYFRAME_MIN_TRANS_M", "0.05"))
KEYFRAME_MIN_ROT_DEG = float(os.environ.get("KEYFRAME_MIN_ROT_DEG", "8.0"))
# nvblox TSDF decay (active carving of stale/unsupported voxels). OFF by default —
# enable + tune on the rig once the decay API is confirmed for your nvblox build.
TSDF_DECAY = os.environ.get("TSDF_DECAY", "0") == "1"
# Cap the camera trajectory streamed to the viewer to the last N poses (the full
# trajectory grows without bound and is re-sent each submap). 0 = send all.
TRAJ_MAX_POSES = int(os.environ.get("TRAJ_MAX_POSES", "300"))
# 1 = full snapshot every submap (viewer replaces wholesale → always aligned).
PCD_KEYFRAME_EVERY = int(os.environ.get("PCD_KEYFRAME_EVERY", "1"))

# ── Batching / frame-drop recovery ───────────────────────────────────────────
WINDOW_TIMEOUT_S  = float(os.environ.get("WINDOW_TIMEOUT_S", "5.0"))
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


def summary() -> str:
    """One-line config echo for the startup log."""
    return (f"window={WINDOW_SIZE} overlap={OVERLAP} voxel={VOXEL_SIZE} "
            f"face={FACE_SIZE} mode={PROCESSING_MODE} cube={CUBE_SIZE} "
            f"keyframe_every={PCD_KEYFRAME_EVERY} crc_quant={CRC_QUANT_M:.3f} "
            f"voxel_snap={CLOUD_VOXEL_SNAP} kf_gate={KEYFRAME_MIN_TRANS_M}m/{KEYFRAME_MIN_ROT_DEG}deg")
