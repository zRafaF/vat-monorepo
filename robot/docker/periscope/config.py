"""
VAT — Remote Periscope: robot-side configuration (env, single place).

All knobs are read from the environment (exported from ``vat.env``). See
``docs/periscope.md`` for the rationale behind each default.
"""

from __future__ import annotations

import os


def _b(name: str, default: str = "0") -> bool:
    return os.environ.get(name, default).strip().lower() in ("1", "true", "yes", "on")


ENABLE       = _b("PERISCOPE_ENABLE", "1")
# Resolution tier: short side in px (360 | 480 | 720). The renderer never
# upscales past the sensor — see common/vat_periscope.render_dims.
RES_TIER     = int(os.environ.get("PERISCOPE_RES", "480"))
# Default aspect ratio "W:H" (client can override per request).
ASPECT       = os.environ.get("PERISCOPE_ASPECT", "1:1").strip()
# FOV bounds (deg). Max capped 120-140 to avoid rectilinear edge blow-up.
MAX_FOV      = float(os.environ.get("PERISCOPE_MAX_FOV", "130"))
MIN_FOV      = float(os.environ.get("PERISCOPE_MIN_FOV", "20"))
# Codec preference: h265 (HEVC) | h264 | mjpeg. Falls back automatically if the
# preferred encoder is unavailable (no PyAV / no NVENC).
CODEC        = os.environ.get("PERISCOPE_CODEC", "h265").strip().lower()
JPEG_QUALITY = int(os.environ.get("PERISCOPE_JPEG_QUALITY", "80"))   # mjpeg fallback
# Bitrate hint for the hardware/software video encoder (bits/s). Scaled internally
# by resolution; this is the ceiling for the 480p tier.
BITRATE      = int(os.environ.get("PERISCOPE_BITRATE", "1500000"))
# Frame rate. Static rate when dynamic is off; else dynamic ramps MIN..MAX.
FPS          = float(os.environ.get("PERISCOPE_FPS", "15"))
FPS_DYNAMIC  = _b("PERISCOPE_FPS_DYNAMIC", "1")
FPS_MIN      = float(os.environ.get("PERISCOPE_FPS_MIN", "8"))
FPS_MAX      = float(os.environ.get("PERISCOPE_FPS_MAX", "24"))
# Seconds after the last aim change during which we run at FPS_MAX (dynamic mode).
ACTIVE_WINDOW_S = float(os.environ.get("PERISCOPE_ACTIVE_WINDOW_S", "1.5"))
# Extra HFOV (deg) rendered beyond the request so the client can micro-pan
# within the received slice without a round trip.
OVERSCAN_DEG = float(os.environ.get("PERISCOPE_OVERSCAN_DEG", "10"))
# Keyframe (IDR) cadence (s) for drop recovery on the best-effort link.
IDR_INTERVAL_S = float(os.environ.get("PERISCOPE_IDR_INTERVAL_S", "2.0"))
# Only stream while a viewer is active: stop encoding this many seconds after the
# last view request (saves uplink when nobody is watching). 0 = always stream.
VIEWER_TIMEOUT_S = float(os.environ.get("PERISCOPE_VIEWER_TIMEOUT_S", "5.0"))
# Bounded send buffer (bytes) on the periscope's own uplink (bufferbloat guard,
# same discipline as the mapping camera). Empty/0 = OS default.
SO_SNDBUF    = os.environ.get("PERISCOPE_SO_SNDBUF", "262144").strip()
# Zenoh endpoint the periscope's own session dials (defaults to the shared one).
ZENOH_CONNECT = os.environ.get("ZENOH_CONNECT", "tcp/127.0.0.1:7447")
ROBOT_NAME    = os.environ.get("ROBOT_NAME", "go2")
