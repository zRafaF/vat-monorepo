"""
VAT mapping server — frame decode & masking.

Turns a raw VAT camera-frame payload (header + JPEG) into an ``IncomingFrame``
the PRISM engine can consume: RGB image at the canonical size, the spherical
valid-pixel mask, the stamped camera height, and the capture timestamp.

Kept separate from the orchestrator so the (pure, testable) decode path is
isolated from Zenoh and the engine.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

import mapping_config as cfg
import vat_protocol as proto
from prism_vggt.utils.masking import get_spherical_valid_mask


@dataclass
class IncomingFrame:
    image: np.ndarray          # HxWx3 uint8 RGB at (TARGET_HEIGHT, TARGET_WIDTH)
    mask: np.ndarray           # HxW bool spherical valid mask
    camera_height: float       # metres above floor (>0); used only to anchor scale
    timestamp: float           # capture time (s, robot clock)


def build_mask() -> np.ndarray:
    """The spherical valid-pixel mask (zenith/nadir exclusion) at canonical size."""
    return get_spherical_valid_mask(
        cfg.TARGET_HEIGHT, cfg.TARGET_WIDTH,
        zenith_deg=cfg.ZENITH_LIMIT, nadir_deg=cfg.NADIR_LIMIT)


def decode_frame(payload: bytes, mask: np.ndarray):
    """Decode a VAT frame payload → ``(seq, IncomingFrame)`` or ``None`` on a
    JPEG failure. Raises ``proto.ProtocolError`` on a malformed header."""
    ts_ns, seq, cam_h, jpeg = proto.unpack_frame(payload)
    arr = np.frombuffer(jpeg, dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    if rgb.shape[1] != cfg.TARGET_WIDTH or rgb.shape[0] != cfg.TARGET_HEIGHT:
        rgb = cv2.resize(rgb, (cfg.TARGET_WIDTH, cfg.TARGET_HEIGHT),
                         interpolation=cv2.INTER_AREA)
    height = cam_h if cam_h > 0 else cfg.CAMERA_HEIGHT
    return seq, IncomingFrame(image=rgb, mask=mask.copy(),
                              camera_height=height, timestamp=ts_ns * 1e-9)
