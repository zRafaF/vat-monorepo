"""
VAT — Remote Periscope: equirectangular → rectilinear slice renderer (robot side).

Wraps the shared geometry in ``common/vat_periscope`` with an OpenCV remap and a
one-entry map cache (rebuild only when the view actually changes), and enforces
the never-upscale-on-the-robot rule via ``render_dims``.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

import vat_periscope as psc


class SliceRenderer:
    """Render a rectilinear slice from an equirectangular BGR frame.

    ``render()`` returns ``(slice_bgr, meta)`` where ``meta`` carries the ACTUAL
    parameters used (so the wire header can echo them for the client's frustum)."""

    def __init__(self):
        import cv2
        self._cv2 = cv2
        self._key = None
        self._mx = None
        self._my = None

    def render(self, equirect_bgr: np.ndarray, yaw_deg: float, pitch_deg: float,
               hfov_deg: float, aspect: str, res_tier: int) -> Tuple[np.ndarray, dict]:
        src_h, src_w = equirect_bgr.shape[:2]
        dims = psc.render_dims(hfov_deg, aspect, res_tier, src_w=src_w)
        out_w, out_h = dims["render_w"], dims["render_h"]
        vfov = psc.vfov_from_hfov(hfov_deg, out_w, out_h)

        # Rebuild the sampling maps only when the view or output size changes.
        key = (src_h, src_w, round(yaw_deg, 2), round(pitch_deg, 2),
               round(hfov_deg, 2), out_w, out_h)
        if key != self._key:
            self._mx, self._my = psc.build_remap(src_h, src_w, yaw_deg, pitch_deg,
                                                  hfov_deg, vfov, out_w, out_h)
            self._key = key

        slice_bgr = self._cv2.remap(equirect_bgr, self._mx, self._my,
                                    interpolation=self._cv2.INTER_LINEAR,
                                    borderMode=self._cv2.BORDER_WRAP)
        aw, ah = psc.parse_aspect(aspect)
        meta = {
            "width": out_w, "height": out_h, "native_w": dims["native_w"],
            "optical": dims["optical"], "yaw_deg": yaw_deg, "pitch_deg": pitch_deg,
            "hfov_deg": hfov_deg, "vfov_deg": vfov,
            "aspect_w": int(round(aw)), "aspect_h": int(round(ah)),
        }
        return slice_bgr, meta
