"""
VAT - Remote Periscope geometry & zoom math (pure NumPy, no cv2 / no Zenoh)
===========================================================================
Shared between the robot (which renders the slice) and the client (which draws
the aiming frustum), so both agree bit-for-bit on the projection. See
docs/periscope.md for the derivation.

Conventions
-----------
* Source panorama is equirectangular: width spans longitude [-180, +180] deg,
  height spans latitude [+90 (top) .. -90 (bottom)] deg. Longitude/latitude 0,0
  is the panorama's forward (image centre column, equator row).
* Periscope aim is (yaw, pitch) in degrees, relative to that forward:
  +yaw pans right, +pitch looks up.
* Camera frame for the frustum: +x right, +y up, +z forward.

No OpenCV here on purpose: build_remap returns the float32 sampling maps; the
robot applies them with cv2.remap. The client only needs the pure-math helpers
(dims, fov, frustum), which import cleanly without cv2.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

# Default RICOH Theta X 4K equirectangular source.
DEFAULT_SRC_W = 3840
DEFAULT_SRC_H = 1920


def px_per_deg(src_w: int = DEFAULT_SRC_W) -> float:
    """Angular resolution of the equirectangular source near the horizon."""
    return src_w / 360.0


def parse_aspect(aspect: str) -> Tuple[float, float]:
    """'1:1' | '4:3' | '16:9' | 'W:H' -> (w, h) floats. Falls back to 1:1."""
    try:
        a, b = str(aspect).lower().replace(" ", "").split(":")
        aw, ah = float(a), float(b)
        if aw > 0 and ah > 0:
            return aw, ah
    except Exception:
        pass
    return 1.0, 1.0


def optical_floor_fov_deg(width_px: int, src_w: int = DEFAULT_SRC_W) -> float:
    """The narrowest HFOV that still fills width_px with real sensor pixels.
    Below this the view is source-limited (digital zoom)."""
    return float(width_px) / px_per_deg(src_w)


def _even(n) -> int:
    """Round to a positive even integer (video encoders want even dimensions)."""
    n = int(round(n))
    n -= n % 2
    return max(2, n)


def render_dims(hfov_deg: float, aspect: str, tier_short_px: int,
                src_w: int = DEFAULT_SRC_W) -> dict:
    """Decide the pixel dimensions to actually render at, honouring the
    never-upscale-on-the-robot rule.

    tier_short_px is the requested resolution tier (short side: 360/480/720).
    Returns a dict with:
      * req_w / req_h    - the display size the client asked for (tier x aspect)
      * render_w / render_h - what the robot renders (== req unless the source has
        fewer real pixels across the FOV, in which case it is the native count and
        the client upscales)
      * native_w - real sensor pixels available across hfov_deg
      * optical  - True if render fills the request from real pixels
    """
    aw, ah = parse_aspect(aspect)
    if aw >= ah:                       # landscape / square: short side = height
        req_h = _even(tier_short_px)
        req_w = _even(tier_short_px * aw / ah)
    else:                              # portrait: short side = width
        req_w = _even(tier_short_px)
        req_h = _even(tier_short_px * ah / aw)
    native_w = hfov_deg * px_per_deg(src_w)
    optical = req_w <= native_w
    render_w = _even(min(req_w, native_w))
    render_h = _even(render_w * req_h / req_w)   # preserve aspect
    return {
        "req_w": req_w, "req_h": req_h,
        "render_w": render_w, "render_h": render_h,
        "native_w": int(round(native_w)), "optical": bool(optical),
    }


def vfov_from_hfov(hfov_deg: float, width_px: int, height_px: int) -> float:
    """Vertical FOV (deg) for a rectilinear view of the given HFOV and pixel aspect.
    Rectilinear relates FOVs through the tangents, not linearly."""
    hf = np.radians(hfov_deg)
    vf = 2.0 * np.arctan(np.tan(hf / 2.0) * float(height_px) / float(width_px))
    return float(np.degrees(vf))


def clamp_view(yaw_deg: float, pitch_deg: float, hfov_deg: float,
               min_fov: float, max_fov: float) -> Tuple[float, float, float]:
    """Wrap yaw to [-180,180], clamp pitch to (-90,90), clamp HFOV to [min,max]."""
    yaw = (float(yaw_deg) + 180.0) % 360.0 - 180.0
    pitch = float(np.clip(pitch_deg, -89.0, 89.0))
    hfov = float(np.clip(hfov_deg, min_fov, max_fov))
    return yaw, pitch, hfov


def _rot_yaw(deg: float) -> np.ndarray:
    """Rotation about +y (up): +yaw pans a forward ray toward +x (right)."""
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_pitch(deg: float) -> np.ndarray:
    """Rotation about +x (right): +pitch tilts a forward ray toward +y (up).
    Forward (0,0,1) -> (0, +sin, +cos)."""
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, s], [0.0, -s, c]])


def view_rotation(yaw_deg: float, pitch_deg: float) -> np.ndarray:
    """Camera->panorama rotation for a given aim (yaw applied after pitch)."""
    return _rot_yaw(yaw_deg) @ _rot_pitch(pitch_deg)


def build_remap(src_h: int, src_w: int, yaw_deg: float, pitch_deg: float,
                hfov_deg: float, vfov_deg: float, out_w: int, out_h: int
                ) -> Tuple[np.ndarray, np.ndarray]:
    """Build (map_x, map_y) float32 arrays of shape (out_h, out_w) that sample the
    equirectangular source into a rectilinear perspective view. Feed straight to
    cv2.remap(equirect, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP).
    """
    xs = (np.arange(out_w, dtype=np.float64) + 0.5) / out_w * 2.0 - 1.0
    ys = (np.arange(out_h, dtype=np.float64) + 0.5) / out_h * 2.0 - 1.0
    gx, gy = np.meshgrid(xs, ys)                     # (out_h, out_w)
    tan_h = np.tan(np.radians(hfov_deg) / 2.0)
    tan_v = np.tan(np.radians(vfov_deg) / 2.0)
    rx = gx * tan_h
    ry = -gy * tan_v                                 # image y is down -> up = -y
    rz = np.ones_like(rx)
    dirs = np.stack([rx, ry, rz], axis=-1)           # (out_h, out_w, 3)
    dirs /= np.linalg.norm(dirs, axis=-1, keepdims=True)
    R = view_rotation(yaw_deg, pitch_deg)            # camera -> panorama
    world = dirs @ R.T                               # rotate each ray
    wx, wy, wz = world[..., 0], world[..., 1], world[..., 2]
    lon = np.arctan2(wx, wz)                         # forward (0,0,1) -> lon 0
    lat = np.arcsin(np.clip(wy, -1.0, 1.0))          # up -> +lat
    map_x = (lon / (2.0 * np.pi) + 0.5) * src_w - 0.5
    map_y = (0.5 - lat / np.pi) * src_h - 0.5
    return map_x.astype(np.float32), map_y.astype(np.float32)


def frustum_edges(hfov_deg: float, vfov_deg: float, far_m: float = 3.0,
                  near_m: float = 0.0) -> np.ndarray:
    """Wireframe of the view frustum in the camera frame (+x right, +y up, +z
    forward), as an (N, 2, 3) array of segment endpoint pairs - ready to reshape
    to (2N, 3) for a VisPy Line(connect='segments'). The client rotates these by
    the camera world orientation and translates to the camera world position
    (apex at origin here)."""
    th = np.tan(np.radians(hfov_deg) / 2.0)
    tv = np.tan(np.radians(vfov_deg) / 2.0)

    def rect(z):
        return np.array([
            [+th * z, +tv * z, z], [-th * z, +tv * z, z],
            [-th * z, -tv * z, z], [+th * z, -tv * z, z],
        ])
    far = rect(far_m)
    apex = np.zeros(3)
    segs = []
    for c in far:                       # apex -> 4 far corners (diverging edges)
        segs.append([apex, c])
    for i in range(4):                  # far rectangle
        segs.append([far[i], far[(i + 1) % 4]])
    if near_m > 0.0:
        near = rect(near_m)
        for i in range(4):
            segs.append([near[i], near[(i + 1) % 4]])
    return np.asarray(segs, dtype=np.float32)


# ---------------------------------------------------------------------------
# Self-test:  python common/vat_periscope.py
# ---------------------------------------------------------------------------

def _selftest() -> None:
    assert abs(px_per_deg() - 10.6667) < 1e-3

    # optical floor: 480 px square -> 45 deg, 1280 px -> 120 deg
    assert abs(optical_floor_fov_deg(480) - 45.0) < 0.1
    assert abs(optical_floor_fov_deg(1280) - 120.0) < 0.1

    # render dims never upscale
    d = render_dims(90.0, "1:1", 480)
    assert d["render_w"] == 480 and d["optical"], d
    d = render_dims(30.0, "1:1", 480)
    assert d["render_w"] == 320 and not d["optical"], d
    d = render_dims(90.0, "16:9", 480)
    assert d["req_w"] == 852 and d["optical"], d       # 480*16/9 -> 852 (even)
    d = render_dims(60.0, "16:9", 480)
    assert not d["optical"], d          # 60 < 80 deg floor -> source-limited

    # vfov
    assert abs(vfov_from_hfov(90.0, 480, 480) - 90.0) < 1e-6
    assert vfov_from_hfov(90.0, 960, 480) < 90.0

    # clamp
    y, p, f = clamp_view(200.0, -120.0, 200.0, 20.0, 130.0)
    assert abs(y + 160.0) < 1e-6 and p == -89.0 and f == 130.0

    # remap: centre pixel of a centred view samples the panorama centre
    W, H, ow, oh = 3840, 1920, 64, 64
    vf = vfov_from_hfov(60.0, ow, oh)
    mx, my = build_remap(H, W, 0.0, 0.0, 60.0, vf, ow, oh)
    cx, cy = mx[oh // 2, ow // 2], my[oh // 2, ow // 2]
    assert abs(cx - W / 2) < 8.0 and abs(cy - H / 2) < 8.0, (cx, cy)
    # +yaw pans right; +pitch looks up (smaller row)
    assert build_remap(H, W, 45.0, 0.0, 60.0, vf, ow, oh)[0][oh // 2, ow // 2] > W / 2
    assert build_remap(H, W, 0.0, 30.0, 60.0, vf, ow, oh)[1][oh // 2, ow // 2] < H / 2

    # frustum: apex at origin, far corners share z = far
    edges = frustum_edges(90.0, 90.0, far_m=2.0)
    assert edges.shape[1:] == (2, 3)
    assert np.allclose(edges[:4, 0, :], 0.0)
    assert np.allclose(edges[:4, 1, 2], 2.0)

    print("vat_periscope self-test OK  "
          "(px/deg={:.3f}, 480sq optical >= {:.0f} deg)".format(
              px_per_deg(), optical_floor_fov_deg(480)))


if __name__ == "__main__":
    _selftest()
