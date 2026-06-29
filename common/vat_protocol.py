"""
VAT — Shared Wire Protocol
==========================
Single source of truth for every binary message that crosses a Zenoh key in
the VAT system.  Robot, server and client all import this module so the byte
layouts can never drift out of sync.

It is intentionally dependency-light: only the standard library + NumPy, so it
runs unchanged inside the robot Docker container (ROS Humble / Py 3.10), on the
CUDA server (Py 3.10+) and on the client (Py 3.12).

Conventions
-----------
* All multi-byte fields are **big-endian** ("network order"), `struct` prefix `!`.
* Quaternions are ``(x, y, z, w)`` Hamilton convention.
* Frames: ``W`` = world/map, ``C`` = camera optical centre, ``B`` = robot base.
* Lengths/positions are metres, angles radians, velocities m/s and rad/s.

Messages
--------
=========================  =============================  ====================
Zenoh key (default names)  Producer → Consumer            Helper
=========================  =============================  ====================
{robot}/prism/camera/frame   robot decimator → server     pack/unpack_frame
server/prism/pcd_delta       server → client              pack/unpack_pcd
server/prism/pcd_snapshot    server → client              pack/unpack_pcd
server/prism/trajectory      server → client              pack/unpack_trajectory
server/prism/pose_correction server → robot fuser (DOWN)   pack/unpack_pose_correction
{robot}/prism/pose           robot fuser → client (UP)     pack/unpack_pose
=========================  =============================  ====================

Every ``unpack_*`` validates its magic and raises ``ProtocolError`` on a bad
or truncated buffer, so callers can ``try/except`` and drop a corrupt sample
instead of crashing.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# Errors
# ─────────────────────────────────────────────────────────────────────────────


class ProtocolError(ValueError):
    """Raised when a buffer fails magic/length validation."""


# ─────────────────────────────────────────────────────────────────────────────
# Magics  (4-byte ASCII tags, read big-endian as int32)
# ─────────────────────────────────────────────────────────────────────────────

MAGIC_FRAME = 0x46524D45  # "FRME"
MAGIC_PCD   = 0x50434400  # "PCD\x00"
MAGIC_TRAJ  = 0x54524A00  # "TRJ\x00"
MAGIC_POSE  = 0x504F5345  # "POSE"
MAGIC_PCOR  = 0x50434F52  # "PCOR"
MAGIC_CMDV  = 0x434D4456  # "CMDV"

# cmd_vel flag bits (bitmask in the CmdVel.flags byte)
CMDV_FLAG_ESTOP = 0x01   # latched emergency stop — robot enters Damp, ignores motion

# Fix-quality enum for pose messages
FIX_DEADRECKON = 0   # propagating on odometry only
FIX_CORRECTED  = 1   # recently anchored by a VGGT correction


# ─────────────────────────────────────────────────────────────────────────────
# Default Zenoh key schema
# ─────────────────────────────────────────────────────────────────────────────


def keys(robot_name: str = "go2", server_prefix: str = "server/prism") -> dict:
    """Return the canonical Zenoh key names for a given robot/server prefix."""
    return {
        # robot → server
        "camera_frame":     f"{robot_name}/prism/camera/frame",
        # server → robot queryable: re-request a missed frame by seq
        "camera_frame_get": f"{robot_name}/prism/camera/frame/get",
        # server/client → robot queryable: fetch the FULL-RES archived
        # frame by seq (same seq the live stream used)
        "camera_archive_get": f"{robot_name}/prism/camera/archive/get",
        # server → client
        "pcd_delta":       f"{server_prefix}/pcd_delta",
        "pcd_snapshot":    f"{server_prefix}/pcd_snapshot",
        # block-sync (diff-based): manifest (pub/sub) + a queryable the client GETs
        # with its requested cube-keys as the query payload → Draco bundle reply.
        "pcd_manifest":    f"{server_prefix}/pcd/manifest",
        "pcd_blocks":      f"{server_prefix}/pcd/blocks",
        # proactive push of changed+removed cubes (pub/sub, low-latency path)
        "pcd_push":        f"{server_prefix}/pcd/push",
        "trajectory":      f"{server_prefix}/trajectory",
        "status":          f"{server_prefix}/status",
        # server → robot (DOWN)
        "pose_correction": f"{server_prefix}/pose_correction",
        # robot → client (UP), relayed by the router
        "pose":            f"{robot_name}/prism/pose",
        # live config (anyone → robot)
        "cfg_throttle_fps": f"{robot_name}/rt/prism/config/throttle_fps",
        "cfg_window_size":  f"{robot_name}/rt/prism/config/window_size",
        # teleoperation (client/server → robot, DOWN): velocity commands + e-stop
        "cmd_vel":         f"{robot_name}/teleop/cmd_vel",
        # liveliness tokens
        "live_server":     f"{server_prefix}/liveliness",
        "live_pose":       f"{robot_name}/prism/pose/liveliness",
        "live_teleop":     f"{robot_name}/teleop/liveliness",
    }


# ═════════════════════════════════════════════════════════════════════════════
# 1. Camera frame   {robot}/prism/camera/frame   (robot decimator → server)
# ═════════════════════════════════════════════════════════════════════════════
#
#   Offset  Bytes  Type     Field
#   ──────  ─────  ───────  ───────────────────────────────────────────────
#   0       4      int32    magic = MAGIC_FRAME
#   4       8      int64    timestamp_ns   (capture time)
#   12      4      uint32   seq            (monotonic; lets the server detect
#                                            & re-request dropped frames)
#   16      4      float32  camera_height  (m above floor; <0 = unknown)
#   20      …      bytes    JPEG image
#
_FRAME_HDR = "!iqIf"
_FRAME_HDR_SIZE = struct.calcsize(_FRAME_HDR)


def pack_frame(timestamp_ns: int, seq: int, camera_height: float,
               jpeg: bytes) -> bytes:
    """Serialise a decimated camera frame.  ``camera_height`` may be negative
    to signal 'unknown' so the server can fall back to its own estimate."""
    return struct.pack(_FRAME_HDR, MAGIC_FRAME, int(timestamp_ns),
                       int(seq) & 0xFFFFFFFF, float(camera_height)) + jpeg


def unpack_frame(buf: bytes) -> Tuple[int, int, float, bytes]:
    """Returns (timestamp_ns, seq, camera_height, jpeg_bytes)."""
    if len(buf) <= _FRAME_HDR_SIZE:
        raise ProtocolError("frame buffer too short")
    magic, ts_ns, seq, cam_h = struct.unpack_from(_FRAME_HDR, buf, 0)
    if magic != MAGIC_FRAME:
        raise ProtocolError(f"bad frame magic 0x{magic & 0xFFFFFFFF:08X}")
    return ts_ns, seq, cam_h, buf[_FRAME_HDR_SIZE:]


# ═════════════════════════════════════════════════════════════════════════════
# 2. Point cloud   pcd_delta / pcd_snapshot   (server → client)
# ═════════════════════════════════════════════════════════════════════════════
#
#   Offset  Bytes  Type      Field
#   ──────  ─────  ────────  ──────────────────────────────────────────────
#   0       4      int32     magic = MAGIC_PCD
#   4       4      int32     version        (engine map version)
#   8       4      int32     n_points
#   12      4      int32     is_snapshot    (1 full / 0 delta)
#   16      4      int32     since_version  (delta base; 0 if snapshot)
#   20      4      int32     encoding       (PCD_ENC_*; how the body is packed)
#   24      …      bytes     body
#
# Bodies:
#   PCD_ENC_RAW_F32   : xyz float32[n,3] (BE) ++ rgb float32[n,3] (BE)  — legacy
#   PCD_ENC_ZLIB_U8   : zlib( xyz float32[n,3] (BE) ++ rgb uint8[n,3] ) — lossless-ish
#   PCD_ENC_ZLIB_QUANT: [bbox_min 3×f32][bbox_span 3×f32] ++
#                       zlib( xyz uint16[n,3] (BE) ++ rgb uint8[n,3] )  — DEFAULT
#       Positions are quantised to 16 bits PER AXIS across the cloud's own bounding
#       box (≈span/65535 resolution → sub-mm at room scale, <1 mm even over tens of
#       metres) and colour to 8 bits, then deflated. ~5–8× smaller than RAW and the
#       smooth integer stream deflates far better than float32. This is the practical
#       sweet spot vs. Draco/G-PCC for ~10⁵ unorganised voxel-centre points without
#       pulling in a binary codec dependency.
#
PCD_ENC_RAW_F32    = 0
PCD_ENC_ZLIB_U8    = 1
PCD_ENC_ZLIB_QUANT = 2

_PCD_HDR = "!iiiiii"
_PCD_HDR_SIZE = struct.calcsize(_PCD_HDR)
_PCD_QHDR = "!6f"                       # bbox_min(3) + bbox_span(3), for QUANT bodies
_PCD_QHDR_SIZE = struct.calcsize(_PCD_QHDR)
_PCD_ZLIB_LEVEL = 6
_QMAX = 65535.0


def _rgb_to_u8(rgb: np.ndarray) -> bytes:
    u = np.clip(np.asarray(rgb, dtype=np.float32), 0.0, 1.0)
    return np.ascontiguousarray((u * 255.0 + 0.5).astype(np.uint8)).tobytes()


_PCD_SANE_LIMIT = 1000.0   # metres; drop points farther than this from the origin


def pack_pcd(version: int, xyz: np.ndarray, rgb: np.ndarray,
             is_snapshot: bool, since_version: int = 0,
             compress: bool = True, quantize: bool = True) -> bytes:
    """Serialise a point cloud. Default = 16-bit-quantised + zlib (smallest).
    ``compress=False`` → legacy raw float32; ``quantize=False`` → zlib + f32 xyz."""
    xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    rgb = np.asarray(rgb, dtype=np.float32).reshape(-1, 3)
    # Drop non-finite / absurd points BEFORE encoding. A single NaN/Inf makes the
    # quantisation bbox NaN and corrupts the ENTIRE cloud (and wrecks any viewer's
    # auto-bounds); a lone far outlier stretches the bbox so the real map collapses
    # into a few quantisation levels. Filter once, at the source.
    if xyz.shape[0]:
        ok = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).max(axis=1) < _PCD_SANE_LIMIT)
        if not ok.all():
            xyz, rgb = xyz[ok], rgb[ok]
    n = int(xyz.shape[0])
    if compress and quantize:
        enc = PCD_ENC_ZLIB_QUANT
        if n > 0:
            mn = xyz.min(axis=0)
            span = np.maximum(xyz.max(axis=0) - mn, 1e-6)   # avoid /0 on flat axes
            q = np.rint((xyz - mn) / span * _QMAX).astype(">u2")
            payload = zlib.compress(np.ascontiguousarray(q).tobytes() + _rgb_to_u8(rgb),
                                    _PCD_ZLIB_LEVEL)
        else:
            mn = np.zeros(3); span = np.ones(3); payload = zlib.compress(b"", _PCD_ZLIB_LEVEL)
        qhdr = struct.pack(_PCD_QHDR, *mn.astype(np.float64), *span.astype(np.float64))
        body = qhdr + payload
    elif compress:
        enc = PCD_ENC_ZLIB_U8
        body = zlib.compress(np.ascontiguousarray(xyz, dtype=">f4").tobytes()
                             + _rgb_to_u8(rgb), _PCD_ZLIB_LEVEL)
    else:
        enc = PCD_ENC_RAW_F32
        body = (np.ascontiguousarray(xyz, dtype=">f4").tobytes()
                + np.ascontiguousarray(rgb, dtype=">f4").tobytes())
    header = struct.pack(_PCD_HDR, MAGIC_PCD, int(version), n,
                         1 if is_snapshot else 0, int(since_version), enc)
    return header + body


def unpack_pcd(buf: bytes):
    """Returns (version, xyz(n,3) f32, rgb(n,3) f32 in [0,1], is_snapshot, since_version)."""
    if len(buf) < _PCD_HDR_SIZE:
        raise ProtocolError("pcd buffer too short")
    magic, version, n, is_snap, since_v, enc = struct.unpack_from(_PCD_HDR, buf, 0)
    if magic != MAGIC_PCD:
        raise ProtocolError(f"bad pcd magic 0x{magic & 0xFFFFFFFF:08X}")
    body = buf[_PCD_HDR_SIZE:]
    if enc == PCD_ENC_ZLIB_QUANT:
        if len(body) < _PCD_QHDR_SIZE:
            raise ProtocolError("pcd quant header too short")
        vals = struct.unpack_from(_PCD_QHDR, body, 0)
        mn = np.array(vals[0:3], dtype=np.float64)
        span = np.array(vals[3:6], dtype=np.float64)
        try:
            raw = zlib.decompress(body[_PCD_QHDR_SIZE:])
        except zlib.error as e:
            raise ProtocolError(f"pcd zlib decompress failed: {e}")
        if n == 0:
            z = np.zeros((0, 3), np.float32)
            return version, z, z, bool(is_snap), since_v
        need = n * 6 + n * 3
        if len(raw) < need:
            raise ProtocolError(f"pcd truncated: need {need}, have {len(raw)}")
        q = np.frombuffer(raw, dtype=">u2", count=n * 3, offset=0).reshape(n, 3).astype(np.float64)
        xyz = (mn + q / _QMAX * span).astype(np.float32)
        rgb = (np.frombuffer(raw, dtype=np.uint8, count=n * 3, offset=n * 6
                             ).reshape(n, 3).astype(np.float32) / 255.0)
        return version, xyz, rgb, bool(is_snap), since_v
    elif enc == PCD_ENC_ZLIB_U8:
        try:
            body = zlib.decompress(body)
        except zlib.error as e:
            raise ProtocolError(f"pcd zlib decompress failed: {e}")
        need = n * 12 + n * 3
        if len(body) < need:
            raise ProtocolError(f"pcd truncated: need {need}, have {len(body)}")
        xyz = np.frombuffer(body, dtype=">f4", count=n * 3, offset=0
                            ).reshape(n, 3).astype(np.float32)
        rgb = (np.frombuffer(body, dtype=np.uint8, count=n * 3, offset=n * 12
                             ).reshape(n, 3).astype(np.float32) / 255.0)
        return version, xyz, rgb, bool(is_snap), since_v
    elif enc == PCD_ENC_RAW_F32:
        need = n * 24
        if len(body) < need:
            raise ProtocolError(f"pcd truncated: need {need}, have {len(body)}")
        xyz = np.frombuffer(body, dtype=">f4", count=n * 3, offset=0
                            ).reshape(n, 3).astype(np.float32)
        rgb = np.frombuffer(body, dtype=">f4", count=n * 3, offset=n * 12
                            ).reshape(n, 3).astype(np.float32)
        return version, xyz, rgb, bool(is_snap), since_v
    raise ProtocolError(f"unknown pcd encoding {enc}")


# ═════════════════════════════════════════════════════════════════════════════
# 3. Trajectory   server/prism/trajectory   (server → client)
# ═════════════════════════════════════════════════════════════════════════════
#
#   Offset  Bytes  Type          Field
#   0       4      int32         magic = MAGIC_TRAJ
#   4       4      int32         n
#   8       n*12   float32[n,3]  camera positions (xyz)
#
_TRAJ_HDR = "!ii"
_TRAJ_HDR_SIZE = struct.calcsize(_TRAJ_HDR)


def pack_trajectory(positions: np.ndarray) -> bytes:
    n = int(positions.shape[0])
    return struct.pack(_TRAJ_HDR, MAGIC_TRAJ, n) + \
        np.ascontiguousarray(positions, dtype=">f4").tobytes()


def unpack_trajectory(buf: bytes) -> np.ndarray:
    if len(buf) < _TRAJ_HDR_SIZE:
        return np.zeros((0, 3), dtype=np.float32)
    magic, n = struct.unpack_from(_TRAJ_HDR, buf, 0)
    if magic != MAGIC_TRAJ:
        raise ProtocolError(f"bad trajectory magic 0x{magic & 0xFFFFFFFF:08X}")
    if n <= 0 or len(buf) < _TRAJ_HDR_SIZE + n * 12:
        return np.zeros((0, 3), dtype=np.float32)
    return np.frombuffer(buf, dtype=">f4", count=n * 3, offset=_TRAJ_HDR_SIZE
                         ).reshape(n, 3).astype(np.float32)


# ═════════════════════════════════════════════════════════════════════════════
# 4. Authoritative robot pose   {robot}/prism/pose   (robot fuser → client)
# ═════════════════════════════════════════════════════════════════════════════
#
#   Offset  Bytes  Type          Field
#   0       4      int32         magic = MAGIC_POSE
#   4       8      int64         timestamp_ns
#   12      4      int32         seq (monotonic)
#   16      12     float32[3]    position xyz       (map frame)
#   28      16     float32[4]    quaternion x,y,z,w (map frame)
#   44      12     float32[3]    linear  velocity   (m/s, map frame)
#   56      12     float32[3]    angular velocity   (rad/s, body frame)
#   68      4      int32         fix_quality (FIX_DEADRECKON / FIX_CORRECTED)
#   → 72 bytes, fixed size.
#
_POSE_FMT = "!iqi" + "3f" + "4f" + "3f" + "3f" + "i"
_POSE_SIZE = struct.calcsize(_POSE_FMT)
assert _POSE_SIZE == 72, _POSE_SIZE


@dataclass
class PoseState:
    timestamp_ns: int
    position: np.ndarray          # (3,)
    quaternion: np.ndarray        # (4,) xyzw
    linear_velocity: np.ndarray   # (3,)
    angular_velocity: np.ndarray  # (3,)
    seq: int = 0
    fix_quality: int = FIX_DEADRECKON


def pack_pose(p: PoseState) -> bytes:
    pos = np.asarray(p.position, dtype=np.float64).reshape(3)
    quat = np.asarray(p.quaternion, dtype=np.float64).reshape(4)
    lin = np.asarray(p.linear_velocity, dtype=np.float64).reshape(3)
    ang = np.asarray(p.angular_velocity, dtype=np.float64).reshape(3)
    return struct.pack(_POSE_FMT, MAGIC_POSE, int(p.timestamp_ns), int(p.seq),
                       *pos, *quat, *lin, *ang, int(p.fix_quality))


def unpack_pose(buf: bytes) -> PoseState:
    if len(buf) < _POSE_SIZE:
        raise ProtocolError("pose buffer too short")
    vals = struct.unpack_from(_POSE_FMT, buf, 0)
    if vals[0] != MAGIC_POSE:
        raise ProtocolError(f"bad pose magic 0x{vals[0] & 0xFFFFFFFF:08X}")
    return PoseState(
        timestamp_ns=vals[1],
        seq=vals[2],
        position=np.array(vals[3:6], dtype=np.float32),
        quaternion=np.array(vals[6:10], dtype=np.float32),
        linear_velocity=np.array(vals[10:13], dtype=np.float32),
        angular_velocity=np.array(vals[13:16], dtype=np.float32),
        fix_quality=vals[16],
    )


# ═════════════════════════════════════════════════════════════════════════════
# 5. VGGT pose correction   server/prism/pose_correction   (server → robot)
# ═════════════════════════════════════════════════════════════════════════════
#
# This is the *camera* pose in the world/map frame as estimated by PRISM-VGGT.
# The robot converts it to a base-frame correction using its own kinematics —
# the server stays kinematics-agnostic.
#
#   Offset  Bytes  Type          Field
#   0       4      int32         magic = MAGIC_PCOR
#   4       8      int64         timestamp_ns  (capture time of the keyframe)
#   12      4      int32         map_version
#   16      12     float32[3]    camera position xyz       (map frame)
#   28      16     float32[4]    camera quaternion x,y,z,w  (map frame)
#   → 44 bytes, fixed size.
#
_PCOR_FMT = "!iqi" + "3f" + "4f"
_PCOR_SIZE = struct.calcsize(_PCOR_FMT)
assert _PCOR_SIZE == 44, _PCOR_SIZE


@dataclass
class PoseCorrection:
    timestamp_ns: int
    map_version: int
    position: np.ndarray      # (3,) camera position in map frame
    quaternion: np.ndarray    # (4,) camera orientation xyzw in map frame


def pack_pose_correction(c: PoseCorrection) -> bytes:
    pos = np.asarray(c.position, dtype=np.float64).reshape(3)
    quat = np.asarray(c.quaternion, dtype=np.float64).reshape(4)
    return struct.pack(_PCOR_FMT, MAGIC_PCOR, int(c.timestamp_ns),
                       int(c.map_version), *pos, *quat)


def unpack_pose_correction(buf: bytes) -> PoseCorrection:
    if len(buf) < _PCOR_SIZE:
        raise ProtocolError("pose_correction buffer too short")
    vals = struct.unpack_from(_PCOR_FMT, buf, 0)
    if vals[0] != MAGIC_PCOR:
        raise ProtocolError(f"bad pose_correction magic 0x{vals[0] & 0xFFFFFFFF:08X}")
    return PoseCorrection(
        timestamp_ns=vals[1],
        map_version=vals[2],
        position=np.array(vals[3:6], dtype=np.float32),
        quaternion=np.array(vals[6:10], dtype=np.float32),
    )


# ═════════════════════════════════════════════════════════════════════════════
# 6. Teleop velocity command   {robot}/teleop/cmd_vel   (client/server → robot)
# ═════════════════════════════════════════════════════════════════════════════
#
# Streamed continuously (~20 Hz) by whoever is driving. The robot's teleop
# bridge relays these to the Go2 sport `Move` API, but only while they keep
# arriving — if the stream stops for longer than the bridge's watchdog window
# the robot is commanded to stop (deadman). A latched e-stop bit forces Damp.
#
#   Offset  Bytes  Type        Field
#   0       4      int32       magic = MAGIC_CMDV
#   4       8      int64       timestamp_ns  (sender clock; for staleness/debug)
#   12      4      uint32      seq           (monotonic)
#   16      4      float32     vx    forward  (m/s,  body frame, +x fwd)
#   20      4      float32     vy    lateral  (m/s,  body frame, +y left)
#   24      4      float32     vyaw  turn     (rad/s, +z up / CCW)
#   28      1      uint8       flags (bit0 = CMDV_FLAG_ESTOP)
#   → 29 bytes, fixed size.
#
_CMDV_FMT = "!iqI3fB"
_CMDV_SIZE = struct.calcsize(_CMDV_FMT)


@dataclass
class CmdVel:
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0
    flags: int = 0
    seq: int = 0
    timestamp_ns: int = 0

    @property
    def estop(self) -> bool:
        return bool(self.flags & CMDV_FLAG_ESTOP)


def pack_cmd_vel(c: CmdVel) -> bytes:
    return struct.pack(_CMDV_FMT, MAGIC_CMDV, int(c.timestamp_ns),
                       int(c.seq) & 0xFFFFFFFF, float(c.vx), float(c.vy),
                       float(c.vyaw), int(c.flags) & 0xFF)


def unpack_cmd_vel(buf: bytes) -> CmdVel:
    if len(buf) < _CMDV_SIZE:
        raise ProtocolError("cmd_vel buffer too short")
    vals = struct.unpack_from(_CMDV_FMT, buf, 0)
    if vals[0] != MAGIC_CMDV:
        raise ProtocolError(f"bad cmd_vel magic 0x{vals[0] & 0xFFFFFFFF:08X}")
    return CmdVel(timestamp_ns=vals[1], seq=vals[2], vx=vals[3], vy=vals[4],
                  vyaw=vals[5], flags=vals[6])


# ═════════════════════════════════════════════════════════════════════════════
# Quaternion helpers   (Hamilton, xyzw order) — used by fuser and predictor
# ═════════════════════════════════════════════════════════════════════════════


def quat_identity() -> np.ndarray:
    return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).reshape(4)
    n = np.linalg.norm(q)
    return quat_identity() if n < 1e-12 else q / n


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a ⊗ b (xyzw)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ], dtype=np.float64)


def quat_conj(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([-x, -y, -z, w], dtype=np.float64)


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate 3-vector v by quaternion q."""
    q = quat_normalize(q)
    vq = np.array([v[0], v[1], v[2], 0.0], dtype=np.float64)
    return quat_mul(quat_mul(q, vq), quat_conj(q))[:3]


def quat_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
    """Quaternion from a rotation vector (axis * angle).  Exact, small-angle safe."""
    rotvec = np.asarray(rotvec, dtype=np.float64).reshape(3)
    theta = np.linalg.norm(rotvec)
    if theta < 1e-9:
        # 2nd-order series; good enough for tiny dt steps
        return quat_normalize(np.array([rotvec[0] / 2, rotvec[1] / 2,
                                        rotvec[2] / 2, 1.0]))
    axis = rotvec / theta
    s = np.sin(theta / 2.0)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s,
                     np.cos(theta / 2.0)], dtype=np.float64)


def quat_slerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between unit quaternions a and b."""
    a = quat_normalize(a)
    b = quat_normalize(b)
    dot = float(np.dot(a, b))
    if dot < 0.0:            # take the short way round
        b = -b
        dot = -dot
    if dot > 0.9995:         # nearly colinear → linear interp
        return quat_normalize(a + t * (b - a))
    theta0 = np.arccos(np.clip(dot, -1.0, 1.0))
    theta = theta0 * t
    s0 = np.cos(theta) - dot * np.sin(theta) / np.sin(theta0)
    s1 = np.sin(theta) / np.sin(theta0)
    return quat_normalize(s0 * a + s1 * b)


def integrate_pose(position: np.ndarray, quaternion: np.ndarray,
                   linear_velocity: np.ndarray, angular_velocity: np.ndarray,
                   dt: float) -> Tuple[np.ndarray, np.ndarray]:
    """Dead-reckon a pose forward by ``dt`` seconds (the client predictor and
    the placeholder fuser both use this).

    position is integrated in the map frame; orientation is integrated with a
    body-frame angular velocity (q_new = q ⊗ Δq)."""
    pos = np.asarray(position, dtype=np.float64).reshape(3)
    quat = quat_normalize(quaternion)
    new_pos = pos + np.asarray(linear_velocity, dtype=np.float64).reshape(3) * dt
    dq = quat_from_rotvec(np.asarray(angular_velocity, dtype=np.float64).reshape(3) * dt)
    new_quat = quat_normalize(quat_mul(quat, dq))
    return new_pos.astype(np.float32), new_quat.astype(np.float32)


# ─────────────────────────────────────────────────────────────────────────────
# Encoding tags (optional Zenoh metadata — purely informational)
# ─────────────────────────────────────────────────────────────────────────────

ENC_FRAME = "application/vat.frame"
ENC_PCD   = "application/vat.pcd"
ENC_POSE  = "application/vat.pose"
ENC_PCOR  = "application/vat.pose_correction"
ENC_TRAJ  = "application/vat.trajectory"
ENC_CMDV  = "application/vat.cmd_vel"


# ═════════════════════════════════════════════════════════════════════════════
# Self-test:  python common/vat_protocol.py
# ═════════════════════════════════════════════════════════════════════════════

def _selftest() -> None:
    rng = np.random.default_rng(0)

    # frame round-trip (incl. unknown camera height + seq)
    jpeg = b"\xff\xd8\xff\xe0fakejpegdata"
    for ch in (1.234, -1.0):
        ts, seq, h, body = unpack_frame(pack_frame(123456789, 77, ch, jpeg))
        assert ts == 123456789 and seq == 77 and abs(h - ch) < 1e-5 and body == jpeg

    # pcd round-trip — DEFAULT = 16-bit-quantised + zlib (xyz lossy ≈ span/65535)
    xyz = rng.random((50, 3), dtype=np.float64).astype(np.float32)
    rgb = rng.random((50, 3), dtype=np.float64).astype(np.float32)
    v, xyz2, rgb2, snap, sv = unpack_pcd(pack_pcd(7, xyz, rgb, True))
    assert v == 7 and snap and sv == 0
    assert np.allclose(xyz, xyz2, atol=1.0 / 65535 + 1e-6)   # 16-bit positions
    assert np.allclose(rgb, rgb2, atol=1.0 / 255 + 1e-6)     # 8-bit colour
    # empty cloud (reset signal) round-trips to 0 points
    ev, ex, er, esnap, _ = unpack_pcd(pack_pcd(0, np.zeros((0, 3), np.float32),
                                               np.zeros((0, 3), np.float32), True))
    assert ev == 0 and esnap and ex.shape == (0, 3)
    # delta header + legacy raw mode round-trip
    vv, x3, r3, snap3, sv3 = unpack_pcd(pack_pcd(9, xyz, rgb, False, since_version=7))
    assert vv == 9 and not snap3 and sv3 == 7
    v4, x4, r4, *_ = unpack_pcd(pack_pcd(7, xyz, rgb, True, compress=False))
    assert np.allclose(rgb, r4, atol=1e-6) and np.allclose(xyz, x4, atol=1e-6)  # raw f32
    # quantised+zlib is the smallest of the three encodings on a realistic cloud
    big = (rng.integers(0, 64, (4000, 3)).astype(np.float32) * 0.05)
    bcol = rng.random((4000, 3)).astype(np.float32)
    s_q = len(pack_pcd(1, big, bcol, True))                       # quantised (default)
    s_u8 = len(pack_pcd(1, big, bcol, True, quantize=False))      # zlib + f32 xyz
    s_raw = len(pack_pcd(1, big, bcol, True, compress=False))     # raw f32
    assert s_q < s_u8 < s_raw, (s_q, s_u8, s_raw)

    # trajectory round-trip
    traj = rng.random((10, 3)).astype(np.float32)
    assert np.allclose(traj, unpack_trajectory(pack_trajectory(traj)), atol=1e-6)

    # pose round-trip
    p = PoseState(timestamp_ns=42, seq=3,
                  position=np.array([1, 2, 3], np.float32),
                  quaternion=quat_normalize([0.1, 0.2, 0.3, 1.0]).astype(np.float32),
                  linear_velocity=np.array([0.5, 0, -0.2], np.float32),
                  angular_velocity=np.array([0, 0, 0.1], np.float32),
                  fix_quality=FIX_CORRECTED)
    q = unpack_pose(pack_pose(p))
    assert q.timestamp_ns == 42 and q.seq == 3 and q.fix_quality == FIX_CORRECTED
    assert np.allclose(q.position, p.position, atol=1e-5)
    assert np.allclose(q.quaternion, p.quaternion, atol=1e-5)

    # pose_correction round-trip
    c = PoseCorrection(timestamp_ns=99, map_version=12,
                       position=np.array([4, 5, 6], np.float32),
                       quaternion=quat_identity().astype(np.float32))
    d = unpack_pose_correction(pack_pose_correction(c))
    assert d.map_version == 12 and np.allclose(d.position, c.position, atol=1e-5)

    # cmd_vel round-trip (incl. e-stop flag)
    cv = CmdVel(vx=0.25, vy=-0.1, vyaw=0.4, flags=CMDV_FLAG_ESTOP, seq=11,
                timestamp_ns=7)
    e = unpack_cmd_vel(pack_cmd_vel(cv))
    assert e.seq == 11 and e.estop and abs(e.vx - 0.25) < 1e-6 \
        and abs(e.vy + 0.1) < 1e-6 and abs(e.vyaw - 0.4) < 1e-6
    assert not CmdVel(vx=0.1).estop

    # quaternion math
    qx = quat_from_rotvec([np.pi / 2, 0, 0])           # 90° about +x
    v = quat_rotate(qx, np.array([0, 1.0, 0]))         # +y → +z
    assert np.allclose(v, [0, 0, 1], atol=1e-6), v
    assert np.allclose(quat_slerp(quat_identity(), qx, 0.0), quat_identity(), atol=1e-6)
    assert np.allclose(quat_slerp(quat_identity(), qx, 1.0), quat_normalize(qx), atol=1e-6)

    # integrate: 1 rad/s about z for 1 s, moving +x at 2 m/s
    pos, quat = integrate_pose([0, 0, 0], quat_identity(), [2, 0, 0], [0, 0, 1.0], 1.0)
    assert np.allclose(pos, [2, 0, 0], atol=1e-6)
    assert np.allclose(quat, quat_from_rotvec([0, 0, 1.0]).astype(np.float32), atol=1e-5)

    # truncation / bad magic guards
    for bad in (b"", b"\x00\x00\x00\x01", pack_pose(p)[:-1]):
        try:
            unpack_pose(bad)
        except ProtocolError:
            pass
        else:
            raise AssertionError("expected ProtocolError")

    print("vat_protocol self-test OK  "
          f"(pose={_POSE_SIZE}B  correction={_PCOR_SIZE}B  "
          f"cmd_vel={_CMDV_SIZE}B  "
          f"frame_hdr={_FRAME_HDR_SIZE}B  pcd_hdr={_PCD_HDR_SIZE}B)")


if __name__ == "__main__":
    _selftest()
