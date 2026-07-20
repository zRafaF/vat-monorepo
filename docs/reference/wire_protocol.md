# Wire Protocol

Every process in VAT talks over [Zenoh](https://zenoh.io) using a small set of binary
messages defined in one place: **`common/vat_protocol.py`**. That module is the single source
of truth — the `pack_*`/`unpack_*` functions and the `keys()` map. This page mirrors it for
quick reference; if the two ever disagree, the code wins.

!!! note "Conventions"
    - All multi-byte fields are **big-endian** ("network order", `struct` prefix `!`).
    - Coordinate frames: **W** = world/map, **C** = camera optical centre, **B** = robot base.
    - `{robot}` is the robot name (default `go2`); `server/prism` is the server prefix. Both
      come from `vat.env` and are assembled by `vat_protocol.keys()`.

## Zenoh key map

| Key | Direction | Payload |
|---|---|---|
| `{robot}/prism/camera/frame` | robot → server | `pack_frame` — decimated camera frame |
| `{robot}/prism/camera/frame/get` | server ↔ robot | queryable: re-request a dropped frame by seq |
| `{robot}/prism/camera/archive/get` | client ↔ robot | queryable: fetch a full-res archived frame by seq |
| `server/prism/pcd_snapshot` | server → client | `pack_pcd` — whole-map point cloud |
| `server/prism/pcd_delta` | server → client | `pack_pcd` — point-cloud delta |
| `server/prism/pcd/manifest`, `/pcd/blocks`, `/pcd/push` | server ↔ client | block-diff cloud sync (`STREAM_MODE=blocks`) |
| `server/prism/trajectory` | server → client | `pack_trajectory` — camera positions |
| `server/prism/pose_correction` | server → robot | `pack_pose_correction` — VGGT camera pose (DOWN) |
| `server/prism/esdf_slice` | server → client | ESDF navigation slice (`COMPUTE_ESDF=1`) |
| `server/prism/status` | server → all | JSON telemetry/heartbeat |
| `{robot}/prism/pose` | robot → client | `pack_pose` — authoritative fused pose (UP) |
| `{robot}/prism/pose/liveliness` | robot | liveliness token for the pose stream |
| `{robot}/rt/prism/config/throttle_fps`, `.../window_size` | any → server/robot | live-tuning config (UTF-8 string) |
| `{robot}/teleop/cmd_vel` | client → robot | `pack_cmd_vel` — velocity command + e-stop |
| `{robot}/teleop/liveliness` | client | liveliness token for teleop |
| `{robot}/prism/periscope/request` / `/frame` / `/keyframe` | client ↔ robot | periscope aim / video / keyframe request |
| `{robot}/prism/rgbd/request` / `/frame` | client ↔ robot | RGBD panel request / frame |
| `{robot}/prism/vo` | camera → fuser | visual-odometry delta (disabled by default) |

## Message magic numbers

Each fixed message begins with a 4-byte ASCII magic so a mis-routed or truncated buffer is
rejected instead of silently misparsed.

| Magic | ASCII | Message |
|---|---|---|
| `0x46524D45` | `FRME` | camera frame |
| `0x50434400` | `PCD\0` | point cloud |
| `0x54524A00` | `TRJ\0` | trajectory |
| `0x504F5345` | `POSE` | authoritative robot pose |
| `0x50434F52` | `PCOR` | VGGT pose correction |
| `0x434D4456` | `CMDV` | teleop velocity command |
| `0x56524551` / `0x50534346` | `VREQ` / `PSCF` | periscope request / frame |
| `0x52474252` / `0x52474246` | `RGBR` / `RGBF` | RGBD request / frame |
| `0x564F444F` | `VODO` | visual-odometry delta |

## Key message layouts

### Camera frame — `{robot}/prism/camera/frame`

| Offset | Bytes | Type | Field |
|---|---|---|---|
| 0 | 4 | int32 | magic = `FRME` |
| 4 | 8 | int64 | timestamp_ns (capture time) |
| 12 | 4 | uint32 | seq (monotonic; lets the server detect & re-request drops) |
| 16 | 4 | float32 | camera_height (m above floor; `<0` = unknown) |
| 20 | … | bytes | encoded image (JPEG/WebP body) |

### Authoritative robot pose — `{robot}/prism/pose`

The pose is **84 bytes** (v2). Acceleration was appended after the original 72-byte v1 layout,
so a v1 reader still parses the first 72 bytes and a v2 reader fills acceleration with zeros for
a legacy buffer.

| Offset | Bytes | Type | Field |
|---|---|---|---|
| 0 | 4 | int32 | magic = `POSE` |
| 4 | 8 | int64 | timestamp_ns |
| 12 | 4 | int32 | seq (monotonic) |
| 16 | 12 | float32[3] | position xyz (map frame) |
| 28 | 16 | float32[4] | quaternion x,y,z,w (map frame) |
| 44 | 12 | float32[3] | linear velocity (m/s, map frame) |
| 56 | 12 | float32[3] | angular velocity (rad/s, body frame) |
| 68 | 4 | int32 | fix_quality (`0` dead-reckon / `1` VGGT-corrected) |
| 72 | 12 | float32[3] | linear acceleration (m/s², map frame) — *v2, appended* |

The acceleration lets the client extrapolate at constant acceleration (rather than constant
velocity) to mask latency through accel/braking without rubber-banding.

### VGGT pose correction — `server/prism/pose_correction`

The **camera** pose in the map frame as estimated by PRISM-VGGT. The robot converts it to a
base-frame correction with its own kinematics; the server stays kinematics-agnostic. Fixed
44 bytes.

| Offset | Bytes | Type | Field |
|---|---|---|---|
| 0 | 4 | int32 | magic = `PCOR` |
| 4 | 8 | int64 | timestamp_ns (keyframe capture time) |
| 12 | 4 | int32 | map_version |
| 16 | 12 | float32[3] | camera position xyz (map frame) |
| 28 | 16 | float32[4] | camera quaternion x,y,z,w (map frame) |

### Point cloud — `server/prism/pcd_snapshot` / `pcd_delta`

24-byte header, then a body whose encoding is selected by the `encoding` field.

| Offset | Bytes | Type | Field |
|---|---|---|---|
| 0 | 4 | int32 | magic = `PCD\0` |
| 4 | 4 | int32 | version (engine map version) |
| 8 | 4 | int32 | n_points |
| 12 | 4 | int32 | is_snapshot (1 full / 0 delta) |
| 16 | 4 | int32 | since_version (delta base; 0 if snapshot) |
| 20 | 4 | int32 | encoding |
| 24 | … | bytes | body |

Encodings: `0` RAW_F32 (legacy), `1` ZLIB_U8, **`2` ZLIB_QUANT (default)** — positions
quantised to 16 bits per axis across the cloud's bounding box, colour to 8 bits, then
zlib-deflated (~5–8× smaller than raw float32).

### Trajectory — `server/prism/trajectory`

8-byte header (magic `TRJ\0` + int32 `n`) followed by `float32[n,3]` camera positions.

### Teleop command — `{robot}/teleop/cmd_vel`

Magic `CMDV`, timestamp_ns, seq, then `vx, vy, vyaw` (float32) and an 8-bit flags byte whose
`0x01` bit is a latched e-stop (forces the robot into Damp). Streamed ~20 Hz; the robot's teleop
bridge stops the robot if the stream lapses past its deadman window.

---

For the remaining messages (periscope, RGBD, visual odometry) and the exact packing code, read
`common/vat_protocol.py` directly — it is thoroughly commented and is the authoritative
definition.
