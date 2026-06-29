# Streaming pipeline refactor — ghosts, dead delta, latency, carving, avatar

Cross-cutting changes to the wire protocol, mapping server, client viewer, and the
PRISM-VGGT submodule. Goal: kill the online ghosts, make the diff actually small,
cut the green-flash/pose-line/cloud staggering, carve stale geometry, and add a
real URDF robot avatar. Everything is behind env flags with safe fallbacks.

## Root causes addressed
1. **Dead delta / "+119 cubes while stationary"** — cube CRC hashed positions at
   1 mm; nvblox marching-cubes vertices "breathe" sub-voxel every re-mesh → ~every
   CRC flips → near-full resend.
2. **Ghosts / breathing** — frame-count batching re-integrated a static scene every
   window, re-touching/thickening blocks.
3. **No carving** — removals were `-0`; walls accumulated, robot "walked inside
   walls" as pose drifted vs the stale map.
4. **Latency staggering** — trajectory+status shared the bulk lane; cloud used a
   pull round-trip; the client re-merged + re-uploaded the whole cloud each submap;
   and (round 2) the manifest was published *before* the push so the repair loop
   pulled everything.

## What changed (by layer)

### Shared (`common/`)
- `vat_blockmap.py`: **occupancy CRC** (`crc_quant_m` ≈ ½ voxel — invariant to
  breathing, flips on carving); **push delta** (`pack/unpack_block_push`);
  `ClientBlockStore.apply_push_bytes` + per-key `take_delta()`.
- `vat_protocol.py`: new key `pcd_push`.
- `vat_cloudbuffer.py` (new): `IncrementalCloud` slab buffer — per-cube
  in-place/append/compact instead of whole-map re-concat each submap.

### Server (`server/mapping/`)
- `block_publisher.py`: **publishes the push BEFORE the manifest** (race fix); push
  = changed cubes (Draco) + removed keys; manifest+query demoted to bootstrap/repair;
  `crc_quant_m`; skips push above `PUSH_MAX_CUBES`.
- `mapping_server.py`: passes `crc_quant_m`, `map_version`; reports `push_kb`.
- `mapping_config.py`: knobs `CRC_QUANT_M`, `CLOUD_VOXEL_SNAP`, `KEYFRAME_MIN_*`,
  `TSDF_DECAY`, `PRISM_RESET_EACH_BATCH`, `RESET_WINDOW_FRAMES`.
- `prism_session.py`: pushes knobs into the engine; experimental reset-each-batch.

### Engine (`server/mapping/PRISM-VGGT/`)
- `engine.py`: **keyframe gating** (`_keyframe_accept`) — integrate only when the
  camera moved; voxel-snap default; skip extraction on fully-gated windows; **decay
  only on submaps that integrated** (avoids eroding a 360° map).
- `tsdf.py`: introspective `decay()` — finds `decay_tsdf`/`decay`/`decay_occupancy`
  on the Mapper or its `_c_mapper`, logs the one used, raises with the candidate list
  if none.

### Client (`client/`)
- `block_sync.py`: subscribes `pcd/push` (apply immediately); manifest is
  repair/bootstrap only, with a `BLOCK_PUSH_GRACE_S` wait so the push wins the race.
- `prism_viewer.py`: trajectory+status on the **fast/control lane**; incremental
  render via `take_delta`+`IncrementalCloud` (fallback to merge); **URDF mesh avatar**
  with `U` toggle.
- `urdf_robot.py` (new): yourdfpy/trimesh loader + FK; fully guarded.
- `robot/docker/kinematics.py`: `LowStateTracker.get_joints()` exposes the 12 angles.

## Env flags (all reversible)

| Flag | Default | Effect |
|---|---|---|
| `CRC_QUANT_M` | `VOXEL_SIZE*0.5` | occupancy-CRC grid (unset `BLOCKMAP_CRC_QUANT_M` → legacy 1 mm) |
| `CLOUD_VOXEL_SNAP` | `1` | voxel-snap streamed surface |
| `KEYFRAME_MIN_TRANS_M` / `KEYFRAME_MIN_ROT_DEG` | `0.05` / `8` | keyframe gating (both `0` = old behaviour) |
| `TSDF_DECAY` | `1` | active carving via nvblox decay |
| `BLOCK_PUSH_GRACE_S` | `0.25` | client wait after manifest before repair pull |
| `PUSH_MAX_CUBES` | `400` | above this → manifest-only (client pulls) |
| `VIEWER_INCREMENTAL` | `1` | incremental client render |
| `PRISM_RESET_EACH_BATCH` | `0` | experimental fresh-rebuild each batch |
| `RESET_WINDOW_FRAMES` | `60` | frames reprocessed when reset-each-batch on |
| `GO2_URDF` | `""` | path to go2w_description URDF → enables mesh avatar |

Full rollback to pre-refactor behaviour:
`CLOUD_VOXEL_SNAP=0 KEYFRAME_MIN_TRANS_M=0 KEYFRAME_MIN_ROT_DEG=0 BLOCKMAP_CRC_QUANT_M= TSDF_DECAY=0 VIEWER_INCREMENTAL=0`

## Carving notes (decay)
- Called only when integration happened this submap, so a still robot freezes the
  map and a moving one refreshes observed surfaces while stale/ghost voxels decay.
- If your nvblox build names decay differently, the guard logs once and disables it.
  Find it on the box: `print([m for m in dir(mapper) if 'decay' in m.lower()])`.
- Too aggressive (map erodes) → lower the decay rate in `MapperParams`/decay args;
  too weak (ghosts persist) → call more often or raise the rate.

## Experimental reset-each-batch
`PRISM_RESET_EACH_BATCH=1` rebuilds from only the last `RESET_WINDOW_FRAMES` frames
each batch: no cross-batch accumulation/drift (no ghost build-up, no walking inside
walls) but reprocesses the window (slower) and keeps **no** global map. Good A/B vs
decay. Lower `RESET_WINDOW_FRAMES` if too slow.

## URDF avatar setup (client)
The description lives at `client/<robot>_description/` (e.g. `go2w_description`,
urdf/ + meshes/) and is **auto-detected** (prefers go2w) — no env var needed (`GO2_URDF` still overrides). Just:
```
cd client && uv sync     # installs yourdfpy + trimesh + pycollada (for .dae)
make viewer              # mesh shows automatically; press U to toggle mesh <-> skeleton
```
Verified locally: the URDF loads, all 12 leg joints map (FR/FL/RR/RL × hip/thigh/calf),
the .dae meshes resolve via the package:// path. Meshes are decimated at load
(`URDF_KEEP`, default 6%) — go2w goes 532k→41k verts — so posed geometry builds in
~0.5 ms/frame (cached + throttled to 20 Hz). Mirrored/rotated/
bent-wrong → tweak `cfg_from_q` (joint sign map) or `_base_to_z_up` in `urdf_robot.py`;
isolated so it can't affect anything else.

## Verified in-sandbox (NumPy + Draco; no GPU/Zenoh/display)
- `python common/vat_blockmap.py` — occupancy CRC: sub-cell jitter → **0 cubes
  change**; legacy 1 mm → many; carving → exact cube removed; push round-trip.
- `python common/vat_protocol.py`, `python common/vat_cloudbuffer.py` — pass.
- End-to-end `BlockGrid → pack_block_push → ClientBlockStore → take_delta →
  IncrementalCloud`: **stationary frame = 0 KB delta**; carve+explore propagate exactly.
- All 13 changed files byte-compile.

NOT runnable here: engine integration, decay, reset mode, live viewer, URDF mesh,
real transport — reasoned correct + flag-guarded; test on the rig.

## Rig test plan
1. Smoke: `python common/vat_blockmap.py && python common/vat_protocol.py && python common/vat_cloudbuffer.py`.
2. Server (`make mapping`): expect `BlockPub … push→…`, `[TSDF] nvblox decay → …()`.
   Drive then hold still: `cubes (+N/-M)` should fall to `+0/-0` at rest; `push_kb` small.
3. Viewer (`make viewer`): green fix flash + pose line + cloud move together; the
   client log should show **pushes**, not `repaired N/N`; `render stalls` stops climbing.
4. Carving: revisit / move past a wall — `-M` removals should appear and ghosts/“inside
   walls” should clear. If not, confirm the decay method name and tune the rate.
5. Avatar: press `U` for the URDF mesh.
6. A/B: try `PRISM_RESET_EACH_BATCH=1`; and the full-rollback env above.

---

# Round 3 — avatar scale, Open3D-style visuals, decay lifetime

From the second round of client logs (latency now good, cloud bounded ~300k):

- **Avatar scale.** The go2w mesh is true-metric (0.61×0.43×0.61 m at zero config, real
  Go2 proportions), so it is not intrinsically small — the *map* is slightly inflated by
  the VGGT metric scale (low floor confidence, `s≈0.59`), which makes a correct robot look
  small against it. The real fix is calibrating `CAMERA_HEIGHT` / the floor anchor; as a
  quick visual match, `URDF_SCALE` (default 1.0) uniformly scales the avatar.
- **Black outlines → Open3D look.** The point cloud and feet markers drew with a black
  edge ring (VisPy default), making everything look segmented. Now `edge_color == face_color`
  with `edge_width=0` and marker antialiasing → clean connected points. The robot mesh uses
  smooth shading with a brighter material, `cull_face=False`, and raised ambient so back
  faces are not near-black.
- **Decay as a tunable point "lifetime" (sliding window).** `TSDF_DECAY` (on) carves stale
  voxels; `DECAY_EVERY_N` (default 1) sets how often decay runs — higher = longer lifetime /
  gentler window, so you can trade drift-resistance vs map completeness without touching the
  nvblox decay rate. This bounds *map* error from pose drift but cannot fix the drift itself
  (VGGT has no online loop closure — that is the remaining architectural limit).

### Benchmark mode (full accumulated scene, no sliding window)
```
TSDF_DECAY=0 PRISM_RESET_EACH_BATCH=0 make mapping
```
Everything accumulates; nothing is carved or windowed — the full scene for SoTA comparison.

## New flags (round 3)
| Flag | Default | Effect |
|---|---|---|
| `URDF_SCALE` | `1.0` | uniform avatar scale (visual match to a mis-scaled map) |
| `URDF_KEEP` | `0.06` | mesh decimation target (lower = lower-poly avatar) |
| `DECAY_EVERY_N` | `1` | apply decay every N submaps (higher = longer point lifetime) |

Verified in-sandbox: all 13 files compile; self-tests pass; URDF loads go2w, decimates
532k→41k verts (~0.5 ms/frame), and `URDF_SCALE` scales the avatar as expected.
