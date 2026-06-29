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

---

# Round 4 — dynamic obstacles (backpack appears / old geometry carves)

Symptoms: a backpack placed on an already-mapped table never appeared, and old
geometry still never carved. Per the nvblox docs, this repo runs **static TSDF**
reconstruction; full dynamic detection (Dynablox freespace + occupancy layer) is a
ROS-node pipeline and is **not exposed in nvblox_torch**. For a TSDF map the supported
levers are (1) keep re-observing the region and (2) decay the TSDF. Two bugs blocked both:

1. **Decay was likely hitting the wrong layer.** `decay()` now introspects the build and
   **logs every decay/deallocate method available**, prefers `decay_tsdf` (TSDF carving),
   falls back to generic `decay`, and only uses `decay_occupancy` last — warning loudly if
   that is all there is, because decaying the unused occupancy layer does nothing to the
   TSDF mesh (the usual reason "old points never disappear"). It also calls
   `deallocate_fully_decayed_blocks` so freed blocks leave the mesh and the manifest. The
   call is signature-robust (no-arg or `mapper_id`). **Check the server log on startup for
   `nvblox decay → decay_tsdf()`** — if it says occupancy-only, that build can't TSDF-decay
   and you should use the sliding window (below).
2. **Keyframe gating starved new observations.** A near-static 360° robot stopped
   integrating, so a change in view was never seen. Added a **time escape**
   (`KEYFRAME_MAX_INTERVAL_S`, default 1 s): integrate at least that often even when still,
   so moved/new objects are observed and decayed in within ~1 s, while genuinely redundant
   frames are still skipped (occupancy-CRC keeps the bandwidth ~0 when nothing changed).

Also: the anti-erosion **edge mask** (drops depth discontinuities) can erase small/thin
objects. It is now env-tunable — lower `EDGE_DILATE_PX` (e.g. 3) and/or raise
`EDGE_REJECT_THRESH` to keep more of a small dynamic object. Default unchanged.

## New flags (round 4)
| Flag | Default | Effect |
|---|---|---|
| `KEYFRAME_MAX_INTERVAL_S` | `1.0` | force integration at least this often (catches dynamics); 0 disables |
| `EDGE_DILATE_PX` | `7` | depth-edge mask dilation; lower keeps more of small/thin objects |
| `EDGE_REJECT_THRESH` | `0.15` | depth step (m) treated as an edge; raise to keep more |
| `EDGE_SMOOTH_THRESH` | `0.08` | depth step (m) kept as smooth surface |

## If decay still cannot carve on your build
Use the sliding window — guaranteed dynamic behaviour without nvblox decay:
```
PRISM_RESET_EACH_BATCH=1 RESET_WINDOW_FRAMES=60 make mapping
```
The map is rebuilt from only recent frames each batch, so moved objects appear and stale
geometry is gone — at the cost of no persistent global map. For SoTA benchmarks (full
accumulated scene) keep `TSDF_DECAY=0 PRISM_RESET_EACH_BATCH=0`.

## Honest limit
Decay/sliding-window bound the *map* error; neither fixes **trajectory drift** (ending up
elsewhere after minutes). VGGT runs open-loop with no loop closure, so pose error
integrates without bound — closing the loop needs a pose-graph / place-recognition backend,
which is a separate project, not a flag.

---

# Round 5 — make reset/sliding-window the main path (world anchor)

Reset mode (`PRISM_RESET_EACH_BATCH=1`) gave clearly better geometry, but each fresh
reconstruction came out in its OWN arbitrary frame → the cloud rotated/jumped, the robot
floated until the next pose, and the cube diff churned massively (`+177/-167`) because the
whole map re-landed in different cubes. Both symptoms are one root cause: no consistent
world frame across reconstructions.

**Fix — persistent rigid world anchor.** Each fresh reset reconstruction is now re-anchored
into ONE world frame before streaming: we match the frames it shares with the previous batch
(by capture timestamp), best-fit a RIGID SE3 (`rigid_anchor_from_poses`, averages
`W·inv(P)` over the shared cameras + orthonormalises), and apply it to the cloud, trajectory,
and pose correction. Result:

- the cloud/robot stop rotating & jumping (consistent frame, robot stays attached);
- static geometry lands in the same cubes/occupancy → the occupancy-CRC delta collapses to
  the frontier (no more full resend) → the incremental renderer only touches changed cubes;
- geometry stays clean (still a fresh reconstruction — no TSDF accumulation), while the only
  thing that drifts is the rigid anchor (slow, bounded by the overlap), not the geometry.

Anchor math is unit-tested (recovers a known SE3 to 1e-15; stays orthonormal under noise).
Flag: `RESET_WORLD_ANCHOR` (default on with reset).

## Suggested settings for the reset path
```
PRISM_RESET_EACH_BATCH=1 RESET_WORLD_ANCHOR=1 RESET_WINDOW_FRAMES=60 make mapping
```
- If residual per-batch shimmer remains, raise `CRC_QUANT_M` (coarser occupancy → absorbs
  small recon variation → even smaller delta) or lower `RESET_WINDOW_FRAMES` (cheaper rebuild).
- `TSDF_DECAY` is irrelevant in reset mode (nothing accumulates); leave it off there.

## Remaining ideas (not yet done)
- **Distance-based window** (`RESET_WINDOW` by metres travelled, not frame count) so the
  window covers a consistent spatial extent regardless of speed.
- **Anchor smoothing**: low-pass the per-batch SE3 to remove micro-jitter.
- **Hybrid**: persistent coarse global map for context + fresh local window overlaid — keeps
  a full map while the local region stays crisp. Bigger project.
- **Drift/loop closure** is still the hard limit (VGGT is open-loop); the anchor bounds frame
  jumpiness but a pose-graph backend is needed for true global consistency.

---

# Round 6 — reset-mode latency: the bottleneck is RECOMPUTE, not transport

Server log in reset mode: `▣ batch … in 7.82s`, with the engine printing
`Processing Submap 0…1…2…3…4…5…6` and **re-initialising the C++ Nvblox Mapper every
batch**. Meanwhile `cloud → client 0–10 KB/s` and `render 55 fps, stalls ~22`.

**Diagnosis (not the octree / bandwidth / rendering):** reset re-runs **VGGT perception
on the whole 60-frame window every batch** (~7 windows × ~0.85 s ≈ 6 s of inference).
That is the 5+ s. The pose/`capture→display` spikes are a side effect — the single
server thread is pinned reprocessing, so frame intake and corrections stall (and a 7 s
batch means a VGGT pose correction only every 7 s, so dead-reckoning drifts between them).

## Fix 1 (implemented): perception cache
The VGGT forward is deterministic for a given set of frames, and consecutive batches
(reset especially) re-request mostly the SAME windows. `_timed_perception` now memoises
the forward keyed by window frame-identity (`PERC_CACHE_WINDOWS`, default 16, ~77 MB/window),
and the cache is **deliberately not cleared on reset()** — so a fresh rebuild reuses the
~6/7 windows it already inferred and runs the network only on the 1 genuinely new window.
Reset perception drops ~6 s → ~0.85 s. (Geometry re-integration of the window still runs,
~2–3 s; see Fix 2 for the rest.)

## Fix 2 (recommended — re-test now): ONLINE mode + working decay
`decay_tsdf` is now confirmed active in the log. That was the missing piece: online mode
(`PRISM_RESET_EACH_BATCH=0`) processes only the **one new window per batch** (~1 s, as the
online submaps in the log show) and decay carves stale/dynamic geometry — so it should now
give reset-like freshness at ~1/7th the latency, with a consistent frame (no anchor needed,
no jumping). The earlier "online looks worse" was with decay silently broken. **Try this
first** — it is the lowest-latency path:
```
PRISM_RESET_EACH_BATCH=0 TSDF_DECAY=1 DECAY_EVERY_N=1 KEYFRAME_MAX_INTERVAL_S=1 make mapping
```
If stale geometry lingers, decay harder (`DECAY_EVERY_N=1`, or raise the nvblox decay rate);
if it erodes too fast, `DECAY_EVERY_N=2`.

## Immediate knob (reset path)
`RESET_WINDOW_FRAMES=24` → ~3 windows instead of ~7 → roughly half the rebuild time, at the
cost of a shorter map. Combine with the perception cache.

## Where the time goes (per the log) — and the lever
| Stage | reset (60-frame) before | with perception cache | online + decay |
|---|---|---|---|
| VGGT perception | ~6 s (7 windows) | ~0.85 s (1 new) | ~0.85 s (1 new) |
| TSDF integrate + mesh | ~2–3 s (7 windows) | ~2–3 s | ~0.3 s (1 window) |
| transport + render | <0.1 s | <0.1 s | <0.1 s |
| **≈ batch latency** | **~7–8 s** | **~3–4 s** | **~1 s** |

Net: transport/rendering were never the problem; the cache halves reset latency, but
**online + working decay is the ~1 s path** and is worth testing before investing more in
reset. If reset's geometry is still preferred, the next step is a sliding-window TSDF rebuilt
from cached perception (re-integrate only recent windows) — keeps reset quality at ~online cost.
