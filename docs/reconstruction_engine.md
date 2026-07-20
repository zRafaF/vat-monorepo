# Reconstruction Engine (PRISM-VGGT)

PRISM-VGGT is the scientific core of VAT: the engine that turns the robot's 360° video into
a live, metric, globally-consistent 3D map. It lives in its own repository and is vendored
into this monorepo as a git submodule at `server/mapping/PRISM-VGGT`; the VAT
[mapping server](setup/server.md) is a thin Zenoh wrapper that drives it online.

> **PRISM** = **P**anoramic **R**econstruction with **I**ncremental **S**LAM and dense
> **M**odeling. Repository: <https://github.com/zRafaF/PRISM-VGGT>.

## What it is (in one paragraph)

PRISM-VGGT wraps a **frozen** panoramic depth-and-pose network (PanoVGGT) in a
**training-free** streaming engine. It processes 360° video in overlapping sliding-window
submaps; each submap's feed-forward geometry is aligned into a single metric,
gravity-levelled world frame by a similarity (Sim(3)) anchor computed from the frames the
windows share, with metric scale grounded by detecting the floor and using the robot's known
camera height. The aligned depth is reprojected from the equirectangular sphere onto six
virtual pinhole cube faces and fused into an [nvblox](https://github.com/nvidia-isaac/nvblox)
TSDF, and the map is exposed as a versioned, per-block-hashed point cloud so the client only
ever downloads the blocks that changed.

Nothing is ever trained. A frozen PanoVGGT checkpoint supplies per-window geometry; PRISM does
all the alignment, fusion, and streaming around it.

## The pipeline, stage by stage

```
360° video  ─▶  sliding-window submaps (window/overlap)
                    │
                    ▼
             PanoVGGT (frozen)  ──▶ per-frame depth + camera poses (native units)
                    │
                    ▼
             metric scale from floor (RANSAC plane + known camera height)
                    │
                    ▼
             Sim(3) anchor into the persistent world frame  +  drift guards
                    │
                    ▼
             equirectangular → 6 pinhole cube faces  ──▶ nvblox TSDF fusion
                    │
                    ▼
             versioned block point cloud  +  camera trajectory  +  ESDF slice
```

### 1. Sliding-window submaps

Video is processed in windows of `WINDOW_SIZE` frames that step forward by
`WINDOW_SIZE − OVERLAP`, so consecutive windows share `OVERLAP` frames. Those shared frames are
the constraint that stitches one submap to the next. The engine runs in two modes: an
**offline/batch** mode (wipe and process everything — used by the interactive Gradio app and
the benchmarks) and the **online/streaming** mode VAT uses (keep the accumulated map, only
process unseen windows).

For low latency, perception and fusion run concurrently as a **one-deep double buffer**: while
window *k* is being fused into the TSDF, window *k+1* is already running PanoVGGT inference on
the GPU. A small cache avoids re-running inference on re-requested windows.

### 2. Perception — PanoVGGT (frozen)

PanoVGGT is a fork of [VGGT](https://github.com/facebookresearch/vggt) adapted for
equirectangular input; it is a nested submodule inside PRISM-VGGT
(`third_party/PanoVGGT`, checkpoint `YijingGuo/PanoVGGT`). For each window it returns per-frame
camera poses, dense local point maps, and depth. PRISM consumes this feed-forward geometry as
given — no fine-tuning.

### 3. Metric scale from the floor

VGGT-family networks are scale-ambiguous. PRISM recovers **absolute metric scale without
LiDAR**: it isolates the lower hemisphere of the local point cloud, fits a floor plane by
RANSAC, checks the plane normal is near-vertical, and sets
`scale = known_camera_height / distance_to_floor`. Scale is stabilised over the first few
confident windows (`SCALE_WARMUP_WINDOWS`, using the running median rather than locking on the
first estimate) and then held, because a freely-varying per-submap scale compounds into map
inflation and ghosting. This is a genuine contribution — and, honestly, the main accuracy
limitation (see [What holds up and what doesn't](#what-holds-up-and-what-doesnt)).

### 4. Alignment into the world frame

Each new submap must be registered into the one persistent world frame. PRISM estimates the
transform from the overlap frames' camera centres and orientations. The **alignment group** is
configurable via `PRISM_ALIGN`:

| Group | DoF | Meaning |
|---|---|---|
| `sim3` | 7 | rotation + translation + one global scale (**VAT default**) |
| `se3` | 6 | rigid only; assumes scale is already grounded |
| `sl4` | 15 | full projective homography (the group VGGT-SLAM uses) |

This is where PRISM most diverges from [VGGT-SLAM](https://github.com/MIT-SPARK/VGGT-SLAM).
VGGT-SLAM aligns *uncalibrated pinhole* submaps on the SL(4) projective manifold inside a GTSAM
factor graph, because pinhole submaps can differ projectively. A fully panoramic 360° sensor
has **no focal/intrinsic ambiguity**, so that projective freedom collapses and a cheap Sim(3)
similarity anchor is sufficient — **no SL(4), no GTSAM, no fork**. Drift is instead controlled
by a stack of lightweight online guards (scale warm-up/lock, overlap-pose pinning, a flip
guard, and a floor re-levelling guard) that substitute for a factor graph.

!!! tip "Why Sim(3) is the default"
    The [PRISM-benchmarks](#benchmarks-the-why-behind-the-choices) alignment-group ablation
    holds the backbone, fusion, and trajectory fixed and varies only the registration group.
    SL(4)'s extra projective freedom fits clean, well-overlapped motion slightly better, but it
    accumulates non-rigid drift when the path **loops** — exactly the failure Sim(3) and SE(3)
    forbid. On smooth and stop-and-go motion Sim(3) is within noise of SL(4); on loops it is
    materially more robust. Real deployments loop, so **VAT defaults to `sim3`**. (The
    PRISM-VGGT submodule's own built-in default is `sl4`; the VAT monorepo overrides it to
    `sim3` — see [Server setup](setup/server.md).) SE(3) is near-identical to Sim(3) once scale
    is grounded.

### 5. Panoramic TSDF fusion (equirectangular → cubemap → nvblox)

nvblox's depth integrator is **pinhole-only** — it cannot ingest a 360° panorama directly. So
PRISM reprojects the sphere onto **six virtual 90°-FOV pinhole cube faces** (`FACE_SIZE` px per
face), converts radial depth to per-face optical depth, crops the seam margins, and calls
nvblox once per face with the face-rotated pose. A depth-edge mask drops pixels at depth
discontinuities so that smeared network depth edges don't carve false surfaces. This
equirect → cubemap → nvblox bridge is one of the clearest standalone engineering
contributions.

The mapper runs in a **reset/hybrid** mode (`PRISM_RESET_EACH_BATCH=1`): it rebuilds a fresh
mini-map from the most recent frames each batch and rigidly re-anchors it into the persistent
world frame (an SE(3), scale-1 fit shared with the previous batch — deliberately *not* a loop
closure), doing a full reset every `RESET_PERIOD_SUBMAPS` batches and extending online in
between. This keeps the accumulated map clean instead of thick and duplicated.

### 6. Outputs

The engine exposes, over Zenoh (see the [Wire Protocol](reference/wire_protocol.md) for keys
and byte layouts):

- a **versioned, per-block-hashed point cloud** — the client fetches a snapshot once, then
  pulls only the blocks whose hash changed, and resyncs cleanly after dropped updates
  (`STREAM_MODE=blocks`);
- the **camera trajectory**;
- a slow **VGGT pose correction** sent down to the robot's fuser;
- an **ESDF slice** (`COMPUTE_ESDF=1`) — a horizontal Euclidean Signed Distance Field, the
  substrate for the planned autonomous navigation (see the [Roadmap](roadmap.md));
- a JSON **status/telemetry** heartbeat.

The world frame is Z-up, right-handed (ROS REP-103 / nvblox convention): floor at Z=0, an
upright camera at Z ≈ camera height.

## Benchmarks: the "why" behind the choices

PRISM-VGGT is evaluated against streaming and offline baselines by a separate harness, the
**PRISM-benchmarks** repository, which owns dataset rendering, fair co-visibility masking, and
metric collection (trajectory ATE/drift, reconstruction F-score, cloud cleanliness and size,
performance/VRAM, and absolute metric-scale accuracy). We deliberately keep the numbers out of
this documentation and out of the report's headline claims, because at the time of writing they
are **preliminary** — the clean runs cover only large, hard scenes, there is no small/easy room
yet, and there are no error bars. They are cited here only to justify design decisions.

### What holds up and what doesn't

Stated honestly, so the docs don't over-claim:

**Defensible.** PRISM beats VGGT-SLAM (its direct competitor) by a wide margin on trajectory
error with a much smaller map; it produces the **cleanest and most compact** maps of the
methods tested and runs within a **bounded (~15 GB) VRAM budget** in real time; and on
non-looping motion its trajectory is competitive with *offline* full-batch networks. It is the
only streaming panoramic method that is metric.

**Does not hold up.** Raw reconstruction F-score **trails offline models** — PRISM's edge is
cleanliness and compactness, not peak surface accuracy (the streaming/fusion tax). The
metric-scale accuracy is good on smooth motion but **degrades on loops** and on hard scenes; it
is not survey-grade. **Loop closure is a real gap** — DINOv2+SALAD retrieval was tried and
degraded on panoramas (wide FOV, repeated features), so the shipped pipeline is
**non-looping**, and error grows on long loops. Closing loops (a GTSAM Sim(3) pose graph) is
future work — see the [Roadmap](roadmap.md).

## Relationship to VAT

PRISM-VGGT is the *scientific* contribution (novel panoramic streaming reconstruction); VAT is
the *engineering* platform around it (asynchronous telepresence: transport, robot-authoritative
pose, client prediction, teleop). The engine does no networking itself — the VAT
[mapping server](setup/server.md) drives it, feeds it decoded camera frames, and publishes its
outputs over Zenoh.
