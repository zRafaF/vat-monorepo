# `vat-record` — passive live-session recorder

Records the live VAT streams **without perturbing the session**, each stream
independently, every sample timestamped on one common clock — so a real-world example
video can be composed afterwards however the story needs.

Composition is **not** here: `compose.py`, the Rerun replay and the figure scripts live in
`uofa-2026-report/realworld/`, next to the report that cites them. This directory is the
capture side — the recorder, the full-res backfill and the console.

Built for the real-capture spec in `uofa-2026-report/PUBLICATION_ROADMAP.md` §3.2.

The narrative version of this page, with the operator runbook, lives in the docs:
**[Capture & record](../../docs/recording.md)**. This file is the reference for the
code.

```bash
# from the repo root
make record ARGS="--scene lab --trajectory-family loop --pass 1 \
                  --camera-height 1.152 --operator rafael"

make backfill ARGS="recordings/data/<session_id>"   # full-res twins, after the walk
make record-ui                                     # the browser console

make record-selftest        # offline: no robot, no Zenoh, no GPU

# then, in uofa-2026-report/realworld/ :
#   make info ARGS=<session>     make export ARGS=<session>     make replay ARGS=<session>
```

---

## Why it can be trusted not to disturb the session

| | |
|---|---|
| **No publishers.** | Not one live key is published to. `--dry-run` proves it: the plan is produced by attaching a stub session whose `declare_publisher` raises. |
| **Full-res costs no uplink.** | Full-resolution panoramas are pulled from the robot's *own* rolling archive by `seq`, not by raising the transmit resolution. `--where robot` opens Zenoh in **peer** mode, like `theta_camera.py` and `pose_fuser.py`, so scouting links the recorder directly to the camera process. Requesting `--panorama-fullres` from the cloud side is refused unless you pass `--force`. |
| **`--where robot` records only the robot's own streams.** | The Zenoh router lives on the *server*, so a recorder on the robot subscribing to `pcd/push`, `pcd/manifest`, `esdf_slice`, `status` or `trajectory` would drag them **inbound across the field link**, competing with the pose downlink. So the robot-side default is `panorama_transmit`, `panorama_fullres`, `periscope`, `poses` (the pose correction already flows down to the robot, and is 44 bytes). Asking for the bulk cloud streams there warns loudly, and manifest repair defaults **off** — a bootstrap pull is a reliable multi-megabyte transfer that also holds `BlockPublisher`'s lock while it collects. **Record the map from the cloud/client side, in a second recorder, at the same time.** |
| **Map keyframes are free.** | The recorder mirrors the map in a `vat_blockmap.ClientBlockStore` (exactly as the client does) and dumps *that* on a timer. A complete map state costs the server nothing. |
| **The only queries are a client's own.** | The manifest-diff repair pull on `{server}/pcd/blocks` — which is what makes a mid-session start yield a complete map — plus the robot's archive queryable. `--pointcloud-snapshot-query-s` adds server-side snapshot queries and is **off** by default because each one costs a full cloud extract. |
| **The periscope is taken as-is.** | The robot encodes it only while an operator client keeps a `ViewRequest` alive (`PERISCOPE_VIEWER_TIMEOUT_S`). The recorder never sends one, so it captures the periscope as the operator actually used it. An empty periscope capture means nobody was aiming it. |

---

## The common clock

The session clock is the **robot capture clock in nanoseconds** — the clock
`FrameInput.timestamp` rides on, end to end:

```
robot camera ts_ns → FRME header → IncomingFrame.timestamp → FrameInput.timestamp
                   → engine.get_poses() → SubmapResult.cam_ts → PCOR.timestamp_ns
```

Four wire messages carry that clock and are recorded **verbatim**:

| message | field |
|---|---|
| `FRME` camera frame | `timestamp_ns` — camera capture |
| `POSE` fused pose | `timestamp_ns` — estimator time |
| `PCOR` pose correction | `timestamp_ns` — capture time of the keyframe the cloud solved |
| `PSCF` periscope frame | `timestamp_ns` — slice capture |

The map transport (`pack_pcd`, `pack_manifest`, `pack_bundle`, `pack_block_push`,
`pack_trajectory`) carries **no timestamp at all** — only a `map_version`, and the
manifest/bundle not even that. Those records get:

* `wall_ns` / `mono_ns` — local arrival, always;
* `src_ts_ns` — arrival mapped onto the session clock via
  `vat_telemetry.ClockOffsetEstimator` (the same minimum-filter the mapping server and
  the viewer use), i.e. *when the client saw it*;
* `ts_src` — `source` | `derived` | `wall`, so an exact capture time is never confused
  with an estimate;
* `capture_ts_ns` — the **true capture time of that `map_version`**, pinned from
  `pose_correction` (exact) or the server `status` stream (approximate).

**Align maps on `capture_ts_ns`; use `src_ts_ns` when you want observed latency.**
`MANIFEST.json` carries the complete `version_pins` table, and the composer backfills
it into rows written before their pin existed (a submap's push is published before its
correction arrives).

---

## Modules

| file | what it owns |
|---|---|
| `vat_record.py` | the `vat-record` CLI: flags, session lifecycle, `meta.json`, safe Ctrl-C flush, `MANIFEST.json`, `--dry-run`, `--selftest` |
| `rec_config.py` | env + Zenoh keys (from `vat_protocol.keys`, never hard-coded) + the config hash / git / `vat.env` provenance |
| `rec_clock.py` | `SessionClock`: stamping, the robot↔local offset, the `map_version`→capture-time index |
| `rec_sinks.py` | `SessionWriter`, atomic blobs, CSV/JSONL/TUM indexes, `Budget` / `RingBudget`, `StreamStats` |
| `rec_base.py` | `StreamRecorder`: attach → tick → close, and the error-swallowing subscribe wrapper |
| `rec_frames.py` | panorama transmit + the full-res archive puller |
| `rec_periscope.py` | periscope elementary stream + timestamp sidecar + ffmpeg remux |
| `rec_cloud.py` | point cloud (blocks *and* snapshot modes), ESDF slices, server status |
| `rec_poses.py` | fused pose (TUM + JSONL), cloud corrections + gate metrics, camera trail |
| `backfill.py` | fetch full-res panoramas into a **finished** recording, from the robot's rolling archive — the recommended way to get 4K frames, since nothing is pulled during the walk |
| `ui.py` | browser console (Gradio): start/stop, live progress, reset the map, fetch full-res, zip + download past recordings |
| `pyproject.toml` | isolated uv project for the console (`cd tools/recorder && uv sync`); the CLI still runs in the client env |
| `fake_rig.py` | **test fixture:** a synthetic robot + cloud publishing the real wire messages on the real Zenoh keys, so the live path can be exercised without a robot (see below) |

Every module has a `_selftest()` and is runnable on its own, matching the convention in
`common/` and `server/mapping/`:

```bash
python tools/recorder/rec_clock.py
python tools/recorder/rec_cloud.py
python tools/recorder/vat_record.py --selftest     # all of them + end-to-end
```

The end-to-end self-test synthesises a 10-second session through the **real** wire
packers, drives a **virtual clock** with per-stream transport latencies (so the
derived-timestamp path is genuinely exercised rather than collapsed into a few
milliseconds of real time) and writes a real recording. If a copy of the composer is
importable it also composes the result, asserting that replaying the raw Draco pushes
reproduces the materialised keyframe exactly; since the composer moved to the report repo
that half is skipped by default and says so. Use `--keep-dir` and point that project at
the session it leaves behind.

---

## Verified against a live bus

The offline `--selftest` drives the handlers directly and never touches Zenoh, so the live
path is tested with `fake_rig.py` — a synthetic robot + cloud that publishes the real wire
messages on the real keys, including the archive queryable, the block-repair queryable and a
**genuine H.264 elementary stream** cut into per-frame access units.

```bash
# terminal 1 — router      cd server/router && ZENOH_LISTEN=tcp/127.0.0.1:7447 uv run python router.py
# terminal 2 — rig         make rig ARGS="--drop-pushes 0.35"
# terminal 3 — recorder    make record ARGS="--duration 30s --scene rig --camera-height 1.15"
```

That run confirmed, on a real router: subscriptions on every key; the archive
query/reply (`?seq=N`, `reply_err` on a miss) pulling real 3840×1920 twins; the
manifest-diff **repair pull healing a 35 %-lossy push stream** (7 pulls, 8 647 cubes);
`periscope.mp4` remuxing into decodable H.264; the two-recorder pattern running
simultaneously; `compose export` resolving all nine streams at 238/238 ticks; and a 4K
mp4 built from `frames/%06d.jpg`. Five bugs only a live bus could surface were found and
fixed this way — see the git log.

Note `--where robot` binds **no** inbound endpoint (`listen/endpoints: []`). A peer's
default listener is `tcp/[::]:0`, which fails to bind outright on a host without IPv6
(common in the robot container); the recorder is a pure consumer and never needs inbound
peers. `--zenoh-listen` overrides.

---

## On-disk layout

```
recordings/<session_id>/
  meta.json                      identity, capture metadata, config hash, clock epoch,
                                 every Zenoh key subscribed and queried
  MANIFEST.json                  per-stream health + derived cross-checks + version pins
  README.txt                     this layout, written into the session itself
  recorder.log

  panorama_transmit/frames/<seq>.jpg      encoded body, byte-exact off the wire
  panorama_transmit/frame_index.csv       seq, src_ts_ns, ts_src, wall_ns, mono_ns,
                                          wire_bytes, image_bytes, camera_height_m,
                                          width, height, latency_ms, file
  panorama_fullres/…                      same shape; same seq/ts/camera_height

  periscope/periscope.h264|hevc|mjpeg     elementary stream, byte-exact
  periscope_timestamps.csv                per frame: capture ts + (segment,
                                          byte_offset, byte_len) + aim + keyframe bit
  periscope.mp4                           ffmpeg remux at the mean rate (nominal timing)

  pointcloud/index.jsonl                  ordered index of every map artefact
  pointcloud/blocks/push_*.bin            raw BPSH frames
  pointcloud/blocks/manifest_*.bin        raw BMNF manifests
  pointcloud/blocks/repair_*.bin          raw BBND bundles from repair pulls
  pointcloud/keyframes/kf_*_v<N>.npz      complete map states (free, from the mirror)
  pointcloud/snapshots/*.bin              whole-map pack_pcd snapshots

  poses/robot_fused.tum                   evo-ready: timestamp tx ty tz qx qy qz qw
  poses/robot_fused.jsonl                 + velocity, acceleration, fix_quality, seq
  poses/cloud_correction.jsonl            corrections + recomputed gate metrics
  poses/trajectory.jsonl + trajectory/*.npy

  esdf/index.jsonl + esdf/slices/*.npz|.bin
  status/status.jsonl
```

`wire_bytes` is the full Zenoh payload (20-byte `FRME` header + image) — that is the
number to quote for the uplink. `image_bytes` is the encoded image alone.

### Reading a recording back

```python
import numpy as np, json
import vat_blockmap as bm, vat_protocol as proto   # repo's own unpackers

# a map keyframe
with np.load("recordings/<id>/pointcloud/keyframes/kf_000007_v112.npz") as z:
    xyz, rgb, version, capture_ns = (z["points"], z["colors"],
                                     int(z["map_version"]), int(z["capture_ts_ns"]))

# or replay the exact transport
store = bm.ClientBlockStore(cube_m=1.0)
for rec in map(json.loads, open("recordings/<id>/pointcloud/index.jsonl")):
    if rec["kind"] == "push":
        store.apply_push_bytes(open(f"recordings/<id>/{rec['file']}", "rb").read())
xyz, rgb = store.merged()
```

the composer's `replay_map()` does exactly that, plus the manifest-removal and snapshot
cases.

---

## Composition (in the report repo)

```bash
cd ../uofa-2026-report/realworld
make info      ARGS=<session>                          # what's in it, and how healthy
make export    ARGS="<session> --fps 10 --link hard"   # aligned per-tick assets
make export    ARGS="<session> --at panorama --map replay"
make periscope ARGS="<session> --decode"
make replay    ARGS=<session>                          # the native Rerun viewer
```

That project vendors frozen copies of `vat_protocol`, `vat_blockmap` and the periscope
decoder, so it reads these recordings without this checkout — see its
`vendor/README.md`. What follows is still written from the recorder's side, because it is
what the on-disk format guarantees.

### Full-res after the fact (preferred)

```bash
make backfill ARGS="recordings/<id> --dry-run"     # cost estimate
make backfill ARGS="recordings/<id> --every 2"     # resumable; skips what it has
```

Pulls the robot's archived twins for the frames the recording already has, writing them
into `panorama_fullres/` with `ts_src=source` (the original capture time) so the composer
cannot tell them from live-recorded frames. Zero realtime cost, and you pick the size
afterwards. `panorama_fullres/backfill.json` logs each run, including seqs the robot had
already evicted from its rolling window.

### A paired robot + cloud capture (live full-res, advanced)

`--where robot` deliberately excludes the map, so a full capture is two sessions. Merge them
at compose time — they share the session clock, so this is a per-stream union:

```bash
# in uofa-2026-report/realworld/
make info   ARGS="<cloud-id> --with <robot-id>"
make export ARGS="<cloud-id> --with <robot-id> --fps 10 --panorama fullres --link hard"
```

Streams recorded on **both** sides (the transmit panorama, the fused pose) are
de-duplicated by `seq`, preferring the copy whose blob is on disk. The map transport is
taken from **one** session only — interleaving two copies of pushes and manifest removals
would corrupt a replay — and the merge says so when it happens.

`export` writes `timeline.csv`, `timeline.jsonl` and a `README.md` explaining the
columns. Each row is one instant on the session clock with every stream resolved to
what belonged there, and a `*_dt_ms` per stream saying how far the chosen sample sits
from the tick — **check those before trusting an alignment.** Poses are *interpolated*
(LERP position, SLERP orientation via `vat_protocol.quat_slerp`) so `pose_dt_ms` is 0;
empty pose columns mean a real dropout the export refused to paper over with a stale
sample.

With `--link hard`, `frames/%06d.jpg` is a numbered hard-link view of the panorama, so:

```bash
ffmpeg -framerate 10 -i frames/%06d.jpg -c:v libx264 -pix_fmt yuv420p panorama.mp4
```

`ffmpeg` stops at the first gap in a `%06d` sequence and still exits 0, so a tick with no
frame of its own repeats the previous one and sets `linked_frame_repeat = 1` rather than
leaving a hole. If leading ticks precede any panorama frame at all, the export says so and
tells you which index to start from.

The timeline is clamped to the **window** in which every continuously-published stream has
data. Streams that are inherently patchy — the gated pose correction, and the decimated /
ring-evicted full-res panorama — are excluded from that calculation so they cannot truncate
the export. `--window full` keeps the whole capture instead, and either way the export warns
when a stream is substantially clipped.

---

## Known limits, stated rather than hidden

* **Onboard leg/IMU odometry at native rate is not recorded.** §3.2 asks for it, but it
  lives on the bridged ROS topics and needs CDR decode (`rosbags`). What *is* recorded
  is the fused pose that odometry produces, plus every cloud correction. For raw
  bridged topics use `tools/probe_robot_data.py`.
* **`PoseCorrectionGate` counters are not on the wire.** `MANIFEST.json` derives
  `suppressed_or_rejected = submaps_seen_in_status − corrections_published`, and each
  correction's gate quantities (Δt, jump, implied speed, rotation delta, the outlier
  threshold that applied) are recomputed in `poses/cloud_correction.jsonl`. The gate's
  own counters are only in the mapping server's log.
* **ESDF distances are recovered from a lossy colour ramp.** `nav_esdf` publishes
  distance→RGB, saturating at 0 m and 1 m with ~2 mm resolution between. Good for the
  video and a qualitative figure; not a substitute for `engine.get_esdf_slice`.
* **`pack_trajectory` is positions only** — no orientations, no per-pose timestamps,
  truncated to the newest `TRAJ_MAX_POSES` (300). For a timestamped trajectory use
  `poses/robot_fused.tum`.
* **The world anchor `T` is never published.** Streamed clouds, trajectory and ESDF are
  already in the persistent world frame, but if `world_anchor` re-fits on fewer than 3
  overlapping frames the frame can shift silently. `status/status.jsonl` and the server
  log are the only places such a discontinuity is explainable.
* **`pcd/push` and `pcd/manifest` are `CongestionControl.DROP`.** Losses are expected
  and are healed by the manifest-diff repair pull — keep `--pointcloud-repair` on when
  recording from the cloud side. It is off by default with `--where robot`, so a
  robot-side map recording *will* have holes; that is the intended trade.
* **`--fullres-ring` evicts blobs but keeps their index rows.** The row is still a true
  record of what the robot captured and what it cost (that is the uplink figure), so it
  stays; `MANIFEST.json` reports `evicted_by_ring`, and `compose.py` drops rows whose
  blob is gone when it loads a recording, so it can never hand you a dead path.
* **`periscope.mp4` has nominal timing.** The stream is variable-rate;
  `periscope_timestamps.csv` is authoritative.
* **At least one low-latency source-stamped stream is required** (`--poses`,
  `--panorama-transmit` or `--periscope`), and the recorder now refuses to start without
  one. The map carries no timestamps, so its session time is arrival minus the robot→local
  offset — and that offset is only learnable from a stream whose arrival closely follows
  its capture. `pose_correction` and the lagged full-res pull are both stamped verbatim but
  deliberately **excluded** from the baseline for exactly this reason.
* **DracoPy is required** for the map mirror, repair and keyframes. Without it every
  byte is still recorded, but understanding it is deferred to offline replay — the
  recorder says so loudly at startup. It is already a `client/` dependency.

---

## Requirements

Runs in the client env (`cd client && uv sync`), which already provides
`eclipse-zenoh`, `DracoPy`, `numpy`, `opencv-python` and `av` (PyAV, for periscope
decode). `ffmpeg` on `PATH` is optional (used to remux the periscope). `open3d` is
optional and only needed for `compose.py render`.

On the robot, `--panorama-fullres` needs `ARCHIVE_ENABLE=true` (it is, in `vat.env`)
and enough space under `ARCHIVE_DIR`.
