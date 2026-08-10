# Capture & record

`vat-record` taps a **live** VAT session and writes every stream to disk, independently
and timestamped on one common clock, without disturbing the session. It exists so we can
walk the robot through a real space once and then compose a real-world example video —
panorama, point cloud, trajectory, periscope, ESDF — however we want afterwards, rather
than deciding the layout in advance and re-running the capture when we change our mind.

```bash
# on the client machine, with router + robot + mapping server running
make record ARGS="--scene lab --trajectory-family loop --pass 1 \
                  --camera-height 1.152 --operator rafael"
# … drive the robot … Ctrl-C to stop cleanly

make compose ARGS="info   recordings/20260808-201500_lab_loop_p1"
make compose ARGS="export recordings/20260808-201500_lab_loop_p1 --fps 10"
```

Everything lands in `recordings/<session_id>/`. The tool lives in
[`tools/recorder/`](https://github.com/zrafaf/vat-monorepo/tree/main/tools/recorder) and
its `README.md` there is the code-level reference.

---

## How it taps Zenoh passively

Every stream the recorder wants is already on the bus, published for someone else. So the
recorder is **a subscriber and nothing more** — it never publishes on a live key, which
means it cannot change what the robot sends, what the mapping server computes, or what
the operator's client sees.

```
        ROBOT                        CLOUD                       CLIENT
  camera/frame  ────────────────────▶ mapping_server ──▶ pcd/push, pcd/manifest ──▶ viewer
  prism/pose    ─────────────────────── router ──────────────────────────────────▶ viewer
  periscope/frame ──────────────────────────────────────────────────────────────▶ viewer
                                          │
                          ┌───────────────┴───────────────┐
                          │   vat-record  (subscriber)     │   ← no publishers
                          │   + query: archive/get         │
                          │   + query: pcd/blocks (repair) │
                          └───────────────────────────────┘
```

You can verify this for any invocation before running it:

```bash
python tools/recorder/vat_record.py --dry-run --all --where robot
```

`--dry-run` builds the real recorders and attaches a **stub** Zenoh session whose
`declare_publisher` raises, then prints exactly which keys would be subscribed and which
queried. What it prints is what runs.

!!! info "Keys come from the schema, never from strings"
    Every key is resolved through
    [`vat_protocol.keys()`](reference/wire_protocol.md) with the same `ROBOT_NAME` /
    `SERVER_PREFIX` the rest of the system uses, so the recorder cannot drift from the
    wire protocol. `tools/recorder/rec_config.py` mirrors `mapping_config.py`'s
    env-first convention.

### The two queries, and why they are legitimate

**The robot's frame archive** — `{robot}/prism/camera/archive/get`. The robot already
keeps a full-resolution twin of every transmitted frame in a local rolling archive
(`ARCHIVE_ENABLE=true`), tagged with the same `seq` / `ts_ns` / `camera_height`. Pulling
from there is how we get 4K panoramas **without raising the transmit resolution and
without loading the field link.**

**The block-repair pull** — `{server}/pcd/blocks`. `pcd/push` and `pcd/manifest` are
`CongestionControl.DROP`, so a recorder loses pushes exactly as a client does. The
recorder therefore does what `client/vat_client/block_sync.py` does: mirror the map,
diff each manifest against the mirror, and pull what is missing. This is also what makes
a **mid-session start** yield a complete map — the first manifest diff pulls everything.

Map *keyframes* need no query at all: the recorder already holds the whole map in its
mirror, so it dumps that on a timer. A complete map state costs the server nothing.

!!! warning "Full-res belongs on the robot"
    `--panorama-fullres` is refused from the cloud side unless you pass `--force`. Run
    the recorder **on the robot** (`--where robot`) — it then opens Zenoh in `peer` mode,
    like `theta_camera.py` and `pose_fuser.py`, so scouting links it directly to the
    camera process and the frames stay on the local bus.

!!! warning "`--where robot` records only the robot's own streams — record the map separately"
    The router lives on the **server**. A recorder on the robot subscribing to
    `pcd/push`, `pcd/manifest`, `esdf_slice`, `status` or `trajectory` would drag all of
    them *inbound* across the field link, competing with the pose downlink — the exact
    opposite of the point.

    So the robot-side default is `panorama_transmit`, `panorama_fullres`, `periscope`,
    `poses` (the pose correction already flows down to the robot, and is 44 bytes).
    Asking for the bulk cloud streams there warns loudly, and manifest repair defaults
    **off**, because a bootstrap pull is a reliable multi-megabyte transfer that also
    holds `BlockPublisher`'s lock while it collects the cubes.

    **For a full capture, run two recorders at once** — one on the robot for the
    full-resolution panorama, one on the cloud/client side for the map, poses and
    periscope. Give them the same `--scene` / `--trajectory-family` / `--pass` so the two
    sessions are obviously a pair; both are on the same session clock, so they compose
    together.

!!! note "The periscope is taken, not requested"
    The robot encodes the periscope only while an operator client keeps a `ViewRequest`
    alive (`PERISCOPE_VIEWER_TIMEOUT_S`, 5 s). The recorder never sends one, so it
    captures the periscope **as the operator actually used it** — which is what a demo
    video wants. If nobody was aiming the periscope, that stream is empty, and
    `MANIFEST.json` says so.

---

## The common clock

This is the part that makes composition possible, so it is worth being precise about.

The session clock is the **robot capture clock in nanoseconds** — the clock
`FrameInput.timestamp` rides on, all the way through the pipeline:

```
robot camera ts_ns → FRME header → IncomingFrame.timestamp → FrameInput.timestamp
                   → engine.get_poses() → SubmapResult.cam_ts → PCOR.timestamp_ns
```

Four messages carry that clock, and the recorder writes it **verbatim** — never
re-stamping a sample with its arrival time:

| stream | timestamp source |
|---|---|
| `FRME` camera frame | `timestamp_ns` — camera capture |
| `POSE` fused pose | `timestamp_ns` — estimator time |
| `PCOR` pose correction | `timestamp_ns` — capture time of the keyframe the cloud solved |
| `PSCF` periscope frame | `timestamp_ns` — slice capture |

The map transport carries **no timestamp** — `pack_pcd`, `pack_manifest`, `pack_bundle`,
`pack_block_push` and `pack_trajectory` have only a `map_version` between them, and the
manifest and bundle not even that. For those the recorder writes three things instead of
inventing one:

* **`wall_ns` / `mono_ns`** — local arrival, always recorded.
* **`src_ts_ns`** — arrival mapped onto the session clock using
  `vat_telemetry.ClockOffsetEstimator`, the same minimum-filter the mapping server and
  the viewer already use. Because the filter removes only the *baseline* offset, this is
  honestly "when the client saw it, on the session clock".
* **`ts_src`** — `source`, `derived` or `wall`, so an exact capture time is never
  mistaken for an estimate.

And, crucially, a fourth for map records:

* **`capture_ts_ns`** — the *true* capture time of that `map_version`, pinned from
  `pose_correction` (exact: it carries both a keyframe capture time and a map version)
  or from the server `status` stream's `newest_frame_robot_ns` (approximate). An exact
  pin never gets overwritten by an approximate one.

!!! tip "Which timestamp to use"
    **Align on `capture_ts_ns`** (falling back to `src_ts_ns`) — that puts the map where
    the robot actually was. **Use `src_ts_ns` for latency claims** — that is when the
    map reached the client. `compose.py` does the former by default and reports both.

    `MANIFEST.json` holds the complete `version_pins` table. A submap's push is
    published before its correction arrives, so the earliest rows of each version have
    no pin inline; `compose.py` backfills them from that table.

---

## Recording a session for the paper

The capture spec is `PUBLICATION_ROADMAP.md` §3.2. Work through it in this order.

### 1. Bring the system up

Follow the [Bring-up Runbook](bringup.md) until Stage 3 is green (trajectory + fused
pose tracking). The recorder needs the router, the robot container and `make mapping`
running; it needs no GPU of its own.

### 2. Measure the camera height

This is the **metric-scale anchor**: PRISM grounds absolute scale on the camera's height
above the floor. Measure it with a tape, floor to lens centre, with the robot standing,
and pass it:

```
--camera-height 1.152 --camera-height-source "tape, floor→lens centre, standing"
--mount-geometry "rear selfie-stick, rigid"
```

The recorder also captures the per-frame `camera_height` from every `FRME` header and
summarises min/mean/max in `MANIFEST.json`, so the anchor is recorded even if you forget
the flag — and so you can cross-check the flag against what the robot actually sent.
Note `vat.env` sets `CAMERA_HEIGHT_M=1.15`, which is a *configured* value; the roadmap
wants a *measured* one, per session.

### 3. Start over clear, flat, visible floor

The metric-scale plane fit needs floor to see. Record whether you did:
`--clear-flat-floor` (or `--no-clear-flat-floor`, which logs a warning that scale may
take longer to commit or fail).

### 4. Start the recorder, then drive

```bash
make record ARGS="--scene corridor-3rd-floor --trajectory-family stop-and-go \
                  --pass 2 --camera-height 1.152 --operator rafael \
                  --note 'fluorescent lighting, two people walked through' \
                  --duration 5m"
```

Cover the three motion families from §3.2, **two or more passes each** so there is a
variance estimate:

| `--trajectory-family` | what to walk |
|---|---|
| `smooth` | one continuous walkthrough, steady pace |
| `stop-and-go` | walk, stop, look around, walk on |
| `loop` | return to the start, so real loop drift is visible |

Then Ctrl-C. The recorder flushes every index, writes a final map keyframe, remuxes the
periscope and writes `MANIFEST.json`. **A recording killed mid-session stays usable** —
index files are flushed per record and blobs are published atomically.

### 5. Full-resolution panoramas (optional, on the robot)

```bash
# ON THE ROBOT — the robot's own streams only (see the warning above)
cd client && uv run python ../tools/recorder/vat_record.py \
    --where robot --panorama-transmit --panorama-fullres \
    --fullres-max-size 8GB --fullres-every 2 --duration 5m \
    --session-id 20260808_corridor_loop_p2_robot \
    --scene corridor-3rd-floor --trajectory-family loop --pass 2 \
    --camera-height 1.152

# AT THE SAME TIME, on the client/server side — the map, poses and periscope
make record ARGS="--session-id 20260808_corridor_loop_p2_cloud \
                  --scene corridor-3rd-floor --trajectory-family loop --pass 2 \
                  --camera-height 1.152 --duration 5m"
```

Full-res 360° is large (`THETA_MODE=4K` → 3840×1920 JPEG at quality 92). It is off by
default and gated behind duration and byte caps. `--fullres-ring` (on by default) keeps
the **newest** window when the cap is reached instead of truncating the tail, and
`--fullres-every N` pulls every Nth frame so a long walk still fits.

!!! note "The archive is best-effort"
    `robot/docker/frame_archive.py` writes on its own thread and *drops* frames under
    back-pressure — realtime is sacred, the archive is not. Pulls are therefore lagged
    (`--fullres-lag`, 2 s) and a miss is recorded as a skip, not an error.
    `MANIFEST.json` reports `archive_misses`.

### 6. Ground truth, if you want real numbers

Also from §3.2, in increasing rigour: tape/laser-measure a handful of real dimensions
(door width, room length, ceiling height) and compare them against the reconstruction
for a real metric-scale error; or capture the same space with a reference scanner and
compute accuracy/completeness; or log real ATE against a motion-capture volume. Record
what you did with `--note` and `--meta`:

```
--meta gt_method=tape --meta gt_door_width_m=0.912 --meta gt_room_length_m=7.44
```

---

## What lands on disk

```
recordings/<session_id>/
  meta.json            identity, capture metadata, config hash, clock epoch,
                       every Zenoh key subscribed and queried
  MANIFEST.json        per-stream health, derived cross-checks, version pins
  README.txt           the layout, written into the session itself
  recorder.log

  panorama_transmit/   frames/<seq>.jpg (byte-exact) + frame_index.csv
  panorama_fullres/    same shape, same seq/ts/camera_height
  periscope/           periscope.h264|hevc|mjpeg — the elementary stream, byte-exact
  periscope_timestamps.csv   AUTHORITATIVE timing: per frame the capture ts +
                             (segment, byte_offset, byte_len) + aim + keyframe bit
  periscope.mp4        ffmpeg remux at the mean rate (nominal timing only)

  pointcloud/index.jsonl        ordered index of every map artefact
  pointcloud/blocks/*.bin       raw pushes / manifests / repair bundles
  pointcloud/keyframes/*.npz    complete map states (points, colors, map_version,
                                capture_ts_ns) — free, from the recorder's mirror
  pointcloud/snapshots/*.bin    whole-map pack_pcd snapshots

  poses/robot_fused.tum         evo-ready: timestamp tx ty tz qx qy qz qw
  poses/robot_fused.jsonl       + velocity, acceleration, fix_quality, seq
  poses/cloud_correction.jsonl  corrections + recomputed gate metrics
  poses/trajectory.jsonl        the streamed camera trail (positions only)

  esdf/index.jsonl + esdf/slices/*.npz|.bin
  status/status.jsonl           map_version↔time pins + the measured uplink
```

Two columns worth knowing: **`wire_bytes`** in the panorama index is the full Zenoh
payload (20-byte `FRME` header + image) and is the number to quote for the uplink;
`image_bytes` is the encoded image alone. `MANIFEST.json → derived.uplink` reports both
what the recorder counted and what the server measured (`robot_kbps`), which should
agree to within the server's throughput EMA.

Because the raw wire bytes are kept byte-exact, offline tooling replays them with the
repo's **own** unpackers (`vat_blockmap.unpack_block_push`, `unpack_manifest`,
`unpack_bundle`, `vat_protocol.unpack_pcd`) — there is no second serialisation to drift
out of sync. `compose.py` asserts this in its self-test: replaying the recorded pushes
reproduces the materialised keyframe exactly.

---

## Composing

```bash
python tools/recorder/compose.py info    recordings/<id>
python tools/recorder/compose.py export  recordings/<id> --fps 10 --link hard
python tools/recorder/compose.py export  recordings/<id> --at panorama --map replay
python tools/recorder/compose.py periscope recordings/<id> --decode
python tools/recorder/compose.py render  recordings/<id> --out demo.mp4    # Open3D
```

`export` is the one to reach for. It builds a timeline — uniform at `--fps`, or one tick
per panorama frame with `--at panorama` — and resolves every stream at every tick:

* **panorama** — nearest frame, with `panorama_transmit_dt_ms` reporting the error;
* **pose** — *interpolated* to the exact tick (LERP position, SLERP orientation via
  `vat_protocol.quat_slerp`), because the pose stream is 30 Hz and interpolating beats
  snapping. Empty pose columns mean a real dropout that the export refused to paper over
  with a stale sample;
* **map** — the nearest materialised keyframe, or with `--map replay` the exact state
  produced by replaying the recorded pushes up to that instant;
* **periscope** — `(segment, byte_offset, byte_len)`, so you can slice the exact encoded
  frame out of the elementary stream;
* **ESDF slice** and **camera trail** — nearest.

Output is `timeline.csv` + `timeline.jsonl` + a `README.md` documenting the columns, and
with `--link hard` a numbered `frames/%06d.jpg` view of the panorama:

```bash
ffmpeg -framerate 10 -i frames/%06d.jpg -c:v libx264 -pix_fmt yuv420p panorama.mp4
```

!!! tip "Always read the `*_dt_ms` columns"
    They are how far each chosen sample sits from its tick. A large value means that
    stream had a gap there — the alignment is telling you the truth, not failing.

The timeline is clamped to the window in which every continuously-published stream has
data. Streams that are patchy by nature — the gated pose correction, and the decimated,
ring-evicted full-res panorama — are deliberately excluded from that calculation so they
cannot truncate the export; `--window full` keeps the whole capture instead. Either way
the export **warns** when a stream is substantially clipped, so a late-starting stream
never silently shortens your video.

One more honesty detail: `ffmpeg` stops at the first gap in a `%06d` sequence and still
exits 0. A tick with no panorama frame of its own therefore repeats the previous one and
sets `linked_frame_repeat = 1`, so `frames/` is always gap-free and the repeat is visible
in the timeline rather than invisible in the video.

`render` is the optional convenience path: it offscreen-renders the point cloud and the
camera trail with Open3D in an oblique "toy box" view, composites the panorama and
periscope panels, and muxes with ffmpeg. `--layout {cloud, cloud+panorama, quad}`.
Open3D is **not** a default client dependency (the live viewer uses VisPy), so install it
only if you want this: `cd client && uv pip install open3d`. If you would rather cut the
video by hand, `export` already gave you everything.

---

## Checking it without a robot

Two levels. The offline self-test drives the recorder's handlers directly:

```bash
make record-selftest
```

Synthesises a ten-second session through the real wire packers, driving a virtual clock
with per-stream transport latencies so the derived-timestamp path is genuinely exercised;
writes a real recording; then composes it and asserts the cross-checks — including that
replaying the raw Draco pushes reproduces the materialised keyframe exactly. No robot, no
Zenoh, no GPU.

```bash
# keep the synthetic recording and poke at it with compose.py
python tools/recorder/vat_record.py --selftest --selftest-keep /tmp/vatrec
python tools/recorder/compose.py info /tmp/vatrec/e2e
```

The second level exercises the **live** path — real Zenoh subscriptions, the archive
query/reply, the block-repair pull, a real H.264 stream — using `fake_rig.py`, a synthetic
robot + cloud that publishes the real wire messages on the real keys:

```bash
# [SERVER] make router
make rig ARGS="--drop-pushes 0.35"          # sheds 35 % of pushes to exercise repair
make record ARGS="--duration 30s --scene rig --camera-height 1.15"
make compose ARGS="info recordings/<id>"
```

This is the right thing to run after touching the recorder and before taking the robot out.
It is also how the live path was validated in the first place: the rig run proved the
archive pull, the repair pull healing a lossy link, a decodable `periscope.mp4`, the
two-recorder pattern, and a 4K video out of `compose export` — and it found five bugs the
offline test could not see.

!!! note "Peer mode binds nothing"
    `--where robot` uses Zenoh peer mode but sets `listen/endpoints: []`. A peer's default
    listener is `tcp/[::]:0`, which fails to bind on a host without IPv6 — common in the
    robot container — and the recorder never needs inbound peers. `--zenoh-listen`
    overrides if you want one.

---

## Limits worth knowing before you rely on a recording

* **Onboard leg/IMU odometry at native rate is not recorded.** §3.2 asks for it, but it
  lives on the bridged ROS topics and needs CDR decode. What *is* recorded is the fused
  pose that odometry produces, plus every cloud correction. For raw bridged topics use
  `tools/probe_robot_data.py`.
* **`PoseCorrectionGate` counters are not on the wire.** `MANIFEST.json` derives
  `suppressed_or_rejected = submaps_seen_in_status − corrections_published`, and each
  correction's gate quantities are recomputed in `poses/cloud_correction.jsonl`. The
  gate's own counters live only in the mapping server's log.
* **ESDF distances come back through a lossy colour ramp** — `nav_esdf` publishes
  distance→RGB, saturating at 0 m and 1 m with ~2 mm resolution between. Fine for the
  video and a qualitative figure; not a substitute for `engine.get_esdf_slice`.
* **`pack_trajectory` is positions only**, truncated to the newest `TRAJ_MAX_POSES`
  (300). For a timestamped trajectory use `poses/robot_fused.tum`.
* **The world anchor `T` is never published.** The streamed cloud, trajectory and ESDF
  are already in the persistent world frame, but if `world_anchor` re-fits on fewer than
  three overlapping frames the frame can shift silently. `status/status.jsonl` and the
  server log are the only places such a discontinuity is explainable.
* **`periscope.mp4` has nominal timing** — the stream is variable-rate;
  `periscope_timestamps.csv` is authoritative.
* **`--fullres-ring` evicts blobs but keeps their index rows** — the row is still a true
  record of what the robot captured and what it cost, so it stays; `MANIFEST.json` reports
  `evicted_by_ring` and `compose.py` ignores rows whose blob is gone.
* **DracoPy is required** for the map mirror, repair and keyframes. Without it every byte
  is still recorded but understanding it is deferred to offline replay; the recorder says
  so loudly at startup. It is already a `client/` dependency.

---

## Flags at a glance

`python tools/recorder/vat_record.py --help` is authoritative. The ones that matter most:

| flag | |
|---|---|
| `--panorama-transmit` / `--panorama-fullres` / `--periscope` / `--pointcloud` / `--poses` / `--esdf` / `--status` / `--trajectory` | each stream, independently toggleable. Naming any selects exactly those; naming none records the default set for your `--where`. `--no-X` forces one off. `--all` adds full-res. |
| `--where {robot,cloud}` | where the recorder runs. `robot` ⇒ Zenoh `peer` mode so the archive pull stays local, and a default stream set limited to the robot's own streams. |
| `--session-id`, `--out` | output folder and root (default `<repo>/recordings/`). |
| `--duration`, `--max-size` | session caps (`5m`, `2GB`); the recorder stops cleanly and finalises. |
| `--fullres-max-size`, `--fullres-every`, `--fullres-ring`, `--fullres-lag` | the full-res stream's own caps. |
| `--pointcloud-keyframe-s` | how often to materialise a complete map keyframe from the mirror (default 10 s; free). |
| `--pointcloud-repair` | manifest-diff repair pulls — what heals dropped pushes and completes a mid-session start. On by default from the cloud side, **off** with `--where robot`. |
| `--pointcloud-snapshot-query-s` | optional server-side snapshot queries as a cross-check. Off by default: each costs a full cloud extract. |
| `--scene`, `--trajectory-family`, `--pass`, `--seed`, `--camera-height`, `--camera-height-source`, `--mount-geometry`, `--clear-flat-floor`, `--operator`, `--note`, `--meta` | the §3.2 session metadata, straight into `meta.json`. |
| `--dry-run` | print the resolved plan (streams, keys, caps) and exit without recording. |
| `--zenoh-listen` | peer mode only: bind an inbound endpoint (default: none). |
| `--selftest` | offline end-to-end check. |
