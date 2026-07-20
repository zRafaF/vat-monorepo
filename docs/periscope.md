# Remote Periscope

The **periscope** is a directable, high-quality video slice cut from the robot's
360° camera. Instead of streaming the whole panorama (which would swamp the
uplink), the operator *points* at a region of interest and the robot transmits
**only that slice**, cropped from the full-resolution 4K frame it already has in
memory. Zooming in narrows the field of view (FOV) while holding the pixel
budget — you trade coverage for angular detail — until the 4K sensor runs out of
real pixels, past which it becomes an honest digital zoom on the client.

This is the concrete realization of the **"Beacon" / virtual monitor** described
in [Architecture → The Client](architecture.md#1-the-client): a
real-time HD feed projected at the point of interest for detailed inspection,
without paying the bandwidth of the full sphere.

!!! success "Status: implemented"
    The periscope works today. It runs in the robot's camera process
    (`robot/docker/periscope/`), is enabled by `PERISCOPE_ENABLE=1` in `vat.env`, and is
    viewed in the client. You can exercise it with `make periscope-probe`. This page also
    documents the projection/zoom math and the design intent behind the current settings.

---

## Why a periscope (the bandwidth argument)

The Theta X panorama is captured at 4K equirectangular (**3840 × 1920**, covering
360° × 180°). Streaming that as video would cost **~20+ Mbit/s** — infeasible on
the cellular uplink the pose and mapping streams already share.

A single cropped slice is a different order of magnitude:

| View | Codec (H.265) | Approx. bitrate |
|---|---|---|
| 360p tier @ ~10 fps | HEVC | ~0.2–0.4 Mbit/s |
| 480p tier @ ~15 fps | HEVC | ~0.4–0.8 Mbit/s |
| 720p tier @ ~24 fps | HEVC | ~1–2 Mbit/s |

Comparable to (or below) the existing mapping stream, and it never has to grow
with zoom because zoom is *free* — it is just a smaller FOV rendered into the
same pixel budget.

---

## The projection & zoom math

This is the heart of the feature; the UI and the robot renderer must agree on it.

### Angular resolution of the source

A 4K equirectangular frame spans 360° horizontally in 3840 px, so near the
horizon the source detail is:

```
px_per_deg = 3840 / 360 = 10.667  px per degree      (vertical: 1920 / 180 = 10.667)
```

For a rectilinear view of horizontal field of view `HFOV`, the amount of **real
sensor detail** available across that view is:

```
native_px = HFOV_deg × 10.667
```

### Optical vs. digital zoom

Let `W` be the width in pixels we render the slice at. Two regimes:

- **`W ≤ native_px` → optical zoom.** The source has *more* real pixels than we
  render, so we downsample the 4K crop — crisp, true detail.
- **`W > native_px` → source-limited (digital) zoom.** The source has *fewer*
  real pixels than the requested output. **The robot must NOT upscale.** It
  renders at the native pixel count (`≤ W`) and the client upscales to the
  display. This keeps the wire payload minimal and puts the (cheap) interpolation
  on the powerful client GPU where it belongs.

The **optical-floor FOV** for a given rendered width is therefore:

```
FOV_floor_deg = W / 10.667
```

Below this FOV you are in digital zoom (robot sends `native_px`, client scales
up); at or above it the view is fully optical.

!!! tip "Robot render rule (one line)"
    ```
    render_px = min(requested_output_px, round(FOV_deg × 10.667))
    ```
    Never render more pixels than the sensor supports for that FOV. The client
    always scales `render_px` up to its display size.

### Worked numbers for the square (1:1) tiers

With a square aspect (see [Aspect ratio](#aspect-ratio)) the render is `N × N`,
so `W = N` and the optical floor is `N / 10.667`:

| Tier (short side `N`) | Optical-floor FOV | Behaviour below the floor |
|---|---|---|
| **360** | ~34° | robot sends `<360` px, client upscales |
| **480** (default) | ~45° | robot sends `<480` px, client upscales |
| **720** (high) | ~68° | robot sends `<720` px, client upscales |

So at the default **480p square** tier, any FOV **≥ 45°** is fully optical;
tighter than 45° is digital zoom. Wider aspects push the floor up because the
floor scales with the **rendered width**, not the tier: e.g. a 480-tall 16:9
render is 854 px wide → optical floor ≈ **80°**.

### FOV cap and edge stretch (no fisheye)

Rectilinear (normal perspective) projection stretches the edges by a factor of

```
edge_stretch = sec²(HFOV / 2)   (relative to the view centre)
```

which diverges to infinity at 180°. To avoid that regime entirely we **cap the
maximum FOV at 120–140°** (default **130°**) rather than implementing a
fisheye/stereographic wide mode:

| Max HFOV | Half-angle | Edge stretch |
|---|---|---|
| 120° | 60° | 4.0× |
| 130° | 65° | 5.6× |
| 140° | 70° | 8.5× |

Beyond ~140° the corner magnification becomes unusable, so the periscope stays
rectilinear throughout and simply refuses to zoom out past the cap. Wide
situational awareness remains the job of the 3D point-cloud map; the periscope is
the *detail* instrument.

### Aspect ratio and vertical FOV

The aspect ratio is **operator-configurable** (see below). For a rectilinear
projection the vertical FOV is **not** a linear scaling of the horizontal FOV —
it relates through the tangents:

```
tan(HFOV/2) / tan(VFOV/2) = width / height
  ⇒  VFOV = 2 · atan( tan(HFOV/2) · height / width )
```

The renderer, the wire message, and the client frustum gizmo must all use this
relation so the drawn frustum matches the pixels actually delivered.

---

## Aspect ratio

Configurable, with a **square (1:1) default** for inspection. Rationale:

- **1:1** — symmetric zoom, no orientation bias when the operator tips the aim
  around, and pixel-efficient for a given angular reach. Reads as "I am pointing
  *at this thing*." Best default for targeted inspection.
- **4:3 / 16:9** — more horizontal situational context for scanning a wall or
  corridor, at the cost of more pixels (higher optical floor) for the same
  vertical detail.

Ship 1:1 as the default with 4:3 and 16:9 presets; the frustum gizmo reflects the
active ratio.

---

## Resolution tiers & frame rate

**Resolution tiers** (short side, px): **360** (low), **480** (default), **720**
(high-detail toggle). The client requests a tier; the robot renders at
`min(tier-derived width, native_px)` per the rule above (never upscaling).

**Frame rate** is configurable and supports an optional **dynamic** mode
(togglable):

- **Static** — a fixed rate (default 15 fps). Inspection is mostly "hold and
  look," so low rates are cheap and fine.
- **Dynamic** — ramp the rate up while the operator is actively re-aiming
  (panning/zooming) and drop it back down when the view is static, between
  `PERISCOPE_FPS_MIN` and `PERISCOPE_FPS_MAX`. This spends bitrate on motion
  smoothness only when it matters. Controlled by `PERISCOPE_FPS_DYNAMIC`.

---

## Transport: Zenoh (for now)

The periscope rides the **existing Zenoh bus**, not a parallel media line.

- **Control channel (client → robot).** The operator's aim
  `(yaw, pitch, fov, aspect, res_tier)` is a tiny message published by the client
  and **subscribed** by the robot. This respects the outbound-only constraint:
  the robot never accepts an inbound connection — it dials out to the hub and the
  router relays the request over that link, exactly like every other stream. See
  [Networking](architecture.md#2-the-cloud).
- **Video channel (robot → client).** Encoded HEVC/H.264 frames published on a
  dedicated key, on its **own publisher/session** with:
    - `reliability = best_effort`, `congestion_control = DROP` — under congestion
      it sheds frames rather than blocking, so it **never starves the pose
      stream**;
    - Zenoh **priority `DATA`** (below pose's `DATA_HIGH`, alongside/at the
      mapping frames) — same QoS discipline established for the camera uplink;
    - a **bounded send buffer** (`so_sndbuf`) — same bufferbloat fix used for the
      mapping uplink.
- **Keyframe recovery.** Because `best_effort` can drop packets, the encoder
  emits an **IDR keyframe every `PERISCOPE_IDR_INTERVAL_S`** and honours a
  `request_keyframe` control key so a freshly-connected (or desynced) client can
  force a resync.

!!! note "Why not a parallel WebRTC/SRT line?"
    A dedicated media transport gives jitter buffering, RTCP and adaptive bitrate
    "for free," but it reintroduces the exact NAT-traversal problem Zenoh already
    solves: the robot cannot accept inbound, so WebRTC would need signalling plus
    a **STUN/TURN relay** — another server component. For a single viewer over
    the existing hub, HEVC-over-Zenoh (with our own IDR + drop-old-frame policy)
    is simpler and reuses solved infrastructure. **Escape hatch:** if we later
    need multi-viewer, adaptive bitrate, or sub-100 ms glass-to-glass, migrate
    *only the media* to WebRTC with the (future) public hub acting as the
    TURN/SFU relay. Don't build that line pre-emptively.

### Re-aim latency & overscan

Re-aiming is a round trip: `client → hub → robot → render → hub → client`
(~200–400 ms on cellular). To make small pans feel instant, the robot renders a
slice **slightly wider than the displayed region** (an overscan margin,
`PERISCOPE_OVERSCAN_DEG`). The client can pan *within* the received margin
immediately and only requests a new centre when the operator moves past it.

---

## Client UX: two frustums + a video panel

The viewer shows the feed **two ways at once**:

1. **In-scene, on the frustum.** A low-resolution copy of the slice is textured
   onto the far face of the aiming frustum inside the 3D map, so the operator
   sees the feed *in situ*, spatially registered to the point cloud.
2. **Full-resolution side panel.** The decoded HEVC stream at full tier
   resolution, for actual detailed inspection.

### The two frustums

The aiming widget is a **3D wireframe frustum** (a "prism") drawn in the scene,
and there are **two** of them:

- **Requested frustum** — where the operator is *currently* pointing. Updates
  instantly on input (drag to aim, scroll to zoom).
- **Actual frustum** — the pose/params the *currently displayed frame* was really
  rendered at. It **lags behind** the requested one during a pan (by the re-aim
  round trip), because it is driven by the parameters echoed back in each video
  frame's header.

Seeing both makes the transport latency legible: the requested prism leads, the
actual prism catches up. When static they coincide.

!!! warning "Anchor the frustums to the CAMERA, not the robot base"
    The camera rides a selfie-stick offset from the body origin
    (`STICK_OFFSET_Z ≈ 0.80 m`, plus the mount rotation — see
    [Architecture → Camera ≠ base](architecture.md#camera-base-the-kinematic-offset)).
    Both frustums' apex must be the **camera world pose**
    `T_world_camera = T_world_base ∘ T_base_camera`, and the periscope
    `yaw/pitch` are defined relative to the **camera forward axis** — not the
    robot base centre. Anchoring to the base would misplace the frustum by the
    stick offset and mis-rotate it as the body tilts.

---

## Robot-side implementation

**Share the already-decoded 4K frame; do not spin a separate capture, and do not
reuse the archive path.** The archive is FPS-capped (it only keeps transmitted
frames), so tapping it would throttle the periscope. Instead the periscope reads
the **live full-resolution frame** the camera process has already decoded, before
it is downscaled for the mapping stream. The pipeline per requested frame:

```
live 4K equirect frame ── remap(yaw,pitch,HFOV,VFOV, render_px) ──▶ HEVC/NVENC ──▶ Zenoh publish
        (shared, in-process)         equirect → rectilinear            (H.265, IDR every N s)
```

- **Reprojection.** Equirectangular → rectilinear via a per-pixel remap
  (`cv2.remap`, or GPU/NPP). Light at 360–720p.
- **Encoding.** Prefer **H.265 (HEVC) via NVENC** — it is ~30–50% smaller than
  H.264 at equal quality and nearly free on the Jetson encoder. Use H.265 when
  the viewer PC's decoder supports it (it is an NVIDIA machine, so it should);
  fall back to **H.264** otherwise. Selectable via `PERISCOPE_CODEC`.
- **Runs inside the camera process** (to share the decoded frame) — *not* a
  separate process. This is a deliberate exception to the one-process-per-role
  pattern, justified by avoiding a second 4K decode.

!!! tip "Code organization"
    The existing robot modules are large single files. The periscope should be
    added as a **small multi-file package** (e.g. `robot/docker/periscope/`)
    rather than another thousand-line module — split by concern:
    `reproject.py` (equirect→rectilinear remap + the zoom math above),
    `encoder.py` (NVENC HEVC/H.264 wrapper), `control.py` (view-request
    subscribe, dynamic-fps + overscan logic), `stream.py` (Zenoh publisher, IDR
    cadence, keyframe requests). The camera process imports the package and hands
    it each decoded 4K frame via a shared in-process frame handle.

---

## Wire protocol (sketch)

New Zenoh keys (under the robot prefix) and messages to add to
`common/vat_protocol.py`:

| Key | Direction | Payload |
|---|---|---|
| `{robot}/periscope/view_request` | client → robot | `ViewRequest` |
| `{robot}/periscope/frame` | robot → client | `PeriscopeFrame` |
| `{robot}/periscope/request_keyframe` | client → robot | (empty / seq) |

**`ViewRequest`** — `yaw_deg`, `pitch_deg`, `hfov_deg`, `aspect` (w:h),
`res_tier` (360|480|720), `seq`, `client_ts_ns`.

**`PeriscopeFrame`** — `codec` (hevc|h264), `is_keyframe`, `width_px`,
`height_px`, **the actual `yaw/pitch/hfov/vfov/aspect` rendered** (so the client
can draw the *actual* frustum and know whether it is upscaling), `native_px`
(source detail available → lets the client label optical vs digital), `seq`,
`capture_ts_ns`, and the encoded byte payload.

Echoing the *actual* render parameters in every frame is what powers the
requested-vs-actual frustum pair.

---

## Configuration (env, via `vat.env`)

| Variable | Default | Meaning |
|---|---|---|
| `PERISCOPE_ENABLE` | `1` | Master on/off. |
| `PERISCOPE_RES` | `480` | Resolution tier short side: `360` \| `480` \| `720`. |
| `PERISCOPE_ASPECT` | `1:1` | View aspect ratio: `1:1` \| `4:3` \| `16:9`. |
| `PERISCOPE_MAX_FOV` | `130` | Max horizontal FOV (deg); cap 120–140 to bound edge stretch. |
| `PERISCOPE_MIN_FOV` | `20` | Tightest zoom (deg); below the optical floor it is digital zoom. |
| `PERISCOPE_CODEC` | `h265` | `h265` (HEVC, preferred) \| `h264` (fallback). |
| `PERISCOPE_FPS` | `15` | Static frame rate when dynamic mode is off. |
| `PERISCOPE_FPS_DYNAMIC` | `1` | Toggle dynamic frame rate (ramp on aim, drop when static). |
| `PERISCOPE_FPS_MIN` | `8` | Dynamic-mode floor. |
| `PERISCOPE_FPS_MAX` | `24` | Dynamic-mode ceiling. |
| `PERISCOPE_OVERSCAN_DEG` | `10` | Extra FOV margin rendered for instant client-side micro-pan. |
| `PERISCOPE_IDR_INTERVAL_S` | `2.0` | Keyframe (IDR) cadence for drop recovery. |
| `PERISCOPE_SO_SNDBUF` | `262144` | Bounded send buffer on the video link (bufferbloat guard). |

---

## Suggested build order

Prototype the **riskiest unknown first**, before touching the viewer:

1. **Reprojection + encode on the robot** — equirect→rectilinear remap of a
   recorded 4K frame at the chosen tiers/FOVs, NVENC HEVC encode, verify the
   optical/digital threshold and edge stretch match the math above.
2. **Zenoh video + control loop** — publish frames (best_effort/DROP, own
   priority, IDR cadence), subscribe to `ViewRequest`, confirm it does not
   perturb the pose stream (reuse the uplink QoS checks).
3. **Client frustum gizmo** — camera-anchored requested/actual frustums, aim &
   zoom input, publish `ViewRequest`.
4. **Client decode + panel + in-scene texture**, then the overscan micro-pan and
   dynamic-fps polish.
