# Robot data sources (Go2-W) — measured

What the Go2-W actually publishes on Zenoh, measured with `make probe_robot`
(`tools/probe_robot_data.py`) on **2026-07** against this firmware. The estimator
must be designed around **this** table, not around what ROS advertises — many
topics exist but carry no data.

> Re-run to refresh: `make probe_robot` (drive the dog during the 15 s window so
> motion channels light up; `PROBE_S=25`, `PROBE_ALL=1` for the point-cloud topics).
> The probe decodes with the **same** `unitree_go` message defs the estimator uses
> (`kinematics._UNITREE_MSG_DEFS`) — both `LowState` and `SportModeState` decoded
> cleanly, so this firmware's message layout matches; no `UNITREE_ROS2_REF` change needed.

## Usable (carry real, changing data)

| Zenoh key | Type | Rate\* | Fields that work | Use |
|---|---|---|---|---|
| `go2/rt/lowstate` | `unitree_go/LowState` | native ~500 Hz | `imu_state.quaternion/gyroscope/accelerometer` (all plausible: \|accel\|≈9.6 at rest, swings under motion); `motor_state[0..11]` leg `q`/`dq`; **`motor_state[12..15].dq`** = wheel speeds; `foot_force[0..3]`≈140 in contact | **The workhorse.** Attitude, wheel odometry, leg FK, contact. |
| `go2/rt/utlidar/imu` | `sensor_msgs/Imu` | ~12 Hz | `orientation`, `angular_velocity`, `linear_acceleration` (own frame; z-accel sign differs) | Secondary IMU (LiDAR). Redundant with lowstate IMU, lower rate. |
| `go2/rt/lf/sportmodestate` | `unitree_go/SportModeState` | ~7 Hz | `imu_state.*` only; `mode` | IMU here duplicates lowstate. **See dead fields below.** |

\* **Rates are the laptop's remote view over VPN/WiFi and understate reality.** The
`/lowstate` rate read 1.1 Hz while driving (uplink saturated by the camera stream)
vs 83 Hz idle — but the fuser runs **on the Jetson** as a local Zenoh peer, so it
receives `/lowstate` at ~native rate regardless. Don't size the estimator off these numbers.

## Dead / unusable (advertised but zero or silent)

| Zenoh key | Type | Status | Consequence |
|---|---|---|---|
| `…/lf/sportmodestate` → `body_height` | float | **0.0** | Fuser's `pos[2]=body_height` → always < 0.05 → Z pinned to `FALLBACK_BODY_HEIGHT` (0.30 m). **Vertical is fake.** |
| `…/lf/sportmodestate` → `velocity`, `yaw_speed` | float[] | **0.0** | No body velocity / yaw rate from sport state (confirms the reason wheel odometry was introduced). |
| `go2/rt/sportmodestate` (full, non-lf) | `SportModeState` | **silent (0 samples)** | High-rate sport state (with a possibly-real `body_height`/`velocity`) is not published/forwarded → `SPORT_TOPIC=sportmodestate` yields nothing. |
| `go2/rt/utlidar/robot_odom` | `nav_msgs/Odometry` | **silent** | No ready-made wheel/lidar odometry. |
| `go2/rt/utlidar/robot_pose` | `PoseStamped` | **silent** | No onboard global pose. |
| `go2/rt/uslam/localization/odom`, `…/frontend/odom` | `nav_msgs/Odometry` | **silent** | Onboard Unitree LiDAR-SLAM odometry **not running/forwarded**. |
| `go2/rt/lio_sam_ros2/mapping/odometry` | `nav_msgs/Odometry` | **silent** | Same. |
| `go2/rt/utlidar/cloud*`, `…/uslam/*cloud*` | `PointCloud2` | **silent** | LiDAR point clouds not flowing. |
| `/gnss` | `String` | empty | No GNSS heading. |

No magnetometer, GNSS, or absolute-heading source anywhere → **yaw is observable
only through the VGGT correction.**

## Implications for the pose estimator

- **Translation** can come only from **wheel odometry** (`motor_state[12..15].dq × WHEEL_RADIUS`).
  Confirmed live; `WHEEL_RADIUS` still needs a measured-drive calibration. No lateral/slip observability.
- **Attitude** comes from the `/lowstate` IMU (good). **Accelerometer is confirmed usable**,
  so an inertial (accel-preintegration / ESKF) estimator is data-feasible — not just wheels+gyro.
- **Z / `body_height` is dead** → compute stance height from leg FK (foot_force + leg joints are
  live) and/or take Z from the VGGT correction; stop trusting `body_height`.
- **No onboard odometry/SLAM** is available today. The Go2's `uslam` / `utlidar/robot_odom`
  topics *exist* and would be a drift-bounded global odometry if enabled — a high-value
  lead, but **currently silent**, so per project policy nothing may be designed around them
  until a probe shows them publishing (investigate: SLAM service off vs bridge QoS mismatch).


---

# Telemetry & latency baseline

Reference figures for what a *healthy* run looks like, so future work has a
yardstick. These are **illustrative** — captured July 2026 on this rig/network
with the config in `vat.env` at the time. They WILL change with hardware, tuning,
Wi-Fi, and map size. Treat the shapes and bottlenecks as the lesson, not the exact
numbers. Re-measure from three places:

- **Client HUD / console** (`make viewer`): the `VAT TELEMETRY` block (latencies,
  throughput, pose rate, render fps).
- **Robot fuser log** (`docker logs` on the Jetson): the `[Fuser] ...` line.
- **Mapping server log** (`make mapping`): per-submap `Process Timing` + the
  `corr published / suppressed / rejected` counters.

## The pipeline, end to end

```
capture --uplink--> server --window wait--> PanoVGGT infer --> pose_correction
   |                  (Wi-Fi, shared)        (per submap)         (down to robot)
   |                                                                   |
   +--- fuser dead-reckons (IMU+wheels) at PUBLISH_HZ, re-anchored by the fix --+
                                                                       |
                                                     fused pose --up--> client (predicts)
```

Two DIFFERENT latencies matter and are easily confused:

- **capture -> display (cloud/frame):** ~0.4-0.8 s. What the HUD's `capture -> display` shows.
- **capture -> VGGT pose applied (the "green" anchor):** ~2-4.5 s (typ ~4 s). NOT on the
  HUD; read it from the fuser log's `corr_lag`. It is long because a frame must wait for
  its whole submap window to close and then be inferred.

## Baseline figures (illustrative)

| Metric | Where | Typical | Notes |
|---|---|---|---|
| robot -> server (uplink) | HUD | ~0.85 s, spikes ~1.3 s | Congested shared Wi-Fi uplink; the camera stream dominates it. THE bottleneck. |
| robot -> server throughput | HUD | ~120-137 KB/s @ ~2.2 fps | Capped by `THROTTLE_FPS` (2.5) and the uplink. |
| server submap processing | server `Total Submap Time` | ~1.3 s (reset batches ~2 s) | PanoVGGT `Perception_Infer` ~0.55 s; the rest is TSDF/mesh/pose math. |
| VGGT correction latency (capture->apply) | fuser `corr_lag` | ~2.2-4.5 s | The real "how stale is the anchor" number. Bounded by `POSE_LAG_S`. |
| correction cadence | server `corr published` | ~1 per submap (~every 2-2.7 s) | `suppressed` = deadband/warm-up, `rejected` = outlier gate. |
| pose -> client | HUD `robot -> client(pose)` | 1-25 ms; rate ~20-30 Hz | Drops to ~5-12 Hz during uplink congestion (pose shares the uplink, `DROP` QoS). |
| server -> client | HUD | ~0-30 ms | Cheap. |
| render | HUD | ~40-47 fps | Client is single-process/GIL-bound. |
| metric scale anchor `s` | server `[Anchor] s=` | ~0.63 | Re-estimated per RESET batch; ~6% wobble across resets = a known pose-jump source. |

## Reading the robot fuser line

```
[Fuser] seq=1642 odom_valid=True vggt=True corrections=12 corr_lag=4.00s stale_dropped=0 body_vx=+0.36m/s pos=[-0.05 0.49 0.38]
```

- `odom_valid` — `/lowstate` is decoding (IMU + wheels present).
- `vggt` — at least one VGGT correction has been fused (False during the first ~15-20 s
  warm-up while the server commits metric scale — expected).
- `corrections` — VGGT fixes fused so far; should climb ~1 per submap while moving.
- `corr_lag` — capture->apply latency of the last fix (the VGGT pose staleness).
- `stale_dropped` — fixes older than `POSE_LAG_S` (should stay 0; if it climbs, raise `POSE_LAG_S`).
- `body_vx` — contact-selected wheel forward speed. `pos` — fused world position (z from body height).

## Known characteristics / gotchas (baseline behaviour)

- **`state` flips to "dead-reckon" between corrections.** Corrections land every ~2-2.7 s;
  `FIX_HOLD_S` controls how long the pose stays labelled VGGT-corrected. If it flickers to
  amber, `FIX_HOLD_S` is shorter than the correction cadence (it is NOT a broken correction
  path — check the fuser `corrections=` counter first).
- **Warm-up (~first 15-20 s): `vggt=False`, `corrections=0`.** The server suppresses
  corrections until metric scale commits (`SCALE_WARMUP_WINDOWS`). Expected, not a fault.
- **Lateral (strafe) motion does not show in dead-reckoning.** Wheel odometry is
  forward-only and the odom factor pins body-lateral velocity to ~0; only VGGT recovers it.
- **Pose can jump ~0.1-1 m on a correction.** Partly the ~4 s staleness, partly the metric
  scale/leveling being re-estimated each RESET batch (`s` wobble above) so the correction
  frame shifts between batches. Stabilising the world anchor across resets is a known
  follow-up.
- **Uplink is the dominant latency + smoothness lever.** When `robot -> server` spikes,
  pose rate to the client collapses together (shared uplink). Reducing frame bandwidth
  (`THROTTLE_FPS`, `FRAME_CODEC`, `TRANSMIT_WIDTH/HEIGHT`) helps everything at once.
