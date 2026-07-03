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
