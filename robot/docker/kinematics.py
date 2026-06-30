"""
VAT — Robot Kinematics
======================
Solves the two geometry problems the rest of the system depends on:

1. **Camera ↔ base transform** (``T_base_camera``).
   PRISM-VGGT estimates the pose of the *camera*, but we want the pose of the
   *robot base*.  The camera sits on a selfie-stick on the back of the Go2-W,
   so its pose relative to the base is a fixed rigid transform — but it is *not*
   the identity: when the body rolls/pitches (even with the wheels planted) the
   stick swings the camera sideways/forward.  We therefore must subtract the
   stick transform to recover the base pose:

       T_world_base = T_world_camera ∘ inverse(T_base_camera)

   In the future the camera may move to an actuated arm; then ``T_base_camera``
   becomes a function of the arm joint angles (forward kinematics from a URDF).
   The :class:`RobotModel` interface hides which case we are in.

2. **Camera height above the floor** (for PRISM metric scale).
   The Go2-W can lie down and stand up, which changes the true height of the
   camera.  Height = body height above floor (from ``SportModeState.body_height``
   or, later, leg/wheel forward kinematics) + the vertical reach of the stick,
   rotated by the current body tilt.

Design notes
------------
* Pure NumPy — no ROS, no scipy.  Runs in the robot Docker container.
* :class:`RobotModel` has two implementations:
    - :class:`SelfieStickModel`  — fixed stick transform (used now).
    - :class:`URDFArmModel`      — placeholder for the actuated-arm future.
* :class:`RobotStateTracker` keeps the latest body state (height, orientation,
  velocities) by subscribing to the bridged ``SportModeState`` over Zenoh.  It
  is best-effort: if the custom Unitree message can't be decoded it falls back
  to configured constants and never raises.

This module is shared by ``theta_camera.py`` (needs camera height) and
``pose_fuser.py`` (needs the camera→base transform + body twist).
"""

from __future__ import annotations

import os
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# vat_protocol is copied next to this file inside the Docker image.
from vat_protocol import (
    quat_identity, quat_normalize, quat_mul, quat_conj, quat_rotate,
)

log = logging.getLogger("kinematics")


# ─────────────────────────────────────────────────────────────────────────────
# Rigid transform  (pose of a child frame expressed in a parent frame)
# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class Transform:
    """Pose of a child frame in a parent frame: p_parent = R·p_child + t."""
    rotation: np.ndarray = field(default_factory=quat_identity)     # (4,) xyzw
    translation: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self):
        self.rotation = quat_normalize(self.rotation)
        self.translation = np.asarray(self.translation, dtype=np.float64).reshape(3)

    def inverse(self) -> "Transform":
        r_inv = quat_conj(self.rotation)
        return Transform(r_inv, -quat_rotate(r_inv, self.translation))

    def compose(self, other: "Transform") -> "Transform":
        """self ∘ other.  If self is T_A_B and other is T_B_C → returns T_A_C."""
        return Transform(
            quat_mul(self.rotation, other.rotation),
            quat_rotate(self.rotation, other.translation) + self.translation,
        )

    def apply(self, point: np.ndarray) -> np.ndarray:
        return quat_rotate(self.rotation, np.asarray(point, dtype=np.float64)) + \
            self.translation

    @staticmethod
    def from_xyz_quat(xyz, quat) -> "Transform":
        return Transform(np.asarray(quat, dtype=np.float64),
                         np.asarray(xyz, dtype=np.float64))


def quat_from_euler_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Build a quaternion (xyzw) from intrinsic ZYX (yaw-pitch-roll) Euler angles."""
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return quat_normalize(np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]))


# ─────────────────────────────────────────────────────────────────────────────
# Robot model interface
# ─────────────────────────────────────────────────────────────────────────────


class RobotModel:
    """Maps between the camera frame and the robot base frame."""

    def camera_in_base(self, joints: Optional[dict] = None) -> Transform:
        """Return ``T_base_camera`` — the pose of the camera in the base frame.

        For a rigid mount this ignores ``joints``; for an actuated arm it is the
        forward kinematics of the arm at the given joint angles."""
        raise NotImplementedError

    def base_from_camera_world(self, camera_world: Transform,
                               joints: Optional[dict] = None) -> Transform:
        """Given the camera pose in the world (from VGGT), return the base pose
        in the world: ``T_world_base = T_world_camera ∘ inverse(T_base_camera)``."""
        return camera_world.compose(self.camera_in_base(joints).inverse())

    def camera_height(self, body_height: float, body_rotation: np.ndarray,
                      joints: Optional[dict] = None) -> float:
        """Height of the camera optical centre above the floor.

        ``body_height`` is the base-frame origin height above the floor
        (``SportModeState.body_height``).  ``body_rotation`` is the base
        orientation in the world (xyzw); used to project the stick reach onto
        the vertical world axis so body tilt is accounted for."""
        t_bc = self.camera_in_base(joints)
        # camera offset expressed in the world, then take the world-up (z) part
        cam_offset_world = quat_rotate(quat_normalize(body_rotation), t_bc.translation)
        return float(body_height + cam_offset_world[2])


class SelfieStickModel(RobotModel):
    """Camera rigidly mounted on a fixed selfie-stick on the robot's back.

    Configure the stick geometry from measurements/CAD.  All values are the
    camera pose *in the base frame* (base origin = body centre, x forward,
    y left, z up — REP-103 / ROS convention)."""

    def __init__(self,
                 offset_xyz=(-0.20, 0.0, 0.55),
                 mount_rpy=(0.0, 0.0, 0.0)):
        # camera sits behind (-x) and above (+z) the body centre by default
        self._t_bc = Transform.from_xyz_quat(
            np.asarray(offset_xyz, dtype=np.float64),
            quat_from_euler_zyx(*mount_rpy),
        )
        log.info(f"[Kinematics] SelfieStickModel  offset={tuple(self._t_bc.translation)} "
                 f"rpy={mount_rpy}")

    def camera_in_base(self, joints: Optional[dict] = None) -> Transform:
        return self._t_bc


class URDFArmModel(RobotModel):
    """PLACEHOLDER for a future camera-on-arm configuration.

    When the camera moves to an actuated arm, ``camera_in_base`` is the forward
    kinematics of the arm chain at the current joint angles, parsed from a URDF
    (e.g. ``yourdfpy`` / ``urdfpy`` to build the kinematic tree, then walk it).
    Until that hardware exists this falls back to a fixed transform so the rest
    of the system keeps working.

    For the Go2-W note that the *wheels* are continuous joints — they do not
    change the body→camera transform, only the body→ground height, which we
    read from ``SportModeState.body_height`` rather than wheel FK for the POC."""

    def __init__(self, urdf_path: str, camera_link: str = "camera_optical_frame",
                 base_link: str = "base", fallback: Optional[Transform] = None):
        self._urdf_path = urdf_path
        self._camera_link = camera_link
        self._base_link = base_link
        self._fallback = fallback or Transform.from_xyz_quat((-0.20, 0.0, 0.55),
                                                             quat_identity())
        self._model = None
        if os.path.exists(urdf_path):
            try:
                self._load(urdf_path)
            except Exception as e:           # pragma: no cover - depends on optional dep
                log.warning(f"[Kinematics] URDF load failed ({e}); using fixed fallback")
        else:
            log.warning(f"[Kinematics] URDF '{urdf_path}' not found; using fixed fallback. "
                        "Fetch the go2_description / go2w URDF and set ROBOT_URDF.")

    def _load(self, urdf_path: str):
        # TODO: build kinematic tree (e.g. `import yourdfpy; yourdfpy.URDF.load(...)`)
        #       and resolve base_link → camera_link.  Left unimplemented on
        #       purpose — the arm hardware does not exist yet.
        raise NotImplementedError("URDF forward kinematics not implemented yet")

    def camera_in_base(self, joints: Optional[dict] = None) -> Transform:
        if self._model is None:
            return self._fallback
        raise NotImplementedError  # pragma: no cover


def build_robot_model() -> RobotModel:
    """Factory driven by environment variables."""
    kind = os.environ.get("ROBOT_MODEL", "selfie_stick").strip().lower()
    if kind == "urdf":
        return URDFArmModel(
            urdf_path=os.environ.get("ROBOT_URDF", "/app/go2w.urdf"),
            camera_link=os.environ.get("CAMERA_LINK", "camera_optical_frame"),
            base_link=os.environ.get("BASE_LINK", "base"),
        )
    # selfie_stick (default)
    ox = float(os.environ.get("STICK_OFFSET_X", "0.0"))
    oy = float(os.environ.get("STICK_OFFSET_Y", "0.0"))
    oz = float(os.environ.get("STICK_OFFSET_Z", "0.80"))
    # Camera→base rotation. PanoVGGT reports the camera in an OPTICAL frame
    # (x right, y down, z forward = equirect centre), but the base is REP-103
    # (x forward, y left, z up). The default maps optical→base. If the panorama's
    # "front" isn't the robot's forward, dial STICK_YAW_TRIM_DEG until the avatar's
    # heading lines up with the cloud.
    r, p, y = (float(v) for v in
               os.environ.get("STICK_MOUNT_RPY_DEG", "-90,0,-90").split(","))
    yaw_trim = float(os.environ.get("STICK_YAW_TRIM_DEG", "0"))
    mount_rpy = (np.deg2rad(r), np.deg2rad(p), np.deg2rad(y + yaw_trim))
    return SelfieStickModel(offset_xyz=(ox, oy, oz), mount_rpy=mount_rpy)


# ─────────────────────────────────────────────────────────────────────────────
# Live body-state tracker  (best-effort SportModeState decode over Zenoh)
# ─────────────────────────────────────────────────────────────────────────────

# Minimal Unitree message definitions so `rosbags` can decode the CDR the bridge
# forwards, WITHOUT a ROS install.  These match unitree_go (ROS2).  If your
# firmware uses a different layout, decoding will fail and the tracker falls
# back to constants — it never crashes.
_UNITREE_MSG_DEFS = {
    "unitree_go/msg/IMUState": """
float32[4] quaternion
float32[3] gyroscope
float32[3] accelerometer
float32[3] rpy
int8 temperature
""",
    "unitree_go/msg/SportModeState": """
TimeSpec stamp
uint32 error_code
IMUState imu_state
uint8 mode
float32 progress
uint8 gait_type
float32 foot_raise_height
float32[3] position
float32 body_height
float32[3] velocity
float32 yaw_speed
float32[4] range_obstacle
int16[4] foot_force
float32[12] foot_position_body
float32[12] foot_speed_body
================================================================================
MSG: unitree_go/msg/TimeSpec
int32 sec
uint32 nanosec
""",
    # LowState carries the per-joint motor angles (q) at ~500 Hz — we use the 12
    # leg joints for forward kinematics (the Go2-W does NOT populate
    # SportModeState.foot_position_body, so FK is the only way to draw limbs).
    "unitree_go/msg/LowState": """
uint8[2] head
uint8 level_flag
uint8 frame_reserve
uint32[2] sn
uint32[2] version
uint16 bandwidth
IMUState imu_state
MotorState[20] motor_state
BmsState bms_state
int16[4] foot_force
int16[4] foot_force_est
uint32 tick
uint8[40] wireless_remote
uint8 bit_flag
float32 adc_reel
int8 temperature_ntc1
int8 temperature_ntc2
float32 power_v
float32 power_a
uint16[4] fan_frequency
uint32 reserve
uint32 crc
================================================================================
MSG: unitree_go/msg/IMUState
float32[4] quaternion
float32[3] gyroscope
float32[3] accelerometer
float32[3] rpy
int8 temperature
================================================================================
MSG: unitree_go/msg/MotorState
uint8 mode
float32 q
float32 dq
float32 ddq
float32 tau_est
float32 q_raw
float32 dq_raw
float32 ddq_raw
int8 temperature
uint32 lost
uint32[2] reserve
================================================================================
MSG: unitree_go/msg/BmsState
uint8 version_high
uint8 version_low
uint8 status
uint8 soc
int32 current
uint16 cycle
int8[2] bq_ntc
int8[2] mcu_ntc
uint16[15] cell_vol
""",
}


# ─────────────────────────────────────────────────────────────────────────────
# Go2 / Go2-W leg forward kinematics  (draw the limbs from /lowstate joint angles)
# ─────────────────────────────────────────────────────────────────────────────
# Geometry from go2_description/xacro/const.xacro (metres). Same for Go2 and the
# wheeled Go2-W hip→thigh→calf chain; on the Go2-W the calf tip is the wheel hub.
_L_ABD   = 0.0955    # thigh_offset: hip abduction link, along ±y
_L_THIGH = 0.213     # thigh_length
_L_CALF  = 0.213     # calf_length
# Hip joint origins in the base frame (x fwd, y left, z up — REP-103).
_HIP_OFFSET = {
    "FR": np.array([ 0.1934, -0.0465, 0.0]),
    "FL": np.array([ 0.1934,  0.0465, 0.0]),
    "RR": np.array([-0.1934, -0.0465, 0.0]),
    "RL": np.array([-0.1934,  0.0465, 0.0]),
}
_SIDE_SIGN = {"FR": -1.0, "FL": 1.0, "RR": -1.0, "RL": 1.0}
# unitree_go LowState.motor_state index order: FR(0-2) FL(3-5) RR(6-8) RL(9-11),
# each [hip, thigh, calf]. On the Go2-W the 4 wheel motors are 12-15 (FR,FL,RR,RL).
LEG_ORDER = ["FR", "FL", "RR", "RL"]
WHEEL_IDX = [12, 13, 14, 15]   # FR, FL, RR, RL wheel motors (Go2-W)
# Wheel rolling radius (m). 7-inch tire ≈ 0.089 m nominal; loaded ≈ 0.085 m.
# CALIBRATE: drive a known distance and scale this so the dead-reckon matches.
# (Flip its sign if driving forward moves the estimate backward.)
WHEEL_RADIUS = float(os.environ.get("WHEEL_RADIUS", "0.085"))


def leg_fk(leg: str, q_hip: float, q_thigh: float, q_calf: float) -> dict:
    """Forward kinematics for one Go2 leg.

    Returns the hip, thigh-root, knee and foot/wheel points in the **base frame**
    (metres). Standard Unitree quadruped chain: hip rotates the leg about +x,
    thigh and calf are pitch joints about +y."""
    s = _SIDE_SIGN[leg]
    c1, s1 = np.cos(q_hip),   np.sin(q_hip)
    c2, s2 = np.cos(q_thigh), np.sin(q_thigh)
    c23, s23 = np.cos(q_thigh + q_calf), np.sin(q_thigh + q_calf)
    L1 = _L_ABD * s
    # points in the hip frame
    thigh_root = np.array([0.0, L1 * c1, L1 * s1])
    knee = np.array([-_L_THIGH * s2,
                     L1 * c1 + _L_THIGH * s1 * c2,
                     L1 * s1 - _L_THIGH * c1 * c2])
    foot = np.array([-_L_CALF * s23 - _L_THIGH * s2,
                     L1 * c1 + _L_CALF * s1 * c23 + _L_THIGH * s1 * c2,
                     L1 * s1 - _L_CALF * c1 * c23 - _L_THIGH * c1 * c2])
    hip = _HIP_OFFSET[leg]
    return {"hip": hip, "thigh_root": hip + thigh_root,
            "knee": hip + knee, "foot": hip + foot}


def base_height_above_ground(legs: dict, body_rotation=None) -> Optional[float]:
    """Vertical distance (m) from the base origin to the ground plane implied by
    the robot's **feet** — the leg-derived stance height.

    The four feet rest on the floor, so the base sits this far above it. We rotate
    each foot (base frame, from :func:`leg_fk`) by the body orientation and take
    its world-up (z) component; the base height is then the negative mean (feet
    are below the base). This makes the height SHRINK as the dog goes prone (legs
    fold and the feet rise toward the body) instead of staying pinned at a fixed
    ``SportModeState.body_height``, and it accounts for body roll/pitch through
    ``body_rotation`` (xyzw, base→world). Returns None if no feet are available."""
    if not legs:
        return None
    R_q = quat_identity() if body_rotation is None else body_rotation
    fz = [quat_rotate(R_q, np.asarray(p["foot"], dtype=np.float64))[2]
          for p in legs.values() if "foot" in p]
    if not fz:
        return None
    return float(max(-np.mean(fz), 0.0))


def camera_height_above_ground(legs: dict, body_rotation, stick_xyz,
                               fallback_base_height: float = 0.30):
    """Camera optical-centre height above the floor, derived from the legs.

    = (leg-derived base height) + the world-up component of the camera mount
    offset (``stick_xyz``, base frame) rotated by the body orientation — so it
    tracks BOTH stance (stand↔prone, via the feet) and tilt (roll/pitch swings
    the stick, moving the camera up/down and left/right). PanoVGGT gives the
    *camera* pose; this is the inverse direction used for the metric-scale height
    and for placing the base under the camera.

    Returns ``(camera_height, base_height, cam_offset_world)``."""
    base_h = base_height_above_ground(legs, body_rotation)
    if base_h is None:
        base_h = float(fallback_base_height)
    cam_off_world = quat_rotate(body_rotation, np.asarray(stick_xyz, dtype=np.float64))
    return float(base_h + cam_off_world[2]), base_h, cam_off_world


@dataclass
class BodyState:
    body_height: float
    rotation: np.ndarray            # (4,) xyzw, body in world
    linear_velocity: np.ndarray     # (3,) m/s, body frame (Unitree reports body frame)
    angular_velocity: np.ndarray    # (3,) rad/s, body frame (gyro)
    stamp_ns: int
    valid: bool = True


class RobotStateTracker:
    """Tracks the latest robot body state from the bridged SportModeState.

    Thread-safe.  Always returns a usable :class:`BodyState`; if no real data
    has arrived (or it can't be decoded) it returns the configured fallback so
    downstream consumers never block or crash."""

    def __init__(self, zenoh_session, robot_name: str,
                 sport_topic: Optional[str] = None,
                 fallback_body_height: float = 0.30):
        # Which sport-state topic to track. The Go2 publishes a high-rate
        # `/sportmodestate` ONLY while the motion service is active, but always
        # publishes a low-frequency `/lf/sportmodestate` (~10 Hz). Default to the
        # always-on lf copy so body/limb state is available even at rest; override
        # with SPORT_TOPIC=sportmodestate when the high-rate stream is wanted.
        if sport_topic is None:
            sport_topic = os.environ.get("SPORT_TOPIC", "lf/sportmodestate")
        self._lock = threading.Lock()
        self._fallback_h = fallback_body_height
        self._state = BodyState(
            body_height=fallback_body_height,
            rotation=quat_identity(),
            linear_velocity=np.zeros(3),
            angular_velocity=np.zeros(3),
            stamp_ns=0,
            valid=False,
        )
        self._decode = self._build_decoder()
        self._decode_failures = 0
        self._logged_failure = False

        key = f"{robot_name}/rt/{sport_topic}"
        try:
            zenoh_session.declare_subscriber(key, self._on_sport_state)
            log.info(f"[StateTracker] Subscribed to '{key}'")
        except Exception as e:
            log.warning(f"[StateTracker] Could not subscribe to '{key}': {e}")

    # -- decoder setup --------------------------------------------------------
    def _build_decoder(self):
        try:
            from rosbags.typesys import Stores, get_typestore, get_types_from_msg
            ts = get_typestore(Stores.ROS2_HUMBLE)
            registered = {}
            for name, definition in _UNITREE_MSG_DEFS.items():
                registered.update(get_types_from_msg(definition, name))
            ts.register(registered)

            def _decode(cdr: bytes):
                return ts.deserialize_cdr(cdr, "unitree_go/msg/SportModeState")
            return _decode
        except Exception as e:
            log.warning(f"[StateTracker] Decoder unavailable ({e}); "
                        "body state will use fallback constants.")
            return None

    # -- zenoh callback -------------------------------------------------------
    def _on_sport_state(self, sample):
        if self._decode is None:
            return
        try:
            msg = self._decode(bytes(sample.payload))
            # Unitree IMU quaternion is (w, x, y, z) → convert to (x, y, z, w)
            q_wxyz = np.asarray(msg.imu_state.quaternion, dtype=np.float64)
            rot = quat_normalize(np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]]))
            lin = np.asarray(msg.velocity, dtype=np.float64).reshape(3)
            ang = np.asarray(msg.imu_state.gyroscope, dtype=np.float64).reshape(3)
            stamp_ns = int(msg.stamp.sec) * 1_000_000_000 + int(msg.stamp.nanosec)
            with self._lock:
                self._state = BodyState(
                    body_height=float(msg.body_height) if msg.body_height > 0.05
                    else self._fallback_h,
                    rotation=rot,
                    linear_velocity=lin,
                    angular_velocity=ang,
                    stamp_ns=stamp_ns or time.time_ns(),
                    valid=True,
                )
        except Exception as e:
            self._decode_failures += 1
            if not self._logged_failure:
                log.warning(f"[StateTracker] SportModeState decode failed ({e}); "
                            "falling back to constants. Check the unitree_go msg "
                            "layout for your firmware. (logged once)")
                self._logged_failure = True

    # -- accessors ------------------------------------------------------------
    def get(self) -> BodyState:
        with self._lock:
            s = self._state
            return BodyState(s.body_height, s.rotation.copy(),
                             s.linear_velocity.copy(), s.angular_velocity.copy(),
                             s.stamp_ns, s.valid)


# ─────────────────────────────────────────────────────────────────────────────
# Low-level joint tracker  (per-leg FK from /lowstate, for limb visualisation)
# ─────────────────────────────────────────────────────────────────────────────


class LowStateTracker:
    """Tracks the 12 leg joint angles from the bridged ``LowState`` and exposes
    each leg's hip→thigh→knee→foot points in the base frame via :func:`leg_fk`.

    Best-effort, like :class:`RobotStateTracker`: if the message can't be decoded
    it simply reports ``valid=False`` and empty legs — never raises. Used by the
    Stage-2 viz to draw the limbs on the Go2-W (whose SportModeState foot array
    is zero)."""

    def __init__(self, zenoh_session, robot_name: str,
                 lowstate_topic: str = "lowstate"):
        self._lock = threading.Lock()
        self._legs: dict = {}        # leg → {hip,thigh_root,knee,foot}
        self._q = [0.0] * 12         # raw 12 leg joint angles (LEG_ORDER×[hip,thigh,calf])
        self._valid = False
        self._stamp_ns = 0
        # odometry fields (for the fuser's dead-reckoner)
        self._imu_quat = quat_identity()    # body attitude, xyzw
        self._gyro = np.zeros(3)            # body angular rate, rad/s
        self._accel = np.array([0.0, 0.0, 9.81])  # body specific force, m/s² (rest = +g up)
        self._body_vx = 0.0                 # contact-selected wheel forward speed, m/s
        self._body_wz = 0.0                 # yaw rate from wheel differential, rad/s
        self._n_contact = 4                 # wheels currently in ground contact
        self._decode = self._build_decoder()
        self._logged_failure = False

        key = f"{robot_name}/rt/{lowstate_topic}"
        try:
            zenoh_session.declare_subscriber(key, self._on_lowstate)
            log.info(f"[LowStateTracker] Subscribed to '{key}' (leg FK)")
        except Exception as e:
            log.warning(f"[LowStateTracker] Could not subscribe to '{key}': {e}")

    def _build_decoder(self):
        try:
            from rosbags.typesys import Stores, get_typestore, get_types_from_msg
            ts = get_typestore(Stores.ROS2_HUMBLE)
            registered = {}
            for name, definition in _UNITREE_MSG_DEFS.items():
                registered.update(get_types_from_msg(definition, name))
            ts.register(registered)
            return lambda cdr: ts.deserialize_cdr(cdr, "unitree_go/msg/LowState")
        except Exception as e:
            log.warning(f"[LowStateTracker] Decoder unavailable ({e}); legs off.")
            return None

    def _on_lowstate(self, sample):
        if self._decode is None:
            return
        try:
            msg = self._decode(bytes(sample.payload))
            ms = msg.motor_state
            q = [float(m.q) for m in ms[:12]]
            legs = {}
            for i, leg in enumerate(LEG_ORDER):
                legs[leg] = leg_fk(leg, q[3 * i], q[3 * i + 1], q[3 * i + 2])
            # --- odometry: IMU attitude/rate + wheel-odometry forward speed ---
            q_wxyz = np.asarray(msg.imu_state.quaternion, dtype=np.float64)
            imu_quat = quat_normalize(
                np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]]))   # wxyz→xyzw
            gyro = np.asarray(msg.imu_state.gyroscope, dtype=np.float64).reshape(3)
            accel = np.asarray(msg.imu_state.accelerometer, dtype=np.float64).reshape(3)
            # Per-wheel ground speed (dq × radius). LEG/CONTACT ODOMETRY: trust only
            # wheels whose foot is in ground contact (foot_force above threshold), so a
            # lifted or unloaded (slipping) wheel doesn't corrupt the body speed. The
            # 360° robot's wheels are normally all planted; this mainly rejects the
            # transient slip during hard accel/turns that fooled the old mean-of-4.
            v_wheel = [float(ms[i].dq) * WHEEL_RADIUS for i in WHEEL_IDX if i < len(ms)]
            try:
                ff = [float(x) for x in msg.foot_force]
            except Exception:
                ff = []
            thresh = float(os.environ.get("FOOT_CONTACT_FORCE", "12.0"))
            contact = [(ff[i] > thresh) if i < len(ff) else True for i in range(len(v_wheel))]
            sel = [v for v, c in zip(v_wheel, contact) if c] or v_wheel
            body_vx = float(np.mean(sel)) if sel else 0.0
            # Yaw rate from L/R wheel differential (FR,FL,RR,RL → right={0,2}, left={1,3}),
            # a cheap cross-check on the gyro. track ~ 2*|hip_y| from leg FK; fall back const.
            try:
                hy = abs(legs[LEG_ORDER[0]]["hip"][1]); track = max(2.0 * hy, 0.15)
            except Exception:
                track = float(os.environ.get("WHEEL_TRACK_M", "0.30"))
            if len(v_wheel) == 4:
                v_right = 0.5 * (v_wheel[0] + v_wheel[2]); v_left = 0.5 * (v_wheel[1] + v_wheel[3])
                body_wz = (v_right - v_left) / max(track, 1e-3)
            else:
                body_wz = float(gyro[2])
            with self._lock:
                self._legs = legs
                self._q = q
                self._imu_quat = imu_quat
                self._gyro = gyro
                self._accel = accel
                self._body_vx = body_vx
                self._body_wz = body_wz
                self._n_contact = int(sum(1 for c in contact if c))
                self._valid = True
                self._stamp_ns = time.time_ns()
        except Exception as e:
            if not self._logged_failure:
                log.warning(f"[LowStateTracker] LowState decode failed ({e}); "
                            "legs disabled. Check the unitree_go layout. (once)")
                self._logged_failure = True

    def get(self):
        """Returns (legs_dict, valid). legs_dict maps leg→{hip,thigh_root,knee,foot}."""
        with self._lock:
            return dict(self._legs), self._valid

    def get_odom(self):
        """Returns (imu_quat_xyzw, gyro_body, body_vx, valid) for the fuser (legacy
        wheel+IMU estimator)."""
        with self._lock:
            return (self._imu_quat.copy(), self._gyro.copy(),
                    self._body_vx, self._valid)

    def get_imu_odom(self):
        """Richer odometry for the GTSAM backend:
        (imu_quat_xyzw, gyro_body(3), accel_body(3), body_vx, body_wz, n_contact, valid).
        ``accel_body`` is the IMU specific force (m/s², ~+g up at rest); ``body_vx`` is
        contact-selected wheel speed; ``body_wz`` the wheel-differential yaw rate."""
        with self._lock:
            return (self._imu_quat.copy(), self._gyro.copy(), self._accel.copy(),
                    self._body_vx, self._body_wz, self._n_contact, self._valid)

    def get_joints(self):
        """Returns (q12, imu_quat_xyzw, valid): the 12 leg joint angles in
        LEG_ORDER×[hip,thigh,calf] order + body attitude — for URDF mesh FK."""
        with self._lock:
            return list(self._q), self._imu_quat.copy(), self._valid


# ─────────────────────────────────────────────────────────────────────────────
# Odometry tracker  (nav_msgs/Odometry — the fuser's propagation source)
# ─────────────────────────────────────────────────────────────────────────────


class OdometryTracker:
    """Tracks the robot's onboard odometry (``nav_msgs/Odometry``, default
    ``/utlidar/robot_odom``) — an always-on, already-integrated pose + twist.

    This is the fuser's propagation source. It is far more reliable than
    integrating ``SportModeState.velocity``: the low-frequency ``lf/`` relay
    zeros that field (just as it zeros ``foot_position_body``), and the high-rate
    copy is only alive while the motion service drives. ``robot_odom`` is a
    standard ROS type, so it decodes with the built-in typestore (no custom
    defs), and it carries a real integrated position that drifts slowly — exactly
    the odometry the EKF/placeholder wants."""

    def __init__(self, zenoh_session, robot_name: str,
                 odom_topic: str = "utlidar/robot_odom"):
        self._lock = threading.Lock()
        self._pose = Transform()           # T_odom_base
        self._lin = np.zeros(3)            # body-frame linear velocity (m/s)
        self._ang = np.zeros(3)            # body-frame angular velocity (rad/s)
        self._valid = False
        self._stamp_ns = 0
        self._decode = self._build_decoder()
        self._logged_failure = False

        key = f"{robot_name}/rt/{odom_topic}"
        try:
            zenoh_session.declare_subscriber(key, self._on_odom)
            log.info(f"[OdomTracker] Subscribed to '{key}'")
        except Exception as e:
            log.warning(f"[OdomTracker] Could not subscribe to '{key}': {e}")

    def _build_decoder(self):
        try:
            from rosbags.typesys import Stores, get_typestore
            ts = get_typestore(Stores.ROS2_HUMBLE)   # nav_msgs/Odometry is built in
            return lambda cdr: ts.deserialize_cdr(cdr, "nav_msgs/msg/Odometry")
        except Exception as e:
            log.warning(f"[OdomTracker] Decoder unavailable ({e}); odom off.")
            return None

    def _on_odom(self, sample):
        if self._decode is None:
            return
        try:
            m = self._decode(bytes(sample.payload))
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            tl = m.twist.twist.linear
            ta = m.twist.twist.angular
            pose = Transform.from_xyz_quat([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            with self._lock:
                self._pose = pose
                self._lin = np.array([tl.x, tl.y, tl.z], dtype=np.float64)
                self._ang = np.array([ta.x, ta.y, ta.z], dtype=np.float64)
                self._valid = True
                self._stamp_ns = time.time_ns()
        except Exception as e:
            if not self._logged_failure:
                log.warning(f"[OdomTracker] Odometry decode failed ({e}); "
                            "odom disabled. (logged once)")
                self._logged_failure = True

    def get(self):
        """Returns (T_odom_base, lin_vel_body, ang_vel_body, valid)."""
        with self._lock:
            return (Transform(self._pose.rotation.copy(), self._pose.translation.copy()),
                    self._lin.copy(), self._ang.copy(), self._valid)


# ─────────────────────────────────────────────────────────────────────────────
# Self-test:  python kinematics.py
# ─────────────────────────────────────────────────────────────────────────────

def _selftest():
    import numpy as np
    from vat_protocol import quat_from_rotvec

    model = SelfieStickModel(offset_xyz=(-0.2, 0.0, 0.55))

    # 1) Camera level above/behind base, body level → base directly below+front.
    cam_world = Transform.from_xyz_quat([1.0, 2.0, 0.6], quat_identity())
    base = model.base_from_camera_world(cam_world)
    # base should be +0.2 in x and -0.55 in z relative to camera (mount inverse)
    assert np.allclose(base.translation, [1.2, 2.0, 0.05], atol=1e-6), base.translation

    # 2) Round-trip: world←camera←base recovers camera.
    cam_back = base.compose(model.camera_in_base())
    assert np.allclose(cam_back.translation, cam_world.translation, atol=1e-6)

    # 3) Body roll right by 90° → stick swings camera; recovered base must stay put.
    roll = quat_from_rotvec([np.pi / 2, 0, 0])
    t_bc = model.camera_in_base()
    cam_world_rolled = Transform(roll, quat_rotate(roll, t_bc.translation) + np.array([0, 0, 0.05]))
    base2 = model.base_from_camera_world(cam_world_rolled)
    assert np.allclose(base2.translation, [0, 0, 0.05], atol=1e-6), base2.translation

    # 4) Camera height accounts for body tilt.
    h_level = model.camera_height(0.30, quat_identity())
    assert abs(h_level - (0.30 + 0.55)) < 1e-6, h_level
    # pitch forward 90°: stick's +z reach now points along world -x → contributes ~0 height
    pitch = quat_from_rotvec([0, np.pi / 2, 0])
    h_pitched = model.camera_height(0.30, pitch)
    assert abs(h_pitched - (0.30 + (-(-0.2)) * 0 + 0.0)) < 0.3  # roughly body height + stick x-proj

    # 5) Leg FK: all joints zero → leg hangs straight down from the hip.
    fr = leg_fk("FR", 0.0, 0.0, 0.0)
    assert np.allclose(fr["hip"], [0.1934, -0.0465, 0.0], atol=1e-6), fr["hip"]
    # foot directly below the hip in x, ~0.426 m below in z (thigh+calf straight)
    assert abs(fr["foot"][0] - fr["hip"][0]) < 1e-6, fr["foot"]
    assert abs(fr["foot"][2] - (-(_L_THIGH + _L_CALF))) < 1e-6, fr["foot"]
    assert fr["foot"][1] < fr["hip"][1]            # FR abduction link points -y (right)
    assert fr["knee"][2] > fr["foot"][2]           # knee sits above the foot
    # FL mirrors FR across the body centreline (y=0)
    fl = leg_fk("FL", 0.0, 0.0, 0.0)
    assert abs(fr["foot"][1] + fl["foot"][1]) < 1e-6, (fr["foot"][1], fl["foot"][1])
    # a bent stance (thigh fwd, calf back) lifts the foot toward the body
    stance = leg_fk("FR", 0.0, 0.8, -1.5)
    assert stance["foot"][2] > fr["foot"][2], stance["foot"]
    print(f"kinematics self-test OK  (h_level={h_level:.3f}m  h_pitched={h_pitched:.3f}m  "
          f"FR foot@zero={np.round(fr['foot'],3)}  stance_z={stance['foot'][2]:.3f})")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    _selftest()
