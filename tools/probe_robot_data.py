"""
VAT bring-up — robot DATA-SOURCE probe
======================================
Answers one question the pose estimator depends on: **which robot topics/fields
actually carry usable data on THIS Go2-W?** (The project caveat warns some
velocity/odom fields are zeroed or absent — this tool proves it either way.)

What it does, text-only (no Rerun/OpenCV), from any machine that can reach the router:
  1. Queries the bridge's topic list (`{ROBOT}/system/get_topics`) and prints every
     ROS topic + type advertised on Zenoh — so we can spot *other* topics that
     might hold usable odom/velocity/pose data.
  2. Subscribes to each topic for a capture window, decodes the CDR with `rosbags`
     (no ROS install), and walks EVERY numeric field.
  3. Reports, per topic, only the fields that are **nonzero or changing** (the
     data-bearing ones), and how many were all-zero/constant.
  4. Prints a POSE-CRITICAL VERDICT: wheel odometry (`LowState.motor_state[*].dq`),
     IMU accel/gyro/quaternion, `SportModeState.velocity` / `yaw_speed` /
     `body_height`, and any `nav_msgs/Odometry`-style topics.

USAGE (run on your laptop; paste the whole printout back):

    make probe_robot                       # 15 s window, default topics
    PROBE_S=25 make probe_robot            # longer window
    PROBE_ALL=1 make probe_robot           # also probe image/cloud topics (heavy)

  >> IMPORTANT: DRIVE / MOVE the dog during the capture window. A field that is
     zero only because the robot is stationary looks the same as a dead field
     otherwise. Moving it makes real velocity/odometry channels light up as
     "CHANGING", which is exactly the signal we need.

Env: ZENOH_ROUTER (or ROUTER_IP:ROUTER_PORT), ROBOT_NAME (go2), PROBE_S (15),
     PROBE_ALL (0), plus optional explicit topic args (ROS names, e.g. /lowstate).
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
import threading
from collections import defaultdict

import zenoh

# ── config ───────────────────────────────────────────────────────────────────
ROUTER = os.environ.get("ZENOH_ROUTER") or (
    f"tcp/{os.environ.get('ROUTER_IP','127.0.0.1')}:{os.environ.get('ROUTER_PORT','7447')}")
ROBOT_NAME = os.environ.get("ROBOT_NAME", "go2")
PROBE_S = float(os.environ.get("PROBE_S", "15"))
PROBE_ALL = os.environ.get("PROBE_ALL", "0") not in ("0", "", "false", "False")

# Topics we ALWAYS probe even if discovery misses them (the pose-critical ones).
ALWAYS = ["/lowstate", "/lf/lowstate", "/sportmodestate", "/lf/sportmodestate"]
# Types we skip by default (bulky media); still listed. Override with PROBE_ALL=1.
HEAVY_TYPE_HINTS = ("image", "compressedimage", "pointcloud", "laserscan", "camerainfo")

# ── unitree message defs (import the SAME defs the estimator uses; embed fallback)
try:
    _ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, os.path.join(_ROOT, "common"))
    sys.path.insert(0, os.path.join(_ROOT, "robot", "docker"))
    from kinematics import _UNITREE_MSG_DEFS as UNITREE_DEFS      # noqa: E402
    _DEFS_SRC = "kinematics._UNITREE_MSG_DEFS (matches the estimator)"
except Exception as _e:                                          # pragma: no cover
    UNITREE_DEFS = {}
    _DEFS_SRC = f"embedded fallback (kinematics import failed: {_e})"


def build_typestore():
    """ROS2 Humble typestore + the unitree_go defs registered on top."""
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    ts = get_typestore(Stores.ROS2_HUMBLE)
    reg = {}
    for name, defn in UNITREE_DEFS.items():
        try:
            reg.update(get_types_from_msg(defn, name))
        except Exception:
            pass
    if reg:
        try:
            ts.register(reg)
        except Exception:
            pass
    return ts


# ── field walking / stats ─────────────────────────────────────────────────────
import numpy as np  # noqa: E402


class Leaf:
    __slots__ = ("n", "nz", "mn", "mx", "nonfin", "first", "last", "changed")

    def __init__(self):
        self.n = 0
        self.nz = 0
        self.mn = math.inf
        self.mx = -math.inf
        self.nonfin = 0
        self.first = None
        self.last = None
        self.changed = False

    def add(self, v):
        v = float(v)
        self.n += 1
        if not math.isfinite(v):
            self.nonfin += 1
            self.last = v
            return
        if abs(v) > 1e-9:
            self.nz += 1
        self.mn = min(self.mn, v)
        self.mx = max(self.mx, v)
        if self.first is None:
            self.first = v
        elif abs(v - self.first) > 1e-6:
            self.changed = True
        self.last = v

    def bearing(self):
        """True if this field carries data (ever nonzero or ever changed)."""
        return self.changed or self.nz > 0

    def fmt(self):
        rng = f"[{self.mn:+.3f},{self.mx:+.3f}]" if self.n and math.isfinite(self.mn) else "[--]"
        tag = "CHANGING" if self.changed else ("nonzero" if self.nz else "zero")
        if self.nonfin:
            tag += f" NONFINITE×{self.nonfin}"
        last = f"{self.last:+.3f}" if (self.last is not None and math.isfinite(self.last)) else "nan"
        return f"last={last:>9}  range={rng:<20} {tag}"


def _fields(o):
    s = getattr(o, "__slots__", None)
    if s:
        return [f for f in s if not f.startswith("__")]
    d = getattr(o, "__dict__", None)
    if d:
        return [k for k in d if not k.startswith("__")]
    return [a for a in dir(o) if not a.startswith("_")
            and not callable(getattr(o, a, None))]


def walk(o, path, stats, depth=0):
    """Recurse a decoded message; record every numeric leaf into stats[path]=Leaf."""
    if o is None or depth > 8:
        return
    if isinstance(o, (bool, int, float, np.integer, np.floating)):
        stats[path].add(o)
        return
    if isinstance(o, (str, bytes, bytearray)):
        return
    if isinstance(o, np.ndarray):
        arr = o.ravel()
        if arr.dtype.kind in "biufc" and arr.size <= 32:
            for i, v in enumerate(arr):
                stats[f"{path}[{i}]"].add(v)
        elif arr.dtype.kind in "biuf":
            # big numeric array → single summary leaf
            lf = stats[f"{path}[{arr.size}]"]
            for v in arr:
                lf.add(v)
        return
    if isinstance(o, (list, tuple)):
        if len(o) <= 32:
            for i, v in enumerate(o):
                walk(v, f"{path}[{i}]", stats, depth + 1)
        else:
            try:
                arr = np.asarray(o, dtype=float).ravel()
                lf = stats[f"{path}[{arr.size}]"]
                for v in arr:
                    lf.add(v)
            except Exception:
                pass
        return
    fs = _fields(o)
    for f in fs:
        try:
            child = getattr(o, f)
        except Exception:
            continue
        walk(child, f"{path}.{f}" if path else f, stats, depth + 1)


# ── per-topic collector ────────────────────────────────────────────────────────
class TopicProbe:
    def __init__(self, ros_name, ros_type, ts):
        self.ros_name = ros_name
        self.ros_type = ros_type
        self.ts = ts
        self.key = f"{ROBOT_NAME}/rt{ros_name}"
        self.count = 0
        self.bytes = 0
        self.t_first = None
        self.t_last = None
        self.decoded = 0
        self.decode_err = None
        self.stats = defaultdict(Leaf)
        self._lock = threading.Lock()

    def on_sample(self, sample):
        try:
            payload = bytes(sample.payload)
        except Exception:
            return
        now = time.time()
        with self._lock:
            self.count += 1
            self.bytes += len(payload)
            if self.t_first is None:
                self.t_first = now
            self.t_last = now
        if not self.ros_type:
            return
        try:
            msg = self.ts.deserialize_cdr(payload, self.ros_type)
        except Exception as e:
            if self.decode_err is None:
                self.decode_err = str(e)
            return
        with self._lock:
            self.decoded += 1
            try:
                walk(msg, "", self.stats)
            except Exception as e:
                if self.decode_err is None:
                    self.decode_err = f"walk: {e}"

    def hz(self):
        if self.count < 2 or not self.t_first or self.t_last <= self.t_first:
            return self.count / max(PROBE_S, 1e-3)
        return (self.count - 1) / (self.t_last - self.t_first)


# ── discovery ───────────────────────────────────────────────────────────────
def discover(z):
    topics = {}
    try:
        replies = list(z.get(f"{ROBOT_NAME}/system/get_topics", timeout=3.0))
        for r in replies:
            if r.ok:
                topics.update(json.loads(bytes(r.result.payload).decode()))
    except Exception as e:
        print(f"  ! topic discovery failed: {e}")
    return topics


def is_heavy(ros_type):
    t = (ros_type or "").lower()
    return any(h in t for h in HEAVY_TYPE_HINTS)


# ── verdict helpers ───────────────────────────────────────────────────────────
def _match_leaves(probe, needle):
    return {p: lf for p, lf in probe.stats.items() if needle in p}


def verdict(probes):
    line = "─" * 78
    print(f"\n{line}\n POSE-CRITICAL VERDICT\n{line}")

    def find_topic(type_needle=None, path_needle=None):
        out = []
        for p in probes:
            if type_needle and type_needle.lower() in (p.ros_type or "").lower():
                out.append(p)
            elif path_needle and any(path_needle in k for k in p.stats):
                out.append(p)
        return out

    # 1) WHEEL ODOMETRY — the sole source of translation in the live estimator.
    print("\n[1] WHEEL ODOMETRY  (LowState.motor_state[*].dq × WHEEL_RADIUS)")
    ls = find_topic(path_needle="motor_state")
    if not ls:
        print("    ✗ No topic with motor_state decoded. LowState not flowing/decoding.")
    for p in ls:
        dq = {p2: lf for p2, lf in p.stats.items() if p2.startswith("motor_state[") and p2.endswith("].dq")}
        live = sorted(int(k[len("motor_state["):k.index("]")]) for k, lf in dq.items() if lf.bearing())
        allidx = sorted(int(k[len("motor_state["):k.index("]")]) for k in dq)
        print(f"    {p.key}: {len(allidx)} motors decoded; dq DATA-BEARING at indices {live or 'NONE'}")
        wheels = [i for i in (12, 13, 14, 15) if i in live]
        if wheels == [12, 13, 14, 15]:
            print("    ✓ Wheel motors 12–15 all report changing dq → wheel odometry is USABLE"
                  " (calibrate WHEEL_RADIUS).")
        elif wheels:
            print(f"    ~ Only some expected wheel indices live: {wheels}. Check contact/motion.")
        elif live:
            print(f"    ! dq is live but NOT at 12–15 — wheels may be at indices {live}."
                  " WHEEL_IDX likely wrong for this firmware.")
        else:
            print("    ✗ No motor dq is changing. If the dog WAS moving during capture,"
                  " wheel odometry is DEAD → the estimator cannot dead-reckon translation.")

    # 2) IMU
    print("\n[2] IMU  (LowState.imu_state: quaternion / gyroscope / accelerometer)")
    imu = find_topic(path_needle="imu_state.accelerometer")
    if not imu:
        print("    ✗ No imu_state decoded.")
    for p in imu:
        acc = [p.stats.get(f"imu_state.accelerometer[{i}]") for i in range(3)]
        gyr = [p.stats.get(f"imu_state.gyroscope[{i}]") for i in range(3)]
        quat = [p.stats.get(f"imu_state.quaternion[{i}]") for i in range(4)]
        amag = None
        if all(a and a.last is not None and math.isfinite(a.last) for a in acc):
            amag = math.sqrt(sum(a.last ** 2 for a in acc))
        qn = None
        if all(q and q.last is not None and math.isfinite(q.last) for q in quat):
            qn = math.sqrt(sum(q.last ** 2 for q in quat))
        print(f"    {p.key}: |accel|≈{amag if amag is None else round(amag,2)} m/s² "
              f"(expect ~9.8 at rest); |quat|≈{qn if qn is None else round(qn,3)} (expect ~1.0)")
        acc_ok = amag is not None and 3.0 < amag < 40.0
        gyr_live = any(g and g.bearing() for g in gyr)
        quat_ok = qn is not None and 0.5 < qn < 1.5
        print(f"    {'✓' if acc_ok else '✗'} accelerometer plausible   "
              f"{'✓' if quat_ok else '✗'} quaternion normalised   "
              f"gyro {'CHANGING' if gyr_live else 'flat (ok if dog was still)'}")

    # 3) SportModeState velocity / yaw_speed / body_height
    print("\n[3] SPORTMODESTATE  (velocity / yaw_speed / body_height)")
    sp = find_topic(path_needle="body_height")
    if not sp:
        print("    ✗ No SportModeState decoded (checked both lf/ and full).")
    for p in sp:
        bh = p.stats.get("body_height")
        ys = p.stats.get("yaw_speed")
        vel = [p.stats.get(f"velocity[{i}]") for i in range(3)]
        vel_live = any(v and v.bearing() for v in vel)
        print(f"    {p.key}:")
        if bh:
            print(f"        body_height  {bh.fmt()}  "
                  f"→ {'USABLE for Z' if bh.bearing() or (bh.last and bh.last>0.05) else 'flat/zero — Z falls back to constant'}")
        print(f"        velocity[0..2] {'CHANGING → usable' if vel_live else 'ZERO/flat → confirms docs: velocity is zeroed on lf/'}")
        if ys:
            print(f"        yaw_speed    {ys.fmt()}")

    # 4) Any odom-style topic
    print("\n[4] ODOMETRY-STYLE TOPICS  (nav_msgs/Odometry, *odom*, pose/twist)")
    od = [p for p in probes if ("odom" in p.ros_name.lower()
          or "odometry" in (p.ros_type or "").lower())]
    if not od:
        print("    (none advertised — consistent with 'Go2-W has no robot_odom')")
    for p in od:
        bearing = [k for k, lf in p.stats.items() if lf.bearing()]
        print(f"    {p.key}  type={p.ros_type}  samples={p.count} decoded={p.decoded}")
        print(f"        data-bearing fields: {len(bearing)} "
              f"{'→ WORTH TESTING as a translation source' if bearing else '→ present but all zero (not usable)'}")

    print(f"\n{line}\n Paste this whole printout back. Best signal comes from a run where"
          f"\n the dog was DRIVEN during the {PROBE_S:.0f}s window.\n{line}")


# ── main ────────────────────────────────────────────────────────────────────
def main():
    print("=" * 78)
    print(f" VAT robot data-source probe   router={ROUTER}  robot={ROBOT_NAME}")
    print(f" window={PROBE_S:.0f}s   msg-defs: {_DEFS_SRC}")
    print("=" * 78)

    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", f'["{ROUTER}"]')
    try:
        z = zenoh.open(conf)
    except Exception as e:
        print(f"FATAL: cannot open Zenoh session: {e}")
        sys.exit(1)

    ts = build_typestore()

    # discovery
    print("\n[discovery] ROS topics advertised by the bridge:")
    advertised = discover(z)
    if advertised:
        for name, typ in sorted(advertised.items()):
            print(f"    {name:38s} {typ}")
    else:
        print("    (no reply — is the robot container / dynamic_bridge.py running?)")

    # build probe set: explicit args > discovered ∪ ALWAYS
    explicit = [a if a.startswith("/") else "/" + a for a in sys.argv[1:]]
    if explicit:
        want = {n: advertised.get(n, "") for n in explicit}
    else:
        want = dict(advertised)
        for n in ALWAYS:
            want.setdefault(n, "")

    probes, skipped = [], []
    for name, typ in sorted(want.items()):
        if is_heavy(typ) and not PROBE_ALL:
            skipped.append((name, typ))
            continue
        p = TopicProbe(name, typ, ts)
        try:
            z.declare_subscriber(p.key, p.on_sample)
            probes.append(p)
        except Exception as e:
            print(f"    ! subscribe failed {p.key}: {e}")
    if skipped:
        print("\n[skipped as heavy — rerun with PROBE_ALL=1 to include]:")
        for name, typ in skipped:
            print(f"    {name:38s} {typ}")

    print(f"\n[capture] listening {PROBE_S:.0f}s on {len(probes)} topics …")
    print("          >> DRIVE / MOVE the dog now so real motion channels light up. <<")
    time.sleep(PROBE_S)

    # report
    line = "─" * 78
    print(f"\n{line}\n PER-TOPIC REPORT  (only fields that are nonzero or changing)\n{line}")
    for p in sorted(probes, key=lambda x: x.ros_name):
        hz = p.hz()
        head = (f"\nTOPIC {p.key}\n   type={p.ros_type or '(unknown)'}   "
                f"samples={p.count}  ~{hz:.1f} Hz  ~{(p.bytes/max(p.count,1)):.0f} B/msg")
        print(head)
        if p.count == 0:
            print("   ✗ NO DATA received (topic advertised but silent, or QoS/DDS issue).")
            continue
        if p.ros_type and p.decoded == 0:
            print(f"   ! could not decode ({p.decode_err}). Raw bytes only.")
            continue
        bearing = [(k, lf) for k, lf in p.stats.items() if lf.bearing()]
        zeros = len(p.stats) - len(bearing)
        if not bearing:
            print(f"   (all {len(p.stats)} numeric fields ZERO/constant this window)")
        for k, lf in sorted(bearing)[:80]:
            print(f"     {k:34s} {lf.fmt()}")
        if len(bearing) > 80:
            print(f"     … +{len(bearing) - 80} more data-bearing fields")
        if zeros:
            print(f"   ({zeros} field(s) all-zero/constant, suppressed)")

    verdict(probes)
    z.close()


if __name__ == "__main__":
    main()
