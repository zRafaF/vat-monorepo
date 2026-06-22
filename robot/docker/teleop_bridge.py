"""
VAT — Teleop Bridge  (robot side)
=================================
Turns the network ``cmd_vel`` stream into Go2-W sport-mode motion, *safely*.

Data path::

    client/server  ──cmd_vel (Zenoh, ~20 Hz)──▶  teleop_bridge  ──Request──▶  /api/sport/request
                                                     │
                                                     └─ watchdog + e-stop + clamps

The robot is commanded to move **only while fresh commands keep arriving**.
This is the core safety property: if the sender crashes or the link drops, the
watchdog stops the robot within ``TELEOP_TIMEOUT_S``.  A latched e-stop bit in
the command forces the robot into Damp (compliant) immediately.

Go2 high-level sport API (see unitree_ros2 ros2_sport_client):
  * Move      api_id 1008   parameter {"x":vx, "y":vy, "z":vyaw}
  * StopMove  api_id 1003   (stop, keep stance)
  * Damp      api_id 1001   (motors compliant — the soft e-stop)
  * BalanceStand api_id 1002 (used to (re)enter a controllable standing state)
Published as unitree_api/msg/Request on /api/sport/request (fire-and-forget).

Safety layers (defence in depth):
  1. The physical Unitree remote ALWAYS overrides this — keep it in hand.
  2. Deadman watchdog: no cmd within TELEOP_TIMEOUT_S → StopMove.
  3. Latched e-stop bit → Damp, ignores motion until cleared.
  4. Velocity clamps: hard caps on vx/vy/vyaw from env.

⚠️  Start conservative (small caps), robot in open space, finger on the remote.

Environment
-----------
  ROBOT_NAME, ZENOH_CONNECT, RMW/CycloneDDS (inherited from start.sh),
  TELEOP_RATE_HZ, TELEOP_TIMEOUT_S, TELEOP_MAX_VX, TELEOP_MAX_VY, TELEOP_MAX_VYAW
"""

from __future__ import annotations

import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import zenoh

import vat_protocol as proto

# Go2 sport-mode API ids
API_DAMP = 1001
API_BALANCE_STAND = 1002
API_STOP_MOVE = 1003
API_MOVE = 1008

ROBOT_NAME = os.environ.get("ROBOT_NAME", "go2")
ZENOH_CONNECT = os.environ.get("ZENOH_CONNECT", "tcp/127.0.0.1:7447")

RATE_HZ = float(os.environ.get("TELEOP_RATE_HZ", "20.0"))
TIMEOUT_S = float(os.environ.get("TELEOP_TIMEOUT_S", "0.3"))
MAX_VX = float(os.environ.get("TELEOP_MAX_VX", "0.3"))
MAX_VY = float(os.environ.get("TELEOP_MAX_VY", "0.2"))
MAX_VYAW = float(os.environ.get("TELEOP_MAX_VYAW", "0.6"))

_KEYS = proto.keys(ROBOT_NAME)


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class TeleopBridge(Node):
    def __init__(self, z: zenoh.Session):
        super().__init__("vat_teleop_bridge")
        self._z = z

        # Publisher to the Go2 sport request channel. unitree_api must be on the
        # overlay (built into the image); import lazily so a missing overlay
        # produces a clear message rather than an import error at module load.
        try:
            from unitree_api.msg import Request
        except Exception as e:  # pragma: no cover
            self.get_logger().error(
                f"unitree_api not available ({e}); teleop cannot publish sport "
                "requests. Rebuild the image with unitree_api in the overlay.")
            raise
        self._Request = Request

        # Commands are control-critical: RELIABLE, keep-last small depth.
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(Request, "/api/sport/request", qos)

        # Latest command state (guarded by lock)
        self._lock = threading.Lock()
        self._last_cmd = proto.CmdVel()
        self._last_rx_ns = 0           # arrival time of last cmd (0 = none yet)
        self._estop_latched = False
        self._req_id = 0
        # so we don't spam identical stop/damp requests every tick
        self._last_action = None       # "move" | "stop" | "damp" | None

        # liveliness so the link checker can see the teleop bridge is up
        try:
            self._live = z.liveliness().declare_token(_KEYS["live_teleop"])
        except Exception:
            self._live = None

        z.declare_subscriber(_KEYS["cmd_vel"], self._on_cmd_vel)
        self.get_logger().info(
            f"[Teleop] cmd_vel←'{_KEYS['cmd_vel']}'  →'/api/sport/request'  "
            f"@ {RATE_HZ:.0f}Hz  deadman={TIMEOUT_S*1000:.0f}ms  "
            f"caps vx<={MAX_VX} vy<={MAX_VY} vyaw<={MAX_VYAW}")

        # control loop
        self._stop_thread = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # -- zenoh callback -------------------------------------------------------
    def _on_cmd_vel(self, sample):
        try:
            cmd = proto.unpack_cmd_vel(bytes(sample.payload))
        except proto.ProtocolError as e:
            self.get_logger().warn(f"[Teleop] bad cmd_vel: {e}")
            return
        with self._lock:
            self._last_cmd = cmd
            self._last_rx_ns = time.time_ns()
            if cmd.estop:
                self._estop_latched = True
            # An explicit zero-velocity command WITHOUT the estop bit clears a
            # previous latch (the driver's way of "re-arming" after an e-stop).
            elif (not cmd.estop and cmd.vx == 0.0 and cmd.vy == 0.0
                  and cmd.vyaw == 0.0):
                self._estop_latched = False

    # -- request helpers ------------------------------------------------------
    def _send(self, api_id: int, params: dict | None = None):
        req = self._Request()
        self._req_id += 1
        req.header.identity.id = int(self._req_id)
        req.header.identity.api_id = int(api_id)
        req.parameter = json.dumps(params) if params else ""
        self._pub.publish(req)

    # -- control loop ---------------------------------------------------------
    def _run(self):
        period = 1.0 / max(RATE_HZ, 1.0)
        last_log = time.time()
        sent = 0
        while not self._stop_thread:
            t0 = time.time()
            now = time.time_ns()
            with self._lock:
                cmd = self._last_cmd
                last_rx = self._last_rx_ns
                estop = self._estop_latched
            age = (now - last_rx) * 1e-9 if last_rx else 1e9

            if estop:
                # Latched soft e-stop: keep the robot compliant. Re-send slowly.
                if self._last_action != "damp":
                    self._send(API_DAMP)
                    self._last_action = "damp"
                    self.get_logger().warn("[Teleop] E-STOP latched → Damp")
            elif age > TIMEOUT_S:
                # Deadman: no fresh command → stop. Send a couple times then idle.
                if self._last_action != "stop":
                    self._send(API_STOP_MOVE)
                    self._last_action = "stop"
                    if last_rx:
                        self.get_logger().warn(
                            f"[Teleop] deadman ({age*1000:.0f}ms silent) → StopMove")
            else:
                vx = _clamp(cmd.vx, -MAX_VX, MAX_VX)
                vy = _clamp(cmd.vy, -MAX_VY, MAX_VY)
                vyaw = _clamp(cmd.vyaw, -MAX_VYAW, MAX_VYAW)
                if vx == 0.0 and vy == 0.0 and vyaw == 0.0:
                    if self._last_action != "stop":
                        self._send(API_STOP_MOVE)
                        self._last_action = "stop"
                else:
                    # Move must be streamed continuously (it decays); send every tick.
                    self._send(API_MOVE, {"x": vx, "y": vy, "z": vyaw})
                    self._last_action = "move"
                    sent += 1

            if time.time() - last_log > 10:
                self.get_logger().info(
                    f"[Teleop] action={self._last_action} moves_sent={sent} "
                    f"estop={estop} last_age={age*1000:.0f}ms")
                last_log = time.time()
            time.sleep(max(0.0, period - (time.time() - t0)))

    def stop(self):
        self._stop_thread = True
        try:
            self._send(API_STOP_MOVE)
        except Exception:
            pass


def _open_session() -> zenoh.Session:
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ZENOH_CONNECT}"]')
    conf.insert_json5("mode", '"peer"')
    while True:
        try:
            return zenoh.open(conf)
        except Exception as e:
            print(f"[Teleop] Zenoh connect failed: {e} — retrying in 5s")
            time.sleep(5)


def main():
    rclpy.init()
    print(f"[Teleop] Connecting to Zenoh at {ZENOH_CONNECT}...")
    z = _open_session()
    print("[Teleop] Connected.")
    node = TeleopBridge(z)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()
        z.close()


if __name__ == "__main__":
    main()
