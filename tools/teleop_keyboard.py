"""
VAT bring-up — keyboard teleop (client side)
============================================
Drive the Go2-W from your laptop, safely, over Zenoh. Streams ``cmd_vel`` at a
fixed rate to the robot's ``teleop_bridge`` which relays it to the Go2 sport
``Move`` API — and which STOPS the robot the moment this stream pauses (deadman).

    ZENOH_ROUTER=tcp/<router-ip>:7447 ROBOT_NAME=go2 python tools/teleop_keyboard.py

Controls (hold to drive — terminal key-repeat keeps it moving):
    W / S   forward / backward      (vx)
    A / D   turn left / right       (vyaw)
    Q / E   strafe left / right     (vy — limited on the wheeled Go2-W)
    SPACE   EMERGENCY STOP (latched, Damp).  Press R to re-arm.
    R       re-arm after an e-stop (sends a clean zero)
    - / =   decrease / increase speed scale
    0       zero velocity now (soft stop)
    Ctrl-C  quit (sends a stop on the way out)

Safety model: this process only ever makes the robot move while you are holding
a key AND packets are arriving. Release everything → it coasts to the deadman
within a few hundred ms and the robot stops. Keep the physical remote in hand.

Deps: eclipse-zenoh, numpy (via vat_protocol). Unix TTY only (uses termios).
"""

from __future__ import annotations

import os
import sys
import termios
import threading
import time
import tty

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, "common"))
import vat_protocol as proto  # noqa: E402

import zenoh  # noqa: E402

ROUTER = os.environ.get("ZENOH_ROUTER", "tcp/127.0.0.1:7447")
ROBOT_NAME = os.environ.get("ROBOT_NAME", "go2")
RATE_HZ = float(os.environ.get("TELEOP_RATE_HZ", "20.0"))
# A key "press" stays active for this long after the last keystroke. Terminal
# auto-repeat refreshes it (~30 ms), so holding a key = continuous motion; the
# moment you let go it lapses and we send zero.
KEY_HOLD_S = float(os.environ.get("TELEOP_KEY_HOLD_S", "0.25"))

# Per-axis step sizes (scaled by the live speed multiplier). Kept modest; the
# robot also hard-clamps these on its side via TELEOP_MAX_*.
STEP_VX = 0.25
STEP_VY = 0.15
STEP_VYAW = 0.5

KEY = proto.keys(ROBOT_NAME)["cmd_vel"]


class KeyState:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_key = ""
        self.last_key_ns = 0
        self.scale = 1.0
        self.estop = False
        self.quit = False

    def feed(self, ch: str):
        with self.lock:
            now = time.time_ns()
            if ch == " ":
                self.estop = True
            elif ch in ("r", "R"):
                self.estop = False
            elif ch in ("-", "_"):
                self.scale = max(0.1, self.scale - 0.1)
            elif ch in ("=", "+"):
                self.scale = min(1.0, self.scale + 0.1)
            elif ch == "0":
                self.last_key = ""
                self.last_key_ns = now
            elif ch in "wsadqeWSADQE":
                self.last_key = ch.lower()
                self.last_key_ns = now

    def current(self):
        with self.lock:
            return (self.last_key, self.last_key_ns, self.scale, self.estop)


def key_to_vel(ch: str, scale: float):
    vx = vy = vyaw = 0.0
    if ch == "w":
        vx = STEP_VX
    elif ch == "s":
        vx = -STEP_VX
    elif ch == "a":
        vyaw = STEP_VYAW
    elif ch == "d":
        vyaw = -STEP_VYAW
    elif ch == "q":
        vy = STEP_VY
    elif ch == "e":
        vy = -STEP_VY
    return vx * scale, vy * scale, vyaw * scale


def reader(state: KeyState):
    """Raw-mode stdin reader; runs in its own thread."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while not state.quit:
            ch = sys.stdin.read(1)
            if ch == "\x03":  # Ctrl-C
                state.quit = True
                break
            state.feed(ch)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", f'["{ROUTER}"]')
    conf.insert_json5("mode", '"client"')
    z = zenoh.open(conf)
    pub = z.declare_publisher(KEY)

    state = KeyState()
    t = threading.Thread(target=reader, args=(state,), daemon=True)
    t.start()

    sys.stderr.write(
        f"\r\nVAT teleop → '{KEY}' @ {RATE_HZ:.0f}Hz   "
        "WASD=drive  QE=strafe  SPACE=E-STOP  R=re-arm  -/=:speed  Ctrl-C=quit\r\n")
    sys.stderr.flush()

    period = 1.0 / max(RATE_HZ, 1.0)
    seq = 0
    try:
        while not state.quit:
            t0 = time.time()
            now = time.time_ns()
            ch, ts, scale, estop = state.current()
            flags = proto.CMDV_FLAG_ESTOP if estop else 0
            if estop:
                vx = vy = vyaw = 0.0
            else:
                active = ts and (now - ts) * 1e-9 <= KEY_HOLD_S
                vx, vy, vyaw = key_to_vel(ch, scale) if active else (0.0, 0.0, 0.0)

            seq += 1
            pub.put(proto.pack_cmd_vel(proto.CmdVel(
                vx=vx, vy=vy, vyaw=vyaw, flags=flags, seq=seq,
                timestamp_ns=now)))

            status = ("E-STOP" if estop
                      else f"vx={vx:+.2f} vy={vy:+.2f} vyaw={vyaw:+.2f} x{scale:.1f}")
            sys.stderr.write(f"\r{status:<48}")
            sys.stderr.flush()
            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        state.quit = True
        # send a few explicit stops so the robot halts promptly on exit
        for _ in range(5):
            seq += 1
            pub.put(proto.pack_cmd_vel(proto.CmdVel(
                vx=0.0, vy=0.0, vyaw=0.0, flags=0, seq=seq,
                timestamp_ns=time.time_ns())))
            time.sleep(0.02)
        z.close()
        sys.stderr.write("\r\n[teleop] stopped, link closed.\r\n")


if __name__ == "__main__":
    main()
