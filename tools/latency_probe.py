"""
VAT — Latency probe  (separate WiFi/medium latency from zenoh + processing)
===========================================================================
Answers "is my pose lag the WiFi, or zenoh/the pipeline?" by measuring the SAME
path two ways and differencing them:

  MEDIUM      = ICMP ping RTT to the target host        → the raw L3 network (WiFi/VPN)
  ZENOH       = query→echo→reply RTT to a probe_responder on that host
                                                        → medium + zenoh serialise/route/queue
  ZENOH OVER  = ZENOH_min − MEDIUM_min                  → what zenoh + the router ADD over the wire

Using the MINIMUMS removes transient queueing so the difference is the structural
zenoh overhead; the SPREAD (avg/p95/max, jitter) of each shows how much the link is
bufferbloating under load. Run it while the camera is streaming AND while it's stopped
to see how much the uplink contention costs — that's the pose-chug smoking gun.

It also passively watches the live pose stream and reports its arrival rate + jitter,
so you can correlate the probe with what the viewer actually receives.

Setup — on the target host (the ROBOT for the pose path), start the responder:
    ZENOH_ROUTER=tcp/<router-ip>:7447 PROBE_NAME=robot python tools/probe_responder.py
Then, from the client:
    ZENOH_ROUTER=tcp/<router-ip>:7447 TARGET_IP=<robot-ip> PROBE_NAME=robot \
        python tools/latency_probe.py
    # or:  make latency TARGET=<robot-ip> NAME=robot

Notes: ICMP and zenoh may not route identically (zenoh always goes via the router);
ping the ROUTER itself to characterise the client↔router leg in isolation.
"""

from __future__ import annotations

import os
import re
import statistics
import subprocess
import sys
import time

import zenoh

ZENOH_ROUTER = os.environ.get("ZENOH_ROUTER", "tcp/127.0.0.1:7447")
ROBOT_NAME = os.environ.get("ROBOT_NAME", "go2")
PROBE_NAME = os.environ.get("PROBE_NAME", "server")
TARGET_IP = os.environ.get("TARGET_IP") or (sys.argv[1] if len(sys.argv) > 1 else "")
N = int(os.environ.get("PROBE_N", "50"))
PROBE_BYTES = int(os.environ.get("PROBE_BYTES", "64"))
POSE_WATCH_S = float(os.environ.get("POSE_WATCH_S", "5.0"))
PROBE_KEY = f"vat/probe/{PROBE_NAME}"
POSE_KEY = f"{ROBOT_NAME}/prism/pose"


def _stats(xs):
    if not xs:
        return None
    xs = sorted(xs)
    p95 = xs[min(len(xs) - 1, int(0.95 * len(xs)))]
    return {"min": xs[0], "avg": statistics.mean(xs), "p95": p95, "max": xs[-1],
            "jitter": statistics.pstdev(xs) if len(xs) > 1 else 0.0, "n": len(xs)}


def _fmt(s):
    return "—" if not s else (f"min {s['min']:6.1f}  avg {s['avg']:6.1f}  "
                              f"p95 {s['p95']:6.1f}  max {s['max']:6.1f}  "
                              f"jitter {s['jitter']:5.1f}  (n={s['n']})")


def icmp_ping(ip, n):
    """Return per-ping RTTs (ms) via the system ping, or None if unavailable."""
    if not ip:
        return None
    try:
        out = subprocess.run(["ping", "-c", str(n), "-i", "0.2", ip],
                             capture_output=True, text=True, timeout=n * 0.4 + 15).stdout
    except Exception as e:
        print(f"[medium] ping failed ({e}); skipping ICMP.")
        return None
    rtts = [float(m) for m in re.findall(r"time=([\d.]+)\s*ms", out)]
    return rtts or None


def zenoh_rtt(z, key, n, payload):
    """Return per-query round-trip times (ms) to the echo responder."""
    rtts, misses = [], 0
    for _ in range(n):
        t0 = time.perf_counter()
        got = False
        try:
            for reply in z.get(key, payload=payload, timeout=2.0):
                if reply.ok:
                    got = True
                    break
        except Exception:
            got = False
        if got:
            rtts.append((time.perf_counter() - t0) * 1e3)
        else:
            misses += 1
        time.sleep(0.05)
    return rtts, misses


def watch_pose(z, seconds):
    """Passively measure the live pose stream's arrival rate + inter-arrival jitter."""
    arrivals = []
    sub = z.declare_subscriber(POSE_KEY, lambda s: arrivals.append(time.perf_counter()))
    time.sleep(seconds)
    try:
        sub.undeclare()
    except Exception:
        pass
    if len(arrivals) < 2:
        return None, 0.0
    gaps = [(b - a) * 1e3 for a, b in zip(arrivals, arrivals[1:])]
    rate = len(arrivals) / seconds
    return _stats(gaps), rate


def main():
    print(f"── VAT latency probe ── router={ZENOH_ROUTER}  target={TARGET_IP or '(none)'}  "
          f"responder='{PROBE_KEY}'\n")
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", f'["{ZENOH_ROUTER}"]')
    z = zenoh.open(conf)
    try:
        med = _stats(icmp_ping(TARGET_IP, N)) if TARGET_IP else None
        zrtts, misses = zenoh_rtt(z, PROBE_KEY, N, bytes(PROBE_BYTES))
        zen = _stats(zrtts)
        pose_gaps, pose_rate = watch_pose(z, POSE_WATCH_S)

        print(f"MEDIUM  (ICMP RTT, ms)   {_fmt(med)}")
        print(f"ZENOH   (echo RTT,  ms)  {_fmt(zen)}"
              + (f"   [{misses} misses/timeouts]" if misses else ""))
        if med and zen:
            overhead = zen["min"] - med["min"]
            print(f"ZENOH OVERHEAD (min−min) {overhead:6.1f} ms   "
                  f"→ {'zenoh/router adds little; latency is the MEDIUM' if overhead < med['min'] else 'zenoh/router adds a large share'}")
        elif zen and not med:
            print("ZENOH OVERHEAD           (give TARGET_IP to compare against ICMP)")
        print()
        if pose_gaps:
            print(f"LIVE POSE  {pose_rate:5.1f} msg/s   inter-arrival gap(ms) {_fmt(pose_gaps)}")
            print("  (a smooth link shows a tight gap ≈ 1000/PUBLISH_HZ; big max/jitter = drops/bufferbloat)")
        else:
            print("LIVE POSE  no samples (is the robot publishing?)")
    finally:
        z.close()


if __name__ == "__main__":
    main()
