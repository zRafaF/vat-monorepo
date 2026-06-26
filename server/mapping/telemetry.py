"""
VAT mapping server — telemetry (clock sync + throughput).

Two small, dependency-free, unit-testable helpers used to profile the pipeline:

* :class:`ClockOffsetEstimator` — the robot has no synced clock (it drifts), so its
  message timestamps can't be compared to the NTP-synced server/client clocks
  directly. One-way ABSOLUTE latency is unobservable without a round trip, but
  ``recv - send = offset + transit`` with ``transit >= 0``, so the running MINIMUM
  of ``(recv - send)`` over a sliding window tracks ``offset (+ min transit)`` and
  follows slow robot-clock drift (the classic NTP/chrony minimum filter). That gives
  (a) a robot→local clock conversion and (b) the *latency above the baseline* (queue/
  jitter) for each arrival — which is what spots stalls and drops.

* :class:`ThroughputMeter` — EMA of bytes/s and msgs/s for a stream.

Absolute one-way latency would need a round-trip ping (a robot-side queryable that
echoes its clock); see docs. For the POC the relative (above-floor) latency is
enough to profile stalls, and the offset gives a usable robot→local conversion.
"""

from __future__ import annotations

import time
from collections import deque


class ClockOffsetEstimator:
    """Estimate (robot_clock → local_clock) offset from one-way arrivals."""

    def __init__(self, window_s: float = 15.0):
        self.window_s = float(window_s)
        self._samples: deque = deque()       # (local_recv_s, delta_s)
        self.offset_s: float | None = None    # local - robot (seconds)
        self.last_latency_s: float = 0.0      # transit above the window minimum

    def update(self, send_ns_robot: int, recv_ns_local: int | None = None) -> float:
        """Feed one arrival; returns the above-floor one-way latency estimate (s)."""
        recv_ns_local = recv_ns_local if recv_ns_local is not None else time.time_ns()
        now = recv_ns_local * 1e-9
        delta = now - send_ns_robot * 1e-9          # = offset + transit
        self._samples.append((now, delta))
        cutoff = now - self.window_s
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()
        self.offset_s = min(d for _, d in self._samples)
        self.last_latency_s = max(delta - self.offset_s, 0.0)
        return self.last_latency_s

    def to_local_ns(self, robot_ns: int) -> int:
        """Convert a robot-clock timestamp to the local (server) clock."""
        off = self.offset_s if self.offset_s is not None else 0.0
        return int(robot_ns + off * 1e9)

    def reset(self):
        self._samples.clear()
        self.offset_s = None
        self.last_latency_s = 0.0


class ThroughputMeter:
    """EMA bytes/s and msgs/s for a stream (call :meth:`add` per message)."""

    def __init__(self, tau_s: float = 2.0):
        self.tau_s = float(tau_s)
        self._last_t: float | None = None
        self.bps: float = 0.0       # bytes/s
        self.mps: float = 0.0       # messages/s
        self.total_bytes: int = 0
        self.total_msgs: int = 0

    def add(self, nbytes: int, now: float | None = None):
        now = now if now is not None else time.monotonic()
        self.total_bytes += int(nbytes)
        self.total_msgs += 1
        if self._last_t is None:
            self._last_t = now
            return
        dt = max(now - self._last_t, 1e-6)
        self._last_t = now
        a = 1.0 - pow(2.71828, -dt / self.tau_s)    # EMA weight for this interval
        self.bps += a * (nbytes / dt - self.bps)
        self.mps += a * (1.0 / dt - self.mps)

    def decay(self, now: float | None = None):
        """Call when idle so the rate falls toward 0 instead of holding the last value."""
        now = now if now is not None else time.monotonic()
        if self._last_t is None:
            return
        dt = max(now - self._last_t, 1e-6)
        a = 1.0 - pow(2.71828, -dt / self.tau_s)
        self.bps += a * (0.0 - self.bps)
        self.mps += a * (0.0 - self.mps)

    @property
    def kbps(self) -> float:
        return self.bps / 1024.0
