"""
VAT recorder — the common clock.
================================
Every recorded sample must land on **one** timeline so the streams can be composed
afterwards without assuming they share a rate. This module owns that timeline.

The session clock is the **robot capture clock in nanoseconds** — the clock that
``FrameInput.timestamp`` rides on, end to end:

    robot camera ts_ns  →  FRME header  →  IncomingFrame.timestamp
                        →  FrameInput.timestamp  →  engine.get_poses()
                        →  SubmapResult.cam_ts   →  PCOR.timestamp_ns

Four wire messages already carry a timestamp on (or derived from) that clock and
are recorded **verbatim** — never re-stamped with arrival time:

===============================  ==========================================
``FRME.timestamp_ns``            robot camera capture time
``POSE.timestamp_ns``            fused-pose estimator time (robot clock)
``PCOR.timestamp_ns``            capture time of the keyframe the cloud solved
``PSCF.timestamp_ns``            periscope slice capture time (robot clock)
===============================  ==========================================

The map-transport messages (``pack_pcd``, ``pack_manifest``, ``pack_bundle``,
``pack_block_push``, ``pack_trajectory``) carry **no timestamp** — only a
``map_version`` (and the manifest/bundle not even that). For those we record:

* ``wall_ns`` / ``mono_ns`` — local arrival, always;
* ``src_ts_ns`` — a *derived* session-clock estimate, ``local - offset``, using
  :class:`vat_telemetry.ClockOffsetEstimator` fed by every source-stamped message
  (the same minimum-filter the mapping server and viewer already use); and
* ``ts_src`` — ``"source"`` | ``"derived"`` | ``"wall"``, so a consumer can always
  tell an exact capture time from an estimate. Nothing is ever silently faked.

On top of that, :meth:`SessionClock.pin_version` builds a **map_version → capture
time** index from the two streams that tie the two together — ``pose_correction``
(exact: keyframe capture ns + map_version) and the server ``status`` JSON
(``newest_frame_robot_ns`` for that map_version). That index is what lets
``compose.py`` say "the map at this panorama frame's timestamp was version V".
"""

from __future__ import annotations

import os
import sys
import threading
import time
from dataclasses import dataclass, asdict
from typing import Dict, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_COMMON = os.path.join(os.path.dirname(os.path.dirname(_HERE)), "common")
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)

from vat_telemetry import ClockOffsetEstimator  # noqa: E402

# ts_src values
TS_SOURCE = "source"     # verbatim from the wire (a real capture timestamp)
TS_DERIVED = "derived"   # local arrival mapped onto the session clock via the offset
TS_WALL = "wall"         # local arrival only — no offset known yet, no source ts


@dataclass(frozen=True)
class Stamp:
    """One sample's position on every clock we know about."""
    src_ts_ns: int          # session clock (robot capture ns)
    ts_src: str             # TS_SOURCE / TS_DERIVED / TS_WALL
    wall_ns: int            # local wall clock at arrival (time.time_ns)
    mono_ns: int            # local monotonic at arrival (immune to NTP steps)
    latency_ms: float       # arrival − capture, above the estimator's baseline

    def as_dict(self) -> dict:
        return asdict(self)


class SessionClock:
    """Thread-safe common-clock bookkeeping shared by every stream recorder."""

    def __init__(self, window_s: float = 15.0, wall_ns=None, mono_ns=None):
        self._lock = threading.Lock()
        self._offset = ClockOffsetEstimator(window_s=window_s)
        self.window_s = float(window_s)
        # Injectable time sources. Production passes nothing; the offline self-test
        # drives a virtual clock so the derived-timestamp path (which depends on
        # arrival time) can be exercised deterministically instead of collapsing
        # ten seconds of synthetic capture into a few milliseconds of real time.
        self._wall_ns_fn = wall_ns or time.time_ns
        self._mono_ns_fn = mono_ns or time.monotonic_ns

        # Session epoch, captured once at construction.
        self.epoch_wall_ns = self._wall_ns_fn()
        self.epoch_mono_ns = self._mono_ns_fn()
        # Session-clock epoch: unknown until the first source-stamped sample tells
        # us the robot↔local offset. Recorded in meta.json once resolved.
        self.epoch_session_ns: Optional[int] = None

        # map_version → {"capture_ns", "source", "wall_ns"}
        self._version_pins: Dict[int, dict] = {}
        # counters for the session manifest
        self.n_source = 0
        self.n_source_unobserved = 0   # verbatim, but kept out of the offset baseline
        self.n_derived = 0
        self.n_wall = 0
        self._offset_first_s: Optional[float] = None
        self._offset_min_s: Optional[float] = None
        self._offset_max_s: Optional[float] = None

    # ── offset maintenance ───────────────────────────────────────────────────
    def observe_source(self, sender_ns: int, recv_wall_ns: Optional[int] = None) -> float:
        """Feed a source-stamped arrival to the offset estimator.

        Call this for every message that carries a robot-clock capture timestamp
        (frames, fused poses, periscope frames, pose corrections). Returns the
        latency above the window baseline, in seconds.
        """
        if sender_ns <= 0:
            return 0.0
        if recv_wall_ns is None:
            recv_wall_ns = self._wall_ns_fn()
        with self._lock:
            lat = self._offset.update(int(sender_ns), recv_wall_ns)
            off = self._offset.offset_s
            if off is not None:
                if self._offset_first_s is None:
                    self._offset_first_s = off
                self._offset_min_s = off if self._offset_min_s is None else min(self._offset_min_s, off)
                self._offset_max_s = off if self._offset_max_s is None else max(self._offset_max_s, off)
                if self.epoch_session_ns is None:
                    # Map the epoch we already captured back onto the session clock.
                    self.epoch_session_ns = int(self.epoch_wall_ns - off * 1e9)
            return lat

    @property
    def offset_s(self) -> Optional[float]:
        """local − robot, seconds (None until the first source-stamped sample)."""
        with self._lock:
            return self._offset.offset_s

    def to_session_ns(self, local_wall_ns: int) -> Optional[int]:
        """Local wall ns → session (robot capture) ns, or None if unknown yet."""
        off = self.offset_s
        return None if off is None else int(local_wall_ns - off * 1e9)

    def to_local_ns(self, session_ns: int) -> int:
        """Session (robot capture) ns → local wall ns. Identity if offset unknown."""
        with self._lock:
            return self._offset.to_local_ns(int(session_ns))

    # ── stamping ─────────────────────────────────────────────────────────────
    def stamp(self, src_ts_ns: Optional[int] = None, *, observe: bool = True) -> Stamp:
        """Build a :class:`Stamp` for a sample arriving now.

        Pass the wire's capture timestamp when the message has one — it is used
        verbatim. Pass ``None`` (or 0) for the map-transport messages that carry no
        timestamp; the session time is then *derived* from arrival and flagged as such.

        ``observe=False`` records the timestamp verbatim but keeps it OUT of the offset
        estimator. Required for ``pose_correction``: its timestamp is the capture time
        of a keyframe the cloud solved seconds ago, so ``arrival − sender`` is dominated
        by pipeline latency, not transport. Feeding it to a running-minimum filter is
        harmless while a real low-latency stream is also being observed, but if it were
        the *only* observed stream the baseline would be inflated by seconds and every
        derived map timestamp would land far too early.
        """
        wall_ns = self._wall_ns_fn()
        mono_ns = self._mono_ns_fn()
        if src_ts_ns and int(src_ts_ns) > 0:
            lat_s = self.observe_source(int(src_ts_ns), wall_ns) if observe else 0.0
            with self._lock:
                self.n_source += 1
                if not observe:
                    self.n_source_unobserved += 1
            return Stamp(int(src_ts_ns), TS_SOURCE, wall_ns, mono_ns, lat_s * 1e3)
        derived = self.to_session_ns(wall_ns)
        if derived is None:
            with self._lock:
                self.n_wall += 1
            return Stamp(wall_ns, TS_WALL, wall_ns, mono_ns, 0.0)
        with self._lock:
            self.n_derived += 1
        return Stamp(derived, TS_DERIVED, wall_ns, mono_ns, 0.0)

    # ── map_version ↔ capture-time index ─────────────────────────────────────
    @property
    def baseline_observed(self) -> bool:
        """True once a low-latency source-stamped stream has taught us the offset.

        Without it, every ``ts_src=derived`` timestamp is either absent (``wall``) or
        biased, so the map cannot be placed on the session clock. ``vat_record`` warns
        at startup when no stream capable of supplying it is enabled.
        """
        return self.offset_s is not None

    def now_wall_ns(self) -> int:
        return self._wall_ns_fn()

    def pin_version(self, map_version: int, capture_ns: int, source: str,
                    wall_ns: Optional[int] = None) -> None:
        """Record that ``map_version`` corresponds to capture time ``capture_ns``.

        ``source`` is where the pin came from — ``"pose_correction"`` (exact: the
        keyframe the cloud actually solved) or ``"status"`` (the newest frame the
        server had ingested for that version). An exact pin never gets overwritten
        by an approximate one.
        """
        if map_version is None or capture_ns is None or int(capture_ns) <= 0:
            return
        mv = int(map_version)
        with self._lock:
            old = self._version_pins.get(mv)
            if old is not None and old["source"] == "pose_correction" \
                    and source != "pose_correction":
                return
            self._version_pins[mv] = {
                "capture_ns": int(capture_ns), "source": source,
                "wall_ns": int(wall_ns if wall_ns is not None
                               else self._wall_ns_fn()),
            }

    def version_pin(self, map_version: int) -> Optional[dict]:
        with self._lock:
            p = self._version_pins.get(int(map_version))
            return dict(p) if p else None

    def version_pins(self) -> Dict[int, dict]:
        with self._lock:
            return {k: dict(v) for k, v in self._version_pins.items()}

    # ── reporting ────────────────────────────────────────────────────────────
    def meta(self) -> dict:
        """The ``clock`` block of ``meta.json``."""
        with self._lock:
            off = self._offset.offset_s
            return {
                "session_clock": "robot_capture_ns",
                "session_clock_note": (
                    "Source-stamped samples (FRME/POSE/PCOR/PSCF) carry this clock "
                    "verbatim; map-transport samples have no wire timestamp and are "
                    "flagged ts_src=derived (local arrival minus the robot->local "
                    "offset) or ts_src=wall (offset not yet known)."),
                "epoch_wall_ns": self.epoch_wall_ns,
                "epoch_mono_ns": self.epoch_mono_ns,
                "epoch_session_ns": self.epoch_session_ns,
                "offset_estimator": "vat_telemetry.ClockOffsetEstimator (min-filter)",
                "offset_window_s": self.window_s,
                "robot_to_local_offset_s": off,
            }

    def summary(self) -> dict:
        """Clock health for ``MANIFEST.json``."""
        with self._lock:
            return {
                "stamps_source": self.n_source,
                "stamps_source_unobserved": self.n_source_unobserved,
                "stamps_derived": self.n_derived,
                "stamps_wall": self.n_wall,
                "offset_s_first_known": self._offset_first_s,
                "offset_s_min": self._offset_min_s,
                "offset_s_max": self._offset_max_s,
                "offset_s_drift": (None if self._offset_min_s is None
                                   else self._offset_max_s - self._offset_min_s),
                "offset_s_final": self._offset.offset_s,
                "version_pins": len(self._version_pins),
            }


# ═════════════════════════════════════════════════════════════════════════════
# Self-test:  python tools/recorder/rec_clock.py
# ═════════════════════════════════════════════════════════════════════════════

def _selftest() -> None:
    c = SessionClock(window_s=5.0)

    # Before any source-stamped sample we cannot map arrival onto the session
    # clock — that MUST be flagged, not guessed.
    s0 = c.stamp(None)
    assert s0.ts_src == TS_WALL and c.offset_s is None
    assert c.to_session_ns(s0.wall_ns) is None

    # A source-stamped sample: recorded verbatim, and it teaches us the offset.
    now = time.time_ns()
    robot_ns = now - 3_000_000_000          # robot clock runs 3 s behind local
    s1 = c.stamp(robot_ns)
    assert s1.ts_src == TS_SOURCE and s1.src_ts_ns == robot_ns
    off = c.offset_s
    assert off is not None and abs(off - 3.0) < 0.5, off
    assert c.epoch_session_ns is not None

    # Now an unstamped sample is DERIVED onto the session clock, ~3 s behind wall.
    s2 = c.stamp(None)
    assert s2.ts_src == TS_DERIVED
    assert abs((s2.wall_ns - s2.src_ts_ns) / 1e9 - 3.0) < 0.5

    # Round-trip session→local→session.
    assert abs(c.to_session_ns(c.to_local_ns(robot_ns)) - robot_ns) < 5_000_000

    # Version pins: an exact pose_correction pin wins over a status approximation.
    c.pin_version(7, 1000, "status")
    c.pin_version(7, 1234, "pose_correction")
    c.pin_version(7, 9999, "status")
    assert c.version_pin(7)["capture_ns"] == 1234
    assert c.version_pin(7)["source"] == "pose_correction"
    c.pin_version(8, 0, "status")            # bogus capture time ignored
    assert c.version_pin(8) is None
    assert len(c.version_pins()) == 1

    m, sm = c.meta(), c.summary()
    assert m["session_clock"] == "robot_capture_ns"
    assert sm["stamps_source"] == 1 and sm["stamps_derived"] == 1 and sm["stamps_wall"] == 1

    # Injectable clock: with a virtual local clock exactly OFFSET ahead of the
    # source clock, a derived stamp must land on the true source time plus whatever
    # transport latency we simulate — this is the property compose.py relies on.
    virt = {"wall": 5_000_000_000, "mono": 0}
    vc = SessionClock(window_s=30.0, wall_ns=lambda: virt["wall"],
                      mono_ns=lambda: virt["mono"])
    OFF = 3_000_000_000
    for k in range(5):                       # source-stamped arrivals, 40 ms latency
        src = 1_000_000_000 + k * 100_000_000
        virt["wall"] = src + OFF + 40_000_000
        s = vc.stamp(src)
        assert s.ts_src == TS_SOURCE and s.src_ts_ns == src
    assert abs(vc.offset_s - (OFF + 40_000_000) / 1e9) < 1e-6
    virt["wall"] = 1_400_000_000 + OFF + 40_000_000
    d = vc.stamp(None)
    assert d.ts_src == TS_DERIVED and d.src_ts_ns == 1_400_000_000
    print(f"rec_clock self-test OK  (offset={off:.3f}s  pins={sm['version_pins']}  "
          f"virtual-clock derivation exact)")


if __name__ == "__main__":
    _selftest()
