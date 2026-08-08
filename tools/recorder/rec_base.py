"""
VAT recorder — stream-recorder base class.
=========================================
One tiny contract shared by every stream so ``vat_record.py`` can treat them
uniformly: build → :meth:`attach` a Zenoh session → periodic :meth:`tick` →
:meth:`close` → :meth:`summary`.

Two invariants the base enforces, because getting either wrong ruins a capture:

* **A bad sample never kills a subscriber.** :meth:`subscribe` wraps every
  callback: a ``ProtocolError`` or a disk hiccup is counted on the stream's
  :class:`~rec_sinks.StreamStats` and the next sample still lands. Zenoh calls
  these on its own threads and an escaping exception there is silent data loss.
* **The recorder is a pure observer.** Streams declare the keys they *subscribe*
  to and the keys they *query*; both lists go into ``meta.json``. Nothing here
  declares a publisher — see ``docs/recording.md`` for why the periscope in
  particular is recorded opportunistically rather than requested.
"""

from __future__ import annotations

import logging
from typing import List, Optional

from rec_clock import SessionClock
from rec_sinks import Budget, SessionWriter, StreamStats

log = logging.getLogger("vat-record")


class StreamRecorder:
    """Base class for one recorded stream."""

    #: short stream name, used for the manifest key and log lines
    name = "stream"

    #: Does this stream publish continuously? Dense streams define the window in
    #: which everything is available (``derived.aligned_window``). Sparse ones — the
    #: gated pose correction, which can legitimately go quiet for many seconds — must
    #: NOT shrink that window, so they are reported separately.
    dense = True

    def __init__(self, sw: SessionWriter, clock: SessionClock,
                 budget: Optional[Budget] = None):
        self.sw = sw
        self.clock = clock
        self.budget = budget or Budget(name=self.name)
        self.stats = StreamStats(self.name)
        self._z = None
        self._closed = False
        self.keys_subscribed: List[str] = []
        self.keys_queried: List[str] = []

    # ── lifecycle ────────────────────────────────────────────────────────────
    def attach(self, z) -> None:
        """Declare subscribers/queryable clients on an open Zenoh session."""
        self._z = z

    def tick(self, now_mono: float) -> None:
        """Called ~1 Hz from the main loop for periodic work (keyframes, pulls)."""

    def close(self) -> None:
        """Flush and finalise. Must be idempotent and must not raise."""
        self._closed = True

    # ── reporting ────────────────────────────────────────────────────────────
    def summary(self) -> dict:
        s = self.stats.summary()
        s["budget"] = self.budget.summary()
        s["keys_subscribed"] = list(self.keys_subscribed)
        s["keys_queried"] = list(self.keys_queried)
        s.update(self.extra_summary())
        return s

    def extra_summary(self) -> dict:
        """Stream-specific fields merged into :meth:`summary`."""
        return {}

    def status_line(self) -> str:
        """One short field for the console progress line."""
        return f"{self.name}={self.stats.n}"

    # ── helpers ──────────────────────────────────────────────────────────────
    def subscribe(self, key: str, handler) -> None:
        """Subscribe to ``key``, funnelling every error into the stream's stats.

        Zenoh invokes handlers on its own threads; an exception escaping there is
        swallowed by the runtime, so a decode bug would look like a dead stream.
        """
        def _wrapped(sample):
            if self._closed:
                # Zenoh subscribers stay live until the session is closed, after the
                # indexes are finalised. Count these instead of vanishing them.
                self.stats.skip()
                return
            try:
                handler(sample)
            except Exception as e:                        # noqa: BLE001 — never die
                self.stats.error(f"{type(e).__name__}: {e}")
                log.debug(f"[{self.name}] sample dropped: {e}", exc_info=True)

        self._z.declare_subscriber(key, _wrapped)
        self.keys_subscribed.append(key)
        log.info(f"[{self.name}] ← '{key}'")

    def note_query(self, key: str) -> None:
        """Record that this stream issues Zenoh queries against ``key``."""
        if key not in self.keys_queried:
            self.keys_queried.append(key)
