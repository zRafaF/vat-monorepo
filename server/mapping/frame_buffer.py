"""
VAT mapping server — seq-keyed frame buffer.
============================================

A small, thread-safe store of received frames keyed by their monotonic ``seq``.
Split out of ``prism_session`` so the buffer bookkeeping (contiguity, gaps,
trimming) is testable on its own and the session module stays about *driving the
engine*.

The engine tracks windows by frame INDEX, so a frame's index within the
contiguous prefix must never shift between calls — hence the buffer always hands
the engine the gap-free prefix starting at the smallest buffered seq, and trims
only whole leading/trailing ranges (never the middle).
"""

from __future__ import annotations

import threading
from typing import Optional, Tuple


class FrameBuffer:
    """Thread-safe ``{seq: frame}`` buffer with contiguity + trim helpers."""

    def __init__(self) -> None:
        self._frames: dict[int, object] = {}
        self._last_processed_seq: int = -1
        self._lock = threading.Lock()

    # ── mutation ──────────────────────────────────────────────────────────────
    def clear(self) -> None:
        with self._lock:
            self._frames.clear()
            self._last_processed_seq = -1

    def add(self, seq: int, frame: object) -> bool:
        """Store ``frame`` under ``seq``; return True if this seq was new."""
        with self._lock:
            new = seq not in self._frames
            self._frames[seq] = frame
            return new

    def has(self, seq: int) -> bool:
        with self._lock:
            return seq in self._frames

    def mark_processed(self, max_seq: Optional[int]) -> None:
        if max_seq is None:
            return
        with self._lock:
            self._last_processed_seq = max_seq

    @property
    def last_processed_seq(self) -> int:
        with self._lock:
            return self._last_processed_seq

    # ── queries ───────────────────────────────────────────────────────────────
    def stats(self) -> Tuple[int, Optional[int], Optional[int], int]:
        """Return ``(total, lo_seq, hi_seq, new_since_processed)``."""
        with self._lock:
            if not self._frames:
                return 0, None, None, 0
            seqs = sorted(self._frames)
            new = sum(1 for s in seqs if s > self._last_processed_seq)
            return len(self._frames), seqs[0], seqs[-1], new

    def missing(self, lo: int, hi: int) -> list[int]:
        with self._lock:
            return [s for s in range(lo, hi + 1) if s not in self._frames]

    def contiguous_prefix(self):
        """Return ``(frames_list, lo, max_seq)`` for the gap-free prefix from the
        smallest buffered seq. A frame's list index is therefore stable across calls
        (the engine indexes windows positionally). Returns ``([], None, None)`` when
        empty. Also returns the first gap seq (or None) for logging by the caller."""
        with self._lock:
            if not self._frames:
                return [], None, None, None
            seqs = sorted(self._frames)
            lo = seqs[0]
            frames_list, s = [], lo
            while s in self._frames:
                frames_list.append(self._frames[s])
                s += 1
            max_seq = s - 1
            gap = s if s <= seqs[-1] else None
            return frames_list, lo, max_seq, gap

    def contiguous_from(self, start_seq: int):
        """Return ``(frames_list, max_seq, gap)`` for the gap-free run starting at
        ``start_seq`` (inclusive). Used by the hybrid ONLINE batches: they must always
        feed the engine frames from the SAME fixed base seq so a frame's list index
        never shifts between calls (the engine tracks online windows by index). Returns
        ``([], None, None)`` if ``start_seq`` isn't buffered."""
        with self._lock:
            if start_seq not in self._frames:
                return [], None, None
            frames_list, s = [], start_seq
            while s in self._frames:
                frames_list.append(self._frames[s])
                s += 1
            max_seq = s - 1
            seqs_hi = max(self._frames)
            gap = s if s <= seqs_hi else None
            return frames_list, max_seq, gap

    # ── trimming (bound memory / latency) ──────────────────────────────────────
    def trim_to_recent(self, keep_last_n: int) -> int:
        """Drop all but the newest ``keep_last_n`` seqs (reset mode only ever needs the
        recent window, so the buffer must not grow without bound). Resets the
        processed cursor so the retained frames re-enter a fresh reconstruction.
        Returns the number of frames dropped."""
        if keep_last_n <= 0:
            return 0
        with self._lock:
            if len(self._frames) <= keep_last_n:
                return 0
            seqs = sorted(self._frames)
            keep_from = seqs[-keep_last_n]
            dropped = [s for s in seqs if s < keep_from]
            for s in dropped:
                del self._frames[s]
            # A leading trim shifts frame indices, so the engine must rebuild — invalidate
            # the processed cursor (reset mode rebuilds every batch anyway).
            self._last_processed_seq = -1
            return len(dropped)

    def behind(self) -> int:
        """How many seqs the newest frame is ahead of the last processed one."""
        with self._lock:
            if not self._frames:
                return 0
            return max(self._frames) - self._last_processed_seq
