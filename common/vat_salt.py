"""
VAT — SALT: place recognition for loop-closure-style anchoring (sensing layer)
==============================================================================
Cheap, dependency-light (NumPy only) building blocks for detecting when the robot
REVISITS a place it mapped earlier. This is step 1 of the "salt" strategy: before
we inject old anchor frames into the VGGT window to fix drift/alignment (and thus
carving), we first need to (a) recognise a revisit and (b) pick a covisible old
keyframe. Here we provide the descriptor + matcher; the server uses them read-only
to LOG revisits and the pose drift, validating the idea before we touch perception.

360° note: the Theta is equirectangular, so a revisit from a different HEADING is
just a horizontal (azimuth) shift of the panorama. The matcher is therefore
yaw-invariant: it compares descriptors over all circular column shifts and keeps
the best — so "same place, different facing" still matches.
"""

from __future__ import annotations

import numpy as np


def descriptor(img, rows: int = 16, cols: int = 64) -> np.ndarray:
    """Compact global descriptor of an equirectangular frame: block-averaged
    grayscale, mean-removed and L2-normalised → a (rows, cols) unit vector. Columns
    are azimuth (so a yaw change = a circular column shift; see :func:`similarity`)."""
    g = np.asarray(img, dtype=np.float32)
    if g.ndim == 3:
        g = g.mean(axis=2)
    H, W = g.shape
    h, w = (H // rows) * rows, (W // cols) * cols
    g = g[:h, :w].reshape(rows, h // rows, cols, w // cols).mean(axis=(1, 3))
    g = g - g.mean()
    n = float(np.linalg.norm(g))
    return (g / n if n > 1e-6 else g).astype(np.float32)


def similarity(a: np.ndarray, b: np.ndarray):
    """Yaw-invariant cosine similarity in [-1, 1]: max over all circular column
    (azimuth) shifts of b. Returns (best_sim, best_shift). a, b are unit (rows,cols)."""
    cols = a.shape[1]
    best, best_s = -1.0, 0
    for s in range(cols):
        v = float((a * np.roll(b, s, axis=1)).sum())
        if v > best:
            best, best_s = v, s
    return best, best_s


# =============================================================================
def _selftest() -> None:
    rng = np.random.default_rng(0)
    # a STRUCTURED panorama (floor/ceiling split + a couple of bright "features")
    base = np.zeros((518, 1036, 3), np.uint8)
    base[:260] = 70; base[260:] = 180
    base[:, 200:260] = 240
    base[100:300, 700:760] = 30
    d0 = descriptor(base)
    assert abs(np.linalg.norm(d0) - 1.0) < 1e-4

    # SAME place, different heading: yaw = horizontal roll (block-aligned 16*20 px),
    # plus sensor noise. Yaw-invariant matcher should still score high.
    block_w = 1036 // 64
    shifted = np.roll(base, block_w * 20, axis=1).astype(np.float32)
    shifted += rng.normal(0, 6, shifted.shape)
    s_same, sh = similarity(d0, descriptor(np.clip(shifted, 0, 255).astype(np.uint8)))
    assert s_same > 0.9, f"yaw-shifted revisit should match (got {s_same:.3f}, shift {sh})"

    # a DIFFERENT place should score clearly lower
    other = np.zeros((518, 1036, 3), np.uint8)
    other[:200] = 200; other[200:] = 40
    other[:, 500:540] = 255
    s_diff, _ = similarity(d0, descriptor(other))
    assert s_diff < s_same - 0.1, f"different place too similar (same={s_same:.3f} diff={s_diff:.3f})"
    print(f"vat_salt self-test OK  (revisit={s_same:.3f}  different={s_diff:.3f}  yaw-shift={sh})")


if __name__ == "__main__":
    _selftest()
