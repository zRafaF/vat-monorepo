"""
VAT recorder — configuration, Zenoh keys & session provenance.
==============================================================
Mirrors the convention of ``server/mapping/mapping_config.py``: every tunable and
every Zenoh key the recorder touches is resolved *here*, once, from the same
environment ``vat.env`` exports — so the recorder cannot drift from the live
system's key schema.

Importing this module also puts the repo's ``common/`` on ``sys.path`` (like
``mapping_config`` does) so the recorder can ``import vat_protocol``.

Three jobs:

* **Keys** — ``KEYS`` comes from :func:`vat_protocol.keys`, never from hard-coded
  strings, plus the two keys the mapping server builds itself (reset / ceiling).
* **Provenance** — :func:`session_provenance` collects the *mapping* config as the
  server would resolve it (by importing ``mapping_config`` under the same env),
  hashes it, and records the git commit + ``vat.env`` digest. That triple is the
  "config hash" the publication roadmap §3.2 asks to log per session.
* **Parsing helpers** — ``'10GB'`` / ``'90s'`` / ``'5m'`` style CLI values.
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys

# ── repo layout ──────────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))                 # tools/recorder
REPO_ROOT = os.path.dirname(os.path.dirname(_HERE))                # repo/
COMMON_DIR = os.path.join(REPO_ROOT, "common")
MAPPING_DIR = os.path.join(REPO_ROOT, "server", "mapping")
if COMMON_DIR not in sys.path:
    sys.path.insert(0, COMMON_DIR)

import vat_protocol as proto  # noqa: E402  (needs the path insert above)

RECORDER_VERSION = "1.0.0"
SCHEMA = "vat.recording/1"

# ── Zenoh / identity ─────────────────────────────────────────────────────────
# ZENOH_ROUTER is what the server/client tools use; ZENOH_CONNECT is the name the
# robot-side processes use for the same endpoint. Accept either so `--where robot`
# works inside the robot container without extra flags.
ZENOH_ROUTER = os.environ.get(
    "ZENOH_ROUTER", os.environ.get("ZENOH_CONNECT", "tcp/127.0.0.1:7447"))
ROBOT_NAME = os.environ.get("ROBOT_NAME", "go2")
SERVER_PREFIX = os.environ.get("SERVER_PREFIX", "server/prism")

KEYS = proto.keys(ROBOT_NAME, SERVER_PREFIX)
# Built by mapping_config, not by proto.keys() — mirrored here so the recorder can
# log/observe them without importing the server module at runtime.
RESET_KEY = f"{SERVER_PREFIX}/cmd/reset"
CEILING_KEY = f"{SERVER_PREFIX}/config/ceiling_z"

# ── map-transport shape (must match the server, else the recording is unusable) ─
# The recorder mirrors the client's block store to (a) repair dropped pushes and
# (b) materialise free full-map keyframes, so it needs the same cube size and the
# same STREAM_MODE the server is publishing with.
STREAM_MODE = os.environ.get("STREAM_MODE", "blocks").strip().lower()
CUBE_SIZE = float(os.environ.get("CUBE_SIZE", "1.0"))

# Grace period after a manifest before diffing + pulling, matching
# client/vat_client/block_sync.py: the server publishes the push BEFORE the
# manifest, so waiting lets the push land and the repair loop find nothing to do.
PUSH_GRACE_S = float(os.environ.get("BLOCK_PUSH_GRACE_S", "0.25"))

# Robot→local clock-offset estimator window (see common/vat_telemetry.py).
CLOCK_WINDOW_S = float(os.environ.get("RECORDER_CLOCK_WINDOW_S", "15.0"))

# Default output root: <repo>/recordings/data/, resolved against the repo rather than the
# current directory so `make record` (which runs from client/, like every other tool)
# doesn't scatter a `client/recordings/` alongside it. Override with RECORDER_OUT_ROOT
# or --out — point it at a big disk for long full-res captures.
DEFAULT_OUT_ROOT = os.environ.get("RECORDER_OUT_ROOT") or os.path.join(
    REPO_ROOT, "recordings", "data")


# ═════════════════════════════════════════════════════════════════════════════
# Parsing helpers
# ═════════════════════════════════════════════════════════════════════════════

_SIZE_SUFFIXES = (
    ("TIB", 1024 ** 4), ("TB", 1024 ** 4), ("T", 1024 ** 4),
    ("GIB", 1024 ** 3), ("GB", 1024 ** 3), ("G", 1024 ** 3),
    ("MIB", 1024 ** 2), ("MB", 1024 ** 2), ("M", 1024 ** 2),
    ("KIB", 1024),      ("KB", 1024),      ("K", 1024),
    ("B", 1),
)


def parse_size(value) -> int:
    """``'10GB'`` / ``'500MB'`` / ``'10737418240'`` → bytes (binary multiples).

    Same semantics as ``robot/docker/frame_archive.parse_size``; duplicated rather
    than imported because that module pulls in cv2, which the recorder does not
    need. ``''`` / ``0`` / ``None`` → 0, meaning *uncapped*.
    """
    if value is None:
        return 0
    s = str(value).strip().upper().replace("_", "").replace(" ", "")
    if not s or s in ("0", "OFF", "NONE"):
        return 0
    for suf, mult in _SIZE_SUFFIXES:
        if s.endswith(suf):
            head = s[: -len(suf)]
            return int(float(head) * mult) if head else 0
    return int(float(s))


def parse_duration(value) -> float:
    """``'90'`` / ``'90s'`` / ``'5m'`` / ``'1h'`` / ``'2m30s'`` → seconds (float).

    ``''`` / ``0`` / ``None`` → 0.0, meaning *no time limit*.
    """
    if value is None:
        return 0.0
    s = str(value).strip().lower().replace(" ", "")
    if not s or s in ("0", "off", "none"):
        return 0.0
    try:                                          # bare number = seconds
        return float(s)
    except ValueError:
        pass
    total, num = 0.0, ""
    units = {"h": 3600.0, "m": 60.0, "s": 1.0}
    for ch in s:
        if ch.isdigit() or ch == ".":
            num += ch
        elif ch in units:
            total += float(num or 0) * units[ch]
            num = ""
        else:
            raise ValueError(f"cannot parse duration {value!r}")
    if num:                                        # trailing bare number = seconds
        total += float(num)
    return total


def human_size(n: int) -> str:
    n = float(n)
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if n < 1024.0 or unit == "TB":
            return f"{n:.1f}{unit}" if unit != "B" else f"{int(n)}B"
        n /= 1024.0
    return f"{n:.1f}TB"


# ═════════════════════════════════════════════════════════════════════════════
# Provenance:  mapping config hash + git commit + vat.env digest
# ═════════════════════════════════════════════════════════════════════════════


def _mapping_config_dict() -> dict:
    """Resolve the mapping server's tunables *as this process's environment would*.

    ``server/mapping/mapping_config.py`` is pure ``os.environ`` reads plus
    ``vat_protocol`` — no torch, no CUDA — so it imports cleanly in the client env.
    Launched through ``make`` (which does ``include vat.env; export``) this yields
    exactly the configuration the server is running with, which is what the
    publication roadmap wants hashed per session.

    Returns ``{}`` (and the caller records ``available: false``) if the import
    fails, so a recording never dies over provenance.
    """
    if MAPPING_DIR not in sys.path:
        sys.path.insert(0, MAPPING_DIR)
    try:
        import mapping_config as mcfg
    except Exception:
        return {}
    out = {}
    for name in dir(mcfg):
        if not name.isupper():
            continue
        val = getattr(mcfg, name)
        if isinstance(val, (str, int, float, bool)) or val is None:
            out[name] = val
    try:
        out["_SUMMARY"] = mcfg.summary()
    except Exception:
        pass
    return out


def _canonical_hash(obj) -> str:
    blob = json.dumps(obj, sort_keys=True, separators=(",", ":"),
                      default=str).encode()
    return hashlib.blake2b(blob, digest_size=16).hexdigest()


def _git(*args) -> str:
    try:
        return subprocess.run(("git",) + args, cwd=REPO_ROOT, timeout=5,
                              capture_output=True, text=True).stdout.strip()
    except Exception:
        return ""


def _file_sha256(path: str) -> str:
    try:
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 16), b""):
                h.update(chunk)
        return h.hexdigest()
    except OSError:
        return ""


def session_provenance() -> dict:
    """Everything needed to reproduce the run's configuration, for ``meta.json``.

    ``mapping_config_hash`` is the roadmap's "config hash"; the full resolved dict
    is included alongside it so the hash is auditable years later without needing
    this exact checkout.
    """
    mapping = _mapping_config_dict()
    vat_env = os.path.join(REPO_ROOT, "vat.env")
    return {
        "mapping_config_available": bool(mapping),
        "mapping_config_hash": _canonical_hash(mapping) if mapping else "",
        "mapping_config": mapping,
        "vat_env_path": vat_env if os.path.exists(vat_env) else "",
        "vat_env_sha256": _file_sha256(vat_env),
        "git": {
            "commit": _git("rev-parse", "HEAD"),
            "branch": _git("rev-parse", "--abbrev-ref", "HEAD"),
            "describe": _git("describe", "--always", "--dirty"),
            "dirty": bool(_git("status", "--porcelain")),
        },
        "recorder_version": RECORDER_VERSION,
        "python": sys.version.split()[0],
    }


def viewer_host(router: str = "") -> str:
    """An address a browser **on another machine** can reach this box on.

    Needed because the tools here are run on a headless server but looked at from a
    laptop: a URL containing ``localhost`` resolves on whichever machine the browser
    is running on, not on the server, so a page loads and then silently shows nothing.
    The router endpoint from ``vat.env`` is the one address every machine in the
    deployment already agrees on (a Tailscale IP in our setup), which makes it a far
    better default than ``localhost``. Falls back to the resolved hostname.
    """
    hostish = (router or ZENOH_ROUTER).split("/")[-1].split(":")[0]
    if hostish and hostish not in ("127.0.0.1", "0.0.0.0", "localhost", "::1"):
        return hostish
    import socket
    try:
        return socket.gethostbyname(socket.gethostname())
    except Exception:
        return "localhost"


def zenoh_summary(keys_subscribed, keys_queried) -> dict:
    """The transport block of ``meta.json``: what we listened to, verbatim."""
    return {
        "router": ZENOH_ROUTER,
        "robot_name": ROBOT_NAME,
        "server_prefix": SERVER_PREFIX,
        "stream_mode": STREAM_MODE,
        "cube_size_m": CUBE_SIZE,
        "keys_subscribed": sorted(set(keys_subscribed)),
        "keys_queried": sorted(set(keys_queried)),
    }


# ═════════════════════════════════════════════════════════════════════════════
# Self-test:  python tools/recorder/rec_config.py
# ═════════════════════════════════════════════════════════════════════════════

def _selftest() -> None:
    assert parse_size("10GB") == 10 * 1024 ** 3
    assert parse_size("500MB") == 500 * 1024 ** 2
    assert parse_size("1024") == 1024
    assert parse_size("") == 0 and parse_size(None) == 0 and parse_size("off") == 0
    assert parse_duration("90") == 90.0
    assert parse_duration("90s") == 90.0
    assert parse_duration("5m") == 300.0
    assert parse_duration("1h") == 3600.0
    assert parse_duration("2m30s") == 150.0
    assert parse_duration("") == 0.0 and parse_duration("off") == 0.0
    assert KEYS["camera_frame"] == f"{ROBOT_NAME}/prism/camera/frame"
    assert KEYS["pcd_push"] == f"{SERVER_PREFIX}/pcd/push"
    assert _canonical_hash({"a": 1, "b": 2}) == _canonical_hash({"b": 2, "a": 1})
    # The router host is what a browser on another machine can reach — 'localhost' is
    # exactly the value this helper exists to avoid handing out.
    assert viewer_host("tcp/100.76.214.80:7447") == "100.76.214.80"
    assert viewer_host("tcp/vat-server:7447") == "vat-server"
    lo = viewer_host("tcp/127.0.0.1:7447")     # loopback router → hostname fallback
    assert lo and ":" not in lo and "/" not in lo, lo
    assert viewer_host()
    p = session_provenance()
    assert "mapping_config_hash" in p and "git" in p
    print(f"rec_config self-test OK  (robot={ROBOT_NAME} server={SERVER_PREFIX} "
          f"stream={STREAM_MODE} cfg_hash={p['mapping_config_hash'] or 'n/a'})")


if __name__ == "__main__":
    _selftest()
