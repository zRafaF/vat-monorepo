"""
VAT mapping server — telemetry shim.

The implementation now lives in ``common/vat_telemetry.py`` so the client viewer
shares the exact same meters. mapping_config puts ``common/`` on sys.path, so this
re-export keeps ``from telemetry import ...`` working in the server unchanged.
"""

from vat_telemetry import ClockOffsetEstimator, ThroughputMeter  # noqa: F401
