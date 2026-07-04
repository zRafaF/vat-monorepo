"""VAT Remote Periscope (robot side) — see docs/periscope.md.

A small package (kept multi-file on purpose) that renders a directable
high-quality slice from the robot's live 360° frame and streams it to the
operator, controlled by the client's view requests.
"""

from . import config
from .service import PeriscopeService

__all__ = ["PeriscopeService", "config"]
