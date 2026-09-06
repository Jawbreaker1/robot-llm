"""Local diagnostic records; never input to navigation decisions."""

import json
import logging
from pathlib import Path
import time


LOGGER = logging.getLogger("robot_agent.navigation_diagnostics")


def enable_navigation_diagnostics(path: Path) -> None:
    """Configure once at the CLI boundary, not inside the robot runtime."""

    if LOGGER.handlers:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    handler = logging.FileHandler(path, encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(message)s"))
    LOGGER.addHandler(handler)
    LOGGER.setLevel(logging.INFO)
    LOGGER.propagate = False


def record_navigation_diagnostic(event: str, **values) -> None:
    if LOGGER.isEnabledFor(logging.INFO):
        LOGGER.info("%s", json.dumps({
            "event": event,
            "recorded_at_unix_ms": time.time_ns() // 1_000_000,
            **values,
        }, ensure_ascii=False, default=str))
