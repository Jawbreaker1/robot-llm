"""Absolute, checkpoint-bounded deadlines for one BLAST episode."""

from dataclasses import dataclass

from .blast_navigation_action_profile import (
    BLAST_NAVIGATION_ACTION_SPECS,
)
from .blast_observation_monitor import SCAN_COMMAND_TIMEOUT_SECONDS
from .physical_navigation_contract import SCAN_FRONT_ARC


DEADLINE_RESPONSE_HEADROOM_MS = 750
DEADLINE_SLICE_HEADROOM_MS = 250
SETTLED_OBSERVATION_HEADROOM_MS = 2_250


@dataclass(frozen=True)
class BlastEpisodeDeadline:
    """One absolute deadline measured by the adapter's monotonic clock."""

    deadline_ms: int | None
    monotonic_ms: object

    @classmethod
    def begin(cls, settings, monotonic_ms):
        duration_ms = getattr(settings, "max_episode_ms", None)
        if duration_ms is None:
            return cls(None, monotonic_ms)
        if (
            type(duration_ms) is not int
            or not 1_000 <= duration_ms <= 60 * 60 * 1_000
        ):
            raise ValueError("BLAST episode deadline is invalid")
        return cls(monotonic_ms() + duration_ms, monotonic_ms)

    def outcome(self, *, cancelled: bool, headroom_ms: int = 0):
        if cancelled:
            return "stopped", "stopped"
        if self.deadline_ms is None:
            return None
        now_ms = self.monotonic_ms()
        if now_ms >= self.deadline_ms:
            return (
                "episode_deadline_elapsed",
                "BLAST episode deadline elapsed",
            )
        if self.deadline_ms - now_ms < headroom_ms:
            return (
                "episode_deadline_headroom_insufficient",
                "BLAST episode has insufficient time for the next action",
            )
        return None


def blast_action_deadline_headroom_ms(action: str) -> int:
    """Reserve nominal action time plus the same host slack used by EV3."""

    if action == SCAN_FRONT_ARC:
        return int(round(SCAN_COMMAND_TIMEOUT_SECONDS * 1_000))
    spec = BLAST_NAVIGATION_ACTION_SPECS[action]
    return (
        spec["total_duration_ms"]
        + spec["slice_count"] * DEADLINE_SLICE_HEADROOM_MS
        + DEADLINE_RESPONSE_HEADROOM_MS
    )


__all__ = (
    "SETTLED_OBSERVATION_HEADROOM_MS",
    "BlastEpisodeDeadline",
    "blast_action_deadline_headroom_ms",
)
