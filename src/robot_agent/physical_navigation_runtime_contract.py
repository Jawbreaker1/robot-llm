"""Configuration and terminal result values for physical navigation."""

from copy import deepcopy
from dataclasses import dataclass
import math
from typing import Mapping, Optional, Tuple

from .navigation_plan_tail import PLAN_TAIL_MAX_AGE_SECONDS
from .physical_navigation_scan_runtime import DEFAULT_SCAN_BUDGET


DEFAULT_MAX_TURNS = 14
MAX_TURNS_PER_EPISODE_SECOND = 4
HARD_MAX_TURNS = 14_400
DEFAULT_MAX_EPISODE_SECONDS = 35.0
MIN_EPISODE_SECONDS = 1.0
MAX_EPISODE_SECONDS = 60.0 * 60.0
SUPPORTED_EPISODE_LOCALES = frozenset(("sv", "en"))
DEFAULT_SCAN_TIMEOUT_SECONDS = (
    (DEFAULT_SCAN_BUDGET["minimum_deadline_ms"] + 999) // 1000
)
MAX_SCAN_TIMEOUT_SECONDS = 120.0


@dataclass(frozen=True)
class PhysicalNavigationRuntimeConfig:
    goal: str
    locale: str
    minimum_forward_progress_mm: int = 420
    goal_heading_tolerance_mdeg: int = 5_000
    max_turns: Optional[int] = None
    max_episode_seconds: float = DEFAULT_MAX_EPISODE_SECONDS
    startup_timeout_seconds: float = 30.0
    request_timeout_seconds: float = 8.0
    scan_timeout_seconds: float = float(DEFAULT_SCAN_TIMEOUT_SECONDS)
    plan_tail_max_age_seconds: float = PLAN_TAIL_MAX_AGE_SECONDS
    max_validation_attempts: int = 2

    def __post_init__(self) -> None:
        duration_is_valid = (
            not isinstance(self.max_episode_seconds, bool)
            and isinstance(self.max_episode_seconds, (int, float))
            and MIN_EPISODE_SECONDS
            <= float(self.max_episode_seconds)
            <= MAX_EPISODE_SECONDS
        )
        if self.max_turns is None and duration_is_valid:
            object.__setattr__(
                self,
                "max_turns",
                min(
                    HARD_MAX_TURNS,
                    max(
                        DEFAULT_MAX_TURNS,
                        int(
                            math.ceil(
                                float(self.max_episode_seconds)
                                * MAX_TURNS_PER_EPISODE_SECOND
                            )
                        ),
                    ),
                ),
            )
        if (
            not isinstance(self.goal, str)
            or not self.goal.strip()
            or len(self.goal) > 2_000
            or self.locale not in SUPPORTED_EPISODE_LOCALES
            or isinstance(self.minimum_forward_progress_mm, bool)
            or not isinstance(self.minimum_forward_progress_mm, int)
            or not 1 <= self.minimum_forward_progress_mm <= 2_000
            or isinstance(self.goal_heading_tolerance_mdeg, bool)
            or not isinstance(self.goal_heading_tolerance_mdeg, int)
            or not 1_000 <= self.goal_heading_tolerance_mdeg <= 45_000
            or isinstance(self.max_turns, bool)
            or not isinstance(self.max_turns, int)
            or not 1 <= self.max_turns <= HARD_MAX_TURNS
            or not duration_is_valid
            or isinstance(self.startup_timeout_seconds, bool)
            or not isinstance(self.startup_timeout_seconds, (int, float))
            or not 0.1 <= float(self.startup_timeout_seconds) <= 60.0
            or isinstance(self.request_timeout_seconds, bool)
            or not isinstance(self.request_timeout_seconds, (int, float))
            or not 0.1 <= float(self.request_timeout_seconds) <= 60.0
            or isinstance(self.scan_timeout_seconds, bool)
            or not isinstance(self.scan_timeout_seconds, (int, float))
            or not DEFAULT_SCAN_TIMEOUT_SECONDS
            <= float(self.scan_timeout_seconds)
            <= MAX_SCAN_TIMEOUT_SECONDS
            or isinstance(self.plan_tail_max_age_seconds, bool)
            or not isinstance(
                self.plan_tail_max_age_seconds,
                (int, float),
            )
            or not 1.0 <= float(self.plan_tail_max_age_seconds) <= 120.0
            or isinstance(self.max_validation_attempts, bool)
            or not isinstance(self.max_validation_attempts, int)
            or not 1 <= self.max_validation_attempts <= 3
        ):
            raise ValueError("physical navigation runtime config is invalid")


@dataclass(frozen=True)
class PhysicalNavigationResult:
    terminal_reason: str
    completed: bool
    turns: int
    actions: Tuple[str, ...]
    model_calls: int
    model_latency_ms: int
    plan_tails_completed: int
    plan_tails_cancelled: int
    final_mission: Mapping[str, object]
    final_navigation: Mapping[str, object]
    shutdown_clean: bool

    def to_dict(self) -> Mapping[str, object]:
        return {
            "terminal_reason": self.terminal_reason,
            "completed": self.completed,
            "turns": self.turns,
            "actions": list(self.actions),
            "model_calls": self.model_calls,
            "model_latency_ms": self.model_latency_ms,
            "plan_tails_completed": self.plan_tails_completed,
            "plan_tails_cancelled": self.plan_tails_cancelled,
            "final_mission": deepcopy(self.final_mission),
            "final_navigation": deepcopy(self.final_navigation),
            "shutdown_clean": self.shutdown_clean,
        }


__all__ = (
    "DEFAULT_MAX_EPISODE_SECONDS",
    "DEFAULT_MAX_TURNS",
    "DEFAULT_SCAN_TIMEOUT_SECONDS",
    "HARD_MAX_TURNS",
    "MAX_EPISODE_SECONDS",
    "MAX_SCAN_TIMEOUT_SECONDS",
    "MAX_TURNS_PER_EPISODE_SECOND",
    "MIN_EPISODE_SECONDS",
    "PhysicalNavigationResult",
    "PhysicalNavigationRuntimeConfig",
    "SUPPORTED_EPISODE_LOCALES",
)
