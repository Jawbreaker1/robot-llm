"""Deterministic interpretation and bounded language for IR observations."""

from collections import deque
from dataclasses import dataclass
from statistics import median
from typing import Iterable, Mapping, Optional


FALLBACK_PHRASES = {
    "strong_return": "Jag får en väldigt stark träff framför mig.",
    "near_return": "Jag märker något framför mig.",
    "mid_return": "Det verkar finnas något längre fram.",
    "far_or_no_clear_return": "Jag får ingen tydlig närträff framför mig.",
}


@dataclass(frozen=True)
class ProximityThresholds:
    strong_return_max: int
    near_return_max: int
    mid_return_max: int

    def __post_init__(self) -> None:
        values = (
            self.strong_return_max,
            self.near_return_max,
            self.mid_return_max,
        )
        if any(isinstance(value, bool) or not isinstance(value, int) for value in values):
            raise ValueError("Proximity zone thresholds must be integers")
        if not (
            0
            <= self.strong_return_max
            < self.near_return_max
            < self.mid_return_max
            <= 100
        ):
            raise ValueError("Proximity zone thresholds must be increasing")

    @classmethod
    def from_config(cls, config: Mapping[str, object]) -> "ProximityThresholds":
        calibration = config["calibration"]["infrared_proximity"]  # type: ignore[index]
        zones = calibration["zones"]  # type: ignore[index]
        return cls(**zones)  # type: ignore[arg-type]


@dataclass(frozen=True)
class ObstacleGatePolicy:
    immediate_enter_max: int
    enter_max: int
    exit_min: int
    median_window: int
    enter_consecutive: int
    exit_consecutive: int

    def __post_init__(self) -> None:
        values = (
            self.immediate_enter_max,
            self.enter_max,
            self.exit_min,
            self.median_window,
            self.enter_consecutive,
            self.exit_consecutive,
        )
        if any(
            isinstance(value, bool) or not isinstance(value, int)
            for value in values
        ):
            raise ValueError("Obstacle gate settings must be integers")
        if not 0 <= self.immediate_enter_max <= self.enter_max < self.exit_min <= 100:
            raise ValueError("Obstacle gate thresholds are invalid")
        if self.median_window <= 0 or self.median_window % 2 == 0:
            raise ValueError("median_window must be a positive odd integer")
        if self.enter_consecutive <= 0 or self.exit_consecutive <= 0:
            raise ValueError("Consecutive decision counts must be positive")

    @classmethod
    def from_config(cls, config: Mapping[str, object]) -> "ObstacleGatePolicy":
        calibration = config["calibration"]["infrared_proximity"]  # type: ignore[index]
        policy = calibration["obstacle_gate"]  # type: ignore[index]
        return cls(**policy)  # type: ignore[arg-type]


DEFAULT_PROXIMITY_THRESHOLDS = ProximityThresholds(
    strong_return_max=16,
    near_return_max=35,
    mid_return_max=47,
)

DEFAULT_OBSTACLE_GATE_POLICY = ObstacleGatePolicy(
    immediate_enter_max=16,
    enter_max=35,
    exit_min=40,
    median_window=3,
    enter_consecutive=2,
    exit_consecutive=3,
)


@dataclass(frozen=True)
class ProximityObservation:
    observed_at_ms: int
    samples: tuple[int, ...]
    filtered_percent: int
    zone: str


def classify_infrared(
    samples: Iterable[int],
    observed_at_ms: int,
    thresholds: ProximityThresholds = DEFAULT_PROXIMITY_THRESHOLDS,
) -> ProximityObservation:
    """Classify reflection/proximity evidence, never distance or clearance."""
    values = tuple(samples)
    if not values:
        raise ValueError("At least one infrared sample is required")
    for value in values:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("Infrared samples must be integers")
        if value < 0 or value > 100:
            raise ValueError("Infrared samples must be in 0..100")

    filtered = int(round(median(values)))
    if filtered <= thresholds.strong_return_max:
        zone = "strong_return"
    elif filtered <= thresholds.near_return_max:
        zone = "near_return"
    elif filtered <= thresholds.mid_return_max:
        zone = "mid_return"
    else:
        zone = "far_or_no_clear_return"

    return ProximityObservation(
        observed_at_ms=observed_at_ms,
        samples=values,
        filtered_percent=filtered,
        zone=zone,
    )


def fallback_comment(observation: ProximityObservation) -> str:
    return FALLBACK_PHRASES[observation.zone]


def validate_generated_comment(
    text: str, max_characters: int = 80, max_words: int = 14
) -> str:
    """Validate model output as speech data, never as an action."""
    if not isinstance(text, str):
        raise ValueError("Comment must be a string")
    if "\x00" in text or "\n" in text or "\r" in text:
        raise ValueError("Comment must be one line")
    normalized = " ".join(text.split())
    if not normalized:
        raise ValueError("Comment must not be empty")
    if len(normalized) > max_characters:
        raise ValueError("Comment is too long")
    if len(normalized.split()) > max_words:
        raise ValueError("Comment contains too many words")
    return normalized


class StableZoneTracker:
    """Emit only stable zone transitions to prevent repetitive chatter."""

    def __init__(self, required_consecutive: int = 3):
        if required_consecutive <= 0:
            raise ValueError("required_consecutive must be positive")
        self.required_consecutive = required_consecutive
        self.current_zone: Optional[str] = None
        self._candidate_zone: Optional[str] = None
        self._candidate_count = 0

    def observe(self, zone: str) -> Optional[str]:
        if zone not in FALLBACK_PHRASES:
            raise ValueError("Unknown proximity zone")
        if zone == self.current_zone:
            self._candidate_zone = None
            self._candidate_count = 0
            return None

        if zone == self._candidate_zone:
            self._candidate_count += 1
        else:
            self._candidate_zone = zone
            self._candidate_count = 1

        if self._candidate_count < self.required_consecutive:
            return None

        self.current_zone = zone
        self._candidate_zone = None
        self._candidate_count = 0
        return zone


class ObstacleEvidenceGate:
    """Conservative IR evidence gate; never a sole collision safeguard.

    ``state`` is ``True`` for near-obstacle evidence, ``False`` when the gate
    has been released by stable higher readings, and ``None`` while startup
    or deadband evidence is unresolved. A released gate does not prove a
    globally clear path. Entry is intentionally faster than release.
    """

    def __init__(
        self,
        policy: ObstacleGatePolicy = DEFAULT_OBSTACLE_GATE_POLICY,
    ):
        self.policy = policy
        self.state: Optional[bool] = None
        self.filtered_percent: Optional[int] = None
        self._samples = deque(maxlen=policy.median_window)
        self._candidate_state: Optional[bool] = None
        self._candidate_count = 0

    @property
    def motion_allowed(self) -> bool:
        return self.state is False

    @property
    def stop_required(self) -> bool:
        return not self.motion_allowed

    def _reset_candidate(self) -> None:
        self._candidate_state = None
        self._candidate_count = 0

    def observe(self, raw_percent: int) -> Optional[bool]:
        if isinstance(raw_percent, bool) or not isinstance(raw_percent, int):
            raise ValueError("Infrared sample must be an integer")
        if raw_percent < 0 or raw_percent > 100:
            raise ValueError("Infrared sample must be in 0..100")

        self._samples.append(raw_percent)
        self.filtered_percent = int(round(median(self._samples)))

        # A single very strong return may stop immediately. A false positive
        # only stops motion; it cannot initiate motion.
        if raw_percent <= self.policy.immediate_enter_max:
            self.state = True
            self._reset_candidate()
            return self.state

        if len(self._samples) < self.policy.median_window:
            return self.state

        if self.filtered_percent <= self.policy.enter_max:
            desired_state = True
            required = self.policy.enter_consecutive
        elif self.filtered_percent >= self.policy.exit_min:
            desired_state = False
            required = self.policy.exit_consecutive
        else:
            self._reset_candidate()
            return self.state

        if desired_state == self.state:
            self._reset_candidate()
            return self.state

        if desired_state == self._candidate_state:
            self._candidate_count += 1
        else:
            self._candidate_state = desired_state
            self._candidate_count = 1

        if self._candidate_count >= required:
            self.state = desired_state
            self._reset_candidate()
        return self.state
