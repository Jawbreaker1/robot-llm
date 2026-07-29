"""Typed simulator observations and host-owned exploration candidates.

This module performs geometry and bookkeeping, not semantic interest
classification.  It publishes exact facts and a bounded feasible menu.  A
selector may rank that menu, while coordinates remain private to the host.
"""

from dataclasses import dataclass
from collections import deque
import math
import threading
from typing import Dict, Optional, Tuple

from .autonomy_contract import (
    EXPLORE_SPACE,
    FORWARD,
    INVESTIGATE_OBSERVATION,
    LEFT,
    RIGHT,
    ROBOT_BASE_FRAME,
    ExplorationCandidate,
    InterestObservation,
)
from .navigation_contract import (
    NavigationContractError,
    identifier,
    integer,
)
from .navigation_simulator import DifferentialDriveSimulator
from .navigation_state import NavigationSnapshot


@dataclass(frozen=True)
class ExplorationPolicy:
    """Deterministic feasibility envelope for simulator-only idle motion."""

    step_mm: int = 180
    minimum_travel_mm: int = 70
    clearance_reserve_mm: int = 180
    tolerance_mm: int = 30
    visit_grid_mm: int = 50
    max_attempted_visits_without_change: int = 2
    max_completed_visits_without_change: int = 1
    range_history_capacity: int = 256

    def __post_init__(self) -> None:
        integer("step_mm", self.step_mm, 1, 2_000)
        integer(
            "minimum_travel_mm",
            self.minimum_travel_mm,
            1,
            self.step_mm,
        )
        integer(
            "clearance_reserve_mm",
            self.clearance_reserve_mm,
            1,
            2_000,
        )
        integer("tolerance_mm", self.tolerance_mm, 1, 1_000)
        if self.tolerance_mm >= self.minimum_travel_mm:
            raise NavigationContractError(
                "invalid_exploration_tolerance",
                "Waypoint tolerance must be smaller than minimum travel",
            )
        integer("visit_grid_mm", self.visit_grid_mm, 1, 10_000)
        integer(
            "max_attempted_visits_without_change",
            self.max_attempted_visits_without_change,
            1,
            1_000,
        )
        integer(
            "max_completed_visits_without_change",
            self.max_completed_visits_without_change,
            1,
            1_000,
        )
        integer(
            "range_history_capacity",
            self.range_history_capacity,
            1,
            100_000,
        )


@dataclass(frozen=True)
class ResolvedExplorationCandidate:
    """Private host mapping from opaque candidate ID to one waypoint."""

    view: ExplorationCandidate
    target_x_mm: int
    target_y_mm: int
    tolerance_mm: int
    memory_key: Tuple[int, int]

    def __post_init__(self) -> None:
        if not isinstance(self.view, ExplorationCandidate):
            raise NavigationContractError(
                "invalid_candidate_view",
                "Resolved candidate requires ExplorationCandidate",
            )
        integer(
            "target_x_mm",
            self.target_x_mm,
            -1_000_000,
            1_000_000,
        )
        integer(
            "target_y_mm",
            self.target_y_mm,
            -1_000_000,
            1_000_000,
        )
        integer("tolerance_mm", self.tolerance_mm, 1, 10_000)
        if (
            not isinstance(self.memory_key, tuple)
            or len(self.memory_key) != 2
            or any(type(value) is not int for value in self.memory_key)
        ):
            raise NavigationContractError(
                "invalid_candidate_memory_key",
                "Candidate memory key is invalid",
            )


@dataclass(frozen=True)
class _RangeSample:
    forward_mm: int
    subject_id: Optional[str]


class RangeObservationTracker:
    """Compare exact forward-range samples only at the same stopped pose."""

    def __init__(self, capacity: int = 256):
        integer("capacity", capacity, 1, 100_000)
        self._capacity = capacity
        self._samples: Dict[Tuple[object, ...], _RangeSample] = {}
        self._order = deque()
        self._lock = threading.Lock()

    @staticmethod
    def _pose_key(snapshot: NavigationSnapshot) -> Tuple[object, ...]:
        return (
            snapshot.robot_id,
            snapshot.controller_instance_id,
            snapshot.pose.x_mm,
            snapshot.pose.y_mm,
            snapshot.pose.heading_mdeg,
        )

    def _store_locked(
        self,
        key: Tuple[object, ...],
        sample: _RangeSample,
    ) -> None:
        if key not in self._samples:
            if len(self._order) >= self._capacity:
                expired = self._order.popleft()
                self._samples.pop(expired, None)
            self._order.append(key)
        self._samples[key] = sample

    def seed(self, snapshot: NavigationSnapshot) -> None:
        """Remember a stopped metric sample without emitting an event."""

        if not isinstance(snapshot, NavigationSnapshot):
            raise NavigationContractError(
                "invalid_range_snapshot",
                "Range tracker requires NavigationSnapshot",
            )
        evidence = snapshot.clearance
        if (
            evidence.source != "simulation_metric"
            or evidence.forward_mm is None
        ):
            return
        sample = _RangeSample(
            forward_mm=evidence.forward_mm,
            subject_id=evidence.forward_object_id,
        )
        with self._lock:
            self._store_locked(self._pose_key(snapshot), sample)

    def capture(
        self,
        snapshot: NavigationSnapshot,
        observation_id: str,
        received_at_host_ms: int,
        valid_until_host_ms: int,
    ) -> Optional[InterestObservation]:
        """Emit one exact sample/transition; never label it interesting."""

        if not isinstance(snapshot, NavigationSnapshot):
            raise NavigationContractError(
                "invalid_range_snapshot",
                "Range tracker requires NavigationSnapshot",
            )
        identifier("observation_id", observation_id)
        integer(
            "received_at_host_ms",
            received_at_host_ms,
            0,
            2**63 - 2,
        )
        integer(
            "valid_until_host_ms",
            valid_until_host_ms,
            received_at_host_ms + 1,
            2**63 - 1,
        )
        evidence = snapshot.clearance
        if (
            evidence.source != "simulation_metric"
            or evidence.forward_mm is None
        ):
            return None
        key = self._pose_key(snapshot)
        current = _RangeSample(
            forward_mm=evidence.forward_mm,
            subject_id=evidence.forward_object_id,
        )
        with self._lock:
            previous = self._samples.get(key)
            self._store_locked(key, current)
        changed = (
            previous is not None
            and (
                previous.forward_mm != current.forward_mm
                or previous.subject_id != current.subject_id
            )
        )
        return InterestObservation(
            observation_id=observation_id,
            producer_id="simulated-forward-range",
            subject_robot_id=snapshot.robot_id,
            controller_instance_id=snapshot.controller_instance_id,
            frame_id=ROBOT_BASE_FRAME,
            modality="RANGE",
            kind=(
                "METRIC_SAMPLE"
                if previous is None
                else (
                    "METRIC_TRANSITION"
                    if changed
                    else "METRIC_UNCHANGED"
                )
            ),
            channel="FORWARD_CLEARANCE",
            observed_at_ms=evidence.observed_at_ms,
            received_at_host_ms=received_at_host_ms,
            valid_until_host_ms=valid_until_host_ms,
            state_version=snapshot.state_version,
            world_model_version=snapshot.world_model_version,
            confidence_milli=1_000,
            clock_domain="simulator_virtual",
            previous_value=(
                None if previous is None else previous.forward_mm
            ),
            current_value=current.forward_mm,
            unit="mm",
            previous_subject_id=(
                None if previous is None else previous.subject_id
            ),
            current_subject_id=current.subject_id,
        )

    @staticmethod
    def is_exact_transition(
        observation: InterestObservation,
    ) -> bool:
        """Return factual inequality, not a thresholded interest score."""

        if not isinstance(observation, InterestObservation):
            return False
        return (
            observation.previous_value is not None
            and (
                observation.previous_value
                != observation.current_value
                or observation.previous_subject_id
                != observation.current_subject_id
            )
        )


class ExplorationMemory:
    """Thread-safe completion/attempt memory keyed by quantized host cells."""

    def __init__(self, grid_mm: int = 50):
        integer("grid_mm", grid_mm, 1, 10_000)
        self.grid_mm = grid_mm
        self._attempts: Dict[Tuple[int, int], int] = {}
        self._completed: Dict[Tuple[int, int], int] = {}
        self._lock = threading.Lock()

    def key_for(self, x_mm: int, y_mm: int) -> Tuple[int, int]:
        integer("x_mm", x_mm, -1_000_000, 1_000_000)
        integer("y_mm", y_mm, -1_000_000, 1_000_000)
        half = self.grid_mm // 2
        return (
            math.floor((x_mm + half) / self.grid_mm),
            math.floor((y_mm + half) / self.grid_mm),
        )

    def counts(self, key: Tuple[int, int]) -> Tuple[int, int]:
        with self._lock:
            return (
                self._attempts.get(key, 0),
                self._completed.get(key, 0),
            )

    def record_attempt(self, key: Tuple[int, int]) -> None:
        with self._lock:
            self._attempts[key] = self._attempts.get(key, 0) + 1

    def record_completed(self, key: Tuple[int, int]) -> None:
        with self._lock:
            completed = self._completed.get(key, 0)
            if completed >= self._attempts.get(key, 0):
                raise NavigationContractError(
                    "completion_without_attempt",
                    "A cell cannot complete without an unmatched attempt",
                )
            self._completed[key] = completed + 1


class SimulatorCandidateGenerator:
    """Create a bounded local menu from trusted simulator geometry."""

    _DIRECTIONS = (
        (FORWARD, 0),
        (LEFT, 45_000),
        (RIGHT, -45_000),
    )

    def __init__(
        self,
        plant: DifferentialDriveSimulator,
        memory: ExplorationMemory,
        policy: ExplorationPolicy = ExplorationPolicy(),
    ):
        if not isinstance(plant, DifferentialDriveSimulator):
            raise NavigationContractError(
                "invalid_autonomy_plant",
                "Idle candidate generation is simulator-only",
            )
        if not isinstance(memory, ExplorationMemory):
            raise NavigationContractError(
                "invalid_exploration_memory",
                "Candidate generator requires ExplorationMemory",
            )
        if not isinstance(policy, ExplorationPolicy):
            raise NavigationContractError(
                "invalid_exploration_policy",
                "Exploration policy is invalid",
            )
        if (
            policy.clearance_reserve_mm
            < plant.settings.near_threshold_mm
        ):
            raise NavigationContractError(
                "unsafe_clearance_reserve",
                "Clearance reserve must cover the simulator near threshold",
            )
        self.plant = plant
        self.memory = memory
        self.policy = policy

    def _clearance_for(
        self,
        snapshot: NavigationSnapshot,
        direction: str,
    ) -> Optional[int]:
        evidence = snapshot.clearance
        if evidence.source != "simulation_metric":
            return None
        if direction == FORWARD:
            return evidence.forward_mm
        if direction == LEFT:
            return evidence.left_mm
        return evidence.right_mm

    def _target_is_safe(self, x_mm: int, y_mm: int) -> bool:
        radius = self.plant.settings.robot_radius_mm
        world = self.plant.world
        if (
            x_mm <= radius
            or y_mm <= radius
            or x_mm >= world.width_mm - radius
            or y_mm >= world.height_mm - radius
        ):
            return False
        for obstacle in world.obstacles:
            minimum = radius + obstacle.radius_mm
            if (
                (x_mm - obstacle.x_mm) ** 2
                + (y_mm - obstacle.y_mm) ** 2
                <= minimum**2
            ):
                return False
        return True

    def generate(
        self,
        snapshot: NavigationSnapshot,
        candidate_set_id: str,
        observation: Optional[InterestObservation],
    ) -> Tuple[ResolvedExplorationCandidate, ...]:
        if not isinstance(snapshot, NavigationSnapshot):
            raise NavigationContractError(
                "invalid_candidate_snapshot",
                "Candidate generation requires NavigationSnapshot",
            )
        identifier("candidate_set_id", candidate_set_id, 96)
        if observation is not None and not isinstance(
            observation,
            InterestObservation,
        ):
            raise NavigationContractError(
                "invalid_candidate_observation",
                "Candidate observation is invalid",
            )
        if observation is not None and (
            observation.subject_robot_id != snapshot.robot_id
            or observation.controller_instance_id
            != snapshot.controller_instance_id
            or observation.frame_id != ROBOT_BASE_FRAME
            or observation.state_version != snapshot.state_version
            or observation.world_model_version
            != snapshot.world_model_version
        ):
            raise NavigationContractError(
                "stale_candidate_observation",
                "Candidate observation does not match its snapshot",
            )
        if (
            snapshot.robot_id != self.plant.robot_id
            or snapshot.controller_instance_id
            != self.plant.controller_instance_id
            or snapshot.motors_running
            or snapshot.touch_pressed
            or snapshot.active_faults
        ):
            raise NavigationContractError(
                "unsafe_candidate_snapshot",
                "Candidates require a current safe stopped snapshot",
            )
        exact_transition = (
            observation is not None
            and RangeObservationTracker.is_exact_transition(observation)
        )
        linked_ids = (
            (observation.observation_id,)
            if exact_transition
            else ()
        )
        task_kind = (
            INVESTIGATE_OBSERVATION
            if linked_ids
            else EXPLORE_SPACE
        )
        candidates = []
        for candidate_index, (direction, offset_mdeg) in enumerate(
            self._DIRECTIONS,
            start=1,
        ):
            clearance = self._clearance_for(snapshot, direction)
            if clearance is None:
                continue
            travel_mm = min(
                self.policy.step_mm,
                clearance - self.policy.clearance_reserve_mm,
            )
            if travel_mm < self.policy.minimum_travel_mm:
                continue
            heading_rad = math.radians(
                (
                    snapshot.pose.heading_mdeg + offset_mdeg
                )
                / 1_000.0
            )
            target_x = int(round(
                snapshot.pose.x_mm + math.cos(heading_rad) * travel_mm
            ))
            target_y = int(round(
                snapshot.pose.y_mm + math.sin(heading_rad) * travel_mm
            ))
            if not self._target_is_safe(target_x, target_y):
                continue
            memory_key = self.memory.key_for(target_x, target_y)
            attempts, completed = self.memory.counts(memory_key)
            if (
                not linked_ids
                and (
                    attempts
                    >= self.policy.max_attempted_visits_without_change
                    or completed
                    >= self.policy.max_completed_visits_without_change
                )
            ):
                continue
            view = ExplorationCandidate(
                candidate_id="{}-c{}".format(
                    candidate_set_id,
                    candidate_index,
                ),
                task_kind=task_kind,
                relative_direction=direction,
                estimated_travel_mm=travel_mm,
                attempted_visits=attempts,
                completed_visits=completed,
                linked_observation_ids=linked_ids,
            )
            candidates.append(ResolvedExplorationCandidate(
                view=view,
                target_x_mm=target_x,
                target_y_mm=target_y,
                tolerance_mm=self.policy.tolerance_mm,
                memory_key=memory_key,
            ))
        return tuple(candidates)

    @staticmethod
    def resolve(
        candidates: Tuple[ResolvedExplorationCandidate, ...],
        candidate_id: str,
    ) -> ResolvedExplorationCandidate:
        identifier("candidate_id", candidate_id)
        matches = tuple(
            candidate
            for candidate in candidates
            if candidate.view.candidate_id == candidate_id
        )
        if len(matches) != 1:
            raise NavigationContractError(
                "unresolved_exploration_candidate",
                "Selected candidate is not in the host registry",
            )
        return matches[0]


__all__ = (
    "ExplorationMemory",
    "ExplorationPolicy",
    "RangeObservationTracker",
    "ResolvedExplorationCandidate",
    "SimulatorCandidateGenerator",
)
