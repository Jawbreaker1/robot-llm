"""Versioned navigation state and a bounded multi-producer proposal inbox."""

from dataclasses import dataclass
from collections import deque
import threading
from typing import Callable, Dict, Optional, Tuple

from .navigation_contract import (
    NavigationContractError,
    PlannerProposal,
    StampedProposal,
    WaypointGoal,
    boolean,
    decode_navigation_proposal,
    identifier,
    integer,
)


@dataclass(frozen=True)
class PoseEstimate:
    x_mm: int
    y_mm: int
    heading_mdeg: int

    def __post_init__(self) -> None:
        integer("x_mm", self.x_mm, -1_000_000, 1_000_000)
        integer("y_mm", self.y_mm, -1_000_000, 1_000_000)
        integer("heading_mdeg", self.heading_mdeg, -180_000, 179_999)


@dataclass(frozen=True)
class ClearanceEvidence:
    """Positive metric clearance exists only in the simulator for now.

    ``physical_ir_reflection`` may describe near-obstacle evidence, but it
    cannot carry a metric clearance and can never prove that forward motion
    is safe.
    """

    source: str
    observed_at_ms: int
    near_obstacle_latched: bool
    forward_mm: Optional[int] = None
    left_mm: Optional[int] = None
    right_mm: Optional[int] = None
    raw_ir_proximity: Optional[int] = None
    forward_object_id: Optional[str] = None

    def __post_init__(self) -> None:
        if self.source not in (
            "simulation_metric",
            "physical_ir_reflection",
            "none",
        ):
            raise NavigationContractError(
                "invalid_clearance_source",
                "Clearance source is invalid",
            )
        integer("observed_at_ms", self.observed_at_ms, 0, 2**63 - 1)
        boolean("near_obstacle_latched", self.near_obstacle_latched)
        for name, value in (
            ("forward_mm", self.forward_mm),
            ("left_mm", self.left_mm),
            ("right_mm", self.right_mm),
        ):
            if value is not None:
                integer(name, value, 0, 1_000_000)
        if self.raw_ir_proximity is not None:
            integer(
                "raw_ir_proximity",
                self.raw_ir_proximity,
                0,
                100,
            )
        if self.forward_object_id is not None:
            identifier("forward_object_id", self.forward_object_id)
            if self.source != "simulation_metric":
                raise NavigationContractError(
                    "untrusted_forward_object_identity",
                    "Only simulation evidence may identify an object",
                )
        if self.source == "simulation_metric":
            if self.forward_mm is None:
                raise NavigationContractError(
                    "missing_metric_clearance",
                    "Simulation evidence requires forward_mm",
                )
            if self.raw_ir_proximity is not None:
                raise NavigationContractError(
                    "mixed_clearance_units",
                    "Simulation metric evidence cannot contain EV3 IR",
                )
        elif any(
            value is not None
            for value in (self.forward_mm, self.left_mm, self.right_mm)
        ):
            raise NavigationContractError(
                "untrusted_metric_clearance",
                "Only simulation evidence may carry millimetres",
            )

    @property
    def positively_cleared_for_simulation(self) -> bool:
        return (
            self.source == "simulation_metric"
            and self.forward_mm is not None
            and not self.near_obstacle_latched
        )


@dataclass(frozen=True)
class NavigationSnapshot:
    robot_id: str
    controller_instance_id: str
    goal_id: str
    goal_epoch: int
    plan_revision: int
    state_version: int
    world_model_version: int
    captured_at_host_ms: int
    state_observed_at_ms: int
    pose: PoseEstimate
    left_encoder_mdeg: int
    right_encoder_mdeg: int
    motors_running: bool
    touch_pressed: bool
    active_faults: Tuple[str, ...]
    clearance: ClearanceEvidence

    def __post_init__(self) -> None:
        identifier("robot_id", self.robot_id)
        identifier("controller_instance_id", self.controller_instance_id)
        identifier("goal_id", self.goal_id)
        integer("goal_epoch", self.goal_epoch, 1, 2**63 - 1)
        integer("plan_revision", self.plan_revision, 1, 2**63 - 1)
        integer("state_version", self.state_version, 1, 2**63 - 1)
        integer(
            "world_model_version",
            self.world_model_version,
            1,
            2**63 - 1,
        )
        integer(
            "captured_at_host_ms",
            self.captured_at_host_ms,
            0,
            2**63 - 1,
        )
        integer(
            "state_observed_at_ms",
            self.state_observed_at_ms,
            0,
            2**63 - 1,
        )
        if self.captured_at_host_ms < self.state_observed_at_ms:
            raise NavigationContractError(
                "future_state_observation",
                "State observation cannot be newer than host capture",
            )
        if not isinstance(self.pose, PoseEstimate):
            raise NavigationContractError(
                "invalid_pose",
                "Navigation snapshot requires PoseEstimate",
            )
        integer(
            "left_encoder_mdeg",
            self.left_encoder_mdeg,
            -(2**63),
            2**63 - 1,
        )
        integer(
            "right_encoder_mdeg",
            self.right_encoder_mdeg,
            -(2**63),
            2**63 - 1,
        )
        boolean("motors_running", self.motors_running)
        boolean("touch_pressed", self.touch_pressed)
        if (
            not isinstance(self.active_faults, tuple)
            or any(
                identifier("active_fault", fault, 96) != fault
                for fault in self.active_faults
            )
            or len(set(self.active_faults)) != len(self.active_faults)
        ):
            raise NavigationContractError(
                "invalid_faults",
                "active_faults must be a unique tuple",
            )
        if not isinstance(self.clearance, ClearanceEvidence):
            raise NavigationContractError(
                "invalid_clearance",
                "Navigation snapshot requires ClearanceEvidence",
            )
        if self.clearance.observed_at_ms > self.captured_at_host_ms:
            raise NavigationContractError(
                "future_safety_observation",
                "Safety evidence cannot be newer than host capture",
            )

    def bound_to(self, goal: WaypointGoal) -> bool:
        return (
            self.goal_id == goal.goal_id
            and self.goal_epoch == goal.goal_epoch
            and self.plan_revision == goal.plan_revision
        )


class StateReducer:
    """Single logical writer for immutable, monotonically versioned state."""

    def __init__(self, initial: NavigationSnapshot):
        if not isinstance(initial, NavigationSnapshot):
            raise NavigationContractError(
                "invalid_initial_snapshot",
                "StateReducer requires NavigationSnapshot",
            )
        self._snapshot = initial
        self._lock = threading.Lock()

    def snapshot(self) -> NavigationSnapshot:
        with self._lock:
            return self._snapshot

    def commit(self, value: NavigationSnapshot) -> None:
        if not isinstance(value, NavigationSnapshot):
            raise NavigationContractError(
                "invalid_snapshot",
                "StateReducer can only commit NavigationSnapshot",
            )
        with self._lock:
            previous = self._snapshot
            if (
                value.robot_id != previous.robot_id
                or value.controller_instance_id
                != previous.controller_instance_id
            ):
                raise NavigationContractError(
                    "controller_identity_changed",
                    "State controller identity changed",
                )
            if (
                value.goal_epoch < previous.goal_epoch
                or value.state_version <= previous.state_version
                or value.world_model_version
                < previous.world_model_version
                or value.captured_at_host_ms
                < previous.captured_at_host_ms
                or value.state_observed_at_ms
                < previous.state_observed_at_ms
            ):
                raise NavigationContractError(
                    "non_monotonic_snapshot",
                    "Navigation snapshot did not advance monotonically",
                )
            if (
                value.goal_epoch == previous.goal_epoch
                and (
                    value.goal_id != previous.goal_id
                    or value.plan_revision < previous.plan_revision
                )
            ):
                raise NavigationContractError(
                    "invalid_goal_transition",
                    "Goal changed without a new epoch",
                )
            self._snapshot = value


@dataclass(frozen=True)
class ProposalSourcePolicy:
    source_id: str
    authority_rank: int
    priority: int
    ttl_ms: int

    def __post_init__(self) -> None:
        identifier("source_id", self.source_id)
        integer("authority_rank", self.authority_rank, 0, 10_000)
        integer("priority", self.priority, 0, 10_000)
        integer("ttl_ms", self.ttl_ms, 1, 10_000)


class ProposalInbox:
    """Thread-safe, bounded inbox with host-owned authority and TTL."""

    def __init__(
        self,
        policies: Tuple[ProposalSourcePolicy, ...],
        clock_ms: Callable[[], int],
        capacity: int = 64,
        replay_window: int = 4_096,
    ):
        if (
            not isinstance(policies, tuple)
            or not policies
            or not callable(clock_ms)
        ):
            raise NavigationContractError(
                "invalid_inbox_configuration",
                "Proposal inbox configuration is invalid",
            )
        integer("capacity", capacity, 1, 10_000)
        integer(
            "replay_window",
            replay_window,
            capacity,
            100_000,
        )
        by_source = {}
        for policy in policies:
            if not isinstance(policy, ProposalSourcePolicy):
                raise NavigationContractError(
                    "invalid_source_policy",
                    "Inbox policies must be ProposalSourcePolicy",
                )
            if policy.source_id in by_source:
                raise NavigationContractError(
                    "duplicate_source_policy",
                    "Proposal source policy is duplicated",
                )
            by_source[policy.source_id] = policy
        self._policies: Dict[str, ProposalSourcePolicy] = by_source
        self._clock_ms = clock_ms
        self._capacity = capacity
        self._replay_window = replay_window
        self._queue = []
        self._proposal_ids = set()
        self._proposal_id_order = deque()
        self._source_high_water: Dict[str, int] = {}
        self._lock = threading.Lock()

    def _now_ms(self) -> int:
        return integer("clock_ms", self._clock_ms(), 0, 2**63 - 1)

    def publish_raw(
        self,
        raw: bytes,
        source_id: str,
        source_sequence: int,
    ) -> StampedProposal:
        return self.publish(
            decode_navigation_proposal(raw),
            source_id,
            source_sequence,
        )

    def publish(
        self,
        proposal: PlannerProposal,
        source_id: str,
        source_sequence: int,
    ) -> StampedProposal:
        if not isinstance(proposal, PlannerProposal):
            raise NavigationContractError(
                "invalid_proposal",
                "Inbox requires PlannerProposal",
            )
        identifier("source_id", source_id)
        integer("source_sequence", source_sequence, 1, 2**63 - 1)
        now_ms = self._now_ms()
        with self._lock:
            return self._publish_locked(
                proposal,
                source_id,
                source_sequence,
                now_ms,
            )

    def publish_host(
        self,
        proposal: PlannerProposal,
        source_id: str,
    ) -> StampedProposal:
        """Atomically allocate a host sequence for a local producer."""

        if not isinstance(proposal, PlannerProposal):
            raise NavigationContractError(
                "invalid_proposal",
                "Inbox requires PlannerProposal",
            )
        identifier("source_id", source_id)
        now_ms = self._now_ms()
        with self._lock:
            source_sequence = (
                self._source_high_water.get(source_id, 0) + 1
            )
            return self._publish_locked(
                proposal,
                source_id,
                source_sequence,
                now_ms,
            )

    def _publish_locked(
        self,
        proposal: PlannerProposal,
        source_id: str,
        source_sequence: int,
        now_ms: int,
    ) -> StampedProposal:
        try:
            policy = self._policies[source_id]
        except KeyError:
            raise NavigationContractError(
                "unknown_proposal_source",
                "Proposal source is not allowlisted",
            ) from None
        if proposal.proposal_id in self._proposal_ids:
            raise NavigationContractError(
                "duplicate_proposal_id",
                "Proposal ID has already been published",
            )
        if source_sequence <= self._source_high_water.get(source_id, 0):
            raise NavigationContractError(
                "replayed_source_sequence",
                "Source sequence did not advance",
            )
        if len(self._queue) >= self._capacity:
            raise NavigationContractError(
                "proposal_inbox_full",
                "Proposal inbox capacity was exhausted",
            )
        stamped = StampedProposal(
            proposal=proposal,
            source_id=source_id,
            source_sequence=source_sequence,
            received_at_ms=now_ms,
            valid_until_ms=now_ms + policy.ttl_ms,
            authority_rank=policy.authority_rank,
            priority=policy.priority,
        )
        self._queue.append(stamped)
        if len(self._proposal_id_order) >= self._replay_window:
            expired_id = self._proposal_id_order.popleft()
            self._proposal_ids.discard(expired_id)
        self._proposal_ids.add(proposal.proposal_id)
        self._proposal_id_order.append(proposal.proposal_id)
        self._source_high_water[source_id] = source_sequence
        return stamped

    def drain(self) -> Tuple[StampedProposal, ...]:
        """Consume every queued proposal, including eventual non-winners."""

        with self._lock:
            values = tuple(self._queue)
            self._queue.clear()
            return values

    def __len__(self) -> int:
        with self._lock:
            return len(self._queue)
