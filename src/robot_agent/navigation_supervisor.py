"""Single-owner motion arbitration for short navigation pulses."""

from dataclasses import dataclass
from collections import deque
import math
import secrets
from typing import Callable, Optional, Tuple

from .navigation_contract import (
    AdvanceSegment,
    DriveCalibrationProfile,
    DrivePulse,
    MotionAuthority,
    NavigationContractError,
    StampedProposal,
    TurnSegment,
    WaypointGoal,
    identifier,
    integer,
)
from .navigation_state import NavigationSnapshot


@dataclass(frozen=True)
class MotionPolicy:
    max_snapshot_age_ms: int = 200
    max_safety_age_ms: int = 100
    max_proposal_ttl_ms: int = 500
    max_pulse_ms: int = 120
    max_linear_speed_mm_s: int = 120
    max_angular_speed_mdeg_s: int = 90_000
    forward_reserve_mm: int = 70
    rearm_safe_samples: int = 2
    proposal_replay_window: int = 4_096

    def __post_init__(self) -> None:
        integer(
            "max_snapshot_age_ms",
            self.max_snapshot_age_ms,
            1,
            60_000,
        )
        integer(
            "max_safety_age_ms",
            self.max_safety_age_ms,
            1,
            self.max_snapshot_age_ms,
        )
        integer(
            "max_proposal_ttl_ms",
            self.max_proposal_ttl_ms,
            1,
            10_000,
        )
        integer("max_pulse_ms", self.max_pulse_ms, 1, 1_000)
        integer(
            "max_linear_speed_mm_s",
            self.max_linear_speed_mm_s,
            1,
            2_000,
        )
        integer(
            "max_angular_speed_mdeg_s",
            self.max_angular_speed_mdeg_s,
            1,
            720_000,
        )
        integer(
            "forward_reserve_mm",
            self.forward_reserve_mm,
            1,
            10_000,
        )
        integer("rearm_safe_samples", self.rearm_safe_samples, 2, 10)
        integer(
            "proposal_replay_window",
            self.proposal_replay_window,
            1,
            100_000,
        )


class MotionSupervisor:
    """The only component allowed to convert intent into wheel commands."""

    def __init__(
        self,
        profile: DriveCalibrationProfile,
        clock_ms: Callable[[], int],
        robot_id: str,
        controller_instance_id: str,
        motion_authority: MotionAuthority,
        policy: MotionPolicy = MotionPolicy(),
        arbiter_id: str = "navigation-motion-supervisor",
        id_factory: Callable[[], str] = lambda: secrets.token_hex(8),
    ):
        if not isinstance(profile, DriveCalibrationProfile):
            raise NavigationContractError(
                "invalid_calibration",
                "Motion supervisor requires DriveCalibrationProfile",
            )
        # There is deliberately no physical backend in this slice.  Even a
        # future verified profile must pass a separate protocol gate.
        if profile.status != "simulation_only":
            raise NavigationContractError(
                "physical_navigation_disabled",
                "Only simulation_only navigation is implemented",
            )
        profile.require_complete_geometry()
        if not callable(clock_ms) or not callable(id_factory):
            raise NavigationContractError(
                "invalid_dependency",
                "Clock and ID factory must be callable",
            )
        if not isinstance(motion_authority, MotionAuthority):
            raise NavigationContractError(
                "missing_motion_authority",
                "Motion supervisor requires MotionAuthority",
            )
        identifier("robot_id", robot_id)
        identifier("controller_instance_id", controller_instance_id)
        identifier("arbiter_id", arbiter_id)
        if not isinstance(policy, MotionPolicy):
            raise NavigationContractError(
                "invalid_motion_policy",
                "Motion policy is invalid",
            )
        if policy.max_pulse_ms > profile.max_pulse_ms:
            raise NavigationContractError(
                "policy_exceeds_calibration",
                "Motion policy pulse exceeds calibration",
            )
        self.profile = profile
        self.policy = policy
        self.robot_id = robot_id
        self.controller_instance_id = controller_instance_id
        self.arbiter_id = arbiter_id
        self._motion_authority = motion_authority
        self._clock_ms = clock_ms
        self._id_factory = id_factory
        self._seen_proposal_ids = set()
        self._seen_proposal_order = deque()
        self._source_high_water = {}
        self._active_goal_epoch: Optional[int] = None
        self._emergency_latched = False
        self._latched_goal_epoch: Optional[int] = None

    @property
    def emergency_latched(self) -> bool:
        return self._emergency_latched

    def _now_ms(self) -> int:
        return integer("clock_ms", self._clock_ms(), 0, 2**63 - 1)

    def _decision_id(self) -> str:
        return identifier("decision_id", self._id_factory(), 128)

    def _authorize_pulse(self, pulse: DrivePulse) -> DrivePulse:
        self._motion_authority.authorize(pulse)
        return pulse

    def _remember_proposal_id(self, proposal_id: str) -> None:
        if (
            len(self._seen_proposal_order)
            >= self.policy.proposal_replay_window
        ):
            expired_id = self._seen_proposal_order.popleft()
            self._seen_proposal_ids.discard(expired_id)
        self._seen_proposal_ids.add(proposal_id)
        self._seen_proposal_order.append(proposal_id)

    def _stop(
        self,
        snapshot: NavigationSnapshot,
        reason_code: str,
        proposal_id: Optional[str] = None,
    ) -> DrivePulse:
        return self._authorize_pulse(DrivePulse(
            decision_id=self._decision_id(),
            arbiter_id=self.arbiter_id,
            robot_id=snapshot.robot_id,
            controller_instance_id=snapshot.controller_instance_id,
            goal_id=snapshot.goal_id,
            goal_epoch=snapshot.goal_epoch,
            plan_revision=snapshot.plan_revision,
            based_on_state_version=snapshot.state_version,
            based_on_world_model_version=(
                snapshot.world_model_version
            ),
            kind="STOP",
            left_speed_dps=0,
            right_speed_dps=0,
            duration_ms=0,
            reason_code=reason_code,
            proposal_id=proposal_id,
        ))

    def force_stop(
        self,
        snapshot: NavigationSnapshot,
        reason_code: str = "terminal_stop",
    ) -> DrivePulse:
        """Create a STOP outside planner and episode budgets."""

        if not isinstance(snapshot, NavigationSnapshot):
            raise NavigationContractError(
                "invalid_snapshot",
                "force_stop requires NavigationSnapshot",
            )
        return self._stop(snapshot, reason_code)

    def cancel(self, pulse: DrivePulse) -> None:
        """Revoke a decision rejected after arbitration but before dispatch."""

        self._motion_authority.cancel(pulse)

    def _fresh_snapshot(
        self,
        snapshot: NavigationSnapshot,
        now_ms: int,
    ) -> Optional[str]:
        if (
            snapshot.robot_id != self.robot_id
            or snapshot.controller_instance_id
            != self.controller_instance_id
        ):
            return "controller_identity_mismatch"
        if (
            snapshot.captured_at_host_ms > now_ms
            or snapshot.state_observed_at_ms > now_ms
            or now_ms - snapshot.state_observed_at_ms
            > self.policy.max_snapshot_age_ms
        ):
            return "stale_navigation_snapshot"
        if (
            snapshot.clearance.observed_at_ms > now_ms
            or now_ms - snapshot.clearance.observed_at_ms
            > self.policy.max_safety_age_ms
        ):
            return "stale_safety_evidence"
        return None

    def _latch(self, snapshot: NavigationSnapshot) -> None:
        self._emergency_latched = True
        self._latched_goal_epoch = snapshot.goal_epoch

    def observe_emergency(self, snapshot: NavigationSnapshot) -> bool:
        """Latch explicit hazards observed outside the arbitration call."""

        if not isinstance(snapshot, NavigationSnapshot):
            raise NavigationContractError(
                "invalid_snapshot",
                "Emergency observation requires NavigationSnapshot",
            )
        if (
            snapshot.robot_id != self.robot_id
            or snapshot.controller_instance_id
            != self.controller_instance_id
        ):
            return False
        if (
            snapshot.touch_pressed
            or snapshot.active_faults
            or snapshot.motors_running
        ):
            self._latch(snapshot)
            return True
        return False

    def request_rearm(
        self,
        goal: WaypointGoal,
        safe_snapshots: Tuple[NavigationSnapshot, ...],
    ) -> bool:
        """Rearm only for a newer goal after multiple fresh safe samples."""

        if not self._emergency_latched:
            return True
        if (
            not isinstance(goal, WaypointGoal)
            or not isinstance(safe_snapshots, tuple)
            or len(safe_snapshots) < self.policy.rearm_safe_samples
            or self._latched_goal_epoch is None
            or goal.goal_epoch <= self._latched_goal_epoch
        ):
            return False
        now_ms = self._now_ms()
        selected = safe_snapshots[-self.policy.rearm_safe_samples :]
        previous_state_version = 0
        previous_safety_time = -1
        for snapshot in selected:
            if (
                not isinstance(snapshot, NavigationSnapshot)
                or not snapshot.bound_to(goal)
                or self._fresh_snapshot(snapshot, now_ms) is not None
                or snapshot.touch_pressed
                or snapshot.active_faults
                or snapshot.motors_running
                or snapshot.state_version <= previous_state_version
                or snapshot.clearance.observed_at_ms
                <= previous_safety_time
            ):
                return False
            previous_state_version = snapshot.state_version
            previous_safety_time = snapshot.clearance.observed_at_ms
        self._emergency_latched = False
        self._latched_goal_epoch = None
        return True

    @staticmethod
    def _binding_matches(
        proposal: StampedProposal,
        snapshot: NavigationSnapshot,
        goal: WaypointGoal,
    ) -> bool:
        value = proposal.proposal
        return (
            value.goal_id == goal.goal_id == snapshot.goal_id
            and value.goal_epoch == goal.goal_epoch
            == snapshot.goal_epoch
            and value.plan_revision == goal.plan_revision
            == snapshot.plan_revision
            and value.based_on_state_version
            == snapshot.state_version
            and value.based_on_world_model_version
            == snapshot.world_model_version
        )

    def _consume_and_filter(
        self,
        proposals: Tuple[StampedProposal, ...],
        snapshot: NavigationSnapshot,
        goal: WaypointGoal,
        now_ms: int,
    ) -> Tuple[StampedProposal, ...]:
        if not isinstance(proposals, tuple):
            raise NavigationContractError(
                "invalid_proposals",
                "Supervisor proposals must be a tuple",
            )
        counts = {}
        sequence_counts = {}
        for value in proposals:
            if not isinstance(value, StampedProposal):
                raise NavigationContractError(
                    "invalid_stamped_proposal",
                    "Supervisor requires StampedProposal values",
                )
            proposal_id = value.proposal.proposal_id
            counts[proposal_id] = counts.get(proposal_id, 0) + 1
            sequence_key = (value.source_id, value.source_sequence)
            sequence_counts[sequence_key] = (
                sequence_counts.get(sequence_key, 0) + 1
            )

        previous_high_water = dict(self._source_high_water)
        valid = []
        batch_high_water = dict(previous_high_water)
        for value in proposals:
            proposal_id = value.proposal.proposal_id
            already_seen = proposal_id in self._seen_proposal_ids
            if not already_seen:
                self._remember_proposal_id(proposal_id)
            current_high = previous_high_water.get(value.source_id, 0)
            batch_high_water[value.source_id] = max(
                batch_high_water.get(value.source_id, 0),
                value.source_sequence,
            )
            if (
                already_seen
                or counts[proposal_id] != 1
                or sequence_counts[
                    (value.source_id, value.source_sequence)
                ]
                != 1
                or value.source_sequence <= current_high
                or value.received_at_ms > now_ms
                or now_ms >= value.valid_until_ms
                or value.valid_until_ms - value.received_at_ms
                > self.policy.max_proposal_ttl_ms
                or not self._binding_matches(value, snapshot, goal)
            ):
                continue
            valid.append(value)
        self._source_high_water = batch_high_water
        return tuple(valid)

    def decide(
        self,
        snapshot: NavigationSnapshot,
        goal: WaypointGoal,
        proposals: Tuple[StampedProposal, ...],
    ) -> DrivePulse:
        if not isinstance(snapshot, NavigationSnapshot):
            raise NavigationContractError(
                "invalid_snapshot",
                "Supervisor requires NavigationSnapshot",
            )
        if not isinstance(goal, WaypointGoal):
            raise NavigationContractError(
                "invalid_goal",
                "Supervisor requires WaypointGoal",
            )
        now_ms = self._now_ms()
        if (
            snapshot.robot_id != self.robot_id
            or snapshot.controller_instance_id
            != self.controller_instance_id
        ):
            return self._stop(
                snapshot,
                "controller_identity_mismatch",
            )
        # An explicit hazard from the expected controller is conservative
        # evidence even when another field in the snapshot is stale.  It
        # must latch before freshness rejection so a later clean snapshot
        # in the same goal cannot resume motion automatically.
        if (
            snapshot.touch_pressed
            or snapshot.active_faults
            or snapshot.motors_running
        ):
            self._latch(snapshot)
            return self._stop(snapshot, "emergency_stop_latched")
        freshness_error = self._fresh_snapshot(snapshot, now_ms)
        if freshness_error is not None:
            return self._stop(snapshot, freshness_error)
        if not snapshot.bound_to(goal):
            return self._stop(snapshot, "goal_binding_mismatch")
        if (
            self._active_goal_epoch is not None
            and goal.goal_epoch < self._active_goal_epoch
        ):
            return self._stop(snapshot, "stale_goal_epoch")
        if (
            self._active_goal_epoch is None
            or goal.goal_epoch > self._active_goal_epoch
        ):
            self._active_goal_epoch = goal.goal_epoch
            self._seen_proposal_ids.clear()
            self._seen_proposal_order.clear()
        if self._emergency_latched:
            return self._stop(snapshot, "emergency_stop_latched")

        valid = self._consume_and_filter(
            proposals,
            snapshot,
            goal,
            now_ms,
        )
        if not valid:
            return self._stop(snapshot, "no_fresh_proposal")

        highest_authority = max(
            value.authority_rank for value in valid
        )
        authority_group = tuple(
            value
            for value in valid
            if value.authority_rank == highest_authority
        )
        highest_priority = max(value.priority for value in authority_group)
        top = tuple(
            value
            for value in authority_group
            if value.priority == highest_priority
        )
        semantic_keys = {
            value.proposal.semantic_key() for value in top
        }
        if len(semantic_keys) != 1:
            return self._stop(snapshot, "ambiguous_top_priority")

        selected = min(
            top,
            key=lambda value: (
                value.source_id,
                value.proposal.proposal_id,
            ),
        )
        proposal = selected.proposal
        if proposal.decision in ("HOLD", "ABORT"):
            return self._stop(
                snapshot,
                "{}_{}".format(
                    proposal.decision.lower(),
                    proposal.reason_code,
                ),
                proposal.proposal_id,
            )

        if isinstance(proposal.segment, AdvanceSegment):
            return self._authorize_advance(snapshot, proposal)
        if isinstance(proposal.segment, TurnSegment):
            return self._authorize_turn(snapshot, proposal)
        return self._stop(
            snapshot,
            "unsupported_segment",
            proposal.proposal_id,
        )

    def _authorize_advance(
        self,
        snapshot: NavigationSnapshot,
        proposal,
    ) -> DrivePulse:
        segment = proposal.segment
        evidence = snapshot.clearance
        if not evidence.positively_cleared_for_simulation:
            return self._stop(
                snapshot,
                "forward_clearance_unknown",
                proposal.proposal_id,
            )

        speed_mm_s = min(
            segment.speed_mm_s,
            self.policy.max_linear_speed_mm_s,
        )
        duration_ms = min(
            self.policy.max_pulse_ms,
            max(1, int(math.ceil(
                segment.distance_mm * 1_000.0 / speed_mm_s
            ))),
        )
        planned_distance_mm = int(
            math.ceil(speed_mm_s * duration_ms / 1_000.0)
        )
        if (
            evidence.forward_mm
            < self.policy.forward_reserve_mm + planned_distance_mm
        ):
            return self._stop(
                snapshot,
                "forward_clearance_insufficient",
                proposal.proposal_id,
            )

        logical_wheel_dps = max(
            1,
            int(math.ceil(
                speed_mm_s
                * self.profile.encoder_mdeg_per_mm
                / 1_000.0
            )),
        )
        logical_wheel_dps = min(
            logical_wheel_dps,
            self.profile.max_wheel_speed_dps,
        )
        return self._authorize_pulse(DrivePulse(
            decision_id=self._decision_id(),
            arbiter_id=self.arbiter_id,
            robot_id=snapshot.robot_id,
            controller_instance_id=snapshot.controller_instance_id,
            goal_id=snapshot.goal_id,
            goal_epoch=snapshot.goal_epoch,
            plan_revision=snapshot.plan_revision,
            based_on_state_version=snapshot.state_version,
            based_on_world_model_version=(
                snapshot.world_model_version
            ),
            kind="DRIVE",
            left_speed_dps=(
                logical_wheel_dps * self.profile.left_motor_sign
            ),
            right_speed_dps=(
                logical_wheel_dps * self.profile.right_motor_sign
            ),
            duration_ms=duration_ms,
            reason_code="authorized_advance",
            proposal_id=proposal.proposal_id,
        ))

    def _authorize_turn(
        self,
        snapshot: NavigationSnapshot,
        proposal,
    ) -> DrivePulse:
        segment = proposal.segment
        angular_speed = min(
            segment.angular_speed_mdeg_s,
            self.policy.max_angular_speed_mdeg_s,
        )
        duration_ms = min(
            self.policy.max_pulse_ms,
            max(1, int(math.ceil(
                abs(segment.angle_mdeg) * 1_000.0 / angular_speed
            ))),
        )
        logical_wheel_dps = max(
            1,
            int(math.ceil(
                angular_speed
                * self.profile.encoder_mdeg_per_body_degree
                / 1_000_000.0
            )),
        )
        logical_wheel_dps = min(
            logical_wheel_dps,
            self.profile.max_wheel_speed_dps,
        )
        direction = 1 if segment.angle_mdeg > 0 else -1
        logical_left = -direction * logical_wheel_dps
        logical_right = direction * logical_wheel_dps
        return self._authorize_pulse(DrivePulse(
            decision_id=self._decision_id(),
            arbiter_id=self.arbiter_id,
            robot_id=snapshot.robot_id,
            controller_instance_id=snapshot.controller_instance_id,
            goal_id=snapshot.goal_id,
            goal_epoch=snapshot.goal_epoch,
            plan_revision=snapshot.plan_revision,
            based_on_state_version=snapshot.state_version,
            based_on_world_model_version=(
                snapshot.world_model_version
            ),
            kind="DRIVE",
            left_speed_dps=(
                logical_left * self.profile.left_motor_sign
            ),
            right_speed_dps=(
                logical_right * self.profile.right_motor_sign
            ),
            duration_ms=duration_ms,
            reason_code="authorized_turn",
            proposal_id=proposal.proposal_id,
        ))
