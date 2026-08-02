"""Small reversible bridge from compact intents to the legacy EV3 runtime.

The bridge accelerates only states whose deterministic facts leave one
supported semantic intent.  Complex navigation, active manoeuvres and any
ambiguous state are delegated to the existing planner before a compact model
request is made.  Motor authority and validation remain in the physical
runtime.
"""

from dataclasses import dataclass, replace
import hashlib
import json
import time
from typing import Callable, Mapping, Optional, Sequence

from .compact_navigation_intent_planner import (
    CompactNavigationIntentPlanner,
)
from .lm_studio_navigation import (
    NavigationPlannerResult,
)
from .maneuver_commitment import empty_commitment
from .navigation_intent_context import build_navigation_intent_prompt
from .navigation_intent_proposal import (
    FOLLOW_DIRECTION,
    SCAN_TARGET,
    NavigationIntentOffer,
)
from .physical_agent_state import (
    AgentPhase,
    ControllerKey,
    GoalAssignment,
    NavigationBasis,
    PhysicalAgentState,
    PlanningCause,
    PlanningTicket,
)
from .physical_intent_contract import IntentPlanningRequest
from .physical_navigation_contract import (
    ADVANCE,
    DECISION_SCHEMA,
    FINISH,
    SCAN_FRONT_ARC,
    NavigationDecision,
)


LEGACY_FULL_PATH = "legacy_full"
ZERO_CALL_FOLLOW_PATH = "compact_zero_call_follow_direction"
ZERO_CALL_SCAN_PATH = "compact_zero_call_scan_target"
_TICKET_TTL_MS = 5_000


@dataclass(frozen=True)
class HybridPlannerTelemetry:
    """One observational record emitted after a planner handoff."""

    decision_path: str
    model_call_count: int
    latency_ms: int
    intent: Optional[str]


@dataclass(frozen=True)
class _FastCase:
    intent: str
    target_id: Optional[str] = None


def _canonical_digest(value: object) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()[:24]


def _safe_mapping(value: object) -> Mapping[str, object]:
    return value if isinstance(value, Mapping) else {}


def _clean_previous_result(value: object) -> bool:
    if value is None:
        return True
    if not isinstance(value, Mapping):
        return False
    operation = value.get("operation")
    status = value.get("status")
    return (
        operation == "pulse" and status == "completed"
    ) or (
        operation == "observe" and status == "observed"
    )


def _obstacle_stop_result(value: object) -> bool:
    return (
        isinstance(value, Mapping)
        and value.get("operation") == "pulse"
        and value.get("requested_action") == ADVANCE
        and value.get("status") in ("denied", "interrupted")
        and value.get("reason") == "infrared_blocked"
    )


def _fast_case(
    *,
    observation: Mapping[str, object],
    mission: Mapping[str, object],
    navigation: Mapping[str, object],
    maneuver_state: Mapping[str, object],
    available_actions: Sequence[str],
    last_tool_result: Optional[Mapping[str, object]],
    validation_feedback: Optional[Mapping[str, object]],
) -> Optional[_FastCase]:
    """Return one supported intent only when published facts are decisive."""

    if validation_feedback is not None or maneuver_state.get("active") is not None:
        return None
    if (
        mission.get("completed") is True
        or mission.get("localization_valid") is not True
        or mission.get("touch_clear") is not True
    ):
        return None
    touch = _safe_mapping(observation.get("touch"))
    infrared = _safe_mapping(observation.get("infrared"))
    budgets = _safe_mapping(observation.get("budgets"))
    if touch.get("pressed") is not False or budgets.get("motion_fault_latched") is not False:
        return None

    available = frozenset(available_actions)
    goal_geometry = _safe_mapping(navigation.get("goal_geometry"))
    conflicts = goal_geometry.get("conflicts")
    if not isinstance(conflicts, list):
        return None
    conflict_ids = {
        item.get("hypothesis_id")
        for item in conflicts
        if isinstance(item, Mapping)
        and isinstance(item.get("hypothesis_id"), str)
    }

    if (
        ADVANCE in available
        and _clean_previous_result(last_tool_result)
        and infrared.get("blocked") is False
        and mission.get("goal_corridor_clear") is True
        and mission.get("goal_heading_aligned") is True
        and mission.get("all_known_hazards_passed") is True
        and not conflict_ids
        and isinstance(mission.get("remaining_longitudinal_progress_mm"), int)
    ):
        delta = _safe_mapping(
            mission.get("candidate_action_longitudinal_deltas_mm")
        ).get(ADVANCE, 0)
        remaining = mission["remaining_longitudinal_progress_mm"]
        if (
            isinstance(delta, int)
            and not isinstance(delta, bool)
            and delta > 0
            and not isinstance(remaining, bool)
            and remaining >= 2 * delta
        ):
            return _FastCase(FOLLOW_DIRECTION)

    eligible_targets = navigation.get(
        "scan_eligible_target_hypothesis_ids",
        (),
    )
    hazards = navigation.get("navigation_hazard_hypotheses")
    if (
        SCAN_FRONT_ARC not in available
        or not (
            _clean_previous_result(last_tool_result)
            or _obstacle_stop_result(last_tool_result)
        )
        or mission.get("goal_corridor_clear") is not False
        or not isinstance(eligible_targets, (list, tuple))
        or len(eligible_targets) != 1
        or not isinstance(hazards, list)
    ):
        return None
    target_id = eligible_targets[0]
    if not isinstance(target_id, str) or target_id not in conflict_ids:
        return None
    matches = [
        item
        for item in hazards
        if isinstance(item, Mapping)
        and item.get("hypothesis_id") == target_id
    ]
    if len(matches) != 1 or matches[0].get("route_commitment_ready") is True:
        return None
    return _FastCase(SCAN_TARGET, target_id=target_id)


class HybridNavigationPlanner:
    """Use the compact zero-call path for clear cases, legacy otherwise."""

    def __init__(
        self,
        *,
        legacy_planner: object,
        compact_client: object,
        monotonic: Callable[[], float] = time.monotonic,
        unix_ms: Callable[[], int] = lambda: time.time_ns() // 1_000_000,
        telemetry: Optional[Callable[[HybridPlannerTelemetry], None]] = None,
    ):
        if (
            not callable(getattr(legacy_planner, "decide", None))
            or not callable(getattr(compact_client, "decide", None))
            or not callable(monotonic)
            or not callable(unix_ms)
            or (telemetry is not None and not callable(telemetry))
        ):
            raise ValueError("hybrid planner dependencies are invalid")
        self._legacy_planner = legacy_planner
        self._compact_client = compact_client
        self._monotonic = monotonic
        self._unix_ms = unix_ms
        self._telemetry = telemetry

    def _publish(self, value: HybridPlannerTelemetry) -> None:
        if self._telemetry is None:
            return
        try:
            self._telemetry(value)
        except Exception:
            return

    @staticmethod
    def _legacy_result(result):
        if not isinstance(result, NavigationPlannerResult):
            return result
        return replace(
            result,
            decision_path=LEGACY_FULL_PATH,
            model_call_count=1,
        )

    def _legacy(self, arguments, started: float):
        result = self._legacy_result(self._legacy_planner.decide(**arguments))
        latency_ms = (
            result.latency_ms
            if isinstance(result, NavigationPlannerResult)
            else max(0, int(round((self._monotonic() - started) * 1_000)))
        )
        self._publish(HybridPlannerTelemetry(
            decision_path=LEGACY_FULL_PATH,
            model_call_count=1,
            latency_ms=latency_ms,
            intent=None,
        ))
        return result

    def decide(
        self,
        *,
        episode_id: str,
        turn: int,
        locale: str,
        observation: Mapping[str, object],
        mission: Mapping[str, object],
        navigation: Mapping[str, object],
        maneuver_state: Mapping[str, object],
        available_actions: Sequence[str],
        last_tool_result: Optional[Mapping[str, object]],
        validation_feedback: Optional[Mapping[str, object]] = None,
        recent_committed_utterances: Sequence[str] = (),
    ) -> NavigationPlannerResult:
        arguments = {
            "episode_id": episode_id,
            "turn": turn,
            "locale": locale,
            "observation": observation,
            "mission": mission,
            "navigation": navigation,
            "maneuver_state": maneuver_state,
            "available_actions": available_actions,
            "last_tool_result": last_tool_result,
            "validation_feedback": validation_feedback,
            "recent_committed_utterances": recent_committed_utterances,
        }
        started = self._monotonic()
        case = _fast_case(
            observation=observation,
            mission=mission,
            navigation=navigation,
            maneuver_state=maneuver_state,
            available_actions=available_actions,
            last_tool_result=last_tool_result,
            validation_feedback=validation_feedback,
        )
        if case is None:
            return self._legacy(arguments, started)

        try:
            now_ms = self._unix_ms()
            digest = _canonical_digest({
                "episode_id": episode_id,
                "turn": turn,
                "state_version": observation.get("state_version"),
                "mission": mission,
                "navigation": navigation,
                "case": {
                    "intent": case.intent,
                    "target_id": case.target_id,
                },
            })
            controller_key = ControllerKey(
                str(navigation.get("robot_id") or "ev3rstorm"),
                "legacy-navigation-runtime",
                str(
                    navigation.get("controller_instance_id")
                    or "live-controller"
                ),
            )
            state_version = observation.get("state_version")
            map_version = navigation.get("map_version")
            basis = NavigationBasis(
                controller_key=controller_key,
                goal_epoch=1,
                controller_state_version=(
                    state_version
                    if isinstance(state_version, int)
                    and not isinstance(state_version, bool)
                    and state_version >= 1
                    else 1
                ),
                world_generation_id="world-{}".format(digest),
                world_model_version=(
                    map_version + 1
                    if isinstance(map_version, int)
                    and not isinstance(map_version, bool)
                    and map_version >= 0
                    else 1
                ),
                navigation_basis_id="basis-{}".format(digest),
                frame_id="frame-{}".format(digest),
                calibration_fingerprint="hybrid-canary-v1-{}".format(digest),
            )
            goal = GoalAssignment(
                goal_id="goal-{}".format(digest),
                goal_epoch=1,
                objective=str(
                    mission.get("user_goal")
                    or "Make safe progress in the assigned direction."
                ),
                source="LIVE_EV3_CANARY",
                locale=locale,
                activated_at_ms=max(0, now_ms - 1),
            )
            ticket = PlanningTicket(
                ticket_id="ticket-{}".format(digest),
                cause=PlanningCause.NEW_GOAL,
                basis=basis,
                created_at_ms=now_ms,
                valid_until_ms=now_ms + _TICKET_TTL_MS,
                consumed_at_ms=now_ms,
            )
            state = PhysicalAgentState(
                controller_key=controller_key,
                phase=AgentPhase.PLANNING,
                goal_epoch=1,
                goal=goal,
                basis=basis,
                planning_ticket=ticket,
            )
            request = IntentPlanningRequest(
                proposal_id="proposal-{}".format(digest),
                state=state,
                ticket=ticket,
            )
            offer = NavigationIntentOffer(
                ticket_id=ticket.ticket_id,
                basis=basis,
                offered_intents=(case.intent,),
                scan_target_ids=(
                    (case.target_id,) if case.intent == SCAN_TARGET else ()
                ),
            )
            planner = CompactNavigationIntentPlanner(
                offer_builder=lambda _request: offer,
                prompt_builder=lambda _request, value: (
                    build_navigation_intent_prompt(
                        goal=goal,
                        mission=mission,
                        navigation=navigation,
                        offer=value,
                    )
                ),
                client=self._compact_client,
                clock_ms=self._unix_ms,
            )
            envelope = planner(request)
            if envelope.proposal.intent != case.intent:
                raise ValueError("compact intent did not match the sole offer")

            if case.intent == FOLLOW_DIRECTION:
                action = ADVANCE
                plan = [ADVANCE, ADVANCE]
                reason_code = "PROGRESS_GOAL"
                assessment = "The forward corridor is clear; continue."
                target_id = None
                decision_path = ZERO_CALL_FOLLOW_PATH
            else:
                action = SCAN_FRONT_ARC
                plan = [SCAN_FRONT_ARC]
                reason_code = "HANDLE_OBSTACLE"
                assessment = "Inspect the blocking obstacle before routing."
                target_id = case.target_id
                decision_path = ZERO_CALL_SCAN_PATH
            decision = NavigationDecision.from_mapping(
                {
                    "schema": DECISION_SCHEMA,
                    "episode_id": episode_id,
                    "turn": turn,
                    "based_on_state_version": observation["state_version"],
                    "action": action,
                    "plan": plan,
                    "reason_code": reason_code,
                    "assessment": assessment,
                    "utterance": None,
                    "perception_target_hypothesis_id": target_id,
                    "maneuver_commitment": empty_commitment(),
                },
                episode_id=episode_id,
                turn=turn,
                state_version=observation["state_version"],
                available_actions=available_actions,
                published_target_ids=tuple(
                    item["hypothesis_id"]
                    for item in navigation[
                        "navigation_hazard_hypotheses"
                    ]
                ),
            )
        except Exception:
            # This path has one concrete proposal, so CompactNavigationIntent-
            # Planner cannot have called the model.  Falling back here is not
            # a double inference and preserves the canary's instant rollback.
            return self._legacy(arguments, started)

        latency_ms = max(
            0,
            int(round((self._monotonic() - started) * 1_000)),
        )
        telemetry = HybridPlannerTelemetry(
            decision_path=decision_path,
            model_call_count=0,
            latency_ms=latency_ms,
            intent=case.intent,
        )
        self._publish(telemetry)
        return NavigationPlannerResult(
            decision=decision,
            latency_ms=latency_ms,
            served_model=None,
            usage=None,
            stats={
                "semantic_intent": case.intent,
            },
            context_byte_count=0,
            prompt_tokens=0,
            completion_tokens=0,
            total_tokens=0,
            estimated_prompt_tokens=0,
            prompt_token_budget=0,
            accounted_prompt_bytes=0,
            context_target_byte_count=0,
            context_hard_byte_count=0,
            decision_path=decision_path,
            model_call_count=0,
        )


__all__ = (
    "HybridNavigationPlanner",
    "HybridPlannerTelemetry",
    "LEGACY_FULL_PATH",
    "ZERO_CALL_FOLLOW_PATH",
    "ZERO_CALL_SCAN_PATH",
)
