"""Fail-open passive-shadow helpers for the legacy navigation runtime."""

from copy import deepcopy
import hashlib
from typing import Callable, Mapping, Optional

from .legacy_decision_shadow_projection import (
    project_validated_legacy_decision,
)
from .legacy_shadow_runtime_context import (
    build_legacy_shadow_basis,
    build_legacy_shadow_goal,
    calibration_fingerprint,
    stable_shadow_id,
)
from .physical_navigation_contract import (
    FINISH,
    NavigationDecision,
    json_bytes,
)


class LegacyShadowRuntimeMixin:
    """Observe legacy execution without affecting its return values or calls."""

    def _shadow_capture(
        self,
        stage: str,
        fact_factory: Callable[[], Mapping[str, object]],
    ) -> bool:
        """Build and publish facts entirely inside the fail-open boundary."""

        shadow = self._canonical_shadow
        if shadow is None:
            return False
        try:
            facts = fact_factory()
            if not isinstance(facts, Mapping):
                raise TypeError("shadow fact factory returned a non-mapping")
            shadow.observe(stage, **deepcopy(dict(facts)))
        except Exception as error:
            # Custom observers are contained here even if they do not use the
            # repository's own fail-open wrapper. Shadowing may disappear;
            # the validated legacy decision and physical execution may not.
            self._disable_canonical_shadow(stage, error)
            return False
        return True

    def _disable_canonical_shadow(
        self,
        fault_stage: str,
        error: Exception,
    ) -> None:
        shadow = self._canonical_shadow
        self._canonical_shadow = None
        self._canonical_shadow_goal = None
        self._canonical_shadow_calibration_fingerprint = None
        if shadow is None:
            return
        try:
            disable = getattr(shadow, "disable", None)
            if callable(disable):
                disable(fault_stage, error)
            else:
                shadow.observe(
                    "shadow_disabled",
                    fault_stage=fault_stage,
                    error_type=type(error).__name__,
                    stable_code="runtime_shadow_failed",
                )
        except Exception:
            pass

    @staticmethod
    def _shadow_digest(value: object) -> Optional[str]:
        """Return a stable digest without allowing telemetry to fault motion."""

        try:
            return hashlib.sha256(json_bytes(value)).hexdigest()
        except Exception:
            return None

    def _shadow_navigation_identity(
        self,
        navigation: Mapping[str, object],
    ) -> Mapping[str, object]:
        return {
            "robot_id": navigation.get("robot_id"),
            "controller_instance_id": navigation.get(
                "controller_instance_id"
            ),
            "frame_id": navigation.get("frame_id"),
            "map_generation_id": navigation.get("map_generation_id"),
            "map_version": navigation.get("map_version"),
            "localization_valid": navigation.get("localization_valid"),
            "navigation_digest": self._shadow_digest(navigation),
        }

    def _shadow_calibration(
        self,
        action_specs: Mapping[str, Mapping[str, object]],
    ) -> Mapping[str, object]:
        return {
            "odometry": {
                "linear_mm_per_encoder_degree": (
                    self.memory.odometry_calibration
                    .linear_mm_per_encoder_degree
                ),
                "turn_mdeg_per_opposed_encoder_degree": (
                    self.memory.odometry_calibration
                    .turn_mdeg_per_opposed_encoder_degree
                ),
            },
            "active_scan": vars(self.active_scan_calibration),
            "collision_geometry": (
                self.memory.hazard_map.calibration.collision_geometry()
            ),
            "action_specs": action_specs,
        }

    def _initialize_canonical_shadow(
        self,
        *,
        action_specs: Mapping[str, Mapping[str, object]],
    ) -> Optional[Mapping[str, object]]:
        if self._canonical_shadow is None:
            return None
        try:
            calibration = self._shadow_calibration(action_specs)
            activated_at_ms = self.unix_ms()
            self._canonical_shadow_goal = build_legacy_shadow_goal(
                episode_id=self.episode_id,
                objective=self.config.goal,
                locale=self.config.locale,
                activated_at_ms=activated_at_ms,
            )
            self._canonical_shadow_calibration_fingerprint = (
                calibration_fingerprint(calibration)
            )
            # The journal's scalar contract deliberately avoids floating-point
            # facts. The exact calibration remains bound by its fingerprint.
            return {
                "fingerprint": (
                    self._canonical_shadow_calibration_fingerprint
                ),
                "odometry": {
                    "linear_mm_per_encoder_degree": format(
                        calibration["odometry"][
                            "linear_mm_per_encoder_degree"
                        ],
                        ".17g",
                    ),
                    "turn_mdeg_per_opposed_encoder_degree": calibration[
                        "odometry"
                    ]["turn_mdeg_per_opposed_encoder_degree"],
                },
                "active_scan": calibration["active_scan"],
                "collision_geometry": calibration["collision_geometry"],
                "action_specs_digest": self._shadow_digest(action_specs),
            }
        except Exception as error:
            self._disable_canonical_shadow("episode_start", error)
            return None

    def _shadow_project_validated_decision(
        self,
        *,
        decision: NavigationDecision,
        post_maneuver_state: Mapping[str, object],
        observation: Mapping[str, object],
        navigation: Mapping[str, object],
        turn: int,
        attempt: int,
    ) -> None:
        goal = self._canonical_shadow_goal
        fingerprint = self._canonical_shadow_calibration_fingerprint
        if (
            self._canonical_shadow is None
            or goal is None
            or fingerprint is None
        ):
            return
        def facts():
            accepted_at_ms = self.unix_ms()
            basis = build_legacy_shadow_basis(
                robot_id=self.memory.robot_id,
                controller_id=getattr(
                    self.transport,
                    "controller_id",
                    "legacy-controller",
                ),
                controller_instance_id=self.memory.controller_instance_id,
                goal_epoch=goal.goal_epoch,
                observation=observation,
                navigation=navigation,
                calibration_fingerprint_value=fingerprint,
            )
            identity = (
                self.episode_id,
                turn,
                attempt,
                basis.navigation_basis_id,
                decision.action,
                decision.plan,
            )
            step_ids = tuple(
                stable_shadow_id("shadow-step", *identity, index)
                for index, _action in enumerate(decision.plan)
            )
            projection = project_validated_legacy_decision(
                decision,
                post_maneuver_state,
                goal=goal,
                basis=basis,
                accepted_at_ms=accepted_at_ms,
                intent_id=stable_shadow_id("shadow-intent", *identity),
                plan_id=stable_shadow_id("shadow-plan", *identity),
                plan_revision=(turn - 1)
                * self.config.max_validation_attempts
                + attempt,
                step_ids=(() if decision.action == FINISH else step_ids),
                scan_profile_id="active-ir-front-arc-v1",
            )
            return {
                "turn": turn,
                "attempt": attempt,
                "projection": projection.to_dict(),
                "physical_authority": "legacy",
                "extra_model_calls": 0,
                "canonical_receipt_created": False,
            }

        self._shadow_capture(
            "canonical_projection",
            facts,
        )

    def _shadow_record_planner_input(
        self,
        *,
        turn: int,
        attempt: int,
        observation: Mapping[str, object],
        mission: Mapping[str, object],
        navigation: Mapping[str, object],
        maneuver_state: Mapping[str, object],
        available_actions,
        last_tool_result,
        validation_feedback,
    ) -> None:
        def facts():
            recent = tuple(self._recent_committed_utterances)
            actions = tuple(available_actions)
            planner_snapshot = {
                "episode_id": self.episode_id,
                "turn": turn,
                "locale": self.config.locale,
                "observation": observation,
                "mission": mission,
                "navigation": navigation,
                "maneuver_state": maneuver_state,
                "available_actions": actions,
                "last_tool_result": last_tool_result,
                "validation_feedback": validation_feedback,
                "recent_committed_utterances": recent,
            }
            return {
                "turn": turn,
                "attempt": attempt,
                "locale": self.config.locale,
                "observation_state_version": observation["state_version"],
                "mission": mission,
                "navigation_identity": self._shadow_navigation_identity(
                    navigation
                ),
                "maneuver_state": maneuver_state,
                "available_actions": actions,
                "last_tool_result_digest": self._shadow_digest(
                    last_tool_result
                ),
                "validation_feedback": validation_feedback,
                "recent_committed_utterances": recent,
                "planner_snapshot_digest": self._shadow_digest(
                    planner_snapshot
                ),
            }

        self._shadow_capture(
            "planner_input",
            facts,
        )

    def _shadow_record_validated_decision(
        self,
        *,
        turn: int,
        attempt: int,
        decision: NavigationDecision,
        observation: Mapping[str, object],
        mission: Mapping[str, object],
        navigation: Mapping[str, object],
        maneuver_before: Mapping[str, object],
        maneuver_after: Mapping[str, object],
        model_latency_ms: int,
        served_model,
        planner_telemetry: Mapping[str, object],
    ) -> None:
        accepted = self._shadow_capture(
            "validated_decision",
            lambda: {
                "turn": turn,
                "attempt": attempt,
                "decision": decision.to_dict(),
                "observation_state_version": observation["state_version"],
                "mission": mission,
                "navigation_identity": self._shadow_navigation_identity(
                    navigation
                ),
                "maneuver_before": maneuver_before,
                "maneuver_after": maneuver_after,
                "model_latency_ms": model_latency_ms,
                "served_model": served_model,
                "planner_telemetry": planner_telemetry,
                "physical_authority": "legacy",
                "extra_model_calls": 0,
            },
        )
        if not accepted:
            return
        self._shadow_project_validated_decision(
            decision=decision,
            post_maneuver_state=maneuver_after,
            observation=observation,
            navigation=navigation,
            turn=turn,
            attempt=attempt,
        )

    def _shadow_record_episode_start(
        self,
        *,
        mission,
        observation: Mapping[str, object],
        action_specs: Mapping[str, Mapping[str, object]],
    ) -> None:
        calibration = self._initialize_canonical_shadow(
            action_specs=action_specs,
        )
        if calibration is None:
            return
        self._shadow_capture(
            "episode_start",
            lambda: {
                "goal": self.config.goal,
                "locale": self.config.locale,
                "mission": {
                    "episode_id": mission.episode_id,
                    "minimum_forward_progress_mm": (
                        mission.minimum_forward_progress_mm
                    ),
                    "origin_x_mm": mission.origin_x_mm,
                    "origin_y_mm": mission.origin_y_mm,
                    "reference_heading_mdeg": (
                        mission.reference_heading_mdeg
                    ),
                    "heading_tolerance_mdeg": (
                        mission.heading_tolerance_mdeg
                    ),
                },
                "controller_id": getattr(
                    self.transport,
                    "controller_id",
                    "legacy-controller",
                ),
                "observation": observation,
                "action_specs_digest": self._shadow_digest(action_specs),
                "navigation_identity": self._shadow_navigation_identity(
                    self.memory.context()
                ),
                "calibration": calibration,
                "physical_authority": "legacy",
                "extra_model_calls": 0,
            },
        )

    def _shadow_record_committed_decision(
        self,
        *,
        turn: int,
        decision: NavigationDecision,
    ) -> None:
        self._shadow_capture(
            "committed_decision",
            lambda: {
                "turn": turn,
                "decision": decision.to_dict(),
                "physical_authority": "legacy",
            },
        )

    def _shadow_record_motion(
        self,
        *,
        turn: Optional[int],
        source: str,
        action: str,
        worker_result: Mapping[str, object],
        observation: Mapping[str, object],
        feedback: Mapping[str, object],
    ) -> None:
        self._shadow_capture(
            "legacy_execution_observed",
            lambda: {
                "turn": turn,
                "execution_source": source,
                "operation": "pulse",
                "action": action,
                "worker_result_digest": self._shadow_digest(worker_result),
                "observation": observation,
                "feedback": feedback,
                "navigation_identity": self._shadow_navigation_identity(
                    self.memory.context()
                ),
                "canonical_receipt_created": False,
                "command_receipt_parity": (
                    "NOT_EVALUATED_LEGACY_PROTOCOL_LACKS_"
                    "CANONICAL_CORRELATION"
                ),
            },
        )

    def _shadow_record_observe(
        self,
        *,
        turn: int,
        source: str,
        worker_state_version: int,
        observation: Mapping[str, object],
        feedback: Mapping[str, object],
    ) -> None:
        self._shadow_capture(
            "legacy_execution_observed",
            lambda: {
                "turn": turn,
                "execution_source": source,
                "operation": "observe",
                "action": "OBSERVE",
                "worker_state_version": worker_state_version,
                "observation": observation,
                "feedback": feedback,
                "canonical_receipt_created": False,
                "command_receipt_parity": (
                    "NOT_EVALUATED_LEGACY_PROTOCOL_LACKS_"
                    "CANONICAL_CORRELATION"
                ),
            },
        )

    def _shadow_record_scan(
        self,
        *,
        turn: int,
        source: str,
        observation: Mapping[str, object],
        feedback: Mapping[str, object],
    ) -> None:
        self._shadow_capture(
            "legacy_execution_observed",
            lambda: {
                "turn": turn,
                "execution_source": source,
                "operation": "SCAN_FRONT_ARC",
                "action": "SCAN_FRONT_ARC",
                "observation": observation,
                "feedback_digest": self._shadow_digest(feedback),
                "feedback_summary": {
                    key: feedback.get(key)
                    for key in (
                        "status",
                        "reason",
                        "target_hypothesis_id",
                        "bilateral_complete",
                        "evidence_disposition",
                    )
                },
                "canonical_receipt_created": False,
                "command_receipt_parity": (
                    "NOT_EVALUATED_LEGACY_PROTOCOL_LACKS_"
                    "CANONICAL_CORRELATION"
                ),
            },
        )

    def _shadow_record_route_handoff(
        self,
        *,
        turn: int,
        route_result,
    ) -> None:
        def facts():
            route = route_result.route
            route_payload = None if route is None else route.to_dict()
            return {
                "turn": turn,
                "handoff_reason": route_result.handoff_reason,
                "actions": tuple(route_result.actions),
                "route_summary": (
                    None
                    if route_payload is None
                    else {
                        key: route_payload.get(key)
                        for key in (
                            "route_id",
                            "version",
                            "status",
                            "target_hypothesis_id",
                            "detour_side",
                            "waypoint_index",
                        )
                    }
                ),
                "route_digest": self._shadow_digest(route_payload),
                "last_tool_result_digest": self._shadow_digest(
                    route_result.last_tool_result
                ),
            }

        self._shadow_capture(
            "local_detour_route_handoff",
            facts,
        )

    def _shadow_record_execution_veto(
        self,
        *,
        turn: int,
        action: str,
        validation: Mapping[str, object],
        observation: Mapping[str, object],
    ) -> None:
        self._shadow_capture(
            "execution_veto",
            lambda: {
                "turn": turn,
                "action": action,
                "validation": validation,
                "observation_state_version": observation["state_version"],
                "observation_digest": self._shadow_digest(observation),
                "physical_authority": "legacy",
                "canonical_receipt_created": False,
            },
        )

    def _shadow_record_plan_tail_handoff(
        self,
        *,
        turn: int,
        tail,
        tail_result,
    ) -> None:
        self._shadow_capture(
            "plan_tail_handoff",
            lambda: {
                "turn": turn,
                "source_plan": tuple(tail.source_plan),
                "remaining_actions": tuple(tail.remaining_actions),
                "cancelled": tail.cancelled,
                "cancelled_reason": tail.cancelled_reason,
                "actions": tuple(tail_result.actions),
                "completed": tail_result.completed,
            },
        )

    def _shadow_record_terminal(
        self,
        *,
        terminal_reason: str,
        completed: bool,
        turns: int,
        actions,
        shutdown_clean: bool,
        counters: Mapping[str, int],
    ) -> None:
        self._shadow_capture(
            "terminal",
            lambda: {
                "terminal_reason": terminal_reason,
                "completed": completed,
                "turns": turns,
                "actions": tuple(actions),
                "shutdown_clean": shutdown_clean,
                "model_calls": counters["model_calls"],
                "model_latency_ms": counters["model_latency_ms"],
                "physical_authority": "legacy",
                "extra_model_calls": 0,
                "offer_parity": "NOT_EVALUATED",
                "canonical_receipt_created": False,
                "command_receipt_parity": (
                    "NOT_EVALUATED_LEGACY_PROTOCOL_LACKS_"
                    "CANONICAL_CORRELATION"
                ),
            },
        )


__all__ = ("LegacyShadowRuntimeMixin",)
