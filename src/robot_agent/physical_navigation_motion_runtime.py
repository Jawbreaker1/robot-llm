"""Verified physical motion dispatch shared by navigation executors."""

from copy import deepcopy
from typing import Mapping, Tuple

from .physical_navigation_contract import (
    SCAN_SAMPLE_OPERATION,
    SCAN_TURN_OPERATION,
)
from .physical_navigation_runtime_errors import (
    PhysicalNavigationRuntimeError,
)
from .physical_odometry import verified_motion_from_result


class PhysicalNavigationMotionRuntimeMixin:
    """Execute semantic pulses and one bounded route heading correction."""

    def _execute_motion(
        self,
        action: str,
        *,
        action_specs: Mapping[str, Mapping[str, object]],
        defer_infrared_hazard: bool = False,
    ) -> Tuple[Mapping[str, object], Mapping[str, object]]:
        self._raise_if_cancelled("immediately_before_motion")
        response = self._active_request(
            "pulse",
            {"action": action},
            self.config.request_timeout_seconds,
        )
        observation = self._observation_from_response(
            "pulse",
            response,
            action,
        )
        result = response["result"]
        outcome = result["outcome"]
        hazard_evidence_deferred = (
            defer_infrared_hazard
            and outcome["status"] != "completed"
            and outcome["reason"] == "infrared_blocked"
        )
        motion = verified_motion_from_result(
            action,
            result,
            self.memory.drive_roles,
        )
        captured_at_ms = self.unix_ms()
        try:
            self.memory.apply_motion_result(
                action,
                result,
                captured_at_ms,
                record_hazard_evidence=not hazard_evidence_deferred,
            )
        except Exception:
            self._offer_invalid_localization(
                captured_at_ms=captured_at_ms,
                publication_stage="motion_result_invalidated",
                observation=observation,
            )
            raise
        self._offer_observation(
            observation,
            captured_at_ms=captured_at_ms,
            publication_stage="motion_result",
        )
        feedback = {
            "operation": "pulse",
            "requested_action": action,
            "status": outcome["status"],
            "reason": outcome["reason"],
            "worker_response_state_version": response["state_version"],
            "encoder_observation": motion.to_dict(),
            "resulting_pose": self.memory.pose.to_dict(),
            "hazard_evidence_deferred": hazard_evidence_deferred,
        }
        self._emit(
            "motion_result",
            action=action,
            outcome=deepcopy(outcome),
            navigation=self.memory.context(),
        )
        return observation, feedback

    def _execute_pass_heading_trim(
        self,
        *,
        observation: Mapping[str, object],
        relative_delta_mdeg: int,
    ) -> Tuple[Mapping[str, object], Mapping[str, object]]:
        try:
            turn_response = self._active_request(
                SCAN_TURN_OPERATION,
                {"relative_delta_mdeg": relative_delta_mdeg},
                self.config.request_timeout_seconds,
            )
            turn_result = turn_response.get("result", {})
            turn_outcome = turn_result.get("outcome", {})
            if (
                turn_result.get("relative_delta_mdeg")
                != relative_delta_mdeg
                or turn_outcome.get("status") != "completed"
                or turn_outcome.get("stop_confirmed") is not True
            ):
                raise PhysicalNavigationRuntimeError(
                    "invalid_scan_turn_result",
                    "Heading trim did not complete",
                )
            self._observation_from_response(
                SCAN_TURN_OPERATION,
                turn_response,
            )
            sample_response = self._active_request(
                SCAN_SAMPLE_OPERATION,
                {},
                self.config.request_timeout_seconds,
            )
            sampled = self._observation_from_response(
                SCAN_SAMPLE_OPERATION,
                sample_response,
            )
            captured_at_ms = self.unix_ms()
            self.memory.ingest_verified_heading_trim(
                sampled,
                captured_at_ms,
            )
            self._offer_observation(
                sampled,
                captured_at_ms=captured_at_ms,
                publication_stage="pass_heading_trim_result",
            )
        except Exception as error:
            self._raise_if_cancelled("during_pass_heading_trim")
            if self.memory.localization_valid:
                self._invalidate_localization(
                    "Route heading trim could not be verified and re-anchored",
                    publication_stage="pass_heading_trim_invalidated",
                )
            return observation, {
                "operation": "pass_heading_trim",
                "status": "failed",
                "reason": getattr(error, "code", type(error).__name__),
                "opening_clear": False,
                "relative_delta_mdeg": relative_delta_mdeg,
            }

        opening_clear = sampled["infrared"]["blocked"] is False
        return sampled, {
            "operation": "pass_heading_trim",
            "status": "completed" if opening_clear else "blocked",
            "reason": (
                "opening_clear"
                if opening_clear
                else "infrared_still_blocked"
            ),
            "opening_clear": opening_clear,
            "relative_delta_mdeg": relative_delta_mdeg,
            "worker_response_state_version": sample_response[
                "state_version"
            ],
            "resulting_pose": self.memory.pose.to_dict(),
        }


__all__ = ("PhysicalNavigationMotionRuntimeMixin",)
