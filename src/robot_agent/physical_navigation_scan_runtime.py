"""Active-scan execution and progress gating for physical navigation."""

from copy import deepcopy
from typing import Mapping, Optional, Tuple

from .active_ir_scan_contract import (
    ActiveIrScanCalibration,
    ModelScanChoice,
    build_scan_request,
    validate_scan_result,
    worst_case_scan_budget,
)
from .physical_navigation_contract import (
    SCAN_FRONT_ARC,
    NavigationDecision,
)
from .physical_navigation_runtime_errors import (
    EpisodeCancelled,
    PhysicalNavigationRuntimeError,
)
from .physical_observation_progress import (
    RestoredScanProgressBarrier,
    observation_progress_signature,
)


DEFAULT_SCAN_BUDGET = worst_case_scan_budget()


def _validate_side_probe_result(result, probe_offsets_mdeg) -> bool:
    expected = frozenset(probe_offsets_mdeg)
    first, second = probe_offsets_mdeg
    refinement = int(round((first + second) / 2.0))
    allowed = expected | {refinement}
    requested = frozenset(
        ray.requested_relative_bearing_mdeg for ray in result.rays
    )
    if not requested.issubset(allowed):
        raise PhysicalNavigationRuntimeError(
            "side_probe_result_mismatch",
            "Closed side probe result does not match its requested side",
        )
    return expected.issubset(requested)


class PhysicalNavigationScanRuntimeMixin:
    """Own the scan lifecycle while the host runtime owns orchestration."""

    def _arm_scan_progress_barrier(
        self,
        *,
        scan_id: str,
        target_hypothesis_id: str,
        observation: Mapping[str, object],
    ) -> None:
        drive_roles = self.memory.drive_roles
        if drive_roles is None:
            raise PhysicalNavigationRuntimeError(
                "drive_roles_unavailable",
                "Scan progress requires bound drive motor roles",
            )
        motor_roles = (drive_roles.left, drive_roles.right)
        self._restored_scan_progress_barriers[
            target_hypothesis_id
        ] = RestoredScanProgressBarrier(
            scan_id=scan_id,
            target_hypothesis_id=target_hypothesis_id,
            map_generation_id=self.memory.generation_id,
            pose=self.memory.pose,
            hazard_ids=self.memory.hazard_map.hazard_ids,
            observation_signature=observation_progress_signature(
                observation,
                motor_roles=motor_roles,
            ),
            motor_roles=motor_roles,
        )

    def _arm_restored_scan_progress_barrier(
        self,
        result,
        observation: Mapping[str, object],
    ) -> None:
        self._arm_scan_progress_barrier(
            scan_id=result.scan_id,
            target_hypothesis_id=result.target_hypothesis_id,
            observation=observation,
        )

    def _scan_blocked_target_ids(
        self,
        observation: Mapping[str, object],
    ) -> frozenset:
        blocked = set()
        for target_id, barrier in tuple(
            self._restored_scan_progress_barriers.items()
        ):
            reason = barrier.rearm_reason(
                map_generation_id=self.memory.generation_id,
                pose=self.memory.pose,
                hazard_ids=self.memory.hazard_map.hazard_ids,
                observation=observation,
            )
            if reason is None:
                blocked.add(target_id)
                continue
            del self._restored_scan_progress_barriers[target_id]
            self._emit(
                "active_scan_rearmed",
                prior_scan_id=barrier.scan_id,
                prior_target_hypothesis_id=(
                    barrier.target_hypothesis_id
                ),
                progress_reason=reason,
            )
        return frozenset(blocked)

    def _execute_scan(
        self,
        decision: NavigationDecision,
        *,
        observation: Mapping[str, object],
        deadline: float,
    ) -> Tuple[Mapping[str, object], Mapping[str, object]]:
        return self._execute_target_scan(
            target_hypothesis_id=(
                decision.perception_target_hypothesis_id
            ),
            calibration=self.active_scan_calibration,
            observation=observation,
            deadline=deadline,
        )

    def _execute_target_scan(
        self,
        *,
        target_hypothesis_id: str,
        calibration: ActiveIrScanCalibration,
        observation: Mapping[str, object],
        deadline: float,
        probe_offsets_mdeg: Optional[Tuple[int, ...]] = None,
    ) -> Tuple[Mapping[str, object], Mapping[str, object]]:
        self._raise_if_cancelled("immediately_before_scan")
        if (
            target_hypothesis_id
            in self._scan_blocked_target_ids(observation)
        ):
            feedback = {
                "operation": SCAN_FRONT_ARC,
                "status": "DENIED",
                "reason": "INTERVENING_NAVIGATION_PROGRESS_REQUIRED",
                "target_hypothesis_id": target_hypothesis_id,
                "host_selected_route_or_side": False,
            }
            self._emit("scan_denied", scan=deepcopy(feedback))
            return observation, feedback
        if self.active_scan_executor is None:
            feedback = {
                "operation": SCAN_FRONT_ARC,
                "status": "unavailable",
                "reason": "physical_active_scan_not_configured",
                "host_selected_route_or_side": False,
            }
            self._emit("scan_unavailable", scan=feedback)
            return observation, feedback
        now_ms = self.unix_ms()
        now_monotonic = self.monotonic()
        now_monotonic_ms = int(now_monotonic * 1000)
        scan_window_ms = min(
            int(self.config.scan_timeout_seconds * 1000),
            max(0, int((deadline - now_monotonic) * 1000)),
        )
        request = build_scan_request(
            choice=ModelScanChoice(target_hypothesis_id),
            frame_id=self.memory.frame_id,
            map_generation_id=self.memory.generation_id,
            map_version=self.memory.hazard_map.revision,
            start_pose=self.memory.pose,
            start_state_version=observation["state_version"],
            created_at_ms=now_ms,
            deadline_ms=now_ms + scan_window_ms,
            created_monotonic_ms=now_monotonic_ms,
            deadline_monotonic_ms=now_monotonic_ms + scan_window_ms,
            calibration=calibration,
        )
        rotation_offsets = tuple(sorted(set(
            (0,) + (
                calibration.coarse_offsets_mdeg
                if probe_offsets_mdeg is None
                else probe_offsets_mdeg
            )
        )))
        rotation_sweep = self.memory.hazard_map.validate_in_place_rotation(
            self.memory.pose,
            rotation_offsets,
            alignment_tolerance_mdeg=(
                calibration.alignment_tolerance_mdeg
            ),
        )
        if not rotation_sweep["allowed"]:
            self._arm_scan_progress_barrier(
                scan_id=request.scan_id,
                target_hypothesis_id=target_hypothesis_id,
                observation=observation,
            )
            feedback = {
                "operation": SCAN_FRONT_ARC,
                "status": "DENIED",
                "reason": rotation_sweep["reason"],
                "target_hypothesis_id": target_hypothesis_id,
                "rotation_sweep": rotation_sweep,
                "host_selected_route_or_side": False,
                "next_scan_eligibility": {
                    "eligible": False,
                    "reason": "INTERVENING_NAVIGATION_PROGRESS_REQUIRED",
                },
            }
            self._emit("scan_denied", scan=deepcopy(feedback))
            return observation, feedback
        try:
            if probe_offsets_mdeg is None:
                result = self.active_scan_executor.execute(
                    request,
                    cancel_requested=self._cancelled,
                )
            else:
                result = self.active_scan_executor.execute_side_probe(
                    request,
                    probe_offsets_mdeg,
                    cancel_requested=self._cancelled,
                )
        except Exception:
            self._invalidate_localization(
                "Active scan failed before heading restoration was verified",
                publication_stage="scan_execution_invalidated",
            )
            raise
        side_probe_complete = None
        try:
            validate_scan_result(
                result,
                request,
                current_frame_id=self.memory.frame_id,
                current_map_generation_id=self.memory.generation_id,
                current_map_version=self.memory.hazard_map.revision,
            )
            if probe_offsets_mdeg is not None:
                side_probe_complete = _validate_side_probe_result(
                    result,
                    probe_offsets_mdeg,
                )
        except Exception:
            self._invalidate_localization(
                "Active scan result could not be validated",
                publication_stage="scan_validation_invalidated",
            )
            raise
        if not result.restored_start_heading or not result.stop_confirmed:
            # A validated cancelled result still contains the exact scan
            # reason and every verified ray. Publish it before escalating an
            # unknown heading/stop state so dashboard evidence survives the
            # terminal fault transition.
            self._emit("scan_result", scan=result.to_dict())
            self._invalidate_localization(
                "Active scan did not verify restoration to its start heading",
                publication_stage="scan_restoration_invalidated",
            )
            self._raise_if_cancelled("scan_cancelled_without_restoration")
            raise PhysicalNavigationRuntimeError(
                "scan_heading_unrestored",
                "Active scan ended without verified heading restoration",
            )
        if self._cancelled():
            self._invalidate_localization(
                "Scan completed but cancellation prevented encoder re-anchoring",
                publication_stage="scan_reanchor_cancelled",
            )
            self._raise_if_cancelled("after_scan_before_encoder_reanchor")
        try:
            self._raise_if_cancelled(
                "immediately_before_post_scan_observe"
            )
            response = self._active_request(
                "observe",
                {},
                self.config.request_timeout_seconds,
            )
            fresh = self._observation_from_response("observe", response)
            captured_at_ms = self.unix_ms()
            self.memory.ingest_verified_scan_completion(
                fresh,
                result,
                captured_at_ms,
            )
        except EpisodeCancelled:
            self._invalidate_localization(
                "Cancellation interrupted post-scan encoder re-anchoring",
                publication_stage="post_scan_observe_cancelled",
            )
            raise
        except Exception:
            if self.memory.localization_valid:
                self._invalidate_localization(
                    "Post-scan encoder re-anchoring failed",
                    publication_stage="post_scan_reanchor_invalidated",
                )
            else:
                self._offer_invalid_localization(
                    captured_at_ms=captured_at_ms,
                    publication_stage="post_scan_reanchor_invalidated",
                    observation=fresh,
                )
            raise
        try:
            updated_hazard = self.memory.hazard_map.record_scan_result(
                result,
                scan_pose=request.start_pose,
            )
        except ValueError as error:
            # Physical stop, heading restoration, and the mandatory fresh
            # encoder anchor were already verified above. A rejected map
            # fusion is therefore logical discarded evidence, never a
            # physical fault or a reason to invalidate localization.
            reason = (
                "scan_boundary_map_integration_rejected"
                if result.bilateral_complete
                else "scan_evidence_map_integration_rejected"
            )
            self._arm_restored_scan_progress_barrier(result, fresh)
            feedback = {
                "operation": SCAN_FRONT_ARC,
                "status": "CANCELLED",
                "reason": reason,
                "target_hypothesis_id": result.target_hypothesis_id,
                "bilateral_complete": False,
                "evidence_disposition": "DISCARDED",
                "integration_error": type(error).__name__,
                "scan": result.to_dict(),
                "next_scan_eligibility": {
                    "eligible": False,
                    "reason": "INTERVENING_NAVIGATION_PROGRESS_REQUIRED",
                },
            }
            if probe_offsets_mdeg is not None:
                feedback["side_probe_complete"] = side_probe_complete
            self._offer_observation(
                fresh,
                captured_at_ms=captured_at_ms,
                publication_stage="scan_reanchor",
            )
            self._emit(
                "scan_result",
                scan=result.to_dict(),
                evidence_disposition="DISCARDED",
                map_integration={
                    "status": "rejected",
                    "reason": feedback["reason"],
                },
            )
            return fresh, feedback
        self.memory.updated_at_ms = max(
            self.memory.updated_at_ms,
            result.completed_at_ms,
        )
        self.memory.save()
        self._arm_restored_scan_progress_barrier(result, fresh)
        scan_evidence = updated_hazard.scan_evidence_history[-1].to_dict()
        self._offer_observation(
            fresh,
            captured_at_ms=captured_at_ms,
            publication_stage="scan_reanchor",
        )
        feedback = {
            "operation": SCAN_FRONT_ARC,
            "status": result.status,
            "reason": result.reason,
            "target_hypothesis_id": result.target_hypothesis_id,
            "bilateral_complete": result.bilateral_complete,
            "evidence_disposition": "MAP_INTEGRATED",
            "scan_evidence": scan_evidence,
            "next_scan_eligibility": {
                "eligible": False,
                "reason": "INTERVENING_NAVIGATION_PROGRESS_REQUIRED",
            },
            "scan": result.to_dict(),
        }
        if probe_offsets_mdeg is not None:
            feedback["side_probe_complete"] = side_probe_complete
        self._emit(
            "scan_result",
            scan=result.to_dict(),
            evidence_disposition="MAP_INTEGRATED",
            scan_evidence=scan_evidence,
        )
        return fresh, feedback

    def _execute_pass_side_probe(
        self,
        *,
        observation: Mapping[str, object],
        deadline: float,
        target_hypothesis_id: str,
        probe_offsets_mdeg: Tuple[int, ...],
    ) -> Tuple[Mapping[str, object], Mapping[str, object]]:
        """Collect restored one-sided evidence for the committed obstacle."""

        target_before = self.memory.hazard_map.get(target_hypothesis_id)
        support_keys_before = (
            frozenset()
            if target_before is None
            else frozenset(
                support.spatial_key
                for support in target_before.collision_supports
            )
        )
        fresh, scan_feedback = self._execute_target_scan(
            target_hypothesis_id=target_hypothesis_id,
            calibration=self.active_scan_calibration,
            observation=observation,
            deadline=deadline,
            probe_offsets_mdeg=probe_offsets_mdeg,
        )
        target_after = self.memory.hazard_map.get(target_hypothesis_id)
        support_keys_after = (
            frozenset()
            if target_after is None
            else frozenset(
                support.spatial_key
                for support in target_after.collision_supports
            )
        )
        scan = scan_feedback.get("scan")
        rays = (
            scan.get("rays", ())
            if isinstance(scan, Mapping)
            else ()
        )
        target_confirmed = any(
            isinstance(ray, Mapping) and ray.get("blocked") is True
            for ray in rays
        )
        new_support_count = len(
            support_keys_after - support_keys_before
        )
        integrated = (
            scan_feedback.get("evidence_disposition")
            == "MAP_INTEGRATED"
        )
        if not integrated:
            status = "failed"
            reason = scan_feedback.get(
                "reason", "SIDE_PROBE_NOT_INTEGRATED"
            )
        elif scan_feedback.get("side_probe_complete") is not True:
            status = "failed"
            reason = "SIDE_PROBE_INCOMPLETE"
        elif not target_confirmed:
            status = "clear"
            reason = "SIDE_PROBE_TARGET_NOT_CONFIRMED"
        elif new_support_count == 0:
            status = "no_spatial_progress"
            reason = "SIDE_PROBE_NO_SPATIAL_PROGRESS"
        else:
            status = "confirmed"
            reason = "SIDE_PROBE_TARGET_CONFIRMED"
        feedback = deepcopy(dict(scan_feedback))
        feedback.update({
            "operation": "pass_side_probe",
            "scan_status": scan_feedback.get("status"),
            "status": status,
            "reason": reason,
            "probe_offsets_mdeg": list(probe_offsets_mdeg),
            "target_confirmed": target_confirmed,
            "new_spatial_support": new_support_count > 0,
            "new_support_count": new_support_count,
            "host_selected_route_or_side": False,
        })
        self._emit(
            "pass_side_probe_result",
            probe=deepcopy(feedback),
        )
        return fresh, feedback

    def _scan_budget_allows(
        self,
        observation: Mapping[str, object],
        action_specs: Mapping[str, Mapping[str, object]],
        deadline: float,
    ) -> bool:
        budgets = observation["budgets"]
        scan_headroom_ms = self._session_renewal_headroom_ms(
            action_specs,
            include_scan=True,
        )
        return (
            self.active_scan_executor is not None
            and budgets["pulse_count_remaining"]
            >= DEFAULT_SCAN_BUDGET["turn_slice_count"]
            and budgets["pulse_duration_ms_remaining"]
            >= DEFAULT_SCAN_BUDGET["turn_duration_ms"]
            and self._effective_worker_process_ms(observation)
            >= scan_headroom_ms
            and self._worker_absolute_max_ms is not None
            and scan_headroom_ms <= self._worker_absolute_max_ms
            and self._remaining_seconds(deadline) * 1000
            >= scan_headroom_ms
        )
