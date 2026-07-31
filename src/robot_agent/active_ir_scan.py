"""Deterministic coarse-to-fine execution of a model-selected IR scan."""

from typing import Mapping, Optional, Sequence, Tuple

from .active_ir_scan_contract import (
    ActiveIrRay,
    ActiveIrScanContractError,
    ActiveIrScanRequest,
    ActiveIrScanResult,
    relative_heading_mdeg,
)


class ActiveIrScanExecutor:
    """Sample a bounded arc without making a navigation-side decision."""

    def __init__(self, *, rig, clock_ms):
        required = ("turn_relative_mdeg", "read_snapshot", "stop")
        if any(not callable(getattr(rig, name, None)) for name in required):
            raise ActiveIrScanContractError(
                "invalid_scan_rig",
                "Active scan rig does not implement the required operations",
            )
        if not callable(clock_ms):
            raise ActiveIrScanContractError(
                "invalid_scan_clock",
                "Active scan clock is invalid",
            )
        self.rig = rig
        self.clock_ms = clock_ms

    @staticmethod
    def _coarse_schedule(offsets: Sequence[int]) -> Tuple[int, ...]:
        negative = sorted((item for item in offsets if item < 0), reverse=True)
        positive = sorted(item for item in offsets if item > 0)
        return tuple(([0] if 0 in offsets else []) + negative + positive)

    @staticmethod
    def _transition_midpoints(
        rays: Sequence[ActiveIrRay],
        fine_step_mdeg: int,
    ) -> Tuple[int, ...]:
        by_bearing = {
            item.requested_relative_bearing_mdeg: item for item in rays
        }
        offsets = sorted(by_bearing)
        candidates = set()
        for left, right in zip(offsets, offsets[1:]):
            if by_bearing[left].blocked == by_bearing[right].blocked:
                continue
            if right - left <= fine_step_mdeg:
                continue
            midpoint = int(round((left + right) / 2.0))
            if midpoint not in by_bearing:
                candidates.add(midpoint)
        return tuple(sorted(candidates))

    @staticmethod
    def _boundaries(
        rays: Sequence[ActiveIrRay],
    ) -> Tuple[Optional[int], Optional[int]]:
        by_bearing = {
            item.requested_relative_bearing_mdeg: item for item in rays
        }
        offsets = sorted(by_bearing)
        boundaries = []
        for first, second in zip(offsets, offsets[1:]):
            if by_bearing[first].blocked != by_bearing[second].blocked:
                boundaries.append(int(round((first + second) / 2.0)))
        left = min((item for item in boundaries if item > 0), default=None)
        right = max((item for item in boundaries if item < 0), default=None)
        return left, right

    @staticmethod
    def _snapshot_fields(snapshot: object) -> Mapping[str, object]:
        fields = {
            "state_version",
            "observed_at_ms",
            "pose_heading_mdeg",
            "touch_pressed",
            "motion_fault_latched",
            "infrared",
        }
        if not isinstance(snapshot, dict) or set(snapshot) != fields:
            raise ActiveIrScanContractError(
                "invalid_scan_snapshot",
                "Active scan snapshot fields are invalid",
            )
        if (
            isinstance(snapshot["state_version"], bool)
            or not isinstance(snapshot["state_version"], int)
            or snapshot["state_version"] <= 0
            or isinstance(snapshot["observed_at_ms"], bool)
            or not isinstance(snapshot["observed_at_ms"], int)
            or snapshot["observed_at_ms"] < 0
            or isinstance(snapshot["pose_heading_mdeg"], bool)
            or not isinstance(snapshot["pose_heading_mdeg"], int)
            or type(snapshot["touch_pressed"]) is not bool
            or type(snapshot["motion_fault_latched"]) is not bool
        ):
            raise ActiveIrScanContractError(
                "invalid_scan_snapshot_value",
                "Active scan snapshot values are invalid",
            )
        infrared = snapshot["infrared"]
        if (
            not isinstance(infrared, dict)
            or set(infrared) != {"raw", "filtered", "blocked"}
            or type(infrared["blocked"]) is not bool
        ):
            raise ActiveIrScanContractError(
                "invalid_scan_snapshot_ir",
                "Active scan IR snapshot is invalid",
            )
        return snapshot

    def _remaining_turn_ms(
        self,
        current_offset_mdeg: int,
        target_offset_mdeg: int,
        request: ActiveIrScanRequest,
    ) -> int:
        calibration = request.calibration
        requested_degrees = (
            abs(target_offset_mdeg - current_offset_mdeg) / 1000.0
        )
        restore_degrees = abs(target_offset_mdeg) / 1000.0
        return int(
            round(
                (requested_degrees + restore_degrees)
                * calibration.estimated_turn_ms_per_degree
                + calibration.settle_ms * 2
            )
        )

    def _turn(
        self,
        current_offset_mdeg: int,
        target_offset_mdeg: int,
        request: ActiveIrScanRequest,
    ) -> int:
        now = int(self.clock_ms())
        if (
            now
            + self._remaining_turn_ms(
                current_offset_mdeg,
                target_offset_mdeg,
                request,
            )
            >= request.deadline_ms
        ):
            raise ActiveIrScanContractError(
                "scan_deadline_reserve",
                "Scan stopped before an unrestorable turn",
            )
        delta = target_offset_mdeg - current_offset_mdeg
        if delta == 0:
            return current_offset_mdeg
        receipt = self.rig.turn_relative_mdeg(
            delta,
            request.calibration,
            request.deadline_ms,
        )
        if (
            not isinstance(receipt, dict)
            or set(receipt)
            != {
                "requested_delta_mdeg",
                "actual_delta_mdeg",
                "completed_at_ms",
                "stop_confirmed",
            }
            or receipt["requested_delta_mdeg"] != delta
            or receipt["stop_confirmed"] is not True
            or isinstance(receipt["actual_delta_mdeg"], bool)
            or not isinstance(receipt["actual_delta_mdeg"], int)
            or isinstance(receipt["completed_at_ms"], bool)
            or not isinstance(receipt["completed_at_ms"], int)
            or receipt["completed_at_ms"] > request.deadline_ms
            or int(self.clock_ms()) > request.deadline_ms
        ):
            raise ActiveIrScanContractError(
                "invalid_or_late_scan_turn",
                "Active scan turn was invalid or late",
            )
        derived_offset = (
            current_offset_mdeg + receipt["actual_delta_mdeg"]
        )
        if (
            abs(derived_offset - target_offset_mdeg)
            > request.calibration.alignment_tolerance_mdeg
        ):
            raise ActiveIrScanContractError(
                "scan_pose_misaligned",
                "Active scan turn did not reach the requested bearing",
            )
        # Keep the command lattice nominal.  The rig and the immediately
        # following snapshot carry the encoder-derived physical heading.
        # Feeding small encoder residuals into the next command would turn a
        # fixed +/-15-degree host profile into arbitrary raw motor deltas.
        return target_offset_mdeg

    def _read_ray(
        self,
        *,
        request: ActiveIrScanRequest,
        requested_offset_mdeg: int,
        ordinal: int,
        previous_time_ms: int,
        previous_state_version: int,
    ) -> ActiveIrRay:
        snapshot = self._snapshot_fields(self.rig.read_snapshot())
        now = int(self.clock_ms())
        if (
            snapshot["observed_at_ms"] <= previous_time_ms
            or snapshot["state_version"] <= previous_state_version
            or snapshot["observed_at_ms"] > now
            or now - snapshot["observed_at_ms"]
            > request.max_snapshot_age_ms
            or now > request.deadline_ms
        ):
            raise ActiveIrScanContractError(
                "stale_scan_snapshot",
                "Active scan snapshot is not fresh and sequential",
            )
        if snapshot["touch_pressed"]:
            raise ActiveIrScanContractError(
                "scan_touch_cancelled",
                "Touch cancelled active scanning",
            )
        if snapshot["motion_fault_latched"]:
            raise ActiveIrScanContractError(
                "scan_motion_fault",
                "Motion fault cancelled active scanning",
            )
        actual_offset = relative_heading_mdeg(
            snapshot["pose_heading_mdeg"],
            request.start_pose.heading_mdeg,
        )
        if (
            abs(actual_offset - requested_offset_mdeg)
            > request.calibration.alignment_tolerance_mdeg
        ):
            raise ActiveIrScanContractError(
                "scan_snapshot_pose_misaligned",
                "Snapshot pose did not match the sampled bearing",
            )
        infrared = snapshot["infrared"]
        return ActiveIrRay(
            ordinal=ordinal,
            requested_relative_bearing_mdeg=requested_offset_mdeg,
            actual_relative_bearing_mdeg=actual_offset,
            observed_at_ms=snapshot["observed_at_ms"],
            state_version=snapshot["state_version"],
            raw=infrared["raw"],
            filtered=infrared["filtered"],
            blocked=infrared["blocked"],
        )

    def execute(
        self,
        request: ActiveIrScanRequest,
        cancel_requested=lambda: False,
    ) -> ActiveIrScanResult:
        if not callable(cancel_requested):
            raise ActiveIrScanContractError(
                "invalid_scan_cancellation_probe",
                "Active scan cancellation probe is invalid",
            )

        def require_active():
            try:
                cancelled = cancel_requested() is True
            except Exception:
                raise ActiveIrScanContractError(
                    "scan_cancellation_probe_failed",
                    "Active scan cancellation probe failed",
                ) from None
            if cancelled:
                raise ActiveIrScanContractError(
                    "scan_external_cancelled",
                    "External cancellation stopped active scanning",
                )

        started = int(self.clock_ms())
        rays = []
        current_offset = 0
        reason = "bilateral_boundaries_observed"
        status = "COMPLETED"
        stop_confirmed = False
        restored = False
        safety_cancelled = False
        try:
            require_active()
            bind_cancel_requested = getattr(
                self.rig,
                "bind_cancel_requested",
                None,
            )
            if callable(bind_cancel_requested):
                bind_cancel_requested(cancel_requested)
            begin_scan = getattr(self.rig, "begin_scan", None)
            if callable(begin_scan):
                begin_scan(request)
            schedule = self._coarse_schedule(
                request.calibration.coarse_offsets_mdeg
            )
            for requested in schedule:
                require_active()
                current_offset = self._turn(
                    current_offset,
                    requested,
                    request,
                )
                require_active()
                ray = self._read_ray(
                    request=request,
                    requested_offset_mdeg=requested,
                    ordinal=len(rays) + 1,
                    previous_time_ms=(
                        request.created_at_ms
                        if not rays
                        else rays[-1].observed_at_ms
                    ),
                    previous_state_version=(
                        request.start_state_version
                        if not rays
                        else rays[-1].state_version
                    ),
                )
                rays.append(ray)

            for requested in self._transition_midpoints(
                rays,
                request.calibration.fine_step_mdeg,
            ):
                require_active()
                current_offset = self._turn(
                    current_offset,
                    requested,
                    request,
                )
                require_active()
                ray = self._read_ray(
                    request=request,
                    requested_offset_mdeg=requested,
                    ordinal=len(rays) + 1,
                    previous_time_ms=rays[-1].observed_at_ms,
                    previous_state_version=rays[-1].state_version,
                )
                rays.append(ray)
        except ActiveIrScanContractError as error:
            status = "CANCELLED"
            reason = error.code
            safety_cancelled = error.code in (
                "scan_touch_cancelled",
                "scan_motion_fault",
                "scan_external_cancelled",
                "scan_cancellation_probe_failed",
            )
        finally:
            try:
                stop_receipt = self.rig.stop()
                stop_confirmed = (
                    isinstance(stop_receipt, dict)
                    and set(stop_receipt) == {"stop_confirmed"}
                    and stop_receipt["stop_confirmed"] is True
                )
                if not stop_confirmed:
                    raise ActiveIrScanContractError(
                        "scan_stop_failed",
                        "Active scan stop was not confirmed",
                    )
            except Exception:
                status = "CANCELLED"
                reason = "scan_stop_failed"
                safety_cancelled = True

        try:
            require_active()
        except ActiveIrScanContractError as error:
            status = "CANCELLED"
            reason = error.code
            safety_cancelled = True

        if not safety_cancelled and current_offset != 0:
            try:
                current_offset = self._turn(
                    current_offset,
                    0,
                    request,
                )
            except ActiveIrScanContractError as error:
                status = "CANCELLED"
                reason = error.code
        if not safety_cancelled and current_offset == 0:
            try:
                require_active()
                final = self._snapshot_fields(self.rig.read_snapshot())
                now = int(self.clock_ms())
                final_offset = relative_heading_mdeg(
                    final["pose_heading_mdeg"],
                    request.start_pose.heading_mdeg,
                )
                restored = (
                    now <= request.deadline_ms
                    and final["observed_at_ms"]
                    <= request.deadline_ms
                    and now - final["observed_at_ms"]
                    <= request.max_snapshot_age_ms
                    and abs(final_offset)
                    <= request.calibration.alignment_tolerance_mdeg
                    and final["touch_pressed"] is False
                    and final["motion_fault_latched"] is False
                )
            except ActiveIrScanContractError:
                restored = False
            if not restored:
                status = "CANCELLED"
                reason = "scan_heading_restoration_unverified"

        left, right = self._boundaries(rays)
        if status == "COMPLETED" and (
            left is None or right is None or not restored
        ):
            status = "CANCELLED"
            reason = "bilateral_boundaries_not_observed"
        completed = int(self.clock_ms())
        if completed > request.deadline_ms:
            status = "CANCELLED"
            reason = "scan_deadline_exceeded"
            restored = False
        result = ActiveIrScanResult(
            scan_id=request.scan_id,
            target_hypothesis_id=request.target_hypothesis_id,
            frame_id=request.frame_id,
            map_generation_id=request.map_generation_id,
            based_on_map_version=request.based_on_map_version,
            started_at_ms=started,
            completed_at_ms=completed,
            status=status,
            reason=reason,
            stop_confirmed=stop_confirmed,
            restored_start_heading=restored,
            rays=tuple(rays),
            left_boundary_mdeg=left if status == "COMPLETED" else None,
            right_boundary_mdeg=right if status == "COMPLETED" else None,
        )
        release_scan = getattr(self.rig, "release_scan", None)
        if callable(release_scan):
            release_scan()
        return result
