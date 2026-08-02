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

    def __init__(
        self,
        *,
        rig,
        clock_ms,
        restoration_headroom_ms: int = 0,
    ):
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
        if (
            isinstance(restoration_headroom_ms, bool)
            or not isinstance(restoration_headroom_ms, int)
            or not 0 <= restoration_headroom_ms <= 60_000
        ):
            raise ActiveIrScanContractError(
                "invalid_scan_restoration_headroom",
                "Active scan restoration headroom is invalid",
            )
        self.rig = rig
        self.clock_ms = clock_ms
        self.restoration_headroom_ms = restoration_headroom_ms

    @staticmethod
    def _evidence_time_ms(
        request: ActiveIrScanRequest,
        monotonic_ms: int,
    ) -> int:
        """Anchor evidence time to scan start without trusting wall-clock jumps."""

        return request.created_at_ms + max(
            0,
            monotonic_ms - request.created_monotonic_ms,
        )

    @staticmethod
    def _coarse_schedule(offsets: Sequence[int]) -> Tuple[int, ...]:
        negative = sorted((item for item in offsets if item < 0), reverse=True)
        positive = sorted(item for item in offsets if item > 0)
        return tuple(([0] if 0 in offsets else []) + negative + positive)

    @staticmethod
    def _transition_midpoints(
        rays: Sequence[ActiveIrRay],
        fine_step_mdeg: int,
        *,
        current_offset_mdeg: int,
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
        ascending = tuple(sorted(candidates))
        if not ascending:
            return ()
        descending = tuple(reversed(ascending))

        def restoring_travel(schedule: Sequence[int]) -> int:
            previous = current_offset_mdeg
            distance = 0
            for target in schedule:
                distance += abs(target - previous)
                previous = target
            return distance + abs(previous)

        # Bearings lie on one axis, so an optimal route is one of the two
        # monotonic sweeps. Include the mandatory restoration to zero when
        # choosing between them and prefer the nearer first ray on a tie.
        return min(
            (ascending, descending),
            key=lambda schedule: (
                restoring_travel(schedule),
                abs(schedule[0] - current_offset_mdeg),
                schedule,
            ),
        )

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
        *,
        admission_deadline_ms: int,
        operation_deadline_ms: int,
        reserve_restoration: bool,
    ) -> int:
        now = int(self.clock_ms())
        required_ms = self._remaining_turn_ms(
            current_offset_mdeg,
            target_offset_mdeg,
            request,
        )
        if not reserve_restoration:
            restore_degrees = abs(target_offset_mdeg) / 1000.0
            required_ms -= int(
                round(
                    restore_degrees
                    * request.calibration.estimated_turn_ms_per_degree
                    + request.calibration.settle_ms
                )
            )
        if (
            now + max(0, required_ms) >= admission_deadline_ms
        ):
            raise ActiveIrScanContractError(
                "scan_deadline_reserve",
                "Scan stopped before an unrestorable turn",
            )
        delta = target_offset_mdeg - current_offset_mdeg
        if delta == 0:
            return current_offset_mdeg
        try:
            receipt = self.rig.turn_relative_mdeg(
                delta,
                request.calibration,
                operation_deadline_ms,
            )
        except ActiveIrScanContractError as error:
            verified_delta = getattr(
                error,
                "verified_actual_delta_mdeg",
                None,
            )
            if (
                not isinstance(verified_delta, bool)
                and isinstance(verified_delta, int)
                and abs(
                    current_offset_mdeg
                    + verified_delta
                    - target_offset_mdeg
                )
                <= request.calibration.alignment_tolerance_mdeg
            ):
                error.verified_scan_offset_mdeg = target_offset_mdeg
            raise
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
            or receipt["completed_at_ms"] > operation_deadline_ms
            or int(self.clock_ms()) > operation_deadline_ms
        ):
            raise ActiveIrScanContractError(
                "invalid_or_late_scan_turn",
                "Active scan turn was invalid or late",
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
        deadline_ms: int,
    ) -> ActiveIrRay:
        # The worker request is bounded by the hard request deadline. The
        # caller's smaller deadline is a soft admission deadline: a verified
        # late sample is discarded, but the channel remains available for
        # bounded stop and heading restoration.
        snapshot = self._snapshot_fields(
            self.rig.read_snapshot(request.deadline_monotonic_ms)
        )
        now_monotonic_ms = int(self.clock_ms())
        now_evidence_ms = self._evidence_time_ms(
            request,
            now_monotonic_ms,
        )
        if now_monotonic_ms > deadline_ms:
            raise ActiveIrScanContractError(
                "scan_deadline_exceeded",
                "Active scan work exceeded its soft deadline",
            )
        if (
            snapshot["observed_at_ms"] <= previous_time_ms
            or snapshot["state_version"] <= previous_state_version
            or snapshot["observed_at_ms"] > now_evidence_ms
            or now_evidence_ms - snapshot["observed_at_ms"]
            > request.max_snapshot_age_ms
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

        started_monotonic_ms = int(self.clock_ms())
        started = self._evidence_time_ms(request, started_monotonic_ms)
        scan_deadline_ms = (
            request.deadline_monotonic_ms
            - self.restoration_headroom_ms
        )
        if scan_deadline_ms <= request.created_monotonic_ms:
            raise ActiveIrScanContractError(
                "invalid_scan_restoration_headroom",
                "Active scan has no time left before restoration reserve",
            )
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
                    admission_deadline_ms=scan_deadline_ms,
                    operation_deadline_ms=request.deadline_monotonic_ms,
                    reserve_restoration=True,
                )
                if int(self.clock_ms()) > scan_deadline_ms:
                    raise ActiveIrScanContractError(
                        "scan_deadline_exceeded",
                        "Active scan work exceeded its soft deadline",
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
                    deadline_ms=scan_deadline_ms,
                )
                rays.append(ray)

            for requested in self._transition_midpoints(
                rays,
                request.calibration.fine_step_mdeg,
                current_offset_mdeg=current_offset,
            ):
                require_active()
                current_offset = self._turn(
                    current_offset,
                    requested,
                    request,
                    admission_deadline_ms=scan_deadline_ms,
                    operation_deadline_ms=request.deadline_monotonic_ms,
                    reserve_restoration=True,
                )
                if int(self.clock_ms()) > scan_deadline_ms:
                    raise ActiveIrScanContractError(
                        "scan_deadline_exceeded",
                        "Active scan work exceeded its soft deadline",
                    )
                require_active()
                ray = self._read_ray(
                    request=request,
                    requested_offset_mdeg=requested,
                    ordinal=len(rays) + 1,
                    previous_time_ms=rays[-1].observed_at_ms,
                    previous_state_version=rays[-1].state_version,
                    deadline_ms=scan_deadline_ms,
                )
                rays.append(ray)
        except ActiveIrScanContractError as error:
            status = "CANCELLED"
            reason = getattr(error, "result_reason", error.code)
            verified_offset = getattr(
                error,
                "verified_scan_offset_mdeg",
                None,
            )
            if (
                not isinstance(verified_offset, bool)
                and isinstance(verified_offset, int)
            ):
                current_offset = verified_offset
            safety_cancelled = error.code in (
                "scan_touch_cancelled",
                "scan_motion_fault",
                "scan_external_cancelled",
                "scan_cancellation_probe_failed",
            ) or getattr(error, "restoration_prohibited", False) is True
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
                # If the active operation already proved that the transport
                # was lost, the same poisoned channel cannot provide an
                # independent stop receipt. Preserve that primary diagnosis
                # instead of replacing it with the less useful cleanup label.
                if reason != "scan_transport_failed":
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
                    admission_deadline_ms=request.deadline_monotonic_ms,
                    operation_deadline_ms=request.deadline_monotonic_ms,
                    reserve_restoration=False,
                )
            except ActiveIrScanContractError as error:
                status = "CANCELLED"
                reason = getattr(error, "result_reason", error.code)
        if not safety_cancelled and current_offset == 0:
            try:
                require_active()
                final = self._snapshot_fields(
                    self.rig.read_snapshot(request.deadline_monotonic_ms)
                )
                now_monotonic_ms = int(self.clock_ms())
                now_evidence_ms = self._evidence_time_ms(
                    request,
                    now_monotonic_ms,
                )
                final_offset = relative_heading_mdeg(
                    final["pose_heading_mdeg"],
                    request.start_pose.heading_mdeg,
                )
                restored = (
                    now_monotonic_ms <= request.deadline_monotonic_ms
                    and final["observed_at_ms"]
                    <= request.deadline_ms
                    and now_evidence_ms - final["observed_at_ms"]
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
        completed_monotonic_ms = int(self.clock_ms())
        completed = self._evidence_time_ms(
            request,
            completed_monotonic_ms,
        )
        if completed_monotonic_ms > request.deadline_monotonic_ms:
            status = "CANCELLED"
            reason = "scan_deadline_exceeded"
            # A final stationary sample is the physical completion proof.
            # Local result assembly crossing the deadline must not erase it.
            if restored:
                completed = final["observed_at_ms"]
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
            # A restored scan can still contribute one-sided boundary
            # evidence even when it cannot prove a complete bilateral
            # obstacle envelope.  Keep every boundary derived from the
            # sampled rays; ``bilateral_complete`` remains the stricter gate
            # for committing to a detour side.
            left_boundary_mdeg=left,
            right_boundary_mdeg=right,
        )
        release_scan = getattr(self.rig, "release_scan", None)
        if callable(release_scan):
            release_scan()
        return result
