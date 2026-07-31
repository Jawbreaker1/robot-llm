"""Trust boundary for model-selected, host-sampled active EV3 IR scans."""

from dataclasses import dataclass
import hashlib
from typing import Mapping, Optional, Tuple

from .physical_odometry import PhysicalPose, normalize_heading_mdeg
from .physical_navigation_contract import (
    SCAN_SAMPLE_COUNT,
    SCAN_SAMPLE_SETTLED_DURATION_MS,
    expected_scan_turn_spec,
)


TOOL_NAME = "SCAN_FRONT_ARC"
ADAPTIVE_COARSE_TO_FINE = "ADAPTIVE_COARSE_TO_FINE"
SCAN_RESULT_SCHEMA = "robot-active-ir-scan-result/v1"
PROVISIONAL_CALIBRATION = "PROVISIONAL_UNVERIFIED_EV3_BODY_TURN"
SCAN_REQUEST_ROUND_TRIP_HEADROOM_MS = 250
SCAN_FIXED_DEADLINE_HEADROOM_MS = 1_000


class ActiveIrScanContractError(ValueError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


@dataclass(frozen=True)
class ActiveIrScanCalibration:
    coarse_offsets_mdeg: Tuple[int, ...] = (
        -60_000,
        -30_000,
        0,
        30_000,
        60_000,
    )
    fine_step_mdeg: int = 15_000
    alignment_tolerance_mdeg: int = 2_500
    estimated_turn_ms_per_degree: int = 30
    settle_ms: int = 120
    provenance: str = PROVISIONAL_CALIBRATION

    def __post_init__(self) -> None:
        if (
            tuple(sorted(set(self.coarse_offsets_mdeg)))
            != self.coarse_offsets_mdeg
            or 0 not in self.coarse_offsets_mdeg
            or min(self.coarse_offsets_mdeg) >= 0
            or max(self.coarse_offsets_mdeg) <= 0
            or any(
                isinstance(value, bool)
                or not isinstance(value, int)
                or not -90_000 <= value <= 90_000
                for value in self.coarse_offsets_mdeg
            )
            or isinstance(self.fine_step_mdeg, bool)
            or not 1_000 <= self.fine_step_mdeg <= 30_000
            or isinstance(self.alignment_tolerance_mdeg, bool)
            or not 0 <= self.alignment_tolerance_mdeg <= 10_000
            or isinstance(self.estimated_turn_ms_per_degree, bool)
            or not 1 <= self.estimated_turn_ms_per_degree <= 1_000
            or isinstance(self.settle_ms, bool)
            or not 0 <= self.settle_ms <= 5_000
            or self.provenance != PROVISIONAL_CALIBRATION
        ):
            raise ActiveIrScanContractError(
                "invalid_scan_calibration",
                "Active IR scan calibration is invalid",
            )


def worst_case_scan_budget(
    calibration: ActiveIrScanCalibration = ActiveIrScanCalibration(),
    *,
    request_round_trip_headroom_ms: int = (
        SCAN_REQUEST_ROUND_TRIP_HEADROOM_MS
    ),
) -> Mapping[str, int]:
    """Return the fixed coarse plus maximum-refinement EV3 scan budget."""
    if (
        not isinstance(calibration, ActiveIrScanCalibration)
        or isinstance(request_round_trip_headroom_ms, bool)
        or not isinstance(request_round_trip_headroom_ms, int)
        or not 1 <= request_round_trip_headroom_ms <= 10_000
    ):
        raise ActiveIrScanContractError(
            "invalid_scan_calibration",
            "Active IR scan calibration is invalid",
        )
    offsets = calibration.coarse_offsets_mdeg
    negative = sorted((item for item in offsets if item < 0), reverse=True)
    positive = sorted(item for item in offsets if item > 0)
    coarse_targets = ([0] if 0 in offsets else []) + negative + positive
    sorted_offsets = sorted(offsets)
    fine_targets = []
    for left, right in zip(sorted_offsets, sorted_offsets[1:]):
        if right - left <= calibration.fine_step_mdeg:
            continue
        midpoint = int(round((left + right) / 2.0))
        if midpoint not in offsets:
            fine_targets.append(midpoint)
    targets = coarse_targets + sorted(set(fine_targets)) + [0]
    current = 0
    turn_deltas = []
    for target in targets:
        delta = target - current
        if delta:
            turn_deltas.append(delta)
        current = target
    try:
        turn_specs = [
            expected_scan_turn_spec(delta) for delta in turn_deltas
        ]
    except Exception as error:
        raise ActiveIrScanContractError(
            "unsupported_scan_calibration_lattice",
            "Scan calibration cannot be represented by the EV3 profile",
        ) from error
    ray_count = len(coarse_targets) + len(set(fine_targets))
    stationary_sample_batches = ray_count + 1
    worker_request_count = (
        len(turn_specs) + stationary_sample_batches + 1
    )
    turn_duration_ms = sum(
        item["total_duration_ms"] for item in turn_specs
    )
    turn_slice_count = sum(item["slice_count"] for item in turn_specs)
    stationary_sampling_ms = (
        stationary_sample_batches * SCAN_SAMPLE_SETTLED_DURATION_MS
    )
    minimum_deadline_ms = (
        turn_duration_ms
        + stationary_sampling_ms
        + worker_request_count * request_round_trip_headroom_ms
        + SCAN_FIXED_DEADLINE_HEADROOM_MS
    )
    return {
        "turn_count": len(turn_specs),
        "turn_slice_count": turn_slice_count,
        "turn_duration_ms": turn_duration_ms,
        "ray_count": ray_count,
        "stationary_sample_batches": stationary_sample_batches,
        "samples_per_batch": SCAN_SAMPLE_COUNT,
        "stationary_sampling_ms": stationary_sampling_ms,
        "worker_request_count": worker_request_count,
        "request_round_trip_headroom_ms": (
            request_round_trip_headroom_ms
        ),
        "minimum_deadline_ms": minimum_deadline_ms,
    }


@dataclass(frozen=True)
class ModelScanChoice:
    target_hypothesis_id: str
    policy: str = ADAPTIVE_COARSE_TO_FINE

    @classmethod
    def from_mapping(cls, value: object):
        if (
            not isinstance(value, dict)
            or set(value)
            != {"tool", "target_hypothesis_id", "policy"}
            or value["tool"] != TOOL_NAME
            or value["policy"] != ADAPTIVE_COARSE_TO_FINE
            or not isinstance(value["target_hypothesis_id"], str)
            or not value["target_hypothesis_id"]
        ):
            raise ActiveIrScanContractError(
                "invalid_model_scan_choice",
                "Model scan choice may select only the target and policy",
            )
        return cls(
            target_hypothesis_id=value["target_hypothesis_id"],
            policy=value["policy"],
        )


@dataclass(frozen=True)
class ActiveIrScanRequest:
    scan_id: str
    target_hypothesis_id: str
    frame_id: str
    map_generation_id: str
    based_on_map_version: int
    start_pose: PhysicalPose
    start_state_version: int
    created_at_ms: int
    deadline_ms: int
    created_monotonic_ms: int
    deadline_monotonic_ms: int
    max_snapshot_age_ms: int
    calibration: ActiveIrScanCalibration

    def __post_init__(self) -> None:
        if any(
            not isinstance(value, str) or not value
            for value in (
                self.scan_id,
                self.target_hypothesis_id,
                self.frame_id,
                self.map_generation_id,
            )
        ):
            raise ActiveIrScanContractError(
                "invalid_scan_identity",
                "Active IR scan identity is invalid",
            )
        if any(
            isinstance(value, bool) or not isinstance(value, int)
            for value in (
                self.based_on_map_version,
                self.start_state_version,
                self.created_at_ms,
                self.deadline_ms,
                self.created_monotonic_ms,
                self.deadline_monotonic_ms,
                self.max_snapshot_age_ms,
            )
        ) or (
            self.based_on_map_version < 0
            or self.start_state_version <= 0
            or self.created_at_ms < 0
            or self.deadline_ms <= self.created_at_ms
            or self.created_monotonic_ms < 0
            or self.deadline_monotonic_ms <= self.created_monotonic_ms
            or self.deadline_ms - self.created_at_ms
            != self.deadline_monotonic_ms - self.created_monotonic_ms
            or not 1 <= self.max_snapshot_age_ms <= 2_000
        ):
            raise ActiveIrScanContractError(
                "invalid_scan_time_or_version",
                "Active IR scan time/version is invalid",
            )


def build_scan_request(
    *,
    choice: ModelScanChoice,
    frame_id: str,
    map_generation_id: str,
    map_version: int,
    start_pose: PhysicalPose,
    start_state_version: int,
    created_at_ms: int,
    deadline_ms: int,
    created_monotonic_ms: Optional[int] = None,
    deadline_monotonic_ms: Optional[int] = None,
    max_snapshot_age_ms: int = 300,
    calibration: ActiveIrScanCalibration = ActiveIrScanCalibration(),
) -> ActiveIrScanRequest:
    if created_monotonic_ms is None:
        created_monotonic_ms = created_at_ms
    if deadline_monotonic_ms is None:
        deadline_monotonic_ms = deadline_ms
    raw = "\0".join(
        (
            map_generation_id,
            choice.target_hypothesis_id,
            str(map_version),
            str(start_state_version),
            str(created_at_ms),
        )
    ).encode("utf-8")
    return ActiveIrScanRequest(
        scan_id="scan-{}".format(hashlib.sha256(raw).hexdigest()[:20]),
        target_hypothesis_id=choice.target_hypothesis_id,
        frame_id=frame_id,
        map_generation_id=map_generation_id,
        based_on_map_version=map_version,
        start_pose=start_pose,
        start_state_version=start_state_version,
        created_at_ms=created_at_ms,
        deadline_ms=deadline_ms,
        created_monotonic_ms=created_monotonic_ms,
        deadline_monotonic_ms=deadline_monotonic_ms,
        max_snapshot_age_ms=max_snapshot_age_ms,
        calibration=calibration,
    )


@dataclass(frozen=True)
class ActiveIrRay:
    ordinal: int
    requested_relative_bearing_mdeg: int
    actual_relative_bearing_mdeg: int
    observed_at_ms: int
    state_version: int
    raw: Optional[int]
    filtered: Optional[int]
    blocked: bool

    def __post_init__(self) -> None:
        if (
            isinstance(self.ordinal, bool)
            or not isinstance(self.ordinal, int)
            or self.ordinal <= 0
            or any(
                isinstance(value, bool)
                or not isinstance(value, int)
                for value in (
                    self.requested_relative_bearing_mdeg,
                    self.actual_relative_bearing_mdeg,
                    self.observed_at_ms,
                    self.state_version,
                )
            )
            or not -90_000
            <= self.requested_relative_bearing_mdeg
            <= 90_000
            or not -100_000
            <= self.actual_relative_bearing_mdeg
            <= 100_000
            or self.observed_at_ms < 0
            or self.state_version <= 0
            or type(self.blocked) is not bool
        ):
            raise ActiveIrScanContractError(
                "invalid_scan_ray",
                "Active IR scan ray is invalid",
            )
        for reading in (self.raw, self.filtered):
            if reading is not None and (
                isinstance(reading, bool)
                or not isinstance(reading, int)
                or not 0 <= reading <= 100
            ):
                raise ActiveIrScanContractError(
                    "invalid_scan_reading",
                    "Active IR scan reading is invalid",
                )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "ordinal": self.ordinal,
            "requested_relative_bearing_mdeg": (
                self.requested_relative_bearing_mdeg
            ),
            "actual_relative_bearing_mdeg": (
                self.actual_relative_bearing_mdeg
            ),
            "observed_at_ms": self.observed_at_ms,
            "state_version": self.state_version,
            "raw": self.raw,
            "filtered": self.filtered,
            "blocked": self.blocked,
        }


@dataclass(frozen=True)
class ActiveIrScanResult:
    scan_id: str
    target_hypothesis_id: str
    frame_id: str
    map_generation_id: str
    based_on_map_version: int
    started_at_ms: int
    completed_at_ms: int
    status: str
    reason: str
    stop_confirmed: bool
    restored_start_heading: bool
    rays: Tuple[ActiveIrRay, ...]
    left_boundary_mdeg: Optional[int]
    right_boundary_mdeg: Optional[int]

    @property
    def bilateral_complete(self) -> bool:
        return (
            self.status == "COMPLETED"
            and self.stop_confirmed
            and self.restored_start_heading
            and self.left_boundary_mdeg is not None
            and self.right_boundary_mdeg is not None
            and self.left_boundary_mdeg > 0
            and self.right_boundary_mdeg < 0
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": SCAN_RESULT_SCHEMA,
            "scan_id": self.scan_id,
            "target_hypothesis_id": self.target_hypothesis_id,
            "frame_id": self.frame_id,
            "map_generation_id": self.map_generation_id,
            "based_on_map_version": self.based_on_map_version,
            "started_at_ms": self.started_at_ms,
            "completed_at_ms": self.completed_at_ms,
            "status": self.status,
            "reason": self.reason,
            "stop_confirmed": self.stop_confirmed,
            "restored_start_heading": self.restored_start_heading,
            "rays": [item.to_dict() for item in self.rays],
            "left_boundary_mdeg": self.left_boundary_mdeg,
            "right_boundary_mdeg": self.right_boundary_mdeg,
            "bilateral_complete": self.bilateral_complete,
            "route_or_side_selected_by_host": False,
        }


def validate_scan_result(
    result: ActiveIrScanResult,
    request: ActiveIrScanRequest,
    *,
    current_frame_id: str,
    current_map_generation_id: str,
    current_map_version: int,
) -> ActiveIrScanResult:
    if not isinstance(result, ActiveIrScanResult):
        raise ActiveIrScanContractError(
            "invalid_scan_result",
            "Scan result has the wrong type",
        )
    if (
        result.scan_id != request.scan_id
        or result.target_hypothesis_id
        != request.target_hypothesis_id
        or result.frame_id != request.frame_id
        or result.map_generation_id != request.map_generation_id
        or result.based_on_map_version != request.based_on_map_version
    ):
        raise ActiveIrScanContractError(
            "scan_correlation_mismatch",
            "Scan result does not match its request",
        )
    if (
        current_frame_id != request.frame_id
        or current_map_generation_id != request.map_generation_id
        or current_map_version != request.based_on_map_version
    ):
        raise ActiveIrScanContractError(
            "stale_scan_map_basis",
            "Map changed before scan evidence could be fused",
        )
    if (
        result.started_at_ms < request.created_at_ms
        or result.completed_at_ms < result.started_at_ms
        or result.completed_at_ms > request.deadline_ms
    ):
        raise ActiveIrScanContractError(
            "scan_deadline_or_chronology",
            "Scan result missed its absolute deadline or chronology",
        )
    previous_time = request.created_at_ms - 1
    previous_state = request.start_state_version
    ordinals = []
    requested_bearings = set()
    for ray in result.rays:
        ordinals.append(ray.ordinal)
        if (
            ray.observed_at_ms <= previous_time
            or ray.observed_at_ms > request.deadline_ms
            or ray.state_version <= previous_state
            or abs(
                ray.actual_relative_bearing_mdeg
                - ray.requested_relative_bearing_mdeg
            )
            > request.calibration.alignment_tolerance_mdeg
        ):
            raise ActiveIrScanContractError(
                "invalid_scan_ray_sequence",
                "Scan rays are stale, out of order, or pose-misaligned",
            )
        previous_time = ray.observed_at_ms
        previous_state = ray.state_version
        if ray.requested_relative_bearing_mdeg in requested_bearings:
            raise ActiveIrScanContractError(
                "duplicate_scan_bearing",
                "Scan sampled one requested bearing more than once",
            )
        requested_bearings.add(ray.requested_relative_bearing_mdeg)
    if ordinals != list(range(1, len(ordinals) + 1)):
        raise ActiveIrScanContractError(
            "invalid_scan_ray_ordinals",
            "Scan ray ordinals are not contiguous",
        )
    if result.status == "COMPLETED":
        sorted_rays = sorted(
            result.rays,
            key=lambda item: item.requested_relative_bearing_mdeg,
        )
        transition_midpoints = {
            int(
                round(
                    (
                        first.requested_relative_bearing_mdeg
                        + second.requested_relative_bearing_mdeg
                    )
                    / 2.0
                )
            )
            for first, second in zip(sorted_rays, sorted_rays[1:])
            if first.blocked != second.blocked
        }
        if (
            not result.bilateral_complete
            or result.left_boundary_mdeg not in transition_midpoints
            or result.right_boundary_mdeg not in transition_midpoints
        ):
            raise ActiveIrScanContractError(
                "completed_scan_lacks_bilateral_evidence",
                "Completed scan lacks derived two-sided boundary evidence",
            )
    if (
        type(result.stop_confirmed) is not bool
        or type(result.restored_start_heading) is not bool
    ):
        raise ActiveIrScanContractError(
            "invalid_scan_stop_confirmation",
            "Scan stop/restoration confirmation is invalid",
        )
    if result.restored_start_heading and not result.stop_confirmed:
        raise ActiveIrScanContractError(
            "unconfirmed_scan_restoration",
            "Restored scan heading requires confirmed motor stop",
        )
    if result.status not in ("COMPLETED", "CANCELLED"):
        raise ActiveIrScanContractError(
            "invalid_scan_status",
            "Scan result status is invalid",
        )
    return result


def relative_heading_mdeg(
    absolute_heading_mdeg: int,
    start_heading_mdeg: int,
) -> int:
    return normalize_heading_mdeg(
        absolute_heading_mdeg - start_heading_mdeg
    )
