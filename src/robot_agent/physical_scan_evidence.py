"""Bounded persisted evidence from physical active IR scan attempts."""

from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

from .active_ir_scan_contract import ActiveIrScanResult
from .physical_odometry import PhysicalPose


MAX_SCAN_ATTEMPTS_PER_HAZARD = 4
MAX_SCAN_ATTEMPTS_PER_MAP = 8
BODY_RELATIVE_BEARING_CONVENTION = "POSITIVE_LEFT_NEGATIVE_RIGHT"


@dataclass(frozen=True)
class ScanRayEvidence:
    """Compact persisted form of one validated active-scan ray."""

    requested_relative_bearing_mdeg: int
    actual_relative_bearing_mdeg: int
    blocked: bool
    raw: Optional[int]
    filtered: Optional[int]

    def __post_init__(self) -> None:
        if (
            isinstance(self.requested_relative_bearing_mdeg, bool)
            or not isinstance(self.requested_relative_bearing_mdeg, int)
            or not -90_000
            <= self.requested_relative_bearing_mdeg
            <= 90_000
            or isinstance(self.actual_relative_bearing_mdeg, bool)
            or not isinstance(self.actual_relative_bearing_mdeg, int)
            or not -100_000 <= self.actual_relative_bearing_mdeg <= 100_000
            or type(self.blocked) is not bool
        ):
            raise ValueError("scan ray evidence is invalid")
        for reading in (self.raw, self.filtered):
            if reading is not None and (
                isinstance(reading, bool)
                or not isinstance(reading, int)
                or not 0 <= reading <= 100
            ):
                raise ValueError("scan ray evidence is invalid")

    @classmethod
    def from_active_ray(cls, ray):
        return cls(
            requested_relative_bearing_mdeg=(
                ray.requested_relative_bearing_mdeg
            ),
            actual_relative_bearing_mdeg=(
                ray.actual_relative_bearing_mdeg
            ),
            blocked=ray.blocked,
            raw=ray.raw,
            filtered=ray.filtered,
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "requested_relative_bearing_mdeg": (
                self.requested_relative_bearing_mdeg
            ),
            "actual_relative_bearing_mdeg": (
                self.actual_relative_bearing_mdeg
            ),
            "blocked": self.blocked,
            "raw": self.raw,
            "filtered": self.filtered,
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, object]):
        fields = {
            "requested_relative_bearing_mdeg",
            "actual_relative_bearing_mdeg",
            "blocked",
            "raw",
            "filtered",
        }
        if not isinstance(value, dict) or set(value) != fields:
            raise ValueError("scan ray evidence fields are invalid")
        return cls(**value)


@dataclass(frozen=True)
class ScanAttemptEvidence:
    """Structured evidence retained after planner feedback moves on."""

    scan_id: str
    completed_at_ms: int
    status: str
    reason: str
    rays: Tuple[ScanRayEvidence, ...]
    left_boundary_mdeg: Optional[int]
    right_boundary_mdeg: Optional[int]
    scan_pose: Optional[PhysicalPose] = None
    based_on_map_version: Optional[int] = None

    def __post_init__(self) -> None:
        if (
            not isinstance(self.scan_id, str)
            or not self.scan_id
            or isinstance(self.completed_at_ms, bool)
            or not isinstance(self.completed_at_ms, int)
            or self.completed_at_ms < 0
            or self.status not in ("COMPLETED", "CANCELLED")
            or not isinstance(self.reason, str)
            or not self.reason
            or len(self.reason) > 160
            or not isinstance(self.rays, tuple)
            or len(self.rays) > 16
            or any(not isinstance(ray, ScanRayEvidence) for ray in self.rays)
            or len({
                ray.requested_relative_bearing_mdeg for ray in self.rays
            }) != len(self.rays)
            or (self.scan_pose is None)
            is not (self.based_on_map_version is None)
            or (
                self.scan_pose is not None
                and not isinstance(self.scan_pose, PhysicalPose)
            )
            or (
                self.based_on_map_version is not None
                and (
                    isinstance(self.based_on_map_version, bool)
                    or not isinstance(self.based_on_map_version, int)
                    or self.based_on_map_version < 0
                )
            )
        ):
            raise ValueError("scan attempt evidence is invalid")
        ordered = sorted(
            self.rays,
            key=lambda ray: ray.requested_relative_bearing_mdeg,
        )
        transitions = {
            int(round(
                (
                    first.requested_relative_bearing_mdeg
                    + second.requested_relative_bearing_mdeg
                )
                / 2.0
            ))
            for first, second in zip(ordered, ordered[1:])
            if first.blocked != second.blocked
        }
        if (
            self.left_boundary_mdeg is not None
            and (
                isinstance(self.left_boundary_mdeg, bool)
                or not isinstance(self.left_boundary_mdeg, int)
                or self.left_boundary_mdeg <= 0
                or self.left_boundary_mdeg not in transitions
            )
        ) or (
            self.right_boundary_mdeg is not None
            and (
                isinstance(self.right_boundary_mdeg, bool)
                or not isinstance(self.right_boundary_mdeg, int)
                or self.right_boundary_mdeg >= 0
                or self.right_boundary_mdeg not in transitions
            )
        ):
            raise ValueError("scan attempt boundary evidence is invalid")

    @classmethod
    def from_scan_result(
        cls,
        result: ActiveIrScanResult,
        *,
        scan_pose: PhysicalPose,
    ):
        if not isinstance(result, ActiveIrScanResult):
            raise ValueError("scan result evidence is invalid")
        if not isinstance(scan_pose, PhysicalPose):
            raise ValueError("scan result pose is invalid")
        return cls(
            scan_id=result.scan_id,
            completed_at_ms=result.completed_at_ms,
            status=result.status,
            reason=result.reason,
            rays=tuple(
                ScanRayEvidence.from_active_ray(ray) for ray in result.rays
            ),
            left_boundary_mdeg=result.left_boundary_mdeg,
            right_boundary_mdeg=result.right_boundary_mdeg,
            scan_pose=scan_pose,
            based_on_map_version=result.based_on_map_version,
        )

    @property
    def observation_pattern(self) -> str:
        if not self.rays:
            return "NO_RAYS"
        blocked = sum(1 for ray in self.rays if ray.blocked)
        if blocked == 0:
            return "ALL_CLEAR"
        if blocked == len(self.rays):
            return "ALL_BLOCKED"
        return "MIXED"

    @property
    def arc_coverage(self) -> str:
        negative = any(
            ray.requested_relative_bearing_mdeg < 0 for ray in self.rays
        )
        positive = any(
            ray.requested_relative_bearing_mdeg > 0 for ray in self.rays
        )
        if negative and positive:
            return "BILATERAL_ARC"
        if negative:
            return "NEGATIVE_ARC_ONLY"
        if positive:
            return "POSITIVE_ARC_ONLY"
        return "CENTER_ONLY" if self.rays else "NO_ARC"

    @property
    def hypothesis_relation(self) -> str:
        if (
            self.observation_pattern == "ALL_CLEAR"
            and self.arc_coverage == "BILATERAL_ARC"
            and self.reason == "bilateral_boundaries_not_observed"
        ):
            return "CONFLICTS_BLOCKED_HYPOTHESIS"
        if any(ray.blocked for ray in self.rays):
            return "SUPPORTS_BLOCKED_HYPOTHESIS"
        return "NO_EVIDENCE"

    @property
    def boundary_coverage(self) -> str:
        if (
            self.left_boundary_mdeg is not None
            and self.right_boundary_mdeg is not None
        ):
            return "BILATERAL_BOUNDARIES"
        if self.left_boundary_mdeg is not None:
            return "POSITIVE_BOUNDARY_ONLY"
        if self.right_boundary_mdeg is not None:
            return "NEGATIVE_BOUNDARY_ONLY"
        return "NO_BOUNDARIES"

    @property
    def bilateral_complete(self) -> bool:
        return (
            self.status == "COMPLETED"
            and self.left_boundary_mdeg is not None
            and self.right_boundary_mdeg is not None
        )

    @property
    def evidence_signature(self) -> Tuple[str, str, str, str]:
        """Classify information content without using language or IDs."""

        return (
            self.observation_pattern,
            self.arc_coverage,
            self.boundary_coverage,
            self.hypothesis_relation,
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "scan_id": self.scan_id,
            "completed_at_ms": self.completed_at_ms,
            "status": self.status,
            "reason": self.reason,
            "bearing_convention": BODY_RELATIVE_BEARING_CONVENTION,
            "observation_pattern": self.observation_pattern,
            "arc_coverage": self.arc_coverage,
            "boundary_coverage": self.boundary_coverage,
            "hypothesis_relation": self.hypothesis_relation,
            "left_boundary_mdeg": self.left_boundary_mdeg,
            "right_boundary_mdeg": self.right_boundary_mdeg,
            "scan_pose": (
                None if self.scan_pose is None else self.scan_pose.to_dict()
            ),
            "based_on_map_version": self.based_on_map_version,
            "rays": [ray.to_dict() for ray in self.rays],
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, object]):
        legacy_fields = {
            "scan_id",
            "completed_at_ms",
            "status",
            "reason",
            "bearing_convention",
            "observation_pattern",
            "arc_coverage",
            "boundary_coverage",
            "hypothesis_relation",
            "left_boundary_mdeg",
            "right_boundary_mdeg",
            "rays",
        }
        fields = legacy_fields | {
            "scan_pose",
            "based_on_map_version",
        }
        if (
            not isinstance(value, dict)
            or set(value) not in (legacy_fields, fields)
            or not isinstance(value["rays"], list)
            or value["bearing_convention"]
            != BODY_RELATIVE_BEARING_CONVENTION
        ):
            raise ValueError("scan attempt evidence fields are invalid")
        attempt = cls(
            scan_id=value["scan_id"],
            completed_at_ms=value["completed_at_ms"],
            status=value["status"],
            reason=value["reason"],
            rays=tuple(
                ScanRayEvidence.from_dict(ray) for ray in value["rays"]
            ),
            left_boundary_mdeg=value["left_boundary_mdeg"],
            right_boundary_mdeg=value["right_boundary_mdeg"],
            scan_pose=(
                None
                if value.get("scan_pose") is None
                else PhysicalPose.from_mapping(value["scan_pose"])
            ),
            based_on_map_version=value.get("based_on_map_version"),
        )
        if (
            value["observation_pattern"] != attempt.observation_pattern
            or value["arc_coverage"] != attempt.arc_coverage
            or value["boundary_coverage"] != attempt.boundary_coverage
            or value["hypothesis_relation"] != attempt.hypothesis_relation
        ):
            raise ValueError("scan attempt derived evidence is invalid")
        return attempt


def retain_scan_attempt_diversity(
    attempts: Tuple[ScanAttemptEvidence, ...],
    limit: int = MAX_SCAN_ATTEMPTS_PER_HAZARD,
) -> Tuple[ScanAttemptEvidence, ...]:
    """Retain recent distinct evidence before duplicate retries.

    Repeated attempts with the same information shape must not evict older
    unilateral boundaries or contradiction evidence.  If genuinely distinct
    evidence exceeds the bound, the newest bounded set wins.
    """

    if (
        not isinstance(attempts, tuple)
        or any(not isinstance(item, ScanAttemptEvidence) for item in attempts)
        or isinstance(limit, bool)
        or not isinstance(limit, int)
        or limit <= 0
    ):
        raise ValueError("scan attempt retention is invalid")
    latest_by_signature = {}
    for attempt in attempts:
        latest_by_signature[attempt.evidence_signature] = attempt
    retained = sorted(
        latest_by_signature.values(),
        key=lambda item: item.completed_at_ms,
    )[-limit:]
    if len(retained) < limit:
        retained_ids = {item.scan_id for item in retained}
        older = [
            item
            for item in reversed(attempts)
            if item.scan_id not in retained_ids
        ][:limit - len(retained)]
        retained.extend(older)
    return tuple(sorted(
        retained,
        key=lambda item: item.completed_at_ms,
    ))


__all__ = (
    "BODY_RELATIVE_BEARING_CONVENTION",
    "MAX_SCAN_ATTEMPTS_PER_HAZARD",
    "MAX_SCAN_ATTEMPTS_PER_MAP",
    "ScanAttemptEvidence",
    "ScanRayEvidence",
    "retain_scan_attempt_diversity",
)
