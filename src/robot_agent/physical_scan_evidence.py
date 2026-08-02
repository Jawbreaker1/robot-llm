"""Bounded persisted evidence from physical active IR scan attempts."""

from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

from .active_ir_scan_contract import ActiveIrScanResult
from .physical_odometry import PhysicalPose


MAX_SCAN_ATTEMPTS_PER_HAZARD = 16
MAX_SCAN_ATTEMPTS_PER_MAP = 64
MAX_COLLISION_SUPPORTS_PER_HAZARD = 512
MAX_COLLISION_SUPPORTS_PER_MAP = 4_096
BODY_RELATIVE_BEARING_CONVENTION = "POSITIVE_LEFT_NEGATIVE_RIGHT"
ANGULAR_COLLISION_SUPPORT_PROVENANCE = (
    "PROVISIONAL_BLOCKED_IR_BEARING_NOT_MEASURED_RANGE_OR_SURFACE"
)


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
class AngularCollisionSupport:
    """Materialized qualitative support independent of detail retention.

    The EV3 IR-PROX reading has no trustworthy metric range.  This record
    therefore persists only a verified scan pose and blocked bearing.  The
    hazard-map calibration later projects that angular fact onto the same
    explicitly provisional collision envelope used for forward detections.
    It is never an object surface or a measured contact/range claim.
    """

    source_scan_id: str
    completed_at_ms: int
    pose_x_mm: int
    pose_y_mm: int
    pose_heading_mdeg: int
    actual_relative_bearing_mdeg: int
    based_on_map_version: int
    provenance: str = ANGULAR_COLLISION_SUPPORT_PROVENANCE

    def __post_init__(self) -> None:
        if (
            not isinstance(self.source_scan_id, str)
            or not self.source_scan_id
            or len(self.source_scan_id) > 128
            or any(ord(character) < 32 for character in self.source_scan_id)
            or any(
                isinstance(value, bool) or not isinstance(value, int)
                for value in (
                    self.completed_at_ms,
                    self.pose_x_mm,
                    self.pose_y_mm,
                    self.pose_heading_mdeg,
                    self.actual_relative_bearing_mdeg,
                    self.based_on_map_version,
                )
            )
            or self.completed_at_ms < 0
            or not -180_000 <= self.pose_heading_mdeg <= 179_999
            or not -100_000
            <= self.actual_relative_bearing_mdeg
            <= 100_000
            or self.based_on_map_version < 0
            or self.provenance != ANGULAR_COLLISION_SUPPORT_PROVENANCE
        ):
            raise ValueError("angular collision support is invalid")

    @property
    def spatial_key(self) -> Tuple[int, int, int, int]:
        """Exact collision geometry, excluding freshness and identifiers."""

        return (
            self.pose_x_mm,
            self.pose_y_mm,
            self.pose_heading_mdeg,
            self.actual_relative_bearing_mdeg,
        )

    @classmethod
    def from_attempt(
        cls,
        attempt: "ScanAttemptEvidence",
    ) -> Tuple["AngularCollisionSupport", ...]:
        if not isinstance(attempt, ScanAttemptEvidence):
            raise ValueError("scan attempt support source is invalid")
        if (
            attempt.scan_pose is None
            or attempt.based_on_map_version is None
            or attempt.hypothesis_relation
            != "SUPPORTS_BLOCKED_HYPOTHESIS"
        ):
            return ()
        pose = attempt.scan_pose
        return tuple(
            cls(
                source_scan_id=attempt.scan_id,
                completed_at_ms=attempt.completed_at_ms,
                pose_x_mm=pose.x_mm,
                pose_y_mm=pose.y_mm,
                pose_heading_mdeg=pose.heading_mdeg,
                actual_relative_bearing_mdeg=(
                    ray.actual_relative_bearing_mdeg
                ),
                based_on_map_version=attempt.based_on_map_version,
            )
            for ray in attempt.rays
            if ray.blocked
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "source_scan_id": self.source_scan_id,
            "completed_at_ms": self.completed_at_ms,
            "pose_x_mm": self.pose_x_mm,
            "pose_y_mm": self.pose_y_mm,
            "pose_heading_mdeg": self.pose_heading_mdeg,
            "actual_relative_bearing_mdeg": (
                self.actual_relative_bearing_mdeg
            ),
            "based_on_map_version": self.based_on_map_version,
            "provenance": self.provenance,
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, object]):
        fields = {
            "source_scan_id",
            "completed_at_ms",
            "pose_x_mm",
            "pose_y_mm",
            "pose_heading_mdeg",
            "actual_relative_bearing_mdeg",
            "based_on_map_version",
            "provenance",
        }
        if not isinstance(value, dict) or set(value) != fields:
            raise ValueError("angular collision support fields are invalid")
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
    all_clear_arc_covers_target_hypothesis: Optional[bool] = None

    def __post_init__(self) -> None:
        if (
            not isinstance(self.scan_id, str)
            or not self.scan_id
            or len(self.scan_id) > 128
            or any(ord(character) < 32 for character in self.scan_id)
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
            or (
                self.all_clear_arc_covers_target_hypothesis is not None
                and type(
                    self.all_clear_arc_covers_target_hypothesis
                ) is not bool
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
            # Absence evidence is destructive to a remembered collision
            # hypothesis, so legacy/unknown applicability must stay
            # fail-closed.  Only an explicit geometry check may contest it.
            if self.all_clear_arc_covers_target_hypothesis is True:
                return "CONFLICTS_BLOCKED_HYPOTHESIS"
            return "NO_EVIDENCE"
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
    def evidence_signature(self) -> Tuple[object, ...]:
        """Return exact decision-relevant spatial facts for retention.

        IDs, timestamps, map revisions, free-form reasons, and raw reflection
        jitter do not make a retry informative.  Verified pose state, actual
        bearings, blocked/clear classification, boundaries, and the derived
        hypothesis relation do.  Full ``PhysicalPose`` state is intentional:
        route evidence from a move-away-and-return trajectory must not silently
        become interchangeable with evidence from the earlier pose epoch.
        """

        pose = (
            None
            if self.scan_pose is None
            else (
                self.scan_pose.x_mm,
                self.scan_pose.y_mm,
                self.scan_pose.heading_mdeg,
                self.scan_pose.verified_motion_count,
                self.scan_pose.total_forward_mm,
                self.scan_pose.total_turn_mdeg,
            )
        )
        ray_facts = tuple(sorted(
            (
                ray.actual_relative_bearing_mdeg,
                ray.blocked,
            )
            for ray in self.rays
        ))
        return (
            pose,
            ray_facts,
            self.left_boundary_mdeg,
            self.right_boundary_mdeg,
            self.hypothesis_relation,
            self.status,
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
            "all_clear_arc_covers_target_hypothesis": (
                self.all_clear_arc_covers_target_hypothesis
            ),
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
        coverage_fields = fields | {
            "all_clear_arc_covers_target_hypothesis",
        }
        if (
            not isinstance(value, dict)
            or set(value) not in (legacy_fields, fields, coverage_fields)
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
            all_clear_arc_covers_target_hypothesis=value.get(
                "all_clear_arc_covers_target_hypothesis"
            ),
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


def collision_supports_from_attempts(
    attempts: Tuple[ScanAttemptEvidence, ...],
) -> Tuple[AngularCollisionSupport, ...]:
    """Materialize every derivable blocked angular fact from legacy detail."""

    if (
        not isinstance(attempts, tuple)
        or any(not isinstance(item, ScanAttemptEvidence) for item in attempts)
    ):
        raise ValueError("scan attempt support history is invalid")
    values = tuple(
        support
        for attempt in attempts
        for support in AngularCollisionSupport.from_attempt(attempt)
    )
    return retain_collision_support_diversity(
        values,
        MAX_COLLISION_SUPPORTS_PER_HAZARD,
    )


def retain_collision_support_diversity(
    supports: Tuple[AngularCollisionSupport, ...],
    limit: int = MAX_COLLISION_SUPPORTS_PER_HAZARD,
) -> Tuple[AngularCollisionSupport, ...]:
    """Keep the newest provenance for each exact angular support geometry."""

    if (
        not isinstance(supports, tuple)
        or any(
            not isinstance(item, AngularCollisionSupport)
            for item in supports
        )
        or isinstance(limit, bool)
        or not isinstance(limit, int)
        or limit <= 0
    ):
        raise ValueError("angular collision support retention is invalid")
    latest_by_spatial_key = {}
    for support in supports:
        current = latest_by_spatial_key.get(support.spatial_key)
        if current is None or (
            support.completed_at_ms,
            support.source_scan_id,
            support.based_on_map_version,
        ) > (
            current.completed_at_ms,
            current.source_scan_id,
            current.based_on_map_version,
        ):
            latest_by_spatial_key[support.spatial_key] = support
    return tuple(sorted(
        latest_by_spatial_key.values(),
        key=lambda item: (
            item.completed_at_ms,
            item.source_scan_id,
            item.spatial_key,
        ),
    )[-limit:])


__all__ = (
    "ANGULAR_COLLISION_SUPPORT_PROVENANCE",
    "AngularCollisionSupport",
    "BODY_RELATIVE_BEARING_CONVENTION",
    "MAX_COLLISION_SUPPORTS_PER_HAZARD",
    "MAX_COLLISION_SUPPORTS_PER_MAP",
    "MAX_SCAN_ATTEMPTS_PER_HAZARD",
    "MAX_SCAN_ATTEMPTS_PER_MAP",
    "ScanAttemptEvidence",
    "ScanRayEvidence",
    "collision_supports_from_attempts",
    "retain_collision_support_diversity",
    "retain_scan_attempt_diversity",
)
