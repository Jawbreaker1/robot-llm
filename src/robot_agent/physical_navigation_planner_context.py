"""Deterministic, evidence-preserving projection for physical planning.

The authoritative navigation state may be generous.  This module builds a
bounded model view without ranking actions or interpreting natural language.
Typed references and deterministic geometry facts decide which hazard details
are focused; every hazard identity and every compact evidence aggregate stays
visible even when raw scan detail is omitted.
"""

from copy import deepcopy
import hashlib
from typing import Mapping, Optional

from .physical_navigation_contract import json_bytes


PLANNER_PROJECTION_SCHEMA = "robot-physical-planner-projection/v1"
SCAN_HISTORY_FIELD = "scan_evidence_history"
COMPACT_ROUTE_SCAN_DETAIL = "COMPACT_CURRENT_POSE_ROUTE_FACTS"
COMPACT_SCAN_SUMMARY = "COMPACT_TYPED_SCAN_SUMMARY"
MAX_PLANNER_EXPERIENCE_BYTES = 24 * 1024
FULL_DETAIL_FOCUS_REASONS = frozenset((
    "ACTIVE_COMMITMENT",
    "LATEST_TOOL_TARGET",
    "LATEST_SCAN_TARGET",
    "CURRENT_POSE_ROUTE_EVIDENCE",
))


class PhysicalPlannerContextError(ValueError):
    pass


def _string_counts(values) -> Mapping[str, int]:
    counts = {}
    for value in values:
        if not isinstance(value, str) or not value:
            raise PhysicalPlannerContextError(
                "scan evidence classification is invalid"
            )
        counts[value] = counts.get(value, 0) + 1
    return {key: counts[key] for key in sorted(counts)}


def _scan_summary(history, *, detailed_count: int) -> Mapping[str, object]:
    if not isinstance(history, list) or any(
        not isinstance(item, Mapping) for item in history
    ):
        raise PhysicalPlannerContextError("scan evidence history is invalid")
    if (
        isinstance(detailed_count, bool)
        or not isinstance(detailed_count, int)
        or not 0 <= detailed_count <= len(history)
    ):
        raise PhysicalPlannerContextError("scan detail count is invalid")
    completed = [
        item.get("completed_at_ms")
        for item in history
        if isinstance(item.get("completed_at_ms"), int)
        and not isinstance(item.get("completed_at_ms"), bool)
    ]
    rays = []
    for item in history:
        item_rays = item.get("rays", [])
        if not isinstance(item_rays, list):
            raise PhysicalPlannerContextError("scan evidence rays are invalid")
        rays.extend(item_rays)
    if any(not isinstance(ray, Mapping) for ray in rays):
        raise PhysicalPlannerContextError("scan evidence ray is invalid")
    scan_ids = [item.get("scan_id") for item in history]
    if any(not isinstance(scan_id, str) or not scan_id for scan_id in scan_ids):
        raise PhysicalPlannerContextError("scan evidence identity is invalid")
    pose_keys = {
        json_bytes(item["scan_pose"])
        for item in history
        if item.get("scan_pose") is not None
    }
    return {
        "attempt_counts": {
            "retained": len(history),
            "projected_detail": detailed_count,
            "omitted_detail": len(history) - detailed_count,
        },
        "scan_ids": scan_ids,
        "completed_at_ms": {
            "first": min(completed) if completed else None,
            "latest": max(completed) if completed else None,
        },
        "categorical_counts": {
            "status": _string_counts(item.get("status") for item in history),
            "observation_pattern": _string_counts(
                item.get("observation_pattern") for item in history
            ),
            "arc_coverage": _string_counts(
                item.get("arc_coverage") for item in history
            ),
            "boundary_coverage": _string_counts(
                item.get("boundary_coverage") for item in history
            ),
            "hypothesis_relation": _string_counts(
                item.get("hypothesis_relation") for item in history
            ),
        },
        "ray_counts": {
            "total": len(rays),
            "blocked": sum(1 for ray in rays if ray.get("blocked") is True),
            "clear": sum(1 for ray in rays if ray.get("blocked") is False),
        },
        "pose_counts": {
            "verified": len(pose_keys),
            "missing": sum(
                1 for item in history if item.get("scan_pose") is None
            ),
        },
    }


def _compact_scan_summary(
    history,
    *,
    persisted_evicted_count: int = 0,
    persisted_eviction_reason=None,
) -> Mapping[str, object]:
    """Publish exact totals/latest typed facts without raw old attempts."""

    _scan_summary(history, detailed_count=0)
    latest = max(
        history,
        key=lambda item: (
            item.get("completed_at_ms", -1),
            item.get("scan_id", ""),
        ),
        default=None,
    )
    latest_facts = None
    if latest is not None:
        latest_facts = [
            deepcopy(latest.get(field))
            for field in (
                "scan_id",
                "completed_at_ms",
                "status",
                "observation_pattern",
                "arc_coverage",
                "boundary_coverage",
                "hypothesis_relation",
            )
        ]
    value = {
        "retained_attempt_count": len(history),
        "persisted_evicted_attempt_count": persisted_evicted_count,
        "total_known_attempt_count": len(history) + persisted_evicted_count,
        "latest_retained_attempt_values": latest_facts,
    }
    if persisted_eviction_reason is not None:
        value["persisted_eviction_reason"] = persisted_eviction_reason
    return value


def _project_experience_ledger(
    value,
    *,
    target_bytes: int = MAX_PLANNER_EXPERIENCE_BYTES,
) -> Mapping[str, object]:
    """Keep exact totals/latest outcomes and a bounded recent detail tail."""

    if not isinstance(value, Mapping):
        raise PhysicalPlannerContextError("experience ledger is invalid")
    projected = deepcopy(dict(value))
    entries = projected.get("entries", [])
    rollups = projected.get("current_basis_action_rollups", [])
    if not isinstance(entries, list) or not isinstance(rollups, list):
        raise PhysicalPlannerContextError("experience ledger is invalid")
    source_entries = deepcopy(entries)
    source_distributions = []
    for rollup in rollups:
        if not isinstance(rollup, dict):
            raise PhysicalPlannerContextError("experience rollup is invalid")
        distribution = rollup.get("outcome_distribution", [])
        if not isinstance(distribution, list):
            raise PhysicalPlannerContextError("experience rollup is invalid")
        source_distributions.append({
            "action": rollup.get("action"),
            "outcome_distribution": deepcopy(distribution),
        })
        bucket_count = rollup.get("outcome_bucket_count", len(distribution))
        attempt_count = rollup.get("attempt_count", 0)
        rollup["outcome_bucket_retained_count"] = 0
        rollup["outcome_bucket_omitted_count"] = bucket_count
        rollup["outcome_attempt_retained_count"] = 0
        rollup["outcome_attempt_omitted_count"] = attempt_count
        rollup["outcome_distribution"] = []
    omitted = []

    def refresh_metadata():
        projected["planner_projection"] = {
            "target_bytes": target_bytes,
            "source_detailed_entry_count": len(source_entries),
            "projected_detailed_entry_count": len(projected["entries"]),
            "omitted_detailed_entry_count": (
                len(source_entries) - len(projected["entries"])
            ),
            "outcome_distributions_omitted": True,
            "omitted_detail_sha256": hashlib.sha256(json_bytes({
                "entries": omitted,
                "outcome_distributions": source_distributions,
            })).hexdigest(),
            "host_ranked_or_selected_action": False,
        }

    refresh_metadata()
    while len(json_bytes(projected)) > target_bytes:
        if not projected["entries"]:
            projected["planner_projection"][
                "target_budget_exceeded_due_to_mandatory_facts"
            ] = True
            break
        omitted.append(projected["entries"].pop(0))
        refresh_metadata()
    return projected


def _bearing_range(rays, *, blocked: bool) -> Mapping[str, object]:
    values = [
        ray.get("actual_relative_bearing_mdeg")
        for ray in rays
        if ray.get("blocked") is blocked
        and isinstance(ray.get("actual_relative_bearing_mdeg"), int)
        and not isinstance(ray.get("actual_relative_bearing_mdeg"), bool)
    ]
    return {
        "count": len(values),
        "minimum_actual_relative_bearing_mdeg": (
            min(values) if values else None
        ),
        "maximum_actual_relative_bearing_mdeg": (
            max(values) if values else None
        ),
    }


def _compact_route_scan_detail(
    attempt: Mapping[str, object],
) -> Mapping[str, object]:
    """Keep exact route facts while dropping raw IR reflection detail."""

    rays = attempt.get("rays", [])
    if not isinstance(rays, list) or any(
        not isinstance(ray, Mapping) for ray in rays
    ):
        raise PhysicalPlannerContextError("scan evidence rays are invalid")
    blocked = _bearing_range(rays, blocked=True)
    clear = _bearing_range(rays, blocked=False)
    return {
        "detail_projection": COMPACT_ROUTE_SCAN_DETAIL,
        "scan_id": attempt.get("scan_id"),
        "values": [
            attempt.get("completed_at_ms"),
            attempt.get("status"),
            attempt.get("bearing_convention"),
            attempt.get("observation_pattern"),
            attempt.get("arc_coverage"),
            attempt.get("boundary_coverage"),
            attempt.get("hypothesis_relation"),
            attempt.get("left_boundary_mdeg"),
            attempt.get("right_boundary_mdeg"),
            deepcopy(attempt.get("scan_pose")),
            attempt.get("based_on_map_version"),
        ],
        "ray_fact_values": [
            len(rays),
            blocked["count"],
            blocked["minimum_actual_relative_bearing_mdeg"],
            blocked["maximum_actual_relative_bearing_mdeg"],
            clear["count"],
            clear["minimum_actual_relative_bearing_mdeg"],
            clear["maximum_actual_relative_bearing_mdeg"],
        ],
    }


def _typed_hazard_references(
    navigation: Mapping[str, object],
    maneuver_state: Mapping[str, object],
    last_tool_result: Optional[Mapping[str, object]],
) -> Mapping[str, tuple[str, ...]]:
    reasons = {}

    def add(value, reason):
        if isinstance(value, str) and value:
            reasons.setdefault(value, set()).add(reason)

    active = maneuver_state.get("active")
    if isinstance(active, Mapping):
        add(active.get("target_hypothesis_id"), "ACTIVE_COMMITMENT")

    goal_geometry = navigation.get("goal_geometry")
    if isinstance(goal_geometry, Mapping):
        conflicts = goal_geometry.get("conflicts", [])
        if isinstance(conflicts, list):
            for conflict in conflicts:
                if isinstance(conflict, Mapping):
                    add(
                        conflict.get("hypothesis_id"),
                        "GOAL_GEOMETRY_CONFLICT",
                    )

    feasibility = navigation.get("action_feasibility")
    if isinstance(feasibility, Mapping):
        motion = feasibility.get("motion_actions")
        if isinstance(motion, Mapping):
            feasibility_values = list(motion.values())
        else:
            feasibility_values = []
        active_scan = feasibility.get("active_scan")
        if isinstance(active_scan, Mapping):
            feasibility_values.append(active_scan)
        for value in feasibility_values:
            if not isinstance(value, Mapping):
                continue
            for field in ("hazard_ids", "monotonic_escape_hazard_ids"):
                identifiers = value.get(field, [])
                if isinstance(identifiers, (list, tuple)):
                    for identifier in identifiers:
                        add(identifier, "ACTION_FEASIBILITY_REFERENCE")

    if isinstance(last_tool_result, Mapping):
        add(last_tool_result.get("target_hypothesis_id"), "LATEST_TOOL_TARGET")
        add(
            last_tool_result.get("perception_target_hypothesis_id"),
            "LATEST_TOOL_TARGET",
        )
        scan = last_tool_result.get("scan")
        if isinstance(scan, Mapping):
            add(scan.get("target_hypothesis_id"), "LATEST_SCAN_TARGET")

    hazards = navigation.get("navigation_hazard_hypotheses", [])
    if isinstance(hazards, list):
        for hazard in hazards:
            if not isinstance(hazard, Mapping):
                continue
            route = hazard.get("route_evidence")
            if (
                isinstance(route, Mapping)
                and isinstance(route.get("applicable_scan_ids"), list)
                and route["applicable_scan_ids"]
            ):
                add(
                    hazard.get("hypothesis_id"),
                    "CURRENT_POSE_ROUTE_EVIDENCE",
                )
    return {
        hypothesis_id: tuple(sorted(values))
        for hypothesis_id, values in sorted(reasons.items())
    }


def _refresh_projection_metadata(
    projected: dict,
    *,
    histories: Mapping[str, list],
    reference_reasons: Mapping[str, tuple[str, ...]],
    detail_focus_reasons: Mapping[str, tuple[str, ...]],
    unresolved_focus_ids,
    target_budget_bytes: int,
    hard_budget_bytes: int,
) -> None:
    hazards = projected["navigation_hazard_hypotheses"]
    retained = 0
    compact = 0
    total = 0
    by_id_detail = {
        hazard["hypothesis_id"]: hazard[SCAN_HISTORY_FIELD]
        for hazard in hazards
    }
    for hazard in hazards:
        hypothesis_id = hazard["hypothesis_id"]
        history = histories[hypothesis_id]
        detailed = hazard[SCAN_HISTORY_FIELD]
        total += len(history)
        retained += len(detailed)
        compact += sum(
            1
            for item in detailed
            if item.get("detail_projection") == COMPACT_ROUTE_SCAN_DETAIL
        )
        focus_reasons = detail_focus_reasons.get(hypothesis_id, ())
        if any(
            reason != "CURRENT_POSE_ROUTE_EVIDENCE"
            for reason in focus_reasons
        ):
            hazard["scan_evidence_summary"] = _scan_summary(
                history,
                detailed_count=len(detailed),
            )
            persisted_evicted = hazard.get(
                "scan_attempts_evicted",
                0,
            )
            hazard["scan_evidence_summary"]["attempt_counts"].update({
                "persisted_evicted": persisted_evicted,
                "total_known": len(history) + persisted_evicted,
            })
            if hazard.get("scan_attempts_eviction_reason") is not None:
                hazard["scan_evidence_summary"][
                    "persisted_eviction_reason"
                ] = hazard.get(
                    "scan_attempts_eviction_reason"
                )
        else:
            hazard["scan_evidence_summary"] = _compact_scan_summary(
                history,
                persisted_evicted_count=hazard.get(
                    "scan_attempts_evicted",
                    0,
                ),
                persisted_eviction_reason=hazard.get(
                    "scan_attempts_eviction_reason"
                ),
            )
            if hypothesis_id in detail_focus_reasons:
                hazard["scan_evidence_summary"]["scan_ids"] = [
                    item["scan_id"] for item in history
                ]
    projected["planner_context_projection"] = {
        "schema": PLANNER_PROJECTION_SCHEMA,
        "host_ranked_or_selected_action": False,
        "hazard_count": len(hazards),
        "all_hazard_ids_preserved": True,
        "referenced_hypothesis_ids": sorted(reference_reasons),
        "focused_hypothesis_ids": sorted(detail_focus_reasons),
        "focus_reasons_by_hypothesis_id": {
            key: list(detail_focus_reasons[key])
            for key in sorted(detail_focus_reasons)
        },
        "reference_reasons_by_hypothesis_id": {
            key: list(reference_reasons[key])
            for key in sorted(reference_reasons)
        },
        "unresolved_focused_hypothesis_ids": sorted(unresolved_focus_ids),
        "scan_attempt_count": total,
        "scan_detail_retained_count": retained,
        "scan_full_detail_retained_count": retained - compact,
        "scan_compact_detail_retained_count": compact,
        "scan_detail_omitted_count": total - retained,
        "compact_scan_latest_fields": [
            "scan_id",
            "completed_at_ms",
            "status",
            "observation_pattern",
            "arc_coverage",
            "boundary_coverage",
            "hypothesis_relation",
        ],
        "compact_route_scan_value_fields": [
            "completed_at_ms",
            "status",
            "bearing_convention",
            "observation_pattern",
            "arc_coverage",
            "boundary_coverage",
            "hypothesis_relation",
            "left_boundary_mdeg",
            "right_boundary_mdeg",
            "scan_pose",
            "based_on_map_version",
        ],
        "compact_route_scan_ray_fact_fields": [
            "ray_count",
            "blocked_count",
            "blocked_minimum_actual_relative_bearing_mdeg",
            "blocked_maximum_actual_relative_bearing_mdeg",
            "clear_count",
            "clear_minimum_actual_relative_bearing_mdeg",
            "clear_maximum_actual_relative_bearing_mdeg",
        ],
        "omitted_scan_detail_sha256": hashlib.sha256(json_bytes([
            {
                "hypothesis_id": hypothesis_id,
                "history": histories[hypothesis_id],
            }
            for hypothesis_id in sorted(histories)
            if not by_id_detail.get(hypothesis_id)
        ])).hexdigest(),
        "target_budget_bytes": target_budget_bytes,
        "hard_budget_bytes": hard_budget_bytes,
        "target_budget_exceeded_due_to_mandatory_facts": False,
    }


def project_navigation_context(
    navigation: Mapping[str, object],
    *,
    maneuver_state: Mapping[str, object],
    last_tool_result: Optional[Mapping[str, object]],
    target_budget_bytes: int,
    hard_budget_bytes: int,
) -> Mapping[str, object]:
    """Project full state without dropping mandatory identities or facts."""

    if (
        not isinstance(navigation, Mapping)
        or not isinstance(maneuver_state, Mapping)
        or isinstance(target_budget_bytes, bool)
        or not isinstance(target_budget_bytes, int)
        or isinstance(hard_budget_bytes, bool)
        or not isinstance(hard_budget_bytes, int)
        or target_budget_bytes <= 0
        or hard_budget_bytes < target_budget_bytes
    ):
        raise PhysicalPlannerContextError(
            "planner projection arguments are invalid"
        )
    hazards = navigation.get("navigation_hazard_hypotheses")
    if not isinstance(hazards, list):
        raise PhysicalPlannerContextError("navigation hazards are invalid")
    identifiers = [
        hazard.get("hypothesis_id")
        for hazard in hazards
        if isinstance(hazard, Mapping)
    ]
    if (
        len(identifiers) != len(hazards)
        or any(not isinstance(item, str) or not item for item in identifiers)
        or len(set(identifiers)) != len(identifiers)
    ):
        raise PhysicalPlannerContextError("navigation hazard IDs are invalid")

    referenced = _typed_hazard_references(
        navigation,
        maneuver_state,
        last_tool_result,
    )
    known_ids = frozenset(identifiers)
    reference_reasons = {
        key: value for key, value in referenced.items() if key in known_ids
    }
    detail_focus_reasons = {
        key: tuple(
            reason for reason in reasons
            if reason in FULL_DETAIL_FOCUS_REASONS
        )
        for key, reasons in reference_reasons.items()
        if any(reason in FULL_DETAIL_FOCUS_REASONS for reason in reasons)
    }
    raw_detail_focus_ids = {
        key
        for key, reasons in detail_focus_reasons.items()
        if any(
            reason != "CURRENT_POSE_ROUTE_EVIDENCE"
            for reason in reasons
        )
    }
    unresolved_focus_ids = set(referenced) - known_ids

    projected = deepcopy(dict(navigation))
    if (
        "scan_front_arc_feasibility" in projected
        and isinstance(projected.get("action_feasibility"), Mapping)
        and isinstance(
            projected["action_feasibility"].get("active_scan"),
            Mapping,
        )
    ):
        del projected["scan_front_arc_feasibility"]
    if "experience_ledger" in projected:
        projected["experience_ledger"] = _project_experience_ledger(
            projected["experience_ledger"]
        )

    histories = {}
    projected_hazards = []
    for hazard in hazards:
        value = deepcopy(dict(hazard))
        history = value.get(SCAN_HISTORY_FIELD, [])
        if not isinstance(history, list):
            raise PhysicalPlannerContextError(
                "navigation scan evidence is invalid"
            )
        histories[value["hypothesis_id"]] = deepcopy(history)
        if value["hypothesis_id"] in raw_detail_focus_ids:
            value[SCAN_HISTORY_FIELD] = deepcopy(history)
        elif value["hypothesis_id"] in detail_focus_reasons:
            route = value.get("route_evidence", {})
            applicable = set(route.get("applicable_scan_ids", []))
            value[SCAN_HISTORY_FIELD] = [
                _compact_route_scan_detail(item)
                for item in history
                if item.get("scan_id") in applicable
            ]
        else:
            value[SCAN_HISTORY_FIELD] = []
        projected_hazards.append(value)
    projected["navigation_hazard_hypotheses"] = projected_hazards
    _refresh_projection_metadata(
        projected,
        histories=histories,
        reference_reasons=reference_reasons,
        detail_focus_reasons=detail_focus_reasons,
        unresolved_focus_ids=unresolved_focus_ids,
        target_budget_bytes=target_budget_bytes,
        hard_budget_bytes=hard_budget_bytes,
    )

    protected_scan_ids = set()
    for hazard in projected_hazards:
        route = hazard.get("route_evidence")
        if isinstance(route, Mapping):
            applicable = route.get("applicable_scan_ids", [])
            if isinstance(applicable, list):
                protected_scan_ids.update(
                    item for item in applicable if isinstance(item, str)
                )
    removable = []
    for hazard in projected_hazards:
        hypothesis_id = hazard["hypothesis_id"]
        for attempt in hazard[SCAN_HISTORY_FIELD]:
            scan_id = attempt.get("scan_id")
            completed_at_ms = attempt.get("completed_at_ms")
            if scan_id in protected_scan_ids:
                continue
            removable.append((
                completed_at_ms if isinstance(completed_at_ms, int) else -1,
                hypothesis_id,
                scan_id if isinstance(scan_id, str) else "",
            ))
    removable.sort()
    by_id = {
        hazard["hypothesis_id"]: hazard for hazard in projected_hazards
    }
    while len(json_bytes(projected)) > target_budget_bytes and removable:
        _completed, hypothesis_id, scan_id = removable.pop(0)
        hazard = by_id[hypothesis_id]
        hazard[SCAN_HISTORY_FIELD] = [
            item
            for item in hazard[SCAN_HISTORY_FIELD]
            if item.get("scan_id") != scan_id
        ]
        _refresh_projection_metadata(
            projected,
            histories=histories,
            reference_reasons=reference_reasons,
            detail_focus_reasons=detail_focus_reasons,
            unresolved_focus_ids=unresolved_focus_ids,
            target_budget_bytes=target_budget_bytes,
            hard_budget_bytes=hard_budget_bytes,
        )

    compactable = []
    for hazard in projected_hazards:
        hypothesis_id = hazard["hypothesis_id"]
        for attempt in hazard[SCAN_HISTORY_FIELD]:
            scan_id = attempt.get("scan_id")
            if (
                scan_id in protected_scan_ids
                and attempt.get("detail_projection")
                != COMPACT_ROUTE_SCAN_DETAIL
            ):
                completed_at_ms = attempt.get("completed_at_ms")
                compactable.append((
                    completed_at_ms
                    if isinstance(completed_at_ms, int)
                    and not isinstance(completed_at_ms, bool)
                    else -1,
                    hypothesis_id,
                    scan_id if isinstance(scan_id, str) else "",
                ))
    compactable.sort()
    while len(json_bytes(projected)) > target_budget_bytes and compactable:
        _completed, hypothesis_id, scan_id = compactable.pop(0)
        hazard = by_id[hypothesis_id]
        hazard[SCAN_HISTORY_FIELD] = [
            (
                _compact_route_scan_detail(item)
                if item.get("scan_id") == scan_id
                else item
            )
            for item in hazard[SCAN_HISTORY_FIELD]
        ]
        _refresh_projection_metadata(
            projected,
            histories=histories,
            reference_reasons=reference_reasons,
            detail_focus_reasons=detail_focus_reasons,
            unresolved_focus_ids=unresolved_focus_ids,
            target_budget_bytes=target_budget_bytes,
            hard_budget_bytes=hard_budget_bytes,
        )

    if (
        len(json_bytes(projected)) > target_budget_bytes
        and "experience_ledger" in navigation
    ):
        current_experience_bytes = len(json_bytes(
            projected["experience_ledger"]
        ))
        excess = len(json_bytes(projected)) - target_budget_bytes
        reduced_target = max(
            2 * 1024,
            current_experience_bytes - excess - 256,
        )
        try:
            reduced_experience = _project_experience_ledger(
                navigation["experience_ledger"],
                target_bytes=reduced_target,
            )
        except PhysicalPlannerContextError:
            projected["planner_context_projection"][
                "experience_target_reduction_blocked_by_mandatory_facts"
            ] = True
        else:
            projected["experience_ledger"] = reduced_experience
            _refresh_projection_metadata(
                projected,
                histories=histories,
                reference_reasons=reference_reasons,
                detail_focus_reasons=detail_focus_reasons,
                unresolved_focus_ids=unresolved_focus_ids,
                target_budget_bytes=target_budget_bytes,
                hard_budget_bytes=hard_budget_bytes,
            )
            projected["planner_context_projection"][
                "experience_target_reduction_blocked_by_mandatory_facts"
            ] = False

    size = len(json_bytes(projected))
    if size > target_budget_bytes:
        projected["planner_context_projection"][
            "target_budget_exceeded_due_to_mandatory_facts"
        ] = True
        size = len(json_bytes(projected))
    if size > hard_budget_bytes:
        raise PhysicalPlannerContextError(
            "mandatory navigation facts exceed the hard planner budget"
        )
    return projected


__all__ = (
    "COMPACT_ROUTE_SCAN_DETAIL",
    "PLANNER_PROJECTION_SCHEMA",
    "PhysicalPlannerContextError",
    "project_navigation_context",
)
