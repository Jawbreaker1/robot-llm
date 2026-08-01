"""Shared validation and mapping normalization for spatial contracts.

The helpers here deliberately return ordinary values or booleans.  Contract
classes remain responsible for their public error codes and messages.
"""

from typing import Mapping, Optional, Tuple

from .navigation_contract import NavigationContractError, identifier


_CIRCLE_COLLISION_FIELDS = {
    "geometry",
    "reference_point",
    "radius_mm",
}
_RECTANGLE_COLLISION_FIELDS = {
    "geometry",
    "reference_point",
    "front_extent_mm",
    "rear_extent_mm",
    "left_extent_mm",
    "right_extent_mm",
    "clearance_margin_mm",
    "calibration_status",
    "calibration_evidence",
}


def optional_identifier(
    name: str,
    value: Optional[str],
    maximum: int = 128,
) -> Optional[str]:
    if value is not None:
        identifier(name, value, maximum)
    return value


def boolean(name: str, value: bool) -> bool:
    if type(value) is not bool:
        raise NavigationContractError(
            "invalid_boolean",
            "{} is invalid".format(name),
        )
    return value


def is_unique_identifier_tuple(
    name: str,
    value: Tuple[str, ...],
    maximum: int = 96,
    require_nonempty: bool = False,
    required_members: Tuple[str, ...] = (),
) -> bool:
    """Return whether a tuple contains valid, unique identifiers.

    ``identifier`` intentionally remains the authority for invalid element
    types and values, preserving its existing contract errors.
    """

    if not isinstance(value, tuple):
        return False
    if require_nonempty and not value:
        return False
    if any(identifier(name, item, maximum) != item for item in value):
        return False
    return (
        len(set(value)) == len(value)
        and set(required_members).issubset(value)
    )


def normalize_collision_geometry_mapping(
    value: Mapping[str, object],
) -> Mapping[str, object]:
    """Copy an exact collision-geometry mapping after field validation."""

    if not isinstance(value, Mapping):
        raise NavigationContractError(
            "invalid_collision_geometry",
            "Collision geometry is invalid",
        )
    expected = {
        "SYMMETRIC_CIRCLE": _CIRCLE_COLLISION_FIELDS,
        "ASYMMETRIC_RECTANGLE": _RECTANGLE_COLLISION_FIELDS,
    }.get(value.get("geometry"), set())
    if set(value) != expected:
        raise NavigationContractError(
            "invalid_collision_geometry",
            "Collision geometry fields are invalid",
        )
    return dict(value)


def validate_evidence_sources(
    evidence_sources: Tuple[str, ...],
    map_quality: str,
) -> None:
    """Validate the provenance tuple and its map-quality normalization."""

    physical_ir = "physical_ir_reflection"
    if (
        not isinstance(evidence_sources, tuple)
        or any(
            source not in ("simulation_metric", physical_ir)
            for source in evidence_sources
        )
        or len(set(evidence_sources)) != len(evidence_sources)
    ):
        raise NavigationContractError(
            "invalid_spatial_evidence_sources",
            "Spatial map evidence sources are invalid",
        )
    expected_sources = {
        "EMPTY": (),
        "SIMULATION_METRIC": ("simulation_metric",),
        "PROVISIONAL_IR": (physical_ir,),
        "METRIC_WITH_PROVISIONAL_IR": (
            physical_ir,
            "simulation_metric",
        ),
    }[map_quality]
    if tuple(sorted(evidence_sources)) != expected_sources:
        raise NavigationContractError(
            "inconsistent_spatial_map_quality",
            "Map quality and evidence sources disagree",
        )
