"""Connected-component object hypotheses from occupied map cells."""

from dataclasses import dataclass
import hashlib
from typing import Set, Tuple

from .spatial_map_contract import (
    SEMANTIC_UNKNOWN,
    ObjectHypothesis,
)


@dataclass(frozen=True)
class OccupiedCellEvidence:
    """Minimal immutable evidence needed by the clustering pass."""

    grid_x: int
    grid_y: int
    occupancy_milli: int
    first_occupied_at_ms: int
    last_occupied_at_ms: int
    occupied_evidence_count: int
    provenance: Tuple[str, ...]
    trusted_simulator_object_ids: Tuple[str, ...]

    @property
    def coordinate(self) -> Tuple[int, int]:
        return (self.grid_x, self.grid_y)


def _neighbors(
    coordinate: Tuple[int, int],
) -> Tuple[Tuple[int, int], ...]:
    x, y = coordinate
    return tuple(
        (x + dx, y + dy)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        if dx or dy
    )


def _components(
    by_coordinate,
) -> Tuple[Tuple[Tuple[int, int], ...], ...]:
    occupied = set(by_coordinate)
    components = []
    while occupied:
        seed = min(occupied)
        occupied.remove(seed)
        pending = [seed]
        component = []
        while pending:
            coordinate = pending.pop()
            component.append(coordinate)
            for neighbor in _neighbors(coordinate):
                if neighbor in occupied:
                    occupied.remove(neighbor)
                    pending.append(neighbor)
        components.append(tuple(sorted(component)))
    return tuple(components)


def _hypothesis_id(
    map_id: str,
    robot_id: str,
    controller_instance_id: str,
    frame_id: str,
    identity_basis: Tuple[str, ...],
) -> str:
    identity = "\0".join((
        map_id,
        robot_id,
        controller_instance_id,
        frame_id,
    ) + identity_basis).encode("utf-8")
    return "object-{}".format(
        hashlib.sha256(identity).hexdigest()[:20]
    )


def _oldest_supported_record(records):
    """Choose an anchor that cannot move merely because a component grows."""

    return min(
        records,
        key=lambda item: (
            item.first_occupied_at_ms,
            item.grid_x,
            item.grid_y,
        ),
    )


def connected_object_hypotheses(
    map_id: str,
    robot_id: str,
    controller_instance_id: str,
    frame_id: str,
    resolution_mm: int,
    cells: Tuple[OccupiedCellEvidence, ...],
) -> Tuple[ObjectHypothesis, ...]:
    """Return deterministic eight-connected occupied components."""

    by_coordinate = {
        item.coordinate: item
        for item in cells
    }
    components = []
    trusted_component_counts = {}
    for component in _components(by_coordinate):
        records = [by_coordinate[item] for item in component]
        trusted_ids: Set[str] = set()
        provenance: Set[str] = set()
        for record in records:
            trusted_ids.update(
                record.trusted_simulator_object_ids
            )
            provenance.update(record.provenance)
        trusted_object_id = (
            next(iter(trusted_ids))
            if len(trusted_ids) == 1
            else None
        )
        if trusted_object_id is not None:
            trusted_component_counts[trusted_object_id] = (
                trusted_component_counts.get(trusted_object_id, 0) + 1
            )
        components.append((
            component,
            records,
            provenance,
            trusted_object_id,
        ))

    hypotheses = []
    for (
        component,
        records,
        provenance,
        trusted_object_id,
    ) in components:
        anchor = _oldest_supported_record(records)
        if (
            trusted_object_id is not None
            and trusted_component_counts[trusted_object_id] == 1
        ):
            # Simulator IDs are opaque, authoritative identities.  Do not
            # couple their host hypothesis ID to a component's mutable bounds.
            identity_basis = ("TRUSTED", trusted_object_id)
        else:
            # Unknown components use their oldest still-supported occupied
            # evidence.  Later growth in any coordinate direction is stable;
            # clearing that evidence deliberately starts a new identity.
            identity_basis = (
                (
                    "TRUSTED_COMPONENT"
                    if trusted_object_id is not None
                    else "OCCUPIED_EVIDENCE"
                ),
                trusted_object_id or "",
                str(anchor.first_occupied_at_ms),
                str(anchor.grid_x),
                str(anchor.grid_y),
            )
        center_x_values = [
            item[0] * resolution_mm + resolution_mm // 2
            for item in component
        ]
        center_y_values = [
            item[1] * resolution_mm + resolution_mm // 2
            for item in component
        ]
        hypotheses.append(ObjectHypothesis(
            hypothesis_id=_hypothesis_id(
                map_id,
                robot_id,
                controller_instance_id,
                frame_id,
                identity_basis,
            ),
            frame_id=frame_id,
            semantic_label=SEMANTIC_UNKNOWN,
            min_x_mm=min(item[0] for item in component)
            * resolution_mm,
            min_y_mm=min(item[1] for item in component)
            * resolution_mm,
            max_x_mm=(max(item[0] for item in component) + 1)
            * resolution_mm,
            max_y_mm=(max(item[1] for item in component) + 1)
            * resolution_mm,
            centroid_x_mm=int(round(
                sum(center_x_values) / len(center_x_values)
            )),
            centroid_y_mm=int(round(
                sum(center_y_values) / len(center_y_values)
            )),
            cell_count=len(component),
            first_seen_at_ms=min(
                record.first_occupied_at_ms
                for record in records
            ),
            last_seen_at_ms=max(
                record.last_occupied_at_ms
                for record in records
            ),
            evidence_count=sum(
                record.occupied_evidence_count
                for record in records
            ),
            confidence_milli=max(
                1,
                min(
                    1_000,
                    int(round(
                        sum(
                            record.occupancy_milli
                            for record in records
                        )
                        / len(records)
                    )),
                ),
            ),
            provenance=tuple(sorted(provenance)),
            trusted_simulator_object_id=trusted_object_id,
        ))
    return tuple(sorted(
        hypotheses,
        key=lambda item: item.hypothesis_id,
    ))
