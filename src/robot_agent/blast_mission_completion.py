"""Host-owned terminal eligibility for one BLAST directional mission."""


# One coarse navigation-grid cell: intentionally roomy for a LEGO platform.
BLAST_GOAL_RADIUS_MM = 150
BLAST_GOAL_HEADING_TOLERANCE_MDEG = 20_000


def blast_directional_completion_allowed(
    *, mission, pose, localization_valid,
) -> bool:
    """Expose COMPLETE inside the coarse verified LEGO goal region."""

    return (
        localization_valid is True
        and mission.heading_aligned(pose)
        and mission.distance_to_target_mm(pose) <= BLAST_GOAL_RADIUS_MM
    )


__all__ = (
    "BLAST_GOAL_HEADING_TOLERANCE_MDEG",
    "BLAST_GOAL_RADIUS_MM",
    "blast_directional_completion_allowed",
)
