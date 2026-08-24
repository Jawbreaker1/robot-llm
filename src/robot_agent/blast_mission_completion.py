"""Host-owned terminal eligibility for one BLAST directional mission."""


BLAST_GOAL_RADIUS_MM = 120
BLAST_GOAL_HEADING_TOLERANCE_MDEG = 20_000


def blast_directional_completion_allowed(
    *, mission, pose, localization_valid, scan_fresh,
) -> bool:
    """Require a verified goal-line crossing before exposing COMPLETE."""

    return (
        localization_valid is True
        and scan_fresh is True
        and mission.heading_aligned(pose)
        and mission.longitudinal_progress_mm(pose)
        >= mission.minimum_forward_progress_mm
        and abs(mission.lateral_offset_mm(pose)) <= BLAST_GOAL_RADIUS_MM
    )


__all__ = (
    "BLAST_GOAL_HEADING_TOLERANCE_MDEG",
    "BLAST_GOAL_RADIUS_MM",
    "blast_directional_completion_allowed",
)
