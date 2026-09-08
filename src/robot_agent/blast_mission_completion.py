"""Host-owned terminal eligibility for one BLAST directional mission."""


# Goal arrival is independent of map resolution. About one 45 mm drive pulse
# leaves LEGO-scale slack without skipping the final approach after a turn.
BLAST_GOAL_RADIUS_MM = 50
BLAST_GOAL_HEADING_TOLERANCE_MDEG = 20_000


def blast_directional_completion_allowed(
    *, mission, pose, localization_valid,
) -> bool:
    """Expose COMPLETE inside the verified physical goal region."""

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
