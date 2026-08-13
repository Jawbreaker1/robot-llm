"""Host-owned terminal eligibility for one BLAST directional mission."""


def blast_directional_completion_allowed(
    *, mission, pose, localization_valid, scan_fresh,
) -> bool:
    """Require verified progress and alignment before exposing COMPLETE."""

    return (
        localization_valid is True
        and scan_fresh is True
        and mission.heading_aligned(pose)
        and mission.longitudinal_progress_mm(pose)
        >= mission.minimum_forward_progress_mm
    )


__all__ = ("blast_directional_completion_allowed",)
