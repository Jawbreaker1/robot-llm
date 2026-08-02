"""Language-independent progress facts for physical navigation."""

from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

from .physical_navigation_contract import validate_observation
from .physical_odometry import PhysicalPose


def observation_progress_signature(
    observation: Mapping[str, object],
    *,
    motor_roles: Optional[Tuple[str, ...]] = None,
) -> Tuple[Tuple[str, object], ...]:
    """Keep physical facts that can change a navigation decision.

    Worker versions, request budgets, and small changes in raw IR reflection
    prove freshness but do not make another stationary sample informative.
    """

    checked = validate_observation(observation)
    if motor_roles is not None and (
        not isinstance(motor_roles, tuple)
        or not motor_roles
        or len(set(motor_roles)) != len(motor_roles)
        or any(not isinstance(role, str) or not role for role in motor_roles)
    ):
        raise ValueError("progress motor roles are invalid")
    relevant_roles = (
        None if motor_roles is None else frozenset(motor_roles)
    )
    infrared = checked["infrared"]
    facts = {
        "infrared_available": (
            infrared["raw"] is not None
            or infrared["filtered"] is not None
        ),
        "infrared_blocked": infrared["blocked"],
        "touch_pressed": checked["touch"]["pressed"],
        "motion_fault_latched": checked["budgets"][
            "motion_fault_latched"
        ],
        "motor_positions": tuple(sorted(
            (motor["role"], motor["position"])
            for motor in checked["motors"]
            if relevant_roles is None or motor["role"] in relevant_roles
        )),
    }
    return tuple((key, facts[key]) for key in sorted(facts))


@dataclass(frozen=True)
class RestoredScanProgressBarrier:
    """Require a typed world or pose change before another active scan.

    Worker state versions and small raw-reflection changes deliberately do
    not rearm scanning.  Otherwise a fresh-but-identical OBSERVE would turn
    ``scan -> observe -> scan`` into the same non-agentic loop under a new
    sequence number.
    """

    scan_id: str
    target_hypothesis_id: str
    map_generation_id: str
    pose: PhysicalPose
    hazard_ids: Tuple[str, ...]
    observation_signature: Tuple[Tuple[str, object], ...]
    motor_roles: Optional[Tuple[str, ...]] = None

    def __post_init__(self) -> None:
        if (
            not isinstance(self.scan_id, str)
            or not self.scan_id
            or not isinstance(self.target_hypothesis_id, str)
            or not self.target_hypothesis_id
            or not isinstance(self.map_generation_id, str)
            or not self.map_generation_id
            or not isinstance(self.pose, PhysicalPose)
            or tuple(sorted(set(self.hazard_ids))) != self.hazard_ids
            or any(
                not isinstance(hypothesis_id, str) or not hypothesis_id
                for hypothesis_id in self.hazard_ids
            )
            or not isinstance(self.observation_signature, tuple)
            or (
                self.motor_roles is not None
                and (
                    not isinstance(self.motor_roles, tuple)
                    or not self.motor_roles
                    or len(set(self.motor_roles)) != len(self.motor_roles)
                    or any(
                        not isinstance(role, str) or not role
                        for role in self.motor_roles
                    )
                )
            )
        ):
            raise ValueError("restored scan progress barrier is invalid")

    def rearm_reason(
        self,
        *,
        map_generation_id: str,
        pose: PhysicalPose,
        hazard_ids: Tuple[str, ...],
        observation: Mapping[str, object],
    ) -> Optional[str]:
        """Return the first typed progress fact that permits a new scan."""

        if map_generation_id != self.map_generation_id:
            return "MAP_GENERATION_CHANGED"
        if pose != self.pose:
            return "VERIFIED_POSE_CHANGED"
        if tuple(sorted(hazard_ids)) != self.hazard_ids:
            return "TARGET_HYPOTHESES_CHANGED"
        # A restored active scan changes the drive encoder anchors even when
        # the verified robot pose is unchanged.  PhysicalPose above owns
        # navigation progress; treating those anchor-only changes as progress
        # lets scans of two targets unlock each other forever.  Sensor and
        # fault facts remain decision-relevant here.
        prior_observation = tuple(
            item
            for item in self.observation_signature
            if item[0] != "motor_positions"
        )
        current_observation = tuple(
            item
            for item in observation_progress_signature(
                observation,
                motor_roles=self.motor_roles,
            )
            if item[0] != "motor_positions"
        )
        if current_observation != prior_observation:
            return "DECISION_RELEVANT_OBSERVATION_CHANGED"
        return None


def observation_information_result(
    before: Mapping[str, object],
    after: Mapping[str, object],
    *,
    motor_roles: Optional[Tuple[str, ...]] = None,
) -> Mapping[str, object]:
    prior = dict(observation_progress_signature(
        before,
        motor_roles=motor_roles,
    ))
    current = dict(observation_progress_signature(
        after,
        motor_roles=motor_roles,
    ))
    changed = sorted(key for key in current if current[key] != prior[key])
    return {
        "information_gain": (
            "DECISION_RELEVANT_CHANGE" if changed else "NONE"
        ),
        "changed_facts": changed,
    }


def observe_without_information_gain(value: object) -> bool:
    return (
        isinstance(value, Mapping)
        and value.get("operation") == "observe"
        and value.get("information_gain") == "NONE"
    )


__all__ = (
    "RestoredScanProgressBarrier",
    "observation_information_result",
    "observation_progress_signature",
    "observe_without_information_gain",
)
