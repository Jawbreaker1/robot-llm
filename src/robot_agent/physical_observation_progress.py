"""Language-independent information-gain facts for physical observations."""

from typing import Mapping

from .physical_navigation_contract import validate_observation


def _signature(observation: Mapping[str, object]) -> Mapping[str, object]:
    """Keep physical facts that can change a navigation decision.

    Worker versions, request budgets, and small changes in raw IR reflection
    prove freshness but do not make another stationary sample informative.
    """

    checked = validate_observation(observation)
    infrared = checked["infrared"]
    return {
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
        )),
    }


def observation_information_result(
    before: Mapping[str, object],
    after: Mapping[str, object],
) -> Mapping[str, object]:
    prior = _signature(before)
    current = _signature(after)
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
    "observation_information_result",
    "observe_without_information_gain",
)
