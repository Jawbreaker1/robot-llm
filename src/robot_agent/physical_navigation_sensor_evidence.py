"""Source-labelled sensor facts used by physical navigation policy."""

from dataclasses import dataclass
from typing import Mapping, Optional


EV3_TOUCH_SOURCE = "ev3_touch"
EV3_IR_PROXIMITY_SOURCE = "ev3_ir_proximity"


@dataclass(frozen=True)
class PhysicalContactEvidence:
    source: str
    pressed: bool


@dataclass(frozen=True)
class PhysicalClearanceEvidence:
    source: str
    sample_available: bool
    blocked: bool
    distance_mm: Optional[int] = None


@dataclass(frozen=True)
class PhysicalSensorEvidence:
    contact: Optional[PhysicalContactEvidence]
    clearance: PhysicalClearanceEvidence


def sensor_evidence_from_validated_ev3_observation(
    observation: Mapping[str, object],
) -> PhysicalSensorEvidence:
    """Project an observation that already passed the EV3 validator.

    Absence of contact evidence remains representable for controllers that do
    not have a contact sensor.  EV3 IR proximity is deliberately not presented
    as a metric distance.
    """

    touch = observation["touch"]
    infrared = observation["infrared"]
    return PhysicalSensorEvidence(
        contact=PhysicalContactEvidence(
            source=EV3_TOUCH_SOURCE,
            pressed=touch["pressed"],
        ),
        clearance=PhysicalClearanceEvidence(
            source=EV3_IR_PROXIMITY_SOURCE,
            sample_available=(
                infrared["raw"] is not None
                or infrared["filtered"] is not None
            ),
            blocked=infrared["blocked"],
        ),
    )
