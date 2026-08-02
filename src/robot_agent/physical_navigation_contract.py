"""Strict contracts shared by the host-side physical navigation modules.

The language model may choose semantic actions and author short conditional
plans.  It never chooses motor power, duration, safety limits, timestamps, or
state versions.  Those values belong to the EV3 worker and the host runtime.
"""

from copy import deepcopy
from dataclasses import dataclass
import json
from typing import Mapping, Optional, Tuple


DECISION_SCHEMA = "robot-physical-navigation-decision/v1"
REQUEST_SCHEMA = "ev3-agent-worker-request/v1"
RESPONSE_SCHEMA = "ev3-agent-worker-response/v2"

ADVANCE = "ADVANCE"
REVERSE = "REVERSE"
TURN_LEFT_90 = "TURN_LEFT_90"
TURN_RIGHT_90 = "TURN_RIGHT_90"
SCAN_FRONT_ARC = "SCAN_FRONT_ARC"
OBSERVE = "OBSERVE"
FINISH = "FINISH"

MOTION_ACTIONS = frozenset(
    (ADVANCE, REVERSE, TURN_LEFT_90, TURN_RIGHT_90)
)
NONMOTION_ACTIONS = frozenset((SCAN_FRONT_ARC, OBSERVE, FINISH))
ACTIONS = MOTION_ACTIONS | NONMOTION_ACTIONS
MAX_PLAN_ACTIONS = 3

SCAN_TURN_OPERATION = "scan_turn"
SCAN_SAMPLE_OPERATION = "scan_sample"
SCAN_TURN_PROFILE_ID = "ev3rstorm-provisional-ir-turn-v2"
SCAN_TURN_CALIBRATION = "provisional_live_encoder_derived"
SCAN_TURN_ALLOWED_DELTAS_MDEG = tuple(
    value
    for value in range(-120_000, 120_001, 15_000)
    if value != 0
)
SCAN_TURN_REFERENCE_BODY_MDEG = 90_000
SCAN_TURN_REFERENCE_ENCODER_DEGREES = 682
SCAN_TURN_REFERENCE_DURATION_MS = 2_560
SCAN_TURN_SPEED_DPS = 250
SCAN_TURN_MAX_SLICE_DURATION_MS = 800
SCAN_TURN_MAX_SIDE_DIVERGENCE_DEGREES = 80
SCAN_SAMPLE_COUNT = 5
SCAN_SAMPLE_FILTER_WINDOW = 3
SCAN_SAMPLE_INTERVAL_MS = 30
SCAN_SAMPLE_SETTLED_DURATION_MS = (
    (SCAN_SAMPLE_COUNT - 1) * SCAN_SAMPLE_INTERVAL_MS
)

EXPECTED_WORKER_SAFETY = {
    "lifetime_motor_lock": True,
    "bound_hardware_topology": True,
    "touch_interrupts_all_motion": True,
    "infrared_blocks_and_interrupts_advance": True,
    "infrared_does_not_block_turns": True,
    "process_signals_interrupt_active_pulses": True,
    "channel_close_interrupts_active_pulses": True,
    "worker_selects_actions": False,
}
EXPECTED_WORKER_OPERATIONS = frozenset(
    (
        "describe",
        "observe",
        "pulse",
        SCAN_TURN_OPERATION,
        SCAN_SAMPLE_OPERATION,
        "stop",
        "shutdown",
    )
)

REASON_CODES = frozenset(
    (
        "PROGRESS_GOAL",
        "PROBE_CLEARANCE",
        "HANDLE_OBSTACLE",
        "VERIFY_RESULT",
        "COMPLETE_GOAL",
    )
)

# These are identity checks for the fixed semantic actions exposed by the EV3
# worker.  They are not host-selected motor commands.
EXPECTED_ACTION_SPECS = {
    ADVANCE: {
        "left_speed_dps": 800,
        "right_speed_dps": 800,
        "slice_durations_ms": [250],
        "slice_count": 1,
        "total_duration_ms": 250,
        "estimated_body_turn_degrees": None,
        "target_mean_abs_encoder_degrees": None,
        "calibration_evidence": None,
        "calibration": "not_applicable",
    },
    REVERSE: {
        "left_speed_dps": -800,
        "right_speed_dps": -800,
        "slice_durations_ms": [250],
        "slice_count": 1,
        "total_duration_ms": 250,
        "estimated_body_turn_degrees": None,
        "target_mean_abs_encoder_degrees": None,
        "calibration_evidence": None,
        "calibration": "not_applicable",
    },
    TURN_LEFT_90: {
        "left_speed_dps": -250,
        "right_speed_dps": 250,
        "slice_durations_ms": [800, 800, 800, 160],
        "slice_count": 4,
        "total_duration_ms": 2560,
        "estimated_body_turn_degrees": 90,
        "target_mean_abs_encoder_degrees": 682,
        "calibration_evidence": {
            "source_action": "live_turn_left_segment",
            "source_speed_dps": 250,
            "source_duration_ms": 800,
            "source_left_encoder_delta_degrees": -210,
            "source_right_encoder_delta_degrees": 219,
            "source_mean_abs_encoder_delta_degrees": 214.5,
            "right_turn_is_mirrored": False,
        },
        "calibration": "provisional_live_encoder_derived",
    },
    TURN_RIGHT_90: {
        "left_speed_dps": 250,
        "right_speed_dps": -250,
        "slice_durations_ms": [800, 800, 800, 160],
        "slice_count": 4,
        "total_duration_ms": 2560,
        "estimated_body_turn_degrees": -90,
        "target_mean_abs_encoder_degrees": 682,
        "calibration_evidence": {
            "source_action": "live_turn_left_segment",
            "source_speed_dps": 250,
            "source_duration_ms": 800,
            "source_left_encoder_delta_degrees": -210,
            "source_right_encoder_delta_degrees": 219,
            "source_mean_abs_encoder_delta_degrees": 214.5,
            "right_turn_is_mirrored": True,
        },
        "calibration": "provisional_live_encoder_derived",
    },
}


def _rounded_ratio(numerator: int, denominator: int) -> int:
    return (numerator + denominator // 2) // denominator


def _scan_turn_durations(total_duration_ms: int):
    count = max(
        1,
        (
            total_duration_ms
            + SCAN_TURN_MAX_SLICE_DURATION_MS
            - 1
        ) // SCAN_TURN_MAX_SLICE_DURATION_MS,
    )
    base, extra = divmod(total_duration_ms, count)
    return [base + (1 if index < extra else 0) for index in range(count)]


def expected_scan_turn_spec(relative_delta_mdeg: int):
    if (
        isinstance(relative_delta_mdeg, bool)
        or not isinstance(relative_delta_mdeg, int)
        or relative_delta_mdeg not in SCAN_TURN_ALLOWED_DELTAS_MDEG
    ):
        raise PhysicalNavigationContractError(
            "invalid_scan_turn",
            "Relative scan turn is outside the fixed host profile",
        )
    magnitude = abs(relative_delta_mdeg)
    duration_ms = _rounded_ratio(
        SCAN_TURN_REFERENCE_DURATION_MS * magnitude,
        SCAN_TURN_REFERENCE_BODY_MDEG,
    )
    target_encoder_degrees = _rounded_ratio(
        SCAN_TURN_REFERENCE_ENCODER_DEGREES * magnitude,
        SCAN_TURN_REFERENCE_BODY_MDEG,
    )
    durations = _scan_turn_durations(duration_ms)
    direction = 1 if relative_delta_mdeg > 0 else -1
    return {
        "relative_delta_mdeg": relative_delta_mdeg,
        "left_speed_dps": -SCAN_TURN_SPEED_DPS * direction,
        "right_speed_dps": SCAN_TURN_SPEED_DPS * direction,
        "slice_durations_ms": durations,
        "slice_count": len(durations),
        "total_duration_ms": duration_ms,
        "target_mean_abs_encoder_degrees": target_encoder_degrees,
        "calibration": SCAN_TURN_CALIBRATION,
        "profile_id": SCAN_TURN_PROFILE_ID,
    }


def expected_scan_turn_profile():
    return {
        "profile_id": SCAN_TURN_PROFILE_ID,
        "calibration": SCAN_TURN_CALIBRATION,
        "allowed_relative_deltas_mdeg": list(
            SCAN_TURN_ALLOWED_DELTAS_MDEG
        ),
        "reference_body_turn_mdeg": (
            SCAN_TURN_REFERENCE_BODY_MDEG
        ),
        "reference_mean_abs_encoder_degrees": (
            SCAN_TURN_REFERENCE_ENCODER_DEGREES
        ),
        "reference_duration_ms": SCAN_TURN_REFERENCE_DURATION_MS,
        "speed_dps": SCAN_TURN_SPEED_DPS,
        "max_slice_duration_ms": SCAN_TURN_MAX_SLICE_DURATION_MS,
        "max_side_divergence_degrees": (
            SCAN_TURN_MAX_SIDE_DIVERGENCE_DEGREES
        ),
        "turns": [
            expected_scan_turn_spec(value)
            for value in SCAN_TURN_ALLOWED_DELTAS_MDEG
        ],
    }


def expected_scan_sample_profile():
    return {
        "sample_count": SCAN_SAMPLE_COUNT,
        "filter_window_samples": SCAN_SAMPLE_FILTER_WINDOW,
        "sample_interval_ms": SCAN_SAMPLE_INTERVAL_MS,
        "settled_duration_ms": SCAN_SAMPLE_SETTLED_DURATION_MS,
        "motors_stopped_before_sampling": True,
        "filter_history_reset_before_sampling": True,
    }


class PhysicalNavigationContractError(ValueError):
    """Untrusted input violated a physical navigation boundary."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _integer(name: str, value: object, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise PhysicalNavigationContractError(
            "invalid_{}".format(name),
            "{} is invalid".format(name),
        )
    return value


def _text(name: str, value: object, maximum: int) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or any(ord(character) < 32 for character in value)
    ):
        raise PhysicalNavigationContractError(
            "invalid_{}".format(name),
            "{} is invalid".format(name),
        )
    return value


def strict_json_loads(raw: bytes, maximum_bytes: int) -> object:
    """Decode bounded UTF-8 JSON while rejecting duplicates and NaN."""

    if (
        not isinstance(raw, bytes)
        or not raw
        or len(raw) > maximum_bytes
    ):
        raise PhysicalNavigationContractError(
            "invalid_json_body",
            "JSON body is empty, oversized, or not bytes",
        )

    def reject_constant(_value):
        raise ValueError("non-finite number")

    def reject_duplicate_keys(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("duplicate key")
            result[key] = value
        return result

    try:
        return json.loads(
            raw.decode("utf-8"),
            parse_constant=reject_constant,
            object_pairs_hook=reject_duplicate_keys,
        )
    except (UnicodeDecodeError, TypeError, ValueError):
        raise PhysicalNavigationContractError(
            "invalid_json",
            "Body was not strict UTF-8 JSON",
        ) from None


def json_bytes(value: object) -> bytes:
    try:
        return json.dumps(
            value,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    except (TypeError, ValueError):
        raise PhysicalNavigationContractError(
            "not_json_safe",
            "Local value could not be serialized safely",
        ) from None


def validate_observation(value: object) -> Mapping[str, object]:
    """Validate the safety-relevant subset of one worker observation."""

    required = {
        "state_version",
        "observed_monotonic_ms",
        "touch",
        "infrared",
        "motors",
        "last_outcome",
        "budgets",
    }
    if not isinstance(value, dict) or set(value) != required:
        raise PhysicalNavigationContractError(
            "invalid_observation_fields",
            "Observation fields did not exactly match the worker contract",
        )
    _integer("state_version", value["state_version"], 1)
    _integer("observation_timestamp", value["observed_monotonic_ms"])

    touch = value["touch"]
    if (
        not isinstance(touch, dict)
        or set(touch) != {"value0", "pressed"}
        or touch["value0"] not in (0, 1)
        or type(touch["pressed"]) is not bool
        or touch["pressed"] != (touch["value0"] == 1)
    ):
        raise PhysicalNavigationContractError(
            "invalid_touch_observation",
            "Touch observation is invalid",
        )

    infrared = value["infrared"]
    if (
        not isinstance(infrared, dict)
        or set(infrared)
        != {"raw", "filtered", "blocked", "reason", "sample_count"}
        or type(infrared["blocked"]) is not bool
    ):
        raise PhysicalNavigationContractError(
            "invalid_infrared_observation",
            "Infrared observation is invalid",
        )
    for field in ("raw", "filtered"):
        reading = infrared[field]
        if reading is not None and (
            isinstance(reading, bool)
            or not isinstance(reading, int)
            or not 0 <= reading <= 100
        ):
            raise PhysicalNavigationContractError(
                "invalid_infrared_reading",
                "Infrared reading is invalid",
            )
    _text("infrared_reason", infrared["reason"], 80)
    _integer("infrared_sample_count", infrared["sample_count"])

    motors = value["motors"]
    if not isinstance(motors, list) or not 1 <= len(motors) <= 8:
        raise PhysicalNavigationContractError(
            "invalid_motor_observation",
            "Motor observation is invalid",
        )
    roles = set()
    for motor in motors:
        if (
            not isinstance(motor, dict)
            or set(motor) != {"role", "position", "state"}
        ):
            raise PhysicalNavigationContractError(
                "invalid_motor_fields",
                "Motor fields are invalid",
            )
        role = _text("motor_role", motor["role"], 64)
        if role in roles:
            raise PhysicalNavigationContractError(
                "duplicate_motor_role",
                "Motor roles must be unique",
            )
        roles.add(role)
        if isinstance(motor["position"], bool) or not isinstance(
            motor["position"], int
        ):
            raise PhysicalNavigationContractError(
                "invalid_motor_position",
                "Motor position is invalid",
            )
        if not isinstance(motor["state"], str) or len(motor["state"]) > 128:
            raise PhysicalNavigationContractError(
                "invalid_motor_state",
                "Motor state is invalid",
            )

    budgets = value["budgets"]
    required_budgets = {
        "pulse_count",
        "pulse_count_remaining",
        "pulse_duration_ms",
        "pulse_duration_ms_remaining",
        "process_ms_remaining",
        "motion_fault_latched",
    }
    if (
        not isinstance(value["last_outcome"], dict)
        or not isinstance(budgets, dict)
        or set(budgets) != required_budgets
        or type(budgets["motion_fault_latched"]) is not bool
    ):
        raise PhysicalNavigationContractError(
            "invalid_worker_budget",
            "Worker budget is invalid",
        )
    for field in required_budgets - {"motion_fault_latched"}:
        _integer(field, budgets[field])
    return deepcopy(value)


@dataclass(frozen=True)
class NavigationDecision:
    """One strictly correlated model decision."""

    episode_id: str
    turn: int
    based_on_state_version: int
    action: str
    plan: Tuple[str, ...]
    reason_code: str
    assessment: str
    utterance: Optional[str]
    perception_target_hypothesis_id: Optional[str]
    maneuver_commitment: Mapping[str, object]

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        episode_id: str,
        turn: int,
        state_version: int,
        available_actions=ACTIONS,
        published_target_ids=(),
    ):
        fields = {
            "schema",
            "episode_id",
            "turn",
            "based_on_state_version",
            "action",
            "plan",
            "reason_code",
            "assessment",
            "utterance",
            "perception_target_hypothesis_id",
            "maneuver_commitment",
        }
        if not isinstance(value, dict) or set(value) != fields:
            raise PhysicalNavigationContractError(
                "invalid_decision_fields",
                "Decision fields did not exactly match the contract",
            )
        if (
            value["schema"] != DECISION_SCHEMA
            or value["episode_id"] != episode_id
            or value["turn"] != turn
            or value["based_on_state_version"] != state_version
        ):
            raise PhysicalNavigationContractError(
                "decision_correlation_mismatch",
                "Decision constants did not match the current turn",
            )
        action = value["action"]
        available = frozenset(available_actions)
        if action not in available:
            raise PhysicalNavigationContractError(
                "unavailable_action",
                "Decision selected an unavailable action",
            )
        plan = value["plan"]
        if (
            not isinstance(plan, list)
            or not 1 <= len(plan) <= MAX_PLAN_ACTIONS
            or plan[0] != action
            or any(item not in available for item in plan)
        ):
            raise PhysicalNavigationContractError(
                "invalid_exact_plan",
                "Plan must start with the selected available action",
            )
        if action in MOTION_ACTIONS:
            if len(plan) < 2 or any(item not in MOTION_ACTIONS for item in plan):
                raise PhysicalNavigationContractError(
                    "motion_plan_requires_tail",
                    "Motion requires an exact two- or three-action motion plan",
                )
        elif len(plan) != 1:
            raise PhysicalNavigationContractError(
                "nonmotion_plan_must_be_singleton",
                "Observation, scan, and finish plans must be singleton",
            )

        target = value["perception_target_hypothesis_id"]
        published_targets = frozenset(published_target_ids)
        if action == SCAN_FRONT_ARC:
            if target not in published_targets:
                raise PhysicalNavigationContractError(
                    "invalid_scan_target",
                    "Scan target must name a published hypothesis",
                )
        elif target is not None:
            raise PhysicalNavigationContractError(
                "unexpected_perception_target",
                "Only SCAN_FRONT_ARC may name a perception target",
            )
        reason = value["reason_code"]
        if reason not in REASON_CODES:
            raise PhysicalNavigationContractError(
                "invalid_reason_code",
                "Decision reason code is invalid",
            )
        assessment = _text("assessment", value["assessment"], 160)
        utterance = value["utterance"]
        if utterance is not None:
            utterance = _text("utterance", utterance, 160)
        commitment = value["maneuver_commitment"]
        if not isinstance(commitment, dict):
            raise PhysicalNavigationContractError(
                "invalid_maneuver_commitment",
                "Maneuver commitment must be an object",
            )
        return cls(
            episode_id=episode_id,
            turn=turn,
            based_on_state_version=state_version,
            action=action,
            plan=tuple(plan),
            reason_code=reason,
            assessment=assessment,
            utterance=utterance,
            perception_target_hypothesis_id=target,
            maneuver_commitment=deepcopy(commitment),
        )

    def to_dict(self) -> Mapping[str, object]:
        return {
            "schema": DECISION_SCHEMA,
            "episode_id": self.episode_id,
            "turn": self.turn,
            "based_on_state_version": self.based_on_state_version,
            "action": self.action,
            "plan": list(self.plan),
            "reason_code": self.reason_code,
            "assessment": self.assessment,
            "utterance": self.utterance,
            "perception_target_hypothesis_id": (
                self.perception_target_hypothesis_id
            ),
            "maneuver_commitment": deepcopy(self.maneuver_commitment),
        }


def observation_safety_signature(
    observation: Mapping[str, object],
) -> Tuple[bool, bool, bool]:
    checked = validate_observation(observation)
    return (
        checked["touch"]["pressed"],
        checked["infrared"]["blocked"],
        checked["budgets"]["motion_fault_latched"],
    )


def motion_budget_allows(
    action: str,
    observation: Mapping[str, object],
    action_specs: Mapping[str, Mapping[str, object]],
) -> bool:
    if action not in MOTION_ACTIONS:
        return True
    if action not in action_specs:
        return False
    checked = validate_observation(observation)
    spec = action_specs[action]
    return (
        checked["budgets"]["motion_fault_latched"] is False
        and spec["slice_count"]
        <= checked["budgets"]["pulse_count_remaining"]
        and spec["total_duration_ms"]
        <= checked["budgets"]["pulse_duration_ms_remaining"]
    )
