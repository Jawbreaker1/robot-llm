"""Strict host-side validation for EV3 pulse execution evidence."""

from typing import Callable, Mapping, Optional

from .physical_navigation_contract import (
    EXPECTED_ACTION_SPECS,
    validate_observation,
)


ACTIVE_MOTOR_STATES = frozenset(("running", "ramping"))
# EV3 tacho positions are separate sysfs reads.  A motor that has already
# passed the verified brake/stop loop can publish one last encoder tick
# between the receipt snapshot and the immediately following observation.
# Keep that non-atomic settling window small and explicit; larger movement,
# active state, or a net direction reversal remains a contract failure.
MAX_FINAL_ENCODER_SETTLING_DEGREES = 5


class EV3PulseContractError(ValueError):
    """A successful worker response violated the pulse evidence contract."""


def _validate_motor_receipts(
    motors: object,
    *,
    allowed_lengths,
    expected_roles: Mapping[str, str],
    expected_directions: Mapping[str, int],
    direction_sides,
    container_message: str,
    item_message: str,
    sides_message: str,
):
    if not isinstance(motors, list) or len(motors) not in allowed_lengths:
        raise EV3PulseContractError(container_message)
    by_side = {}
    roles = set()
    fields = {
        "side",
        "role",
        "position_before",
        "position_after",
        "position_delta",
        "state",
    }
    for motor in motors:
        if (
            not isinstance(motor, dict)
            or set(motor) != fields
            or motor["side"] not in ("left", "right")
            or motor["side"] in by_side
            or not isinstance(motor["role"], str)
            or not motor["role"]
            or motor["role"] in roles
            or (
                expected_roles
                and motor["role"] != expected_roles[motor["side"]]
            )
            or any(
                isinstance(motor[name], bool)
                or not isinstance(motor[name], int)
                for name in (
                    "position_before",
                    "position_after",
                    "position_delta",
                )
            )
            or motor["position_delta"]
            != motor["position_after"] - motor["position_before"]
            or not isinstance(motor["state"], str)
            or (
                motor["side"] in direction_sides
                and motor["position_delta"] != 0
                and motor["position_delta"]
                * expected_directions[motor["side"]]
                < 0
            )
        ):
            raise EV3PulseContractError(item_message)
        by_side[motor["side"]] = motor
        roles.add(motor["role"])
    if motors and set(by_side) != {"left", "right"}:
        raise EV3PulseContractError(sides_message)
    return by_side


def _validate_encoder_proof(
    verification: object,
    *,
    require_checks: bool,
    message: str,
) -> None:
    if (
        not isinstance(verification, dict)
        or set(verification) != {"passed", "error", "checks"}
        or type(verification["passed"]) is not bool
        or (
            verification["error"] is not None
            and not isinstance(verification["error"], str)
        )
        or not isinstance(verification["checks"], list)
        or (require_checks and not verification["checks"])
        or any(
            not isinstance(check, dict)
            or type(check.get("passed")) is not bool
            for check in verification["checks"]
        )
    ):
        raise EV3PulseContractError(message)


def _validate_partial_start_proof(segment, motors_by_side) -> None:
    commanded_sides = segment["commanded_sides"]
    verification = segment["encoder_verification"]
    checks = verification["checks"]
    if (
        segment["status"] != "interrupted"
        or verification["passed"] is not False
        or len(checks) != 3
    ):
        raise EV3PulseContractError(
            "pulse recovery segment encoder proof is invalid"
        )
    by_side = {}
    paired = None
    for check in checks:
        side = check.get("side")
        if side in ("left", "right") and side not in by_side:
            by_side[side] = check
        elif side == "paired" and paired is None:
            paired = check
        else:
            raise EV3PulseContractError(
                "pulse recovery segment encoder proof is invalid"
            )
    if (
        set(by_side) != {"left", "right"}
        or paired is None
        or any(
            set(by_side[side])
            != {"role", "side", "position_delta", "passed"}
            or by_side[side]["role"] != motors_by_side[side]["role"]
            or by_side[side]["position_delta"]
            != motors_by_side[side]["position_delta"]
            or by_side[side]["passed"] != (side in commanded_sides)
            for side in ("left", "right")
        )
        or set(paired) != {"role", "side", "position_delta", "passed"}
        or paired["role"] != "paired_start_completion"
        or paired["position_delta"] != 0
        or paired["passed"] is not False
    ):
        raise EV3PulseContractError(
            "pulse recovery segment encoder proof is invalid"
        )


def _validate_pulse_segments(
    segments: object,
    *,
    outer_motors,
    outer_status: str,
    outer_duration_ms: int,
    outer_started_monotonic_ms,
    outer_completed_monotonic_ms: int,
    outer_stop,
    expected_roles: Mapping[str, str],
    expected_directions: Mapping[str, int],
    stop_validator: Callable[[object], object],
) -> None:
    if not isinstance(segments, list):
        raise EV3PulseContractError(
            "pulse recovery segments are invalid"
        )
    if not segments:
        if outer_motors:
            raise EV3PulseContractError(
                "started pulse lacks temporal encoder segments"
            )
        return
    fields = {
        "segment_index",
        "kind",
        "commanded_sides",
        "duration_ms",
        "status",
        "reason",
        "started_monotonic_ms",
        "completed_monotonic_ms",
        "motors",
        "encoder_verification",
        "stop",
    }
    kinds = {
        "paired": ["left", "right"],
        "paired_retry": ["left", "right"],
        "left_catch_up": ["left"],
        "right_catch_up": ["right"],
    }
    previous_after = None
    previous_completed_ms = None
    first_before = None
    final_after = None
    for ordinal, segment in enumerate(segments, 1):
        if (
            not isinstance(segment, dict)
            or set(segment) != fields
            or isinstance(segment["segment_index"], bool)
            or segment["segment_index"] != ordinal
            or not (
                segment["kind"] in kinds
                and segment["commanded_sides"]
                == kinds[segment["kind"]]
                or segment["kind"] == "partial_start"
                and isinstance(segment["commanded_sides"], list)
                and 1 <= len(segment["commanded_sides"]) <= 2
                and segment["commanded_sides"]
                == [
                    side
                    for side in ("left", "right")
                    if side in segment["commanded_sides"]
                ]
            )
            or isinstance(segment["duration_ms"], bool)
            or not isinstance(segment["duration_ms"], int)
            or not 1 <= segment["duration_ms"] <= 800
            or segment["status"]
            not in ("completed", "interrupted", "verification_failed")
            or not isinstance(segment["reason"], str)
            or not segment["reason"]
            or isinstance(segment["started_monotonic_ms"], bool)
            or not isinstance(segment["started_monotonic_ms"], int)
            or isinstance(segment["completed_monotonic_ms"], bool)
            or not isinstance(segment["completed_monotonic_ms"], int)
            or segment["completed_monotonic_ms"]
            < segment["started_monotonic_ms"]
            or (
                previous_completed_ms is not None
                and segment["started_monotonic_ms"] < previous_completed_ms
            )
        ):
            raise EV3PulseContractError(
                "pulse recovery segment is invalid"
            )
        by_side = _validate_motor_receipts(
            segment["motors"],
            allowed_lengths=(2,),
            expected_roles=expected_roles,
            expected_directions=expected_directions,
            direction_sides=(
                segment["commanded_sides"]
                if isinstance(segment["encoder_verification"], dict)
                and segment["encoder_verification"].get("passed") is True
                else ()
            ),
            container_message=(
                "pulse recovery segment motors are invalid"
            ),
            item_message="pulse recovery segment motor is invalid",
            sides_message="pulse recovery segment sides are invalid",
        )
        before = {
            side: by_side[side]["position_before"]
            for side in ("left", "right")
        }
        after = {
            side: by_side[side]["position_after"]
            for side in ("left", "right")
        }
        if previous_after is not None and before != previous_after:
            raise EV3PulseContractError(
                "pulse recovery segments are not continuous"
            )
        if first_before is None:
            first_before = before
        previous_after = after
        final_after = after
        previous_completed_ms = segment["completed_monotonic_ms"]
        verification = segment["encoder_verification"]
        _validate_encoder_proof(
            verification,
            require_checks=True,
            message="pulse recovery segment encoder proof is invalid",
        )
        if (
            verification["passed"]
            and (
                verification["error"] is not None
                or any(
                    check["passed"] is not True
                    for check in verification["checks"]
                )
            )
            or (
                not verification["passed"]
                and (
                    not isinstance(verification["error"], str)
                    or not verification["error"]
                    or not any(
                        check["passed"] is False
                        for check in verification["checks"]
                    )
                )
            )
            or (
                segment["status"] == "completed"
                and verification["passed"] is not True
            )
            or (
                segment["status"] == "verification_failed"
                and verification["passed"] is not False
            )
            or (
                segment["kind"] == "partial_start"
                and (
                    ordinal != len(segments)
                    or segment["status"] != "interrupted"
                    or verification["passed"] is not False
                )
            )
        ):
            raise EV3PulseContractError(
                "pulse recovery segment encoder proof is invalid"
            )
        if segment["kind"] == "partial_start":
            _validate_partial_start_proof(segment, by_side)
        stop_validator(segment["stop"])

    outer_by_side = {
        motor["side"]: motor for motor in outer_motors
    }
    if (
        segments[0]["kind"] not in ("paired", "partial_start")
        or segments[0]["duration_ms"] != outer_duration_ms
        or any(segment["kind"] == "paired" for segment in segments[1:])
        or outer_started_monotonic_ms
        != segments[0]["started_monotonic_ms"]
        or outer_completed_monotonic_ms
        != segments[-1]["completed_monotonic_ms"]
        or (
            outer_status == "completed"
            and outer_stop != segments[-1]["stop"]
        )
        or set(outer_by_side) != {"left", "right"}
        or any(
            outer_by_side[side]["position_before"] != first_before[side]
            or outer_by_side[side]["position_after"] != final_after[side]
            for side in ("left", "right")
        )
        or (
            outer_status == "completed"
            and segments[-1]["status"] != "completed"
        )
        or (
            segments[-1]["kind"] == "partial_start"
            and outer_status != "interrupted"
        )
    ):
        raise EV3PulseContractError(
            "pulse recovery segment aggregate is invalid"
        )


def _validate_pulse_slice(
    value: object,
    *,
    ordinal: int,
    expected_count: int,
    expected_duration_ms: int,
    expected_roles: Mapping[str, str],
    expected_directions: Mapping[str, int],
    stop_validator: Callable[[object], object],
) -> None:
    fields = {
        "slice_index",
        "slice_count",
        "duration_ms",
        "status",
        "reason",
        "started_monotonic_ms",
        "completed_monotonic_ms",
        "motors",
        "segments",
        "encoder_verification",
        "stop",
    }
    statuses = {
        "completed",
        "interrupted",
        "denied",
        "verification_failed",
    }
    if (
        not isinstance(value, dict)
        or set(value) != fields
        or isinstance(value["slice_index"], bool)
        or not isinstance(value["slice_index"], int)
        or value["slice_index"] != ordinal
        or isinstance(value["slice_count"], bool)
        or not isinstance(value["slice_count"], int)
        or value["slice_count"] != expected_count
        or isinstance(value["duration_ms"], bool)
        or not isinstance(value["duration_ms"], int)
        or value["duration_ms"] != expected_duration_ms
        or value["status"] not in statuses
        or not isinstance(value["reason"], str)
        or not value["reason"]
        or (
            value["started_monotonic_ms"] is not None
            and (
                isinstance(value["started_monotonic_ms"], bool)
                or not isinstance(value["started_monotonic_ms"], int)
            )
        )
        or isinstance(value["completed_monotonic_ms"], bool)
        or not isinstance(value["completed_monotonic_ms"], int)
        or (
            value["started_monotonic_ms"] is not None
            and value["completed_monotonic_ms"]
            < value["started_monotonic_ms"]
        )
    ):
        raise EV3PulseContractError("pulse slice receipt is invalid")
    motors = value["motors"]
    _validate_motor_receipts(
        motors,
        allowed_lengths=(0, 2),
        expected_roles=expected_roles,
        expected_directions=expected_directions,
        direction_sides=(),
        container_message="pulse slice motors are invalid",
        item_message="pulse slice motor receipt is invalid",
        sides_message="pulse slice motor sides are invalid",
    )
    verification = value["encoder_verification"]
    status = value["status"]
    started = value["started_monotonic_ms"]
    _validate_encoder_proof(
        verification,
        require_checks=False,
        message="pulse slice encoder proof is invalid",
    )
    checks = verification["checks"]
    if status == "denied":
        valid_status_proof = (
            started is None
            and motors == []
            and verification["passed"] is False
            and verification["error"] is None
            and checks == []
        )
    else:
        valid_status_proof = started is not None and len(motors) == 2
        if status == "completed":
            valid_status_proof = (
                valid_status_proof
                and verification["passed"] is True
                and verification["error"] is None
                and bool(checks)
                and all(check["passed"] is True for check in checks)
                and all(
                    motor["position_delta"]
                    * expected_directions[motor["side"]]
                    > 0
                    for motor in motors
                )
            )
        elif status == "verification_failed":
            valid_status_proof = (
                valid_status_proof
                and verification["passed"] is False
                and isinstance(verification["error"], str)
                and bool(verification["error"])
                and bool(checks)
                and any(check["passed"] is False for check in checks)
            )
        elif verification["passed"]:
            valid_status_proof = (
                valid_status_proof
                and verification["error"] is None
                and bool(checks)
                and all(check["passed"] is True for check in checks)
            )
        else:
            valid_status_proof = (
                valid_status_proof
                and isinstance(verification["error"], str)
                and bool(verification["error"])
                and bool(checks)
                and any(check["passed"] is False for check in checks)
            )
    if not valid_status_proof:
        raise EV3PulseContractError(
            "pulse slice status proof is invalid"
        )
    _validate_pulse_segments(
        value["segments"],
        outer_motors=motors,
        outer_status=status,
        outer_duration_ms=value["duration_ms"],
        outer_started_monotonic_ms=started,
        outer_completed_monotonic_ms=value["completed_monotonic_ms"],
        outer_stop=value["stop"],
        expected_roles=expected_roles,
        expected_directions=expected_directions,
        stop_validator=stop_validator,
    )
    stop_validator(value["stop"])


def _drive_geometry(worker_description: object, action_spec):
    expected_roles = {}
    forward_signs = {"left": 1, "right": 1}
    if isinstance(worker_description, dict):
        geometry = worker_description.get("drive_geometry")
        if isinstance(geometry, dict):
            expected_roles = {
                "left": geometry.get("left_motor_role"),
                "right": geometry.get("right_motor_role"),
            }
            signs = geometry.get("forward_speed_sign")
            if isinstance(signs, dict):
                forward_signs = {
                    side: signs.get(expected_roles[side])
                    for side in ("left", "right")
                }
    if expected_roles and (
        not all(
            isinstance(expected_roles[side], str) and expected_roles[side]
            for side in ("left", "right")
        )
        or expected_roles["left"] == expected_roles["right"]
        or any(
            forward_signs[side] not in (-1, 1)
            for side in ("left", "right")
        )
    ):
        raise EV3PulseContractError(
            "worker drive geometry is invalid"
        )
    expected_directions = {
        side: (
            1
            if action_spec[side + "_speed_dps"] * forward_signs[side] > 0
            else -1
        )
        for side in ("left", "right")
    }
    return expected_roles, expected_directions


def validate_pulse_result(
    arguments: Mapping[str, object],
    response: Mapping[str, object],
    *,
    worker_description: Optional[Mapping[str, object]],
    stop_validator: Callable[[object], object],
) -> None:
    """Validate one successful pulse response and its temporal evidence."""

    result = response["result"]
    if (
        not isinstance(result, dict)
        or set(result) != {"action", "outcome", "observation", "stop"}
        or result["action"] != arguments["action"]
        or not isinstance(result["outcome"], dict)
    ):
        raise EV3PulseContractError("worker pulse result is invalid")
    outcome = result["outcome"]
    action = arguments["action"]
    try:
        action_spec = EXPECTED_ACTION_SPECS[action]
    except (KeyError, TypeError):
        raise EV3PulseContractError(
            "worker pulse action is invalid"
        ) from None
    expected_roles, expected_directions = _drive_geometry(
        worker_description,
        action_spec,
    )
    fields = {
        "kind",
        "action",
        "status",
        "reason",
        "started_monotonic_ms",
        "completed_monotonic_ms",
        "stop_confirmed",
        "requested_slice_count",
        "completed_slice_count",
        "slices",
        "encoder_verification",
    }
    statuses = {
        "completed",
        "interrupted",
        "denied",
        "verification_failed",
    }
    if (
        set(outcome) != fields
        or outcome["kind"] != "pulse"
        or outcome["action"] != arguments["action"]
        or outcome["status"] not in statuses
        or not isinstance(outcome["reason"], str)
        or not outcome["reason"]
        or outcome["stop_confirmed"] is not True
        or isinstance(outcome["requested_slice_count"], bool)
        or not isinstance(outcome["requested_slice_count"], int)
        or outcome["requested_slice_count"] != action_spec["slice_count"]
        or isinstance(outcome["completed_slice_count"], bool)
        or not isinstance(outcome["completed_slice_count"], int)
        or not 0
        <= outcome["completed_slice_count"]
        <= outcome["requested_slice_count"]
        or not isinstance(outcome["slices"], list)
        or len(outcome["slices"]) > outcome["requested_slice_count"]
        or isinstance(outcome["started_monotonic_ms"], bool)
        or (
            outcome["started_monotonic_ms"] is not None
            and not isinstance(outcome["started_monotonic_ms"], int)
        )
        or isinstance(outcome["completed_monotonic_ms"], bool)
        or not isinstance(outcome["completed_monotonic_ms"], int)
        or outcome["completed_monotonic_ms"] < 0
        or (
            outcome["started_monotonic_ms"] is not None
            and outcome["started_monotonic_ms"] < 0
        )
        or (
            outcome["started_monotonic_ms"] is not None
            and outcome["completed_monotonic_ms"]
            < outcome["started_monotonic_ms"]
        )
    ):
        raise EV3PulseContractError("worker pulse outcome is invalid")
    slices = outcome["slices"]
    for ordinal, value in enumerate(slices, 1):
        _validate_pulse_slice(
            value,
            ordinal=ordinal,
            expected_count=outcome["requested_slice_count"],
            expected_duration_ms=action_spec["slice_durations_ms"][
                ordinal - 1
            ],
            expected_roles=expected_roles,
            expected_directions=expected_directions,
            stop_validator=stop_validator,
        )
    if not slices:
        if (
            outcome["status"] != "denied"
            or outcome["started_monotonic_ms"] is not None
            or outcome["completed_slice_count"] != 0
        ):
            raise EV3PulseContractError(
                "empty pulse outcome is invalid"
            )
    else:
        if any(value["status"] != "completed" for value in slices[:-1]):
            raise EV3PulseContractError(
                "pulse slices do not form a completed prefix"
            )
        terminal = slices[-1]
        all_completed = terminal["status"] == "completed"
        if all_completed and len(slices) != action_spec["slice_count"]:
            raise EV3PulseContractError(
                "pulse outcome lacks a terminal slice"
            )
        if all_completed:
            expected_status = "completed"
            expected_reason = "semantic_action_completed"
        else:
            expected_status = terminal["status"]
            expected_reason = terminal["reason"]
            if (
                terminal["reason"] == "cancel_requested"
                and any(
                    value["status"] == "completed"
                    for value in slices[:-1]
                )
            ):
                expected_status = "interrupted"
        first_started = next(
            (
                value["started_monotonic_ms"]
                for value in slices
                if value["started_monotonic_ms"] is not None
            ),
            None,
        )
        if (
            outcome["status"] != expected_status
            or outcome["reason"] != expected_reason
            or outcome["started_monotonic_ms"] != first_started
            or outcome["completed_monotonic_ms"]
            != terminal["completed_monotonic_ms"]
            or result["stop"] != terminal["stop"]
        ):
            raise EV3PulseContractError(
                "pulse terminal outcome is uncorrelated"
            )
    completed_count = sum(
        value["status"] == "completed" for value in slices
    )
    verified_count = sum(
        value["encoder_verification"]["passed"] is True
        for value in slices
    )
    verification = outcome["encoder_verification"]
    if (
        not isinstance(verification, dict)
        or set(verification)
        != {
            "passed",
            "verified_slice_count",
            "requested_slice_count",
        }
        or type(verification["passed"]) is not bool
        or isinstance(verification["verified_slice_count"], bool)
        or not isinstance(verification["verified_slice_count"], int)
        or isinstance(verification["requested_slice_count"], bool)
        or not isinstance(verification["requested_slice_count"], int)
        or verification["verified_slice_count"] != verified_count
        or verification["requested_slice_count"]
        != outcome["requested_slice_count"]
        or outcome["completed_slice_count"] != completed_count
        or verification["passed"]
        != (verified_count == outcome["requested_slice_count"])
        or (
            outcome["status"] == "completed"
            and (
                verification["passed"] is not True
                or completed_count != outcome["requested_slice_count"]
                or verified_count != outcome["requested_slice_count"]
            )
        )
    ):
        raise EV3PulseContractError(
            "worker pulse encoder summary is invalid"
        )
    observation = validate_observation(result["observation"])
    if (
        observation["state_version"] != response["state_version"]
        or observation["last_outcome"] != outcome
    ):
        raise EV3PulseContractError(
            "worker pulse observation is uncorrelated"
        )
    evidence = next(
        (value for value in reversed(slices) if value["motors"]),
        None,
    )
    if evidence is not None:
        observed_motors = {
            motor["role"]: motor for motor in observation["motors"]
        }
        final_observation_correlated = True
        for motor in evidence["motors"]:
            observed = observed_motors.get(motor["role"])
            if observed is None:
                final_observation_correlated = False
                break
            settling_delta = (
                observed["position"] - motor["position_after"]
            )
            observed_net_delta = (
                observed["position"] - motor["position_before"]
            )
            receipt_delta = motor["position_delta"]
            if (
                abs(settling_delta)
                > MAX_FINAL_ENCODER_SETTLING_DEGREES
                or frozenset(observed["state"].split())
                & ACTIVE_MOTOR_STATES
                or (
                    receipt_delta != 0
                    and observed_net_delta * receipt_delta < 0
                )
            ):
                final_observation_correlated = False
                break
        if not final_observation_correlated:
            raise EV3PulseContractError(
                "pulse final encoder observation is uncorrelated"
            )
    latched = observation["budgets"]["motion_fault_latched"]
    recoverable_latch_mismatch = (
        outcome["reason"] == "encoder_undertravel_observed"
        and latched is not False
    )
    hard_fault_latch_mismatch = (
        outcome["reason"]
        in ("encoder_verification_failed", "motor_fault")
        and latched is not True
    )
    if recoverable_latch_mismatch or hard_fault_latch_mismatch:
        raise EV3PulseContractError(
            "pulse fault latch state is uncorrelated"
        )
    stop_validator(result["stop"])
