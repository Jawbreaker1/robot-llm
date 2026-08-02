"""Pure mapping from verified EV3 outcomes to canonical command receipts.

The normal mapper accepts only responses already validated by
``EV3NavigationSSHTransport.request("pulse", ...)``.  It deliberately does
not repeat encoder-proof validation or decide plan progress, odometry,
replanning, deadlines, retries, or reducer events.
"""

from typing import Mapping

from .ev3_navigation_transport import (
    EV3NavigationCommittedNotDispatchedError,
)
from .physical_agent_state import (
    ControllerCommandReceipt,
    PhysicalAgentStateError,
    ReceiptOutcome,
    StepCommandAuthorization,
)
from .physical_navigation_contract import (
    MOTION_ACTIONS,
    RESPONSE_SCHEMA,
    validate_controller_instance_id,
)


_STATUS_OUTCOMES = {
    "completed": ReceiptOutcome.COMPLETED,
    "interrupted": ReceiptOutcome.STOPPED,
    "verification_failed": ReceiptOutcome.STOPPED,
}
_PULSE_STATUSES = frozenset((*_STATUS_OUTCOMES, "denied"))
_LOCAL_REJECTION_CODES = {
    EV3NavigationCommittedNotDispatchedError.CANCELLED: (
        "ev3:cancelled_after_commit"
    ),
    EV3NavigationCommittedNotDispatchedError.CANCELLATION_PROBE_FAILED: (
        "ev3:cancel_probe_failed_after_commit"
    ),
}
_MAX_RECEIPT_CODE_LENGTH = 160


class EV3CanonicalReceiptError(ValueError):
    """Verified EV3 evidence cannot be bound to a canonical command."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _context(
    authorization: StepCommandAuthorization,
    expected_request_id: str,
    current_controller_instance_id: str,
    received_at_host_ms: int,
) -> None:
    if not isinstance(authorization, StepCommandAuthorization):
        raise EV3CanonicalReceiptError(
            "invalid_authorization",
            "receipt mapping requires a step command authorization",
        )
    allowed_request_characters = (
        "abcdefghijklmnopqrstuvwxyz"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "0123456789._:-"
    )
    if (
        not isinstance(expected_request_id, str)
        or not expected_request_id
        or len(expected_request_id) > 128
        or any(
            character not in allowed_request_characters
            for character in expected_request_id
        )
    ):
        raise EV3CanonicalReceiptError(
            "invalid_request_id",
            "expected EV3 request identity is invalid",
        )
    try:
        current_instance = validate_controller_instance_id(
            current_controller_instance_id
        )
    except ValueError:
        raise EV3CanonicalReceiptError(
            "invalid_controller_instance",
            "current controller instance identity is invalid",
        ) from None
    if current_instance != authorization.controller_key.controller_instance_id:
        raise EV3CanonicalReceiptError(
            "controller_instance_mismatch",
            "authorization belongs to a different controller instance",
        )
    if (
        isinstance(received_at_host_ms, bool)
        or not isinstance(received_at_host_ms, int)
        or received_at_host_ms < 0
    ):
        raise EV3CanonicalReceiptError(
            "invalid_receipt_time",
            "receipt ingress time is invalid",
        )


def _receipt_code(reason: object) -> str:
    code = "ev3:{}".format(reason) if isinstance(reason, str) else None
    if (
        code is None
        or not reason
        or reason != reason.strip()
        or len(code) > _MAX_RECEIPT_CODE_LENGTH
        or any(ord(character) < 32 for character in code)
    ):
        raise EV3CanonicalReceiptError(
            "invalid_ev3_reason",
            "validated EV3 outcome reason is not a bounded receipt code",
        )
    return code


def _receipt_outcome(outcome: Mapping[str, object]) -> ReceiptOutcome:
    status = outcome["status"]
    if status != "denied":
        return _STATUS_OUTCOMES[status]
    if (
        outcome.get("started_monotonic_ms") is None
        and type(outcome.get("completed_slice_count")) is int
        and outcome.get("completed_slice_count") == 0
    ):
        return ReceiptOutcome.REJECTED_NOT_STARTED
    # EV3 may deny a later slice after a verified completed prefix.  Motion
    # has then started, so the canonical result is stopped, never rejected.
    return ReceiptOutcome.STOPPED


def _build_receipt(
    *,
    authorization: StepCommandAuthorization,
    outcome: ReceiptOutcome,
    resulting_controller_state_version: int,
    received_at_host_ms: int,
    stop_confirmed: bool,
    code: str,
) -> ControllerCommandReceipt:
    try:
        return ControllerCommandReceipt(
            outcome=outcome,
            controller_key=authorization.controller_key,
            step_key=authorization.step_key,
            action_id=authorization.action_id,
            command_id=authorization.command_id,
            host_dispatch_sequence=authorization.host_dispatch_sequence,
            command_fingerprint=authorization.command_fingerprint,
            based_on_navigation_basis_id=(
                authorization.based_on_navigation_basis_id
            ),
            based_on_controller_state_version=(
                authorization.based_on_controller_state_version
            ),
            resulting_controller_state_version=(
                resulting_controller_state_version
            ),
            received_at_host_ms=received_at_host_ms,
            stop_confirmed=stop_confirmed,
            code=code,
        )
    except (PhysicalAgentStateError, TypeError, ValueError) as error:
        raise EV3CanonicalReceiptError(
            "canonical_receipt_rejected",
            "canonical command receipt rejected EV3 evidence",
        ) from error


def receipt_from_validated_ev3_pulse(
    *,
    authorization: StepCommandAuthorization,
    expected_action: str,
    expected_request_id: str,
    response: Mapping[str, object],
    current_controller_instance_id: str,
    received_at_host_ms: int,
) -> ControllerCommandReceipt:
    """Bind one transport-validated pulse result to its authorization."""

    _context(
        authorization,
        expected_request_id,
        current_controller_instance_id,
        received_at_host_ms,
    )
    if (
        not isinstance(expected_action, str)
        or expected_action not in MOTION_ACTIONS
    ):
        raise EV3CanonicalReceiptError(
            "invalid_expected_action",
            "expected action is not an EV3 motion pulse",
        )
    if (
        not isinstance(response, Mapping)
        or set(response)
        != {
            "schema",
            "controller_id",
            "request_id",
            "ok",
            "state_version",
            "result",
        }
        or response.get("schema") != RESPONSE_SCHEMA
        or response.get("ok") is not True
        or response.get("request_id") != expected_request_id
        or response.get("controller_id")
        != authorization.controller_key.controller_id
    ):
        raise EV3CanonicalReceiptError(
            "invalid_validated_response",
            "EV3 pulse response envelope is invalid or uncorrelated",
        )
    result = response["result"]
    if not isinstance(result, Mapping) or set(result) != {
        "action",
        "outcome",
        "observation",
        "stop",
    }:
        raise EV3CanonicalReceiptError(
            "invalid_validated_response",
            "EV3 pulse result is invalid",
        )
    outcome = result["outcome"]
    observation = result["observation"]
    stop = result["stop"]
    status = outcome.get("status") if isinstance(outcome, Mapping) else None
    resulting_version = response["state_version"]
    if (
        result["action"] != expected_action
        or not isinstance(outcome, Mapping)
        or outcome.get("kind") != "pulse"
        or outcome.get("action") != expected_action
        or status not in _PULSE_STATUSES
        or outcome.get("stop_confirmed") is not True
        or not isinstance(observation, Mapping)
        or observation.get("state_version") != resulting_version
        or not isinstance(stop, Mapping)
        or stop.get("stop_confirmed") is not True
    ):
        raise EV3CanonicalReceiptError(
            "invalid_validated_response",
            "EV3 pulse result lost action, version, or stop correlation",
        )
    return _build_receipt(
        authorization=authorization,
        outcome=_receipt_outcome(outcome),
        resulting_controller_state_version=resulting_version,
        received_at_host_ms=received_at_host_ms,
        stop_confirmed=True,
        code=_receipt_code(outcome.get("reason")),
    )


def receipt_from_committed_not_dispatched(
    *,
    authorization: StepCommandAuthorization,
    error: EV3NavigationCommittedNotDispatchedError,
    expected_request_id: str,
    current_controller_instance_id: str,
    received_at_host_ms: int,
) -> ControllerCommandReceipt:
    """Settle a durable dispatch record proven to have sent zero bytes."""

    _context(
        authorization,
        expected_request_id,
        current_controller_instance_id,
        received_at_host_ms,
    )
    if (
        not isinstance(error, EV3NavigationCommittedNotDispatchedError)
        or error.request_id != expected_request_id
        or error.reason not in _LOCAL_REJECTION_CODES
        or error.record_committed is not True
        or error.write_attempted is not False
        or type(error.bytes_sent) is not int
        or error.bytes_sent != 0
        or error.physical_outcome_known is not True
        or error.transport_reusable is not True
    ):
        raise EV3CanonicalReceiptError(
            "invalid_committed_not_dispatched",
            "transport error does not prove a committed zero-byte dispatch",
        )
    return _build_receipt(
        authorization=authorization,
        outcome=ReceiptOutcome.REJECTED_NOT_STARTED,
        resulting_controller_state_version=(
            authorization.based_on_controller_state_version
        ),
        received_at_host_ms=received_at_host_ms,
        stop_confirmed=False,
        code=_LOCAL_REJECTION_CODES[error.reason],
    )


__all__ = (
    "EV3CanonicalReceiptError",
    "receipt_from_committed_not_dispatched",
    "receipt_from_validated_ev3_pulse",
)
