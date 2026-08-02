import copy
import unittest

from robot_agent.ev3_canonical_receipt import (
    EV3CanonicalReceiptError,
    receipt_from_committed_not_dispatched,
    receipt_from_validated_ev3_pulse,
)
from robot_agent.ev3_navigation_transport import (
    EV3NavigationCommittedNotDispatchedError,
    EV3NavigationSSHTransport,
    EV3NavigationTransportError,
)
from robot_agent.physical_agent_state import (
    ControllerKey,
    PlanStepKey,
    ReceiptOutcome,
    StepCommandAuthorization,
)
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    RESPONSE_SCHEMA,
)
from tests.test_physical_navigation_core import recovered_motion_result


def authorization(*, controller_instance_id="worker-boot-1"):
    return StepCommandAuthorization(
        action_id="action-1",
        command_id="command-1",
        host_dispatch_sequence=1,
        controller_key=ControllerKey(
            robot_id="ev3rstorm-01",
            controller_id="ev3-main",
            controller_instance_id=controller_instance_id,
        ),
        step_key=PlanStepKey(
            plan_id="plan-1",
            plan_revision=1,
            cursor=0,
            step_id="step-1",
        ),
        based_on_navigation_basis_id="basis-1",
        based_on_controller_state_version=4,
        command_fingerprint="sha256:command-1",
        issued_at_ms=100,
        valid_until_ms=1_100,
    )


def pulse_response(
    *,
    action=ADVANCE,
    status="completed",
    reason=None,
    state_version=5,
    started_monotonic_ms=10,
    completed_slice_count=1,
):
    if reason is None:
        reason = {
            "completed": "semantic_action_completed",
            "denied": "infrared_blocked",
            "interrupted": "cancel_requested",
            "verification_failed": "encoder_undertravel_observed",
        }[status]
    outcome = {
        "kind": "pulse",
        "action": action,
        "status": status,
        "reason": reason,
        "started_monotonic_ms": started_monotonic_ms,
        "completed_monotonic_ms": 20,
        "stop_confirmed": True,
        "requested_slice_count": 1,
        "completed_slice_count": completed_slice_count,
        "slices": [],
        "encoder_verification": {
            "passed": status == "completed",
            "verified_slice_count": completed_slice_count,
            "requested_slice_count": 1,
        },
    }
    return {
        "schema": RESPONSE_SCHEMA,
        "controller_id": "ev3-main",
        "request_id": "host-0001",
        "ok": True,
        "state_version": state_version,
        "result": {
            "action": action,
            "outcome": outcome,
            "observation": {
                "state_version": state_version,
            },
            "stop": {"stop_confirmed": True},
        },
    }


def map_pulse(response, *, auth=None, action=ADVANCE, instance="worker-boot-1"):
    return receipt_from_validated_ev3_pulse(
        authorization=authorization() if auth is None else auth,
        expected_action=action,
        expected_request_id="host-0001",
        response=response,
        current_controller_instance_id=instance,
        received_at_host_ms=130,
    )


class EV3CanonicalPulseReceiptTests(unittest.TestCase):
    def test_maps_a_response_after_full_transport_validation(self):
        result = copy.deepcopy(recovered_motion_result())
        result["observation"]["state_version"] = 5
        response = {
            "schema": RESPONSE_SCHEMA,
            "controller_id": "ev3-main",
            "request_id": "host-0001",
            "ok": True,
            "state_version": 5,
            "result": result,
        }
        transport = EV3NavigationSSHTransport(
            target="robot@ev3.local",
            controller_id="ev3-main",
            remote_worker_path=(
                "/home/robot/robot-llm/ev3/navigation_worker.py"
            ),
        )
        validated = transport._validate_response(response, "host-0001")
        transport._validate_success_result(
            "pulse",
            {"action": ADVANCE},
            validated,
        )

        receipt = map_pulse(validated)

        self.assertEqual(receipt.outcome, ReceiptOutcome.COMPLETED)
        self.assertEqual(receipt.resulting_controller_state_version, 5)

    def test_completed_pulse_copies_exact_authorization_provenance(self):
        auth = authorization()
        receipt = map_pulse(pulse_response(), auth=auth)

        self.assertEqual(receipt.outcome, ReceiptOutcome.COMPLETED)
        self.assertEqual(receipt.controller_key, auth.controller_key)
        self.assertEqual(receipt.step_key, auth.step_key)
        self.assertEqual(receipt.action_id, auth.action_id)
        self.assertEqual(receipt.command_id, auth.command_id)
        self.assertEqual(
            receipt.host_dispatch_sequence,
            auth.host_dispatch_sequence,
        )
        self.assertEqual(
            receipt.command_fingerprint,
            auth.command_fingerprint,
        )
        self.assertEqual(
            receipt.based_on_navigation_basis_id,
            auth.based_on_navigation_basis_id,
        )
        self.assertEqual(
            receipt.based_on_controller_state_version,
            auth.based_on_controller_state_version,
        )
        self.assertEqual(receipt.resulting_controller_state_version, 5)
        self.assertEqual(receipt.received_at_host_ms, 130)
        self.assertTrue(receipt.stop_confirmed)
        self.assertEqual(receipt.code, "ev3:semantic_action_completed")
        self.assertFalse(hasattr(receipt, "disposition"))

    def test_status_mapping_preserves_started_denial_as_stopped(self):
        cases = (
            (
                pulse_response(
                    status="denied",
                    started_monotonic_ms=None,
                    completed_slice_count=0,
                ),
                ReceiptOutcome.REJECTED_NOT_STARTED,
            ),
            (
                pulse_response(status="interrupted"),
                ReceiptOutcome.STOPPED,
            ),
            (
                pulse_response(status="verification_failed"),
                ReceiptOutcome.STOPPED,
            ),
            (
                pulse_response(
                    status="denied",
                    started_monotonic_ms=10,
                    completed_slice_count=1,
                ),
                ReceiptOutcome.STOPPED,
            ),
        )
        for response, expected in cases:
            with self.subTest(
                status=response["result"]["outcome"]["status"],
                started=response["result"]["outcome"][
                    "started_monotonic_ms"
                ],
            ):
                self.assertEqual(map_pulse(response).outcome, expected)

    def test_action_controller_instance_and_controller_id_are_bound(self):
        wrong_result_action = pulse_response()
        wrong_result_action["result"]["action"] = "REVERSE"
        wrong_outcome_action = pulse_response()
        wrong_outcome_action["result"]["outcome"]["action"] = "REVERSE"
        wrong_controller = pulse_response()
        wrong_controller["controller_id"] = "other-controller"
        wrong_request = pulse_response()
        wrong_request["request_id"] = "host-0002"

        cases = (
            ({"response": wrong_result_action}, "invalid_validated_response"),
            ({"response": wrong_outcome_action}, "invalid_validated_response"),
            ({"response": wrong_controller}, "invalid_validated_response"),
            ({"response": wrong_request}, "invalid_validated_response"),
            (
                {"response": pulse_response(), "action": "REVERSE"},
                "invalid_validated_response",
            ),
            (
                {"response": pulse_response(), "instance": "worker-boot-2"},
                "controller_instance_mismatch",
            ),
        )
        for arguments, code in cases:
            with self.subTest(code=code, arguments=arguments):
                with self.assertRaises(EV3CanonicalReceiptError) as caught:
                    map_pulse(**arguments)
                self.assertEqual(caught.exception.code, code)

    def test_response_version_and_stop_proof_remain_correlated(self):
        mismatches = []
        observation_mismatch = pulse_response()
        observation_mismatch["result"]["observation"]["state_version"] = 6
        mismatches.append(observation_mismatch)
        outcome_unstopped = pulse_response()
        outcome_unstopped["result"]["outcome"]["stop_confirmed"] = False
        mismatches.append(outcome_unstopped)
        proof_unstopped = pulse_response()
        proof_unstopped["result"]["stop"]["stop_confirmed"] = False
        mismatches.append(proof_unstopped)

        for response in mismatches:
            with self.subTest(response=response):
                with self.assertRaises(EV3CanonicalReceiptError) as caught:
                    map_pulse(response)
                self.assertEqual(
                    caught.exception.code,
                    "invalid_validated_response",
                )

        with self.assertRaises(EV3CanonicalReceiptError) as regressed:
            map_pulse(pulse_response(state_version=3))
        self.assertEqual(
            regressed.exception.code,
            "canonical_receipt_rejected",
        )

    def test_worker_reason_is_bounded_without_truncation(self):
        for reason in ("", " padded", "line\nbreak", "x" * 157):
            with self.subTest(reason=reason):
                with self.assertRaises(EV3CanonicalReceiptError) as caught:
                    map_pulse(pulse_response(reason=reason))
                self.assertEqual(caught.exception.code, "invalid_ev3_reason")

    def test_remote_error_envelope_cannot_become_a_receipt(self):
        response = pulse_response()
        response["ok"] = False
        response["error"] = response.pop("result")

        with self.assertRaises(EV3CanonicalReceiptError) as caught:
            map_pulse(response)

        self.assertEqual(caught.exception.code, "invalid_validated_response")


class EV3CommittedNotDispatchedReceiptTests(unittest.TestCase):
    def map_error(self, error, *, auth=None, instance="worker-boot-1"):
        return receipt_from_committed_not_dispatched(
            authorization=authorization() if auth is None else auth,
            error=error,
            expected_request_id="host-0001",
            current_controller_instance_id=instance,
            received_at_host_ms=130,
        )

    def test_typed_zero_byte_outcomes_close_as_local_rejections(self):
        cases = (
            (
                EV3NavigationCommittedNotDispatchedError.CANCELLED,
                "ev3:cancelled_after_commit",
            ),
            (
                EV3NavigationCommittedNotDispatchedError.
                CANCELLATION_PROBE_FAILED,
                "ev3:cancel_probe_failed_after_commit",
            ),
        )
        for reason, expected_code in cases:
            with self.subTest(reason=reason):
                auth = authorization()
                error = EV3NavigationCommittedNotDispatchedError(
                    "host-0001",
                    reason,
                )
                receipt = self.map_error(error, auth=auth)

                self.assertEqual(
                    receipt.outcome,
                    ReceiptOutcome.REJECTED_NOT_STARTED,
                )
                self.assertEqual(receipt.controller_key, auth.controller_key)
                self.assertEqual(receipt.step_key, auth.step_key)
                self.assertEqual(receipt.command_id, auth.command_id)
                self.assertEqual(
                    receipt.resulting_controller_state_version,
                    auth.based_on_controller_state_version,
                )
                self.assertFalse(receipt.stop_confirmed)
                self.assertEqual(receipt.code, expected_code)

    def test_generic_transport_error_cannot_fabricate_local_rejection(self):
        with self.assertRaises(EV3CanonicalReceiptError) as caught:
            self.map_error(EV3NavigationTransportError("write failed"))

        self.assertEqual(
            caught.exception.code,
            "invalid_committed_not_dispatched",
        )

    def test_zero_byte_error_must_match_exact_ev3_request(self):
        error = EV3NavigationCommittedNotDispatchedError(
            "host-0002",
            EV3NavigationCommittedNotDispatchedError.CANCELLED,
        )

        with self.assertRaises(EV3CanonicalReceiptError) as caught:
            self.map_error(error)

        self.assertEqual(
            caught.exception.code,
            "invalid_committed_not_dispatched",
        )

    def test_all_zero_byte_proof_fields_are_required(self):
        changes = (
            ("record_committed", False),
            ("write_attempted", True),
            ("bytes_sent", False),
            ("bytes_sent", 1),
            ("physical_outcome_known", False),
            ("transport_reusable", False),
            ("reason", "UNKNOWN"),
        )
        for field, value in changes:
            with self.subTest(field=field, value=value):
                error = EV3NavigationCommittedNotDispatchedError(
                    "host-0001",
                    EV3NavigationCommittedNotDispatchedError.CANCELLED,
                )
                setattr(error, field, value)
                with self.assertRaises(EV3CanonicalReceiptError) as caught:
                    self.map_error(error)
                self.assertEqual(
                    caught.exception.code,
                    "invalid_committed_not_dispatched",
                )

    def test_local_rejection_is_bound_to_current_worker_instance(self):
        error = EV3NavigationCommittedNotDispatchedError(
            "host-0001",
            EV3NavigationCommittedNotDispatchedError.CANCELLED,
        )

        with self.assertRaises(EV3CanonicalReceiptError) as caught:
            self.map_error(error, instance="worker-boot-2")

        self.assertEqual(
            caught.exception.code,
            "controller_instance_mismatch",
        )


if __name__ == "__main__":
    unittest.main()
