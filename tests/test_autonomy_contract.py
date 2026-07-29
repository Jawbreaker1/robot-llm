from dataclasses import replace
import json
import subprocess
import sys
import unittest

from robot_agent.autonomy_contract import (
    MAX_AUTONOMY_SELECTION_BYTES,
    ROBOT_BASE_FRAME,
    ExplorationCandidate,
    InterestObservation,
    InterestSelectionContext,
    InterestSelectionProposal,
    decode_interest_selection,
)
from robot_agent.navigation_contract import NavigationContractError


def observation(observation_id="range-one"):
    return InterestObservation(
        observation_id=observation_id,
        producer_id="simulated-forward-range",
        subject_robot_id="ev3rstorm-sim",
        controller_instance_id="controller-one",
        frame_id=ROBOT_BASE_FRAME,
        modality="RANGE",
        kind="METRIC_SAMPLE",
        channel="FORWARD_CLEARANCE",
        observed_at_ms=10_000,
        received_at_host_ms=19_900,
        valid_until_host_ms=25_000,
        state_version=7,
        world_model_version=3,
        confidence_milli=1_000,
        previous_value=320,
        current_value=155,
        unit="mm",
        previous_subject_id=None,
        current_subject_id="box-a",
    )


def candidate(candidate_id="candidate-left"):
    return ExplorationCandidate(
        candidate_id=candidate_id,
        task_kind="INVESTIGATE_OBSERVATION",
        relative_direction="LEFT",
        estimated_travel_mm=160,
        attempted_visits=0,
        completed_visits=0,
        linked_observation_ids=("range-one",),
    )


def context():
    return InterestSelectionContext(
        proposal_id="selection-proposal-one",
        robot_id="ev3rstorm-sim",
        controller_instance_id="controller-one",
        autonomy_session_id="idle-session-one",
        lease_generation=4,
        candidate_set_id="candidate-set-one",
        frame_id=ROBOT_BASE_FRAME,
        state_version=7,
        world_model_version=3,
        captured_at_ms=20_000,
        valid_until_ms=25_000,
        remaining_tasks=3,
        observations=(observation(),),
        candidates=(
            candidate(),
            replace(
                candidate(),
                candidate_id="candidate-right",
                relative_direction="RIGHT",
            ),
        ),
    )


def proposal(decision="SELECT"):
    common = {
        "proposal_id": "selection-proposal-one",
        "robot_id": "ev3rstorm-sim",
        "controller_instance_id": "controller-one",
        "autonomy_session_id": "idle-session-one",
        "lease_generation": 4,
        "candidate_set_id": "candidate-set-one",
        "based_on_state_version": 7,
        "based_on_world_model_version": 3,
        "decision": decision,
        "confidence_milli": 870,
    }
    if decision == "SELECT":
        common["selected_candidate_id"] = "candidate-left"
    else:
        common["reason_code"] = "NOTHING_WORTHWHILE"
    return InterestSelectionProposal(**common)


class AutonomyContractTests(unittest.TestCase):
    def test_select_and_hold_strict_round_trip(self):
        for value in (proposal(), proposal("HOLD"), proposal("ABORT")):
            with self.subTest(decision=value.decision):
                decoded = decode_interest_selection(
                    json.dumps(value.to_dict()).encode("utf-8")
                )
                self.assertEqual(decoded, value)

    def test_context_accepts_exact_binding_and_deadline_boundary(self):
        selection_context = context()
        selection = proposal()

        selection_context.assert_accepts(selection, 20_000)
        selection_context.assert_accepts(selection, 24_999)

        with self.assertRaises(NavigationContractError) as caught:
            selection_context.assert_accepts(selection, 25_000)
        self.assertEqual(
            caught.exception.code,
            "expired_interest_selection",
        )

    def test_context_rejects_stale_binding_and_unknown_candidate(self):
        selection_context = context()
        stale = replace(
            proposal(),
            based_on_world_model_version=2,
        )
        unknown = replace(
            proposal(),
            selected_candidate_id="candidate-not-offered",
        )

        with self.assertRaises(NavigationContractError) as caught:
            selection_context.assert_accepts(stale, 20_100)
        self.assertEqual(
            caught.exception.code,
            "stale_interest_selection",
        )
        with self.assertRaises(NavigationContractError) as caught:
            selection_context.assert_accepts(unknown, 20_100)
        self.assertEqual(
            caught.exception.code,
            "unknown_selected_candidate",
        )

    def test_context_rejects_mismatched_or_expired_observation(self):
        mutations = (
            {"subject_robot_id": "another-robot"},
            {"controller_instance_id": "another-controller"},
            {"frame_id": "CAMERA_FRAME"},
            {"state_version": 8},
            {"world_model_version": 4},
            {"received_at_host_ms": 20_001},
            {"valid_until_host_ms": 20_000},
            {"valid_until_host_ms": 24_999},
        )

        for changes in mutations:
            with self.subTest(changes=changes):
                changed = replace(observation(), **changes)
                with self.assertRaises(
                    NavigationContractError
                ) as caught:
                    replace(context(), observations=(changed,))
                self.assertEqual(
                    caught.exception.code,
                    "stale_interest_observation",
                )

    def test_observation_requires_an_exclusive_host_validity_window(self):
        with self.assertRaises(NavigationContractError):
            replace(
                observation(),
                valid_until_host_ms=19_900,
            )

        with self.assertRaises(NavigationContractError):
            replace(
                candidate(),
                attempted_visits=True,
            )
        with self.assertRaises(NavigationContractError) as caught:
            replace(
                candidate(),
                attempted_visits=0,
                completed_visits=1,
            )
        self.assertEqual(
            caught.exception.code,
            "invalid_exploration_visit_counts",
        )

    def test_duplicate_extra_missing_and_oversize_json_are_rejected(self):
        raw = json.dumps(proposal().to_dict()).encode("utf-8")
        duplicate = raw.replace(
            b'"proposal_id": "selection-proposal-one"',
            (
                b'"proposal_id": "selection-proposal-one", '
                b'"proposal_id": "selection-proposal-two"'
            ),
        )
        extra = proposal().to_dict()
        extra["target_x_mm"] = 900
        missing = proposal().to_dict()
        missing.pop("confidence_milli")
        invalid_values = (
            duplicate,
            json.dumps(extra).encode("utf-8"),
            json.dumps(missing).encode("utf-8"),
            b"",
            b"x" * (MAX_AUTONOMY_SELECTION_BYTES + 1),
        )
        for raw_value in invalid_values:
            with self.subTest(raw=raw_value[:40]):
                with self.assertRaises(NavigationContractError):
                    decode_interest_selection(raw_value)

    def test_bool_as_integer_and_non_finite_values_are_rejected(self):
        boolean_value = proposal().to_dict()
        boolean_value["lease_generation"] = True
        non_finite = (
            b'{"schema":"robot-autonomy-interest-selection/v1",'
            b'"proposal_id":"p","robot_id":"r",'
            b'"controller_instance_id":"c","autonomy_session_id":"s",'
            b'"lease_generation":1,"candidate_set_id":"set",'
            b'"based_on_state_version":1,'
            b'"based_on_world_model_version":1,'
            b'"decision":"HOLD","confidence_milli":NaN,'
            b'"reason_code":"NONE"}'
        )

        for raw in (
            json.dumps(boolean_value).encode("utf-8"),
            non_finite,
        ):
            with self.assertRaises(NavigationContractError):
                decode_interest_selection(raw)

    def test_candidate_view_contains_no_coordinates_or_motion_settings(self):
        view = candidate().to_dict()

        self.assertEqual(
            set(view),
            {
                "candidate_id",
                "task_kind",
                "relative_direction",
                "estimated_travel_mm",
                "attempted_visits",
                "completed_visits",
                "linked_observation_ids",
            },
        )
        serialized = json.dumps(context().to_dict())
        for forbidden in (
            "target_x",
            "target_y",
            "heading",
            "speed",
            "duration",
            "authority",
            "priority",
            "goal_epoch",
        ):
            self.assertNotIn(forbidden, serialized)

    def test_generic_nonlinguistic_modalities_need_no_locale_routing(self):
        audio = InterestObservation(
            observation_id="audio-energy-one",
            producer_id="future-microphone",
            subject_robot_id="ev3rstorm-sim",
            controller_instance_id="controller-one",
            frame_id=ROBOT_BASE_FRAME,
            modality="AUDIO",
            kind="METRIC_SAMPLE",
            channel="LEFT_ENERGY",
            observed_at_ms=40_000,
            received_at_host_ms=40_010,
            valid_until_host_ms=41_000,
            state_version=9,
            world_model_version=5,
            confidence_milli=700,
            current_value=812,
            unit="milli",
        )

        self.assertEqual(audio.modality, "AUDIO")
        self.assertNotIn("locale", audio.to_dict())

    def test_unknown_observation_link_is_rejected(self):
        invalid_candidate = replace(
            candidate(),
            linked_observation_ids=("not-in-context",),
        )

        with self.assertRaises(NavigationContractError) as caught:
            replace(context(), candidates=(invalid_candidate,))
        self.assertEqual(
            caught.exception.code,
            "unknown_linked_observation",
        )

    def test_contract_import_does_not_load_execution_or_transport(self):
        code = (
            "import sys;"
            "import robot_agent.autonomy_contract;"
            "assert 'robot_agent.autonomy_runtime' not in sys.modules;"
            "assert 'robot_agent.navigation_simulator' not in sys.modules;"
            "assert 'robot_agent.lm_studio_autonomy' not in sys.modules"
        )
        completed = subprocess.run(
            [sys.executable, "-c", code],
            check=False,
            env={
                "PYTHONPATH": "src",
                "PYTHONDONTWRITEBYTECODE": "1",
            },
            capture_output=True,
            text=True,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)


if __name__ == "__main__":
    unittest.main()
