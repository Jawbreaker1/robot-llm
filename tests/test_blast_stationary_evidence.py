import copy
import unittest

from robot_agent.blast_observation_monitor import (
    CONTROLLER_ID,
    ROBOT_ID,
    SETTLED_OBSERVATION_COMMAND,
    BlastControllerError,
)
from robot_agent.blast_stationary_evidence import (
    BlastStationaryEvidenceStatus,
    collect_blast_stationary_evidence,
)


def observation(distance_mm=500):
    return {
        "distance_mm": distance_mm,
        "motion_active": False,
        "motor_angles_deg": {
            "left_drive": 10,
            "right_drive": -10,
            "body": 158,
        },
        "imu": {"heading_deg": 0.0},
    }


class Controller:
    def __init__(self, distances=(500,), *, clock=None, generation=None):
        self.distances = list(distances)
        self.clock = clock if clock is not None else [1_000]
        self.generation = generation if generation is not None else [1]
        self.commands = []
        self.snapshot_count = 0
        self.command_hook = None
        self.snapshot_hook = None
        self.snapshot_value = {
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "state": "online",
            "last_observed_at_monotonic_ms": self.clock[0],
            "observation": observation(),
        }

    def snapshot(self):
        self.snapshot_count += 1
        if self.snapshot_hook is not None:
            self.snapshot_hook(self)
        return copy.deepcopy(self.snapshot_value)

    def command(self, command, *, cancel_requested=None):
        self.commands.append(command)
        self.assert_not_cancelled(cancel_requested)
        if self.command_hook is not None:
            injected = self.command_hook(self, len(self.commands))
            if injected is not None:
                return injected
        distance = self.distances.pop(0)
        self.clock[0] += 1
        result_observation = observation(distance)
        self.snapshot_value.update({
            "state": "online",
            "last_observed_at_monotonic_ms": self.clock[0],
            "observation": result_observation,
        })
        return {
            "robot_id": ROBOT_ID,
            "controller_id": CONTROLLER_ID,
            "command": command,
            "accepted": True,
            "completed": True,
            "observation_settled": True,
            "observation": copy.deepcopy(result_observation),
        }

    @staticmethod
    def assert_not_cancelled(cancel_requested):
        if cancel_requested is not None and cancel_requested():
            raise AssertionError("test command was unexpectedly cancelled")


class BlastStationaryEvidenceTests(unittest.TestCase):
    @staticmethod
    def collect(controller, *, control=lambda: None, pause=lambda: None,
                **updates):
        values = {
            "controller": controller,
            "expected_drive_angles": {
                "left_drive": 10,
                "right_drive": -10,
            },
            "minimum_safe_distance_mm": 120,
            "control_outcome": control,
            "session_generation": lambda: controller.generation[0],
            "monotonic_ms": lambda: controller.clock[0],
            "pause": pause,
            "reconnect_poll_limit": 4,
        }
        values.update(updates)
        return collect_blast_stationary_evidence(**values)

    def test_returns_measured_safe_only_from_fresh_matched_receipt(self):
        controller = Controller((500,))

        result = self.collect(controller)

        self.assertEqual(
            result.status, BlastStationaryEvidenceStatus.MEASURED_SAFE,
        )
        self.assertEqual(result.settled_attempts, 1)
        self.assertEqual(result.reconnect_generations, 0)
        self.assertEqual(
            controller.commands, [SETTLED_OBSERVATION_COMMAND],
        )
        self.assertEqual(result.observation["distance_mm"], 500)
        self.assertEqual(
            result.receipt["command"], SETTLED_OBSERVATION_COMMAND,
        )

    def test_exact_no_valid_distance_is_typed_but_never_clear(self):
        controller = Controller((2_000,))

        result = self.collect(controller)

        self.assertEqual(
            result.status, BlastStationaryEvidenceStatus.EXACT_NVD,
        )
        self.assertNotEqual(
            result.status, BlastStationaryEvidenceStatus.MEASURED_SAFE,
        )
        self.assertEqual(result.observation["distance_mm"], 2_000)

    def test_invalid_or_unsettled_receipt_gets_at_most_two_attempts(self):
        for fault in ("invalid", "unsettled", "stale", "mismatched"):
            with self.subTest(fault=fault):
                controller = Controller((500, 500))

                def inject(value, _attempt):
                    value.clock[0] += int(fault != "stale")
                    observed = observation(
                        None if fault == "invalid" else 500,
                    )
                    value.snapshot_value.update({
                        "last_observed_at_monotonic_ms": value.clock[0],
                        "observation": observed,
                    })
                    receipt_observation = copy.deepcopy(observed)
                    if fault == "mismatched":
                        receipt_observation["distance_mm"] = 2_000
                    return {
                        "robot_id": ROBOT_ID,
                        "controller_id": CONTROLLER_ID,
                        "command": SETTLED_OBSERVATION_COMMAND,
                        "accepted": True,
                        "completed": True,
                        "observation_settled": fault != "unsettled",
                        "observation": receipt_observation,
                    }

                controller.command_hook = inject
                result = self.collect(controller)

                self.assertEqual(
                    result.status,
                    BlastStationaryEvidenceStatus.EXHAUSTED,
                )
                self.assertEqual(result.settled_attempts, 2)
                self.assertEqual(
                    controller.commands,
                    [SETTLED_OBSERVATION_COMMAND] * 2,
                )

    def test_reconnect_requires_one_new_generation_and_new_receipt(self):
        controller = Controller((500,))

        def disconnect(value, attempt):
            if attempt == 1:
                value.snapshot_value["state"] = "offline"
                raise BlastControllerError(
                    "controller_unavailable", "injected disconnect",
                    motion_started=False,
                )
            return None

        def reconnect(value):
            if value.snapshot_value["state"] == "offline":
                value.generation[0] = 2
                value.snapshot_value["state"] = "online"

        controller.command_hook = disconnect
        result = self.collect(controller, pause=lambda: reconnect(controller))

        self.assertEqual(
            result.status, BlastStationaryEvidenceStatus.MEASURED_SAFE,
        )
        self.assertEqual(result.reconnect_generations, 1)
        self.assertEqual(result.settled_attempts, 2)
        self.assertEqual(
            controller.commands, [SETTLED_OBSERVATION_COMMAND] * 2,
        )

    def test_motorless_timeout_discards_old_snapshot_until_new_generation(self):
        controller = Controller((444,))

        def timeout_once(value, attempt):
            if attempt != 1:
                return None
            value.clock[0] += 1
            value.snapshot_value.update({
                "last_observed_at_monotonic_ms": value.clock[0],
                "observation": observation(999),
            })
            raise BlastControllerError(
                "controller_command_timeout",
                "injected motorless observation timeout",
                motion_started=False,
            )

        def reconnect(value):
            if value.generation[0] == 1:
                value.generation[0] = 2

        controller.command_hook = timeout_once
        result = self.collect(controller, pause=lambda: reconnect(controller))

        self.assertEqual(
            result.status, BlastStationaryEvidenceStatus.MEASURED_SAFE,
        )
        self.assertEqual(result.observation["distance_mm"], 444)
        self.assertNotEqual(result.observation["distance_mm"], 999)
        self.assertEqual(result.reconnect_generations, 1)
        self.assertEqual(result.settled_attempts, 2)

    def test_second_generation_change_exhausts_without_third_command(self):
        controller = Controller((500,))

        def fail_each_generation(value, attempt):
            value.snapshot_value["state"] = "offline"
            if attempt == 2:
                value.generation[0] = 3
            raise BlastControllerError(
                "controller_unavailable", "injected disconnect",
                motion_started=False,
            )

        def reconnect_once(value):
            if value.generation[0] == 1:
                value.generation[0] = 2
                value.snapshot_value["state"] = "online"

        controller.command_hook = fail_each_generation
        result = self.collect(
            controller, pause=lambda: reconnect_once(controller),
        )

        self.assertEqual(
            result.status, BlastStationaryEvidenceStatus.EXHAUSTED,
        )
        self.assertEqual(result.reconnect_generations, 1)
        self.assertEqual(result.settled_attempts, 2)
        self.assertEqual(
            controller.commands, [SETTLED_OBSERVATION_COMMAND] * 2,
        )

    def test_offline_wait_is_bounded_without_consuming_settle_attempt(self):
        controller = Controller((500,))
        controller.snapshot_value["state"] = "offline"

        result = self.collect(controller)

        self.assertEqual(
            result.status, BlastStationaryEvidenceStatus.EXHAUSTED,
        )
        self.assertEqual(result.settled_attempts, 0)
        self.assertEqual(controller.commands, [])
        self.assertEqual(result.reason, "controller_offline")

    def test_missing_stationary_encoder_anchor_never_issues_a_command(self):
        controller = Controller((500,))
        controller.snapshot_value["observation"]["motor_angles_deg"].pop(
            "left_drive",
        )

        result = self.collect(controller)

        self.assertEqual(
            result.status, BlastStationaryEvidenceStatus.EXHAUSTED,
        )
        self.assertEqual(result.reason, "drive_encoders_missing")
        self.assertEqual(result.settled_attempts, 0)
        self.assertEqual(controller.commands, [])

    def test_control_wins_before_during_and_after_stationary_receipt(self):
        for phase in ("before", "during", "after"):
            with self.subTest(phase=phase):
                controller = Controller((500,))
                state = {"calls": 0, "trigger": False}

                def control():
                    state["calls"] += 1
                    if phase == "before":
                        return "stopped"
                    if phase == "during" and state["trigger"]:
                        return "stopped"
                    if phase == "after" and state["calls"] >= 4:
                        return "stopped"
                    return None

                if phase == "during":
                    def trigger(value, _attempt):
                        state["trigger"] = True
                        value.clock[0] += 1
                        settled = observation()
                        value.snapshot_value.update({
                            "last_observed_at_monotonic_ms": value.clock[0],
                            "observation": settled,
                        })
                        return {
                            "robot_id": ROBOT_ID,
                            "controller_id": CONTROLLER_ID,
                            "command": SETTLED_OBSERVATION_COMMAND,
                            "accepted": True,
                            "completed": True,
                            "observation_settled": True,
                            "observation": settled,
                        }
                    controller.command_hook = trigger

                result = self.collect(controller, control=control)

                self.assertEqual(
                    result.status,
                    BlastStationaryEvidenceStatus.CONTROLLED,
                )
                self.assertEqual(result.control, "stopped")
                self.assertEqual(
                    controller.commands,
                    [] if phase == "before"
                    else [SETTLED_OBSERVATION_COMMAND],
                )

    def test_imu_state_and_heading_are_diagnostic_for_stationary_evidence(self):
        for imu in (
            {"ready": True, "stationary": False, "heading_deg": 38.96},
            {"ready": False, "stationary": False},
        ):
            with self.subTest(imu=imu):
                controller = Controller((500,))

                def inject(value, _attempt):
                    value.clock[0] += 1
                    settled = observation()
                    settled["imu"] = dict(imu)
                    value.snapshot_value.update({
                        "last_observed_at_monotonic_ms": value.clock[0],
                        "observation": settled,
                    })
                    return {
                        "robot_id": ROBOT_ID,
                        "controller_id": CONTROLLER_ID,
                        "command": SETTLED_OBSERVATION_COMMAND,
                        "accepted": True,
                        "completed": True,
                        "observation_settled": True,
                        "observation": copy.deepcopy(settled),
                    }

                controller.command_hook = inject
                result = self.collect(controller)

                self.assertEqual(
                    result.status,
                    BlastStationaryEvidenceStatus.MEASURED_SAFE,
                )

    def test_integrity_faults_exhaust_without_retry_or_motion(self):
        for fault in ("motion", "encoder", "body", "close"):
            with self.subTest(fault=fault):
                controller = Controller((500,))

                def inject(value, _attempt):
                    value.clock[0] += 1
                    settled = observation(40 if fault == "close" else 500)
                    if fault == "motion":
                        settled["motion_active"] = True
                    elif fault == "encoder":
                        settled["motor_angles_deg"]["left_drive"] = 40
                    elif fault == "body":
                        settled["motor_angles_deg"]["body"] = 120
                    value.snapshot_value.update({
                        "last_observed_at_monotonic_ms": value.clock[0],
                        "observation": settled,
                    })
                    return {
                        "robot_id": ROBOT_ID,
                        "controller_id": CONTROLLER_ID,
                        "command": SETTLED_OBSERVATION_COMMAND,
                        "accepted": True,
                        "completed": True,
                        "observation_settled": True,
                        "observation": copy.deepcopy(settled),
                    }

                controller.command_hook = inject
                result = self.collect(controller)

                self.assertEqual(
                    result.status,
                    BlastStationaryEvidenceStatus.EXHAUSTED,
                )
                self.assertEqual(result.settled_attempts, 1)
                self.assertEqual(
                    controller.commands, [SETTLED_OBSERVATION_COMMAND],
                )

    def test_fresh_unsettled_close_range_stops_before_second_measurement(self):
        controller = Controller((500, 500))

        def close_unsettled(value, _attempt):
            value.clock[0] += 1
            observed = observation(40)
            value.snapshot_value.update({
                "last_observed_at_monotonic_ms": value.clock[0],
                "observation": observed,
            })
            return {
                "robot_id": ROBOT_ID,
                "controller_id": CONTROLLER_ID,
                "command": SETTLED_OBSERVATION_COMMAND,
                "accepted": True,
                "completed": True,
                "observation_settled": False,
                "observation": copy.deepcopy(observed),
            }

        controller.command_hook = close_unsettled

        result = self.collect(controller)

        self.assertEqual(
            result.status, BlastStationaryEvidenceStatus.EXHAUSTED,
        )
        self.assertEqual(result.reason, "measured_clearance_unsafe")
        self.assertEqual(result.settled_attempts, 1)
        self.assertEqual(
            controller.commands, [SETTLED_OBSERVATION_COMMAND],
        )

    def test_asymmetric_close_evidence_stops_before_second_measurement(self):
        for close_source in ("receipt", "snapshot"):
            with self.subTest(close_source=close_source):
                controller = Controller((500, 500))

                def asymmetric(value, _attempt):
                    value.clock[0] += 1
                    receipt_observation = observation(
                        40 if close_source == "receipt" else 2_000,
                    )
                    snapshot_observation = observation(
                        2_000 if close_source == "receipt" else 40,
                    )
                    value.snapshot_value.update({
                        "last_observed_at_monotonic_ms": value.clock[0],
                        "observation": snapshot_observation,
                    })
                    return {
                        "robot_id": ROBOT_ID,
                        "controller_id": CONTROLLER_ID,
                        "command": SETTLED_OBSERVATION_COMMAND,
                        "accepted": True,
                        "completed": True,
                        "observation_settled": False,
                        "observation": receipt_observation,
                    }

                controller.command_hook = asymmetric

                result = self.collect(controller)

                self.assertEqual(
                    result.status, BlastStationaryEvidenceStatus.EXHAUSTED,
                )
                self.assertEqual(
                    result.reason, "measured_clearance_unsafe",
                )
                self.assertEqual(result.settled_attempts, 1)
                self.assertEqual(
                    controller.commands, [SETTLED_OBSERVATION_COMMAND],
                )

    def test_fresh_close_snapshot_overrides_malformed_receipt(self):
        for fault in ("incomplete", "wrong_identity", "missing_observation"):
            with self.subTest(fault=fault):
                controller = Controller((500, 500))

                def malformed_receipt(value, _attempt):
                    value.clock[0] += 1
                    value.snapshot_value.update({
                        "last_observed_at_monotonic_ms": value.clock[0],
                        "observation": observation(40),
                    })
                    receipt = {
                        "robot_id": ROBOT_ID,
                        "controller_id": CONTROLLER_ID,
                        "command": SETTLED_OBSERVATION_COMMAND,
                        "accepted": True,
                        "completed": fault != "incomplete",
                        "observation_settled": False,
                        "observation": observation(500),
                    }
                    if fault == "wrong_identity":
                        receipt["robot_id"] = "wrong-robot"
                    elif fault == "missing_observation":
                        receipt.pop("observation")
                    return receipt

                controller.command_hook = malformed_receipt

                result = self.collect(controller)

                self.assertEqual(
                    result.status, BlastStationaryEvidenceStatus.EXHAUSTED,
                )
                self.assertEqual(
                    result.reason, "measured_clearance_unsafe",
                )
                self.assertEqual(result.settled_attempts, 1)
                self.assertEqual(
                    controller.commands, [SETTLED_OBSERVATION_COMMAND],
                )

    def test_fresh_close_receipt_overrides_missing_or_stale_snapshot(self):
        for fault in ("missing", "stale"):
            with self.subTest(fault=fault):
                controller = Controller((500, 500))

                def malformed_snapshot(value, _attempt):
                    if fault != "stale":
                        value.clock[0] += 1
                    value.snapshot_value.update({
                        "last_observed_at_monotonic_ms": value.clock[0],
                        "observation": (
                            None if fault == "missing" else observation(500)
                        ),
                    })
                    return {
                        "robot_id": ROBOT_ID,
                        "controller_id": CONTROLLER_ID,
                        "command": SETTLED_OBSERVATION_COMMAND,
                        "accepted": True,
                        "completed": True,
                        "observation_settled": False,
                        "observation": observation(40),
                    }

                controller.command_hook = malformed_snapshot

                result = self.collect(controller)

                self.assertEqual(
                    result.status, BlastStationaryEvidenceStatus.EXHAUSTED,
                )
                self.assertEqual(
                    result.reason, "measured_clearance_unsafe",
                )
                self.assertEqual(result.settled_attempts, 1)
                self.assertEqual(
                    controller.commands, [SETTLED_OBSERVATION_COMMAND],
                )

    def test_result_is_detached_from_controller_mutation(self):
        controller = Controller((500,))

        result = self.collect(controller)
        controller.snapshot_value["observation"]["distance_mm"] = 1

        self.assertEqual(result.observation["distance_mm"], 500)
        self.assertEqual(result.receipt["observation"]["distance_mm"], 500)


if __name__ == "__main__":
    unittest.main()
