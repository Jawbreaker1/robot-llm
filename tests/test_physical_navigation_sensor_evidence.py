import copy
import unittest

from robot_agent.physical_navigation_contract import (
    observation_safety_signature,
    validate_observation,
)
from robot_agent.physical_navigation_sensor_evidence import (
    EV3_IR_PROXIMITY_SOURCE,
    EV3_TOUCH_SOURCE,
    PhysicalSensorEvidence,
    sensor_evidence_from_validated_ev3_observation,
)
from robot_agent.physical_observation_progress import (
    observation_progress_signature,
)


def observation(*, touch=False, blocked=False, raw=60, filtered=60, fault=False):
    return {
        "state_version": 1,
        "observed_monotonic_ms": 10,
        "touch": {"value0": 1 if touch else 0, "pressed": touch},
        "infrared": {
            "raw": raw,
            "filtered": filtered,
            "blocked": blocked,
            "reason": "test_evidence",
            "sample_count": 5,
        },
        "motors": [
            {"role": "left_drive", "position": 10, "state": ""},
            {"role": "right_drive", "position": 20, "state": ""},
        ],
        "last_outcome": {"kind": "observe", "status": "completed"},
        "budgets": {
            "pulse_count": 0,
            "pulse_count_remaining": 40,
            "pulse_duration_ms": 0,
            "pulse_duration_ms_remaining": 32_000,
            "process_ms_remaining": 40_000,
            "motion_fault_latched": fault,
        },
    }


class PhysicalNavigationSensorEvidenceTests(unittest.TestCase):
    def test_ev3_extraction_preserves_sensor_provenance(self):
        evidence = sensor_evidence_from_validated_ev3_observation(
            validate_observation(observation())
        )

        self.assertEqual(evidence.contact.source, EV3_TOUCH_SOURCE)
        self.assertFalse(evidence.contact.pressed)
        self.assertEqual(
            evidence.clearance.source,
            EV3_IR_PROXIMITY_SOURCE,
        )
        self.assertTrue(evidence.clearance.sample_available)
        self.assertFalse(evidence.clearance.blocked)
        self.assertIsNone(evidence.clearance.distance_mm)

    def test_contact_and_clearance_are_independent(self):
        for touch in (False, True):
            for blocked in (False, True):
                with self.subTest(touch=touch, blocked=blocked):
                    evidence = sensor_evidence_from_validated_ev3_observation(
                        validate_observation(
                            observation(touch=touch, blocked=blocked)
                        )
                    )
                    self.assertEqual(evidence.contact.pressed, touch)
                    self.assertEqual(evidence.clearance.blocked, blocked)

    def test_missing_contact_is_distinct_from_unpressed(self):
        unpressed = sensor_evidence_from_validated_ev3_observation(
            validate_observation(observation())
        )
        absent = PhysicalSensorEvidence(
            contact=None,
            clearance=unpressed.clearance,
        )

        self.assertIsNotNone(unpressed.contact)
        self.assertFalse(unpressed.contact.pressed)
        self.assertIsNone(absent.contact)

    def test_ev3_derived_behavior_is_unchanged(self):
        expected_keys = {
            "infrared_available",
            "infrared_blocked",
            "motion_fault_latched",
            "motor_positions",
            "touch_pressed",
        }
        for touch in (False, True):
            for blocked in (False, True):
                for fault in (False, True):
                    with self.subTest(
                        touch=touch,
                        blocked=blocked,
                        fault=fault,
                    ):
                        value = observation(
                            touch=touch,
                            blocked=blocked,
                            fault=fault,
                        )
                        self.assertEqual(
                            observation_safety_signature(value),
                            (touch, blocked, fault),
                        )
                        facts = dict(observation_progress_signature(value))
                        self.assertEqual(set(facts), expected_keys)
                        self.assertTrue(facts["infrared_available"])
                        self.assertEqual(facts["infrared_blocked"], blocked)
                        self.assertEqual(facts["touch_pressed"], touch)
                        self.assertEqual(
                            facts["motion_fault_latched"],
                            fault,
                        )
                        self.assertEqual(
                            facts["motor_positions"],
                            (("left_drive", 10), ("right_drive", 20)),
                        )

        unavailable = observation(blocked=True, raw=None, filtered=None)
        unavailable_facts = dict(
            observation_progress_signature(unavailable)
        )
        self.assertFalse(unavailable_facts["infrared_available"])
        self.assertTrue(unavailable_facts["infrared_blocked"])

        jittered = copy.deepcopy(observation())
        jittered["infrared"]["raw"] = 57
        jittered["infrared"]["filtered"] = 58
        self.assertEqual(
            observation_progress_signature(observation()),
            observation_progress_signature(jittered),
        )


if __name__ == "__main__":
    unittest.main()
