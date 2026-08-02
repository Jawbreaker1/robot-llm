import unittest

from robot_agent.maneuver_commitment import (
    ActiveManeuver,
    FACT_GOAL_CORRIDOR_CLEAR,
    ManeuverCommitment,
    ManeuverCommitmentError,
    empty_commitment,
)
from robot_agent.physical_navigation_contract import (
    ADVANCE,
    FINISH,
    OBSERVE,
    SCAN_FRONT_ARC,
)
from robot_agent.physical_odometry import PhysicalPose


TARGET_ID = "box-a"


class _HazardMap:
    hazard_ids = (TARGET_ID, "box-b")


def _active_maneuver():
    return ActiveManeuver(
        commitment_id="maneuver-a",
        revision=1,
        objective="Pass the remembered obstacle",
        target_hypothesis_id=TARGET_ID,
        detour_side="LEFT_OF_GOAL",
        success_fact_keys=(FACT_GOAL_CORRIDOR_CLEAR,),
        current_focus_fact_key=FACT_GOAL_CORRIDOR_CLEAR,
        started_turn=1,
        last_confirmed_turn=1,
    )


def _complete_proposal():
    active = _active_maneuver()
    return {
        "id": active.commitment_id,
        "revision": active.revision,
        "transition": "COMPLETE",
        "objective": active.objective,
        "target_hypothesis_id": active.target_hypothesis_id,
        "detour_side": active.detour_side,
        "success_fact_keys": list(active.success_fact_keys),
        "current_focus_fact_key": None,
        "revision_reason": None,
    }


class ManeuverCommitmentPersistenceTests(unittest.TestCase):
    def _apply_none(
        self,
        commitment,
        *,
        action=ADVANCE,
        turn=2,
        target_id=None,
    ):
        return commitment.apply(
            empty_commitment(),
            action=action,
            turn=turn,
            hazard_map=_HazardMap(),
            pose=PhysicalPose(),
            fact_values={},
            perception_target_hypothesis_id=target_id,
        )

    def _apply_complete(self, commitment, *, action, fact_value=True):
        return commitment.apply(
            _complete_proposal(),
            action=action,
            turn=2,
            hazard_map=_HazardMap(),
            pose=PhysicalPose(),
            fact_values={FACT_GOAL_CORRIDOR_CLEAR: fact_value},
        )

    def test_none_preserves_an_active_intent(self):
        commitment = ManeuverCommitment(_active_maneuver())

        state = self._apply_none(commitment)

        self.assertEqual(state["active"], _active_maneuver().prompt_dict())
        self.assertIsNone(state["last_terminal"])

    def test_reading_state_does_not_expire_an_active_intent(self):
        commitment = ManeuverCommitment(_active_maneuver())

        first = commitment.state(1)
        much_later = commitment.state(10_000)

        self.assertEqual(much_later, first)
        self.assertEqual(commitment.active, _active_maneuver())

    def test_none_allows_scanning_the_active_target(self):
        commitment = ManeuverCommitment(_active_maneuver())

        state = self._apply_none(
            commitment,
            action=SCAN_FRONT_ARC,
            target_id=TARGET_ID,
        )

        self.assertEqual(
            state["active"]["target_hypothesis_id"],
            TARGET_ID,
        )

    def test_none_rejects_scanning_a_different_target(self):
        commitment = ManeuverCommitment(_active_maneuver())

        with self.assertRaises(ManeuverCommitmentError) as caught:
            self._apply_none(
                commitment,
                action=SCAN_FRONT_ARC,
                target_id="box-b",
            )

        self.assertEqual(caught.exception.code, "route_changed_during_scan")

    def test_instances_keep_independent_intents(self):
        first = ManeuverCommitment(_active_maneuver())
        second = ManeuverCommitment()

        self._apply_none(first)

        self.assertIsNotNone(first.state(2)["active"])
        self.assertIsNone(second.state(2)["active"])

    def test_observe_can_complete_an_intent_without_finishing_the_mission(self):
        commitment = ManeuverCommitment(_active_maneuver())

        state = self._apply_complete(commitment, action=OBSERVE)

        self.assertIsNone(state["active"])
        self.assertEqual(state["last_terminal"]["transition"], "COMPLETE")

    def test_finish_remains_valid_when_completing_an_intent(self):
        commitment = ManeuverCommitment(_active_maneuver())

        state = self._apply_complete(commitment, action=FINISH)

        self.assertIsNone(state["active"])
        self.assertEqual(state["last_terminal"]["transition"], "COMPLETE")

    def test_motion_cannot_complete_an_intent(self):
        commitment = ManeuverCommitment(_active_maneuver())

        with self.assertRaises(ManeuverCommitmentError) as caught:
            self._apply_complete(commitment, action=ADVANCE)

        self.assertEqual(
            caught.exception.code,
            "complete_requires_observe_or_finish",
        )

    def test_complete_still_requires_every_intent_fact(self):
        commitment = ManeuverCommitment(_active_maneuver())

        with self.assertRaises(ManeuverCommitmentError) as caught:
            self._apply_complete(
                commitment,
                action=OBSERVE,
                fact_value=False,
            )

        self.assertEqual(caught.exception.code, "maneuver_facts_not_complete")


if __name__ == "__main__":
    unittest.main()
