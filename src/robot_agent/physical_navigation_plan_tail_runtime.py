"""Runtime support for short model-authored motion tails."""

from dataclasses import dataclass
from typing import Mapping, Tuple

from .navigation_plan_tail import NavigationPlanTail
from .physical_navigation_experience import PLAN_TAIL_ACTION_SOURCE
from .physical_navigation_mission import DirectionalMission


@dataclass(frozen=True)
class PhysicalNavigationPlanTailResult:
    observation: Mapping[str, object]
    last_tool_result: Mapping[str, object]
    actions: Tuple[str, ...]
    completed: int
    cancelled: int


class PhysicalNavigationPlanTailRuntimeMixin:
    """Execute a fresh exact tail without selecting or substituting actions."""

    def _execute_navigation_plan_tail(
        self,
        *,
        tail: NavigationPlanTail,
        turn: int,
        deadline: float,
        observation: Mapping[str, object],
        last_tool_result: Mapping[str, object],
        action_specs: Mapping[str, Mapping[str, object]],
        mission: DirectionalMission,
        maneuver,
    ) -> PhysicalNavigationPlanTailResult:
        actions = []
        completed = 0
        cancelled = 0
        while not tail.complete:
            self._raise_if_cancelled("before_plan_tail_action_validation")
            _mission_value, navigation = self._goal_state(
                mission,
                observation,
                action_specs,
            )
            next_action = tail.next_action(
                now_monotonic=self.monotonic(),
                map_context=navigation,
                observation=observation,
                maneuver_state=maneuver.state(turn),
                fact_values=navigation["fact_values"],
                localization_valid=self.memory.localization_valid,
            )
            if next_action is None:
                cancelled += 1
                self._emit(
                    "plan_tail_cancelled",
                    reason=tail.cancelled_reason,
                    source_plan=list(tail.source_plan),
                )
                break
            basis_before = self._experience_basis(observation)
            veto = self._execution_veto(
                action=next_action,
                observation=observation,
                action_specs=action_specs,
                deadline=deadline,
            )
            if veto is not None:
                tail.cancel(veto["code"])
                cancelled += 1
                last_tool_result = {
                    "operation": "pulse",
                    "status": "tail_vetoed",
                    "validation": veto,
                }
                self._emit(
                    "plan_tail_cancelled",
                    reason=tail.cancelled_reason,
                    source_plan=list(tail.source_plan),
                )
                self._record_experience(
                    turn=turn,
                    action=next_action,
                    source=PLAN_TAIL_ACTION_SOURCE,
                    result=last_tool_result,
                    basis_before=basis_before,
                    observation_after=observation,
                )
                break
            self._raise_if_cancelled("immediately_before_plan_tail_motion")
            observation, last_tool_result = self._execute_motion(
                next_action,
                action_specs=action_specs,
            )
            actions.append(next_action)
            self._record_experience(
                turn=turn,
                action=next_action,
                source=PLAN_TAIL_ACTION_SOURCE,
                result=last_tool_result,
                basis_before=basis_before,
                observation_after=observation,
            )
            if last_tool_result["status"] != "completed":
                tail.cancel("tail_motion_not_completed")
                cancelled += 1
                break
            tail.mark_executed(
                next_action,
                map_context=self.memory.context(),
                observation=observation,
            )
            if tail.complete:
                completed += 1
                self._emit(
                    "plan_tail_completed",
                    source_plan=list(tail.source_plan),
                )
        return PhysicalNavigationPlanTailResult(
            observation=observation,
            last_tool_result=last_tool_result,
            actions=tuple(actions),
            completed=completed,
            cancelled=cancelled,
        )


__all__ = (
    "PhysicalNavigationPlanTailResult",
    "PhysicalNavigationPlanTailRuntimeMixin",
)
