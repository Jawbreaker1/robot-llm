"""Hardware-free CLI for the concurrent navigation and expression runtime."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import itertools
import json
from pathlib import Path
import sys
import threading
import time
from typing import Mapping, Optional, Sequence, Tuple

from .concurrent_runtime import (
    ConcurrentBehaviorRuntime,
    ConcurrentRuntimePolicy,
    ConcurrentRuntimeResult,
)
from .interaction_contract import (
    ExpressionIntent,
    ExpressionProposal,
    InteractionContractError,
    InteractionSnapshot,
    expression_proposal_id_for_snapshot,
)
from .lm_studio import (
    DEFAULT_BASE_URL,
    DEFAULT_MODEL,
    LMStudioError,
)
from .navigation_contract import (
    MotionAuthority,
    NavigationContractError,
)
from .navigation_demo import DEFAULT_CONFIG_PATH, load_demo_config
from .navigation_episode import NavigationLimits
from .navigation_simulator import (
    DifferentialDriveSimulator,
    SimulationWorld,
)
from .navigation_state import ProposalInbox, ProposalSourcePolicy
from .navigation_supervisor import MotionPolicy, MotionSupervisor


REPORT_SCHEMA = "robot-concurrent-simulation-report/v1"
DEFAULT_LOCALE = "sv-SE"
DEFAULT_TICK_MS = 5
DEFAULT_EVENT_LIMIT = 96
MAX_EVENT_LIMIT = 256


def _locale(value: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > 64
        or any(ord(character) < 32 for character in value)
    ):
        raise argparse.ArgumentTypeError("locale is invalid")
    return value


def _bounded_integer(
    name: str,
    minimum: int,
    maximum: int,
):
    def convert(raw: str) -> int:
        try:
            value = int(raw, 10)
        except (TypeError, ValueError):
            raise argparse.ArgumentTypeError(
                "{} must be an integer".format(name)
            ) from None
        if not minimum <= value <= maximum:
            raise argparse.ArgumentTypeError(
                "{} must be between {} and {}".format(
                    name,
                    minimum,
                    maximum,
                )
            )
        return value

    return convert


def _bounded_seconds(raw: str) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError(
            "LM Studio timeout must be a number"
        ) from None
    if not 0.1 <= value <= 10.0:
        raise argparse.ArgumentTypeError(
            "LM Studio timeout must be between 0.1 and 10 seconds"
        )
    return value


class DeterministicGrumpyExpressionPlanner:
    """Typed fixture planner used when no model server is requested.

    The fixture deliberately avoids pretending to translate arbitrary
    locales.  Its short ``Grrr!`` utterance is locale-neutral; the locale
    binding is still exact and can therefore exercise the real contracts.
    """

    def __init__(self, response_locale: str):
        try:
            self._response_locale = _locale(response_locale)
        except argparse.ArgumentTypeError as error:
            raise InteractionContractError(
                "invalid_response_locale",
                str(error),
            ) from None

    @property
    def response_locale(self) -> str:
        return self._response_locale

    def __call__(
        self,
        snapshot: InteractionSnapshot,
    ) -> ExpressionProposal:
        if not isinstance(snapshot, InteractionSnapshot):
            raise InteractionContractError(
                "invalid_interaction_snapshot",
                "fixture planner requires InteractionSnapshot",
            )
        if snapshot.response_locale != self._response_locale:
            raise InteractionContractError(
                "response_locale_mismatch",
                "fixture planner locale did not match the snapshot",
            )
        evidence_id = (
            None
            if snapshot.evidence is None
            else snapshot.evidence.evidence_id
        )
        common = {
            "proposal_id": expression_proposal_id_for_snapshot(snapshot),
            "robot_id": snapshot.robot_id,
            "controller_instance_id": snapshot.controller_instance_id,
            "goal_id": snapshot.goal_id,
            "goal_epoch": snapshot.goal_epoch,
            "plan_revision": snapshot.plan_revision,
            "based_on_interaction_state_version": (
                snapshot.interaction_state_version
            ),
            "based_on_world_model_version": (
                snapshot.world_model_version
            ),
            "obstruction_epoch": snapshot.obstruction_epoch,
            "based_on_evidence_id": evidence_id,
        }
        if snapshot.evidence is None:
            return ExpressionProposal(
                **common,
                decision="HOLD",
                confidence_milli=1_000,
                reason_code="NO_BLOCKING_EVIDENCE",
            )

        object_id = snapshot.evidence.object_id
        gesture_kind = (
            "PROPELLER_WAVE" if object_id is not None else None
        )
        utterance = (
            "Grrr! {}!".format(object_id)
            if object_id is not None
            else "Grrr!"
        )
        return ExpressionProposal(
            **common,
            decision="EXPRESS",
            confidence_milli=1_000,
            intent=ExpressionIntent(
                utterance=utterance,
                utterance_locale=self._response_locale,
                gesture_kind=gesture_kind,
                affect_label="very_grumpy",
                intensity=950,
                repetitions=1 if gesture_kind is not None else 0,
            ),
        )


class _CallbackRecorder:
    """Thread-safe virtual speech and arm callbacks for the CLI report."""

    def __init__(self, simulated_speech_seconds: float):
        self._speech_seconds = simulated_speech_seconds
        self._lock = threading.Lock()
        self._sequence = 0
        self._records = []

    def _append(self, kind: str, detail: Mapping[str, object]) -> None:
        with self._lock:
            self._sequence += 1
            value = {
                "sequence": self._sequence,
                "kind": kind,
            }
            value.update(detail)
            self._records.append(value)

    def speak(
        self,
        utterance: str,
        locale: str,
        cancel_event: object,
    ) -> None:
        self._append(
            "virtual_speech_started",
            {
                "locale": locale,
                "utterance": utterance,
            },
        )
        cancel_event.wait(self._speech_seconds)
        self._append(
            "virtual_speech_returned",
            {
                "cancelled": bool(cancel_event.is_set()),
            },
        )

    def arm_segment(
        self,
        speed_dps: int,
        duration_ms: int,
        cancel_event: object,
    ) -> None:
        self._append(
            "virtual_arm_segment",
            {
                "speed_dps": speed_dps,
                "duration_ms": duration_ms,
                "cancelled_before": bool(cancel_event.is_set()),
            },
        )

    def snapshot(self) -> Tuple[Mapping[str, object], ...]:
        with self._lock:
            return tuple(dict(record) for record in self._records)


@dataclass(frozen=True)
class ConcurrentDemoOutcome:
    runtime: ConcurrentRuntimeResult
    planner_mode: str
    response_locale: str
    tick_ms: int
    scenario: str
    callbacks: Tuple[Mapping[str, object], ...]
    model: Optional[str] = None

    def to_report(
        self,
        event_limit: int = DEFAULT_EVENT_LIMIT,
    ) -> Mapping[str, object]:
        if (
            isinstance(event_limit, bool)
            or not isinstance(event_limit, int)
            or not 1 <= event_limit <= MAX_EVENT_LIMIT
        ):
            raise ValueError("event_limit is invalid")
        navigation = self.runtime.navigation
        navigation_value = navigation.to_dict()
        relevant_kinds = {
            "runtime_started",
            "runtime_stopped",
            "worker_started",
            "worker_stopped",
            "navigation_tick",
            "expression_accepted",
            "expression_not_selected",
            "planner_failure",
            "planner_result_cancelled",
            "planner_cooldown_drop",
            "planner_budget_drop",
            "stale_expression_drop",
            "speech_started",
            "speech_completed",
            "speech_cancelled",
            "speech_failure",
            "stale_speech_drop",
            "navigation_pause_requested",
            "navigation_pause_deferred",
            "navigation_pause_ack",
            "navigation_pause_released",
            "arm_exclusive_timeout",
            "gesture_started",
            "gesture_completed",
            "stale_gesture_drop",
            "arm_segment_started",
            "arm_cancelled",
            "arm_failure",
            "cancel_requested",
            "queue_drop",
        }
        ordered = [
            event.to_dict()
            for event in self.runtime.events
            if event.kind in relevant_kinds
        ]
        displayed = ordered[:event_limit]
        speech_start = next(
            (
                event.sequence
                for event in self.runtime.events
                if event.kind == "speech_started"
            ),
            None,
        )
        speech_end = next(
            (
                event.sequence
                for event in self.runtime.events
                if (
                    event.kind in (
                        "speech_completed",
                        "speech_cancelled",
                        "speech_failure",
                    )
                    and speech_start is not None
                    and event.sequence > speech_start
                )
            ),
            None,
        )
        interleaved_tick = bool(
            speech_start is not None
            and speech_end is not None
            and any(
                event.kind == "navigation_tick"
                and speech_start < event.sequence < speech_end
                for event in self.runtime.events
            )
        )
        return {
            "schema": REPORT_SCHEMA,
            "status": (
                "completed" if navigation.completed else "not_completed"
            ),
            "mode": "simulation_only",
            "hardware_used": False,
            "warning": (
                "Synthetic simulator callbacks only; no robot, audio "
                "device, or arm motor was used."
            ),
            "scenario": self.scenario,
            "planner": {
                "mode": self.planner_mode,
                "response_locale": self.response_locale,
                "model": self.model,
            },
            "navigation": {
                "goal_id": navigation.goal_id,
                "completed": navigation.completed,
                "termination": navigation.termination,
                "ticks": navigation.ticks,
                "proposals": navigation.proposals,
                "actions": navigation.actions,
                "total_motion_ms": navigation.total_motion_ms,
                "terminal_stop_verified": (
                    navigation.terminal_stop_verified
                ),
                "final_pose": navigation_value["final_pose"],
                "active_faults": list(
                    navigation.final_snapshot.active_faults
                ),
            },
            "concurrency": {
                "tick_ms": self.tick_ms,
                "speech_worker_independent_of_navigation": True,
                "speech_navigation_interleaving_observed": (
                    interleaved_tick
                ),
                "arm_execution_policy": (
                    "pause_request -> stopped_ack -> revalidate -> "
                    "host_wave -> release"
                ),
                "wheel_and_arm_overlap_allowed": False,
                "clean_shutdown": self.runtime.clean_shutdown,
                "workers_alive": list(self.runtime.workers_alive),
                "metrics": self.runtime.metrics.to_dict(),
            },
            "virtual_callbacks": list(self.callbacks),
            "event_order": displayed,
            "event_order_matching_count": len(ordered),
            "event_order_truncated": len(ordered) > len(displayed),
        }


def _build_planner(
    use_lm_studio: bool,
    response_locale: str,
    lm_studio_url: str,
    model: str,
    lm_timeout_seconds: float,
):
    if not use_lm_studio:
        return (
            DeterministicGrumpyExpressionPlanner(response_locale),
            "deterministic_typed_fixture",
            None,
        )
    from .lm_studio_expression import LMStudioExpressionPlanner

    return (
        LMStudioExpressionPlanner(
            response_locale=response_locale,
            base_url=lm_studio_url,
            model=model,
            timeout_seconds=lm_timeout_seconds,
        ),
        "lm_studio_structured_output",
        model,
    )


def run_concurrent_demo(
    config_path: Path = DEFAULT_CONFIG_PATH,
    scenario: str = "obstacle",
    response_locale: str = DEFAULT_LOCALE,
    tick_ms: int = DEFAULT_TICK_MS,
    use_lm_studio: bool = False,
    lm_studio_url: str = DEFAULT_BASE_URL,
    model: str = DEFAULT_MODEL,
    lm_timeout_seconds: float = 10.0,
) -> ConcurrentDemoOutcome:
    """Run one bounded concurrent episode using virtual output callbacks."""

    if scenario not in ("obstacle", "clear"):
        raise ValueError("scenario is invalid")
    if (
        isinstance(tick_ms, bool)
        or not isinstance(tick_ms, int)
        or not 1 <= tick_ms <= 1_000
    ):
        raise ValueError("tick_ms is invalid")
    try:
        response_locale = _locale(response_locale)
    except argparse.ArgumentTypeError as error:
        raise InteractionContractError(
            "invalid_response_locale",
            str(error),
        ) from None
    config = load_demo_config(config_path)
    world = config["world"]
    if scenario == "clear":
        world = SimulationWorld(
            width_mm=world.width_mm,
            height_mm=world.height_mm,
            obstacles=(),
        )

    authority = MotionAuthority()
    plant = DifferentialDriveSimulator(
        world,
        config["profile"],
        config["start"],
        authority,
        settings=config["settings"],
    )
    decision_ids = itertools.count(1)
    supervisor = MotionSupervisor(
        plant.profile,
        plant.clock_ms,
        plant.robot_id,
        plant.controller_instance_id,
        authority,
        policy=MotionPolicy(max_pulse_ms=plant.profile.max_pulse_ms),
        id_factory=lambda: "concurrent-demo-decision-{}".format(
            next(decision_ids)
        ),
    )
    inbox = ProposalInbox(
        (
            ProposalSourcePolicy(
                "goal-seeking",
                authority_rank=10,
                priority=50,
                ttl_ms=200,
            ),
            ProposalSourcePolicy(
                "obstacle-avoidance",
                authority_rank=20,
                priority=100,
                ttl_ms=120,
            ),
        ),
        plant.clock_ms,
    )
    planner, planner_mode, selected_model = _build_planner(
        use_lm_studio,
        response_locale,
        lm_studio_url,
        model,
        lm_timeout_seconds,
    )
    recorder = _CallbackRecorder(
        simulated_speech_seconds=max(
            0.01,
            min(0.05, tick_ms * 3 / 1_000.0),
        )
    )
    runtime = ConcurrentBehaviorRuntime(
        plant=plant,
        supervisor=supervisor,
        inbox=inbox,
        goal=config["goal"],
        response_locale=response_locale,
        expression_planner=planner,
        speaker=recorder.speak,
        arm_segment_executor=recorder.arm_segment,
        navigation_limits=NavigationLimits(
            max_ticks=500,
            max_elapsed_ms=60_000,
            max_proposals=1_000,
            max_replans=500,
            max_actions=480,
            max_total_motion_ms=55_000,
            max_no_progress_ticks=120,
        ),
        policy=ConcurrentRuntimePolicy(
            tick_interval_s=tick_ms / 1_000.0,
        ),
        host_clock_ms=lambda: time.monotonic_ns() // 1_000_000,
    )
    result = runtime.run()
    return ConcurrentDemoOutcome(
        runtime=result,
        planner_mode=planner_mode,
        response_locale=response_locale,
        tick_ms=tick_ms,
        scenario=scenario,
        callbacks=recorder.snapshot(),
        model=selected_model,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run simulator-only concurrent navigation, speech, and "
            "propeller-expression callbacks."
        )
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument(
        "--scenario",
        choices=("obstacle", "clear"),
        default="obstacle",
    )
    parser.add_argument("--locale", type=_locale, default=DEFAULT_LOCALE)
    parser.add_argument(
        "--tick-ms",
        type=_bounded_integer("tick-ms", 1, 1_000),
        default=DEFAULT_TICK_MS,
    )
    parser.add_argument(
        "--event-limit",
        type=_bounded_integer(
            "event-limit",
            1,
            MAX_EVENT_LIMIT,
        ),
        default=DEFAULT_EVENT_LIMIT,
    )
    parser.add_argument(
        "--lm-studio",
        action="store_true",
        help="Use the local structured-output expression planner.",
    )
    parser.add_argument(
        "--lm-studio-url",
        default=DEFAULT_BASE_URL,
    )
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument(
        "--lm-timeout-seconds",
        type=_bounded_seconds,
        default=10.0,
    )
    parser.add_argument("--compact", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.lm_studio and (
        args.lm_studio_url != DEFAULT_BASE_URL
        or args.model != DEFAULT_MODEL
        or args.lm_timeout_seconds != 10.0
    ):
        parser.error(
            "LM Studio connection options require --lm-studio"
        )
    try:
        outcome = run_concurrent_demo(
            config_path=args.config,
            scenario=args.scenario,
            response_locale=args.locale,
            tick_ms=args.tick_ms,
            use_lm_studio=args.lm_studio,
            lm_studio_url=args.lm_studio_url,
            model=args.model,
            lm_timeout_seconds=args.lm_timeout_seconds,
        )
        report = outcome.to_report(args.event_limit)
    except (
        InteractionContractError,
        LMStudioError,
        NavigationContractError,
        OSError,
        ValueError,
    ) as error:
        print(
            json.dumps(
                {
                    "schema": REPORT_SCHEMA,
                    "status": "configuration_failed",
                    "mode": "simulation_only",
                    "hardware_used": False,
                    "error": type(error).__name__,
                    "message": str(error)[:240],
                },
                ensure_ascii=False,
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2

    print(
        json.dumps(
            report,
            ensure_ascii=False,
            indent=None if args.compact else 2,
            sort_keys=True,
        )
    )
    return 0 if outcome.runtime.navigation.completed else 1


if __name__ == "__main__":
    raise SystemExit(main())
