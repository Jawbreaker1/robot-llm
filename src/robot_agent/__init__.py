"""Transport-independent building blocks for the Robot LLM experiment."""

from .commentary import (
    DEFAULT_OBSTACLE_GATE_POLICY,
    DEFAULT_PROXIMITY_THRESHOLDS,
    ObstacleEvidenceGate,
    ObstacleGatePolicy,
    ProximityObservation,
    ProximityThresholds,
    StableZoneTracker,
    classify_infrared,
    fallback_comment,
    validate_generated_comment,
)
from .contract import CommandResult, MotionCommand, MotorState, RobotState
from .lm_studio import (
    LMStudioError,
    ModelCandidate,
    NativeLMStudioClient,
)
from .safety import SafetyLimits, SafetyPolicy, SafetyViolation
from .shadow_commentary import (
    ShadowCommentResult,
    ShadowSpeechError,
    run_shadow_comment,
)
from .simulated_robot import SimulatedRobot

__all__ = [
    "CommandResult",
    "MotionCommand",
    "MotorState",
    "RobotState",
    "LMStudioError",
    "ModelCandidate",
    "NativeLMStudioClient",
    "DEFAULT_OBSTACLE_GATE_POLICY",
    "DEFAULT_PROXIMITY_THRESHOLDS",
    "ObstacleEvidenceGate",
    "ObstacleGatePolicy",
    "ProximityObservation",
    "ProximityThresholds",
    "SafetyLimits",
    "SafetyPolicy",
    "SafetyViolation",
    "ShadowCommentResult",
    "ShadowSpeechError",
    "SimulatedRobot",
    "StableZoneTracker",
    "classify_infrared",
    "fallback_comment",
    "run_shadow_comment",
    "validate_generated_comment",
]
