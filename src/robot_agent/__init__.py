"""Lazily exported building blocks for the Robot LLM experiment.

Keeping package initialization side-effect free matters: a read-only research
command must not load the robot execution stack merely because Python imports
the package before its submodule.
"""

from importlib import import_module


_EXPORTS = {
    "ActionContext": (".robot_api", "ActionContext"),
    "ActionReceipt": (".robot_api", "ActionReceipt"),
    "AnswerIntent": (".research_loop", "AnswerIntent"),
    "CapabilityGate": (".robot_api", "CapabilityGate"),
    "ClosedLoopAgent": (".agent_loop", "ClosedLoopAgent"),
    "CommandResult": (".contract", "CommandResult"),
    "ControllerCapabilities": (
        ".robot_api",
        "ControllerCapabilities",
    ),
    "CurrentWeather": (".research", "CurrentWeather"),
    "DEFAULT_OBSTACLE_GATE_POLICY": (
        ".commentary",
        "DEFAULT_OBSTACLE_GATE_POLICY",
    ),
    "DEFAULT_PROXIMITY_THRESHOLDS": (
        ".commentary",
        "DEFAULT_PROXIMITY_THRESHOLDS",
    ),
    "DecisionProposal": (".agent_loop", "DecisionProposal"),
    "EvidenceProvenance": (".research", "EvidenceProvenance"),
    "EpisodeResult": (".agent_loop", "EpisodeResult"),
    "LMStudioError": (".lm_studio", "LMStudioError"),
    "LMStudioResearchPlanner": (
        ".lm_studio_research",
        "LMStudioResearchPlanner",
    ),
    "LoopLimits": (".agent_loop", "LoopLimits"),
    "ModelCandidate": (".lm_studio", "ModelCandidate"),
    "MotionCommand": (".contract", "MotionCommand"),
    "MotionRequest": (".robot_api", "MotionRequest"),
    "MotorCapability": (".robot_api", "MotorCapability"),
    "MotorPositionGoal": (".agent_loop", "MotorPositionGoal"),
    "MotorState": (".contract", "MotorState"),
    "NativeLMStudioClient": (".lm_studio", "NativeLMStudioClient"),
    "ObstacleEvidenceGate": (
        ".commentary",
        "ObstacleEvidenceGate",
    ),
    "ObstacleGatePolicy": (".commentary", "ObstacleGatePolicy"),
    "ObservationEnvelope": (".robot_api", "ObservationEnvelope"),
    "OpenMeteoWeatherTool": (
        ".research",
        "OpenMeteoWeatherTool",
    ),
    "PlanningContext": (".agent_loop", "PlanningContext"),
    "ProposalError": (".agent_loop", "ProposalError"),
    "ProximityObservation": (".commentary", "ProximityObservation"),
    "ProximityThresholds": (".commentary", "ProximityThresholds"),
    "ResearchDecision": (".research_loop", "ResearchDecision"),
    "ResearchEpisodeResult": (
        ".research_loop",
        "ResearchEpisodeResult",
    ),
    "ResearchError": (".research", "ResearchError"),
    "ResearchEvidenceEnvelope": (
        ".research_loop",
        "ResearchEvidenceEnvelope",
    ),
    "ResearchGoal": (".research_loop", "ResearchGoal"),
    "ResearchLimits": (".research_loop", "ResearchLimits"),
    "ResearchLoop": (".research_loop", "ResearchLoop"),
    "ResearchLoopError": (".research_loop", "ResearchLoopError"),
    "ResearchPlanningContext": (
        ".research_loop",
        "ResearchPlanningContext",
    ),
    "ResearchToolRegistry": (
        ".research_loop",
        "ResearchToolRegistry",
    ),
    "ResolvedLocation": (".research", "ResolvedLocation"),
    "RobotActionRejected": (".robot_api", "RobotActionRejected"),
    "RobotAPI": (".robot_api", "RobotAPI"),
    "RobotAPIContractError": (
        ".robot_api",
        "RobotAPIContractError",
    ),
    "RobotAPIError": (".robot_api", "RobotAPIError"),
    "RobotState": (".contract", "RobotState"),
    "SafetyLimits": (".safety", "SafetyLimits"),
    "SafetyPolicy": (".safety", "SafetyPolicy"),
    "SafetyViolation": (".safety", "SafetyViolation"),
    "ShadowCommentResult": (
        ".shadow_commentary",
        "ShadowCommentResult",
    ),
    "ShadowSpeechError": (".shadow_commentary", "ShadowSpeechError"),
    "SimulatedRobot": (".simulated_robot", "SimulatedRobot"),
    "SimulatedRobotAPI": (".robot_api", "SimulatedRobotAPI"),
    "StableZoneTracker": (".commentary", "StableZoneTracker"),
    "StopRequest": (".robot_api", "StopRequest"),
    "SupervisorRemoteError": (
        ".supervisor_transport",
        "SupervisorRemoteError",
    ),
    "SupervisorSSHChannelPoisonedError": (
        ".supervisor_transport",
        "SupervisorSSHChannelPoisonedError",
    ),
    "SupervisorSSHConfigurationError": (
        ".supervisor_transport",
        "SupervisorSSHConfigurationError",
    ),
    "SupervisorSSHError": (
        ".supervisor_transport",
        "SupervisorSSHError",
    ),
    "SupervisorSSHProtocolError": (
        ".supervisor_transport",
        "SupervisorSSHProtocolError",
    ),
    "SupervisorSSHSession": (
        ".supervisor_transport",
        "SupervisorSSHSession",
    ),
    "SupervisorSSHTimeoutError": (
        ".supervisor_transport",
        "SupervisorSSHTimeoutError",
    ),
    "SupervisorSSHTransportError": (
        ".supervisor_transport",
        "SupervisorSSHTransportError",
    ),
    "ToolCallIntent": (".research_loop", "ToolCallIntent"),
    "WeatherResearchRequest": (
        ".research",
        "WeatherResearchRequest",
    ),
    "WeatherResearchResult": (".research", "WeatherResearchResult"),
    "WeatherTool": (".research", "WeatherTool"),
    "classify_infrared": (".commentary", "classify_infrared"),
    "decode_decision_proposal": (
        ".agent_loop",
        "decode_decision_proposal",
    ),
    "decode_research_decision": (
        ".research_loop",
        "decode_research_decision",
    ),
    "fallback_comment": (".commentary", "fallback_comment"),
    "run_motion_free_supervisor_preflight": (
        ".supervisor_transport",
        "run_motion_free_supervisor_preflight",
    ),
    "run_shadow_comment": (
        ".shadow_commentary",
        "run_shadow_comment",
    ),
    "validate_generated_comment": (
        ".commentary",
        "validate_generated_comment",
    ),
}

__all__ = sorted(_EXPORTS)


def __getattr__(name):
    try:
        module_name, attribute_name = _EXPORTS[name]
    except KeyError:
        raise AttributeError(
            "module {!r} has no attribute {!r}".format(__name__, name)
        ) from None
    value = getattr(import_module(module_name, __name__), attribute_name)
    globals()[name] = value
    return value


def __dir__():
    return sorted(set(globals()) | set(__all__))
