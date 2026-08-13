# Navigation ownership

This is the target contract for both physical LEGO robots. The implementations
are migrated toward it in small, separately validated changes.

1. Each navigation episode acquires full surroundings evidence before the
   first navigation decision. The scan's motor order is an implementation
   detail, not a left/right route choice.
2. Gemma chooses the bounded semantic intent, such as turning, scanning, or
   advancing while the path remains clear.
3. The robot-specific executor may use several low-level motor pulses for that
   intent. It checks safety and progress between pulses and stops on a relevant
   event; this does not require a model call per pulse.
4. The host may reject or stop an unsafe action. It must not choose a substitute
   direction, waypoint route, or follow-up semantic action.
5. Ordinary missing or noisy evidence returns to observation and replanning.
   User Stop, ambiguous motor state, controller loss, or materially invalid
   localization remain hard stops.

The navigation runtimes support mobile navigation and surroundings perception.
Other physical requests must be reported as unsupported and must not be
converted into a directional navigation mission.
