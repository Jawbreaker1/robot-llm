# Navigation ownership

This is the target contract for both physical LEGO robots. The implementations
are migrated toward it in small, separately validated changes.

1. Each navigation episode begins with a full surroundings scan before the
   first navigation decision. Missing rays remain unknown; they do not by
   themselves invalidate the episode. The scan's motor order is an
   implementation detail, not a left/right route choice.
2. The configured model (currently Qwen) chooses the bounded semantic intent,
   such as turning, scanning, or
   advancing while the path remains clear.
3. The robot-specific executor may use several low-level motor pulses for that
   intent. It checks safety and progress between pulses and stops on a relevant
   event; this does not require a model call per pulse.
4. The host may reject or stop an unsafe action. It must not choose a substitute
   direction, waypoint route, or follow-up semantic action.
5. Short missing/noisy readings do not erase useful earlier evidence or require
   another scan or model call. Unknown space may be explored; it is not a
   blanket no-go area. Actual failed motion returns control to the model with
   goal, route, map and verified pose retained. User Stop, ambiguous motor state,
   controller loss, or materially invalid localization remain hard stops.
6. BLAST has one episode-local motion heading. Short command/scan gyro deltas
   refine encoder motion; raw absolute gyro drift while waiting for the model
   must not rotate the estimated robot or the map. The same heading is used by
   waypoint alignment, motion feedback, scan projection and the displayed pose.
7. Observations update the map instead of replacing it wholesale. No-return
   and incomplete scans leave earlier measured obstacles in memory. Nearby
   repeat hits replace older evidence; a measured ray through it can clear it.

The navigation runtimes support mobile navigation and surroundings perception.
Other physical requests must be reported as unsupported and must not be
converted into a directional navigation mission.
