# Navigation ownership

This is the target contract for both physical LEGO robots. The implementations
are migrated toward it in small, separately validated changes.

1. Each navigation episode begins with a full surroundings scan before the
   first navigation decision. Missing rays remain unknown; they do not by
   themselves invalidate the episode. The scan's motor order is an
   implementation detail, not a left/right route choice.
2. The configured model (Qwen or Astra) chooses the semantic intent and route.
   Movement is allowed under uncertainty: absence of a clear return is not
   an obstacle and does not require proving an entire leg free before moving.
3. The robot-specific executor may use several low-level motor pulses for that
   intent. It checks safety and progress between pulses and stops on a relevant
   event; this does not require a model call per pulse.
4. The host may reject or stop an unsafe action. It must not choose a substitute
   direction, waypoint route, or follow-up semantic action.
   Only `FOLLOW_WAYPOINT` delegates alignment to the retained route. Direct
   quarter-turns and `ADVANCE` are independent investigation/recovery actions:
   an old waypoint must not shorten a chosen turn or forbid moving away from
   the goal. Existing sensor/body admission still applies.
5. Short missing/noisy readings do not erase useful earlier evidence or require
   another scan or model call. Unknown space may be explored; it is not a
   blanket no-go area. Actual failed motion returns control to the model with
   goal, route, map and verified pose retained. An ambiguous motor state,
   controller loss, or materially invalid localization stops physical movement
   until reconciled; it does not declare the mission finished. User Stop ends it.
6. BLAST has one episode-local motion heading. Short command/scan gyro deltas
   refine encoder motion; raw absolute gyro drift while waiting for the model
   must not rotate the estimated robot or the map. The same heading is used by
   waypoint alignment, motion feedback, scan projection and the displayed pose.
7. Observations update the map instead of replacing it wholesale. No-return
   and incomplete scans leave earlier measured obstacles in memory. Nearby
   repeat hits replace older evidence; a measured ray through it can clear it.
8. Keep three kinds of planner input separate: `observation` and
   `local_map_evidence` contain sensor facts; `obstacle_estimates` are the
   model's retained hypotheses; `execution` describes motion size, permission
   and the next steering action. A permitted motor does not certify empty
   space. Empty hypotheses do not certify an obstacle-free room. Current and
   historical planner headings use degrees in the same episode frame.
   BLAST retains the latest 128 ultrasonic rays with sensor-face origins,
   headings, distances/no-return state and settling quality, including readings
   after movement. EV3 retains its existing 128 positioned IR observations.
   Neither ray history turns a missing return into an observed object edge.
9. A stationary measurement is not a rotating survey. EV3's `OBSERVE` reuses
   the scan's settled-sample operation without wheel movement or loss of route,
   pose, or earlier rays. It is available to Qwen, not a mandatory step before
   every movement. Detected obstacles interrupt the relevant movement and
   return its measured outcome to the planner; uncertain/no-echo readings alone
   must not create an endless rescan loop.
10. EV3's existing `SCAN_FRONT_ARC` inspects -60..+60 degrees at 15-degree
    intervals, then targets the starting heading. It rotates the whole body,
    not an independently movable head. The startup 360-degree survey is
    unchanged. Actual encoder poses locate every ray, including partial turns;
    a missing ray stays unknown. Qwen chooses whether and where to inspect an
   obstacle. Reflection transitions constrain angular extent, not exact metric
   edges or whole-body clearance, and do not author a detour.
11. Keep compact model-owned obstacle rectangles. Numeric edges are estimates;
    absent edges remain unknown. Confidence or repeated guesses do not turn
    them into measured map geometry. The existing motion feedback and history
    report actual outcomes; do not add a second geometric interpretation of
    those outcomes to judge or rewrite the rectangles. Planner and map use
    the same retained hypotheses. Qwen owns their revision and the route.

The navigation runtimes support mobile navigation and surroundings perception.
Other physical requests must be reported as unsupported and must not be
converted into a directional navigation mission.
