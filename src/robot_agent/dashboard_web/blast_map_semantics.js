((global) => {
  "use strict";

  const CLASSIFICATION = "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER";
  const GEOMETRY_KIND = "PROVISIONAL_ULTRASONIC_ECHO_CLUSTER";
  const SOURCE_ID = "blast-settled-measured-planar-projection";
  const PROVENANCE = (
    "SETTLED_MEASURED_ULTRASONIC + PROVISIONAL_YAW_ONLY"
  );
  const ROUTE_KINDS = new Set([
    "LATERAL_CLEARANCE",
    "REACQUIRE_GOAL_HEADING",
    "PASS_BEYOND_TARGET",
    "MERGE_GOAL_AXIS",
    "RESUME_GOAL_HEADING",
  ]);
  const ROUTE_STATUSES = new Set(["ACTIVE", "COMPLETE", "INVALID"]);
  const WAYPOINT_STATUSES = new Set(["COMPLETED", "ACTIVE", "UPCOMING"]);
  const SCAN_MAX_ABSOLUTE_BEARING_MDEG = 180000;
  const ROUTE_FIELDS = Object.freeze([
    "schema", "read_only", "provisional", "route_id", "version",
    "status", "detour_side", "active_index", "waypoints",
  ]);
  const WAYPOINT_FIELDS = Object.freeze([
    "ordinal", "kind", "x_mm", "y_mm", "heading_mdeg", "fact_key",
    "status",
  ]);
  const FINAL_GOAL_FIELDS = Object.freeze([
    "kind", "navigation_enforced", "origin_x_mm", "origin_y_mm",
    "target_x_mm", "target_y_mm", "goal_radius_mm",
    "distance_to_goal_mm", "desired_heading_mdeg",
    "minimum_forward_progress_mm", "heading_tolerance_mdeg",
    "current_forward_progress_mm", "current_lateral_offset_mm",
    "remaining_forward_progress_mm",
  ]);
  const ADVISORY_WAYPOINT_FIELDS = Object.freeze([
    "x_mm", "y_mm", "purpose", "source", "read_only",
  ]);
  const SCAN_SIDES = new Set([
    "center", "left_near", "left_far", "right_near", "right_far",
    "left_1", "left_2", "left_3", "left_4",
    "right_1", "right_2", "right_3", "right_4",
  ]);

  function hasExactFields(value, fields) {
    const keys = Object.keys(value);
    return keys.length === fields.length && fields.every((field) => (
      Object.prototype.hasOwnProperty.call(value, field)
    ));
  }

  function normalizeRoute(value, helpers) {
    if (value === null || value === undefined) return null;
    const route = helpers.record(value);
    const routeId = helpers.identifier(route.route_id);
    const version = helpers.nonnegativeInteger(route.version);
    const activeIndex = helpers.nonnegativeInteger(route.active_index);
    if (
      !hasExactFields(route, ROUTE_FIELDS)
      || route.schema !== "robot-local-detour-route/v1"
      || route.read_only !== true || route.provisional !== true
      || routeId === null || version === null || version < 1
      || version > 1_000_000
      || !ROUTE_STATUSES.has(route.status)
      || !["LEFT_OF_GOAL", "RIGHT_OF_GOAL"].includes(route.detour_side)
      || activeIndex === null || !Array.isArray(route.waypoints)
      || route.waypoints.length < 1 || route.waypoints.length > 5
      || activeIndex > route.waypoints.length
    ) return undefined;
    const waypoints = route.waypoints.map((value, index) => {
      const waypoint = helpers.record(value);
      const pose = helpers.normalizeTracePose(waypoint);
      const factKey = waypoint.fact_key === null
        ? null : helpers.identifier(waypoint.fact_key);
      if (
        !hasExactFields(waypoint, WAYPOINT_FIELDS)
        || waypoint.ordinal !== index || !ROUTE_KINDS.has(waypoint.kind)
        || pose === null || factKey === null && waypoint.fact_key !== null
        || !WAYPOINT_STATUSES.has(waypoint.status)
      ) return null;
      let expected = "UPCOMING";
      if (route.status === "COMPLETE" || index < activeIndex) {
        expected = "COMPLETED";
      } else if (route.status === "ACTIVE" && index === activeIndex) {
        expected = "ACTIVE";
      }
      if (waypoint.status !== expected) return null;
      return Object.freeze({
        ordinal: index, kind: waypoint.kind,
        xMm: pose.xMm, yMm: pose.yMm, headingMdeg: pose.headingMdeg,
        factKey, status: waypoint.status,
      });
    });
    if (
      waypoints.some((item) => item === null)
      || route.status === "ACTIVE" && activeIndex >= waypoints.length
      || route.status === "COMPLETE" && activeIndex !== waypoints.length
    ) return undefined;
    return Object.freeze({
      schema: "robot-local-detour-route/v1", readOnly: true,
      provisional: true, routeId, version, status: route.status,
      detourSide: route.detour_side, activeIndex,
      waypoints: Object.freeze(waypoints),
    });
  }

  function normalizeFinalGoal(value, geometryToleranceMm, helpers) {
    const goal = helpers.record(value);
    const originX = helpers.coordinate(goal.origin_x_mm);
    const originY = helpers.coordinate(goal.origin_y_mm);
    const targetX = helpers.coordinate(goal.target_x_mm);
    const targetY = helpers.coordinate(goal.target_y_mm);
    const desiredHeadingMdeg = helpers.heading(goal.desired_heading_mdeg);
    const minimumForwardProgressMm = helpers.positive(
      goal.minimum_forward_progress_mm,
    );
    const headingToleranceMdeg = helpers.positive(
      goal.heading_tolerance_mdeg,
    );
    const currentForwardProgressMm = helpers.coordinate(
      goal.current_forward_progress_mm,
    );
    const currentLateralOffsetMm = helpers.coordinate(
      goal.current_lateral_offset_mm,
    );
    const remainingForwardProgressMm = helpers.nonnegativeInteger(
      goal.remaining_forward_progress_mm,
    );
    const distanceToGoalMm = helpers.nonnegativeInteger(
      goal.distance_to_goal_mm,
    );
    const goalRadiusMm = helpers.nonnegativeInteger(goal.goal_radius_mm);
    if (
      !hasExactFields(goal, FINAL_GOAL_FIELDS)
      || goal.kind !== "DIRECTIONAL_HEADING"
      || typeof goal.navigation_enforced !== "boolean"
      || [originX, originY, targetX, targetY, desiredHeadingMdeg,
        minimumForwardProgressMm, headingToleranceMdeg,
        currentForwardProgressMm, currentLateralOffsetMm,
        remainingForwardProgressMm, distanceToGoalMm,
        goalRadiusMm].some((item) => item === null)
      || minimumForwardProgressMm > helpers.maxCoordinateMm
      || headingToleranceMdeg > 180000
      || goalRadiusMm < 45 || goalRadiusMm > 500
      || distanceToGoalMm > 2 * helpers.maxCoordinateMm
    ) return null;
    const heading = desiredHeadingMdeg / 1000 * Math.PI / 180;
    const expectedX = originX + minimumForwardProgressMm * Math.cos(heading);
    const expectedY = originY + minimumForwardProgressMm * Math.sin(heading);
    const expectedRemaining = Math.max(
      0, minimumForwardProgressMm - currentForwardProgressMm,
    );
    const expectedDistance = Math.round(Math.hypot(
      minimumForwardProgressMm - currentForwardProgressMm,
      currentLateralOffsetMm,
    ));
    if (
      Math.abs(targetX - expectedX) > geometryToleranceMm
      || Math.abs(targetY - expectedY) > geometryToleranceMm
      || Math.abs(remainingForwardProgressMm - expectedRemaining) > 1
      || Math.abs(distanceToGoalMm - expectedDistance) > 1
    ) return null;
    return Object.freeze({
      kind: goal.kind, navigationEnforced: goal.navigation_enforced,
      originX, originY, targetX, targetY, goalRadiusMm, distanceToGoalMm,
      desiredHeadingMdeg, minimumForwardProgressMm, headingToleranceMdeg,
      currentForwardProgressMm, currentLateralOffsetMm,
      remainingForwardProgressMm,
    });
  }

  function normalizeAdvisoryWaypoint(value, helpers) {
    if (value === null || value === undefined) return null;
    const waypoint = helpers.record(value);
    const xMm = helpers.coordinate(waypoint.x_mm);
    const yMm = helpers.coordinate(waypoint.y_mm);
    const purpose = helpers.strictText(waypoint.purpose, 120);
    if (
      !hasExactFields(waypoint, ADVISORY_WAYPOINT_FIELDS)
      || xMm === null || yMm === null || purpose === null
      || waypoint.source !== "GEMMA_MODEL" || waypoint.read_only !== true
    ) return undefined;
    return Object.freeze({
      xMm, yMm, purpose, source: "GEMMA_MODEL", readOnly: true,
    });
  }

  function normalizeObstacle(value, nowUnixMs, helpers) {
    const item = helpers.record(value);
    const hypothesisId = helpers.identifier(item.hypothesis_id);
    const xMm = helpers.coordinate(item.x_mm);
    const yMm = helpers.coordinate(item.y_mm);
    const radius = helpers.positive(item.support_radius_mm);
    const evidenceCount = helpers.nonnegativeInteger(item.evidence_count);
    const confidence = helpers.finite(item.confidence_milli);
    const observed = helpers.finite(item.observed_at_unix_ms);
    const scanIds = Array.isArray(item.source_scan_ids)
      ? item.source_scan_ids.map(helpers.identifier) : [];
    const points = Array.isArray(item.support_points)
      ? item.support_points.map((value) => {
        const point = helpers.record(value);
        const pointX = helpers.coordinate(point.x_mm);
        const pointY = helpers.coordinate(point.y_mm);
        const range = helpers.finite(point.measured_range_mm);
        const bearing = helpers.finite(point.relative_bearing_mdeg);
        if (
          !SCAN_SIDES.has(point.side) || pointX === null || pointY === null
          || range === null || range < 0
          || range >= helpers.noValidDistanceMm
          || bearing === null
          || bearing < -SCAN_MAX_ABSOLUTE_BEARING_MDEG
          || bearing > SCAN_MAX_ABSOLUTE_BEARING_MDEG
          || point.side === "center" && bearing !== 0
          || point.side.startsWith("left_") && bearing <= 0
          || point.side.startsWith("right_") && bearing >= 0
        ) return null;
        return Object.freeze({
          side: point.side, xMm: pointX, yMm: pointY,
          measuredRangeMm: range, relativeBearingMdeg: bearing,
        });
      }) : [];
    if (
      item.classification !== CLASSIFICATION || item.label !== CLASSIFICATION
      || item.geometry_kind !== GEOMETRY_KIND || item.provisional !== true
      || item.read_only !== true || item.settled_measured_only !== true
      || item.quality !== "PROVISIONAL_YAW_ONLY" || hypothesisId === null
      || xMm === null || yMm === null || radius === null
      || radius > helpers.maxSupportRadiusMm || evidenceCount === null
      || evidenceCount < 1 || evidenceCount > helpers.maxPoints
      || points.length !== evidenceCount || points.some((point) => point === null)
      || confidence === null || confidence < 1 || confidence > 400
      || helpers.identifier(item.source_id) !== SOURCE_ID
      || item.provenance !== PROVENANCE || scanIds.length < 1
      || scanIds.length > helpers.maxViews || scanIds.some((id) => id === null)
      || new Set(scanIds).size !== scanIds.length || observed === null
      || observed < 0 || observed > nowUnixMs
    ) return null;
    const meanX = points.reduce((sum, point) => sum + point.xMm, 0) / points.length;
    const meanY = points.reduce((sum, point) => sum + point.yMm, 0) / points.length;
    const support = Math.max(...points.map((point) => (
      Math.hypot(point.xMm - xMm, point.yMm - yMm)
    )));
    const meanBearing = points.reduce(
      (sum, point) => sum + point.relativeBearingMdeg, 0,
    ) / points.length;
    const relation = meanBearing > 7500
      ? "LEFT_OF_SCAN" : meanBearing < -7500
        ? "RIGHT_OF_SCAN" : "FRONT_OF_SCAN";
    const bearing = relation.replace("_OF_SCAN", "");
    if (
      Math.abs(xMm - meanX) > 1.5 || Math.abs(yMm - meanY) > 1.5
      || radius < support || radius > support + 100
      || item.relation !== relation || item.bearing !== bearing
    ) return null;
    return Object.freeze({
      hypothesisId, classification: CLASSIFICATION, label: CLASSIFICATION,
      xMm, yMm, anchorPose: null, geometryKind: GEOMETRY_KIND,
      supportRadiusMm: radius, supportPoints: Object.freeze(points),
      sourceScanIds: Object.freeze(scanIds), bearing, relation, evidenceCount,
      settledMeasuredOnly: true, readOnly: true, provisional: true,
      confidenceMilli: confidence, sourceId: SOURCE_ID,
      provenance: PROVENANCE, observedAtUnixMs: observed,
      ageMs: nowUnixMs - observed,
    });
  }

  function appendRoutePoints(route, addPoint) {
    if (!route) return;
    route.waypoints.forEach((item) => addPoint(item.xMm, item.yMm));
  }

  function appendGoalPoints(goal, addPoint) {
    addPoint(goal.originX, goal.originY);
    addPoint(goal.targetX - goal.goalRadiusMm, goal.targetY);
    addPoint(goal.targetX + goal.goalRadiusMm, goal.targetY);
    addPoint(goal.targetX, goal.targetY - goal.goalRadiusMm);
    addPoint(goal.targetX, goal.targetY + goal.goalRadiusMm);
  }

  function appendAdvisoryWaypointPoint(waypoint, addPoint) {
    if (waypoint) addPoint(waypoint.xMm, waypoint.yMm);
  }

  function appendObstaclePoints(hypothesis, addPoint) {
    if (hypothesis?.classification !== CLASSIFICATION) return;
    const { xMm, yMm, supportRadiusMm: radius } = hypothesis;
    addPoint(xMm - radius, yMm); addPoint(xMm + radius, yMm);
    addPoint(xMm, yMm - radius); addPoint(xMm, yMm + radius);
    hypothesis.supportPoints.forEach((point) => addPoint(point.xMm, point.yMm));
  }

  function appendSharedObstaclePoints(robots, addPoint) {
    robots.forEach((robot) => robot.objectHypotheses.forEach((item) => (
      appendObstaclePoints(item, addPoint)
    )));
  }

  function objectEntries(map, sharedSchema) {
    if (map.schema !== sharedSchema) {
      return map.objectHypotheses.map((hypothesis) => ({
        hypothesis,
        sourceRobotId: null,
      }));
    }
    return map.robots.flatMap((robot) => robot.objectHypotheses.map(
      (hypothesis) => ({ hypothesis, sourceRobotId: robot.robotId }),
    ));
  }

  function renderRoute(layer, route, projection, ui) {
    if (!route) return;
    const group = ui.svg("g", {
      class: "map-local-detour-route", "data-route-id": route.routeId,
      "data-route-status": route.status, "data-detour-side": route.detourSide,
      "data-active-index": route.activeIndex, "data-provisional": "true",
    });
    ui.title(group, [
      ui.t("map.navigation_trace.route_title", { side: route.detourSide }),
      ui.t("map.navigation_trace.route_provisional"),
    ]);
    const projected = route.waypoints.map((item) => projection.point(item.xMm, item.yMm));
    if (projected.length > 1) group.appendChild(ui.svg("polyline", {
      points: projected.map((point) => `${point.x},${point.y}`).join(" "),
      class: "map-local-detour-route-line",
    }));
    route.waypoints.forEach((item, index) => {
      const point = projected[index];
      const node = ui.svg("g", {
        class: `map-local-detour-waypoint is-${item.status.toLowerCase()}`,
        "data-ordinal": item.ordinal, "data-kind": item.kind,
        "data-status": item.status, "data-fact-key": item.factKey || "",
      });
      const name = ui.t(`mission.route.waypoint.${item.kind}`);
      ui.title(node, [name,
        ui.t(`map.navigation_trace.route_waypoint_status.${item.status}`),
        ui.t("map.navigation_trace.route_waypoint_pose", {
          x: ui.format(item.xMm), y: ui.format(item.yMm),
          heading: ui.format(item.headingMdeg / 1000, { maximumFractionDigits: 1 }),
        }),
      ]);
      node.appendChild(ui.svg("circle", {
        cx: point.x, cy: point.y, r: item.status === "ACTIVE" ? 11 : 8,
        class: "map-local-detour-waypoint-marker",
      }));
      node.appendChild(ui.svg("path", {
        d: ui.heading(point, item.headingMdeg / 1000 * Math.PI / 180, 30),
        class: "map-local-detour-waypoint-heading",
      }));
      const label = ui.svg("text", {
        x: point.x + 13, y: point.y + (index % 2 === 0 ? -14 : 22),
        class: "map-navigation-trace-label map-local-detour-waypoint-label",
      });
      label.textContent = `${item.ordinal + 1}. ${name}`;
      node.appendChild(label); group.appendChild(node);
    });
    layer.appendChild(group);
  }

  function renderGoal(layer, goal, projection, ui) {
    const origin = projection.point(goal.originX, goal.originY);
    const target = projection.point(goal.targetX, goal.targetY);
    const edge = projection.point(
      goal.targetX + goal.goalRadiusMm, goal.targetY,
    );
    const radius = Math.max(8, Math.abs(edge.x - target.x));
    const enforced = goal.navigationEnforced === true;
    const group = ui.svg("g", {
      class: "map-final-goal", "data-kind": goal.kind,
      "data-navigation-enforced": String(enforced),
      "data-goal-radius-mm": goal.goalRadiusMm,
      "data-distance-to-goal-mm": goal.distanceToGoalMm,
      "data-current-lateral-offset-mm": goal.currentLateralOffsetMm,
      "data-minimum-forward-progress-mm": goal.minimumForwardProgressMm,
      "data-current-forward-progress-mm": goal.currentForwardProgressMm,
      "data-remaining-forward-progress-mm": goal.remainingForwardProgressMm,
      "data-desired-heading-mdeg": goal.desiredHeadingMdeg,
      "data-heading-tolerance-mdeg": goal.headingToleranceMdeg,
    });
    ui.title(group, [
      ui.t(enforced
        ? "map.navigation_trace.final_goal_enforced_title"
        : "map.navigation_trace.final_goal_title"),
      ui.t("map.navigation_trace.goal_distance", {
        distance: ui.format(goal.distanceToGoalMm),
        radius: ui.format(goal.goalRadiusMm),
        lateral: ui.format(goal.currentLateralOffsetMm),
      }),
      ui.t("map.navigation_trace.goal_progress", {
        current: ui.format(goal.currentForwardProgressMm),
        target: ui.format(goal.minimumForwardProgressMm),
        remaining: ui.format(goal.remainingForwardProgressMm),
      }),
      ui.t("map.navigation_trace.goal_heading", {
        heading: ui.format(goal.desiredHeadingMdeg / 1000, {
          maximumFractionDigits: 1,
        }),
        tolerance: ui.format(goal.headingToleranceMdeg / 1000, {
          maximumFractionDigits: 1,
        }),
      }),
    ]);
    group.appendChild(ui.svg("line", {
      x1: origin.x, y1: origin.y, x2: target.x, y2: target.y,
      class: "map-final-goal-line",
    }));
    group.appendChild(ui.svg("circle", {
      cx: target.x, cy: target.y, r: radius, class: "map-final-goal-zone",
    }));
    group.appendChild(ui.svg("circle", {
      cx: target.x, cy: target.y, r: 11, class: "map-final-goal-target",
    }));
    group.appendChild(ui.svg("path", {
      d: ui.heading(
        target, goal.desiredHeadingMdeg / 1000 * Math.PI / 180, 48,
      ),
      class: "map-final-goal-heading",
    }));
    const label = ui.svg("text", {
      x: target.x + radius + 8, y: target.y - 10,
      class: "map-navigation-trace-label map-final-goal-label",
    });
    label.textContent = ui.t(enforced
      ? "map.navigation_trace.final_goal_enforced_label"
      : "map.navigation_trace.final_goal_label", {
      distance: ui.format(goal.distanceToGoalMm),
    });
    group.appendChild(label); layer.appendChild(group);
  }

  function renderAdvisoryWaypoint(layer, waypoint, robotPose, projection, ui) {
    if (!waypoint) return;
    const point = projection.point(waypoint.xMm, waypoint.yMm);
    const group = ui.svg("g", {
      class: "map-advisory-waypoint", "data-source": waypoint.source,
      "data-read-only": "true", "data-purpose": waypoint.purpose,
    });
    ui.title(group, [
      ui.t("map.navigation_trace.advisory_waypoint_title"),
      waypoint.purpose,
      ui.t("map.navigation_trace.advisory_waypoint_pose", {
        x: ui.format(waypoint.xMm), y: ui.format(waypoint.yMm),
      }),
      ui.t("map.navigation_trace.advisory_waypoint_read_only"),
    ]);
    if (robotPose) {
      const robot = projection.point(robotPose.xMm, robotPose.yMm);
      group.appendChild(ui.svg("line", {
        x1: robot.x, y1: robot.y, x2: point.x, y2: point.y,
        class: "map-advisory-waypoint-line",
      }));
    }
    group.appendChild(ui.svg("rect", {
      x: point.x - 8, y: point.y - 8, width: 16, height: 16,
      transform: `rotate(45 ${point.x} ${point.y})`,
      class: "map-advisory-waypoint-marker",
    }));
    const label = ui.svg("text", {
      x: point.x + 15, y: point.y - 15,
      class: "map-navigation-trace-label map-advisory-waypoint-label",
    });
    label.textContent = ui.t("map.navigation_trace.advisory_waypoint_label");
    group.appendChild(label); layer.appendChild(group);
  }

  function renderObstacles(layer, hypotheses, projection, ui) {
    hypotheses.filter((item) => item.classification === CLASSIFICATION)
      .forEach((item) => {
        const center = projection.point(item.xMm, item.yMm);
        const edge = projection.point(item.xMm + item.supportRadiusMm, item.yMm);
        const radius = Math.max(9, Math.abs(edge.x - center.x));
        const group = ui.svg("g", {
          class: "map-provisional-ultrasonic-obstacle",
          "data-hypothesis-id": item.hypothesisId,
          "data-classification": item.classification,
          "data-geometry-kind": item.geometryKind, "data-relation": item.relation,
          "data-evidence-count": item.evidenceCount,
          "data-support-radius-mm": item.supportRadiusMm,
          "data-source-scan-ids": item.sourceScanIds.join(" "),
          "data-provisional": "true", "data-read-only": "true",
          "data-settled-measured-only": "true",
        });
        ui.title(group, [ui.t("map.objects.ultrasonic_obstacle_cluster"),
          ui.t("map.objects.provisional_inference"),
          ui.t("map.objects.ultrasonic_support", {
            count: ui.format(item.evidenceCount), radius: ui.format(item.supportRadiusMm),
          }),
          ui.t(`map.objects.ultrasonic_relation.${item.relation.toLowerCase()}`),
          ui.t("map.objects.source_scans", { scans: item.sourceScanIds.join(", ") }),
          ...ui.tooltip(item),
        ]);
        group.appendChild(ui.svg("circle", {
          cx: center.x, cy: center.y, r: radius,
          class: "map-provisional-ultrasonic-obstacle-envelope",
        }));
        group.appendChild(ui.svg("rect", {
          x: center.x - 6, y: center.y - 6, width: 12, height: 12,
          transform: `rotate(45 ${center.x} ${center.y})`,
          class: "map-provisional-ultrasonic-obstacle-center",
        }));
        const label = ui.svg("text", {
          x: center.x + radius + 8, y: center.y - 8,
          class: "map-object-label map-provisional-ultrasonic-obstacle-label",
        });
        label.textContent = ui.t("map.objects.ultrasonic_obstacle_short");
        group.appendChild(label); layer.appendChild(group);
      });
  }

  function renderSharedObstacles(layer, robots, projection, ui) {
    robots.forEach((robot) => {
      if (robot.objectHypotheses.length === 0) return;
      const group = ui.svg("g", {
        class: "map-shared-provisional-obstacles",
        "data-robot-id": robot.robotId,
        "data-controller-instance-id": robot.controllerInstanceId,
      });
      renderObstacles(group, robot.objectHypotheses, projection, ui);
      layer.appendChild(group);
    });
  }

  global.RobotBlastMapSemantics = Object.freeze({
    CLASSIFICATION, GEOMETRY_KIND, SCAN_SIDES,
    appendAdvisoryWaypointPoint, appendGoalPoints, appendObstaclePoints,
    appendRoutePoints, appendSharedObstaclePoints, normalizeAdvisoryWaypoint,
    normalizeFinalGoal, normalizeObstacle, normalizeRoute, objectEntries,
    renderAdvisoryWaypoint, renderGoal, renderObstacles, renderRoute,
    renderSharedObstacles,
  });
})(typeof window === "undefined" ? globalThis : window);
