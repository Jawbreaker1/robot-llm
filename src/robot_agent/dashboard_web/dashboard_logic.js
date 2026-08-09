((global) => {
  "use strict";

  const TERMINAL_TURN_STATES = new Set([
    "answered",
    "clarification_required",
    "failed",
  ]);
  const TURN_POLL_POLICY = Object.freeze({
    unknownAfterFailures: 8,
    baseDelayMs: 800,
    maxDelayMs: 5000,
  });
  const SPATIAL_MAP_SCHEMA = "robot-spatial-map/v1";
  const SHARED_SPATIAL_MAP_SCHEMA = "robot-spatial-map/v2";
  const SHARED_FIXED_START = "SHARED_FIXED_START";
  const SHARED_SNAPSHOT_SEMANTICS = "LATEST_AVAILABLE_NOT_ATOMIC";
  const MAX_SHARED_ROBOTS = 16;
  const MAX_SPATIAL_CELLS = 10000;
  const MAX_SENSOR_RAYS = 256;
  const MAX_OBJECT_HYPOTHESES = 256;
  const MAX_QUALITATIVE_OBSERVATIONS = 1024;
  const MAX_POSE_HISTORY = 2048;
  const MAX_SCAN_EVIDENCE = 64;
  const MAX_SCAN_RAYS_PER_EVIDENCE = 16;
  const NAVIGATION_TRACE_SCHEMA = "robot-navigation-trace/v1";
  const BLAST_SCAN_PROJECTION_SCHEMA = (
    "blast-planar-scan-projection/v1"
  );
  const MAX_NAVIGATION_SCAN_VIEWS = 16;
  const MAX_NAVIGATION_SCAN_POINTS = 5;
  const MAX_LOCAL_COORDINATE_MM = 1_000_000;
  const MAX_MEASURED_RANGE_MM = 10_000;
  const LOCAL_DETOUR_WAYPOINT_KINDS = new Set([
    "LATERAL_CLEARANCE",
    "REACQUIRE_GOAL_HEADING",
    "PASS_BEYOND_TARGET",
    "MERGE_GOAL_AXIS",
    "RESUME_GOAL_HEADING",
  ]);
  const SESSION_REJECTED_CODE = "session_token_rejected";
  const BODY_RELATIVE_BEARING_CONVENTION = (
    "POSITIVE_LEFT_NEGATIVE_RIGHT"
  );
  const ANGULAR_NONMETRIC_IR_SCAN = "ANGULAR_NONMETRIC_IR_SCAN";

  function record(value) {
    return value && typeof value === "object" && !Array.isArray(value)
      ? value
      : {};
  }

  function finite(value) {
    return Number.isFinite(value) ? value : null;
  }

  function positive(value) {
    return Number.isFinite(value) && value > 0 ? value : null;
  }

  function text(value) {
    return typeof value === "string" && value.length > 0 ? value : null;
  }

  function identifier(value) {
    return strictText(value, 128);
  }

  function strictText(value, maximumLength) {
    return (
      typeof value === "string"
      && value.length > 0
      && value.length <= maximumLength
      && value === value.trim()
      && !/[\x00-\x1f]/.test(value)
    ) ? value : null;
  }

  function nonnegativeInteger(value) {
    return Number.isSafeInteger(value) && value >= 0 ? value : null;
  }

  function provenance(value) {
    const label = text(value);
    if (label === "PROVISIONAL_IR") {
      return "PROVISIONAL IR";
    }
    return label;
  }

  function ageAt(observedAtUnixMs, nowUnixMs) {
    const observed = finite(observedAtUnixMs);
    if (observed === null || observed > nowUnixMs) {
      return null;
    }
    return nowUnixMs - observed;
  }

  function observationAge(value, nowUnixMs) {
    const observation = record(value);
    const liveAge = ageAt(
      observation.observed_at_unix_ms,
      nowUnixMs,
    );
    if (liveAge !== null) {
      return liveAge;
    }
    const explicitAge = finite(observation.age_ms);
    if (explicitAge !== null && explicitAge >= 0) {
      return explicitAge;
    }
    return null;
  }

  function localCoordinate(value) {
    const coordinate = finite(value);
    return (
      coordinate !== null
      && Math.abs(coordinate) <= MAX_LOCAL_COORDINATE_MM
    ) ? coordinate : null;
  }

  function localHeading(value) {
    const heading = finite(value);
    return (
      heading !== null
      && heading >= -180000
      && heading <= 179999
    ) ? heading : null;
  }

  function normalizeTracePose(value) {
    const pose = record(value);
    const xMm = localCoordinate(pose.x_mm);
    const yMm = localCoordinate(pose.y_mm);
    const headingMdeg = localHeading(pose.heading_mdeg);
    if (xMm === null || yMm === null || headingMdeg === null) {
      return null;
    }
    return Object.freeze({ xMm, yMm, headingMdeg });
  }

  function normalizeFinalGoal(value) {
    const goal = record(value);
    const originX = localCoordinate(goal.origin_x_mm);
    const originY = localCoordinate(goal.origin_y_mm);
    const targetX = localCoordinate(goal.target_x_mm);
    const targetY = localCoordinate(goal.target_y_mm);
    const desiredHeadingMdeg = localHeading(
      goal.desired_heading_mdeg,
    );
    const minimumForwardProgressMm = positive(
      goal.minimum_forward_progress_mm,
    );
    const headingToleranceMdeg = positive(
      goal.heading_tolerance_mdeg,
    );
    const currentForwardProgressMm = finite(
      goal.current_forward_progress_mm,
    );
    const remainingForwardProgressMm = finite(
      goal.remaining_forward_progress_mm,
    );
    if (
      goal.kind !== "DIRECTIONAL_HEADING"
      || typeof goal.navigation_enforced !== "boolean"
      || originX === null
      || originY === null
      || targetX === null
      || targetY === null
      || desiredHeadingMdeg === null
      || minimumForwardProgressMm === null
      || minimumForwardProgressMm > MAX_LOCAL_COORDINATE_MM
      || headingToleranceMdeg === null
      || headingToleranceMdeg > 180000
      || currentForwardProgressMm === null
      || Math.abs(currentForwardProgressMm) > MAX_LOCAL_COORDINATE_MM
      || remainingForwardProgressMm === null
      || remainingForwardProgressMm < 0
      || remainingForwardProgressMm > MAX_LOCAL_COORDINATE_MM
    ) {
      return null;
    }
    const headingRadians = desiredHeadingMdeg / 1000 * Math.PI / 180;
    const expectedTargetX = originX
      + minimumForwardProgressMm * Math.cos(headingRadians);
    const expectedTargetY = originY
      + minimumForwardProgressMm * Math.sin(headingRadians);
    const expectedRemaining = Math.max(
      0,
      minimumForwardProgressMm - currentForwardProgressMm,
    );
    if (
      Math.abs(targetX - expectedTargetX) > 1.5
      || Math.abs(targetY - expectedTargetY) > 1.5
      || Math.abs(remainingForwardProgressMm - expectedRemaining) > 1
    ) {
      return null;
    }
    return Object.freeze({
      kind: "DIRECTIONAL_HEADING",
      navigationEnforced: goal.navigation_enforced,
      originX,
      originY,
      targetX,
      targetY,
      desiredHeadingMdeg,
      minimumForwardProgressMm,
      headingToleranceMdeg,
      currentForwardProgressMm,
      remainingForwardProgressMm,
    });
  }

  function normalizeImuHeading(value, nowUnixMs) {
    if (value === null) {
      return null;
    }
    const heading = record(value);
    const headingMdeg = localHeading(heading.heading_mdeg);
    const observedAtUnixMs = finite(heading.observed_at_unix_ms);
    if (
      heading.reference !== "EPISODE_START"
      || headingMdeg === null
      || observedAtUnixMs === null
      || observedAtUnixMs < 0
      || observedAtUnixMs > nowUnixMs
    ) {
      return undefined;
    }
    return Object.freeze({
      headingMdeg,
      reference: "EPISODE_START",
      observedAtUnixMs,
      ageMs: nowUnixMs - observedAtUnixMs,
    });
  }

  function normalizePlannedLeg(value) {
    if (value === null) {
      return null;
    }
    const leg = record(value);
    const bindPose = normalizeTracePose(leg.bind_pose);
    const waypoint = normalizeTracePose(leg.waypoint);
    const sideSearch = (
      leg.kind === "SIDE_SEARCH"
      && leg.scope === "SEARCH_POSITION_ONLY"
      && leg.route_eligible === false
    );
    const localDetourRoute = (
      LOCAL_DETOUR_WAYPOINT_KINDS.has(leg.kind)
      && leg.scope === "LOCAL_DETOUR_ROUTE"
      && leg.route_eligible === true
    );
    if (
      (!sideSearch && !localDetourRoute)
      || leg.clearance_proven !== false
      || leg.passage_proven !== false
      || !["LEFT", "RIGHT"].includes(leg.selected_side)
      || bindPose === null
      || waypoint === null
    ) {
      return undefined;
    }
    return Object.freeze({
      kind: leg.kind,
      scope: leg.scope,
      clearanceProven: false,
      passageProven: false,
      routeEligible: leg.route_eligible,
      selectedSide: leg.selected_side,
      bindPose,
      waypoint,
    });
  }

  function normalizePlanarScanPoint(value) {
    const point = record(value);
    const measuredRangeMm = positive(point.measured_range_mm);
    const relativeBearingMdeg = finite(point.relative_bearing_mdeg);
    const sensorOriginX = localCoordinate(point.sensor_origin_x_mm);
    const sensorOriginY = localCoordinate(point.sensor_origin_y_mm);
    const beamHeadingMdeg = localHeading(point.beam_heading_mdeg);
    const nominalEchoX = localCoordinate(point.nominal_echo_x_mm);
    const nominalEchoY = localCoordinate(point.nominal_echo_y_mm);
    const allowedSides = new Set([
      "center",
      "left_near",
      "left_far",
      "right_near",
      "right_far",
    ]);
    if (
      !allowedSides.has(point.side)
      || measuredRangeMm === null
      || measuredRangeMm > MAX_MEASURED_RANGE_MM
      || relativeBearingMdeg === null
      || relativeBearingMdeg < -90000
      || relativeBearingMdeg > 90000
      || sensorOriginX === null
      || sensorOriginY === null
      || beamHeadingMdeg === null
      || nominalEchoX === null
      || nominalEchoY === null
      || (point.side === "center" && relativeBearingMdeg !== 0)
      || (point.side.startsWith("left_") && relativeBearingMdeg <= 0)
      || (point.side.startsWith("right_") && relativeBearingMdeg >= 0)
    ) {
      return null;
    }
    const headingRadians = beamHeadingMdeg / 1000 * Math.PI / 180;
    const expectedEchoX = sensorOriginX
      + measuredRangeMm * Math.cos(headingRadians);
    const expectedEchoY = sensorOriginY
      + measuredRangeMm * Math.sin(headingRadians);
    if (
      Math.abs(nominalEchoX - expectedEchoX) > 1.5
      || Math.abs(nominalEchoY - expectedEchoY) > 1.5
    ) {
      return null;
    }
    return Object.freeze({
      side: point.side,
      measuredRangeMm,
      relativeBearingMdeg,
      sensorOriginX,
      sensorOriginY,
      beamHeadingMdeg,
      nominalEchoX,
      nominalEchoY,
    });
  }

  function normalizePlanarScanView(value, nowUnixMs) {
    const view = record(value);
    const projection = record(view.projection);
    const scanPose = normalizeTracePose(view.scan_pose);
    const scanId = text(view.scan_id);
    const observedAtUnixMs = finite(view.observed_at_unix_ms);
    if (
      scanId === null
      || scanId.length > 128
      || observedAtUnixMs === null
      || observedAtUnixMs < 0
      || observedAtUnixMs > nowUnixMs
      || scanPose === null
      || projection.schema !== BLAST_SCAN_PROJECTION_SCHEMA
      || projection.frame !== "EPISODE_LOCAL_ODOMETRY"
      || projection.quality !== "PROVISIONAL_YAW_ONLY"
      || projection.vertical_pitch_compensated !== false
      || projection.ultrasonic_beam_width_modeled !== false
      || projection.scan_turn_translation_compensated !== false
      || !Array.isArray(projection.points)
      || projection.points.length > MAX_NAVIGATION_SCAN_POINTS
    ) {
      return null;
    }
    const points = projection.points.map(normalizePlanarScanPoint);
    if (
      points.some((point) => point === null)
      || new Set(points.map((point) => point.side)).size !== points.length
    ) {
      return null;
    }
    return Object.freeze({
      scanId,
      observedAtUnixMs,
      ageMs: nowUnixMs - observedAtUnixMs,
      scanPose,
      quality: "PROVISIONAL_YAW_ONLY",
      verticalPitchCompensated: false,
      ultrasonicBeamWidthModeled: false,
      scanTurnTranslationCompensated: false,
      points: Object.freeze(points),
    });
  }

  function normalizeNavigationTrace(value, frameId, nowUnixMs) {
    if (value === null || value === undefined) {
      return null;
    }
    const trace = record(value);
    const finalGoal = normalizeFinalGoal(trace.final_goal);
    const imuHeading = normalizeImuHeading(trace.imu_heading, nowUnixMs);
    const plannedLeg = normalizePlannedLeg(trace.planned_leg);
    if (
      trace.schema !== NAVIGATION_TRACE_SCHEMA
      || trace.read_only !== true
      || text(trace.frame_id) === null
      || frameId === null
      || trace.frame_id !== frameId
      || trace.provenance
        !== "PROVISIONAL_ENCODER_ODOMETRY + PROVISIONAL_YAW_ONLY"
      || finalGoal === null
      || imuHeading === undefined
      || plannedLeg === undefined
      || (
        plannedLeg?.scope === "SEARCH_POSITION_ONLY"
        && finalGoal.navigationEnforced !== false
      )
      || (
        plannedLeg?.scope === "LOCAL_DETOUR_ROUTE"
        && finalGoal.navigationEnforced !== true
      )
      || !Array.isArray(trace.planar_scan_views)
      || trace.planar_scan_views.length > MAX_NAVIGATION_SCAN_VIEWS
    ) {
      return null;
    }
    const planarScanViews = trace.planar_scan_views.map((scan) => (
      normalizePlanarScanView(scan, nowUnixMs)
    ));
    if (
      planarScanViews.some((scan) => scan === null)
      || new Set(planarScanViews.map((scan) => scan.scanId)).size
        !== planarScanViews.length
    ) {
      return null;
    }
    return Object.freeze({
      schema: NAVIGATION_TRACE_SCHEMA,
      readOnly: true,
      frameId,
      provenance: trace.provenance,
      finalGoal,
      imuHeading,
      plannedLeg,
      planarScanViews: Object.freeze(planarScanViews),
    });
  }

  function normalizeBounds(value) {
    const bounds = record(value);
    const minX = finite(bounds.min_x_mm);
    const minY = finite(bounds.min_y_mm);
    const maxX = finite(bounds.max_x_mm);
    const maxY = finite(bounds.max_y_mm);
    if (
      minX === null
      || minY === null
      || maxX === null
      || maxY === null
      || maxX <= minX
      || maxY <= minY
    ) {
      return null;
    }
    return Object.freeze({
      minX,
      minY,
      maxX,
      maxY,
    });
  }

  function normalizeCollisionGeometry(value) {
    const geometry = record(value);
    if (
      geometry.reference_point !== "DIFFERENTIAL_DRIVE_ORIGIN"
    ) {
      return null;
    }
    if (geometry.geometry === "SYMMETRIC_CIRCLE") {
      const radiusMm = positive(geometry.radius_mm);
      if (radiusMm === null) {
        return null;
      }
      return Object.freeze({
        geometry: "SYMMETRIC_CIRCLE",
        referencePoint: "DIFFERENTIAL_DRIVE_ORIGIN",
        radiusMm,
      });
    }
    if (geometry.geometry !== "ASYMMETRIC_RECTANGLE") {
      return null;
    }
    const frontExtentMm = positive(geometry.front_extent_mm);
    const rearExtentMm = positive(geometry.rear_extent_mm);
    const leftExtentMm = positive(geometry.left_extent_mm);
    const rightExtentMm = positive(geometry.right_extent_mm);
    const clearanceMarginMm = finite(geometry.clearance_margin_mm);
    if (
      frontExtentMm === null
      || rearExtentMm === null
      || leftExtentMm === null
      || rightExtentMm === null
      || clearanceMarginMm === null
      || clearanceMarginMm < 0
    ) {
      return null;
    }
    return Object.freeze({
      geometry: "ASYMMETRIC_RECTANGLE",
      referencePoint: "DIFFERENTIAL_DRIVE_ORIGIN",
      frontExtentMm,
      rearExtentMm,
      leftExtentMm,
      rightExtentMm,
      clearanceMarginMm,
      calibrationStatus: text(geometry.calibration_status),
      calibrationEvidence: text(geometry.calibration_evidence),
    });
  }

  function normalizeScanEvidence(value, frameId, nowUnixMs) {
    const evidence = record(value);
    const hypothesisAnchor = record(evidence.hypothesis_anchor_pose);
    const hypothesisAnchorX = finite(hypothesisAnchor.x_mm);
    const hypothesisAnchorY = finite(hypothesisAnchor.y_mm);
    const hypothesisAnchorHeading = finite(
      hypothesisAnchor.heading_mdeg,
    );
    const scanPoseValue = evidence.scan_pose === null
      ? null
      : record(evidence.scan_pose);
    const scanX = scanPoseValue === null
      ? null
      : finite(scanPoseValue.x_mm);
    const scanY = scanPoseValue === null
      ? null
      : finite(scanPoseValue.y_mm);
    const scanHeading = scanPoseValue === null
      ? null
      : finite(scanPoseValue.heading_mdeg);
    const basedOnMapVersion = evidence.based_on_map_version === null
      ? null
      : finite(evidence.based_on_map_version);
    const hasScanPose = (
      scanX !== null
      && scanY !== null
      && scanHeading !== null
      && basedOnMapVersion !== null
    );
    const completedAtUnixMs = finite(evidence.completed_at_unix_ms);
    if (
      evidence.read_only !== true
      || evidence.provisional !== true
      || evidence.geometry_kind !== ANGULAR_NONMETRIC_IR_SCAN
      || evidence.bearing_convention
        !== BODY_RELATIVE_BEARING_CONVENTION
      || text(evidence.target_hypothesis_id) === null
      || text(evidence.scan_id) === null
      || text(evidence.frame_id) === null
      || (
        frameId !== null
        && evidence.frame_id !== frameId
      )
      || hypothesisAnchorX === null
      || hypothesisAnchorY === null
      || hypothesisAnchorHeading === null
      || (
        scanPoseValue !== null
        && !hasScanPose
      )
      || (
        scanPoseValue === null
        && basedOnMapVersion !== null
      )
      || completedAtUnixMs === null
      || completedAtUnixMs > nowUnixMs
      || !["COMPLETED", "CANCELLED"].includes(evidence.status)
      || ![
        "NO_RAYS",
        "ALL_CLEAR",
        "ALL_BLOCKED",
        "MIXED",
      ].includes(evidence.observation_pattern)
      || ![
        "NO_ARC",
        "CENTER_ONLY",
        "NEGATIVE_ARC_ONLY",
        "POSITIVE_ARC_ONLY",
        "BILATERAL_ARC",
      ].includes(evidence.arc_coverage)
      || ![
        "NO_BOUNDARIES",
        "POSITIVE_BOUNDARY_ONLY",
        "NEGATIVE_BOUNDARY_ONLY",
        "BILATERAL_BOUNDARIES",
      ].includes(evidence.boundary_coverage)
      || ![
        "NO_EVIDENCE",
        "SUPPORTS_BLOCKED_HYPOTHESIS",
        "CONFLICTS_BLOCKED_HYPOTHESIS",
      ].includes(evidence.hypothesis_relation)
      || !Array.isArray(evidence.rays)
    ) {
      return null;
    }
    const requestedBearings = new Set();
    const rays = evidence.rays
      .slice(0, MAX_SCAN_RAYS_PER_EVIDENCE)
      .flatMap((rawRay) => {
        const ray = record(rawRay);
        const requestedBearingMdeg = finite(
          ray.requested_relative_bearing_mdeg,
        );
        const actualBearingMdeg = finite(
          ray.actual_relative_bearing_mdeg,
        );
        const rawIrProximity = finite(ray.raw_ir_proximity);
        const filteredIrProximity = finite(
          ray.filtered_ir_proximity,
        );
        if (
          requestedBearingMdeg === null
          || requestedBearingMdeg < -90000
          || requestedBearingMdeg > 90000
          || actualBearingMdeg === null
          || actualBearingMdeg < -100000
          || actualBearingMdeg > 100000
          || typeof ray.blocked !== "boolean"
          || requestedBearings.has(requestedBearingMdeg)
          || (
            rawIrProximity !== null
            && (rawIrProximity < 0 || rawIrProximity > 100)
          )
          || (
            filteredIrProximity !== null
            && (
              filteredIrProximity < 0
              || filteredIrProximity > 100
            )
          )
        ) {
          return [];
        }
        requestedBearings.add(requestedBearingMdeg);
        return [Object.freeze({
          requestedBearingMdeg,
          actualBearingMdeg,
          blocked: ray.blocked,
          rawIrProximity,
          filteredIrProximity,
        })];
      });
    if (rays.length !== evidence.rays.length) {
      return null;
    }
    const leftBoundaryMdeg = finite(evidence.left_boundary_mdeg);
    const rightBoundaryMdeg = finite(evidence.right_boundary_mdeg);
    if (
      (leftBoundaryMdeg !== null && leftBoundaryMdeg <= 0)
      || (rightBoundaryMdeg !== null && rightBoundaryMdeg >= 0)
    ) {
      return null;
    }
    return Object.freeze({
      targetHypothesisId: evidence.target_hypothesis_id,
      frameId: evidence.frame_id,
      hypothesisAnchorPose: Object.freeze({
        xMm: hypothesisAnchorX,
        yMm: hypothesisAnchorY,
        headingMdeg: hypothesisAnchorHeading,
      }),
      scanPose: hasScanPose
        ? Object.freeze({
          xMm: scanX,
          yMm: scanY,
          headingMdeg: scanHeading,
        })
        : null,
      basedOnMapVersion,
      spatiallyRenderable: hasScanPose,
      scanId: evidence.scan_id,
      completedAtUnixMs,
      ageMs: ageAt(completedAtUnixMs, nowUnixMs),
      status: evidence.status,
      reason: text(evidence.reason),
      bearingConvention: BODY_RELATIVE_BEARING_CONVENTION,
      geometryKind: ANGULAR_NONMETRIC_IR_SCAN,
      observationPattern: evidence.observation_pattern,
      arcCoverage: evidence.arc_coverage,
      boundaryCoverage: evidence.boundary_coverage,
      hypothesisRelation: evidence.hypothesis_relation,
      leftBoundaryMdeg,
      rightBoundaryMdeg,
      rays: Object.freeze(rays),
      provisional: true,
      readOnly: true,
    });
  }

  function normalizeSharedFrameTransform(value, identity) {
    const transform = record(value);
    const transformProvenance = Array.isArray(transform.provenance)
      ? transform.provenance
      : [];
    const provenanceValid = (
      transformProvenance.length > 0
      && transformProvenance.length <= 32
      && transformProvenance.every((item) => identifier(item) !== null)
      && new Set(transformProvenance).size === transformProvenance.length
    );
    const txMm = localCoordinate(transform.tx_mm);
    const tyMm = localCoordinate(transform.ty_mm);
    const yawMdeg = localHeading(transform.yaw_mdeg);
    const positionUncertaintyMm = nonnegativeInteger(
      transform.position_uncertainty_mm,
    );
    const yawUncertaintyMdeg = nonnegativeInteger(
      transform.yaw_uncertainty_mdeg,
    );
    if (
      transform.source_robot_id !== identity.robotId
      || transform.source_controller_id !== identity.controllerInstanceId
      || transform.source_frame_id !== identity.localFrameId
      || transform.source_generation_id !== identity.localGenerationId
      || transform.world_frame_id !== identity.worldFrameId
      || transform.world_generation_id !== identity.worldGenerationId
      || !Number.isSafeInteger(txMm)
      || !Number.isSafeInteger(tyMm)
      || !Number.isSafeInteger(yawMdeg)
      || positionUncertaintyMm === null
      || positionUncertaintyMm > MAX_LOCAL_COORDINATE_MM
      || yawUncertaintyMdeg === null
      || yawUncertaintyMdeg > 180000
      || !provenanceValid
    ) {
      return null;
    }
    return Object.freeze({
      sourceRobotId: identity.robotId,
      sourceControllerId: identity.controllerInstanceId,
      sourceFrameId: identity.localFrameId,
      sourceGenerationId: identity.localGenerationId,
      worldFrameId: identity.worldFrameId,
      worldGenerationId: identity.worldGenerationId,
      txMm,
      tyMm,
      yawMdeg,
      positionUncertaintyMm,
      yawUncertaintyMdeg,
      provenance: Object.freeze([...transformProvenance]),
    });
  }

  function normalizeSharedPose(value, identity) {
    const pose = record(value);
    const xMm = localCoordinate(pose.x_mm);
    const yMm = localCoordinate(pose.y_mm);
    const headingMdeg = localHeading(pose.heading_mdeg);
    const stateVersion = nonnegativeInteger(pose.state_version);
    const observedAtUnixMs = nonnegativeInteger(
      pose.observed_at_unix_ms,
    );
    const ageMs = nonnegativeInteger(pose.age_ms);
    const sourceId = identifier(pose.source_id);
    const poseProvenance = strictText(pose.provenance, 512);
    if (
      pose.frame_id !== identity.worldFrameId
      || pose.local_frame_id !== identity.localFrameId
      || !Number.isSafeInteger(xMm)
      || !Number.isSafeInteger(yMm)
      || !Number.isSafeInteger(headingMdeg)
      || stateVersion === null
      || observedAtUnixMs === null
      || ageMs === null
      || sourceId === null
      || poseProvenance === null
    ) {
      return null;
    }
    return Object.freeze({
      xMm,
      yMm,
      headingMdeg,
      frameId: identity.worldFrameId,
      localFrameId: identity.localFrameId,
      stateVersion,
      observedAtUnixMs,
      ageMs,
      sourceId,
      provenance: poseProvenance,
    });
  }

  function sharedRobotWithoutPose(
    identity,
    reasonCode,
    transform = null,
    contractValid = false,
  ) {
    return Object.freeze({
      contractValid,
      readOnly: true,
      status: "unavailable",
      reasonCode,
      robotId: identity.robotId,
      controllerInstanceId: identity.controllerInstanceId,
      localFrameId: identity.localFrameId,
      localGenerationId: identity.localGenerationId,
      worldFrameId: identity.worldFrameId,
      worldGenerationId: identity.worldGenerationId,
      robotPose: null,
      poseHistory: Object.freeze([]),
      poseHistoryEvicted: 0,
      collisionGeometry: null,
      frameTransform: transform,
      sourceMapId: null,
      sourceMapVersion: null,
      sourceStatus: null,
      capturedAtUnixMs: null,
      sourceAgeMs: null,
    });
  }

  function normalizeSharedRobot(value, worldIdentity) {
    const robot = record(value);
    const identity = Object.freeze({
      robotId: identifier(robot.robot_id),
      controllerInstanceId: identifier(robot.controller_instance_id),
      localFrameId: identifier(robot.local_frame_id),
      localGenerationId: identifier(robot.local_generation_id),
      worldFrameId: worldIdentity.worldFrameId,
      worldGenerationId: worldIdentity.worldGenerationId,
    });
    const identityValid = (
      identity.robotId !== null
      && identity.controllerInstanceId !== null
      && identity.localFrameId !== null
      && identity.localGenerationId !== null
    );
    const transform = identityValid
      ? normalizeSharedFrameTransform(robot.frame_transform, identity)
      : null;
    const unavailableShapeValid = (
      robot.status === "unavailable"
      && robot.robot_pose === null
      && Array.isArray(robot.pose_history)
      && robot.pose_history.length === 0
      && robot.collision_geometry === null
      && robot.source_map_id === null
      && robot.source_map_version === null
      && robot.source_status === null
      && robot.captured_at_unix_ms === null
      && robot.source_age_ms === null
    );
    if (
      robot.read_only === true
      && identityValid
      && transform !== null
      && unavailableShapeValid
    ) {
      return sharedRobotWithoutPose(
        identity,
        identifier(robot.reason_code) || "source_unavailable",
        transform,
        true,
      );
    }
    if (
      robot.read_only !== true
      || !identityValid
      || transform === null
      || robot.status !== "available"
    ) {
      return sharedRobotWithoutPose(
        identity,
        "source_identity_mismatch",
        transform,
      );
    }
    const robotPose = normalizeSharedPose(robot.robot_pose, identity);
    const history = Array.isArray(robot.pose_history)
      ? robot.pose_history
      : null;
    const retained = history === null
      ? []
      : history.slice(-MAX_POSE_HISTORY);
    const poseHistory = retained.map((pose) => (
      normalizeSharedPose(pose, identity)
    ));
    const sourceEvicted = nonnegativeInteger(
      robot.pose_history_evicted,
    );
    const sourceMapVersion = nonnegativeInteger(robot.source_map_version);
    const capturedAtUnixMs = nonnegativeInteger(
      robot.captured_at_unix_ms,
    );
    const sourceAgeMs = nonnegativeInteger(robot.source_age_ms);
    const sourceMapId = identifier(robot.source_map_id);
    const collisionGeometry = normalizeCollisionGeometry(
      robot.collision_geometry,
    );
    const sourceStatusValid = [
      "available", "pose_only", "qualitative_only",
    ].includes(robot.source_status);
    const latestPose = poseHistory[poseHistory.length - 1];
    const currentMatchesHistory = (
      robotPose !== null
      && latestPose !== undefined
      && ["xMm", "yMm", "headingMdeg"].every((field) => (
        robotPose[field] === latestPose[field]
      ))
    );
    const locallyEvicted = Math.max(0, history?.length - retained.length);
    const poseHistoryEvicted = sourceEvicted === null
      ? null
      : sourceEvicted + locallyEvicted;
    if (
      history === null
      || poseHistory.some((pose) => pose === null)
      || !currentMatchesHistory
      || sourceEvicted === null
      || !Number.isSafeInteger(poseHistoryEvicted)
      || sourceMapVersion === null
      || capturedAtUnixMs === null
      || sourceAgeMs === null
      || sourceMapId === null
      || !sourceStatusValid
      || robotPose.observedAtUnixMs > capturedAtUnixMs
      || (
        robot.collision_geometry !== null
        && collisionGeometry === null
      )
    ) {
      return sharedRobotWithoutPose(
        identity,
        "source_pose_invalid",
        transform,
      );
    }
    return Object.freeze({
      contractValid: true,
      readOnly: true,
      status: "available",
      reasonCode: identifier(robot.reason_code) || "pose_transformed",
      ...identity,
      robotPose,
      poseHistory: Object.freeze(poseHistory),
      poseHistoryEvicted,
      collisionGeometry,
      frameTransform: transform,
      sourceMapId,
      sourceMapVersion,
      sourceStatus: robot.source_status,
      capturedAtUnixMs,
      sourceAgeMs,
    });
  }

  function normalizeSharedSpatialMap(value, nowUnixMs = Date.now()) {
    const map = record(value);
    const now = Number.isFinite(nowUnixMs) && nowUnixMs >= 0
      ? nowUnixMs
      : Date.now();
    const worldIdentity = Object.freeze({
      worldFrameId: identifier(map.frame_id),
      worldGenerationId: identifier(map.world_generation_id),
    });
    const emptyEvidence = [
      map.cells,
      map.sensor_rays,
      map.qualitative_observations,
      map.scan_evidence_history,
      map.object_hypotheses,
    ].every((items) => Array.isArray(items) && items.length === 0);
    const topValid = (
      map.schema === SHARED_SPATIAL_MAP_SCHEMA
      && map.read_only === true
      && map.frame_kind === SHARED_FIXED_START
      && map.snapshot_semantics === SHARED_SNAPSHOT_SEMANTICS
      && worldIdentity.worldFrameId !== null
      && worldIdentity.worldGenerationId !== null
      && strictText(map.map_id, 384) !== null
      && identifier(map.source_id) !== null
      && strictText(map.provenance, 512) !== null
      && Array.isArray(map.robots)
      && map.robots.length <= MAX_SHARED_ROBOTS
      && ["available", "degraded", "unavailable"].includes(map.status)
      && map.bounds === null
      && map.navigation_authority === null
      && emptyEvidence
    );
    const seen = new Set();
    const robots = topValid ? map.robots.map((rawRobot) => {
      const normalized = normalizeSharedRobot(rawRobot, worldIdentity);
      const key = `${normalized.robotId}\u0000${
        normalized.controllerInstanceId
      }`;
      if (
        normalized.robotId === null
        || normalized.controllerInstanceId === null
        || seen.has(key)
      ) {
        return sharedRobotWithoutPose(
          normalized,
          "source_identity_duplicate",
          normalized.frameTransform,
        );
      }
      seen.add(key);
      return normalized;
    }) : [];
    const availableCount = robots.filter((robot) => (
      robot.status === "available"
    )).length;
    const detectedStatus = !topValid || availableCount === 0
      ? "unavailable"
      : availableCount === robots.length
        ? "available"
        : "degraded";
    const status = (
      !topValid
      || map.status === "unavailable"
      || detectedStatus === "unavailable"
    )
      ? "unavailable"
      : map.status === "degraded" || detectedStatus === "degraded"
        ? "degraded"
        : "available";
    const capturedAtUnixMs = nonnegativeInteger(
      map.captured_at_unix_ms,
    );
    const mapAgeMs = ageAt(capturedAtUnixMs, now);
    return Object.freeze({
      schema: topValid ? SHARED_SPATIAL_MAP_SCHEMA : null,
      contractValid: topValid,
      status,
      reasonCode: topValid
        ? (identifier(map.reason_code) || "shared_pose_snapshot")
        : "shared_contract_invalid",
      mapId: topValid ? map.map_id : null,
      robotId: null,
      frameId: topValid ? worldIdentity.worldFrameId : null,
      frameKind: topValid ? SHARED_FIXED_START : null,
      worldGenerationId: topValid
        ? worldIdentity.worldGenerationId
        : null,
      snapshotSemantics: topValid ? SHARED_SNAPSHOT_SEMANTICS : null,
      mapVersion: null,
      basedOnStateVersion: null,
      basedOnWorldModelVersion: null,
      capturedAtUnixMs,
      ageMs: mapAgeMs,
      sourceId: topValid ? map.source_id : null,
      provenance: topValid ? map.provenance : null,
      bounds: null,
      resolutionMm: null,
      collisionGeometry: null,
      hazardRetention: null,
      scanAttemptRetention: null,
      robotPose: null,
      poseHistory: Object.freeze([]),
      poseHistoryEvicted: 0,
      cells: Object.freeze([]),
      sensorRays: Object.freeze([]),
      objectHypotheses: Object.freeze([]),
      qualitativeObservations: Object.freeze([]),
      qualitativeObservationsEvicted: 0,
      scanEvidenceHistory: Object.freeze([]),
      scanEvidenceHistoryEvicted: 0,
      navigationTrace: null,
      robots: Object.freeze(robots),
    });
  }

  function normalizeSpatialMap(value, nowUnixMs = Date.now()) {
    if (record(value).schema === SHARED_SPATIAL_MAP_SCHEMA) {
      return normalizeSharedSpatialMap(value, nowUnixMs);
    }
    const map = record(value);
    const now = Number.isFinite(nowUnixMs) && nowUnixMs >= 0
      ? nowUnixMs
      : Date.now();
    const contractValid = (
      map.schema === SPATIAL_MAP_SCHEMA
      && map.read_only === true
    );
    const capturedAtUnixMs = finite(map.captured_at_unix_ms);
    const explicitAge = finite(map.age_ms);
    const mapAgeMs = explicitAge !== null && explicitAge >= 0
      ? explicitAge
      : ageAt(capturedAtUnixMs, now);
    const bounds = contractValid ? normalizeBounds(map.bounds) : null;
    const collisionGeometry = contractValid
      ? normalizeCollisionGeometry(map.collision_geometry)
      : null;
    const hazardRetentionValue = record(map.hazard_retention);
    const hazardCapacity = Number.isSafeInteger(
      hazardRetentionValue.capacity,
    ) && hazardRetentionValue.capacity > 0
      ? hazardRetentionValue.capacity
      : null;
    const hazardRetainedCount = Number.isSafeInteger(
      hazardRetentionValue.retained_count,
    ) && hazardRetentionValue.retained_count >= 0
      ? hazardRetentionValue.retained_count
      : null;
    const hazardEvictedCount = Number.isSafeInteger(
      hazardRetentionValue.evicted_count,
    ) && hazardRetentionValue.evicted_count >= 0
      ? hazardRetentionValue.evicted_count
      : null;
    const hazardRetention = (
      contractValid
      && hazardCapacity !== null
      && hazardRetainedCount !== null
      && hazardRetainedCount <= hazardCapacity
      && hazardEvictedCount !== null
    )
      ? Object.freeze({
        capacity: hazardCapacity,
        retainedCount: hazardRetainedCount,
        evictedCount: hazardEvictedCount,
        lastEvictionReason: text(
          hazardRetentionValue.last_eviction_reason,
        ),
      })
      : null;
    const scanAttemptRetentionValue = record(
      map.scan_attempt_retention,
    );
    const scanPerHazardCapacity = Number.isSafeInteger(
      scanAttemptRetentionValue.per_hazard_capacity,
    ) && scanAttemptRetentionValue.per_hazard_capacity > 0
      ? scanAttemptRetentionValue.per_hazard_capacity
      : null;
    const scanMapCapacity = Number.isSafeInteger(
      scanAttemptRetentionValue.map_capacity,
    ) && scanAttemptRetentionValue.map_capacity > 0
      ? scanAttemptRetentionValue.map_capacity
      : null;
    const retainedScanAttemptCount = Number.isSafeInteger(
      scanAttemptRetentionValue.retained_count,
    ) && scanAttemptRetentionValue.retained_count >= 0
      ? scanAttemptRetentionValue.retained_count
      : null;
    const evictedScanAttemptCount = Number.isSafeInteger(
      scanAttemptRetentionValue.evicted_count,
    ) && scanAttemptRetentionValue.evicted_count >= 0
      ? scanAttemptRetentionValue.evicted_count
      : null;
    const scanAttemptRetention = (
      contractValid
      && scanPerHazardCapacity !== null
      && scanMapCapacity !== null
      && scanPerHazardCapacity <= scanMapCapacity
      && retainedScanAttemptCount !== null
      && retainedScanAttemptCount <= scanMapCapacity
      && evictedScanAttemptCount !== null
    )
      ? Object.freeze({
        perHazardCapacity: scanPerHazardCapacity,
        mapCapacity: scanMapCapacity,
        retainedCount: retainedScanAttemptCount,
        evictedCount: evictedScanAttemptCount,
        lastEvictionReason: text(
          scanAttemptRetentionValue.last_eviction_reason,
        ),
      })
      : null;
    const resolutionMm = positive(map.resolution_mm);
    const rawCells = contractValid && Array.isArray(map.cells)
      ? map.cells.slice(0, MAX_SPATIAL_CELLS)
      : [];
    const cells = rawCells.flatMap((rawCell) => {
      const cell = record(rawCell);
      const xMm = finite(cell.x_mm);
      const yMm = finite(cell.y_mm);
      if (xMm === null || yMm === null) {
        return [];
      }
      const state = (
        cell.state === "FREE"
        || cell.state === "OCCUPIED"
        || cell.state === "UNCERTAIN"
      )
        ? cell.state
        : "UNKNOWN";
      return [Object.freeze({
        xMm,
        yMm,
        sizeMm: positive(cell.size_mm) || resolutionMm,
        state,
        confidenceMilli: finite(cell.confidence_milli),
        sourceId: text(cell.source_id),
        provenance: provenance(cell.provenance),
        ageMs: ageAt(cell.observed_at_unix_ms, now),
      })];
    });
    const rawRays = contractValid && Array.isArray(map.sensor_rays)
      ? map.sensor_rays.slice(0, MAX_SENSOR_RAYS)
      : [];
    const sensorRays = rawRays.flatMap((rawRay) => {
      const ray = record(rawRay);
      const originX = finite(ray.origin_x_mm);
      const originY = finite(ray.origin_y_mm);
      const endX = finite(ray.end_x_mm);
      const endY = finite(ray.end_y_mm);
      const validUntil = finite(
        ray.valid_until_unix_ms === undefined
          ? ray.fresh_until_unix_ms
          : ray.valid_until_unix_ms,
      );
      if (
        originX === null
        || originY === null
        || endX === null
        || endY === null
        || validUntil === null
        || validUntil <= now
      ) {
        return [];
      }
      return [Object.freeze({
        originX,
        originY,
        endX,
        endY,
        sourceId: text(ray.source_id),
        provenance: provenance(ray.provenance),
        ageMs: ageAt(ray.observed_at_unix_ms, now),
        validUntilUnixMs: validUntil,
      })];
    });
    const rawHypotheses = (
      contractValid
      && Array.isArray(map.object_hypotheses)
    )
      ? map.object_hypotheses.slice(0, MAX_OBJECT_HYPOTHESES)
      : [];
    const objectHypotheses = rawHypotheses.flatMap((rawHypothesis) => {
      const hypothesis = record(rawHypothesis);
      const xMm = finite(hypothesis.x_mm);
      const yMm = finite(hypothesis.y_mm);
      const provisional = (
        hypothesis.provisional === true
        && hypothesis.geometry_kind === "QUALITATIVE_FORWARD_ENVELOPE"
      );
      const anchor = record(hypothesis.anchor_pose);
      const anchorX = finite(anchor.x_mm);
      const anchorY = finite(anchor.y_mm);
      const anchorHeading = finite(anchor.heading_mdeg);
      const anchorPose = (
        provisional
        && anchorX !== null
        && anchorY !== null
        && anchorHeading !== null
      )
        ? Object.freeze({
          xMm: anchorX,
          yMm: anchorY,
          headingMdeg: anchorHeading,
        })
        : null;
      const confidence = finite(hypothesis.confidence_milli);
      const validUntil = finite(hypothesis.valid_until_unix_ms);
      if (
        (
          !provisional
          && (xMm === null || yMm === null)
        )
        || (
          provisional
          && (
            anchorPose === null
            || confidence === null
            || confidence < 0
            || confidence > 400
          )
        )
        || validUntil !== null
        && validUntil <= now
      ) {
        return [];
      }
      return [Object.freeze({
        hypothesisId: text(hypothesis.hypothesis_id),
        label: text(hypothesis.label),
        xMm: provisional ? null : xMm,
        yMm: provisional ? null : yMm,
        anchorPose,
        geometryKind: text(hypothesis.geometry_kind),
        bearing: text(hypothesis.bearing),
        relation: text(hypothesis.relation),
        provisional,
        confidenceMilli: confidence,
        sourceId: text(hypothesis.source_id),
        provenance: provenance(hypothesis.provenance),
        ageMs: observationAge(hypothesis, now),
      })];
    });
    const rawQualitative = (
      contractValid
      && Array.isArray(map.qualitative_observations)
    )
      ? map.qualitative_observations.slice(
        -MAX_QUALITATIVE_OBSERVATIONS,
      )
      : [];
    const qualitativeObservations = rawQualitative.flatMap((rawValue) => {
      const observation = record(rawValue);
      const relation = text(observation.relation);
      if (relation === null) {
        return [];
      }
      const rawIr = finite(observation.raw_ir_proximity);
      const confidence = finite(observation.confidence_milli);
      return [Object.freeze({
        bearing: text(observation.bearing),
        relation,
        rawIrProximity: (
          rawIr !== null && rawIr >= 0 && rawIr <= 100
            ? rawIr
            : null
        ),
        confidenceMilli: (
          confidence !== null
          && confidence >= 0
          && confidence <= 1000
            ? confidence
            : null
        ),
        sourceId: text(observation.source_id),
        provenance: provenance(observation.provenance),
        provisional: observation.provisional === true,
        ageMs: observationAge(observation, now),
      })];
    });
    const poseValue = record(map.robot_pose);
    const poseX = finite(poseValue.x_mm);
    const poseY = finite(poseValue.y_mm);
    const headingMdeg = finite(poseValue.heading_mdeg);
    const robotPose = (
      contractValid
      && poseX !== null
      && poseY !== null
      && headingMdeg !== null
    )
      ? Object.freeze({
        xMm: poseX,
        yMm: poseY,
        headingMdeg,
        sourceId: text(poseValue.source_id) || text(map.source_id),
        provenance: (
          provenance(poseValue.provenance)
          || provenance(map.provenance)
        ),
        ageMs: (
          ageAt(poseValue.observed_at_unix_ms, now)
          ?? mapAgeMs
        ),
      })
      : null;
    const rawPoseHistory = (
      contractValid
      && Array.isArray(map.pose_history)
    )
      ? map.pose_history.slice(-MAX_POSE_HISTORY)
      : [];
    const poseHistory = [];
    rawPoseHistory.forEach((rawValue) => {
      const value = record(rawValue);
      const xMm = finite(value.x_mm);
      const yMm = finite(value.y_mm);
      const historyHeadingMdeg = finite(value.heading_mdeg);
      if (
        xMm === null
        || yMm === null
        || historyHeadingMdeg === null
        || (
          text(value.frame_id) !== null
          && text(map.frame_id) !== null
          && value.frame_id !== map.frame_id
        )
      ) {
        return;
      }
      const previous = poseHistory[poseHistory.length - 1];
      if (
        previous
        && previous.xMm === xMm
        && previous.yMm === yMm
        && previous.headingMdeg === historyHeadingMdeg
      ) {
        return;
      }
      poseHistory.push(Object.freeze({
        xMm,
        yMm,
        headingMdeg: historyHeadingMdeg,
        sourceId: text(value.source_id) || text(map.source_id),
        provenance: (
          provenance(value.provenance)
          || provenance(map.provenance)
        ),
        ageMs: (
          ageAt(value.observed_at_unix_ms, now)
          ?? observationAge(value, now)
          ?? mapAgeMs
        ),
      }));
    });
    const rawScanEvidence = (
      contractValid
      && Array.isArray(map.scan_evidence_history)
    )
      ? map.scan_evidence_history.slice(-MAX_SCAN_EVIDENCE)
      : [];
    const scanEvidenceHistory = rawScanEvidence.flatMap((value) => {
      const evidence = normalizeScanEvidence(
        value,
        text(map.frame_id),
        now,
      );
      return evidence === null ? [] : [evidence];
    });
    const navigationTrace = contractValid
      ? normalizeNavigationTrace(
        map.navigation_trace,
        text(map.frame_id),
        now,
      )
      : null;
    return Object.freeze({
      schema: contractValid ? SPATIAL_MAP_SCHEMA : null,
      contractValid,
      status: text(map.status) || "unknown",
      reasonCode: text(map.reason_code),
      robotId: text(map.robot_id),
      frameId: text(map.frame_id),
      frameKind: text(map.frame_kind),
      mapVersion: Number.isSafeInteger(map.map_version)
        ? map.map_version
        : null,
      basedOnStateVersion: (
        Number.isSafeInteger(map.based_on_state_version)
        && map.based_on_state_version >= 0
          ? map.based_on_state_version
          : null
      ),
      basedOnWorldModelVersion: (
        Number.isSafeInteger(map.based_on_world_model_version)
        && map.based_on_world_model_version >= 0
          ? map.based_on_world_model_version
          : null
      ),
      capturedAtUnixMs,
      ageMs: mapAgeMs,
      sourceId: text(map.source_id),
      provenance: provenance(map.provenance),
      bounds,
      resolutionMm,
      collisionGeometry,
      hazardRetention,
      scanAttemptRetention,
      robotPose,
      poseHistory: Object.freeze(poseHistory),
      poseHistoryEvicted: (
        Number.isSafeInteger(map.pose_history_evicted)
        && map.pose_history_evicted >= 0
          ? map.pose_history_evicted
          : 0
      ),
      cells: Object.freeze(cells),
      sensorRays: Object.freeze(sensorRays),
      objectHypotheses: Object.freeze(objectHypotheses),
      qualitativeObservations: Object.freeze(
        qualitativeObservations,
      ),
      qualitativeObservationsEvicted: (
        Number.isSafeInteger(map.qualitative_observations_evicted)
        && map.qualitative_observations_evicted >= 0
          ? map.qualitative_observations_evicted
          : 0
      ),
      scanEvidenceHistory: Object.freeze(scanEvidenceHistory),
      scanEvidenceHistoryEvicted: (
        Number.isSafeInteger(map.scan_evidence_history_evicted)
        && map.scan_evidence_history_evicted >= 0
          ? map.scan_evidence_history_evicted
          : 0
      ),
      navigationTrace,
    });
  }

  function replaceRenderedItems(container, items, renderItem) {
    if (!container || typeof container.replaceChildren !== "function") {
      throw new TypeError("A render container is required.");
    }
    if (typeof renderItem !== "function") {
      throw new TypeError("A render callback is required.");
    }
    const normalized = Array.isArray(items) ? items : [];
    container.replaceChildren(...normalized.map((item) => renderItem(item)));
    return normalized;
  }

  function selectSpatialMapEndpoint(capabilities) {
    return record(capabilities).shared_spatial_map === "read_only"
      ? "/api/v1/shared-map"
      : "/api/v1/map";
  }

  function isTerminalSessionError(value) {
    return Boolean(
      value
      && typeof value === "object"
      && value.status === 403
      && value.code === SESSION_REJECTED_CODE,
    );
  }

  function createSessionGuard() {
    let expired = false;
    const listeners = new Set();

    function observe(error) {
      if (!isTerminalSessionError(error)) {
        return false;
      }
      if (expired) {
        return true;
      }
      expired = true;
      Array.from(listeners).forEach((listener) => listener());
      return true;
    }

    function subscribe(listener) {
      if (typeof listener !== "function") {
        throw new TypeError("Session expiry listener is invalid.");
      }
      listeners.add(listener);
      if (expired) {
        listener();
      }
      return () => listeners.delete(listener);
    }

    return Object.freeze({
      isExpired: () => expired,
      observe,
      subscribe,
    });
  }

  function createDashboardRequest(options = {}) {
    const sessionToken = typeof options.sessionToken === "string"
      ? options.sessionToken
      : "";
    const sessionGuard = options.sessionGuard;
    const fetchRequest = options.fetchRequest || (
      typeof global.fetch === "function" ? global.fetch.bind(global) : null
    );
    const setTimer = options.setTimeout || global.setTimeout;
    const clearTimer = options.clearTimeout || global.clearTimeout;
    const AbortControllerClass = (
      options.AbortController || global.AbortController
    );
    if (
      !sessionGuard
      || typeof sessionGuard.isExpired !== "function"
      || typeof sessionGuard.observe !== "function"
      || typeof fetchRequest !== "function"
      || typeof setTimer !== "function"
      || typeof clearTimer !== "function"
      || typeof AbortControllerClass !== "function"
    ) {
      throw new TypeError("Dashboard request dependencies are invalid.");
    }

    function failure(code, status = null, cause = null) {
      const error = new Error(code);
      error.code = code;
      error.status = status;
      if (cause) {
        error.cause = cause;
      }
      return error;
    }

    return async function request(path, requestOptions = {}) {
      const method = requestOptions.method || "GET";
      const headers = { Accept: "application/json" };
      const fetchOptions = {
        method,
        headers,
        cache: "no-store",
        credentials: "same-origin",
      };
      if (path.startsWith("/api/")) {
        if (sessionGuard.isExpired()) {
          throw failure(SESSION_REJECTED_CODE, 403);
        }
        if (
          !sessionToken
          || sessionToken === "__ROBOT_DASHBOARD_TOKEN__"
        ) {
          throw failure("dashboard_session_missing");
        }
        headers["X-Robot-Dashboard-Token"] = sessionToken;
      }
      if (method === "POST" || method === "PUT") {
        if (Object.hasOwn(requestOptions, "rawBody")) {
          Object.assign(headers, requestOptions.headers || {});
          fetchOptions.body = requestOptions.rawBody;
        } else {
          headers["Content-Type"] = "application/json";
          fetchOptions.body = JSON.stringify(requestOptions.body || {});
        }
      }
      const controller = new AbortControllerClass();
      const parentAbort = () => controller.abort();
      if (requestOptions.signal) {
        if (requestOptions.signal.aborted) {
          controller.abort();
        } else {
          requestOptions.signal.addEventListener(
            "abort",
            parentAbort,
            { once: true },
          );
        }
      }
      const timer = setTimer(
        () => controller.abort(),
        requestOptions.timeout || 15000,
      );
      fetchOptions.signal = controller.signal;
      try {
        const response = await fetchRequest(path, fetchOptions);
        const raw = await response.text();
        let payload = {};
        if (raw) {
          try {
            payload = JSON.parse(raw);
          } catch (_error) {
            throw failure("invalid_server_json", response.status);
          }
        }
        if (!response.ok) {
          const responseError = record(payload.error);
          const requestError = failure(
            text(responseError.code) || "http_error",
            response.status,
          );
          sessionGuard.observe(requestError);
          throw requestError;
        }
        return payload;
      } catch (error) {
        if (error && error.name === "AbortError") {
          throw failure("request_timeout", null, error);
        }
        if (error && error.code) {
          throw error;
        }
        throw failure("network_error", null, error);
      } finally {
        clearTimer(timer);
        if (requestOptions.signal) {
          requestOptions.signal.removeEventListener(
            "abort",
            parentAbort,
          );
        }
      }
    };
  }

  function transitionTurnPoll(current, event) {
    const previous = (
      current && typeof current === "object" && !Array.isArray(current)
        ? current
        : {}
    );
    const previousFailures = Number.isSafeInteger(previous.failures)
      && previous.failures >= 0
      ? previous.failures
      : 0;
    const previousConnection = (
      previous.connection === "retrying"
      || previous.connection === "unknown"
    )
      ? previous.connection
      : "connected";
    if (event && event.type === "success") {
      const turn = (
        event.turn && typeof event.turn === "object" && !Array.isArray(event.turn)
          ? event.turn
          : {}
      );
      return Object.freeze({
        failures: 0,
        connection: "connected",
        becameUnknown: false,
        recovered: previousConnection !== "connected",
        terminal: TERMINAL_TURN_STATES.has(turn.status),
        retry: false,
        retryDelayMs: null,
      });
    }
    if (!event || event.type !== "failure") {
      throw new TypeError("Turn polling requires a success or failure event.");
    }
    const failures = previousFailures + 1;
    const connection = failures >= TURN_POLL_POLICY.unknownAfterFailures
      ? "unknown"
      : "retrying";
    return Object.freeze({
      failures,
      connection,
      becameUnknown: (
        connection === "unknown"
        && previousConnection !== "unknown"
      ),
      recovered: false,
      terminal: false,
      retry: true,
      retryDelayMs: Math.min(
        TURN_POLL_POLICY.maxDelayMs,
        TURN_POLL_POLICY.baseDelayMs * failures,
      ),
    });
  }

  global.RobotDashboardLogic = Object.freeze({
    MAX_SHARED_ROBOTS,
    MAX_POSE_HISTORY,
    MAX_QUALITATIVE_OBSERVATIONS,
    MAX_SCAN_EVIDENCE,
    SHARED_SPATIAL_MAP_SCHEMA,
    SPATIAL_MAP_SCHEMA,
    TURN_POLL_POLICY,
    createDashboardRequest,
    createSessionGuard,
    isTerminalSessionError,
    normalizeSharedSpatialMap,
    normalizeSpatialMap,
    replaceRenderedItems,
    selectSpatialMapEndpoint,
    transitionTurnPoll,
  });
})(typeof window === "undefined" ? globalThis : window);
