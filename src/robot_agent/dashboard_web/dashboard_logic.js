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
  const MAX_SPATIAL_CELLS = 10000;
  const MAX_SENSOR_RAYS = 256;
  const MAX_OBJECT_HYPOTHESES = 256;
  const MAX_QUALITATIVE_OBSERVATIONS = 256;
  const MAX_POSE_HISTORY = 256;
  const MAX_SCAN_EVIDENCE = 8;
  const MAX_SCAN_RAYS_PER_EVIDENCE = 16;
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

  function normalizeSpatialMap(value, nowUnixMs = Date.now()) {
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
        0,
        MAX_QUALITATIVE_OBSERVATIONS,
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
      scanEvidenceHistory: Object.freeze(scanEvidenceHistory),
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
    SPATIAL_MAP_SCHEMA,
    TURN_POLL_POLICY,
    createDashboardRequest,
    createSessionGuard,
    isTerminalSessionError,
    normalizeSpatialMap,
    replaceRenderedItems,
    transitionTurnPoll,
  });
})(typeof window === "undefined" ? globalThis : window);
