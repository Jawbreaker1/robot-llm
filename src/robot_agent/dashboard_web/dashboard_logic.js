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
      robotPose,
      cells: Object.freeze(cells),
      sensorRays: Object.freeze(sensorRays),
      objectHypotheses: Object.freeze(objectHypotheses),
      qualitativeObservations: Object.freeze(
        qualitativeObservations,
      ),
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
    normalizeSpatialMap,
    replaceRenderedItems,
    transitionTurnPoll,
  });
})(typeof window === "undefined" ? globalThis : window);
