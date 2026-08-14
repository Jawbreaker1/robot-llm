((global) => {
  "use strict";

  const SVG_NAMESPACE = "http://www.w3.org/2000/svg";
  const LOCAL_ODOMETRY = "LOCAL_ODOMETRY";
  const SHARED_SPATIAL_MAP_SCHEMA = "robot-spatial-map/v2";
  const QUALITATIVE_FORWARD_ENVELOPE = (
    "QUALITATIVE_FORWARD_ENVELOPE"
  );
  const SVG_WIDTH = 1000;
  const SVG_HEIGHT = 620;
  const MAX_RENDERED_QUALITATIVE_OBSERVATIONS = 100;
  const blastMapSemantics = global.RobotBlastMapSemantics;

  function create(options = {}) {
    const documentApi = options.document;
    const normalizeSpatialMap = options.normalizeSpatialMap;
    const t = options.translate;
    const formatNumber = options.formatNumber;
    if (
      !documentApi
      || typeof documentApi.getElementById !== "function"
      || typeof documentApi.createElement !== "function"
      || typeof documentApi.createElementNS !== "function"
      || typeof normalizeSpatialMap !== "function"
      || typeof t !== "function"
      || typeof formatNumber !== "function"
      || !blastMapSemantics
    ) {
      throw new TypeError("Spatial map presenter dependencies are invalid.");
    }

    const byId = (id) => documentApi.getElementById(id);
    const safeText = (value, fallback = t("common.missing")) => (
      typeof value === "string" && value.length > 0 ? value : fallback
    );

    function createElement(tag, className, text) {
      const node = documentApi.createElement(tag);
      if (className) {
        node.className = className;
      }
      if (text !== undefined && text !== null) {
        node.textContent = String(text);
      }
      return node;
    }

    function createSvgElement(tag, attributes = {}) {
      const node = documentApi.createElementNS(SVG_NAMESPACE, tag);
      Object.entries(attributes).forEach(([name, value]) => {
        if (value !== null && value !== undefined) {
          node.setAttribute(name, String(value));
        }
      });
      return node;
    }

    function appendSvgTitle(node, parts) {
      const values = parts.filter((value) => (
        typeof value === "string" && value.length > 0
      ));
      if (values.length === 0) {
        return;
      }
      const title = createSvgElement("title");
      title.textContent = values.join(" · ");
      node.appendChild(title);
    }

    function localizedValue(key, fallback) {
      const translated = t(key);
      return translated === key ? fallback : translated;
    }

    function formatMapAge(ageMs) {
      if (!Number.isFinite(ageMs) || ageMs < 0) {
        return t("common.missing");
      }
      if (ageMs < 1000) {
        return t("map.age.milliseconds", {
          value: formatNumber(Math.round(ageMs)),
        });
      }
      if (ageMs < 60000) {
        return t("map.age.seconds", {
          value: formatNumber(ageMs / 1000, {
            maximumFractionDigits: 1,
          }),
        });
      }
      return t("map.age.minutes", {
        value: formatNumber(ageMs / 60000, {
          maximumFractionDigits: 1,
        }),
      });
    }

    function mapTooltipParts(item) {
      return [
        item.sourceId
          ? t("map.tooltip.source", { source: item.sourceId })
          : "",
        Number.isFinite(item.ageMs)
          ? t("map.tooltip.age", { age: formatMapAge(item.ageMs) })
          : "",
        item.provenance
          ? t("map.tooltip.provenance", {
            provenance: item.provenance,
          })
          : "",
      ];
    }

    function mapProjection(bounds) {
      const width = SVG_WIDTH;
      const height = SVG_HEIGHT;
      const padding = 34;
      const worldWidth = bounds.maxX - bounds.minX;
      const worldHeight = bounds.maxY - bounds.minY;
      const scale = Math.min(
        (width - 2 * padding) / worldWidth,
        (height - 2 * padding) / worldHeight,
      );
      const contentWidth = worldWidth * scale;
      const contentHeight = worldHeight * scale;
      const offsetX = (width - contentWidth) / 2;
      const offsetY = (height - contentHeight) / 2;
      return {
        scale,
        point(xMm, yMm) {
          return {
            x: offsetX + (xMm - bounds.minX) * scale,
            y: height - offsetY - (yMm - bounds.minY) * scale,
          };
        },
      };
    }

    function appendNavigationTracePoints(trace, addPoint) {
      if (!trace) {
        return;
      }
      blastMapSemantics.appendGoalPoints(trace.finalGoal, addPoint);
      blastMapSemantics.appendAdvisoryWaypointPoint(
        trace.advisoryWaypoint, addPoint,
      );
      if (trace.plannedLeg) {
        addPoint(
          trace.plannedLeg.bindPose.xMm,
          trace.plannedLeg.bindPose.yMm,
        );
        addPoint(
          trace.plannedLeg.waypoint.xMm,
          trace.plannedLeg.waypoint.yMm,
        );
      }
      blastMapSemantics.appendRoutePoints(trace.localDetourRoute, addPoint);
      trace.planarScanViews.forEach((view) => {
        addPoint(view.scanPose.xMm, view.scanPose.yMm);
        view.points.forEach((point) => {
          addPoint(point.sensorOriginX, point.sensorOriginY);
          addPoint(point.nominalEchoX, point.nominalEchoY);
        });
      });
    }

    function localOdometryScene(map) {
      const points = [];
      const cues = [];
      const cueKeys = new Set();

      function addPoint(xMm, yMm) {
        if (Number.isFinite(xMm) && Number.isFinite(yMm)) {
          points.push({ xMm, yMm });
        }
      }

      function addCue(anchorPose, evidence) {
        if (
          !anchorPose
          || !Number.isFinite(anchorPose.xMm)
          || !Number.isFinite(anchorPose.yMm)
          || !Number.isFinite(anchorPose.headingMdeg)
          || evidence.provisional !== true
          || evidence.bearing !== "FORWARD"
        ) {
          return;
        }
        const key = [
          anchorPose.xMm,
          anchorPose.yMm,
          anchorPose.headingMdeg,
          evidence.relation,
        ].join(":");
        if (cueKeys.has(key)) {
          return;
        }
        cueKeys.add(key);
        addPoint(anchorPose.xMm, anchorPose.yMm);
        cues.push({ anchorPose, evidence });
      }

      if (map.robotPose) {
        addPoint(map.robotPose.xMm, map.robotPose.yMm);
      }
      map.poseHistory.forEach((pose) => {
        addPoint(pose.xMm, pose.yMm);
      });
      appendNavigationTracePoints(map.navigationTrace, addPoint);
      map.objectHypotheses.forEach((hypothesis) => {
        blastMapSemantics.appendObstaclePoints(hypothesis, addPoint);
        if (
          hypothesis.provisional
          && hypothesis.geometryKind === QUALITATIVE_FORWARD_ENVELOPE
        ) {
          addCue(hypothesis.anchorPose, hypothesis);
        }
      });
      if (
        map.robotPose
        && map.qualitativeObservations.length > 0
      ) {
        addCue(
          map.robotPose,
          map.qualitativeObservations[
            map.qualitativeObservations.length - 1
          ],
        );
      }
      const latestScans = new Map();
      map.scanEvidenceHistory.forEach((scan) => {
        if (scan.spatiallyRenderable && scan.scanPose) {
          addPoint(scan.scanPose.xMm, scan.scanPose.yMm);
          latestScans.set(scan.targetHypothesisId, scan);
        }
      });
      footprintCorners(
        map.robotPose,
        map.collisionGeometry,
        true,
      ).forEach((point) => addPoint(point.xMm, point.yMm));
      return {
        points,
        cues,
        latestScans: Array.from(latestScans.values()),
      };
    }

    function fittedMetricProjection(points) {
      const minX = Math.min(...points.map((point) => point.xMm));
      const maxX = Math.max(...points.map((point) => point.xMm));
      const minY = Math.min(...points.map((point) => point.yMm));
      const maxY = Math.max(...points.map((point) => point.yMm));
      const spanX = maxX - minX;
      const spanY = maxY - minY;
      const padding = 100;
      const scaleCandidates = [];
      if (spanX > 0) {
        scaleCandidates.push((SVG_WIDTH - 2 * padding) / spanX);
      }
      if (spanY > 0) {
        scaleCandidates.push((SVG_HEIGHT - 2 * padding) / spanY);
      }
      const scale = scaleCandidates.length > 0
        ? Math.min(...scaleCandidates)
        : 1;
      const centerX = (minX + maxX) / 2;
      const centerY = (minY + maxY) / 2;
      return {
        point(xMm, yMm) {
          return {
            x: SVG_WIDTH / 2 + (xMm - centerX) * scale,
            y: SVG_HEIGHT / 2 - (yMm - centerY) * scale,
          };
        },
      };
    }

    function screenPoint(origin, headingRadians, length) {
      return {
        x: origin.x + Math.cos(headingRadians) * length,
        y: origin.y - Math.sin(headingRadians) * length,
      };
    }

    function bodyPointToWorld(pose, forwardMm, leftMm) {
      const heading = pose.headingMdeg / 1000 * Math.PI / 180;
      return {
        xMm: pose.xMm
          + forwardMm * Math.cos(heading)
          - leftMm * Math.sin(heading),
        yMm: pose.yMm
          + forwardMm * Math.sin(heading)
          + leftMm * Math.cos(heading),
      };
    }

    function footprintCorners(pose, geometry, includeMargin = false) {
      if (!pose || geometry?.geometry !== "ASYMMETRIC_RECTANGLE") {
        return [];
      }
      const margin = includeMargin ? geometry.clearanceMarginMm : 0;
      return [
        bodyPointToWorld(
          pose,
          geometry.frontExtentMm + margin,
          geometry.leftExtentMm + margin,
        ),
        bodyPointToWorld(
          pose,
          geometry.frontExtentMm + margin,
          -geometry.rightExtentMm - margin,
        ),
        bodyPointToWorld(
          pose,
          -geometry.rearExtentMm - margin,
          -geometry.rightExtentMm - margin,
        ),
        bodyPointToWorld(
          pose,
          -geometry.rearExtentMm - margin,
          geometry.leftExtentMm + margin,
        ),
      ];
    }

    function sharedWorldScene(map) {
      const points = [];
      function addPoint(point) {
        if (
          point
          && Number.isFinite(point.xMm)
          && Number.isFinite(point.yMm)
        ) {
          points.push(point);
        }
      }
      map.robots.forEach((robot) => {
        if (robot.status !== "available" || !robot.robotPose) {
          return;
        }
        robot.poseHistory.forEach(addPoint);
        addPoint(robot.robotPose);
        appendNavigationTracePoints(
          robot.navigationTrace,
          (xMm, yMm) => addPoint({ xMm, yMm }),
        );
        footprintCorners(
          robot.robotPose,
          robot.collisionGeometry,
          true,
        ).forEach(addPoint);
        if (
          robot.collisionGeometry?.geometry === "SYMMETRIC_CIRCLE"
        ) {
          const radius = robot.collisionGeometry.radiusMm;
          addPoint({
            xMm: robot.robotPose.xMm + radius,
            yMm: robot.robotPose.yMm,
          });
          addPoint({
            xMm: robot.robotPose.xMm - radius,
            yMm: robot.robotPose.yMm,
          });
          addPoint({
            xMm: robot.robotPose.xMm,
            yMm: robot.robotPose.yMm + radius,
          });
          addPoint({
            xMm: robot.robotPose.xMm,
            yMm: robot.robotPose.yMm - radius,
          });
        }
      });
      blastMapSemantics.appendSharedObstaclePoints(
        map.robots,
        (xMm, yMm) => addPoint({ xMm, yMm }),
      );
      return Object.freeze({ points: Object.freeze(points) });
    }

    function scanBearingLabel(bearingMdeg) {
      const degrees = Math.abs(bearingMdeg) / 1000;
      const value = formatNumber(degrees, {
        maximumFractionDigits: 1,
      });
      if (bearingMdeg > 0) {
        return t("map.scan.bearing_left", { value });
      }
      if (bearingMdeg < 0) {
        return t("map.scan.bearing_right", { value });
      }
      return t("map.scan.bearing_center");
    }

    function retainedScanCounts(map) {
      const result = new Map();
      map.scanEvidenceHistory.forEach((scan) => {
        result.set(
          scan.targetHypothesisId,
          (result.get(scan.targetHypothesisId) || 0) + 1,
        );
      });
      return result;
    }

    function headingArrowPath(origin, headingRadians, length) {
      const end = screenPoint(origin, headingRadians, length);
      const left = screenPoint(
        end,
        headingRadians + Math.PI - 0.55,
        12,
      );
      const right = screenPoint(
        end,
        headingRadians + Math.PI + 0.55,
        12,
      );
      return [
        `M ${origin.x} ${origin.y}`,
        `L ${end.x} ${end.y}`,
        `M ${left.x} ${left.y}`,
        `L ${end.x} ${end.y}`,
        `L ${right.x} ${right.y}`,
      ].join(" ");
    }

    function blastRenderUi() {
      return {
        svg: createSvgElement, title: appendSvgTitle, t,
        format: formatNumber, heading: headingArrowPath,
        tooltip: mapTooltipParts,
      };
    }

    function renderNavigationTrace(
      layer,
      map,
      projection,
      scanLayer = layer,
    ) {
      const trace = map.navigationTrace;
      if (!trace) {
        return;
      }

      blastMapSemantics.renderGoal(
        layer, trace.finalGoal, projection, blastRenderUi(),
      );

      const leg = trace.plannedLeg;
      if (leg) {
        const enforcedRoute = (
          leg.scope === "LOCAL_DETOUR_ROUTE"
          && leg.routeEligible === true
        );
        const bind = projection.point(
          leg.bindPose.xMm,
          leg.bindPose.yMm,
        );
        const waypoint = projection.point(
          leg.waypoint.xMm,
          leg.waypoint.yMm,
        );
        const waypointHeading = (
          leg.waypoint.headingMdeg / 1000 * Math.PI / 180
        );
        const group = createSvgElement("g", {
          class: "map-planned-leg",
          "data-kind": leg.kind,
          "data-scope": leg.scope,
          "data-selected-side": leg.selectedSide,
          "data-clearance-proven": String(leg.clearanceProven),
          "data-passage-proven": String(leg.passageProven),
          "data-route-eligible": String(leg.routeEligible),
        });
        appendSvgTitle(group, [
          t(
            enforcedRoute
              ? "map.navigation_trace.local_detour_title"
              : "map.navigation_trace.planned_leg_title",
            {
              kind: leg.kind,
              side: leg.selectedSide,
            },
          ),
          t(
            enforcedRoute
              ? "map.navigation_trace.local_detour_conservative"
              : "map.navigation_trace.search_position_only",
          ),
        ]);
        group.appendChild(createSvgElement("path", {
          d: `M ${bind.x} ${bind.y} L ${waypoint.x} ${waypoint.y}`,
          class: "map-planned-leg-line",
        }));
        group.appendChild(createSvgElement("circle", {
          cx: bind.x,
          cy: bind.y,
          r: 5,
          class: "map-planned-leg-bind",
        }));
        group.appendChild(createSvgElement("circle", {
          cx: waypoint.x,
          cy: waypoint.y,
          r: 9,
          class: "map-planned-waypoint",
        }));
        group.appendChild(createSvgElement("path", {
          d: headingArrowPath(waypoint, waypointHeading, 36),
          class: "map-planned-waypoint-heading",
        }));
        const label = createSvgElement("text", {
          x: waypoint.x + 13,
          y: waypoint.y + 20,
          class: "map-navigation-trace-label map-planned-waypoint-label",
        });
        label.textContent = t(
          enforcedRoute
            ? "map.navigation_trace.local_detour_waypoint_label"
            : "map.navigation_trace.planned_waypoint_label",
        );
        group.appendChild(label);
        layer.appendChild(group);
      }

      blastMapSemantics.renderAdvisoryWaypoint(
        layer, trace.advisoryWaypoint, map.robotPose,
        projection, blastRenderUi(),
      );

      blastMapSemantics.renderRoute(
        layer,
        trace.localDetourRoute,
        projection,
        blastRenderUi(),
      );

      trace.planarScanViews.forEach((view, viewIndex) => {
        const group = createSvgElement("g", {
          class: "map-blast-scan-view",
          "data-scan-id": view.scanId,
          "data-quality": view.quality,
          "data-vertical-pitch-compensated": "false",
          "data-ultrasonic-beam-width-modeled": "false",
          "data-scan-turn-translation-compensated": "false",
          "data-projection-frame": view.projectionFrame,
          "data-projection-local-frame": (
            view.projectionLocalFrame || ""
          ),
        });
        appendSvgTitle(group, [
          t("map.navigation_trace.scan_title", {
            count: formatNumber(viewIndex + 1),
          }),
          t("map.navigation_trace.scan_limitations"),
          t("map.tooltip.age", { age: formatMapAge(view.ageMs) }),
        ]);
        view.points.forEach((point) => {
          const origin = projection.point(
            point.sensorOriginX,
            point.sensorOriginY,
          );
          const echo = projection.point(
            point.nominalEchoX,
            point.nominalEchoY,
          );
          const rayGroup = createSvgElement("g", {
            class: "map-blast-scan-ray",
            "data-side": point.side,
            "data-measured-range-mm": point.measuredRangeMm,
            "data-relative-bearing-mdeg": point.relativeBearingMdeg,
            "data-beam-heading-mdeg": point.beamHeadingMdeg,
            "data-quality": "PROVISIONAL_YAW_ONLY",
          });
          appendSvgTitle(rayGroup, [
            t("map.navigation_trace.scan_ray_title", {
              side: point.side,
              range: formatNumber(point.measuredRangeMm),
            }),
            t("map.navigation_trace.scan_limitations"),
          ]);
          rayGroup.appendChild(createSvgElement("line", {
            x1: origin.x,
            y1: origin.y,
            x2: echo.x,
            y2: echo.y,
            class: "map-blast-scan-ray-line",
          }));
          rayGroup.appendChild(createSvgElement("circle", {
            cx: origin.x,
            cy: origin.y,
            r: 3,
            class: "map-blast-scan-origin",
          }));
          rayGroup.appendChild(createSvgElement("circle", {
            cx: echo.x,
            cy: echo.y,
            r: 6,
            class: "map-blast-scan-echo",
          }));
          group.appendChild(rayGroup);
        });
        const scanPose = projection.point(
          view.scanPose.xMm,
          view.scanPose.yMm,
        );
        const label = createSvgElement("text", {
          x: scanPose.x + 10,
          y: scanPose.y - 12,
          class: "map-navigation-trace-label map-blast-scan-label",
        });
        label.textContent = t("map.navigation_trace.scan_label", {
          count: formatNumber(viewIndex + 1),
        });
        group.appendChild(label);
        scanLayer.appendChild(group);
      });
    }

    function renderLocalOdometryMap(map, scene, drawable) {
      const layer = byId("map-local-odometry-layer");
      layer.replaceChildren();
      if (!drawable) {
        return;
      }
      layer.setAttribute("data-provisional", "true");
      layer.setAttribute("data-geometry", "screen-space-nonmetric");
      if (map.navigationTrace) {
        layer.setAttribute("data-geometry", "mixed-local-odometry");
      }
      const projection = fittedMetricProjection(scene.points);

      const layerLabel = createSvgElement("text", {
        x: 32,
        y: 42,
        class: "map-local-layer-label",
      });
      layerLabel.textContent = t(
        map.navigationTrace
          ? "map.navigation_trace.layer_label"
          : "map.local_odometry.layer_label",
      );
      layer.appendChild(layerLabel);
      const layerNote = createSvgElement("text", {
        x: 32,
        y: 65,
        class: "map-local-layer-note",
      });
      layerNote.textContent = t(
        map.navigationTrace
          ? "map.navigation_trace.layer_note"
          : "map.local_odometry.layer_note",
      );
      layer.appendChild(layerNote);

      renderPath(
        layer,
        map.poseHistory,
        projection,
        map.navigationTrace
          ? "map.navigation_trace.odometry_title"
          : "map.path.title",
      );
      renderNavigationTrace(layer, map, projection);
      blastMapSemantics.renderObstacles(
        layer,
        map.objectHypotheses,
        projection,
        blastRenderUi(),
      );

      scene.cues.forEach(({ anchorPose, evidence }) => {
        const anchor = projection.point(
          anchorPose.xMm,
          anchorPose.yMm,
        );
        const heading = (
          anchorPose.headingMdeg / 1000
        ) * Math.PI / 180;
        const halfAngle = Math.PI / 7;
        const left = screenPoint(anchor, heading + halfAngle, 64);
        const right = screenPoint(anchor, heading - halfAngle, 64);
        const tickStart = screenPoint(anchor, heading, 46);
        const tickEnd = screenPoint(anchor, heading, 68);
        let relationClass = "is-unknown";
        if (evidence.relation === "NEAR_OBSTACLE") {
          relationClass = "is-near";
        } else if (evidence.relation === "NO_NEAR_REFLECTION") {
          relationClass = "is-no-near-reflection";
        }
        const group = createSvgElement("g", {
          class: `map-local-ir-cue ${relationClass}`,
          "data-provisional": "true",
          "data-metric-distance": "none",
        });
        appendSvgTitle(group, [
          t("map.local_odometry.ir_title", {
            relation: relationLabel(evidence.relation),
          }),
          t("map.local_odometry.nonmetric"),
          ...mapTooltipParts(evidence),
        ]);
        group.appendChild(createSvgElement("path", {
          d: [
            `M ${anchor.x} ${anchor.y}`,
            `L ${left.x} ${left.y}`,
            `L ${right.x} ${right.y}`,
            "Z",
          ].join(" "),
          class: "map-local-ir-wedge",
        }));
        group.appendChild(createSvgElement("line", {
          x1: tickStart.x,
          y1: tickStart.y,
          x2: tickEnd.x,
          y2: tickEnd.y,
          class: "map-local-ir-tick",
        }));
        group.appendChild(createSvgElement("circle", {
          cx: anchor.x,
          cy: anchor.y,
          r: 7,
          class: "map-local-ir-anchor",
        }));
        const cueLabel = createSvgElement("text", {
          x: tickEnd.x + 10,
          y: tickEnd.y - 8,
          class: "map-local-ir-label",
        });
        cueLabel.textContent = t("map.local_odometry.ir_label");
        group.appendChild(cueLabel);
        layer.appendChild(group);
      });

      const scanCounts = retainedScanCounts(map);
      scene.latestScans.forEach((scan) => {
        const anchor = projection.point(
          scan.scanPose.xMm,
          scan.scanPose.yMm,
        );
        const baseHeading = (
          scan.scanPose.headingMdeg / 1000
        ) * Math.PI / 180;
        const attemptCount = scanCounts.get(
          scan.targetHypothesisId,
        ) || 1;
        const group = createSvgElement("g", {
          class: "map-local-scan",
          "data-provisional": "true",
          "data-geometry": "angular-nonmetric",
          "data-metric-distance": "none",
          "data-attempt-count": attemptCount,
          "data-scan-id": scan.scanId,
          "data-origin-x-mm": scan.scanPose.xMm,
          "data-origin-y-mm": scan.scanPose.yMm,
          "data-origin-heading-mdeg": scan.scanPose.headingMdeg,
          "data-based-on-map-version": scan.basedOnMapVersion,
        });
        appendSvgTitle(group, [
          t("map.scan.title", { count: formatNumber(attemptCount) }),
          t("map.scan.nonmetric"),
          t("map.scan.hypothesis", {
            id: scan.targetHypothesisId,
          }),
          Number.isFinite(scan.ageMs)
            ? t("map.tooltip.age", {
              age: formatMapAge(scan.ageMs),
            })
            : "",
        ]);
        scan.rays.forEach((ray) => {
          const rayHeading = baseHeading
            + ray.actualBearingMdeg / 1000 * Math.PI / 180;
          const end = screenPoint(anchor, rayHeading, 86);
          const state = ray.blocked
            ? t("map.scan.blocked")
            : t("map.scan.clear");
          const rayGroup = createSvgElement("g", {
            class: `map-local-scan-ray ${
              ray.blocked ? "is-blocked" : "is-clear"
            }`,
            "data-actual-bearing-mdeg": ray.actualBearingMdeg,
            "data-metric-distance": "none",
          });
          appendSvgTitle(rayGroup, [
            t("map.scan.ray_title", {
              bearing: scanBearingLabel(ray.actualBearingMdeg),
              state,
            }),
            t("map.scan.nonmetric"),
          ]);
          rayGroup.appendChild(createSvgElement("line", {
            x1: anchor.x,
            y1: anchor.y,
            x2: end.x,
            y2: end.y,
            class: "map-local-scan-ray-line",
          }));
          rayGroup.appendChild(createSvgElement("circle", {
            cx: end.x,
            cy: end.y,
            r: ray.blocked ? 5 : 3,
            class: "map-local-scan-ray-end",
          }));
          const rayLabel = createSvgElement("text", {
            x: end.x,
            y: end.y - 9,
            class: "map-local-scan-ray-label",
            "text-anchor": "middle",
          });
          rayLabel.textContent = scanBearingLabel(
            ray.actualBearingMdeg,
          );
          rayGroup.appendChild(rayLabel);
          group.appendChild(rayGroup);
        });
        group.appendChild(createSvgElement("circle", {
          cx: anchor.x,
          cy: anchor.y,
          r: 8,
          class: "map-local-scan-anchor",
        }));
        const scanLabelPosition = screenPoint(
          anchor,
          baseHeading,
          116,
        );
        const scanLabel = createSvgElement("text", {
          x: scanLabelPosition.x,
          y: scanLabelPosition.y + 18,
          class: "map-local-scan-label",
          "text-anchor": "middle",
        });
        scanLabel.textContent = t("map.scan.label", {
          count: formatNumber(attemptCount),
        });
        group.appendChild(scanLabel);
        layer.appendChild(group);
      });

      if (map.robotPose) {
        const robot = projection.point(
          map.robotPose.xMm,
          map.robotPose.yMm,
        );
        const heading = (
          map.robotPose.headingMdeg / 1000
        ) * Math.PI / 180;
        const headingEnd = screenPoint(robot, heading, 58);
        const group = createSvgElement("g", {
          class: "map-local-robot",
          "data-provisional": "true",
          "data-collision-geometry": (
            map.collisionGeometry?.geometry || "unavailable"
          ),
        });
        appendSvgTitle(group, [
          t("map.local_odometry.robot_title"),
          map.collisionGeometry
            ? t("map.footprint.no_contact_inference")
            : "",
          ...mapTooltipParts(map.robotPose),
        ]);
        if (
          map.collisionGeometry?.geometry
          === "ASYMMETRIC_RECTANGLE"
        ) {
          const outerCorners = footprintCorners(
            map.robotPose,
            map.collisionGeometry,
            true,
          ).map((point) => projection.point(point.xMm, point.yMm));
          const bodyCorners = footprintCorners(
            map.robotPose,
            map.collisionGeometry,
          ).map((point) => projection.point(point.xMm, point.yMm));
          group.appendChild(createSvgElement("polygon", {
            points: outerCorners
              .map((point) => `${point.x},${point.y}`)
              .join(" "),
            class: "map-local-robot-clearance",
            "data-clearance-margin-mm": (
              map.collisionGeometry.clearanceMarginMm
            ),
          }));
          const footprint = createSvgElement("polygon", {
            points: bodyCorners
              .map((point) => `${point.x},${point.y}`)
              .join(" "),
            class: "map-local-robot-footprint",
            "data-front-extent-mm": (
              map.collisionGeometry.frontExtentMm
            ),
            "data-rear-extent-mm": (
              map.collisionGeometry.rearExtentMm
            ),
            "data-left-extent-mm": (
              map.collisionGeometry.leftExtentMm
            ),
            "data-right-extent-mm": (
              map.collisionGeometry.rightExtentMm
            ),
          });
          appendSvgTitle(footprint, [
            t("map.footprint.asymmetric_title", {
              front: formatNumber(
                map.collisionGeometry.frontExtentMm,
              ),
              rear: formatNumber(
                map.collisionGeometry.rearExtentMm,
              ),
              left: formatNumber(
                map.collisionGeometry.leftExtentMm,
              ),
              right: formatNumber(
                map.collisionGeometry.rightExtentMm,
              ),
            }),
            t("map.footprint.no_contact_inference"),
          ]);
          group.appendChild(footprint);
        } else if (
          map.collisionGeometry?.geometry === "SYMMETRIC_CIRCLE"
        ) {
          const radiusPoint = bodyPointToWorld(
            map.robotPose,
            map.collisionGeometry.radiusMm,
            0,
          );
          const projectedRadius = projection.point(
            radiusPoint.xMm,
            radiusPoint.yMm,
          );
          group.appendChild(createSvgElement("circle", {
            cx: robot.x,
            cy: robot.y,
            r: Math.hypot(
              projectedRadius.x - robot.x,
              projectedRadius.y - robot.y,
            ),
            class: "map-local-robot-boundary",
          }));
        } else {
          group.appendChild(createSvgElement("circle", {
            cx: robot.x,
            cy: robot.y,
            r: 24,
            class: "map-local-robot-boundary",
          }));
        }
        group.appendChild(createSvgElement("line", {
          x1: robot.x,
          y1: robot.y,
          x2: headingEnd.x,
          y2: headingEnd.y,
          class: "map-local-robot-heading",
        }));
        const imuHeading = map.navigationTrace?.imuHeading;
        if (imuHeading) {
          const imuRadians = (
            imuHeading.headingMdeg / 1000 * Math.PI / 180
          );
          const imuLine = createSvgElement("path", {
            d: headingArrowPath(robot, imuRadians, 48),
            class: "map-local-imu-heading",
            "data-heading-mdeg": imuHeading.headingMdeg,
            "data-reference": imuHeading.reference,
          });
          appendSvgTitle(imuLine, [
            t("map.navigation_trace.imu_title", {
              heading: formatNumber(imuHeading.headingMdeg / 1000, {
                maximumFractionDigits: 1,
              }),
            }),
            t("map.navigation_trace.imu_reference"),
            t("map.tooltip.age", {
              age: formatMapAge(imuHeading.ageMs),
            }),
          ]);
          group.appendChild(imuLine);
        }
        group.appendChild(createSvgElement("circle", {
          cx: robot.x,
          cy: robot.y,
          r: 17,
          class: "map-local-robot-body",
        }));
        if (map.collisionGeometry) {
          const footprintLabel = createSvgElement("text", {
            x: robot.x,
            y: robot.y + 34,
            class: "map-local-footprint-label",
            "text-anchor": "middle",
          });
          footprintLabel.textContent = t("map.footprint.label");
          group.appendChild(footprintLabel);
        }
        layer.appendChild(group);
      }
    }

    function mapCellClass(cellState) {
      if (cellState === "FREE") {
        return "map-cell map-cell-free";
      }
      if (cellState === "OCCUPIED") {
        return "map-cell map-cell-occupied";
      }
      if (cellState === "UNCERTAIN") {
        return "map-cell map-cell-uncertain";
      }
      return "map-cell map-cell-unknown";
    }

    function renderPath(
      layer,
      poses,
      projection,
      titleKey = "map.path.title",
    ) {
      if (!Array.isArray(poses) || poses.length < 2) {
        return;
      }
      const points = poses.map((pose) => projection.point(
        pose.xMm,
        pose.yMm,
      ));
      const path = createSvgElement("path", {
        d: points.map((point, index) => (
          `${index === 0 ? "M" : "L"} ${point.x} ${point.y}`
        )).join(" "),
        class: "map-path",
      });
      appendSvgTitle(path, [
        t(titleKey),
        ...mapTooltipParts(poses[poses.length - 1]),
      ]);
      layer.appendChild(path);
      layer.appendChild(createSvgElement("circle", {
        cx: points[0].x,
        cy: points[0].y,
        r: 6,
        class: "map-path-start",
      }));
    }

    function statusLabel(status) {
      if (status === "available") {
        return t("map.status.live");
      }
      if (status === "degraded") {
        return t("map.status.degraded");
      }
      if (status === "qualitative_only") {
        return t("map.status.qualitative_only");
      }
      if (status === "pose_only") {
        return t("map.status.pose_only");
      }
      if (status === "unavailable") {
        return t("map.status.empty");
      }
      return safeText(status);
    }

    function renderMapMetadata(map) {
      const details = byId("map-metadata");
      const reasonKey = map.reasonCode
        ? `map.reason.${map.reasonCode}`
        : "";
      const reason = map.reasonCode
        ? localizedValue(reasonKey, map.reasonCode)
        : t("common.missing");
      const values = [
        [t("map.details.status"), statusLabel(map.status)],
        [t("map.details.reason"), reason],
        [t("map.details.source"), safeText(map.sourceId)],
        [t("map.details.age"), formatMapAge(map.ageMs)],
        [t("map.details.provenance"), safeText(map.provenance)],
        [
          t("map.details.version"),
          map.mapVersion === null
            ? t("common.missing")
            : formatNumber(map.mapVersion),
        ],
        [
          t("map.details.state_version"),
          map.basedOnStateVersion === null
            ? t("common.missing")
            : formatNumber(map.basedOnStateVersion),
        ],
        [
          t("map.details.world_version"),
          map.basedOnWorldModelVersion === null
            ? t("common.missing")
            : formatNumber(map.basedOnWorldModelVersion),
        ],
        [t("map.details.robot"), safeText(map.robotId)],
        [
          t("map.details.collision_geometry"),
          map.collisionGeometry
            ? localizedValue(
              `map.footprint.geometry.${
                map.collisionGeometry.geometry.toLocaleLowerCase(
                  "en-US",
                )
              }`,
              map.collisionGeometry.geometry,
            )
            : t("common.missing"),
        ],
        [
          t("map.details.scan_attempts"),
          map.scanEvidenceHistoryEvicted > 0
            ? t("map.details.scan_attempts_truncated", {
              count: formatNumber(map.scanEvidenceHistory.length),
              evicted: formatNumber(map.scanEvidenceHistoryEvicted),
              total: formatNumber(
                map.scanEvidenceHistory.length
                + map.scanEvidenceHistoryEvicted,
              ),
            })
            : formatNumber(map.scanEvidenceHistory.length),
        ],
        [
          t("map.details.scan_memory_retention"),
          map.scanAttemptRetention
            ? t("map.details.scan_memory_retention_value", {
              retained: formatNumber(
                map.scanAttemptRetention.retainedCount,
              ),
              mapCapacity: formatNumber(
                map.scanAttemptRetention.mapCapacity,
              ),
              perHazardCapacity: formatNumber(
                map.scanAttemptRetention.perHazardCapacity,
              ),
              evicted: formatNumber(
                map.scanAttemptRetention.evictedCount,
              ),
              reason: map.scanAttemptRetention.lastEvictionReason,
            })
            : t("common.missing"),
        ],
        [
          t("map.details.hazard_retention"),
          map.hazardRetention
            ? t("map.details.hazard_retention_value", {
              capacity: formatNumber(map.hazardRetention.capacity),
              evicted: formatNumber(map.hazardRetention.evictedCount),
              retained: formatNumber(map.hazardRetention.retainedCount),
            })
            : t("common.missing"),
        ],
        [
          t("map.details.path_points"),
          map.poseHistoryEvicted > 0
            ? t("map.details.path_points_truncated", {
              count: formatNumber(map.poseHistory.length),
              evicted: formatNumber(map.poseHistoryEvicted),
            })
            : formatNumber(map.poseHistory.length),
        ],
      ];
      details.replaceChildren(...values.map(([label, value]) => {
        const row = createElement("div");
        row.appendChild(createElement("dt", "", label));
        row.appendChild(createElement("dd", "", value));
        return row;
      }));
    }

    function renderMapObjects(map) {
      const list = byId("map-object-list");
      const scanCounts = retainedScanCounts(map);
      const entries = blastMapSemantics.objectEntries(
        map,
        SHARED_SPATIAL_MAP_SCHEMA,
      );
      byId("map-object-count").textContent = formatNumber(
        entries.length,
      );
      if (entries.length === 0) {
        list.replaceChildren(createElement(
          "p",
          "map-object-empty",
          t("map.objects.empty"),
        ));
        return;
      }
      list.replaceChildren(...entries.map((entry) => {
        const { hypothesis, sourceRobotId } = entry;
        const item = createElement("article", "map-object-item");
        const hypothesisScans = map.scanEvidenceHistory.filter(
          (scan) => (
            scan.targetHypothesisId === hypothesis.hypothesisId
          ),
        );
        const latestScan = hypothesisScans.length > 0
          ? hypothesisScans[hypothesisScans.length - 1]
          : null;
        item.appendChild(createElement(
          "strong",
          "",
          hypothesis.classification
            === "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER"
            ? t("map.objects.ultrasonic_obstacle_cluster")
            : safeText(
              hypothesis.label,
              safeText(
                hypothesis.hypothesisId,
                t("map.objects.unnamed"),
              ),
            ),
        ));
        if (
          hypothesis.classification
            === "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER"
        ) {
          item.appendChild(createElement(
            "span",
            "map-provenance-badge is-provisional-inference",
            t("map.objects.provisional_inference"),
          ));
        } else if (hypothesis.provenance) {
          item.appendChild(createElement(
            "span",
            "map-provenance-badge",
            hypothesis.provenance,
          ));
        } else {
          item.appendChild(createElement("span"));
        }
        const details = [
          hypothesis.classification
            === "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER"
            ? t(`map.objects.ultrasonic_relation.${
              hypothesis.relation.toLocaleLowerCase("en-US")
            }`)
            : "",
          hypothesis.classification
            === "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER"
            ? t("map.objects.ultrasonic_support", {
              count: formatNumber(hypothesis.evidenceCount),
              radius: formatNumber(hypothesis.supportRadiusMm),
            })
            : "",
          hypothesis.classification
            === "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER"
            ? t("map.objects.source_scans", {
              scans: hypothesis.sourceScanIds.join(", "),
            })
            : "",
          hypothesis.provisional && hypothesis.relation
            && hypothesis.classification
              !== "PROVISIONAL_ULTRASONIC_OBSTACLE_CLUSTER"
            ? relationLabel(hypothesis.relation)
            : "",
          hypothesis.provisional && hypothesis.bearing
            ? t("map.qualitative.bearing", {
              bearing: hypothesis.bearing,
            })
            : "",
          hypothesis.sourceId
            ? t("map.tooltip.source", {
              source: hypothesis.sourceId,
            })
            : "",
          sourceRobotId
            ? t("map.tooltip.source", { source: sourceRobotId })
            : "",
          Number.isFinite(hypothesis.ageMs)
            ? t("map.tooltip.age", {
              age: formatMapAge(hypothesis.ageMs),
            })
            : "",
          Number.isFinite(hypothesis.confidenceMilli)
            ? t("map.objects.confidence", {
              value: formatNumber(
                hypothesis.confidenceMilli / 10,
                { maximumFractionDigits: 1 },
              ),
            })
            : "",
          scanCounts.has(hypothesis.hypothesisId)
            ? t("map.scan.attempt_count", {
              count: formatNumber(
                scanCounts.get(hypothesis.hypothesisId),
              ),
            })
            : "",
          latestScan
            ? scanPatternLabel(latestScan.observationPattern)
            : "",
        ].filter(Boolean);
        item.appendChild(createElement(
          "small",
          "",
          details.length > 0
            ? details.join(" · ")
            : t("map.objects.no_metadata"),
        ));
        return item;
      }));
    }

    function relationLabel(relation) {
      const key = `map.qualitative.relation.${String(
        relation || "unknown",
      ).toLocaleLowerCase("en-US")}`;
      return localizedValue(
        key,
        safeText(relation, t("map.qualitative.relation.unknown")),
      );
    }

    function scanPatternLabel(pattern) {
      const key = `map.scan.pattern.${String(
        pattern || "unknown",
      ).toLocaleLowerCase("en-US")}`;
      return localizedValue(key, safeText(pattern));
    }

    function renderQualitativeObservations(map) {
      const observations = map.qualitativeObservations;
      const rendered = observations.slice(
        -MAX_RENDERED_QUALITATIVE_OBSERVATIONS,
      );
      const list = byId("map-qualitative-list");
      const count = byId("map-qualitative-count");
      const evicted = map.qualitativeObservationsEvicted;
      const total = observations.length + evicted;
      const retentionSummary = t("map.qualitative.retention", {
        shown: formatNumber(rendered.length),
        retained: formatNumber(observations.length),
        evicted: formatNumber(evicted),
        total: formatNumber(total),
      });
      count.textContent = (
        rendered.length < observations.length || evicted > 0
      )
        ? retentionSummary
        : formatNumber(observations.length);
      count.title = retentionSummary;
      if (observations.length === 0) {
        list.replaceChildren(createElement(
          "p",
          "map-qualitative-empty",
          t("map.qualitative.empty"),
        ));
        return;
      }
      list.replaceChildren(...rendered.map((observation) => {
        const item = createElement(
          "article",
          "map-qualitative-item",
        );
        item.appendChild(createElement(
          "strong",
          "",
          relationLabel(observation.relation),
        ));
        item.appendChild(createElement(
          "span",
          "map-provenance-badge",
          observation.provisional
            ? t("map.qualitative.provisional")
            : t("map.read_only"),
        ));
        const values = [
          [
            t("map.qualitative.raw"),
            Number.isFinite(observation.rawIrProximity)
              ? t("map.qualitative.raw_value", {
                value: formatNumber(observation.rawIrProximity),
              })
              : t("common.missing"),
          ],
          [
            t("map.qualitative.confidence"),
            Number.isFinite(observation.confidenceMilli)
              ? t("map.objects.confidence", {
                value: formatNumber(
                  observation.confidenceMilli / 10,
                  { maximumFractionDigits: 1 },
                ),
              })
              : t("common.missing"),
          ],
          [
            t("map.qualitative.age"),
            formatMapAge(observation.ageMs),
          ],
        ];
        const facts = createElement(
          "dl",
          "map-qualitative-facts",
        );
        values.forEach(([label, value]) => {
          const row = createElement("div");
          row.appendChild(createElement("dt", "", label));
          row.appendChild(createElement("dd", "", value));
          facts.appendChild(row);
        });
        item.appendChild(facts);
        const details = [
          observation.bearing
            ? t("map.qualitative.bearing", {
              bearing: observation.bearing,
            })
            : "",
          ...mapTooltipParts(observation),
        ].filter(Boolean);
        item.appendChild(createElement(
          "small",
          "",
          details.length > 0
            ? details.join(" · ")
            : t("map.objects.no_metadata"),
        ));
        return item;
      }));
    }

    function renderEmptyState(map) {
      const title = byId("map-empty-title");
      const body = byId("map-empty-body");
      if (
        map.status === "qualitative_only"
        || (
          map.bounds === null
          && map.qualitativeObservations.length > 0
        )
      ) {
        title.textContent = t("map.empty.qualitative_title");
        body.textContent = t("map.empty.qualitative_body");
      } else if (
        map.status === "pose_only"
        || (map.bounds === null && map.robotPose)
      ) {
        title.textContent = t("map.empty.pose_title");
        body.textContent = t("map.empty.pose_body");
      } else {
        title.textContent = t("map.empty.title");
        body.textContent = t("map.empty.body");
      }
    }

    function renderMetricMap(map, mapDrawable) {
      const cellLayer = byId("map-cell-layer");
      const pathLayer = byId("map-path-layer");
      const rayLayer = byId("map-ray-layer");
      const objectLayer = byId("map-object-layer");
      const robotLayer = byId("map-robot-layer");
      cellLayer.replaceChildren();
      pathLayer.replaceChildren();
      rayLayer.replaceChildren();
      objectLayer.replaceChildren();
      robotLayer.replaceChildren();
      if (!mapDrawable) {
        return;
      }
      const projection = mapProjection(map.bounds);
      renderPath(pathLayer, map.poseHistory, projection);
      map.cells.forEach((cell) => {
        const point = projection.point(cell.xMm, cell.yMm);
        const size = Number.isFinite(cell.sizeMm)
          ? Math.max(3, cell.sizeMm * projection.scale)
          : 8;
        const shape = createSvgElement("rect", {
          x: point.x - size / 2,
          y: point.y - size / 2,
          width: size,
          height: size,
          rx: 1,
          class: mapCellClass(cell.state),
        });
        appendSvgTitle(shape, [
          t(`map.cell.${cell.state.toLocaleLowerCase("en-US")}`),
          ...mapTooltipParts(cell),
        ]);
        cellLayer.appendChild(shape);
      });
      map.sensorRays.forEach((ray) => {
        const origin = projection.point(ray.originX, ray.originY);
        const end = projection.point(ray.endX, ray.endY);
        const line = createSvgElement("line", {
          x1: origin.x,
          y1: origin.y,
          x2: end.x,
          y2: end.y,
          class: "map-sensor-ray",
        });
        appendSvgTitle(line, [
          t("map.legend.sensor_ray"),
          ...mapTooltipParts(ray),
        ]);
        rayLayer.appendChild(line);
        rayLayer.appendChild(createSvgElement("circle", {
          cx: origin.x,
          cy: origin.y,
          r: 5,
          class: "map-sensor-origin",
        }));
      });
      map.objectHypotheses.forEach((hypothesis) => {
        if (
          !Number.isFinite(hypothesis.xMm)
          || !Number.isFinite(hypothesis.yMm)
        ) {
          return;
        }
        const point = projection.point(
          hypothesis.xMm,
          hypothesis.yMm,
        );
        const group = createSvgElement("g");
        appendSvgTitle(group, mapTooltipParts(hypothesis));
        group.appendChild(createSvgElement("circle", {
          cx: point.x,
          cy: point.y,
          r: 10,
          class: "map-object-marker",
        }));
        const label = createSvgElement("text", {
          x: point.x + 15,
          y: point.y - 13,
          class: "map-object-label",
        });
        label.textContent = safeText(
          hypothesis.label,
          safeText(
            hypothesis.hypothesisId,
            t("map.objects.unnamed"),
          ),
        ).slice(0, 36);
        group.appendChild(label);
        objectLayer.appendChild(group);
      });
      if (map.robotPose) {
        const robot = projection.point(
          map.robotPose.xMm,
          map.robotPose.yMm,
        );
        const headingRadians = (
          map.robotPose.headingMdeg / 1000
        ) * Math.PI / 180;
        const headingLength = 54;
        const headingX = robot.x
          + Math.cos(headingRadians) * headingLength;
        const headingY = robot.y
          - Math.sin(headingRadians) * headingLength;
        const group = createSvgElement("g");
        appendSvgTitle(group, [
          t("map.legend.robot"),
          ...mapTooltipParts(map.robotPose),
        ]);
        group.appendChild(createSvgElement("line", {
          x1: robot.x,
          y1: robot.y,
          x2: headingX,
          y2: headingY,
          class: "map-robot-heading",
        }));
        group.appendChild(createSvgElement("circle", {
          cx: robot.x,
          cy: robot.y,
          r: 17,
          class: "map-robot-body",
        }));
        robotLayer.appendChild(group);
      }
    }

    function renderSharedWorldMap(map, scene, drawable) {
      const cellLayer = byId("map-cell-layer");
      const pathLayer = byId("map-path-layer");
      const rayLayer = byId("map-ray-layer");
      const objectLayer = byId("map-object-layer");
      const robotLayer = byId("map-robot-layer");
      cellLayer.replaceChildren();
      pathLayer.replaceChildren();
      rayLayer.replaceChildren();
      objectLayer.replaceChildren();
      robotLayer.replaceChildren();
      if (!drawable) {
        return;
      }
      const projection = fittedMetricProjection(scene.points);
      map.robots.forEach((robot, index) => {
        if (robot.status !== "available" || !robot.robotPose) {
          return;
        }
        const robotClass = `map-shared-robot-${index % 8}`;
        const pathGroup = createSvgElement("g", {
          class: `map-shared-robot-path ${robotClass}`,
          "data-robot-id": robot.robotId,
          "data-controller-instance-id": robot.controllerInstanceId,
        });
        renderPath(pathGroup, robot.poseHistory, projection);
        pathLayer.appendChild(pathGroup);
        if (robot.navigationTrace) {
          const traceAttributes = {
            class: `map-shared-navigation-trace ${robotClass}`,
            "data-robot-id": robot.robotId,
            "data-controller-instance-id": robot.controllerInstanceId,
            "data-provisional": "true",
          };
          const tracePathGroup = createSvgElement("g", traceAttributes);
          const traceRayGroup = createSvgElement("g", traceAttributes);
          const traceAnchor = projection.point(
            robot.robotPose.xMm,
            robot.robotPose.yMm,
          );
          const traceLabel = createSvgElement("text", {
            x: traceAnchor.x + 22,
            y: traceAnchor.y + 35,
            class: (
              "map-navigation-trace-label "
              + "map-shared-navigation-trace-label"
            ),
          });
          traceLabel.textContent = `${robot.robotId} · ${t(
            "map.navigation_trace.layer_label",
          )}`;
          tracePathGroup.appendChild(traceLabel);
          renderNavigationTrace(
            tracePathGroup,
            { navigationTrace: robot.navigationTrace },
            projection,
            traceRayGroup,
          );
          if (tracePathGroup.children.length > 0) {
            pathLayer.appendChild(tracePathGroup);
          }
          if (traceRayGroup.children.length > 0) {
            rayLayer.appendChild(traceRayGroup);
          }
        }

        const pose = robot.robotPose;
        const robotPoint = projection.point(pose.xMm, pose.yMm);
        const headingRadians = pose.headingMdeg / 1000 * Math.PI / 180;
        const headingEnd = screenPoint(robotPoint, headingRadians, 54);
        const group = createSvgElement("g", {
          class: `map-shared-robot ${robotClass}`,
          "data-robot-id": robot.robotId,
          "data-controller-instance-id": robot.controllerInstanceId,
          "data-local-frame-id": robot.localFrameId,
          "data-local-generation-id": robot.localGenerationId,
          "data-world-frame-id": robot.worldFrameId,
          "data-world-generation-id": robot.worldGenerationId,
        });
        appendSvgTitle(group, [
          robot.robotId,
          robot.controllerInstanceId,
          ...mapTooltipParts(pose),
        ]);
        if (
          robot.collisionGeometry?.geometry === "ASYMMETRIC_RECTANGLE"
        ) {
          const corners = footprintCorners(
            pose,
            robot.collisionGeometry,
          ).map((point) => projection.point(point.xMm, point.yMm));
          group.appendChild(createSvgElement("polygon", {
            points: corners.map((point) => (
              `${point.x},${point.y}`
            )).join(" "),
            class: `map-local-robot-footprint map-shared-footprint ${
              robotClass
            }`,
            "data-front-extent-mm": (
              robot.collisionGeometry.frontExtentMm
            ),
            "data-rear-extent-mm": (
              robot.collisionGeometry.rearExtentMm
            ),
            "data-left-extent-mm": (
              robot.collisionGeometry.leftExtentMm
            ),
            "data-right-extent-mm": (
              robot.collisionGeometry.rightExtentMm
            ),
          }));
        } else if (
          robot.collisionGeometry?.geometry === "SYMMETRIC_CIRCLE"
        ) {
          const radiusPoint = projection.point(
            pose.xMm + robot.collisionGeometry.radiusMm,
            pose.yMm,
          );
          group.appendChild(createSvgElement("circle", {
            cx: robotPoint.x,
            cy: robotPoint.y,
            r: Math.abs(radiusPoint.x - robotPoint.x),
            class: `map-local-robot-boundary map-shared-footprint ${
              robotClass
            }`,
            "data-radius-mm": robot.collisionGeometry.radiusMm,
          }));
        }
        group.appendChild(createSvgElement("line", {
          x1: robotPoint.x,
          y1: robotPoint.y,
          x2: headingEnd.x,
          y2: headingEnd.y,
          class: `map-robot-heading map-shared-heading ${robotClass}`,
          "data-heading-mdeg": pose.headingMdeg,
        }));
        group.appendChild(createSvgElement("circle", {
          cx: robotPoint.x,
          cy: robotPoint.y,
          r: 17,
          class: `map-robot-body map-shared-body ${robotClass}`,
        }));
        const label = createSvgElement("text", {
          x: robotPoint.x,
          y: robotPoint.y - 25,
          class: `map-object-label map-shared-robot-label ${robotClass}`,
          "text-anchor": "middle",
        });
        label.textContent = robot.robotId;
        group.appendChild(label);
        robotLayer.appendChild(group);
      });
      blastMapSemantics.renderSharedObstacles(
        objectLayer,
        map.robots,
        projection,
        blastRenderUi(),
      );
    }

    function render(spatialMap, connection = "connected", nowUnixMs) {
      const map = normalizeSpatialMap(spatialMap, nowUnixMs);
      const status = byId("map-connection-status");
      const sharedMap = map.schema === SHARED_SPATIAL_MAP_SCHEMA;
      const sharedScene = sharedMap
        ? sharedWorldScene(map)
        : Object.freeze({ points: Object.freeze([]) });
      const sharedDrawable = (
        sharedMap
        && map.contractValid
        && (map.status === "available" || map.status === "degraded")
        && sharedScene.points.length > 0
      );
      const localOdometrySceneValue = sharedMap
        ? { points: [], cues: [], latestScans: [] }
        : localOdometryScene(map);
      const localOdometryDrawable = (
        map.contractValid
        && map.frameKind === LOCAL_ODOMETRY
        && (
          map.status === "pose_only"
          || map.status === "qualitative_only"
          || map.status === "degraded"
        )
        && map.bounds === null
        && localOdometrySceneValue.points.length > 0
      );
      const mapDrawable = (
        map.contractValid
        && (
          map.status === "available"
          || map.status === "degraded"
        )
        && map.bounds !== null
      );
      if (connection === "waiting") {
        status.className = "state-chip state-idle";
        status.textContent = t("map.status.waiting");
      } else if (connection === "offline") {
        status.className = "state-chip state-fault";
        status.textContent = t("map.status.offline");
      } else if (!map.contractValid) {
        status.className = "state-chip state-fault";
        status.textContent = t("map.status.invalid");
      } else if (map.status === "degraded") {
        status.className = "state-chip state-locked";
        status.textContent = t("map.status.degraded");
      } else if (mapDrawable || sharedDrawable) {
        status.className = "state-chip state-ready";
        status.textContent = t("map.status.live");
      } else if (map.status === "qualitative_only") {
        status.className = "state-chip state-ready";
        status.textContent = t("map.status.qualitative_only");
      } else if (map.status === "pose_only") {
        status.className = "state-chip state-idle";
        status.textContent = t("map.status.pose_only");
      } else {
        status.className = "state-chip state-idle";
        status.textContent = t("map.status.empty");
      }
      byId("map-frame-label").textContent = safeText(
        map.frameId,
        t("map.frame.unavailable"),
      );
      byId("map-empty-state").hidden = (
        mapDrawable || localOdometryDrawable || sharedDrawable
      );
      renderEmptyState(map);
      renderMapMetadata(map);
      renderQualitativeObservations(map);
      renderMapObjects(map);
      if (sharedMap) {
        renderSharedWorldMap(map, sharedScene, sharedDrawable);
      } else {
        renderMetricMap(map, mapDrawable);
      }
      renderLocalOdometryMap(
        map,
        localOdometrySceneValue,
        localOdometryDrawable,
      );
      return map;
    }

    return Object.freeze({ render });
  }

  global.RobotSpatialMapPresenter = Object.freeze({
    MAX_RENDERED_QUALITATIVE_OBSERVATIONS,
    create,
  });
})(typeof window === "undefined" ? globalThis : window);
