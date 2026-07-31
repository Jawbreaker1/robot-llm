((global) => {
  "use strict";

  const SVG_NAMESPACE = "http://www.w3.org/2000/svg";
  const LOCAL_ODOMETRY = "LOCAL_ODOMETRY";
  const QUALITATIVE_FORWARD_ENVELOPE = (
    "QUALITATIVE_FORWARD_ENVELOPE"
  );
  const SVG_WIDTH = 1000;
  const SVG_HEIGHT = 620;

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
      map.objectHypotheses.forEach((hypothesis) => {
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
      return { points, cues };
    }

    function localOdometryProjection(points) {
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

    function renderLocalOdometryMap(map, scene, drawable) {
      const layer = byId("map-local-odometry-layer");
      layer.replaceChildren();
      if (!drawable) {
        return;
      }
      layer.setAttribute("data-provisional", "true");
      layer.setAttribute("data-geometry", "screen-space-nonmetric");
      const projection = localOdometryProjection(scene.points);

      const layerLabel = createSvgElement("text", {
        x: 32,
        y: 42,
        class: "map-local-layer-label",
      });
      layerLabel.textContent = t("map.local_odometry.layer_label");
      layer.appendChild(layerLabel);
      const layerNote = createSvgElement("text", {
        x: 32,
        y: 65,
        class: "map-local-layer-note",
      });
      layerNote.textContent = t("map.local_odometry.layer_note");
      layer.appendChild(layerNote);

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
        });
        appendSvgTitle(group, [
          t("map.local_odometry.robot_title"),
          ...mapTooltipParts(map.robotPose),
        ]);
        group.appendChild(createSvgElement("circle", {
          cx: robot.x,
          cy: robot.y,
          r: 24,
          class: "map-local-robot-boundary",
        }));
        group.appendChild(createSvgElement("line", {
          x1: robot.x,
          y1: robot.y,
          x2: headingEnd.x,
          y2: headingEnd.y,
          class: "map-local-robot-heading",
        }));
        group.appendChild(createSvgElement("circle", {
          cx: robot.x,
          cy: robot.y,
          r: 17,
          class: "map-local-robot-body",
        }));
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
      byId("map-object-count").textContent = formatNumber(
        map.objectHypotheses.length,
      );
      if (map.objectHypotheses.length === 0) {
        list.replaceChildren(createElement(
          "p",
          "map-object-empty",
          t("map.objects.empty"),
        ));
        return;
      }
      list.replaceChildren(...map.objectHypotheses.map((hypothesis) => {
        const item = createElement("article", "map-object-item");
        item.appendChild(createElement(
          "strong",
          "",
          safeText(
            hypothesis.label,
            safeText(
              hypothesis.hypothesisId,
              t("map.objects.unnamed"),
            ),
          ),
        ));
        if (hypothesis.provenance) {
          item.appendChild(createElement(
            "span",
            "map-provenance-badge",
            hypothesis.provenance,
          ));
        } else {
          item.appendChild(createElement("span"));
        }
        const details = [
          hypothesis.provisional && hypothesis.relation
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

    function renderQualitativeObservations(map) {
      const observations = map.qualitativeObservations;
      const list = byId("map-qualitative-list");
      byId("map-qualitative-count").textContent = formatNumber(
        observations.length,
      );
      if (observations.length === 0) {
        list.replaceChildren(createElement(
          "p",
          "map-qualitative-empty",
          t("map.qualitative.empty"),
        ));
        return;
      }
      list.replaceChildren(...observations.map((observation) => {
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
      const rayLayer = byId("map-ray-layer");
      const objectLayer = byId("map-object-layer");
      const robotLayer = byId("map-robot-layer");
      cellLayer.replaceChildren();
      rayLayer.replaceChildren();
      objectLayer.replaceChildren();
      robotLayer.replaceChildren();
      if (!mapDrawable) {
        return;
      }
      const projection = mapProjection(map.bounds);
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

    function render(spatialMap, connection = "connected", nowUnixMs) {
      const map = normalizeSpatialMap(spatialMap, nowUnixMs);
      const status = byId("map-connection-status");
      const localOdometrySceneValue = localOdometryScene(map);
      const localOdometryDrawable = (
        map.contractValid
        && map.frameKind === LOCAL_ODOMETRY
        && (
          map.status === "pose_only"
          || map.status === "qualitative_only"
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
      } else if (mapDrawable) {
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
        mapDrawable || localOdometryDrawable
      );
      renderEmptyState(map);
      renderMapMetadata(map);
      renderQualitativeObservations(map);
      renderMapObjects(map);
      renderMetricMap(map, mapDrawable);
      renderLocalOdometryMap(
        map,
        localOdometrySceneValue,
        localOdometryDrawable,
      );
      return map;
    }

    return Object.freeze({ render });
  }

  global.RobotSpatialMapPresenter = Object.freeze({ create });
})(typeof window === "undefined" ? globalThis : window);
