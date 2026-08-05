((global) => {
  "use strict";

  function record(value) {
    return value && typeof value === "object" && !Array.isArray(value)
      ? value
      : {};
  }

  function text(value, fallback = "") {
    return typeof value === "string" && value.length > 0
      ? value
      : fallback;
  }

  function integer(value) {
    return Number.isSafeInteger(value) ? value : null;
  }

  function create(options = {}) {
    const document = options.document;
    const t = options.translate;
    const humanState = options.humanState;
    const setStatus = options.setStatus;
    const formatDateTime = options.formatDateTime;
    const formatNumber = options.formatNumber;
    if (
      !document
      || typeof t !== "function"
      || typeof humanState !== "function"
      || typeof setStatus !== "function"
      || typeof formatDateTime !== "function"
      || typeof formatNumber !== "function"
    ) {
      throw new TypeError("Controller panel options are invalid.");
    }

    function element(tag, className, content) {
      const node = document.createElement(tag);
      if (className) {
        node.className = className;
      }
      if (content !== undefined && content !== null) {
        node.textContent = String(content);
      }
      return node;
    }

    function row(telemetry, label, value) {
      const item = element("div");
      item.appendChild(element("dt", "", label));
      item.appendChild(element("dd", "", value));
      telemetry.appendChild(item);
    }

    function renderCard(controller, runtime) {
      const observation = record(runtime.observation);
      const battery = record(observation.battery);
      const imu = record(observation.imu);
      const card = element("article", "controller-card");
      const header = element("header", "controller-card-header");
      const title = element("div");
      title.appendChild(element(
        "strong",
        "",
        text(runtime.display_name, text(controller.display_name)),
      ));
      title.appendChild(element("small", "", text(controller.controller_id)));
      header.appendChild(title);
      const runtimeState = text(runtime.state, controller.lifecycle);
      header.appendChild(element(
        "span",
        `state-chip ${runtimeState === "online" ? "state-ready" : "state-idle"}`,
        humanState(runtimeState),
      ));
      card.appendChild(header);

      const telemetry = element("dl", "telemetry-list");
      const voltageMv = integer(battery.voltage_mv);
      const currentMa = integer(battery.current_ma);
      const batteryValue = voltageMv === null
        ? t("common.missing")
        : t("registry.telemetry.battery_value", {
          voltage: formatNumber(voltageMv / 1000, { maximumFractionDigits: 2 }),
          current: currentMa === null ? t("common.missing") : currentMa,
        });
      const heading = Number.isFinite(imu.heading_deg)
        ? t("registry.telemetry.heading_value", {
          value: formatNumber(imu.heading_deg, { maximumFractionDigits: 1 }),
        })
        : t("common.missing");
      const motors = record(observation.motor_angles_deg);
      const motorValue = Object.entries(motors).map(([name, angle]) => (
        `${name}: ${Number.isFinite(angle) ? Math.round(angle) : "—"}°`
      )).join(" · ") || t("common.missing");

      row(
        telemetry,
        t("registry.field.last_observed"),
        Number.isFinite(runtime.last_observed_at_unix_ms)
          ? formatDateTime(runtime.last_observed_at_unix_ms)
          : t("common.missing"),
      );
      row(telemetry, t("registry.telemetry.battery"), batteryValue);
      row(
        telemetry,
        t("registry.telemetry.distance"),
        Number.isFinite(observation.distance_mm)
          ? `${Math.round(observation.distance_mm)} mm`
          : t("common.missing"),
      );
      row(
        telemetry,
        t("registry.telemetry.color"),
        text(observation.color, t("common.missing")).replace("Color.", ""),
      );
      row(telemetry, t("registry.telemetry.heading"), heading);
      row(
        telemetry,
        t("registry.telemetry.motion"),
        typeof observation.motion_active === "boolean"
          ? humanState(observation.motion_active ? "active" : "inactive")
          : t("common.missing"),
      );
      row(telemetry, t("registry.telemetry.motors"), motorValue);
      row(
        telemetry,
        t("registry.field.status_reason"),
        text(runtime.reason_code, text(controller.status_reason_code)),
      );
      card.appendChild(telemetry);
      return card;
    }

    function render(container, nodes, runtimeControllers) {
      const runtimes = new Map(
        (Array.isArray(runtimeControllers) ? runtimeControllers : []).map(
          (runtime) => [runtime.controller_id, record(runtime)],
        ),
      );
      container.replaceChildren();
      (Array.isArray(nodes) ? nodes : [])
        .filter((node) => (
          node.node_kind === "controller"
          && (
            node.status_reason_code !== "future_component"
            || runtimes.has(node.controller_id)
          )
        ))
        .forEach((controller) => {
          container.appendChild(renderCard(
            controller,
            runtimes.get(controller.controller_id) || {},
          ));
        });
    }

    function visibleRobots(robots, nodes, runtimeControllers) {
      const runtimeByController = runtimeIndex(runtimeControllers);
      const visibleRobotIds = new Set(
        (Array.isArray(nodes) ? nodes : [])
          .filter((node) => (
            node.node_kind === "controller"
            && (
              node.status_reason_code !== "future_component"
              || runtimeByController.has(node.controller_id)
            )
          ))
          .map((node) => node.robot_id)
          .filter(Boolean),
      );
      return (Array.isArray(robots) ? robots : []).filter(
        (robot) => visibleRobotIds.has(robot.robot_id),
      );
    }

    function runtimes(value) {
      return Array.isArray(value) ? value.map(record) : [];
    }

    function runtimeIndex(value) {
      return new Map(
        runtimes(value).map((runtime) => [runtime.controller_id, runtime]),
      );
    }

    function stateForRobot(robot, runtimeControllers) {
      const runtime = runtimes(runtimeControllers).find(
        (item) => item.robot_id === robot.robot_id,
      );
      return text(runtime && runtime.state, text(robot.lifecycle, "configured"));
    }

    function nodeWithRuntimeState(node, runtimeByController) {
      const runtime = runtimeByController.get(node.controller_id);
      return runtime ? { ...node, lifecycle: runtime.state } : node;
    }

    function renderBlastStatus(runtimeControllers) {
      const blast = runtimes(runtimeControllers).find(
        (controller) => controller.controller_id === "blast-01.hub",
      ) || {};
      const state = text(blast.state, "unobserved");
      setStatus(
        "status-blast",
        state === "online"
          ? "online"
          : state === "connecting"
            ? "busy"
            : state === "offline"
              ? "offline"
              : "idle",
        humanState(state),
      );
    }

    return Object.freeze({
      nodeWithRuntimeState,
      render,
      renderBlastStatus,
      runtimeIndex,
      runtimes,
      stateForRobot,
      visibleRobots,
    });
  }

  global.RobotControllerPanel = Object.freeze({ create });
})(typeof window === "undefined" ? globalThis : window);
