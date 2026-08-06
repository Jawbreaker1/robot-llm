((global) => {
  "use strict";

  const BLAST_CONTROLLER_ID = "blast-01.hub";
  const BLAST_COMMAND_GROUPS = Object.freeze([
    ["drive_forward", "drive_reverse"],
    ["turn_left", "turn_right"],
    ["claw_open", "claw_close"],
    ["body_left", "body_right"],
    ["stop"],
  ]);

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
    const request = options.request;
    const showToast = options.showToast;
    const onCommandComplete = options.onCommandComplete;
    if (
      !document
      || typeof t !== "function"
      || typeof humanState !== "function"
      || typeof setStatus !== "function"
      || typeof formatDateTime !== "function"
      || typeof formatNumber !== "function"
      || typeof request !== "function"
      || typeof showToast !== "function"
      || typeof onCommandComplete !== "function"
    ) {
      throw new TypeError("Controller panel options are invalid.");
    }
    const pendingControllers = new Set();
    const pendingStops = new Set();
    let lastRender = null;

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

    function rerender() {
      if (lastRender) {
        render(
          lastRender.container,
          lastRender.nodes,
          lastRender.runtimeControllers,
        );
      }
    }

    async function sendCommand(controllerId, command) {
      const isStop = command === "stop";
      if (
        (isStop && pendingStops.has(controllerId))
        || (!isStop && (
          pendingControllers.has(controllerId)
          || pendingStops.has(controllerId)
        ))
      ) {
        return;
      }
      (isStop ? pendingStops : pendingControllers).add(controllerId);
      rerender();
      try {
        await request(
          `/api/v1/controllers/${encodeURIComponent(controllerId)}/commands`,
          {
            method: "POST",
            body: { command },
            timeout: 16000,
          },
        );
        showToast(t("registry.control.completed", {
          command: t(`registry.control.${command}`),
        }));
      } catch (error) {
        if (error && error.code === "controller_command_interrupted") {
          showToast(t("registry.control.interrupted"));
        } else {
          showToast(t("registry.control.failed"), true);
        }
      } finally {
        (isStop ? pendingStops : pendingControllers).delete(controllerId);
        await onCommandComplete();
        rerender();
      }
    }

    function renderControls(card, controller, runtime, observation) {
      if (controller.controller_id !== BLAST_CONTROLLER_ID) {
        return;
      }
      const online = runtime.state === "online";
      const observed = Number.isFinite(runtime.last_observed_at_unix_ms);
      const moving = observation.motion_active === true;
      const pending = pendingControllers.has(controller.controller_id);
      const stopPending = pendingStops.has(controller.controller_id);
      const enabled = online && observed && !moving && !pending && !stopPending;
      const controls = element("section", "controller-controls");
      controls.appendChild(element(
        "strong",
        "controller-controls-title",
        t("registry.control.title"),
      ));
      BLAST_COMMAND_GROUPS.forEach((commands) => {
        const group = element("div", "controller-control-group");
        commands.forEach((command) => {
          const button = element(
            "button",
            command === "stop"
              ? "button button-danger"
              : "button button-secondary",
            t(`registry.control.${command}`),
          );
          button.type = "button";
          button.disabled = command === "stop"
            ? !(online && observed && !stopPending)
            : !enabled;
          button.addEventListener("click", () => {
            sendCommand(controller.controller_id, command);
          });
          group.appendChild(button);
        });
        controls.appendChild(group);
      });
      controls.appendChild(element(
        "small",
        "controller-control-status",
        pending || stopPending
          ? t("registry.control.running")
          : !online
            ? t("registry.control.offline")
            : moving
              ? t("registry.control.moving")
              : !observed
                ? t("registry.control.waiting")
                : t("registry.control.ready"),
      ));
      card.appendChild(controls);
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
        text(
          runtime.reason_code,
          Number.isFinite(runtime.last_observed_at_unix_ms)
            ? t("common.missing")
            : text(controller.status_reason_code),
        ),
      );
      card.appendChild(telemetry);
      renderControls(card, controller, runtime, observation);
      return card;
    }

    function render(container, nodes, runtimeControllers) {
      lastRender = { container, nodes, runtimeControllers };
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
