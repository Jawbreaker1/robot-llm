(() => {
  "use strict";
  const TOKEN_META = document.querySelector('meta[name="robot-dashboard-token"]');
  const SESSION_TOKEN = TOKEN_META ? TOKEN_META.content : "";
  const TERMINAL_TURN_STATES = new Set([
    "answered",
    "clarification_required",
    "failed",
  ]);
  const {
    createDashboardRequest,
    createSessionGuard,
    normalizeSpatialMap,
    TURN_POLL_POLICY,
    replaceRenderedItems,
    transitionTurnPoll,
  } = window.RobotDashboardLogic;
  const EVENT_LIMIT = 100;
  const MAX_LOCAL_EVENTS = 2000;
  const MAX_ROBOT_DIALOGUE_MESSAGES = 40;
  const MAP_POLL_INTERVAL_MS = 2000;
  let microphoneInput = null;
  let robotControl = null;
  const i18n = window.RobotI18n.createDefaultI18n();
  const t = (key, args) => i18n.t(key, args);
  const ERROR_MESSAGE_KEYS = Object.freeze({
    dashboard_session_missing: "errors.dashboard_session_missing",
    invalid_server_json: "errors.invalid_server_json",
    request_timeout: "errors.request_timeout",
    network_error: "errors.network_error",
    host_rejected: "errors.origin_rejected",
    origin_rejected: "errors.origin_rejected",
    session_token_rejected: "errors.session_token_rejected",
    invalid_headers: "errors.invalid_request",
    invalid_query: "errors.invalid_request",
    invalid_json: "errors.invalid_request",
    invalid_request: "errors.invalid_request",
    invalid_request_fields: "errors.invalid_request",
    invalid_target: "errors.invalid_request",
    content_encoding_rejected: "errors.invalid_request",
    content_type_rejected: "errors.invalid_request",
    transfer_encoding_rejected: "errors.invalid_request",
    request_too_large: "errors.invalid_request",
    method_not_allowed: "errors.invalid_request",
    route_not_found: "errors.invalid_request",
    conversation_not_found: "errors.conversation_not_found",
    turn_not_found: "errors.turn_not_found",
    settings_revision_conflict: "errors.settings_revision_conflict",
    conversation_version_conflict: "errors.conversation_version_conflict",
    duplicate_client_request: "errors.duplicate_client_request",
    idempotency_conflict: "errors.idempotency_conflict",
    chat_queue_full: "errors.chat_queue_full",
    service_stopping: "errors.service_stopping",
    conversation_turn_active: "errors.conversation_turn_active",
    invalid_response_locale: "errors.invalid_response_locale",
    invalid_chat_mode: "errors.invalid_chat_mode",
    runtime_unavailable: "errors.runtime_unavailable",
    lm_studio_unreachable: "errors.runtime_unavailable",
    model_not_ready: "errors.model_not_ready",
    spatial_map_unavailable: "errors.spatial_map_unavailable",
    robot_control_disabled: "errors.robot_control_disabled",
    robot_not_idle: "errors.robot_not_idle",
    robot_episode_active: "errors.robot_episode_active",
    robot_settings_revision_conflict: "errors.robot_settings_revision_conflict",
    robot_idempotency_conflict: "errors.robot_idempotency_conflict",
    robot_service_stopping: "errors.robot_service_stopping",
    robot_input_disabled: "errors.robot_input_disabled",
    robot_input_idempotency_conflict: "errors.robot_input_idempotency_conflict",
    robot_input_inflight: "errors.robot_input_inflight",
    invalid_robot_input: "errors.invalid_robot_request",
    invalid_robot_locale: "errors.invalid_robot_request",
    invalid_robot_text: "errors.invalid_robot_request",
    invalid_robot_identifier: "errors.invalid_robot_request",
    invalid_robot_integer: "errors.invalid_robot_request",
    invalid_robot_settings: "errors.invalid_robot_request",
    invalid_robot_settings_fields: "errors.invalid_robot_request",
    invalid_robot_request: "errors.invalid_robot_request",
    invalid_robot_request_fields: "errors.invalid_robot_request",
    robot_request_too_large: "errors.invalid_robot_request",
    invalid_robot_query: "errors.invalid_robot_request",
    robot_emergency_stop_failed: "errors.robot_emergency_stop_failed",
  });

  const state = {
    bootstrap: null,
    settings: null,
    originalSettings: null,
    registry: null,
    spatialMap: null,
    mapBusy: false,
    mapConnection: "waiting",
    experiments: [],
    conversation: null,
    activeTurn: null,
    optimisticContent: null,
    robotDialogue: [],
    robotOptimisticContent: null,
    events: [],
    eventIds: new Set(),
    afterSequence: 0,
    eventsPaused: false,
    selectedEvent: null,
    workbenchReadOnlyInvariant: true,
    turnPollGeneration: 0,
    turnPollFailures: 0,
    turnPollConnection: "connected",
    bootstrapBusy: false,
    settingsDirty: false,
    modelReady: null,
    lmProbe: {
      phase: "idle",
      result: null,
      errorCode: null,
    },
    workbenchViolationAnnounced: false,
    eventStreamState: "live",
    eventGapActive: false,
    eventGapDroppedTotal: 0,
  };

  const byId = (id) => document.getElementById(id);
  const sessionGuard = createSessionGuard();
  sessionGuard.subscribe(() => {
    byId("session-expired-notice").hidden = false;
    if (microphoneInput) {
      microphoneInput.cancel();
    }
  });
  const api = createDashboardRequest({
    sessionToken: SESSION_TOKEN,
    sessionGuard,
  });
  const welcomeMessage = byId("welcome-message");
  const safeArray = (value) => (Array.isArray(value) ? value : []);
  const safeObject = (value) => (
    value && typeof value === "object" && !Array.isArray(value) ? value : {}
  );
  const safeText = (value, fallback = t("common.missing")) => (
    typeof value === "string" && value.length > 0 ? value : fallback
  );
  const safeInteger = (value, fallback = null) => (
    Number.isSafeInteger(value) ? value : fallback
  );

  function createElement(tag, className, text) {
    const node = document.createElement(tag);
    if (className) {
      node.className = className;
    }
    if (text !== undefined && text !== null) {
      node.textContent = String(text);
    }
    return node;
  }

  function cloneJSON(value) {
    return JSON.parse(JSON.stringify(value));
  }

  function randomId(prefix) {
    if (window.crypto && typeof window.crypto.randomUUID === "function") {
      return `${prefix}-${window.crypto.randomUUID()}`;
    }
    const bytes = new Uint8Array(16);
    window.crypto.getRandomValues(bytes);
    const suffix = Array.from(bytes, (value) => value.toString(16).padStart(2, "0")).join("");
    return `${prefix}-${suffix}`;
  }

  function formatTime(unixMs) {
    return i18n.time(unixMs, {
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
      fractionalSecondDigits: 3,
    });
  }

  function formatDateTime(unixMs) {
    return i18n.dateTime(unixMs, {
      dateStyle: "medium",
      timeStyle: "medium",
    });
  }

  function humanState(value) {
    const key = `state.${value}`;
    const translated = t(key);
    return translated === key ? safeText(value, t("state.unknown")) : translated;
  }

  function localizedError(error, fallbackKey = "errors.generic") {
    const code = safeText(error && error.code, "");
    const key = ERROR_MESSAGE_KEYS[code] || fallbackKey;
    return t(key);
  }

  function localizedCatalogValue(key, fallback) {
    if (typeof key !== "string" || key.length === 0) {
      return fallback;
    }
    const translated = t(key);
    return translated === key ? fallback : translated;
  }

  const spatialMapPresenter = window.RobotSpatialMapPresenter.create({
    document,
    normalizeSpatialMap,
    translate: t,
    formatNumber: (value, options) => i18n.number(value, options),
  });

  function showToast(message, isError = false) {
    const region = byId("toast-region");
    const toast = createElement("div", isError ? "toast is-error" : "toast", message);
    region.appendChild(toast);
    window.setTimeout(() => toast.remove(), 4500);
  }

  function setStatus(id, status, value) {
    const node = byId(id);
    if (!node) {
      return;
    }
    node.dataset.status = status;
    const valueNode = node.querySelector(".status-value");
    if (valueNode) {
      valueNode.textContent = value;
    }
  }

  function applyStaticTranslations() {
    document.documentElement.lang = i18n.locale;
    document.documentElement.dir = i18n.direction;
    const bindings = [
      ["data-i18n", null],
      ["data-i18n-aria-label", "aria-label"],
      ["data-i18n-title", "title"],
      ["data-i18n-placeholder", "placeholder"],
      ["data-i18n-alt", "alt"],
    ];
    const roots = [document];
    if (welcomeMessage && !welcomeMessage.isConnected) {
      roots.push(welcomeMessage);
    }
    bindings.forEach(([dataAttribute, targetAttribute]) => {
      roots.forEach((root) => root.querySelectorAll(`[${dataAttribute}]`).forEach((node) => {
        const key = node.getAttribute(dataAttribute);
        if (!key) {
          return;
        }
        if (targetAttribute) {
          node.setAttribute(targetAttribute, t(key));
        } else {
          node.textContent = t(key);
        }
      }));
    });
    roots.forEach((root) => root.querySelectorAll("[data-i18n-prompt]").forEach((node) => {
      const key = node.getAttribute("data-i18n-prompt");
      if (key) {
        node.dataset.prompt = t(key);
      }
    }));
    const selector = byId("ui-language");
    if (selector) {
      selector.value = i18n.locale;
    }
  }

  function activateView(viewName, focusHeading = true) {
    document.querySelectorAll("[data-view-panel]").forEach((panel) => {
      const active = panel.dataset.viewPanel === viewName;
      panel.hidden = !active;
      panel.classList.toggle("is-active", active);
    });
    document.querySelectorAll("[data-view]").forEach((button) => {
      const active = button.dataset.view === viewName;
      button.classList.toggle("is-active", active);
      if (active) {
        button.setAttribute("aria-current", "page");
      } else {
        button.removeAttribute("aria-current");
      }
    });
    if (
      robotControl
      && (viewName === "robot" || viewName === "workbench")
    ) {
      const previousTarget = robotControl.selectedTarget();
      const nextTarget = robotControl.selectConversationView(viewName);
      if (microphoneInput && previousTarget !== nextTarget) {
        microphoneInput.cancel();
      }
    }
    if (focusHeading) {
      const panel = document.querySelector(`[data-view-panel="${viewName}"]`);
      const heading = panel ? panel.querySelector("h2") : null;
      if (heading) {
        heading.setAttribute("tabindex", "-1");
        heading.focus({ preventScroll: true });
      }
    }
  }

  function activateInspectorTab(tabName, moveFocus = false) {
    document.querySelectorAll("[data-inspector-tab]").forEach((tab) => {
      const active = tab.dataset.inspectorTab === tabName;
      tab.setAttribute("aria-selected", active ? "true" : "false");
      tab.tabIndex = active ? 0 : -1;
      if (active && moveFocus) {
        tab.focus();
      }
    });
    document.querySelectorAll("[data-inspector-panel]").forEach((panel) => {
      const active = panel.dataset.inspectorPanel === tabName;
      panel.hidden = !active;
      panel.classList.toggle("is-active", active);
    });
  }

  function openInspector() {
    const inspector = byId("agent-inspector");
    inspector.classList.add("is-mobile-visible");
    byId("open-inspector-button").setAttribute("aria-expanded", "true");
    byId("close-inspector-button").focus();
  }

  function closeInspector() {
    const inspector = byId("agent-inspector");
    inspector.classList.remove("is-mobile-visible");
    byId("open-inspector-button").setAttribute("aria-expanded", "false");
    byId("open-inspector-button").focus();
  }

  function renderRuntime(runtime) {
    const runtimeObject = safeObject(runtime);
    const lmStudio = safeObject(runtimeObject.lm_studio);
    const lmState = safeText(lmStudio.state, "unknown");
    const modelLoaded = lmStudio.configured_model_loaded;
    state.modelReady = lmState === "unknown"
      ? null
      : lmState === "online" && modelLoaded === true;
    const lmStatus = lmState === "online" && modelLoaded === false
      ? "warning"
      : lmState === "online"
        ? "online"
        : lmState === "offline"
          ? "offline"
          : "unknown";
    const lmLabel = lmState === "online" && modelLoaded === false
      ? t("runtime.model_not_loaded")
      : lmState === "online"
        ? safeText(lmStudio.model, t("runtime.connected"))
        : humanState(lmState);
    setStatus(
      "status-lm-studio",
      lmStatus,
      lmLabel,
    );
    byId("setting-lm-base-url").value = safeText(lmStudio.base_url, "");
    byId("setting-lm-model").value = safeText(lmStudio.model, "");

    const ev3 = safeObject(runtimeObject.ev3);
    const ev3State = safeText(ev3.state, "unobserved");
    setStatus(
      "status-ev3",
      ev3State === "online" ? "online" : ev3State === "offline" ? "idle" : "idle",
      humanState(ev3State),
    );
  }

  function activeTurnComposerStatus(turn) {
    if (state.turnPollConnection === "unknown") {
      return t("chat.poll_connection_unknown");
    }
    if (state.turnPollConnection === "retrying") {
      return t("chat.poll_failed");
    }
    return t("chat.turn_progress", {
      state: humanState(turn.status),
      id: safeText(turn.turn_id),
    });
  }

  function enforceCapabilities(bootstrap) {
    const capabilities = safeObject(bootstrap.capabilities);
    const workbench = safeObject(capabilities.workbench);
    state.workbenchReadOnlyInvariant = (
      workbench.tool_effects === "read_only"
      && workbench.physical_control === false
      && workbench.ssh === false
      && workbench.tts === false
    );
    if (!robotControl) {
      setStatus(
        "status-motion",
        state.workbenchReadOnlyInvariant ? "locked" : "fault",
        state.workbenchReadOnlyInvariant
          ? t("capability.locked")
          : t("capability.contract_breach"),
      );
    }

    const turnActive = state.activeTurn
      && !TERMINAL_TURN_STATES.has(state.activeTurn.status);
    const chatEnabled = capabilities.chat === true
      && state.workbenchReadOnlyInvariant
      && !turnActive
      && state.modelReady === true;
    const researchOption = byId("turn-mode").querySelector('option[value="research_required"]');
    const researchTools = safeArray(capabilities.research);
    researchOption.disabled = !researchTools.includes("weather.current");
    setStatus(
      "status-research",
      researchOption.disabled ? "idle" : "ready",
      researchOption.disabled ? t("capability.unavailable") : t("capability.weather_ready"),
    );
    let composerEnabled = chatEnabled;
    if (robotControl) {
      composerEnabled = robotControl.reconcileComposer(chatEnabled);
    } else {
      byId("message-input").disabled = !chatEnabled;
      byId("send-button").disabled = !chatEnabled;
      byId("new-conversation-button").disabled = !chatEnabled;
    }
    if (!robotControl || !robotControl.isRobotTarget()) {
      byId("composer-status").textContent = turnActive
        ? activeTurnComposerStatus(state.activeTurn)
        : chatEnabled
          ? t("capability.chat_ready")
          : state.modelReady === false
            ? t("capability.model_not_ready")
            : t("capability.chat_unavailable");
    }
    if (microphoneInput) {
      microphoneInput.setAvailability(
        safeObject(capabilities.speech_to_text),
        composerEnabled,
      );
    }
    if (
      !state.workbenchReadOnlyInvariant
      && !state.workbenchViolationAnnounced
    ) {
      state.workbenchViolationAnnounced = true;
      showToast(t("capability.read_only_violation"), true);
    } else if (state.workbenchReadOnlyInvariant) {
      state.workbenchViolationAnnounced = false;
    }
  }

  function appendNodeRow(container, node, root = false) {
    const row = createElement("div", root ? "node-row node-root" : "node-row");
    row.appendChild(createElement("span", "node-connector"));
    row.lastChild.setAttribute("aria-hidden", "true");
    const kind = root ? "R" : safeText(node.node_kind, "N").charAt(0).toUpperCase();
    const icon = createElement("div", "node-icon", kind);
    icon.setAttribute("aria-hidden", "true");
    row.appendChild(icon);
    const copy = createElement("div", "node-copy");
    const fallbackName = safeText(
      node.display_name,
      safeText(node.node_id, t("registry.unnamed_node")),
    );
    copy.appendChild(createElement(
      "strong",
      "",
      localizedCatalogValue(node.display_name_key, fallbackName),
    ));
    const identity = [
      safeText(node.node_kind, root ? t("registry.robot") : t("registry.node")),
      safeText(node.controller_id || node.source_id, ""),
    ].filter(Boolean).join(" · ");
    copy.appendChild(createElement("small", "", identity));
    row.appendChild(copy);
    row.appendChild(createElement("span", "node-state", humanState(node.lifecycle || node.state)));
    container.appendChild(row);
  }

  function renderRegistry(registry) {
    state.registry = safeObject(registry);
    const tree = byId("registry-tree");
    tree.replaceChildren();
    const robots = safeArray(state.registry.robots);
    const nodes = safeArray(state.registry.nodes);
    if (robots.length === 0) {
      appendNodeRow(tree, {
        display_name: "EV3RSTORM",
        node_kind: "robot",
        lifecycle: "configured",
      }, true);
    }
    robots.forEach((robot) => {
      appendNodeRow(tree, {
        display_name: safeText(robot.display_name, robot.robot_id),
        display_name_key: robot.display_name_key,
        node_kind: safeText(robot.robot_kind, "robot"),
        lifecycle: safeText(robot.lifecycle, "configured"),
      }, true);
      const nodeIds = new Set(safeArray(robot.node_ids));
      nodes.filter((node) => (
        node.robot_id === robot.robot_id || nodeIds.has(node.node_id)
      )).forEach((node) => appendNodeRow(tree, node));
    });
    const systemNodes = nodes.filter((node) => !node.robot_id);
    if (systemNodes.length > 0) {
      appendNodeRow(tree, {
        display_name: t("registry.host_and_providers"),
        node_kind: "system",
        lifecycle: "configured",
      }, true);
      systemNodes.forEach((node) => appendNodeRow(tree, node));
    }
    if (!nodes.some((node) => node.node_kind === "camera" || node.node_kind === "microphone")) {
      const future = createElement("div", "node-row is-future");
      future.appendChild(createElement("span", "node-connector"));
      const icon = createElement("div", "node-icon", "P");
      icon.setAttribute("aria-hidden", "true");
      future.appendChild(icon);
      const copy = createElement("div", "node-copy");
      copy.appendChild(createElement("strong", "", t("registry.future_sources")));
      copy.appendChild(createElement("small", "", t("registry.future_sources_note")));
      future.appendChild(copy);
      future.appendChild(createElement("span", "node-state", t("registry.not_configured")));
      tree.appendChild(future);
    }
    const controller = nodes.find((node) => node.node_kind === "controller") || nodes[0] || {};
    const details = byId("controller-details");
    details.replaceChildren();
    [
      [t("registry.field.state"), humanState(controller.lifecycle)],
      [t("registry.field.controller_id"), safeText(controller.controller_id)],
      [t("registry.field.instance_id"), safeText(controller.controller_instance_id)],
      [t("registry.field.last_observed"), formatDateTime(controller.last_observed_at_unix_ms)],
      [t("registry.field.status_reason"), safeText(controller.status_reason_code)],
      [
        t("registry.field.physical_capabilities"),
        controller.control_exposed === true
          ? t("registry.physical_rejected")
          : t("registry.physical_locked"),
      ],
    ].forEach(([label, value]) => {
      const row = createElement("div");
      row.appendChild(createElement("dt", "", label));
      row.appendChild(createElement("dd", "", value));
      details.appendChild(row);
    });
    byId("fleet-aggregate-status").textContent = robots.length > 0
      ? humanState(robots[0].lifecycle)
      : t("registry.not_observed");
    if (
      !robotControl
      && nodes.some((node) => node.control_exposed === true)
    ) {
      setStatus("status-motion", "fault", t("capability.rejected"));
    }
  }

  function renderSpatialMap(spatialMap, connection = "connected") {
    state.spatialMap = safeObject(spatialMap);
    state.mapConnection = connection;
    spatialMapPresenter.render(state.spatialMap, connection);
  }

  function renderExperiments(experiments) {
    const list = byId("experiment-list");
    state.experiments = replaceRenderedItems(list, experiments, (experiment) => {
      const card = createElement("article", "experiment-card");
      card.appendChild(createElement(
        "div",
        "experiment-id",
        safeText(experiment.experiment_id, t("experiments.missing_id")),
      ));
      const copy = createElement("div");
      copy.appendChild(createElement(
        "h3",
        "",
        localizedCatalogValue(
          experiment.title_key,
          safeText(experiment.title, t("experiments.untitled")),
        ),
      ));
      copy.appendChild(createElement(
        "p",
        "",
        localizedCatalogValue(
          experiment.summary_key,
          safeText(experiment.summary, t("experiments.no_summary")),
        ),
      ));
      card.appendChild(copy);
      card.appendChild(createElement("span", "state-chip state-idle", humanState(experiment.status)));
      return card;
    });
  }

  function renderBootstrap(bootstrap) {
    const nextBootstrap = safeObject(bootstrap);
    const previousInstanceId = safeText(
      safeObject(state.bootstrap).server_instance_id,
      "",
    );
    const nextInstanceId = safeText(
      nextBootstrap.server_instance_id,
      "",
    );
    const restarted = Boolean(
      previousInstanceId
      && nextInstanceId
      && previousInstanceId !== nextInstanceId,
    );
    if (restarted) {
      state.turnPollGeneration += 1;
      window.location.reload();
      return;
    }
    state.bootstrap = nextBootstrap;
    renderRuntime(state.bootstrap.runtime);
    enforceCapabilities(state.bootstrap);
    renderRegistry(state.bootstrap.registry);
    renderExperiments(state.bootstrap.experiments);
    if (robotControl) {
      robotControl.renderLocale();
    }
  }

  function settingsFromForm() {
    const numberValue = (id) => {
      const value = Number(byId(id).value);
      return Number.isSafeInteger(value) ? value : null;
    };
    return {
      chat_mode: byId("setting-chat-mode").value,
      log_level: byId("setting-log-level").value,
      research: {
        max_elapsed_ms: numberValue("setting-max-elapsed-ms"),
        max_planner_latency_ms: numberValue("setting-planner-latency-ms"),
        max_planner_turns: numberValue("setting-max-planner-turns"),
        max_tool_calls: numberValue("setting-max-tool-calls"),
        max_replans: numberValue("setting-max-replans"),
        tool_request_ttl_ms: numberValue("setting-tool-request-ttl-ms"),
        evidence_ttl_ms: numberValue("setting-evidence-ttl-ms"),
        max_weather_observation_skew_ms: numberValue("setting-weather-skew-ms"),
      },
    };
  }

  function settingsChanges() {
    const original = safeObject(state.originalSettings);
    const draft = settingsFromForm();
    const changes = {};
    if (draft.chat_mode !== original.chat_mode) {
      changes.chat_mode = draft.chat_mode;
    }
    if (draft.log_level !== original.log_level) {
      changes.log_level = draft.log_level;
    }
    const originalResearch = safeObject(original.research);
    const researchChanges = {};
    Object.keys(draft.research).forEach((key) => {
      if (draft.research[key] !== originalResearch[key]) {
        researchChanges[key] = draft.research[key];
      }
    });
    if (Object.keys(researchChanges).length > 0) {
      changes.research = researchChanges;
    }
    return changes;
  }

  function updateSettingsDirtyState() {
    const dirty = Object.keys(settingsChanges()).length > 0;
    state.settingsDirty = dirty;
    byId("save-settings-button").disabled = (
      !dirty || !state.workbenchReadOnlyInvariant
    );
    byId("reset-settings-button").disabled = !dirty;
    byId("settings-status").textContent = dirty
      ? t("settings.unsaved")
      : t("settings.no_unsaved");
  }

  function renderSettings(settings) {
    state.settings = cloneJSON(safeObject(settings));
    state.originalSettings = cloneJSON(safeObject(settings));
    const research = safeObject(settings.research);
    byId("setting-chat-mode").value = safeText(settings.chat_mode, "conversation");
    byId("setting-log-level").value = safeText(settings.log_level, "info");
    byId("setting-max-elapsed-ms").value = safeInteger(research.max_elapsed_ms, 30000);
    byId("setting-planner-latency-ms").value = safeInteger(research.max_planner_latency_ms, 10000);
    byId("setting-max-planner-turns").value = safeInteger(research.max_planner_turns, 6);
    byId("setting-max-tool-calls").value = safeInteger(research.max_tool_calls, 1);
    byId("setting-max-replans").value = safeInteger(research.max_replans, 4);
    byId("setting-tool-request-ttl-ms").value = safeInteger(research.tool_request_ttl_ms, 8000);
    byId("setting-evidence-ttl-ms").value = safeInteger(research.evidence_ttl_ms, 600000);
    byId("setting-weather-skew-ms").value = safeInteger(research.max_weather_observation_skew_ms, 3600000);
    byId("settings-revision").textContent = t("settings.revision", {
      revision: safeInteger(settings.revision, t("common.missing")),
    });
    byId("turn-mode").value = settings.chat_mode === "research_required"
      ? "research_required"
      : "conversation";
    updateModeCopy();
    updateSettingsDirtyState();
  }

  async function saveSettings(event) {
    event.preventDefault();
    const form = byId("settings-form");
    if (!form.reportValidity()) {
      return;
    }
    const changes = settingsChanges();
    if (Object.keys(changes).length === 0) {
      return;
    }
    byId("save-settings-button").disabled = true;
    byId("settings-status").textContent = t("settings.saving");
    try {
      const payload = await api("/api/v1/settings", {
        method: "PUT",
        body: {
          expected_revision: state.originalSettings.revision,
          changes,
        },
      });
      renderSettings(payload.settings);
      showToast(t("settings.saved"));
    } catch (error) {
      updateSettingsDirtyState();
      const message = localizedError(error, "settings.save_failed");
      byId("settings-status").textContent = message;
      showToast(message, true);
    }
  }

  function resetSettings() {
    renderSettings(state.originalSettings);
  }

  function renderMessage(message, extraClass = "") {
    const role = message.role === "user" ? "user" : message.role === "assistant" ? "assistant" : "system";
    const article = createElement("article", `message message-${role}${extraClass ? ` ${extraClass}` : ""}`);
    const header = createElement("div", "message-header");
    header.appendChild(createElement(
      "span",
      "message-author",
      t(`chat.author.${safeText(message.author_key, role)}`),
    ));
    header.appendChild(createElement("time", "message-time", formatTime(message.created_at_unix_ms)));
    article.appendChild(header);
    article.appendChild(createElement("div", "message-body", safeText(message.content, "")));
    const citations = safeArray(message.citation_ids);
    if (citations.length > 0) {
      const meta = createElement("div", "message-meta");
      citations.forEach((citationId) => {
        const button = createElement("button", "citation-button", citationId);
        button.type = "button";
        button.addEventListener("click", async () => {
          activateInspectorTab("evidence", false);
          openInspector();
          if (!message.turn_id) {
            return;
          }
          try {
            const payload = await api(
              `/api/v1/turns/${encodeURIComponent(message.turn_id)}`,
            );
            renderEvidence(safeObject(payload.turn), citationId);
          } catch (error) {
            showToast(
              localizedError(error, "chat.history_evidence_failed"),
              true,
            );
          }
        });
        meta.appendChild(button);
      });
      article.appendChild(meta);
    }
    return article;
  }

  function renderPendingMessage() {
    const article = createElement("article", "message message-assistant is-pending");
    const header = createElement("div", "message-header");
    header.appendChild(createElement("span", "message-author", t("chat.author.assistant")));
    header.appendChild(createElement("span", "message-mode", humanState(state.activeTurn && state.activeTurn.status)));
    article.appendChild(header);
    const body = createElement("div", "message-body", t("chat.working"));
    const dots = createElement("span", "thinking-dots");
    for (let index = 0; index < 3; index += 1) {
      dots.appendChild(createElement("span"));
    }
    body.appendChild(dots);
    article.appendChild(body);
    return article;
  }

  function renderConversation() {
    const feed = byId("message-feed");
    const previousScrollTop = feed.scrollTop;
    const keepAtBottom = (
      feed.scrollHeight - feed.scrollTop - feed.clientHeight < 80
    );
    const robotTarget = selectedConversationTarget() === "robot";
    const messages = robotTarget
      ? state.robotDialogue
      : safeArray(safeObject(state.conversation).messages);
    const optimisticContent = robotTarget
      ? state.robotOptimisticContent
      : state.optimisticContent;
    feed.replaceChildren();
    if (
      messages.length === 0
      && !optimisticContent
      && (robotTarget || !state.activeTurn)
    ) {
      feed.appendChild(welcomeMessage);
    } else {
      messages.forEach((message) => feed.appendChild(renderMessage(message)));
      if (optimisticContent) {
        feed.appendChild(renderMessage({
          role: "user",
          content: optimisticContent,
          created_at_unix_ms: Date.now(),
          citation_ids: [],
        }, "is-pending"));
      }
      if (
        !robotTarget
        && state.activeTurn
        && !TERMINAL_TURN_STATES.has(state.activeTurn.status)
      ) {
        feed.appendChild(renderPendingMessage());
      } else if (
        !robotTarget
        && state.activeTurn
        && state.activeTurn.status === "failed"
      ) {
        feed.appendChild(renderMessage({
          role: "system",
          content: t("chat.episode_aborted", {
            code: safeText(state.activeTurn.error_code, t("common.unknown")),
          }),
          created_at_unix_ms: state.activeTurn.completed_at_unix_ms,
          citation_ids: [],
        }, "message-error"));
      }
    }
    if (keepAtBottom) {
      feed.scrollTop = feed.scrollHeight;
    } else {
      feed.scrollTop = previousScrollTop;
    }
    renderContext();
  }

  function renderContext() {
    const details = byId("context-details");
    details.replaceChildren();
    const conversation = safeObject(state.conversation);
    [
      [
        t("chat.context.conversation_id"),
        safeText(conversation.conversation_id, t("chat.context.not_created")),
      ],
      [
        t("chat.context.version"),
        safeInteger(conversation.version, t("common.missing")),
      ],
      [t("chat.context.mode"), safeText(conversation.context_mode, "typed_history")],
      [
        t("chat.context.turn_count"),
        i18n.number(
          safeArray(conversation.messages).filter((message) => message.role === "user").length,
        ),
      ],
    ].forEach(([label, value]) => {
      const row = createElement("div");
      row.appendChild(createElement("dt", "", label));
      row.appendChild(createElement("dd", "", value));
      details.appendChild(row);
    });
  }

  function eventMatchesTurn(event, turnId) {
    return safeObject(event.correlation).turn_id === turnId;
  }

  function renderActivity(turn) {
    const trace = byId("activity-trace");
    trace.replaceChildren();
    const status = turn ? safeText(turn.status, "queued") : "idle";
    const statusBadge = byId("episode-status");
    statusBadge.textContent = turn ? humanState(status) : t("chat.activity.waiting");
    statusBadge.dataset.status = status === "answered"
      ? "completed"
      : status === "failed"
        ? "failed"
        : status === "queued" || status === "running"
          ? "busy"
          : "idle";
    const correlated = turn
      ? state.events.filter((event) => eventMatchesTurn(event, turn.turn_id)).slice(-10)
      : [];
    if (correlated.length === 0) {
      const item = createElement("li");
      item.dataset.state = status === "failed" ? "failed" : status === "answered" ? "complete" : status === "idle" ? "idle" : "active";
      item.appendChild(createElement("span", "trace-dot"));
      const copy = createElement("div");
      copy.appendChild(createElement(
        "strong",
        "",
        turn ? humanState(status) : t("chat.activity.no_episode"),
      ));
      copy.appendChild(createElement(
        "p",
        "",
        turn ? safeText(turn.turn_id) : t("chat.activity.note"),
      ));
      item.appendChild(copy);
      trace.appendChild(item);
    } else {
      correlated.forEach((event) => {
        const item = createElement("li");
        const type = safeText(event.event_type, "");
        item.dataset.state = event.level === "error"
          ? "failed"
          : type === "research.tool_completed"
            ? "evidence"
            : type === "chat.turn_answered"
              ? "complete"
              : "active";
        item.appendChild(createElement("span", "trace-dot"));
        const copy = createElement("div");
        copy.appendChild(createElement("strong", "", safeText(event.event_type, event.category)));
        copy.appendChild(createElement("p", "", safeText(event.message)));
        item.appendChild(copy);
        trace.appendChild(item);
      });
    }
    const episode = safeObject(turn && turn.episode);
    const metricValues = turn ? [
      safeInteger(episode.planner_turns, t("common.missing")),
      safeInteger(episode.tool_calls, t("common.missing")),
      safeInteger(episode.replans, t("common.missing")),
      safeInteger(episode.final_context_version, t("common.missing")),
    ] : [
      t("common.missing"),
      t("common.missing"),
      t("common.missing"),
      t("common.missing"),
    ];
    byId("episode-metrics").querySelectorAll("dd").forEach((node, index) => {
      node.textContent = String(metricValues[index]);
    });
  }

  function renderEvidence(turn, selectedCitationId = null) {
    const list = byId("evidence-list");
    list.replaceChildren();
    const episode = safeObject(turn && turn.episode);
    const envelopes = safeArray(episode.evidence);
    const allCitations = turn ? safeArray(turn.citation_ids) : [];
    const citations = selectedCitationId && allCitations.includes(selectedCitationId)
      ? [selectedCitationId]
      : allCitations;
    if (citations.length === 0) {
      list.className = "empty-compact";
      list.appendChild(createElement("p", "", t("chat.evidence.empty")));
      list.appendChild(createElement("small", "", t("chat.evidence.empty_note")));
      return;
    }
    list.className = "";
    citations.forEach((citationId) => {
      const envelope = envelopes.find((candidate) => (
        candidate.evidence_id === citationId
      )) || {};
      const payload = safeObject(envelope.payload);
      const location = safeObject(payload.location);
      const providerEvidence = safeArray(payload.evidence);
      const provenance = safeObject(
        safeObject(providerEvidence[providerEvidence.length - 1]).provenance,
      );
      const card = createElement("article", "evidence-card");
      card.appendChild(createElement("strong", "", citationId));
      card.appendChild(createElement(
        "p",
        "",
        [location.name, location.country_name]
          .filter((value) => typeof value === "string" && value)
          .join(", ")
          || t("chat.evidence.verified_fallback"),
      ));
      card.appendChild(createElement(
        "small",
        "",
        [
          safeText(
            provenance.provider,
            safeText(envelope.tool_name, t("chat.evidence.read_only")),
          ),
          t("chat.evidence.validity"),
          safeText(provenance.raw_sha256, "").slice(0, 12),
        ].filter(Boolean).join(" · "),
      ));
      list.appendChild(card);
    });
  }

  function renderTurnAnnouncement() {
    if (!state.activeTurn || !TERMINAL_TURN_STATES.has(state.activeTurn.status)) {
      return;
    }
    byId("chat-announcer").textContent = state.activeTurn.status === "answered"
      ? t("chat.announcer.answer", {
        text: safeText(state.activeTurn.answer_text, ""),
      })
      : state.activeTurn.status === "clarification_required"
        ? t("chat.announcer.clarification", {
          text: safeText(state.activeTurn.clarification_question, ""),
        })
        : t("chat.announcer.stopped");
  }

  function renderTurn(turn) {
    const previousTurn = state.activeTurn;
    state.activeTurn = safeObject(turn);
    renderActivity(state.activeTurn);
    renderEvidence(state.activeTurn);
    const visibleStateChanged = (
      !previousTurn
      || previousTurn.turn_id !== state.activeTurn.turn_id
      || previousTurn.status !== state.activeTurn.status
    );
    if (visibleStateChanged) {
      renderConversation();
    }
    if (TERMINAL_TURN_STATES.has(state.activeTurn.status)) {
      enforceCapabilities(safeObject(state.bootstrap));
      byId("composer-status").textContent = state.modelReady !== true
        ? t("capability.model_not_ready")
        : state.activeTurn.status === "answered"
          ? t("chat.answer_ready")
          : state.activeTurn.status === "clarification_required"
            ? t("chat.clarification_needed")
            : t("chat.episode_stopped", {
              code: safeText(state.activeTurn.error_code, t("common.unknown")),
            });
      if (visibleStateChanged) {
        renderTurnAnnouncement();
      }
    } else {
      enforceCapabilities(safeObject(state.bootstrap));
      byId("composer-status").textContent = activeTurnComposerStatus(state.activeTurn);
    }
  }

  function renderConversationSubtitle() {
    if (!state.conversation) {
      return;
    }
    byId("conversation-subtitle").textContent = t("chat.conversation_version", {
      mode: safeText(state.conversation.context_mode, "typed_history"),
      version: safeInteger(state.conversation.version, t("common.missing")),
    });
  }

  async function createConversation() {
    const payload = await api("/api/v1/conversations", {
      method: "POST",
      body: {},
    });
    state.conversation = safeObject(payload.conversation);
    state.activeTurn = null;
    state.optimisticContent = null;
    renderConversationSubtitle();
    renderConversation();
    renderActivity(null);
    renderEvidence(null);
    return state.conversation;
  }

  async function refreshConversation() {
    if (!state.conversation || !state.conversation.conversation_id) {
      return;
    }
    const id = encodeURIComponent(state.conversation.conversation_id);
    const payload = await api(`/api/v1/conversations/${id}`);
    state.conversation = safeObject(payload.conversation);
    renderConversationSubtitle();
    renderConversation();
  }

  async function startNewConversation() {
    if (state.activeTurn && !TERMINAL_TURN_STATES.has(state.activeTurn.status)) {
      showToast(t("chat.wait_for_terminal"), true);
      return;
    }
    if (microphoneInput) {
      microphoneInput.cancel();
    }
    byId("new-conversation-button").disabled = true;
    try {
      await createConversation();
      byId("message-input").focus();
      showToast(t("chat.created"));
    } catch (error) {
      showToast(localizedError(error, "chat.create_failed"), true);
    } finally {
      enforceCapabilities(safeObject(state.bootstrap));
    }
  }

  async function pollTurn(turnId, generation) {
    if (
      sessionGuard.isExpired()
      || generation !== state.turnPollGeneration
    ) {
      return;
    }
    try {
      const payload = await api(`/api/v1/turns/${encodeURIComponent(turnId)}`);
      if (generation !== state.turnPollGeneration) {
        return;
      }
      const turn = safeObject(payload.turn);
      const transition = transitionTurnPoll(
        {
          failures: state.turnPollFailures,
          connection: state.turnPollConnection,
        },
        { type: "success", turn },
      );
      state.turnPollFailures = transition.failures;
      state.turnPollConnection = transition.connection;
      renderTurn(turn);
      if (transition.recovered) {
        showToast(t("chat.poll_recovered"));
      }
      if (transition.terminal) {
        await refreshConversation();
        renderTurn(turn);
        return;
      }
    } catch (error) {
      if (sessionGuard.isExpired()) {
        return;
      }
      const transition = transitionTurnPoll(
        {
          failures: state.turnPollFailures,
          connection: state.turnPollConnection,
        },
        { type: "failure" },
      );
      state.turnPollFailures = transition.failures;
      state.turnPollConnection = transition.connection;
      byId("send-button").disabled = true;
      byId("message-input").disabled = true;
      byId("new-conversation-button").disabled = true;
      byId("composer-status").textContent = transition.connection === "unknown"
        ? t("chat.poll_connection_unknown")
        : localizedError(error, "chat.poll_failed");
      if (transition.becameUnknown) {
        showToast(t("chat.poll_connection_unknown"), true);
      }
      window.setTimeout(
        () => pollTurn(turnId, generation),
        transition.retryDelayMs,
      );
      return;
    }
    window.setTimeout(
      () => pollTurn(turnId, generation),
      TURN_POLL_POLICY.baseDelayMs,
    );
  }

  function selectedConversationTarget() {
    return robotControl ? robotControl.selectedTarget() : "workbench";
  }

  async function submitCurrentContent(target) {
    const input = byId("message-input");
    const content = input.value.trim();
    if (!content) {
      return;
    }
    if (target === "robot" && robotControl) {
      if (microphoneInput) {
        microphoneInput.cancel();
      }
      state.robotOptimisticContent = content;
      renderConversation();
      const accepted = await robotControl.submitInput(
        content,
        i18n.locale,
      );
      if (accepted) {
        input.value = "";
      } else {
        state.robotOptimisticContent = null;
        renderConversation();
      }
      return;
    }
    if (!state.workbenchReadOnlyInvariant) {
      return;
    }
    if (state.activeTurn && !TERMINAL_TURN_STATES.has(state.activeTurn.status)) {
      showToast(t("chat.episode_in_progress"), true);
      return;
    }
    if (microphoneInput) {
      microphoneInput.cancel();
    }
    byId("send-button").disabled = true;
    input.disabled = true;
    state.optimisticContent = content;
    renderConversation();
    try {
      if (!state.conversation) {
        await createConversation();
        state.optimisticContent = content;
        renderConversation();
      }
      const conversationId = encodeURIComponent(state.conversation.conversation_id);
      const payload = await api(`/api/v1/conversations/${conversationId}/turns`, {
        method: "POST",
        body: {
          client_request_id: randomId("ui"),
          expected_conversation_version: state.conversation.version,
          content,
          mode: byId("turn-mode").value,
          response_locale: i18n.locale,
        },
        timeout: 20000,
      });
      input.value = "";
      state.optimisticContent = null;
      state.turnPollFailures = 0;
      state.turnPollConnection = "connected";
      renderTurn(payload.turn);
      await refreshConversation();
      state.turnPollGeneration += 1;
      pollTurn(payload.turn.turn_id, state.turnPollGeneration);
    } catch (error) {
      state.optimisticContent = null;
      renderConversation();
      enforceCapabilities(safeObject(state.bootstrap));
      showToast(localizedError(error, "chat.send_failed"), true);
    }
  }

  async function submitTurn(event) {
    event.preventDefault();
    await submitCurrentContent(selectedConversationTarget());
  }

  function eventTime(event) {
    return safeInteger(event.recorded_at_unix_ms, safeInteger(event.occurred_at_unix_ms, null));
  }

  function eventVisible(event) {
    const severity = byId("event-severity-filter").value;
    const category = byId("event-plane-filter").value;
    const search = byId("event-search-input").value.trim().toLocaleLowerCase(i18n.formatLocale);
    if (severity && event.level !== severity) {
      return false;
    }
    if (category && event.category !== category) {
      return false;
    }
    if (!search) {
      return true;
    }
    return [
      event.event_id,
      event.event_type,
      event.message,
      safeObject(event.correlation).turn_id,
      safeObject(event.correlation).tool_call_id,
    ].some((value) => (
      typeof value === "string"
      && value.toLocaleLowerCase(i18n.formatLocale).includes(search)
    ));
  }

  function openEventDetail(event, focusClose = true) {
    state.selectedEvent = event;
    const source = safeObject(event.source);
    const correlation = safeObject(event.correlation);
    byId("event-detail-heading").textContent = safeText(
      event.event_type,
      t("events.detail_fallback"),
    );
    const fields = byId("event-detail-fields");
    fields.replaceChildren();
    [
      [t("events.field.event_id"), safeText(event.event_id)],
      [
        t("events.field.sequence"),
        safeInteger(event.sequence, t("common.missing")),
      ],
      [t("events.field.time"), formatDateTime(eventTime(event))],
      [t("events.field.level"), safeText(event.level)],
      [t("events.field.category"), safeText(event.category)],
      [t("events.field.source_id"), safeText(source.source_id)],
      [t("events.field.robot_id"), safeText(source.robot_id)],
      [t("events.field.node_id"), safeText(source.node_id)],
      [t("events.field.conversation_id"), safeText(correlation.conversation_id)],
      [t("events.field.turn_id"), safeText(correlation.turn_id)],
      [t("events.field.tool_call_id"), safeText(correlation.tool_call_id)],
      [t("events.field.request_id"), safeText(correlation.request_id)],
    ].forEach(([label, value]) => {
      const row = createElement("div");
      row.appendChild(createElement("dt", "", label));
      row.appendChild(createElement("dd", "", value));
      fields.appendChild(row);
    });
    byId("event-detail-json").textContent = JSON.stringify(event, null, 2);
    byId("event-detail").hidden = false;
    if (focusClose) {
      byId("close-event-detail").focus();
    }
  }

  function renderEvents() {
    const body = byId("event-table-body");
    body.replaceChildren();
    const visible = state.events.filter(eventVisible).slice().reverse();
    if (visible.length === 0) {
      const row = createElement("tr", "empty-row");
      const cell = createElement("td", "", state.events.length === 0
        ? t("events.empty")
        : t("events.no_match"));
      cell.colSpan = 6;
      row.appendChild(cell);
      body.appendChild(row);
      return;
    }
    visible.forEach((event) => {
      const row = createElement("tr");
      row.appendChild(createElement("td", "mono-cell", formatTime(eventTime(event))));
      const severityCell = createElement("td");
      const severity = createElement("span", "severity-label", safeText(event.level, "info"));
      severity.dataset.severity = safeText(event.level, "info");
      severityCell.appendChild(severity);
      row.appendChild(severityCell);
      row.appendChild(createElement("td", "mono-cell", safeText(event.category)));
      row.appendChild(createElement("td", "mono-cell", safeText(event.event_type)));
      row.appendChild(createElement("td", "", safeText(event.message)));
      const actionCell = createElement("td");
      const button = createElement("button", "event-row-button", "→");
      button.type = "button";
      button.setAttribute("aria-label", t("events.show", {
        type: safeText(event.event_type, t("events.event_fallback")),
      }));
      button.addEventListener("click", () => openEventDetail(event));
      actionCell.appendChild(button);
      row.appendChild(actionCell);
      body.appendChild(row);
    });
  }

  function ingestEvents(payload) {
    const events = safeArray(payload.events);
    events.forEach((event) => {
      const eventId = safeText(event.event_id, "");
      if (!eventId || state.eventIds.has(eventId)) {
        return;
      }
      state.eventIds.add(eventId);
      state.events.push(event);
    });
    if (state.events.length > MAX_LOCAL_EVENTS) {
      const removed = state.events.splice(0, state.events.length - MAX_LOCAL_EVENTS);
      removed.forEach((event) => state.eventIds.delete(event.event_id));
    }
    const next = safeInteger(payload.next_after_sequence, null);
    const newest = safeInteger(payload.newest_sequence, null);
    if (next !== null) {
      state.afterSequence = next;
    } else if (newest !== null && newest > state.afterSequence) {
      state.afterSequence = newest;
    }
    if (payload.gap === true || safeInteger(payload.dropped_total, 0) > 0) {
      state.eventGapActive = true;
      state.eventGapDroppedTotal = safeInteger(payload.dropped_total, 0);
    } else {
      state.eventGapActive = false;
      state.eventGapDroppedTotal = 0;
    }
    renderEventGap();
    renderEvents();
    if (state.activeTurn) {
      renderActivity(state.activeTurn);
      renderEvidence(state.activeTurn);
    }
  }

  function renderEventGap() {
    const gapNotice = byId("event-gap-notice");
    gapNotice.hidden = !state.eventGapActive;
    if (!gapNotice.hidden) {
      gapNotice.textContent = t("events.gap", {
        count: i18n.number(state.eventGapDroppedTotal),
      });
    }
  }

  function renderEventStreamStatus() {
    const label = byId("event-stream-label");
    const parent = label.parentElement;
    parent.classList.toggle("is-offline", state.eventStreamState === "offline");
    parent.classList.toggle("is-paused", state.eventsPaused);
    if (state.eventsPaused) {
      label.textContent = t("events.paused");
      return;
    }
    if (state.eventStreamState === "offline") {
      label.textContent = t("events.offline");
      return;
    }
    if (state.eventStreamState === "reconnecting") {
      label.textContent = t("events.reconnecting");
      return;
    }
    label.textContent = t("events.live", {
      sequence: i18n.number(state.afterSequence),
    });
  }

  async function pollEvents() {
    if (sessionGuard.isExpired()) {
      return;
    }
    if (!state.eventsPaused) {
      try {
        const payload = await api(`/api/v1/events?after_sequence=${state.afterSequence}&limit=${EVENT_LIMIT}`);
        ingestEvents(payload);
        state.eventStreamState = "live";
        renderEventStreamStatus();
      } catch (error) {
        state.eventStreamState = "offline";
        renderEventStreamStatus();
      }
    }
    if (!sessionGuard.isExpired()) {
      window.setTimeout(pollEvents, document.hidden ? 5000 : 1500);
    }
  }

  function toggleEventsPaused() {
    state.eventsPaused = !state.eventsPaused;
    const button = byId("pause-events-button");
    button.setAttribute("aria-pressed", state.eventsPaused ? "true" : "false");
    button.textContent = state.eventsPaused ? t("events.resume") : t("events.pause");
    state.eventStreamState = state.eventsPaused
      ? state.eventStreamState
      : "reconnecting";
    renderEventStreamStatus();
  }

  function exportEvents() {
    if (state.events.length === 0) {
      showToast(t("events.nothing_to_export"), true);
      return;
    }
    const jsonl = `${state.events.map((event) => JSON.stringify(event)).join("\n")}\n`;
    const blob = new Blob([jsonl], { type: "application/x-ndjson;charset=utf-8" });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = `robot-llm-events-${Date.now()}.jsonl`;
    link.click();
    URL.revokeObjectURL(url);
    showToast(t("events.exported"));
  }

  function updateModeCopy() {
    if (robotControl && robotControl.isRobotTarget()) {
      return;
    }
    const research = byId("turn-mode").value === "research_required";
    byId("mode-capability-note").textContent = research
      ? t("mode.research_note")
      : t("mode.conversation_note");
  }

  function renderProbeResult() {
    const node = byId("probe-result");
    const probe = safeObject(state.lmProbe);
    if (probe.phase === "checking") {
      node.dataset.status = "checking";
      node.textContent = t("runtime.checking");
      return;
    }
    if (probe.phase === "completed") {
      const lmStudio = safeObject(probe.result);
      node.dataset.status = safeText(lmStudio.state, "unknown");
      node.textContent = lmStudio.configured_model_loaded === false
        ? `${humanState(lmStudio.state)} · ${t("runtime.model_not_loaded")}`
        : `${humanState(lmStudio.state)} · ${safeText(lmStudio.model, t("runtime.no_model"))}`;
      return;
    }
    if (probe.phase === "failed") {
      node.dataset.status = "offline";
      node.textContent = localizedError(
        { code: probe.errorCode },
        "runtime.probe_failed",
      );
      return;
    }
    node.dataset.status = "idle";
    node.textContent = t("settings.runtime.probe_idle");
  }

  async function probeLMStudio() {
    const button = byId("probe-lm-studio-button");
    button.disabled = true;
    state.lmProbe = {
      phase: "checking",
      result: null,
      errorCode: null,
    };
    renderProbeResult();
    try {
      const payload = await api("/api/v1/runtime/lm-studio/probe", {
        method: "POST",
        body: {},
      });
      const lmStudio = safeObject(payload.lm_studio);
      const runtime = safeObject(state.bootstrap.runtime);
      runtime.lm_studio = lmStudio;
      state.bootstrap.runtime = runtime;
      renderRuntime(runtime);
      enforceCapabilities(state.bootstrap);
      state.lmProbe = {
        phase: "completed",
        result: { ...lmStudio },
        errorCode: null,
      };
      renderProbeResult();
    } catch (error) {
      const runtime = safeObject(state.bootstrap && state.bootstrap.runtime);
      runtime.lm_studio = {
        ...safeObject(runtime.lm_studio),
        state: "offline",
        configured_model_loaded: false,
      };
      if (state.bootstrap) {
        state.bootstrap.runtime = runtime;
      }
      renderRuntime(runtime);
      enforceCapabilities(safeObject(state.bootstrap));
      state.lmProbe = {
        phase: "failed",
        result: null,
        errorCode: safeText(error && error.code, "network_error"),
      };
      renderProbeResult();
    } finally {
      button.disabled = false;
    }
  }

  async function refreshBootstrap(silent = true) {
    if (sessionGuard.isExpired() || state.bootstrapBusy) {
      return;
    }
    state.bootstrapBusy = true;
    try {
      const payload = await api("/api/v1/bootstrap");
      renderBootstrap(payload);
    } catch (error) {
      setStatus("status-lm-studio", "offline", t("runtime.dashboard_offline"));
      if (!silent) {
        showToast(localizedError(error, "server.unreachable"), true);
      }
    } finally {
      state.bootstrapBusy = false;
    }
  }

  async function refreshSpatialMap(silent = true) {
    if (sessionGuard.isExpired() || state.mapBusy) {
      return;
    }
    state.mapBusy = true;
    try {
      const payload = await api("/api/v1/map", {
        timeout: 5000,
      });
      renderSpatialMap(payload.map, "connected");
    } catch (error) {
      renderSpatialMap(state.spatialMap, "offline");
      if (!silent) {
        showToast(
          localizedError(error, "errors.spatial_map_unavailable"),
          true,
        );
      }
    } finally {
      state.mapBusy = false;
    }
  }

  function renderLocalizedState() {
    const activeElement = document.activeElement;
    const activeId = activeElement && activeElement.id;
    const scrollSnapshots = [
      byId("agent-inspector"),
      document.querySelector("[data-view-panel]:not([hidden])"),
      document.querySelector(".event-table-wrap"),
      byId("event-detail"),
      byId("event-detail-json"),
    ].filter(Boolean).map((node) => ({
      node,
      top: node.scrollTop,
      left: node.scrollLeft,
    }));
    const viewportScroll = {
      x: window.scrollX,
      y: window.scrollY,
    };
    const selection = activeElement
      && typeof activeElement.selectionStart === "number"
      ? {
        start: activeElement.selectionStart,
        end: activeElement.selectionEnd,
        direction: activeElement.selectionDirection,
      }
      : null;

    applyStaticTranslations();
    if (state.bootstrap) {
      renderRuntime(state.bootstrap.runtime);
      enforceCapabilities(state.bootstrap);
      renderRegistry(state.bootstrap.registry);
      renderExperiments(state.bootstrap.experiments);
    }
    renderSpatialMap(state.spatialMap, state.mapConnection);
    renderProbeResult();
    if (state.settings) {
      byId("settings-revision").textContent = t("settings.revision", {
        revision: safeInteger(state.settings.revision, t("common.missing")),
      });
      updateSettingsDirtyState();
    }
    renderConversationSubtitle();
    renderConversation();
    if (state.activeTurn) {
      renderTurn(state.activeTurn);
      renderTurnAnnouncement();
    } else {
      renderActivity(null);
      renderEvidence(null);
    }
    renderEvents();
    renderEventGap();
    renderEventStreamStatus();
    const pauseButton = byId("pause-events-button");
    pauseButton.textContent = state.eventsPaused
      ? t("events.resume")
      : t("events.pause");
    updateModeCopy();
    if (microphoneInput) {
      microphoneInput.renderLocale();
    }
    if (robotControl) {
      robotControl.renderLocale();
    }
    if (state.selectedEvent && !byId("event-detail").hidden) {
      openEventDetail(state.selectedEvent, false);
    }

    const focusTarget = activeId ? byId(activeId) : null;
    if (focusTarget && document.activeElement !== focusTarget) {
      focusTarget.focus({ preventScroll: true });
    }
    if (
      selection
      && focusTarget
      && typeof focusTarget.setSelectionRange === "function"
    ) {
      focusTarget.setSelectionRange(
        selection.start,
        selection.end,
        selection.direction,
      );
    }
    scrollSnapshots.forEach(({ node, top, left }) => {
      node.scrollTop = top;
      node.scrollLeft = left;
    });
    if (
      window.scrollX !== viewportScroll.x
      || window.scrollY !== viewportScroll.y
    ) {
      window.scrollTo(viewportScroll.x, viewportScroll.y);
    }
  }

  function bindInteractions() {
    document.querySelectorAll("[data-view]").forEach((button) => {
      button.addEventListener("click", () => activateView(button.dataset.view));
    });
    document.querySelectorAll("[data-inspector-tab]").forEach((tab) => {
      tab.addEventListener("click", () => activateInspectorTab(tab.dataset.inspectorTab));
      tab.addEventListener("keydown", (event) => {
        if (event.key !== "ArrowLeft" && event.key !== "ArrowRight") {
          return;
        }
        event.preventDefault();
        const tabs = Array.from(document.querySelectorAll("[data-inspector-tab]"));
        const direction = event.key === "ArrowRight" ? 1 : -1;
        const next = (tabs.indexOf(tab) + direction + tabs.length) % tabs.length;
        activateInspectorTab(tabs[next].dataset.inspectorTab, true);
      });
    });
    byId("open-inspector-button").addEventListener("click", openInspector);
    byId("close-inspector-button").addEventListener("click", closeInspector);
    byId("new-conversation-button").addEventListener("click", startNewConversation);
    byId("composer-form").addEventListener("submit", submitTurn);
    byId("composer-target").addEventListener("change", () => {
      if (microphoneInput) {
        microphoneInput.cancel();
      }
    });
    byId("message-input").addEventListener("keydown", (event) => {
      if (event.key === "Enter" && !event.shiftKey) {
        event.preventDefault();
        byId("composer-form").requestSubmit();
      }
    });
    byId("turn-mode").addEventListener("change", updateModeCopy);
    byId("ui-language").addEventListener("change", (event) => {
      i18n.setLocale(event.currentTarget.value);
    });
    document.querySelectorAll("[data-prompt]").forEach((button) => {
      button.addEventListener("click", () => {
        byId("message-input").value = button.dataset.prompt;
        byId("turn-mode").value = button.dataset.promptMode || "conversation";
        updateModeCopy();
        byId("message-input").focus();
      });
    });
    byId("settings-form").addEventListener("submit", saveSettings);
    byId("settings-form").addEventListener("input", updateSettingsDirtyState);
    byId("settings-form").addEventListener("change", updateSettingsDirtyState);
    byId("reset-settings-button").addEventListener("click", resetSettings);
    byId("probe-lm-studio-button").addEventListener("click", probeLMStudio);
    byId("pause-events-button").addEventListener("click", toggleEventsPaused);
    byId("export-events-button").addEventListener("click", exportEvents);
    byId("event-severity-filter").addEventListener("change", renderEvents);
    byId("event-plane-filter").addEventListener("change", renderEvents);
    byId("event-search-input").addEventListener("input", renderEvents);
    byId("close-event-detail").addEventListener("click", () => {
      byId("event-detail").hidden = true;
      state.selectedEvent = null;
    });
    byId("copy-event-json").addEventListener("click", async () => {
      if (!state.selectedEvent) {
        return;
      }
      try {
        await navigator.clipboard.writeText(JSON.stringify(state.selectedEvent, null, 2));
        showToast(t("events.json_copied"));
      } catch (_error) {
        showToast(t("events.json_copy_failed"), true);
      }
    });
    document.addEventListener("keydown", (event) => {
      if (
        (event.metaKey || event.ctrlKey)
        && event.key.toLocaleLowerCase(i18n.formatLocale) === "j"
      ) {
        event.preventDefault();
        activateView("events");
      }
      if (event.key === "Escape") {
        if (microphoneInput) {
          microphoneInput.cancel();
        }
        if (!byId("event-detail").hidden) {
          byId("event-detail").hidden = true;
        }
        if (byId("agent-inspector").classList.contains("is-mobile-visible")) {
          closeInspector();
        }
      }
    });
  }
  async function initialize() {
    applyStaticTranslations();
    i18n.subscribe(renderLocalizedState);
    bindInteractions();
    microphoneInput = window.RobotMicrophoneInput.create({
      document,
      logic: window.RobotSpeechInputLogic,
      translate: t,
      randomId,
      request: api,
      onTranscript: (text, metadata) => {
        const input = byId("message-input");
        const target = selectedConversationTarget();
        input.value = text;
        input.focus();
        if (metadata.autoSend && !input.disabled) {
          void submitCurrentContent(target);
        }
      },
      onError: (message) => showToast(message, true),
      getUiLocale: () => i18n.locale,
      workletUrl: "assets/pcm_capture_worklet.js",
    });
    microphoneInput.initialize();
    robotControl = window.RobotControlUI.create({
      document,
      request: api,
      translate: t,
      randomId,
      showToast,
      getLocale: () => i18n.locale,
      sessionGuard,
      formatError: (error) => localizedError(
        error,
        "errors.robot_control_failed",
      ),
      onAvailabilityChanged: (enabled) => {
        if (!microphoneInput) {
          return;
        }
        const capabilities = safeObject(
          safeObject(state.bootstrap).capabilities,
        );
        microphoneInput.setAvailability(
          safeObject(capabilities.speech_to_text),
          enabled,
        );
      },
      onInputAccepted: (originalText, turn) => {
        const acceptedAt = Date.now();
        const additions = [{
          role: "user",
          content: originalText,
          created_at_unix_ms: acceptedAt,
          citation_ids: [],
        }];
        const answerText = safeText(safeObject(turn).answer_text, "");
        if (answerText) {
          additions.push({
            role: "assistant", author_key: "robot",
            content: answerText,
            created_at_unix_ms: acceptedAt,
            citation_ids: [],
          });
        }
        state.robotDialogue = state.robotDialogue
          .concat(additions)
          .slice(-MAX_ROBOT_DIALOGUE_MESSAGES);
        state.robotOptimisticContent = null;
        byId("message-input").value = "";
        renderConversation();
      },
      onTargetChanged: renderConversation,
    });
    try {
      const [bootstrapPayload, settingsPayload] = await Promise.all([
        api("/api/v1/bootstrap"),
        api("/api/v1/settings"),
        robotControl.initialize(),
      ]);
      renderBootstrap(bootstrapPayload);
      renderSettings(settingsPayload.settings || bootstrapPayload.settings);
      probeLMStudio();
      refreshSpatialMap(true);
    } catch (error) {
      byId("composer-status").textContent = t("server.start_failed");
      showToast(localizedError(error, "server.start_failed"), true);
    }
    if (sessionGuard.isExpired()) {
      return;
    }
    pollEvents();
    window.setInterval(() => refreshBootstrap(true), 10000);
    window.setInterval(
      () => refreshSpatialMap(true),
      MAP_POLL_INTERVAL_MS,
    );
  }

  initialize();
})();
