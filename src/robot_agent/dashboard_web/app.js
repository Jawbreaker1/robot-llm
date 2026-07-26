(() => {
  "use strict";

  const TOKEN_META = document.querySelector('meta[name="robot-dashboard-token"]');
  const SESSION_TOKEN = TOKEN_META ? TOKEN_META.content : "";
  const TERMINAL_TURN_STATES = new Set([
    "answered",
    "clarification_required",
    "failed",
  ]);
  const EVENT_LIMIT = 100;
  const MAX_LOCAL_EVENTS = 500;

  const state = {
    bootstrap: null,
    settings: null,
    originalSettings: null,
    registry: null,
    experiments: [],
    conversation: null,
    activeTurn: null,
    optimisticContent: null,
    events: [],
    eventIds: new Set(),
    afterSequence: 0,
    eventsPaused: false,
    selectedEvent: null,
    readOnlyInvariant: true,
    turnPollGeneration: 0,
    turnPollFailures: 0,
    bootstrapBusy: false,
    settingsDirty: false,
    modelReady: null,
  };

  const byId = (id) => document.getElementById(id);
  const welcomeMessage = byId("welcome-message");
  const safeArray = (value) => (Array.isArray(value) ? value : []);
  const safeObject = (value) => (
    value && typeof value === "object" && !Array.isArray(value) ? value : {}
  );
  const safeText = (value, fallback = "—") => (
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
    if (!Number.isFinite(unixMs)) {
      return "—";
    }
    try {
      return new Intl.DateTimeFormat("sv-SE", {
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit",
        fractionalSecondDigits: 3,
      }).format(new Date(unixMs));
    } catch (_error) {
      return new Date(unixMs).toLocaleTimeString("sv-SE");
    }
  }

  function formatDateTime(unixMs) {
    if (!Number.isFinite(unixMs)) {
      return "—";
    }
    return new Intl.DateTimeFormat("sv-SE", {
      dateStyle: "medium",
      timeStyle: "medium",
    }).format(new Date(unixMs));
  }

  function humanState(value) {
    const labels = {
      unknown: "okänd",
      online: "ansluten",
      offline: "offline",
      unobserved: "inte observerad",
      configured: "konfigurerad",
      active: "aktiv",
      inactive: "inaktiv",
      queued: "köad",
      running: "arbetar",
      answered: "besvarad",
      clarification_required: "behöver förtydligande",
      failed: "misslyckad",
      verified: "verifierad",
      waiting: "väntar",
    };
    return labels[value] || safeText(value, "okänd");
  }

  async function api(path, options = {}) {
    const method = options.method || "GET";
    const headers = { Accept: "application/json" };
    const request = {
      method,
      headers,
      cache: "no-store",
      credentials: "same-origin",
    };
    if (path.startsWith("/api/")) {
      if (!SESSION_TOKEN || SESSION_TOKEN === "__ROBOT_DASHBOARD_TOKEN__") {
        throw new Error("Dashboardens sessionsnyckel saknas.");
      }
      headers["X-Robot-Dashboard-Token"] = SESSION_TOKEN;
    }
    if (method === "POST" || method === "PUT") {
      headers["Content-Type"] = "application/json";
      request.body = JSON.stringify(options.body || {});
    }
    const controller = new AbortController();
    const timer = window.setTimeout(() => controller.abort(), options.timeout || 15000);
    request.signal = controller.signal;
    try {
      const response = await fetch(path, request);
      const raw = await response.text();
      let payload = {};
      if (raw) {
        try {
          payload = JSON.parse(raw);
        } catch (_error) {
          throw new Error("Servern returnerade ogiltig JSON.");
        }
      }
      if (!response.ok) {
        const error = safeObject(payload.error);
        const requestError = new Error(
          safeText(error.message, `HTTP ${response.status}`),
        );
        requestError.status = response.status;
        requestError.code = safeText(error.code, "http_error");
        throw requestError;
      }
      return payload;
    } catch (error) {
      if (error && error.name === "AbortError") {
        throw new Error("Den lokala servern svarade inte inom tidsgränsen.");
      }
      throw error;
    } finally {
      window.clearTimeout(timer);
    }
  }

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
      ? "konfigurerad modell ej laddad"
      : lmState === "online"
        ? safeText(lmStudio.model, "ansluten")
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

  function enforceCapabilities(bootstrap) {
    const capabilities = safeObject(bootstrap.capabilities);
    state.readOnlyInvariant = (
      bootstrap.physical_control_enabled === false
      && capabilities.physical_control === false
      && capabilities.ssh === false
      && capabilities.tts === false
    );
    setStatus(
      "status-motion",
      state.readOnlyInvariant ? "locked" : "fault",
      state.readOnlyInvariant ? "låst" : "kontraktsbrott",
    );

    const turnActive = state.activeTurn
      && !TERMINAL_TURN_STATES.has(state.activeTurn.status);
    const chatEnabled = capabilities.chat === true
      && state.readOnlyInvariant
      && !turnActive
      && state.modelReady === true;
    byId("message-input").disabled = !chatEnabled;
    byId("send-button").disabled = !chatEnabled;
    byId("new-conversation-button").disabled = !chatEnabled;
    const researchOption = byId("turn-mode").querySelector('option[value="research_required"]');
    const researchTools = safeArray(capabilities.research);
    researchOption.disabled = !researchTools.includes("weather.current");
    setStatus(
      "status-research",
      researchOption.disabled ? "idle" : "ready",
      researchOption.disabled ? "inga verktyg" : "weather.current",
    );
    byId("composer-status").textContent = chatEnabled
      ? "Redo · modellresultat saknar fysisk auktoritet"
      : state.modelReady === false
        ? "LM Studio eller den konfigurerade modellen är inte redo"
        : "Chatt är inte tillgänglig";
    if (!state.readOnlyInvariant) {
      showToast("Servern bröt mot dashboardens read-only-kontrakt. Mutationer har stängts.", true);
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
    copy.appendChild(createElement("strong", "", safeText(node.display_name, safeText(node.node_id, "Namnlös nod"))));
    const identity = [
      safeText(node.node_kind, root ? "robot" : "nod"),
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
        display_name: "Host & providers",
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
      copy.appendChild(createElement("strong", "", "Kameror och mikrofoner"));
      copy.appendChild(createElement("small", "", "framtida perceptionskällor"));
      future.appendChild(copy);
      future.appendChild(createElement("span", "node-state", "ej konfigurerade"));
      tree.appendChild(future);
    }
    const controller = nodes.find((node) => node.node_kind === "controller") || nodes[0] || {};
    const details = byId("controller-details");
    details.replaceChildren();
    [
      ["Tillstånd", humanState(controller.lifecycle)],
      ["Controller ID", safeText(controller.controller_id)],
      ["Instance ID", safeText(controller.controller_instance_id)],
      ["Senast observerad", formatDateTime(controller.last_observed_at_unix_ms)],
      ["Statusorsak", safeText(controller.status_reason_code)],
      ["Fysiska funktioner", controller.control_exposed === true ? "Avvisade" : "Låsta"],
    ].forEach(([label, value]) => {
      const row = createElement("div");
      row.appendChild(createElement("dt", "", label));
      row.appendChild(createElement("dd", "", value));
      details.appendChild(row);
    });
    byId("fleet-aggregate-status").textContent = robots.length > 0
      ? humanState(robots[0].lifecycle)
      : "Inte observerad";
    if (nodes.some((node) => node.control_exposed === true)) {
      state.readOnlyInvariant = false;
      setStatus("status-motion", "fault", "capability avvisad");
    }
  }

  function renderExperiments(experiments) {
    state.experiments = safeArray(experiments);
    if (state.experiments.length === 0) {
      return;
    }
    const list = byId("experiment-list");
    list.replaceChildren();
    state.experiments.forEach((experiment) => {
      const card = createElement("article", "experiment-card");
      card.appendChild(createElement("div", "experiment-id", safeText(experiment.experiment_id, "EXP-—")));
      const copy = createElement("div");
      copy.appendChild(createElement("h3", "", safeText(experiment.title, "Namnlöst experiment")));
      copy.appendChild(createElement("p", "", safeText(experiment.summary, "Ingen sammanfattning.")));
      card.appendChild(copy);
      card.appendChild(createElement("span", "state-chip state-idle", humanState(experiment.status)));
      list.appendChild(card);
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
    byId("save-settings-button").disabled = !dirty || !state.readOnlyInvariant;
    byId("reset-settings-button").disabled = !dirty;
    byId("settings-status").textContent = dirty
      ? "Osparade lokala ändringar."
      : "Inga osparade ändringar.";
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
    byId("settings-revision").textContent = `Revision ${safeInteger(settings.revision, "—")}`;
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
    byId("settings-status").textContent = "Sparar och validerar…";
    try {
      const payload = await api("/api/v1/settings", {
        method: "PUT",
        body: {
          expected_revision: state.originalSettings.revision,
          changes,
        },
      });
      renderSettings(payload.settings);
      showToast("Inställningarna sparades lokalt.");
    } catch (error) {
      byId("settings-status").textContent = safeText(error.message, "Inställningarna kunde inte sparas.");
      updateSettingsDirtyState();
      showToast(safeText(error.message, "Inställningarna kunde inte sparas."), true);
    }
  }

  function resetSettings() {
    renderSettings(state.originalSettings);
  }

  function renderMessage(message, extraClass = "") {
    const role = message.role === "user" ? "user" : message.role === "assistant" ? "assistant" : "system";
    const article = createElement("article", `message message-${role}${extraClass ? ` ${extraClass}` : ""}`);
    const header = createElement("div", "message-header");
    header.appendChild(createElement("span", "message-author", role === "user" ? "Du" : role === "assistant" ? "Gemma" : "System"));
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
              safeText(
                error.message,
                "Historisk evidens kunde inte hämtas.",
              ),
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
    header.appendChild(createElement("span", "message-author", "Gemma"));
    header.appendChild(createElement("span", "message-mode", humanState(state.activeTurn && state.activeTurn.status)));
    article.appendChild(header);
    const body = createElement("div", "message-body", "Arbetar");
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
    const keepAtBottom = (
      feed.scrollHeight - feed.scrollTop - feed.clientHeight < 80
    );
    const messages = state.conversation ? safeArray(state.conversation.messages) : [];
    feed.replaceChildren();
    if (messages.length === 0 && !state.optimisticContent && !state.activeTurn) {
      feed.appendChild(welcomeMessage);
    } else {
      messages.forEach((message) => feed.appendChild(renderMessage(message)));
      if (state.optimisticContent) {
        feed.appendChild(renderMessage({
          role: "user",
          content: state.optimisticContent,
          created_at_unix_ms: Date.now(),
          citation_ids: [],
        }));
      }
      if (state.activeTurn && !TERMINAL_TURN_STATES.has(state.activeTurn.status)) {
        feed.appendChild(renderPendingMessage());
      } else if (state.activeTurn && state.activeTurn.status === "failed") {
        feed.appendChild(renderMessage({
          role: "system",
          content: `Episoden avbröts: ${safeText(state.activeTurn.error_code, "okänt fel")}`,
          created_at_unix_ms: state.activeTurn.completed_at_unix_ms,
          citation_ids: [],
        }, "message-error"));
      }
    }
    if (keepAtBottom) {
      feed.scrollTop = feed.scrollHeight;
    }
    renderContext();
  }

  function renderContext() {
    const details = byId("context-details");
    details.replaceChildren();
    const conversation = safeObject(state.conversation);
    [
      ["Conversation ID", safeText(conversation.conversation_id, "Inte skapad")],
      ["Context version", safeInteger(conversation.version, "—")],
      ["Kontextläge", safeText(conversation.context_mode, "typed_history")],
      ["Antal turer", safeArray(conversation.messages).filter((message) => message.role === "user").length],
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
    statusBadge.textContent = turn ? humanState(status) : "Väntar";
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
      copy.appendChild(createElement("strong", "", turn ? humanState(status) : "Ingen episod ännu"));
      copy.appendChild(createElement("p", "", turn ? safeText(turn.turn_id) : "Typade beslut och verktygsanrop visas här."));
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
      safeInteger(episode.planner_turns, "—"),
      safeInteger(episode.tool_calls, "—"),
      safeInteger(episode.replans, "—"),
      safeInteger(episode.final_context_version, "—"),
    ] : ["—", "—", "—", "—"];
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
      list.appendChild(createElement("p", "", "Ingen extern evidens har hämtats."));
      list.appendChild(createElement("small", "", "Researchresultat visas med hostmyntade evidence-ID:n."));
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
          || "Verifierad citation från den skrivskyddade researchloopen.",
      ));
      card.appendChild(createElement(
        "small",
        "",
        [
          safeText(provenance.provider, safeText(envelope.tool_name, "read-only")),
          `TTL till monotonic ${safeInteger(envelope.valid_until_monotonic_ms, "—")}`,
          safeText(provenance.raw_sha256, "").slice(0, 12),
        ].filter(Boolean).join(" · "),
      ));
      list.appendChild(card);
    });
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
        ? "LM Studio eller den konfigurerade modellen är inte redo"
        : state.activeTurn.status === "answered"
          ? "Svar verifierat · redo för nästa tur"
          : state.activeTurn.status === "clarification_required"
            ? "Ett förtydligande behövs"
            : `Episoden stoppades · ${safeText(state.activeTurn.error_code, "okänt fel")}`;
      if (visibleStateChanged) {
        byId("chat-announcer").textContent = state.activeTurn.status === "answered"
          ? `Gemma svarade: ${safeText(state.activeTurn.answer_text, "")}`
          : state.activeTurn.status === "clarification_required"
            ? `Gemma behöver ett förtydligande: ${safeText(state.activeTurn.clarification_question, "")}`
            : "Agentepisoden stoppades utan svar.";
      }
    } else {
      byId("send-button").disabled = true;
      byId("message-input").disabled = true;
      byId("composer-status").textContent = `${humanState(state.activeTurn.status)} · ${safeText(state.activeTurn.turn_id)}`;
    }
  }

  async function createConversation() {
    const payload = await api("/api/v1/conversations", {
      method: "POST",
      body: {},
    });
    state.conversation = safeObject(payload.conversation);
    state.activeTurn = null;
    state.optimisticContent = null;
    byId("conversation-subtitle").textContent = `typed_history · version ${safeInteger(state.conversation.version, 1)}`;
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
    byId("conversation-subtitle").textContent = `${safeText(state.conversation.context_mode, "typed_history")} · version ${safeInteger(state.conversation.version, "—")}`;
    renderConversation();
  }

  async function startNewConversation() {
    if (state.activeTurn && !TERMINAL_TURN_STATES.has(state.activeTurn.status)) {
      showToast("Vänta tills den pågående episoden är terminal.", true);
      return;
    }
    byId("new-conversation-button").disabled = true;
    try {
      await createConversation();
      byId("message-input").focus();
      showToast("Ny lokal konversation skapad.");
    } catch (error) {
      showToast(safeText(error.message, "Konversationen kunde inte skapas."), true);
    } finally {
      enforceCapabilities(safeObject(state.bootstrap));
    }
  }

  async function pollTurn(turnId, generation) {
    if (generation !== state.turnPollGeneration) {
      return;
    }
    try {
      const payload = await api(`/api/v1/turns/${encodeURIComponent(turnId)}`);
      if (generation !== state.turnPollGeneration) {
        return;
      }
      const turn = safeObject(payload.turn);
      state.turnPollFailures = 0;
      renderTurn(turn);
      if (TERMINAL_TURN_STATES.has(turn.status)) {
        await refreshConversation();
        renderTurn(turn);
        return;
      }
    } catch (error) {
      state.turnPollFailures += 1;
      byId("composer-status").textContent = safeText(error.message, "Turn-polling misslyckades.");
      if (state.turnPollFailures >= 8) {
        state.turnPollGeneration += 1;
        renderTurn({
          ...safeObject(state.activeTurn),
          status: "failed",
          error_code: "turn_poll_failed",
          completed_at_unix_ms: Date.now(),
        });
        showToast(
          "Turnstatus kunde inte återhämtas. Starta en ny konversation eller ladda om sidan.",
          true,
        );
        return;
      }
    }
    const retryDelay = Math.min(
      5000,
      800 * Math.max(1, state.turnPollFailures),
    );
    window.setTimeout(() => pollTurn(turnId, generation), retryDelay);
  }

  async function submitTurn(event) {
    event.preventDefault();
    const input = byId("message-input");
    const content = input.value.trim();
    if (!content || !state.readOnlyInvariant) {
      return;
    }
    if (state.activeTurn && !TERMINAL_TURN_STATES.has(state.activeTurn.status)) {
      showToast("En episod pågår redan.", true);
      return;
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
        },
        timeout: 20000,
      });
      input.value = "";
      state.optimisticContent = null;
      renderTurn(payload.turn);
      await refreshConversation();
      state.turnPollGeneration += 1;
      state.turnPollFailures = 0;
      pollTurn(payload.turn.turn_id, state.turnPollGeneration);
    } catch (error) {
      state.optimisticContent = null;
      renderConversation();
      enforceCapabilities(safeObject(state.bootstrap));
      showToast(safeText(error.message, "Meddelandet kunde inte skickas."), true);
    }
  }

  function eventTime(event) {
    return safeInteger(event.recorded_at_unix_ms, safeInteger(event.occurred_at_unix_ms, null));
  }

  function eventVisible(event) {
    const severity = byId("event-severity-filter").value;
    const category = byId("event-plane-filter").value;
    const search = byId("event-search-input").value.trim().toLocaleLowerCase("sv-SE");
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
    ].some((value) => typeof value === "string" && value.toLocaleLowerCase("sv-SE").includes(search));
  }

  function openEventDetail(event) {
    state.selectedEvent = event;
    const source = safeObject(event.source);
    const correlation = safeObject(event.correlation);
    byId("event-detail-heading").textContent = safeText(event.event_type, "Händelsedetaljer");
    const fields = byId("event-detail-fields");
    fields.replaceChildren();
    [
      ["Event ID", safeText(event.event_id)],
      ["Sekvens", safeInteger(event.sequence, "—")],
      ["Tid", formatDateTime(eventTime(event))],
      ["Nivå", safeText(event.level)],
      ["Kategori", safeText(event.category)],
      ["Source ID", safeText(source.source_id)],
      ["Robot ID", safeText(source.robot_id)],
      ["Node ID", safeText(source.node_id)],
      ["Conversation ID", safeText(correlation.conversation_id)],
      ["Turn ID", safeText(correlation.turn_id)],
      ["Tool call ID", safeText(correlation.tool_call_id)],
      ["Request ID", safeText(correlation.request_id)],
    ].forEach(([label, value]) => {
      const row = createElement("div");
      row.appendChild(createElement("dt", "", label));
      row.appendChild(createElement("dd", "", value));
      fields.appendChild(row);
    });
    byId("event-detail-json").textContent = JSON.stringify(event, null, 2);
    byId("event-detail").hidden = false;
    byId("close-event-detail").focus();
  }

  function renderEvents() {
    const body = byId("event-table-body");
    body.replaceChildren();
    const visible = state.events.filter(eventVisible).slice().reverse();
    if (visible.length === 0) {
      const row = createElement("tr", "empty-row");
      const cell = createElement("td", "", state.events.length === 0
        ? "Väntar på den första tekniska händelsen."
        : "Inga händelser matchar filtret.");
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
      button.setAttribute("aria-label", `Visa ${safeText(event.event_type, "händelse")}`);
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
    const gapNotice = byId("event-gap-notice");
    if (payload.gap === true || safeInteger(payload.dropped_total, 0) > 0) {
      gapNotice.hidden = false;
      gapNotice.textContent = `Loggströmmen har ett gap. Totalt bortfall: ${safeInteger(payload.dropped_total, 0)}.`;
    } else {
      gapNotice.hidden = true;
    }
    renderEvents();
    if (state.activeTurn) {
      renderActivity(state.activeTurn);
      renderEvidence(state.activeTurn);
    }
  }

  async function pollEvents() {
    if (!state.eventsPaused) {
      try {
        const payload = await api(`/api/v1/events?after_sequence=${state.afterSequence}&limit=${EVENT_LIMIT}`);
        ingestEvents(payload);
        const label = byId("event-stream-label");
        label.textContent = `Live · sekvens ${state.afterSequence}`;
        label.parentElement.classList.remove("is-offline");
      } catch (error) {
        const label = byId("event-stream-label");
        label.textContent = "Loggserver offline";
        label.parentElement.classList.add("is-offline");
      }
    }
    window.setTimeout(pollEvents, document.hidden ? 5000 : 1500);
  }

  function toggleEventsPaused() {
    state.eventsPaused = !state.eventsPaused;
    const button = byId("pause-events-button");
    button.setAttribute("aria-pressed", state.eventsPaused ? "true" : "false");
    button.textContent = state.eventsPaused ? "Återuppta ström" : "Pausa ström";
    const label = byId("event-stream-label");
    label.textContent = state.eventsPaused ? "Pausad lokalt" : "Återansluter…";
    label.parentElement.classList.toggle("is-paused", state.eventsPaused);
  }

  function exportEvents() {
    if (state.events.length === 0) {
      showToast("Det finns inga lokala events att exportera.", true);
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
    showToast("Den redigerade lokala eventbufferten exporterades.");
  }

  function updateModeCopy() {
    const research = byId("turn-mode").value === "research_required";
    byId("mode-capability-note").textContent = research
      ? "Skrivskyddad evidens · weather.current"
      : "Read-only verktyg kan väljas semantiskt · ingen fysisk capability";
  }

  async function probeLMStudio() {
    const button = byId("probe-lm-studio-button");
    button.disabled = true;
    byId("probe-result").textContent = "Kontrollerar den lokala runtime-processen…";
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
      byId("probe-result").dataset.status = safeText(lmStudio.state, "unknown");
      byId("probe-result").textContent = lmStudio.configured_model_loaded === false
        ? `${humanState(lmStudio.state)} · konfigurerad modell ej laddad`
        : `${humanState(lmStudio.state)} · ${safeText(lmStudio.model, "ingen modell")}`;
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
      byId("probe-result").dataset.status = "offline";
      byId("probe-result").textContent = safeText(error.message, "Anslutningskontrollen misslyckades.");
    } finally {
      button.disabled = false;
    }
  }

  async function refreshBootstrap(silent = true) {
    if (state.bootstrapBusy) {
      return;
    }
    state.bootstrapBusy = true;
    try {
      const payload = await api("/api/v1/bootstrap");
      renderBootstrap(payload);
    } catch (error) {
      setStatus("status-lm-studio", "offline", "dashboard offline");
      if (!silent) {
        showToast(safeText(error.message, "Dashboardservern kunde inte nås."), true);
      }
    } finally {
      state.bootstrapBusy = false;
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
    byId("message-input").addEventListener("keydown", (event) => {
      if (event.key === "Enter" && !event.shiftKey) {
        event.preventDefault();
        byId("composer-form").requestSubmit();
      }
    });
    byId("turn-mode").addEventListener("change", updateModeCopy);
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
        showToast("Händelsens JSON kopierades.");
      } catch (_error) {
        showToast("JSON kunde inte kopieras.", true);
      }
    });
    document.addEventListener("keydown", (event) => {
      if ((event.metaKey || event.ctrlKey) && event.key.toLocaleLowerCase("sv-SE") === "j") {
        event.preventDefault();
        activateView("events");
      }
      if (event.key === "Escape") {
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
    bindInteractions();
    try {
      const [bootstrapPayload, settingsPayload] = await Promise.all([
        api("/api/v1/bootstrap"),
        api("/api/v1/settings"),
      ]);
      renderBootstrap(bootstrapPayload);
      renderSettings(settingsPayload.settings || bootstrapPayload.settings);
      probeLMStudio();
    } catch (error) {
      byId("composer-status").textContent = "Dashboardservern kunde inte startas.";
      showToast(safeText(error.message, "Dashboardservern kunde inte startas."), true);
    }
    pollEvents();
    window.setInterval(() => refreshBootstrap(true), 10000);
  }

  initialize();
})();
