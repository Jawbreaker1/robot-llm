((global) => {
  "use strict";

  const CONTROL_STATES = Object.freeze([
    "DISABLED",
    "IDLE",
    "STARTING",
    "RUNNING",
    "STOPPING",
    "FAULTED",
  ]);
  const ACTIVE_STATES = new Set(["STARTING", "RUNNING", "STOPPING"]);
  const CONVERSATION_VIEWS = Object.freeze(["robot", "workbench"]);
  const CONVERSATION_TARGETS = Object.freeze(["robot", "workbench"]);
  const POLL_ACTIVE_MS = 400;
  const POLL_IDLE_MS = 2000;

  function checkedConversationValue(value, allowed, name) {
    if (!allowed.includes(value)) {
      throw new TypeError(`${name} is invalid`);
    }
    return value;
  }

  function defaultConversationTarget(view) {
    return checkedConversationValue(
      view,
      CONVERSATION_VIEWS,
      "Conversation view",
    ) === "robot" ? "robot" : "workbench";
  }

  function createConversationTargetState(initialView = "workbench") {
    let view = checkedConversationValue(
      initialView,
      CONVERSATION_VIEWS,
      "Conversation view",
    );
    const overrides = {
      robot: null,
      workbench: null,
    };

    function selected() {
      return overrides[view] || defaultConversationTarget(view);
    }

    return Object.freeze({
      clearOverride: () => {
        overrides[view] = null;
        return selected();
      },
      override: (target) => {
        overrides[view] = checkedConversationValue(
          target,
          CONVERSATION_TARGETS,
          "Conversation target",
        );
        return selected();
      },
      selectView: (nextView) => {
        view = checkedConversationValue(
          nextView,
          CONVERSATION_VIEWS,
          "Conversation view",
        );
        return selected();
      },
      selected,
      snapshot: () => Object.freeze({
        overrides: Object.freeze({ ...overrides }),
        selected: selected(),
        view,
      }),
    });
  }

  function safeObject(value) {
    return value && typeof value === "object" && !Array.isArray(value)
      ? value
      : {};
  }

  function safeArray(value) {
    return Array.isArray(value) ? value : [];
  }

  function safeText(value, fallback = "—") {
    return typeof value === "string" && value.length > 0
      ? value
      : fallback;
  }

  function physicalTurnControl(value) {
    const turn = safeObject(value);
    if (turn.intent === "STOP_TASK") {
      const stoppedControl = safeObject(turn.control);
      return Object.keys(stoppedControl).length > 0 ? stoppedControl : null;
    }
    if (turn.intent !== "PHYSICAL_TASK") {
      return null;
    }
    const control = safeObject(safeObject(turn.episode).control);
    return Object.keys(control).length > 0 ? control : null;
  }

  function normalizeControl(value) {
    const source = safeObject(value);
    const state = CONTROL_STATES.includes(source.state)
      ? source.state
      : "DISABLED";
    const settings = safeObject(source.settings);
    const episode = safeObject(source.episode);
    const runtime = safeObject(source.runtime);
    return {
      schema: safeText(source.schema, "robot-control/v1"),
      sequence: Number.isSafeInteger(source.sequence) ? source.sequence : 0,
      state,
      enabled: source.enabled === true,
      accepting: source.accepting === true,
      settings: {
        revision: Number.isSafeInteger(settings.revision) ? settings.revision : 0,
        model: safeText(settings.model, ""),
        max_episode_ms: Number.isSafeInteger(settings.max_episode_ms)
          ? settings.max_episode_ms
          : 900000,
        speech_enabled: settings.speech_enabled === true,
      },
      episode: {
        episode_id: safeText(episode.episode_id, ""),
        goal: safeText(episode.goal, ""),
        locale: safeText(episode.locale, ""),
        terminal_reason: safeText(episode.terminal_reason, ""),
      },
      runtime: {
        current_action: safeText(runtime.current_action, ""),
        obstacle: safeObject(runtime.obstacle),
        plan: safeArray(runtime.plan).filter((item) => typeof item === "string"),
        scan: safeObject(runtime.scan),
        model_latency_ms: Number.isSafeInteger(runtime.model_latency_ms)
          ? runtime.model_latency_ms
          : null,
        planner_context_bytes: Number.isSafeInteger(
          runtime.planner_context_bytes,
        ) ? runtime.planner_context_bytes : null,
        prompt_tokens: Number.isSafeInteger(runtime.prompt_tokens)
          ? runtime.prompt_tokens
          : null,
        completion_tokens: Number.isSafeInteger(runtime.completion_tokens)
          ? runtime.completion_tokens
          : null,
        total_tokens: Number.isSafeInteger(runtime.total_tokens)
          ? runtime.total_tokens
          : null,
        speech_status: safeText(runtime.speech_status, "idle"),
        speech_error_code: safeText(runtime.speech_error_code, ""),
        message: safeText(runtime.message, ""),
      },
      last_error_code: safeText(source.last_error_code, ""),
    };
  }

  function composerPolicy(control, target, chatEnabled, busy = false) {
    const robotTarget = target === "robot";
    const robotReady = (
      control.enabled
      && control.accepting
      && !busy
    );
    return {
      robotTarget,
      composerEnabled: robotTarget ? robotReady : chatEnabled === true,
      robotInputEnabled: robotReady,
      robotStartEnabled: robotReady,
      turnModeEnabled: !robotTarget && chatEnabled === true,
      newConversationEnabled: !robotTarget && chatEnabled === true,
    };
  }

  function shouldApplySnapshot(currentValue, candidateValue) {
    const current = normalizeControl(currentValue);
    const candidate = normalizeControl(candidateValue);
    return candidate.sequence >= current.sequence;
  }

  function preferredInitialTarget(controlValue, userSelectedTarget) {
    if (typeof userSelectedTarget !== "boolean") {
      throw new TypeError("Target-selection state is invalid");
    }
    const candidate = normalizeControl(controlValue);
    const robotReady = (
      candidate.enabled
      && candidate.accepting
      && candidate.state === "IDLE"
    );
    return robotReady && !userSelectedTarget
      ? "robot"
      : "workbench";
  }

  function create(options) {
    if (
      !options
      || !options.document
      || typeof options.request !== "function"
      || typeof options.translate !== "function"
      || typeof options.randomId !== "function"
      || typeof options.showToast !== "function"
      || typeof options.getLocale !== "function"
      || !options.sessionGuard
      || typeof options.sessionGuard.subscribe !== "function"
      || typeof options.sessionGuard.isExpired !== "function"
      || !global.RobotMissionPanelUI
      || typeof global.RobotMissionPanelUI.create !== "function"
    ) {
      throw new Error("Robot control UI configuration is invalid");
    }
    const document = options.document;
    const request = options.request;
    const translate = options.translate;
    const randomId = options.randomId;
    const showToast = options.showToast;
    const getLocale = options.getLocale;
    const sessionGuard = options.sessionGuard;
    const formatError = typeof options.formatError === "function"
      ? options.formatError
      : () => translate("errors.generic");
    const onAvailabilityChanged = (
      typeof options.onAvailabilityChanged === "function"
        ? options.onAvailabilityChanged
        : () => {}
    );
    const onInputAccepted = typeof options.onInputAccepted === "function"
      ? options.onInputAccepted
      : () => {};
    const onTargetChanged = typeof options.onTargetChanged === "function"
      ? options.onTargetChanged
      : () => {};
    const byId = (id) => document.getElementById(id);
    const missionPanel = global.RobotMissionPanelUI.create({
      document,
      request,
      translate,
      getLocale,
    });
    let control = normalizeControl({});
    let busy = false;
    let interpretingInput = false;
    let settingsDirty = false;
    let chatEnabled = false;
    let pollTimer = null;
    let stopped = false;
    let userSelectedTarget = false;
    const conversationTarget = createConversationTargetState(
      options.initialConversationView || "workbench",
    );

    function selectedTarget() {
      return conversationTarget.selected();
    }

    function syncTargetSelector() {
      byId("composer-target").value = selectedTarget();
    }

    function overrideTarget(target) {
      conversationTarget.override(target);
      syncTargetSelector();
      renderComposer();
      onTargetChanged(selectedTarget());
      return selectedTarget();
    }

    function selectConversationView(view) {
      conversationTarget.selectView(view);
      syncTargetSelector();
      renderComposer();
      onTargetChanged(selectedTarget());
      return selectedTarget();
    }

    function compactFact(value, preferredKeys) {
      const fact = safeObject(value);
      for (const key of preferredKeys) {
        if (
          typeof fact[key] === "string"
          || Number.isFinite(fact[key])
        ) {
          return String(fact[key]);
        }
      }
      const keys = Object.keys(fact);
      if (keys.length === 0) {
        return translate("common.none");
      }
      return keys.slice(0, 3).map((key) => (
        `${key}: ${String(fact[key])}`
      )).join(" · ");
    }

    function stateTranslation(state) {
      const key = `robot.state.${state}`;
      const value = translate(key);
      return value === key ? state : value;
    }

    function renderRuntime() {
      const runtime = control.runtime;
      byId("robot-current-action").textContent = safeText(
        runtime.current_action,
        translate("common.none"),
      );
      byId("robot-obstacle").textContent = compactFact(
        runtime.obstacle,
        ["label", "target_id", "distance_mm", "relation"],
      );
      byId("robot-plan").textContent = runtime.plan.length > 0
        ? runtime.plan.join(" → ")
        : translate("common.none");
      byId("robot-scan").textContent = compactFact(
        runtime.scan,
        ["state", "target_id", "result", "direction"],
      );
      byId("robot-model-latency").textContent = (
        runtime.model_latency_ms === null
          ? translate("common.missing")
          : [
            `${runtime.model_latency_ms} ms`,
            runtime.planner_context_bytes === null
              ? null
              : `${runtime.planner_context_bytes} B`,
            runtime.total_tokens === null
              ? null
              : `${runtime.total_tokens} tok`,
          ].filter(Boolean).join(" · ")
      );
      const speechKey = `robot.speech.${runtime.speech_status}`;
      const speechValue = translate(speechKey);
      const speechLabel = speechValue === speechKey
        ? runtime.speech_status
        : speechValue;
      byId("robot-speech-status").textContent = (
        runtime.speech_status === "failed" && runtime.speech_error_code
          ? `${speechLabel} · ${runtime.speech_error_code}`
          : speechLabel
      );
    }

    function renderSettings(force = false) {
      const idle = (
        control.enabled
        && control.state === "IDLE"
        && !busy
        && !stopped
      );
      const model = byId("robot-setting-model");
      const maximum = byId("robot-setting-max-episode-ms");
      const speech = byId("robot-setting-speech-enabled");
      if (!settingsDirty || force) {
        model.value = control.settings.model;
        maximum.value = String(control.settings.max_episode_ms);
        speech.checked = control.settings.speech_enabled;
        settingsDirty = false;
      }
      model.disabled = !idle;
      maximum.disabled = !idle;
      speech.disabled = !idle;
      byId("robot-save-settings-button").disabled = !idle || !settingsDirty;
      if (!control.enabled) {
        byId("robot-settings-status").textContent = translate(
          "robot.settings.runtime_missing",
        );
      } else if (control.state !== "IDLE") {
        byId("robot-settings-status").textContent = translate(
          "robot.settings.idle_only",
        );
      } else if (!settingsDirty) {
        byId("robot-settings-status").textContent = translate(
          "robot.settings.revision",
          { revision: control.settings.revision },
        );
      }
    }

    function renderGlobalStatus() {
      const motionStatus = byId("status-motion");
      const stateStatus = control.state === "FAULTED"
        ? "fault"
        : ACTIVE_STATES.has(control.state)
          ? "busy"
          : control.state === "IDLE"
            ? "ready"
            : "offline";
      if (motionStatus) {
        motionStatus.dataset.status = stateStatus;
        const value = motionStatus.querySelector(".status-value");
        if (value) {
          value.textContent = control.enabled
            ? stateTranslation(control.state)
            : translate("robot.summary.runtime_missing");
        }
      }
    }

    function renderComposer() {
      const policy = composerPolicy(
        control,
        selectedTarget(),
        chatEnabled,
        busy || stopped,
      );
      const input = byId("message-input");
      const hasGoal = input.value.trim().length > 0;
      input.disabled = !policy.composerEnabled;
      byId("send-button").disabled = !policy.composerEnabled;
      byId("new-conversation-button").disabled = (
        !policy.newConversationEnabled
      );
      byId("new-conversation-button").hidden = policy.robotTarget;
      byId("turn-mode").disabled = !policy.turnModeEnabled;
      byId("turn-mode-control").hidden = policy.robotTarget;
      byId("send-button").querySelector(".button-label").textContent = (
        policy.robotTarget
          ? translate("robot.actions.start")
          : translate("workbench.composer.send")
      );
      byId("send-button").disabled = !policy.composerEnabled || !hasGoal;
      byId("mode-capability-note").textContent = policy.robotTarget
        ? translate("robot.composer.robot_note")
        : translate("robot.composer.workbench_note");
      byId("composer-status").textContent = policy.robotTarget
        ? (
          interpretingInput
            ? translate("robot.composer.interpreting")
            : policy.composerEnabled
            ? translate("robot.composer.ready")
            : translate("robot.composer.unavailable", {
              state: stateTranslation(control.state),
            })
        )
        : (
          chatEnabled
            ? translate("capability.chat_ready")
            : translate("capability.chat_unavailable")
        );
      onAvailabilityChanged(policy.composerEnabled);
    }

    function render() {
      const stateNode = byId("robot-control-state");
      stateNode.textContent = control.state;
      stateNode.dataset.state = control.state;
      stateNode.className = "state-chip";
      stateNode.classList.add(
        control.state === "IDLE"
          ? "state-ready"
          : control.state === "FAULTED"
            ? "state-fault"
            : ACTIVE_STATES.has(control.state)
              ? "state-running"
              : "state-idle",
      );
      byId("robot-control-summary").textContent = (
        control.last_error_code
          ? translate("robot.summary.fault", {
            code: control.last_error_code,
          })
          : control.runtime.message
            ? control.runtime.message
            : control.enabled
              ? translate("robot.summary.state", {
                state: stateTranslation(control.state),
              })
              : translate("robot.summary.runtime_missing")
      );
      const active = ACTIVE_STATES.has(control.state);
      const faulted = control.state === "FAULTED";
      const stopButton = byId("robot-stop-button");
      stopButton.disabled = busy || !(active || faulted);
      stopButton.textContent = translate(
        faulted
          ? "robot.actions.acknowledge_fault"
          : "robot.actions.stop",
      );
      byId("robot-emergency-stop-button").disabled = (
        stopped || !control.enabled
      );
      renderRuntime();
      renderSettings();
      renderGlobalStatus();
      renderComposer();
    }

    function setControl(value, forceSettings = false) {
      const next = normalizeControl(value);
      if (!shouldApplySnapshot(control, next)) {
        return false;
      }
      control = next;
      renderSettings(forceSettings);
      render();
      missionPanel.setControl(value);
      return true;
    }

    async function refresh(silent = true) {
      if (stopped) {
        return;
      }
      try {
        const payload = await request("/api/v1/robot/status", {
          timeout: 5000,
        });
        setControl(payload.control);
      } catch (error) {
        if (!silent) {
          showToast(formatError(error), true);
        }
      }
    }

    function schedulePoll() {
      if (stopped) {
        return;
      }
      if (pollTimer !== null) {
        global.clearTimeout(pollTimer);
      }
      const delay = ACTIVE_STATES.has(control.state)
        ? POLL_ACTIVE_MS
        : POLL_IDLE_MS;
      pollTimer = global.setTimeout(async () => {
        await refresh(true);
        schedulePoll();
      }, delay);
    }

    function stopPolling() {
      if (stopped) {
        return;
      }
      stopped = true;
      chatEnabled = false;
      if (pollTimer !== null) {
        global.clearTimeout(pollTimer);
        pollTimer = null;
      }
      missionPanel.stopPolling();
      render();
    }

    async function submitInput(text, locale = getLocale()) {
      const cleanText = typeof text === "string" ? text.trim() : "";
      const policy = composerPolicy(
        control,
        "robot",
        chatEnabled,
        busy || stopped,
      );
      if (!cleanText || !policy.robotInputEnabled) {
        showToast(
          translate("robot.errors.not_ready", {
            state: stateTranslation(control.state),
          }),
          true,
        );
        return false;
      }
      interpretingInput = true;
      busy = true;
      render();
      try {
        const payload = await request("/api/v1/robot/turns", {
          method: "POST",
          body: {
            text: cleanText,
            locale,
            client_request_id: randomId("robot-ui"),
            expected_revision: control.settings.revision,
          },
          // The backend gives interactive input its own short deadline. Keep
          // the browser request alive beyond the configurable upper bound so
          // it never reports a timeout while the server can still act.
          timeout: 65000,
        });
        const turn = safeObject(payload.turn);
        const nextControl = physicalTurnControl(turn);
        if (nextControl) {
          setControl(nextControl);
        }
        onInputAccepted(cleanText, turn);
        showToast(translate(
          turn.intent === "PHYSICAL_TASK"
            ? "robot.toasts.started"
            : turn.intent === "STOP_TASK"
              ? "robot.toasts.stop_requested"
              : "robot.toasts.input_accepted",
        ));
        return true;
      } catch (error) {
        showToast(formatError(error), true);
        await refresh(true);
        return false;
      } finally {
        interpretingInput = false;
        busy = false;
        render();
      }
    }

    async function command(path, successKey) {
      busy = true;
      render();
      try {
        const payload = await request(path, {
          method: "POST",
          body: {},
          timeout: 10000,
        });
        setControl(payload.control);
        showToast(translate(successKey));
      } catch (error) {
        showToast(formatError(error), true);
        await refresh(true);
      } finally {
        busy = false;
        render();
      }
    }

    async function saveSettings(event) {
      event.preventDefault();
      if (!settingsDirty || control.state !== "IDLE") {
        return;
      }
      const changes = {
        model: byId("robot-setting-model").value.trim(),
        max_episode_ms: Number(
          byId("robot-setting-max-episode-ms").value,
        ),
        speech_enabled: byId(
          "robot-setting-speech-enabled",
        ).checked,
      };
      busy = true;
      render();
      byId("robot-settings-status").textContent = translate(
        "robot.settings.saving",
      );
      try {
        const payload = await request("/api/v1/robot/settings", {
          method: "PUT",
          body: {
            expected_revision: control.settings.revision,
            changes,
          },
        });
        control.settings = normalizeControl({
          ...control,
          settings: payload.settings,
        }).settings;
        settingsDirty = false;
        renderSettings(true);
        showToast(translate("robot.toasts.settings_saved"));
      } catch (error) {
        showToast(formatError(error), true);
      } finally {
        busy = false;
        render();
      }
    }

    function markSettingsDirty() {
      settingsDirty = true;
      renderSettings();
    }

    async function initialize() {
      syncTargetSelector();
      sessionGuard.subscribe(stopPolling);
      if (stopped) {
        return;
      }
      byId("composer-target").addEventListener("change", (event) => {
        userSelectedTarget = true;
        overrideTarget(event.currentTarget.value);
      });
      byId("message-input").addEventListener("input", renderComposer);
      byId("robot-stop-button").addEventListener(
        "click",
        () => command(
          "/api/v1/robot/stop",
          "robot.toasts.stop_requested",
        ),
      );
      byId("robot-emergency-stop-button").addEventListener(
        "click",
        () => command(
          "/api/v1/robot/emergency-stop",
          "robot.toasts.emergency_stop_sent",
        ),
      );
      byId("robot-settings-form").addEventListener(
        "submit",
        saveSettings,
      );
      [
        "robot-setting-model",
        "robot-setting-max-episode-ms",
        "robot-setting-speech-enabled",
      ].forEach((id) => {
        byId(id).addEventListener("input", markSettingsDirty);
        byId(id).addEventListener("change", markSettingsDirty);
      });
      await refresh(false);
      if (stopped) {
        return;
      }
      if (
        preferredInitialTarget(control, userSelectedTarget) === "robot"
      ) {
        overrideTarget("robot");
      }
      renderSettings(true);
      void missionPanel.initialize();
      schedulePoll();
    }

    function reconcileComposer(nextChatEnabled) {
      chatEnabled = !stopped && nextChatEnabled === true;
      renderComposer();
      return composerPolicy(
        control,
        selectedTarget(),
        chatEnabled,
        busy || stopped,
      ).composerEnabled;
    }

    function renderLocale() {
      render();
      missionPanel.renderLocale();
    }

    return Object.freeze({
      initialize,
      isRobotTarget: () => selectedTarget() === "robot",
      overrideTarget,
      reconcileComposer,
      refresh,
      renderLocale,
      selectConversationView,
      selectedTarget,
      submitInput,
      stopPolling,
    });
  }

  global.RobotControlUI = Object.freeze({
    ACTIVE_STATES,
    CONVERSATION_TARGETS,
    CONVERSATION_VIEWS,
    CONTROL_STATES,
    composerPolicy,
    create,
    createConversationTargetState,
    defaultConversationTarget,
    normalizeControl,
    physicalTurnControl,
    preferredInitialTarget,
    shouldApplySnapshot,
  });
})(window);
