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
  const POLL_ACTIVE_MS = 400;
  const POLL_IDLE_MS = 2000;

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
        speech_status: safeText(runtime.speech_status, "idle"),
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
      && control.state === "IDLE"
      && !busy
    );
    return {
      robotTarget,
      composerEnabled: robotTarget ? robotReady : chatEnabled === true,
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

  function create(options) {
    if (
      !options
      || !options.document
      || typeof options.request !== "function"
      || typeof options.translate !== "function"
      || typeof options.randomId !== "function"
      || typeof options.showToast !== "function"
      || typeof options.getLocale !== "function"
    ) {
      throw new Error("Robot control UI configuration is invalid");
    }
    const document = options.document;
    const request = options.request;
    const translate = options.translate;
    const randomId = options.randomId;
    const showToast = options.showToast;
    const getLocale = options.getLocale;
    const formatError = typeof options.formatError === "function"
      ? options.formatError
      : () => translate("errors.generic");
    const onAvailabilityChanged = (
      typeof options.onAvailabilityChanged === "function"
        ? options.onAvailabilityChanged
        : () => {}
    );
    const onGoalAccepted = typeof options.onGoalAccepted === "function"
      ? options.onGoalAccepted
      : () => {};
    const byId = (id) => document.getElementById(id);
    let control = normalizeControl({});
    let busy = false;
    let settingsDirty = false;
    let chatEnabled = false;
    let pollTimer = null;

    function selectedTarget() {
      const selector = byId("composer-target");
      return selector && selector.value === "workbench"
        ? "workbench"
        : "robot";
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
          : `${runtime.model_latency_ms} ms`
      );
      const speechKey = `robot.speech.${runtime.speech_status}`;
      const speechValue = translate(speechKey);
      byId("robot-speech-status").textContent = speechValue === speechKey
        ? runtime.speech_status
        : speechValue;
    }

    function renderSettings(force = false) {
      const idle = control.enabled && control.state === "IDLE" && !busy;
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
      const robotStatus = byId("status-ev3");
      const motionStatus = byId("status-motion");
      const stateStatus = control.state === "FAULTED"
        ? "fault"
        : ACTIVE_STATES.has(control.state)
          ? "busy"
          : control.state === "IDLE"
            ? "ready"
            : "offline";
      if (robotStatus) {
        robotStatus.dataset.status = stateStatus;
        const value = robotStatus.querySelector(".status-value");
        if (value) {
          value.textContent = stateTranslation(control.state);
        }
      }
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
        busy,
      );
      const input = byId("message-input");
      const hasGoal = input.value.trim().length > 0;
      input.disabled = !policy.composerEnabled;
      byId("send-button").disabled = !policy.composerEnabled;
      byId("new-conversation-button").disabled = (
        !policy.newConversationEnabled
      );
      byId("turn-mode").disabled = !policy.turnModeEnabled;
      byId("robot-start-button").disabled = (
        !policy.robotStartEnabled || !hasGoal
      );
      byId("mode-capability-note").textContent = policy.robotTarget
        ? translate("robot.composer.robot_note")
        : translate("robot.composer.workbench_note");
      byId("composer-status").textContent = policy.robotTarget
        ? (
          policy.composerEnabled
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
      byId("robot-stop-button").disabled = busy || !active;
      byId("robot-emergency-stop-button").disabled = (
        !control.enabled
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
      return true;
    }

    async function refresh(silent = true) {
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

    async function startGoal(goal, locale = getLocale()) {
      const cleanGoal = typeof goal === "string" ? goal.trim() : "";
      const policy = composerPolicy(
        control,
        "robot",
        chatEnabled,
        busy,
      );
      if (!cleanGoal || !policy.robotStartEnabled) {
        showToast(
          translate("robot.errors.not_ready", {
            state: stateTranslation(control.state),
          }),
          true,
        );
        return false;
      }
      busy = true;
      render();
      try {
        const payload = await request("/api/v1/robot/episodes", {
          method: "POST",
          body: {
            goal: cleanGoal,
            locale,
            client_request_id: randomId("robot-ui"),
            expected_revision: control.settings.revision,
          },
          timeout: 20000,
        });
        const episode = safeObject(payload.episode);
        setControl(episode.control);
        onGoalAccepted(cleanGoal, episode);
        showToast(translate("robot.toasts.started"));
        return true;
      } catch (error) {
        showToast(formatError(error), true);
        await refresh(true);
        return false;
      } finally {
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
      byId("composer-target").addEventListener("change", renderComposer);
      byId("message-input").addEventListener("input", renderComposer);
      byId("robot-start-button").addEventListener("click", () => {
        byId("composer-target").value = "robot";
        startGoal(byId("message-input").value, getLocale());
      });
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
      renderSettings(true);
      schedulePoll();
    }

    function reconcileComposer(nextChatEnabled) {
      chatEnabled = nextChatEnabled === true;
      renderComposer();
      return composerPolicy(
        control,
        selectedTarget(),
        chatEnabled,
        busy,
      ).composerEnabled;
    }

    function renderLocale() {
      render();
    }

    return Object.freeze({
      initialize,
      isRobotTarget: () => selectedTarget() === "robot",
      reconcileComposer,
      refresh,
      renderLocale,
      startGoal,
    });
  }

  global.RobotControlUI = Object.freeze({
    ACTIVE_STATES,
    CONTROL_STATES,
    composerPolicy,
    create,
    normalizeControl,
    shouldApplySnapshot,
  });
})(window);
