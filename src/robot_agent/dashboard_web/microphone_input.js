((global) => {
  "use strict";

  const ACTIVE_PHASES = new Set([
    "requesting",
    "listening",
    "speech",
    "stopping",
    "transcribing",
    "queued",
  ]);
  const POLL_INTERVAL_MS = 250;
  const TRANSCRIPTION_TIMEOUT_MS = 30000;
  const STATUS_KEYS = Object.freeze({
    unsupported: "microphone.status.unsupported",
    unavailable: "microphone.status.unavailable",
    idle: "microphone.status.idle",
    requesting: "microphone.status.requesting",
    listening: "microphone.status.listening",
    speech: "microphone.status.speech",
    stopping: "microphone.status.stopping",
    transcribing: "microphone.status.transcribing",
    queued: "microphone.status.queued",
    transcript_ready: "microphone.status.transcript_ready",
    no_speech: "microphone.status.no_speech",
    cancelled: "microphone.status.cancelled",
    permission_denied: "microphone.status.permission_denied",
    device_missing: "microphone.status.device_missing",
    device_fallback: "microphone.status.device_fallback",
    device_lost: "microphone.status.device_lost",
    audio_failed: "microphone.status.audio_failed",
    request_failed: "microphone.status.request_failed",
    transcription_failed: "microphone.status.transcription_failed",
    expired: "microphone.status.expired",
  });
  const ERROR_STATUS_BY_NAME = Object.freeze({
    AbortError: "cancelled",
    NotAllowedError: "permission_denied",
    SecurityError: "permission_denied",
    NotFoundError: "device_missing",
    DevicesNotFoundError: "device_missing",
    OverconstrainedError: "device_missing",
    ConstraintNotSatisfiedError: "device_missing",
    NotReadableError: "audio_failed",
    TrackStartError: "audio_failed",
  });

  function requiredFunction(value, name) {
    if (typeof value !== "function") {
      throw new TypeError(`${name} is required.`);
    }
    return value;
  }

  function elementById(documentValue, id) {
    const element = documentValue.getElementById(id);
    if (!element) {
      throw new TypeError(`Microphone element ${id} is missing.`);
    }
    return element;
  }

  function create(options = {}) {
    const environment = options.environment || global;
    const documentValue = options.document || environment.document;
    const logic = options.logic || environment.RobotSpeechInputLogic;
    if (!documentValue || !logic) {
      throw new TypeError("Microphone dependencies are invalid.");
    }
    const controller = new MicrophoneController({
      environment,
      document: documentValue,
      logic,
      translate: requiredFunction(options.translate, "translate"),
      randomId: requiredFunction(options.randomId, "randomId"),
      request: requiredFunction(options.request, "request"),
      onTranscript: requiredFunction(options.onTranscript, "onTranscript"),
      onError: requiredFunction(options.onError, "onError"),
      workletUrl: options.workletUrl || "assets/pcm_capture_worklet.js",
    });
    return controller.publicApi;
  }

  class MicrophoneController {
    constructor(dependencies) {
      this.environment = dependencies.environment;
      this.document = dependencies.document;
      this.logic = dependencies.logic;
      this.translate = dependencies.translate;
      this.randomId = dependencies.randomId;
      this.request = dependencies.request;
      this.onTranscript = dependencies.onTranscript;
      this.onError = dependencies.onError;
      this.workletUrl = dependencies.workletUrl;
      this.elements = this._elements();
      this.phase = "unavailable";
      this.settings = this._loadSettings();
      this.levelPercent = 0;
      this.thresholdPercent = this.logic.meterPercent(
        this.logic.thresholdDb(-58, this.settings.sensitivity),
      );
      this.capability = Object.freeze({
        enabled: false,
        input_format: "audio/wav",
        sample_rate_hz: this.logic.TARGET_SAMPLE_RATE_HZ,
        channels: 1,
        max_duration_ms: 20000,
      });
      this.composerEnabled = false;
      this.generation = 0;
      this.permissionObserved = false;
      this.devices = [];
      this.stream = null;
      this.context = null;
      this.source = null;
      this.worklet = null;
      this.sampleChunks = [];
      this.sampleFrames = 0;
      this.vad = null;
      this.captureTimer = null;
      this.pollTimer = null;
      this.requestController = null;
      this.transcriptionDeadlineMs = null;
      this.activeTranscriptionId = null;
      this.activeRequestId = null;
      this.initialized = false;
      this.supported = this._browserSupported();
      const controller = this;
      this.publicApi = Object.freeze({
        initialize: () => this.initialize(),
        setAvailability: (capability, enabled) => (
          this.setAvailability(capability, enabled)
        ),
        renderLocale: () => this.renderLocale(),
        cancel: () => this.cancel(),
        destroy: () => this.destroy(),
        get phase() {
          return controller.phase;
        },
      });
    }

    _elements() {
      const ids = {
        button: "microphone-button",
        buttonLabel: "microphone-button",
        cancel: "cancel-transcription-button",
        status: "microphone-status",
        meter: "microphone-meter",
        meterFill: "microphone-meter-fill",
        meterThreshold: "microphone-meter-threshold",
        settingsForm: "microphone-settings-form",
        settingsState: "microphone-settings-state",
        settingsMeter: "microphone-settings-meter",
        settingsMeterFill: "microphone-settings-meter-fill",
        settingsMeterThreshold: "microphone-settings-meter-threshold",
        device: "microphone-device",
        language: "speech-input-language",
        sensitivity: "microphone-sensitivity",
        sensitivityOutput: "microphone-sensitivity-output",
        silenceMs: "microphone-silence-ms",
        maxUtteranceMs: "microphone-max-utterance-ms",
        echoCancellation: "microphone-echo-cancellation",
        noiseSuppression: "microphone-noise-suppression",
        autoGainControl: "microphone-auto-gain",
        autoSend: "microphone-auto-send",
        refresh: "refresh-microphones-button",
      };
      const elements = {};
      Object.keys(ids).forEach((name) => {
        elements[name] = elementById(this.document, ids[name]);
      });
      elements.buttonLabel = elements.button.querySelector(
        ".microphone-button-label",
      );
      if (!elements.buttonLabel) {
        throw new TypeError("Microphone button label is missing.");
      }
      return elements;
    }

    _browserSupported() {
      const mediaDevices = this.environment.navigator
        && this.environment.navigator.mediaDevices;
      const AudioContextValue = this.environment.AudioContext;
      return (
        this.environment.isSecureContext !== false
        && mediaDevices
        && typeof mediaDevices.getUserMedia === "function"
        && typeof mediaDevices.enumerateDevices === "function"
        && typeof AudioContextValue === "function"
        && typeof this.environment.AudioWorkletNode === "function"
        && typeof this.environment.Blob === "function"
        && typeof this.environment.AbortController === "function"
      );
    }

    _loadSettings() {
      let value = null;
      try {
        const raw = this.environment.localStorage
          ? this.environment.localStorage.getItem(
            this.logic.SETTINGS_STORAGE_KEY,
          )
          : null;
        value = raw ? JSON.parse(raw) : null;
      } catch (_error) {
        value = null;
      }
      return this.logic.normalizeSettings(value);
    }

    _saveSettings() {
      try {
        if (this.environment.localStorage) {
          this.environment.localStorage.setItem(
            this.logic.SETTINGS_STORAGE_KEY,
            JSON.stringify(this.settings),
          );
        }
        this.elements.settingsState.dataset.status = "saved";
        this.elements.settingsState.textContent = this.translate(
          "settings.microphone.saved",
        );
      } catch (_error) {
        this.elements.settingsState.dataset.status = "failed";
        this.elements.settingsState.textContent = this.translate(
          "settings.microphone.storage_failed",
        );
      }
    }

    _settingsFromForm() {
      const silenceMs = this.elements.silenceMs.value;
      const maxUtteranceMs = this.elements.maxUtteranceMs.value;
      return this.logic.normalizeSettings({
        deviceId: this.elements.device.value,
        language: this.elements.language.value,
        sensitivity: Number(this.elements.sensitivity.value),
        silenceMs: silenceMs === ""
          ? this.settings.silenceMs
          : Number(silenceMs),
        maxUtteranceMs: maxUtteranceMs === ""
          ? this.settings.maxUtteranceMs
          : Number(maxUtteranceMs),
        echoCancellation: this.elements.echoCancellation.checked,
        noiseSuppression: this.elements.noiseSuppression.checked,
        autoGainControl: this.elements.autoGainControl.checked,
        autoSend: this.elements.autoSend.checked,
      });
    }

    _renderSettings() {
      this.elements.device.value = this.settings.deviceId;
      if (this.elements.device.value !== this.settings.deviceId) {
        const saved = this.document.createElement("option");
        saved.value = this.settings.deviceId;
        saved.textContent = this.translate(
          "settings.microphone.saved_device",
        );
        this.elements.device.appendChild(saved);
        this.elements.device.value = this.settings.deviceId;
      }
      this.elements.language.value = this.settings.language;
      this.elements.sensitivity.value = String(this.settings.sensitivity);
      this.elements.sensitivityOutput.value = `${this.settings.sensitivity} %`;
      this.elements.sensitivityOutput.textContent = (
        `${this.settings.sensitivity} %`
      );
      this.elements.silenceMs.value = String(this.settings.silenceMs);
      this.elements.maxUtteranceMs.value = String(
        this.settings.maxUtteranceMs,
      );
      this.elements.echoCancellation.checked = (
        this.settings.echoCancellation
      );
      this.elements.noiseSuppression.checked = (
        this.settings.noiseSuppression
      );
      this.elements.autoGainControl.checked = this.settings.autoGainControl;
      this.elements.autoSend.checked = this.settings.autoSend;
    }

    _handleSettingsChange() {
      this.settings = this._settingsFromForm();
      this._renderSettings();
      this._saveSettings();
      if (this.vad) {
        const threshold = this.logic.thresholdDb(
          this.vad.noiseFloorDb,
          this.settings.sensitivity,
        );
        this._renderMeter(
          Number(this.elements.meter.getAttribute("aria-valuenow")) || 0,
          this.logic.meterPercent(threshold),
        );
      }
    }

    _bind() {
      this.elements.button.addEventListener("click", () => {
        if (this.phase === "listening" || this.phase === "speech") {
          this._finishCapture(this.generation, "manual");
        } else if (!ACTIVE_PHASES.has(this.phase)) {
          this.start();
        }
      });
      this.elements.cancel.addEventListener("click", () => this.cancel());
      this.elements.settingsForm.addEventListener("submit", (event) => {
        event.preventDefault();
      });
      this.elements.settingsForm.addEventListener(
        "input",
        (event) => {
          if (event.target === this.elements.sensitivity) {
            this._handleSettingsChange();
          }
        },
      );
      this.elements.settingsForm.addEventListener(
        "change",
        () => this._handleSettingsChange(),
      );
      this.elements.refresh.addEventListener("click", () => {
        this.refreshDevices(true);
      });
      const mediaDevices = this.environment.navigator
        && this.environment.navigator.mediaDevices;
      if (
        mediaDevices
        && typeof mediaDevices.addEventListener === "function"
      ) {
        mediaDevices.addEventListener("devicechange", () => {
          this.refreshDevices(false);
        });
      }
      if (typeof this.environment.addEventListener === "function") {
        this.environment.addEventListener("pagehide", () => this.destroy());
      }
    }

    async initialize() {
      if (this.initialized) {
        return;
      }
      this.initialized = true;
      this._bind();
      this._renderSettings();
      this._renderMeter(0, this.logic.meterPercent(
        this.logic.thresholdDb(-58, this.settings.sensitivity),
      ));
      if (!this.supported) {
        this._setPhase("unsupported");
        return;
      }
      this._setPhase("unavailable");
      await this.refreshDevices(false);
    }

    setAvailability(capability, enabled) {
      const value = (
        capability && typeof capability === "object"
          ? capability
          : {}
      );
      const valid = (
        value.enabled === true
        && value.input_format === "audio/wav"
        && value.sample_rate_hz === this.logic.TARGET_SAMPLE_RATE_HZ
        && value.channels === 1
        && Number.isSafeInteger(value.max_duration_ms)
        && value.max_duration_ms >= 3000
        && value.max_duration_ms <= 30000
      );
      this.capability = Object.freeze({
        enabled: valid,
        input_format: "audio/wav",
        sample_rate_hz: this.logic.TARGET_SAMPLE_RATE_HZ,
        channels: 1,
        max_duration_ms: valid ? value.max_duration_ms : 20000,
      });
      this.composerEnabled = valid && enabled === true;
      if (!this.composerEnabled && ACTIVE_PHASES.has(this.phase)) {
        this.cancel();
      }
      if (!ACTIVE_PHASES.has(this.phase)) {
        this._setPhase(
          !this.supported
            ? "unsupported"
            : this.composerEnabled
              ? "idle"
              : "unavailable",
        );
      } else {
        this._renderPhase();
      }
    }

    renderLocale() {
      this._renderSettings();
      this._renderDeviceOptions();
      this._renderPhase();
      this._renderMeter(this.levelPercent, this.thresholdPercent);
    }

    _setPhase(phase, announceError = false) {
      this.phase = phase;
      this._renderPhase();
      if (
        announceError
        && phase !== "cancelled"
        && phase !== "no_speech"
      ) {
        this.onError(this.translate(STATUS_KEYS[phase] || STATUS_KEYS.audio_failed));
      }
    }

    _renderPhase() {
      const statusKey = STATUS_KEYS[this.phase] || STATUS_KEYS.audio_failed;
      this.elements.status.textContent = this.translate(statusKey);
      const listening = this.phase === "listening" || this.phase === "speech";
      const busy = ACTIVE_PHASES.has(this.phase);
      this.elements.button.setAttribute(
        "aria-pressed",
        listening ? "true" : "false",
      );
      this.elements.button.dataset.state = this.phase;
      this.elements.button.disabled = listening
        ? false
        : busy || !this.composerEnabled || !this.supported;
      this.elements.cancel.hidden = !busy;
      const labelKey = listening
        ? "microphone.button.stop"
        : "microphone.button.label";
      const ariaKey = listening
        ? "microphone.button.stop"
        : "microphone.button.start";
      this.elements.buttonLabel.textContent = this.translate(labelKey);
      this.elements.button.setAttribute("aria-label", this.translate(ariaKey));
    }

    _renderMeter(levelPercent, thresholdPercent) {
      const level = Math.max(0, Math.min(100, Math.round(levelPercent)));
      const threshold = Math.max(
        0,
        Math.min(100, Math.round(thresholdPercent)),
      );
      this.levelPercent = level;
      this.thresholdPercent = threshold;
      [
        [
          this.elements.meter,
          this.elements.meterFill,
          this.elements.meterThreshold,
        ],
        [
          this.elements.settingsMeter,
          this.elements.settingsMeterFill,
          this.elements.settingsMeterThreshold,
        ],
      ].forEach(([meter, fill, marker]) => {
        meter.setAttribute("aria-valuenow", String(level));
        meter.setAttribute(
          "aria-valuetext",
          this.translate("microphone.meter.value", {
            level,
            threshold,
          }),
        );
        fill.style.width = `${level}%`;
        marker.style.left = `${threshold}%`;
      });
    }

    _renderDeviceOptions() {
      const selected = this.settings.deviceId;
      const options = [];
      const defaultOption = this.document.createElement("option");
      defaultOption.value = "default";
      defaultOption.textContent = this.translate(
        "settings.microphone.default_device",
      );
      options.push(defaultOption);
      let unnamedIndex = 0;
      this.devices.forEach((device) => {
        if (
          !device
          || device.kind !== "audioinput"
          || !device.deviceId
          || device.deviceId === "default"
        ) {
          return;
        }
        unnamedIndex += 1;
        const option = this.document.createElement("option");
        option.value = device.deviceId;
        option.textContent = device.label || this.translate(
          "settings.microphone.unnamed_device",
          { index: unnamedIndex },
        );
        options.push(option);
      });
      if (
        selected !== "default"
        && !options.some((option) => option.value === selected)
        && !this.permissionObserved
      ) {
        const saved = this.document.createElement("option");
        saved.value = selected;
        saved.textContent = this.translate(
          "settings.microphone.saved_device",
        );
        options.push(saved);
      }
      this.elements.device.replaceChildren(...options);
      if (
        this.permissionObserved
        && selected !== "default"
        && !options.some((option) => option.value === selected)
      ) {
        this.settings = this.logic.normalizeSettings({
          ...this.settings,
          deviceId: "default",
        });
        this._saveSettings();
        if (!ACTIVE_PHASES.has(this.phase)) {
          this._setPhase("device_fallback");
        }
      }
      this.elements.device.value = this.settings.deviceId;
    }

    async refreshDevices(announceFailure) {
      if (!this.supported) {
        return;
      }
      try {
        const devices = await (
          this.environment.navigator.mediaDevices.enumerateDevices()
        );
        this.devices = Array.isArray(devices)
          ? devices.filter((device) => device && device.kind === "audioinput")
          : [];
        this._renderDeviceOptions();
      } catch (_error) {
        if (announceFailure) {
          if (ACTIVE_PHASES.has(this.phase)) {
            this.onError(this.translate(STATUS_KEYS.device_missing));
          } else {
            this._setPhase("device_missing", true);
          }
        }
      }
    }

    _supportedAudioConstraints() {
      const mediaDevices = this.environment.navigator.mediaDevices;
      const supported = typeof mediaDevices.getSupportedConstraints === "function"
        ? mediaDevices.getSupportedConstraints()
        : {};
      const audio = {};
      if (supported.channelCount !== false) {
        audio.channelCount = { ideal: 1 };
      }
      if (supported.echoCancellation) {
        audio.echoCancellation = { ideal: this.settings.echoCancellation };
      }
      if (supported.noiseSuppression) {
        audio.noiseSuppression = { ideal: this.settings.noiseSuppression };
      }
      if (supported.autoGainControl) {
        audio.autoGainControl = { ideal: this.settings.autoGainControl };
      }
      if (this.settings.deviceId !== "default") {
        audio.deviceId = { exact: this.settings.deviceId };
      }
      return { audio, video: false };
    }

    async _openStream() {
      const mediaDevices = this.environment.navigator.mediaDevices;
      try {
        return await mediaDevices.getUserMedia(
          this._supportedAudioConstraints(),
        );
      } catch (error) {
        const missingSelected = (
          this.settings.deviceId !== "default"
          && (
            error && error.name === "NotFoundError"
            || error && error.name === "OverconstrainedError"
          )
        );
        if (!missingSelected) {
          throw error;
        }
        this.settings = this.logic.normalizeSettings({
          ...this.settings,
          deviceId: "default",
        });
        this._saveSettings();
        this._renderSettings();
        return mediaDevices.getUserMedia(
          this._supportedAudioConstraints(),
        );
      }
    }

    _effectiveSettings() {
      return this.logic.normalizeSettings({
        ...this.settings,
        maxUtteranceMs: Math.min(
          this.settings.maxUtteranceMs,
          this.capability.max_duration_ms,
        ),
      });
    }

    _now() {
      return (
        this.environment.performance
        && typeof this.environment.performance.now === "function"
      )
        ? this.environment.performance.now()
        : Date.now();
    }

    _wallNow() {
      const DateValue = this.environment.Date || Date;
      return typeof DateValue.now === "function"
        ? DateValue.now()
        : new DateValue().getTime();
    }

    _cancelActiveTranscription() {
      const transcriptionId = this.activeTranscriptionId;
      const requestId = this.activeRequestId;
      this.activeTranscriptionId = null;
      this.activeRequestId = null;
      const path = transcriptionId
        ? `/api/v1/stt/transcriptions/${encodeURIComponent(transcriptionId)}`
        : requestId
          ? `/api/v1/stt/requests/${encodeURIComponent(requestId)}`
          : null;
      if (!path) {
        return;
      }
      Promise.resolve(
        this.request(
          path,
          { method: "DELETE", timeout: 5000 },
        ),
      ).catch(() => {
        // Local cancellation already won; backend cleanup is best effort.
      });
    }

    async start() {
      if (
        !this.supported
        || !this.composerEnabled
        || ACTIVE_PHASES.has(this.phase)
      ) {
        return;
      }
      const generation = this.generation + 1;
      this.generation = generation;
      this.sampleChunks = [];
      this.sampleFrames = 0;
      this.vad = null;
      this._setPhase("requesting");
      try {
        const stream = await this._openStream();
        if (generation !== this.generation) {
          stream.getTracks().forEach((track) => track.stop());
          return;
        }
        this.stream = stream;
        this.permissionObserved = true;
        void this.refreshDevices(false);
        const context = new this.environment.AudioContext({
          latencyHint: "interactive",
        });
        this.context = context;
        if (
          context.state === "suspended"
          && typeof context.resume === "function"
        ) {
          await context.resume();
        }
        await context.audioWorklet.addModule(this.workletUrl);
        if (generation !== this.generation) {
          await this._cleanupCapture();
          return;
        }
        this.source = context.createMediaStreamSource(stream);
        this.worklet = new this.environment.AudioWorkletNode(
          context,
          "robot-pcm-capture",
        );
        this.worklet.port.onmessage = (event) => {
          this._receiveSamples(event, generation);
        };
        this.vad = this.logic.createVadState(this._now());
        this.source.connect(this.worklet);
        this.worklet.connect(context.destination);
        stream.getAudioTracks().forEach((track) => {
          track.addEventListener("ended", () => {
            if (
              generation === this.generation
              && (this.phase === "listening" || this.phase === "speech")
            ) {
              this._failGeneration(generation, "device_lost");
            }
          }, { once: true });
        });
        const effective = this._effectiveSettings();
        this.captureTimer = this.environment.setTimeout(
          () => this._finishCapture(generation, "maximum"),
          effective.maxUtteranceMs,
        );
        this._setPhase("listening");
      } catch (error) {
        if (generation !== this.generation) {
          return;
        }
        await this._cleanupCapture();
        const phase = ERROR_STATUS_BY_NAME[
          error && typeof error.name === "string" ? error.name : ""
        ] || "audio_failed";
        this._setPhase(phase, phase !== "cancelled");
      }
    }

    _receiveSamples(event, generation) {
      if (
        generation !== this.generation
        || (this.phase !== "listening" && this.phase !== "speech")
      ) {
        return;
      }
      const data = event && event.data;
      if (
        !data
        || data.type !== "samples"
        || !(data.samples instanceof ArrayBuffer)
      ) {
        return;
      }
      const samples = new Float32Array(data.samples);
      const effective = this._effectiveSettings();
      const maximumFrames = Math.ceil(
        this.context.sampleRate * effective.maxUtteranceMs / 1000,
      );
      if (this.sampleFrames < maximumFrames) {
        const remaining = maximumFrames - this.sampleFrames;
        const accepted = samples.length <= remaining
          ? samples
          : samples.slice(0, remaining);
        this.sampleChunks.push(accepted);
        this.sampleFrames += accepted.length;
      }
      const transition = this.logic.advanceVad(
        this.vad,
        {
          nowMs: this._now(),
          levelDb: this.logic.levelDb(samples),
        },
        effective,
      );
      this.vad = transition.state;
      this._renderMeter(
        transition.levelPercent,
        transition.thresholdPercent,
      );
      if (transition.action === this.logic.VAD_ACTION.SPEECH_STARTED) {
        this._setPhase("speech");
      } else if (
        transition.action === this.logic.VAD_ACTION.STOP_SILENCE
        || transition.action === this.logic.VAD_ACTION.STOP_MAX_DURATION
      ) {
        this._finishCapture(generation, transition.action);
      } else if (
        transition.action === this.logic.VAD_ACTION.STOP_NO_SPEECH
      ) {
        this._finishCapture(generation, "no_speech");
      }
    }

    async _cleanupCapture() {
      if (this.captureTimer !== null) {
        this.environment.clearTimeout(this.captureTimer);
        this.captureTimer = null;
      }
      if (this.worklet) {
        this.worklet.port.onmessage = null;
        try {
          this.worklet.disconnect();
        } catch (_error) {
          // A disconnected graph is already safe.
        }
      }
      if (this.source) {
        try {
          this.source.disconnect();
        } catch (_error) {
          // A disconnected graph is already safe.
        }
      }
      if (this.stream) {
        this.stream.getTracks().forEach((track) => track.stop());
      }
      const context = this.context;
      this.worklet = null;
      this.source = null;
      this.stream = null;
      this.context = null;
      if (context && typeof context.close === "function") {
        try {
          await context.close();
        } catch (_error) {
          // Cleanup must continue even if the browser already closed it.
        }
      }
    }

    async _finishCapture(generation, reason) {
      if (
        generation !== this.generation
        || (this.phase !== "listening" && this.phase !== "speech")
      ) {
        return;
      }
      const speechObserved = this.vad && this.vad.phase === "speech";
      const sourceRateHz = this.context ? this.context.sampleRate : 0;
      this._setPhase("stopping");
      await this._cleanupCapture();
      if (generation !== this.generation) {
        return;
      }
      if (!speechObserved || reason === "no_speech") {
        this.sampleChunks = [];
        this.sampleFrames = 0;
        this._renderMeter(0, this.logic.meterPercent(
          this.logic.thresholdDb(-58, this.settings.sensitivity),
        ));
        this._setPhase("no_speech");
        return;
      }
      try {
        const captured = this.logic.concatenateSamples(this.sampleChunks);
        const samples = this.logic.speechWindow(
          captured,
          sourceRateHz,
          this.vad,
        );
        const resampled = this.logic.resampleMono(
          samples,
          sourceRateHz,
          this.logic.TARGET_SAMPLE_RATE_HZ,
        );
        const wav = this.logic.encodePCM16Wav(
          resampled,
          this.logic.TARGET_SAMPLE_RATE_HZ,
        );
        this.sampleChunks = [];
        this.sampleFrames = 0;
        const audio = new this.environment.Blob(
          [wav],
          { type: "audio/wav" },
        );
        await this._submitTranscription(audio, generation);
      } catch (_error) {
        if (generation === this.generation) {
          this._setPhase("audio_failed", true);
        }
      }
    }

    async _submitTranscription(audio, generation) {
      const requestId = this.randomId("stt");
      this.activeRequestId = requestId;
      this.requestController = new this.environment.AbortController();
      this.transcriptionDeadlineMs = this._now() + TRANSCRIPTION_TIMEOUT_MS;
      this._setPhase("transcribing");
      try {
        const payload = await this.request("/api/v1/stt/transcriptions", {
          method: "POST",
          rawBody: audio,
          headers: {
            "Content-Type": "audio/wav",
            "X-Robot-STT-Request-ID": requestId,
            "X-Robot-STT-Language": this.settings.language,
          },
          signal: this.requestController.signal,
          timeout: 20000,
        });
        if (generation !== this.generation) {
          return;
        }
        this._acceptTranscriptionEnvelope(payload, generation);
      } catch (error) {
        if (generation !== this.generation) {
          return;
        }
        this._cancelActiveTranscription();
        const cancelled = error && error.name === "AbortError";
        this._setPhase(cancelled ? "cancelled" : "request_failed", !cancelled);
      }
    }

    _acceptTranscriptionEnvelope(payload, generation) {
      const transcription = (
        payload
        && typeof payload === "object"
        && payload.transcription
        && typeof payload.transcription === "object"
        && !Array.isArray(payload.transcription)
      )
        ? payload.transcription
        : {};
      const id = typeof transcription.transcription_id === "string"
        ? transcription.transcription_id
        : "";
      const status = transcription.status;
      if (
        status === "completed"
        && typeof transcription.text === "string"
        && transcription.text.trim().length > 0
        && transcription.text.length <= 4000
      ) {
        this.activeTranscriptionId = id || null;
        this._completeTranscript(transcription, generation);
        return;
      }
      if (status === "failed") {
        this.activeTranscriptionId = null;
        this.activeRequestId = null;
        this._setPhase("transcription_failed", true);
        return;
      }
      if (status === "cancelled") {
        this.activeTranscriptionId = null;
        this.activeRequestId = null;
        this._setPhase("cancelled");
        return;
      }
      if (!id || (status !== "queued" && status !== "running")) {
        this._cancelActiveTranscription();
        this._setPhase("request_failed", true);
        return;
      }
      this.activeTranscriptionId = id;
      this._setPhase("queued");
      this.pollTimer = this.environment.setTimeout(
        () => this._pollTranscription(id, generation),
        POLL_INTERVAL_MS,
      );
    }

    async _pollTranscription(id, generation) {
      this.pollTimer = null;
      if (
        generation !== this.generation
        || !this.requestController
        || this.requestController.signal.aborted
      ) {
        return;
      }
      if (this._now() >= this.transcriptionDeadlineMs) {
        this._cancelActiveTranscription();
        this._setPhase("request_failed", true);
        return;
      }
      try {
        const payload = await this.request(
          `/api/v1/stt/transcriptions/${encodeURIComponent(id)}`,
          {
            signal: this.requestController.signal,
            timeout: 10000,
          },
        );
        if (generation !== this.generation) {
          return;
        }
        this._acceptTranscriptionEnvelope(payload, generation);
      } catch (error) {
        if (generation !== this.generation) {
          return;
        }
        const cancelled = error && error.name === "AbortError";
        const expired = error && error.code === "stt_expired";
        this._cancelActiveTranscription();
        this._setPhase(
          cancelled ? "cancelled" : expired ? "expired" : "request_failed",
          !cancelled,
        );
      }
    }

    _completeTranscript(transcription, generation) {
      if (generation !== this.generation) {
        return;
      }
      const validUntil = transcription.valid_until_unix_ms;
      if (
        !Number.isSafeInteger(validUntil)
        || validUntil <= this._wallNow()
      ) {
        this.activeTranscriptionId = null;
        this._setPhase("expired", true);
        return;
      }
      if (this.pollTimer !== null) {
        this.environment.clearTimeout(this.pollTimer);
        this.pollTimer = null;
      }
      this.requestController = null;
      this.activeTranscriptionId = null;
      this.activeRequestId = null;
      this._setPhase("transcript_ready");
      this.onTranscript(transcription.text.trim(), {
        autoSend: this.settings.autoSend,
        transcriptionId: transcription.transcription_id,
        detectedLocale: (
          transcription.detected_language
          || transcription.detected_locale
          || null
        ),
      });
    }

    async _failGeneration(generation, phase) {
      if (generation !== this.generation) {
        return;
      }
      this.generation += 1;
      await this._cleanupCapture();
      this._setPhase(phase, true);
    }

    cancel() {
      if (!ACTIVE_PHASES.has(this.phase)) {
        return;
      }
      this.generation += 1;
      if (this.pollTimer !== null) {
        this.environment.clearTimeout(this.pollTimer);
        this.pollTimer = null;
      }
      if (this.requestController) {
        this.requestController.abort();
        this.requestController = null;
      }
      this._cancelActiveTranscription();
      this.sampleChunks = [];
      this.sampleFrames = 0;
      this._cleanupCapture();
      this._renderMeter(0, this.logic.meterPercent(
        this.logic.thresholdDb(-58, this.settings.sensitivity),
      ));
      this._setPhase("cancelled");
    }

    destroy() {
      this.cancel();
      this.composerEnabled = false;
      this._setPhase(this.supported ? "unavailable" : "unsupported");
    }
  }

  global.RobotMicrophoneInput = Object.freeze({ create });
})(typeof window === "undefined" ? globalThis : window);
