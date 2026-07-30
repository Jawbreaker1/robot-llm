((global) => {
  "use strict";

  const TARGET_SAMPLE_RATE_HZ = 16000;
  const SETTINGS_SCHEMA_VERSION = 3;
  const SETTINGS_STORAGE_KEY = "robot-dashboard-microphone-v3";
  const LEGACY_SETTINGS_STORAGE_KEY = "robot-dashboard-microphone-v2";
  const EARLIEST_SETTINGS_STORAGE_KEY = "robot-dashboard-microphone-v1";
  const DEFAULT_SETTINGS = Object.freeze({
    deviceId: "default",
    language: "auto",
    sensitivity: 65,
    silenceMs: 1200,
    maxUtteranceMs: 12000,
    echoCancellation: true,
    noiseSuppression: true,
    autoGainControl: false,
    keepReady: true,
    autoSend: true,
  });
  const VAD_ACTION = Object.freeze({
    NONE: "none",
    SPEECH_STARTED: "speech_started",
    STOP_SILENCE: "stop_silence",
    STOP_NO_SPEECH: "stop_no_speech",
    STOP_MAX_DURATION: "stop_max_duration",
  });
  const ALLOWED_LANGUAGES = new Set(["auto", "sv", "en"]);
  const MIN_LEVEL_DB = -72;
  const MAX_LEVEL_DB = 0;
  const MIN_SILENCE_MS = 400;
  const MAX_SILENCE_MS = 3000;
  const MIN_UTTERANCE_MS = 3000;
  const MAX_UTTERANCE_MS = 30000;
  const NO_SPEECH_TIMEOUT_MS = 5000;
  const CALIBRATION_MS = 200;
  const SPEECH_ATTACK_FRAMES = 2;
  const SPEECH_RELEASE_HYSTERESIS_DB = 4;
  const SPEECH_PREROLL_MS = 1500;
  const MIN_UPLOAD_DURATION_MS = 250;

  function boundedInteger(value, fallback, minimum, maximum) {
    const number = Number(value);
    return Number.isSafeInteger(number)
      ? Math.min(maximum, Math.max(minimum, number))
      : fallback;
  }

  function boundedBoolean(value, fallback) {
    return typeof value === "boolean" ? value : fallback;
  }

  function safeDeviceId(value) {
    return (
      typeof value === "string"
      && value.length > 0
      && value.length <= 512
      && !value.includes("\u0000")
    )
      ? value
      : DEFAULT_SETTINGS.deviceId;
  }

  function normalizeSettings(value) {
    const candidate = (
      value && typeof value === "object" && !Array.isArray(value)
        ? value
        : {}
    );
    return Object.freeze({
      deviceId: safeDeviceId(candidate.deviceId),
      language: ALLOWED_LANGUAGES.has(candidate.language)
        ? candidate.language
        : DEFAULT_SETTINGS.language,
      sensitivity: boundedInteger(
        candidate.sensitivity,
        DEFAULT_SETTINGS.sensitivity,
        0,
        100,
      ),
      silenceMs: boundedInteger(
        candidate.silenceMs,
        DEFAULT_SETTINGS.silenceMs,
        MIN_SILENCE_MS,
        MAX_SILENCE_MS,
      ),
      maxUtteranceMs: boundedInteger(
        candidate.maxUtteranceMs,
        DEFAULT_SETTINGS.maxUtteranceMs,
        MIN_UTTERANCE_MS,
        MAX_UTTERANCE_MS,
      ),
      echoCancellation: boundedBoolean(
        candidate.echoCancellation,
        DEFAULT_SETTINGS.echoCancellation,
      ),
      noiseSuppression: boundedBoolean(
        candidate.noiseSuppression,
        DEFAULT_SETTINGS.noiseSuppression,
      ),
      autoGainControl: boundedBoolean(
        candidate.autoGainControl,
        DEFAULT_SETTINGS.autoGainControl,
      ),
      keepReady: boundedBoolean(
        candidate.keepReady,
        DEFAULT_SETTINGS.keepReady,
      ),
      autoSend: boundedBoolean(
        candidate.autoSend,
        DEFAULT_SETTINGS.autoSend,
      ),
    });
  }

  function levelDb(samples) {
    if (!samples || !Number.isSafeInteger(samples.length) || samples.length === 0) {
      return MIN_LEVEL_DB;
    }
    let sum = 0;
    for (let index = 0; index < samples.length; index += 1) {
      const sample = Number(samples[index]);
      if (Number.isFinite(sample)) {
        const clipped = Math.min(1, Math.max(-1, sample));
        sum += clipped * clipped;
      }
    }
    const rms = Math.sqrt(sum / samples.length);
    if (rms <= 0.000001) {
      return MIN_LEVEL_DB;
    }
    return Math.max(MIN_LEVEL_DB, Math.min(MAX_LEVEL_DB, 20 * Math.log10(rms)));
  }

  function meterPercent(db) {
    const finiteDb = Number.isFinite(db) ? db : MIN_LEVEL_DB;
    return Math.round(
      100 * (
        Math.min(MAX_LEVEL_DB, Math.max(MIN_LEVEL_DB, finiteDb))
        - MIN_LEVEL_DB
      ) / (MAX_LEVEL_DB - MIN_LEVEL_DB),
    );
  }

  function thresholdDb(noiseFloorDb, sensitivity) {
    const floor = Number.isFinite(noiseFloorDb) ? noiseFloorDb : -58;
    const normalizedSensitivity = boundedInteger(
      sensitivity,
      DEFAULT_SETTINGS.sensitivity,
      0,
      100,
    );
    const marginDb = 16 - normalizedSensitivity * 0.1;
    return Math.min(-24, Math.max(-55, floor + marginDb));
  }

  function createVadState(startedAtMs = 0) {
    const started = Number.isFinite(startedAtMs) && startedAtMs >= 0
      ? startedAtMs
      : 0;
    return Object.freeze({
      phase: "waiting",
      startedAtMs: started,
      lastObservedAtMs: started,
      speechStartedAtMs: null,
      lastVoiceAtMs: null,
      noiseFloorDb: -58,
      aboveThresholdFrames: 0,
    });
  }

  function advanceVad(previous, observation, rawSettings) {
    const current = (
      previous && typeof previous === "object" && !Array.isArray(previous)
        ? previous
        : createVadState()
    );
    const settings = normalizeSettings(rawSettings);
    const rawNow = Number(observation && observation.nowMs);
    const nowMs = Number.isFinite(rawNow)
      ? Math.max(current.lastObservedAtMs, rawNow)
      : current.lastObservedAtMs;
    const rawLevel = Number(observation && observation.levelDb);
    const observedLevelDb = Number.isFinite(rawLevel)
      ? Math.min(MAX_LEVEL_DB, Math.max(MIN_LEVEL_DB, rawLevel))
      : MIN_LEVEL_DB;
    const elapsedMs = nowMs - current.startedAtMs;
    const currentThresholdDb = thresholdDb(
      current.noiseFloorDb,
      settings.sensitivity,
    );
    const voice = observedLevelDb >= currentThresholdDb;
    const sustainedVoice = observedLevelDb >= Math.max(
      MIN_LEVEL_DB,
      currentThresholdDb - SPEECH_RELEASE_HYSTERESIS_DB,
    );
    let phase = current.phase;
    let speechStartedAtMs = current.speechStartedAtMs;
    let lastVoiceAtMs = current.lastVoiceAtMs;
    let noiseFloorDb = current.noiseFloorDb;
    let aboveThresholdFrames = voice
      ? current.aboveThresholdFrames + 1
      : 0;
    let action = VAD_ACTION.NONE;

    if (phase === "waiting") {
      const calibrating = elapsedMs < CALIBRATION_MS;
      if (!voice || calibrating && observedLevelDb < -32) {
        const weight = calibrating ? 0.16 : 0.035;
        noiseFloorDb = (
          current.noiseFloorDb * (1 - weight)
          + observedLevelDb * weight
        );
      }
      const strongInitialSpeech = calibrating && observedLevelDb >= -32;
      if (
        aboveThresholdFrames >= SPEECH_ATTACK_FRAMES
        || strongInitialSpeech
      ) {
        phase = "speech";
        speechStartedAtMs = nowMs;
        lastVoiceAtMs = nowMs;
        action = VAD_ACTION.SPEECH_STARTED;
      } else if (elapsedMs >= Math.min(
        NO_SPEECH_TIMEOUT_MS,
        settings.maxUtteranceMs,
      )) {
        action = VAD_ACTION.STOP_NO_SPEECH;
      }
    } else if (phase === "speech") {
      if (sustainedVoice) {
        lastVoiceAtMs = nowMs;
      } else if (
        lastVoiceAtMs !== null
        && nowMs - lastVoiceAtMs >= settings.silenceMs
      ) {
        action = VAD_ACTION.STOP_SILENCE;
      }
    }

    if (elapsedMs >= settings.maxUtteranceMs) {
      action = phase === "speech"
        ? VAD_ACTION.STOP_MAX_DURATION
        : VAD_ACTION.STOP_NO_SPEECH;
    }

    const nextState = Object.freeze({
      phase,
      startedAtMs: current.startedAtMs,
      lastObservedAtMs: nowMs,
      speechStartedAtMs,
      lastVoiceAtMs,
      noiseFloorDb,
      aboveThresholdFrames,
    });
    const nextThresholdDb = thresholdDb(
      noiseFloorDb,
      settings.sensitivity,
    );
    return Object.freeze({
      state: nextState,
      action,
      levelDb: observedLevelDb,
      thresholdDb: nextThresholdDb,
      levelPercent: meterPercent(observedLevelDb),
      thresholdPercent: meterPercent(nextThresholdDb),
    });
  }

  function concatenateSamples(chunks) {
    const normalized = Array.isArray(chunks)
      ? chunks.filter((chunk) => chunk && Number.isSafeInteger(chunk.length))
      : [];
    const length = normalized.reduce((total, chunk) => total + chunk.length, 0);
    const result = new Float32Array(length);
    let offset = 0;
    normalized.forEach((chunk) => {
      result.set(chunk, offset);
      offset += chunk.length;
    });
    return result;
  }

  function resampleMono(samples, sourceRateHz, targetRateHz = TARGET_SAMPLE_RATE_HZ) {
    if (
      !samples
      || !Number.isSafeInteger(samples.length)
      || !Number.isFinite(sourceRateHz)
      || !Number.isFinite(targetRateHz)
      || sourceRateHz <= 0
      || targetRateHz <= 0
    ) {
      throw new TypeError("PCM resampling parameters are invalid.");
    }
    const source = samples instanceof Float32Array
      ? samples
      : Float32Array.from(samples);
    if (source.length === 0 || sourceRateHz === targetRateHz) {
      return source.slice();
    }
    const outputLength = Math.max(
      1,
      Math.round(source.length * targetRateHz / sourceRateHz),
    );
    const output = new Float32Array(outputLength);
    const ratio = sourceRateHz / targetRateHz;
    if (ratio > 1) {
      for (let outputIndex = 0; outputIndex < outputLength; outputIndex += 1) {
        const start = outputIndex * ratio;
        const end = Math.min(source.length, (outputIndex + 1) * ratio);
        const first = Math.floor(start);
        const last = Math.ceil(end);
        let weighted = 0;
        let weightTotal = 0;
        for (let sourceIndex = first; sourceIndex < last; sourceIndex += 1) {
          const overlap = Math.max(
            0,
            Math.min(end, sourceIndex + 1) - Math.max(start, sourceIndex),
          );
          if (overlap > 0 && sourceIndex < source.length) {
            weighted += source[sourceIndex] * overlap;
            weightTotal += overlap;
          }
        }
        output[outputIndex] = weightTotal > 0 ? weighted / weightTotal : 0;
      }
      return output;
    }
    for (let outputIndex = 0; outputIndex < outputLength; outputIndex += 1) {
      const position = outputIndex * ratio;
      const left = Math.min(source.length - 1, Math.floor(position));
      const right = Math.min(source.length - 1, left + 1);
      const fraction = position - left;
      output[outputIndex] = (
        source[left] * (1 - fraction)
        + source[right] * fraction
      );
    }
    return output;
  }

  function speechWindow(
    samples,
    sourceRateHz,
    vadState,
    prerollMs = SPEECH_PREROLL_MS,
    minimumDurationMs = MIN_UPLOAD_DURATION_MS,
  ) {
    if (
      !samples
      || !Number.isSafeInteger(samples.length)
      || !Number.isFinite(sourceRateHz)
      || sourceRateHz <= 0
      || !vadState
      || typeof vadState !== "object"
      || !Number.isFinite(vadState.startedAtMs)
      || !Number.isFinite(vadState.speechStartedAtMs)
      || vadState.speechStartedAtMs < vadState.startedAtMs
      || !Number.isFinite(prerollMs)
      || prerollMs < 0
      || !Number.isFinite(minimumDurationMs)
      || minimumDurationMs < 0
    ) {
      throw new TypeError("Speech window parameters are invalid.");
    }
    const source = samples instanceof Float32Array
      ? samples
      : Float32Array.from(samples);
    const waitingMs = vadState.speechStartedAtMs - vadState.startedAtMs;
    const trimMs = Math.max(0, waitingMs - prerollMs);
    const startFrame = Math.min(
      source.length,
      Math.floor(trimMs * sourceRateHz / 1000),
    );
    const trimmed = source.slice(startFrame);
    const minimumFrames = Math.ceil(
      minimumDurationMs * sourceRateHz / 1000,
    );
    if (trimmed.length >= minimumFrames) {
      return trimmed;
    }
    const padded = new Float32Array(minimumFrames);
    padded.set(trimmed);
    return padded;
  }

  function encodePCM16Wav(samples, sampleRateHz = TARGET_SAMPLE_RATE_HZ) {
    if (
      !samples
      || !Number.isSafeInteger(samples.length)
      || !Number.isSafeInteger(sampleRateHz)
      || sampleRateHz <= 0
      || sampleRateHz > 384000
    ) {
      throw new TypeError("PCM WAV parameters are invalid.");
    }
    const pcm = samples instanceof Float32Array
      ? samples
      : Float32Array.from(samples);
    const dataBytes = pcm.length * 2;
    const buffer = new ArrayBuffer(44 + dataBytes);
    const view = new DataView(buffer);
    const writeText = (offset, value) => {
      for (let index = 0; index < value.length; index += 1) {
        view.setUint8(offset + index, value.charCodeAt(index));
      }
    };
    writeText(0, "RIFF");
    view.setUint32(4, 36 + dataBytes, true);
    writeText(8, "WAVE");
    writeText(12, "fmt ");
    view.setUint32(16, 16, true);
    view.setUint16(20, 1, true);
    view.setUint16(22, 1, true);
    view.setUint32(24, sampleRateHz, true);
    view.setUint32(28, sampleRateHz * 2, true);
    view.setUint16(32, 2, true);
    view.setUint16(34, 16, true);
    writeText(36, "data");
    view.setUint32(40, dataBytes, true);
    for (let index = 0; index < pcm.length; index += 1) {
      const sample = Math.min(1, Math.max(-1, Number(pcm[index]) || 0));
      const integer = sample < 0
        ? Math.round(sample * 32768)
        : Math.round(sample * 32767);
      view.setInt16(44 + index * 2, integer, true);
    }
    return new Uint8Array(buffer);
  }

  global.RobotSpeechInputLogic = Object.freeze({
    DEFAULT_SETTINGS,
    EARLIEST_SETTINGS_STORAGE_KEY,
    LEGACY_SETTINGS_STORAGE_KEY,
    SETTINGS_SCHEMA_VERSION,
    SETTINGS_STORAGE_KEY,
    TARGET_SAMPLE_RATE_HZ,
    VAD_ACTION,
    advanceVad,
    concatenateSamples,
    createVadState,
    encodePCM16Wav,
    levelDb,
    meterPercent,
    normalizeSettings,
    resampleMono,
    speechWindow,
    thresholdDb,
  });
})(typeof window === "undefined" ? globalThis : window);
