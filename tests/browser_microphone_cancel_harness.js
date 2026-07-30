"use strict";

const fs = require("fs");
const vm = require("vm");

const logicPath = process.argv[2];
const microphonePath = process.argv[3];
const workletPath = process.argv[4];
if (!logicPath || !microphonePath || !workletPath) {
  throw new Error(
    "speech logic, microphone, and worklet source paths are required",
  );
}

globalThis.window = {};
vm.runInThisContext(
  fs.readFileSync(logicPath, "utf8"),
  { filename: logicPath },
);
vm.runInThisContext(
  fs.readFileSync(microphonePath, "utf8"),
  { filename: microphonePath },
);

function exerciseWorkletProtocol() {
  let Processor = null;
  const outputs = [];
  class HarnessAudioWorkletProcessor {
    constructor() {
      this.port = {
        onmessage: null,
        postMessage(message) {
          outputs.push({
            captureGeneration: message.captureGeneration,
            sampleBytes: message.samples.byteLength,
            type: message.type,
          });
        },
      };
    }
  }
  vm.runInNewContext(
    fs.readFileSync(workletPath, "utf8"),
    {
      AudioWorkletProcessor: HarnessAudioWorkletProcessor,
      registerProcessor(name, value) {
        if (name !== "robot-pcm-capture") {
          throw new Error("unexpected worklet processor name");
        }
        Processor = value;
      },
    },
    { filename: workletPath },
  );
  if (typeof Processor !== "function") {
    throw new Error("worklet processor was not registered");
  }
  const processor = new Processor();
  const samples = (length, level) => {
    const value = new Float32Array(length);
    value.fill(level);
    return value;
  };
  processor.process([[samples(2048, 0.9)]]);
  const idleOutputCount = outputs.length;
  const idleOffset = processor.offset;
  processor.port.onmessage({
    data: {
      type: "capture-control",
      action: "start",
      captureGeneration: 7,
    },
  });
  processor.process([[samples(512, 0.4)]]);
  const partialOffset = processor.offset;
  processor.port.onmessage({
    data: {
      type: "capture-control",
      action: "stop",
      captureGeneration: 7,
    },
  });
  const stoppedOffset = processor.offset;
  processor.port.onmessage({
    data: {
      type: "capture-control",
      action: "start",
      captureGeneration: 8,
    },
  });
  processor.process([[samples(1024, 0.5)]]);
  processor.port.onmessage({
    data: {
      type: "capture-control",
      action: "stop",
      captureGeneration: 7,
    },
  });
  processor.process([[samples(1024, 0.6)]]);
  const outputsAfterStaleStop = outputs.slice();
  processor.port.onmessage({
    data: {
      type: "capture-control",
      action: "stop",
      captureGeneration: 8,
    },
  });
  processor.process([[samples(2048, 0.8)]]);
  return {
    activeGenerationAfterStop: processor.captureGeneration,
    idleOffset,
    idleOutputCount,
    outputCountAfterFinalStop: outputs.length,
    outputsAfterStaleStop,
    partialOffset,
    stoppedOffset,
  };
}

const workletProtocol = exerciseWorkletProtocol();

class FakeElement {
  constructor(id) {
    this.id = id;
    this.attributes = new Map();
    this.checked = false;
    this.children = [];
    this.dataset = {};
    this.disabled = false;
    this.hidden = false;
    this.listeners = new Map();
    this.style = {};
    this.textContent = "";
    this.value = "";
    this.buttonLabel = null;
  }

  addEventListener(type, listener) {
    const listeners = this.listeners.get(type) || [];
    listeners.push(listener);
    this.listeners.set(type, listeners);
  }

  dispatch(type, values = {}) {
    const event = {
      preventDefault() {},
      target: this,
      ...values,
    };
    (this.listeners.get(type) || []).forEach((listener) => {
      listener(event);
    });
  }

  querySelector(selector) {
    return selector === ".microphone-button-label"
      ? this.buttonLabel
      : null;
  }

  setAttribute(name, value) {
    this.attributes.set(name, String(value));
  }

  getAttribute(name) {
    return this.attributes.has(name)
      ? this.attributes.get(name)
      : null;
  }

  appendChild(child) {
    this.children.push(child);
    return child;
  }

  replaceChildren(...children) {
    this.children = children;
  }
}

const elementIds = [
  "microphone-button",
  "cancel-transcription-button",
  "microphone-status",
  "microphone-meter",
  "microphone-meter-fill",
  "microphone-meter-threshold",
  "microphone-settings-form",
  "microphone-settings-state",
  "microphone-settings-meter",
  "microphone-settings-meter-fill",
  "microphone-settings-meter-threshold",
  "microphone-device",
  "speech-input-language",
  "microphone-sensitivity",
  "microphone-sensitivity-output",
  "microphone-silence-ms",
  "microphone-max-utterance-ms",
  "microphone-echo-cancellation",
  "microphone-noise-suppression",
  "microphone-auto-gain",
  "microphone-keep-ready",
  "microphone-auto-send",
  "refresh-microphones-button",
];
const elements = Object.fromEntries(
  elementIds.map((id) => [id, new FakeElement(id)]),
);
elements["microphone-button"].buttonLabel = new FakeElement(
  "microphone-button-label",
);

const documentValue = {
  createElement() {
    return new FakeElement("created");
  },
  getElementById(id) {
    return elements[id] || null;
  },
};

class FakeTrack {
  constructor() {
    this.listeners = new Map();
    this.stopped = false;
    this.stopCalls = 0;
  }

  addEventListener(type, listener) {
    this.listeners.set(type, listener);
  }

  stop() {
    this.stopped = true;
    this.stopCalls += 1;
  }
}

const track = new FakeTrack();
const stream = {
  getAudioTracks() {
    return [track];
  },
  getTracks() {
    return [track];
  },
};

let activeWorklet = null;
let audioContextCreations = 0;
let audioContextCloses = 0;
let workletModuleLoads = 0;
let workletPortCloseCalls = 0;
const workletControls = [];
class FakeAudioContext {
  constructor() {
    audioContextCreations += 1;
    this.audioWorklet = {
      async addModule() {
        workletModuleLoads += 1;
      },
    };
    this.destination = {};
    this.sampleRate = 48000;
    this.state = "running";
  }

  createMediaStreamSource() {
    return {
      connect() {},
      disconnect() {},
    };
  }

  async close() {
    this.state = "closed";
    audioContextCloses += 1;
  }
}

class FakeAudioWorkletNode {
  constructor() {
    this.captureGeneration = null;
    this.port = {
      onmessage: null,
      close: () => {
        workletPortCloseCalls += 1;
      },
      postMessage: (message) => {
        workletControls.push({ ...message });
        if (
          message.type === "capture-control"
          && message.action === "start"
        ) {
          this.captureGeneration = message.captureGeneration;
        } else if (
          message.type === "capture-control"
          && message.action === "stop"
          && message.captureGeneration === this.captureGeneration
        ) {
          this.captureGeneration = null;
        }
      },
    };
    activeWorklet = this;
  }

  connect() {}

  disconnect() {}
}

class FakeBlob {
  constructor(parts, options) {
    this.parts = parts;
    this.type = options.type;
  }
}

let monotonicNow = 0;
let uiLocale = "sv";
let getUserMediaCalls = 0;
const storedSettings = new Map();
const environment = {
  AbortController,
  AudioContext: FakeAudioContext,
  AudioWorkletNode: FakeAudioWorkletNode,
  Blob: FakeBlob,
  Date,
  clearTimeout,
  isSecureContext: true,
  localStorage: {
    getItem(key) {
      return storedSettings.has(key) ? storedSettings.get(key) : null;
    },
    setItem(key, value) {
      storedSettings.set(key, value);
    },
  },
  navigator: {
    permissions: {
      async query() {
        return { state: "granted" };
      },
    },
    mediaDevices: {
      addEventListener() {},
      async enumerateDevices() {
        return [
          {
            deviceId: "default",
            kind: "audioinput",
            label: "Default microphone",
          },
        ];
      },
      async getUserMedia() {
        getUserMediaCalls += 1;
        return stream;
      },
      getSupportedConstraints() {
        return {
          autoGainControl: true,
          channelCount: true,
          echoCancellation: true,
          noiseSuppression: true,
        };
      },
    },
  },
  performance: {
    now() {
      return monotonicNow;
    },
  },
  setTimeout,
};

const calls = [];
const transcripts = [];
let postResolve = null;
let postStartedResolve = null;
const postStarted = new Promise((resolve) => {
  postStartedResolve = resolve;
});

function request(path, options = {}) {
  calls.push({ path, options });
  if (path === "/api/v1/stt/transcriptions") {
    return new Promise((resolve) => {
      postResolve = resolve;
      postStartedResolve();
    });
  }
  return Promise.resolve({});
}

function tick() {
  return new Promise((resolve) => setImmediate(resolve));
}

function emit(
  level,
  nowMs,
  captureGeneration = activeWorklet.captureGeneration,
) {
  monotonicNow = nowMs;
  const samples = new Float32Array(4800);
  samples.fill(level);
  activeWorklet.port.onmessage({
    data: {
      type: "samples",
      captureGeneration,
      samples: samples.buffer,
    },
  });
}

async function run() {
  const microphone = window.RobotMicrophoneInput.create({
    document: documentValue,
    environment,
    logic: window.RobotSpeechInputLogic,
    onError() {},
    onTranscript(text, metadata) {
      transcripts.push({ text, metadata });
    },
    getUiLocale() {
      return uiLocale;
    },
    randomId() {
      return "stt-fixed-request";
    },
    request,
    translate(key) {
      return key;
    },
  });
  await microphone.initialize();
  microphone.setAvailability(
    {
      channels: 1,
      enabled: true,
      input_format: "audio/wav",
      max_duration_ms: 20000,
      sample_rate_hz: 16000,
    },
    true,
  );
  await tick();
  await tick();
  const permissionPrimePhase = microphone.phase;
  const permissionPrimeResources = {
    audioContextCloses,
    audioContextCreations,
    getUserMediaCalls,
    trackStopCalls: track.stopCalls,
    workletModuleLoads,
  };
  const idlePortHandlerInstalled = (
    activeWorklet
    && typeof activeWorklet.port.onmessage === "function"
  );
  const controlsBeforeFirstTalk = workletControls.slice();
  emit(0.9, 25, 1);
  emit(0.9, 50, 1);
  const idleChunkPhase = microphone.phase;
  const swedishUiDefault = elements["speech-input-language"].value;
  uiLocale = "en";
  microphone.renderLocale();
  const englishUiDefault = elements["speech-input-language"].value;
  elements["speech-input-language"].value = "sv";
  elements["microphone-settings-form"].dispatch("change", {
    target: elements["speech-input-language"],
  });
  uiLocale = "en";
  microphone.renderLocale();
  const explicitOverrideAfterUiChange = (
    elements["speech-input-language"].value
  );
  elements["microphone-button"].dispatch("click");
  await tick();
  await tick();
  if (!activeWorklet || typeof activeWorklet.port.onmessage !== "function") {
    throw new Error("microphone capture did not start");
  }
  const firstCaptureGeneration = activeWorklet.captureGeneration;
  emit(0.9, 75, firstCaptureGeneration + 100);
  emit(0.9, 100, firstCaptureGeneration + 100);
  const staleChunkPhase = microphone.phase;

  emit(0.5, 150);
  emit(0.5, 250);
  for (let nowMs = 350; nowMs <= 1500; nowMs += 50) {
    emit(0, nowMs);
  }
  await postStarted;

  const post = calls[0];
  const beforeCancel = calls.map((call) => ({
    method: call.options.method,
    path: call.path,
  }));
  microphone.cancel();
  await tick();
  const cancelledSignal = post.options.signal.aborted;
  const cancellation = calls.find(
    (call) => call.options.method === "DELETE",
  );

  postResolve({
    transcription: {
      detected_language: "sv",
      status: "completed",
      text: "Det här resultatet kom för sent.",
      transcription_id: "late-transcription",
      valid_until_unix_ms: Date.now() + 10000,
    },
  });
  await tick();
  await tick();
  const warmResourcesAfterFirstTurn = {
    audioContextCloses,
    audioContextCreations,
    getUserMediaCalls,
    trackStopCalls: track.stopCalls,
    workletModuleLoads,
  };
  const portHandlerAfterFirstTurn = (
    typeof activeWorklet.port.onmessage === "function"
  );
  elements["microphone-button"].dispatch("click");
  await tick();
  await tick();
  const secondTurnPhase = microphone.phase;
  const warmResourcesDuringSecondTurn = {
    audioContextCloses,
    audioContextCreations,
    getUserMediaCalls,
    trackStopCalls: track.stopCalls,
    workletModuleLoads,
  };
  microphone.cancel();
  await tick();
  const finalPhase = microphone.phase;
  microphone.setAvailability(
    {
      channels: 1,
      enabled: true,
      input_format: "audio/wav",
      max_duration_ms: 20000,
      sample_rate_hz: 16000,
    },
    true,
  );
  elements["microphone-keep-ready"].checked = false;
  elements["microphone-settings-form"].dispatch("change", {
    target: elements["microphone-keep-ready"],
  });
  await tick();
  await tick();
  const phaseAfterKeepReadyDisabled = microphone.phase;
  microphone.destroy();
  await tick();
  await tick();
  const resourcesAfterDestroy = {
    audioContextCloses,
    audioContextCreations,
    getUserMediaCalls,
    trackStopCalls: track.stopCalls,
    workletModuleLoads,
  };
  const reloadedMicrophone = window.RobotMicrophoneInput.create({
    document: documentValue,
    environment,
    logic: window.RobotSpeechInputLogic,
    onError() {},
    onTranscript() {},
    getUiLocale() {
      return uiLocale;
    },
    randomId() {
      return "unused-reloaded-request";
    },
    request,
    translate(key) {
      return key;
    },
  });
  await reloadedMicrophone.initialize();
  const explicitOverrideAfterReload = (
    elements["speech-input-language"].value
  );
  const persistedSettings = JSON.parse(
    storedSettings.get(
      window.RobotSpeechInputLogic.SETTINGS_STORAGE_KEY,
    ),
  );
  reloadedMicrophone.destroy();
  storedSettings.clear();
  storedSettings.set(
    window.RobotSpeechInputLogic.LEGACY_SETTINGS_STORAGE_KEY,
    JSON.stringify({
      deviceId: "razer-device-id",
      language: "auto",
      sensitivity: 72,
      silenceMs: 800,
      maxUtteranceMs: 17000,
      echoCancellation: false,
      noiseSuppression: false,
      autoGainControl: false,
      autoSend: false,
    }),
  );
  uiLocale = "en";
  const migratedMicrophone = window.RobotMicrophoneInput.create({
    document: documentValue,
    environment,
    logic: window.RobotSpeechInputLogic,
    onError() {},
    onTranscript() {},
    getUiLocale() {
      return uiLocale;
    },
    randomId() {
      return "unused-migrated-request";
    },
    request,
    translate(key) {
      return key;
    },
  });
  await migratedMicrophone.initialize();
  const migratedSettings = JSON.parse(
    storedSettings.get(
      window.RobotSpeechInputLogic.SETTINGS_STORAGE_KEY,
    ),
  );
  migratedMicrophone.destroy();

  process.stdout.write(JSON.stringify({
    beforeCancel,
    cancelledSignal,
    callSequence: calls.map((call) => ({
      method: call.options.method,
      path: call.path,
    })),
    cancellationPath: cancellation ? cancellation.path : null,
    captureControls: workletControls,
    controlsBeforeFirstTalk,
    finalPhase,
    swedishUiDefault,
    englishUiDefault,
    explicitOverrideAfterUiChange,
    explicitOverrideAfterReload,
    persistedLanguageExplicit: persistedSettings.languageExplicit,
    persistedSchemaVersion: persistedSettings.schemaVersion,
    migratedSettings,
    idleChunkPhase,
    idlePortHandlerInstalled,
    permissionPrimePhase,
    permissionPrimeResources,
    postRequestId: post.options.headers["X-Robot-STT-Request-ID"],
    phaseAfterKeepReadyDisabled,
    portHandlerAfterFirstTurn,
    resourcesAfterDestroy,
    secondTurnPhase,
    staleChunkPhase,
    transcriptCount: transcripts.length,
    warmResourcesAfterFirstTurn,
    warmResourcesDuringSecondTurn,
    workletPortCloseCalls,
    workletProtocol,
  }));
}

run().catch((error) => {
  process.stderr.write(`${error.stack || error}\n`);
  process.exitCode = 1;
});
