"use strict";

const fs = require("fs");
const vm = require("vm");

const logicPath = process.argv[2];
const microphonePath = process.argv[3];
if (!logicPath || !microphonePath) {
  throw new Error("speech logic and microphone source paths are required");
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
  }

  addEventListener(type, listener) {
    this.listeners.set(type, listener);
  }

  stop() {
    this.stopped = true;
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
class FakeAudioContext {
  constructor() {
    this.audioWorklet = {
      async addModule() {},
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

  async close() {}
}

class FakeAudioWorkletNode {
  constructor() {
    this.port = { onmessage: null };
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
const environment = {
  AbortController,
  AudioContext: FakeAudioContext,
  AudioWorkletNode: FakeAudioWorkletNode,
  Blob: FakeBlob,
  Date,
  clearTimeout,
  isSecureContext: true,
  localStorage: {
    getItem() {
      return null;
    },
    setItem() {},
  },
  navigator: {
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

function emit(level, nowMs) {
  monotonicNow = nowMs;
  const samples = new Float32Array(4800);
  samples.fill(level);
  activeWorklet.port.onmessage({
    data: {
      type: "samples",
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
  elements["microphone-button"].dispatch("click");
  await tick();
  await tick();
  if (!activeWorklet || typeof activeWorklet.port.onmessage !== "function") {
    throw new Error("microphone capture did not start");
  }

  emit(0.5, 100);
  emit(0.5, 200);
  for (let nowMs = 300; nowMs <= 1000; nowMs += 100) {
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
  const finalPhase = microphone.phase;
  microphone.destroy();

  process.stdout.write(JSON.stringify({
    beforeCancel,
    cancelledSignal,
    callSequence: calls.map((call) => ({
      method: call.options.method,
      path: call.path,
    })),
    cancellationPath: cancellation ? cancellation.path : null,
    finalPhase,
    postRequestId: post.options.headers["X-Robot-STT-Request-ID"],
    transcriptCount: transcripts.length,
  }));
}

run().catch((error) => {
  process.stderr.write(`${error.stack || error}\n`);
  process.exitCode = 1;
});
