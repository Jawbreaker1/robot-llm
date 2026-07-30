"use strict";

const CHUNK_FRAMES = 1024;

class RobotPCMCaptureProcessor extends AudioWorkletProcessor {
  constructor() {
    super();
    this.buffer = new Float32Array(CHUNK_FRAMES);
    this.offset = 0;
    this.captureGeneration = null;
    this.port.onmessage = (event) => {
      this.handleControl(event && event.data);
    };
  }

  resetBuffer() {
    this.buffer = new Float32Array(CHUNK_FRAMES);
    this.offset = 0;
  }

  handleControl(message) {
    if (
      !message
      || message.type !== "capture-control"
      || !Number.isSafeInteger(message.captureGeneration)
      || message.captureGeneration <= 0
    ) {
      return;
    }
    if (message.action === "start") {
      this.captureGeneration = message.captureGeneration;
      this.resetBuffer();
    } else if (
      message.action === "stop"
      && message.captureGeneration === this.captureGeneration
    ) {
      this.captureGeneration = null;
      this.resetBuffer();
    }
  }

  process(inputs) {
    if (this.captureGeneration === null) {
      return true;
    }
    const channels = inputs[0];
    if (!channels || channels.length === 0 || channels[0].length === 0) {
      return true;
    }
    const frameCount = channels[0].length;
    for (let frame = 0; frame < frameCount; frame += 1) {
      let sum = 0;
      for (let channel = 0; channel < channels.length; channel += 1) {
        sum += channels[channel][frame] || 0;
      }
      this.buffer[this.offset] = sum / channels.length;
      this.offset += 1;
      if (this.offset === this.buffer.length) {
        const samples = this.buffer;
        this.port.postMessage(
          {
            type: "samples",
            captureGeneration: this.captureGeneration,
            samples: samples.buffer,
          },
          [samples.buffer],
        );
        this.resetBuffer();
      }
    }
    return true;
  }
}

registerProcessor("robot-pcm-capture", RobotPCMCaptureProcessor);
