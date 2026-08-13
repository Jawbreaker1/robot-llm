from html.parser import HTMLParser
import json
from pathlib import Path
import re
import subprocess
import unittest

from robot_agent.dashboard_contract import (
    EXPERIMENT_SUMMARY_KEYS,
    EXPERIMENT_TITLE_KEYS,
    REGISTRY_DISPLAY_NAME_KEYS,
    RESPONSE_LOCALES,
)


WEB_ROOT = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "robot_agent"
    / "dashboard_web"
)


class AssetParser(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.ids = []
        self.attributes = []
        self.elements = []
        self.scripts = []
        self.script_elements = []
        self.links = []
        self._script_without_src = False
        self.inline_script_data = []

    def handle_starttag(self, tag, attrs):
        values = dict(attrs)
        self.elements.append((tag, values))
        self.attributes.extend((tag, name, value) for name, value in attrs)
        if "id" in values:
            self.ids.append(values["id"])
        if tag == "script":
            self.scripts.append(values.get("src"))
            self.script_elements.append(values)
            self._script_without_src = "src" not in values
        if tag == "link":
            self.links.append(values.get("href"))

    def handle_endtag(self, tag):
        if tag == "script":
            self._script_without_src = False

    def handle_data(self, data):
        if self._script_without_src and data.strip():
            self.inline_script_data.append(data)


class DashboardWebContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = (WEB_ROOT / "index.html").read_text(encoding="utf-8")
        cls.css = (WEB_ROOT / "styles.css").read_text(encoding="utf-8")
        cls.i18n = (WEB_ROOT / "i18n.js").read_text(encoding="utf-8")
        cls.dashboard_logic = (
            WEB_ROOT / "dashboard_logic.js"
        ).read_text(encoding="utf-8")
        cls.blast_map_semantics = (
            WEB_ROOT / "blast_map_semantics.js"
        ).read_text(encoding="utf-8")
        cls.speech_input_logic = (
            WEB_ROOT / "speech_input_logic.js"
        ).read_text(encoding="utf-8")
        cls.controller_panel = (
            WEB_ROOT / "controller_panel.js"
        ).read_text(encoding="utf-8")
        cls.microphone_input = (
            WEB_ROOT / "microphone_input.js"
        ).read_text(encoding="utf-8")
        cls.pcm_capture_worklet = (
            WEB_ROOT / "pcm_capture_worklet.js"
        ).read_text(encoding="utf-8")
        cls.spatial_map_presenter = (
            WEB_ROOT / "spatial_map_presenter.js"
        ).read_text(encoding="utf-8")
        cls.robot_mission_panel = (
            WEB_ROOT / "robot_mission_panel.js"
        ).read_text(encoding="utf-8")
        cls.robot_control = (
            WEB_ROOT / "robot_control.js"
        ).read_text(encoding="utf-8")
        cls.javascript = (WEB_ROOT / "app.js").read_text(encoding="utf-8")
        cls.javascript_assets = "\n".join(
            (
                cls.i18n,
                cls.blast_map_semantics,
                cls.dashboard_logic,
                cls.controller_panel,
                cls.speech_input_logic,
                cls.microphone_input,
                cls.pcm_capture_worklet,
                cls.spatial_map_presenter,
                cls.robot_mission_panel,
                cls.robot_control,
                cls.javascript,
            )
        )
        cls.parser = AssetParser()
        cls.parser.feed(cls.html)
        cls.i18n_contract = cls._inspect_i18n_contract()
        cls.speech_input_contract = cls._inspect_speech_input_contract()

    @classmethod
    def _inspect_i18n_contract(cls):
        script = r"""
const fs = require("fs");
const vm = require("vm");
const source = fs.readFileSync(process.argv[1], "utf8");
const window = {};
vm.runInNewContext(source, { window, Intl }, { filename: process.argv[1] });
const api = window.RobotI18n;
if (!api || !api.CATALOGS || typeof api.resolveLocale !== "function") {
  throw new Error("i18n.js did not expose the RobotI18n contract");
}
const catalogs = {};
for (const [locale, catalog] of Object.entries(api.CATALOGS)) {
  const entries = Object.entries(catalog.messages || {});
  catalogs[locale] = {
    keys: entries.map(([key]) => key).sort(),
    values: Object.fromEntries(
      entries.filter(([, value]) => typeof value === "string"),
    ),
    invalidValues: entries
      .filter(([, value]) => (
        typeof value !== "function"
        && (typeof value !== "string" || value.trim().length === 0)
      ))
      .map(([key]) => key),
    formatLocale: catalog.formatLocale,
    direction: catalog.dir,
  };
}
const cases = {
  swedishRegion: ["sv-SE"],
  swedishExtension: ["sv-FI-u-ca-gregory"],
  britishEnglish: ["en-GB"],
  americanEnglish: ["en-US"],
  invalid: ["not_a_locale"],
  unsupported: ["fr-FR"],
  skipInvalidAndUnsupported: ["not_a_locale", "fr-FR", "en-US"],
};
const resolutions = {};
for (const [name, candidates] of Object.entries(cases)) {
  resolutions[name] = api.resolveLocale(candidates);
}
const copySamples = {};
for (const locale of Object.keys(api.CATALOGS)) {
  const instance = api.createDefaultI18n({
    candidates: [locale],
    environment: {},
  });
  copySamples[locale] = {
    droppedEvents: instance.t("events.gap", { count: "7" }),
    namedRobot: instance.t("workbench.target.named_robot", { name: "BLAST" }),
    unknownKey: instance.t("future.unknown.key"),
  };
}

const persistedReads = [];
const persistedEnvironment = {
  localStorage: {
    getItem(key) {
      persistedReads.push(key);
      return "en";
    },
    setItem() {},
  },
  navigator: {
    languages: ["sv-SE"],
    language: "sv-SE",
  },
  document: {
    documentElement: { lang: "sv" },
  },
};
const persisted = api.createDefaultI18n({
  environment: persistedEnvironment,
});

const blockedEnvironment = {
  localStorage: {
    getItem() {
      throw new Error("storage read blocked");
    },
    setItem() {
      throw new Error("storage write blocked");
    },
  },
  navigator: {
    languages: ["en-US"],
    language: "en-US",
  },
};
const blocked = api.createDefaultI18n({
  environment: blockedEnvironment,
});
const blockedInitial = {
  locale: blocked.locale,
  formatLocale: blocked.formatLocale,
};
let blockedSetSurvived = true;
try {
  blocked.setLocale("sv-SE");
} catch (_error) {
  blockedSetSurvived = false;
}

const storageWrites = [];
const switching = api.createDefaultI18n({
  environment: {
    localStorage: {
      getItem() {
        return null;
      },
      setItem(key, value) {
        storageWrites.push([key, value]);
      },
    },
    navigator: {
      languages: ["sv-SE"],
      language: "sv-SE",
    },
  },
});
const sampleNumber = 1234567.89;
const sampleDate = Date.UTC(2026, 6, 27, 10, 5, 0);
const dateOptions = {
  timeZone: "UTC",
  year: "numeric",
  month: "2-digit",
  day: "2-digit",
  hour: "2-digit",
  minute: "2-digit",
};
const swedishNumber = switching.number(sampleNumber);
const swedishDate = switching.dateTime(sampleDate, dateOptions);
const swedishPlural = switching.plural(2);
const swedishUnknown = switching.t("common.unknown");
const notifications = [];
const unsubscribe = switching.subscribe((instance) => {
  notifications.push({
    locale: instance.locale,
    formatLocale: instance.formatLocale,
  });
});
const switched = switching.setLocale("en-US");
const englishNumber = switching.number(sampleNumber);
const englishDate = switching.dateTime(sampleDate, dateOptions);
const englishPlural = switching.plural(2);
const englishUnknown = switching.t("common.unknown");
unsubscribe();
switching.setLocale("sv", { persist: false });

process.stdout.write(JSON.stringify({
  exports: Object.keys(api).sort(),
  catalogs,
  resolutions,
  copySamples,
  runtime: {
    persisted: {
      reads: persistedReads,
      locale: persisted.locale,
      formatLocale: persisted.formatLocale,
      translation: persisted.t("common.unknown"),
    },
    blockedStorage: {
      initial: blockedInitial,
      setSurvived: blockedSetSurvived,
      localeAfterSet: blocked.locale,
      formatLocaleAfterSet: blocked.formatLocale,
    },
    switching: {
      supportedLocales: switching.supportedLocales,
      switched,
      notifications,
      storageWrites,
      finalLocale: switching.locale,
      finalFormatLocale: switching.formatLocale,
      translationsChanged: swedishUnknown !== englishUnknown,
      numberChanged: swedishNumber !== englishNumber,
      formatterMatches: {
        swedishNumber: swedishNumber
          === new Intl.NumberFormat("sv-SE").format(sampleNumber),
        swedishDate: swedishDate
          === new Intl.DateTimeFormat("sv-SE", dateOptions).format(sampleDate),
        swedishPlural: swedishPlural
          === new Intl.PluralRules("sv-SE").select(2),
        englishNumber: englishNumber
          === new Intl.NumberFormat("en-US").format(sampleNumber),
        englishDate: englishDate
          === new Intl.DateTimeFormat("en-US", dateOptions).format(sampleDate),
        englishPlural: englishPlural
          === new Intl.PluralRules("en-US").select(2),
      },
    },
  },
}));
"""
        completed = subprocess.run(
            [
                "node",
                "--input-type=commonjs",
                "-e",
                script,
                str(WEB_ROOT / "i18n.js"),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
        )
        if completed.returncode != 0:
            raise AssertionError(completed.stderr)
        return json.loads(completed.stdout)

    @classmethod
    def _inspect_speech_input_contract(cls):
        script = r"""
const fs = require("fs");
const vm = require("vm");
const source = fs.readFileSync(process.argv[1], "utf8");
const context = {};
vm.runInNewContext(source, context, { filename: process.argv[1] });
const logic = context.RobotSpeechInputLogic;
if (
  !logic
  || typeof logic.normalizeSettings !== "function"
  || typeof logic.advanceVad !== "function"
  || typeof logic.resampleMono !== "function"
  || typeof logic.speechWindow !== "function"
  || typeof logic.encodePCM16Wav !== "function"
) {
  throw new Error("speech_input_logic.js did not expose its runtime contract");
}

const defaults = logic.normalizeSettings(null);
const bounded = logic.normalizeSettings({
  deviceId: "",
  language: "fr",
  sensitivity: 999,
  silenceMs: 1,
  maxUtteranceMs: 999999,
  echoCancellation: false,
  noiseSuppression: "yes",
  autoGainControl: false,
  keepReady: false,
  autoSend: false,
});
const sourcePcm = Float32Array.from([
  0, 0.25, 0.5, 0.75,
  1, 0.75, 0.5, 0.25,
  0, -0.25, -0.5, -0.75,
]);
const downsampled = logic.resampleMono(sourcePcm, 48000, 16000);
const windowed = logic.speechWindow(
  Float32Array.from({ length: 16000 }, (_value, index) => index),
  16000,
  { startedAtMs: 1000, speechStartedAtMs: 1800 },
);
const paddedWindow = logic.speechWindow(
  Float32Array.from([0.5, -0.5]),
  16000,
  { startedAtMs: 1000, speechStartedAtMs: 1000 },
);
const lateOnsetWindow = logic.speechWindow(
  Float32Array.from({ length: 32000 }, (_value, index) => index + 1),
  16000,
  { startedAtMs: 1000, speechStartedAtMs: 2200 },
);
const boundedPrerollWindow = logic.speechWindow(
  Float32Array.from({ length: 48000 }, (_value, index) => index + 1),
  16000,
  { startedAtMs: 1000, speechStartedAtMs: 3000 },
);
const wav = logic.encodePCM16Wav(downsampled, 16000);
const wavView = new DataView(wav.buffer, wav.byteOffset, wav.byteLength);
const ascii = (offset, length) => Array.from(
  wav.slice(offset, offset + length),
  (value) => String.fromCharCode(value),
).join("");

let vad = logic.createVadState(1000);
const vadActions = [];
[
  [1050, -64],
  [1250, -20],
  [1300, -20],
  [1500, -65],
  [2200, -65],
].forEach(([nowMs, sampleLevelDb]) => {
  const transition = logic.advanceVad(
    vad,
    { nowMs, levelDb: sampleLevelDb },
    { ...defaults, silenceMs: 600 },
  );
  vad = transition.state;
  vadActions.push(transition.action);
});
let quiet = logic.createVadState(0);
let quietAction = null;
for (const nowMs of [250, 1000, 3000, 5000]) {
  const transition = logic.advanceVad(
    quiet,
    { nowMs, levelDb: -70 },
    defaults,
  );
  quiet = transition.state;
  quietAction = transition.action;
}
let hysteresis = logic.createVadState(0);
const hysteresisActions = [];
[
  [250, -25],
  [300, -25],
  [1700, -51],
  [2800, -60],
  [3000, -60],
].forEach(([nowMs, sampleLevelDb]) => {
  const transition = logic.advanceVad(
    hysteresis,
    { nowMs, levelDb: sampleLevelDb },
    defaults,
  );
  hysteresis = transition.state;
  hysteresisActions.push(transition.action);
});

process.stdout.write(JSON.stringify({
  exports: Object.keys(logic).sort(),
  frozen: Object.isFrozen(logic),
  defaults,
  defaultsFrozen: Object.isFrozen(defaults),
  bounded,
  resampling: {
    length: downsampled.length,
    samples: Array.from(downsampled),
  },
  speechWindow: {
    length: windowed.length,
    first: windowed[0],
    paddedLength: paddedWindow.length,
    paddedFirst: paddedWindow[0],
    paddedSecond: paddedWindow[1],
    paddedLast: paddedWindow[paddedWindow.length - 1],
    lateOnsetLength: lateOnsetWindow.length,
    lateOnsetFirst: lateOnsetWindow[0],
    boundedPrerollLength: boundedPrerollWindow.length,
    boundedPrerollFirst: boundedPrerollWindow[0],
  },
  wav: {
    length: wav.length,
    riff: ascii(0, 4),
    wave: ascii(8, 4),
    format: wavView.getUint16(20, true),
    channels: wavView.getUint16(22, true),
    sampleRate: wavView.getUint32(24, true),
    bitsPerSample: wavView.getUint16(34, true),
    dataBytes: wavView.getUint32(40, true),
  },
  vadActions,
  vadPhase: vad.phase,
  quietAction,
  quietPhase: quiet.phase,
  hysteresisActions,
}));
"""
        completed = subprocess.run(
            [
                "node",
                "--input-type=commonjs",
                "-e",
                script,
                str(WEB_ROOT / "speech_input_logic.js"),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
        )
        if completed.returncode != 0:
            raise AssertionError(completed.stderr)
        return json.loads(completed.stdout)

    def test_html_has_one_token_placeholder_and_external_assets_only(self):
        self.assertEqual(
            self.html.count("__ROBOT_DASHBOARD_TOKEN__"),
            1,
        )
        self.assertEqual(
            self.parser.scripts,
            [
                "assets/i18n.js",
                "assets/blast_map_semantics.js",
                "assets/dashboard_logic.js",
                "assets/controller_panel.js",
                "assets/speech_input_logic.js",
                "assets/microphone_input.js",
                "assets/spatial_map_presenter.js",
                "assets/robot_mission_panel.js",
                "assets/robot_control.js",
                "assets/app.js",
            ],
        )
        self.assertTrue(
            all(
                "defer" in attributes
                for attributes in self.parser.script_elements
            )
        )
        self.assertIn("assets/styles.css", self.parser.links)
        self.assertEqual(
            self.html.count("assets/robot-llm-mascot.png"),
            1,
        )
        self.assertEqual(
            self.html.count("assets/robot-llm-head.png"),
            2,
        )
        self.assertIn(
            (
                "link",
                {
                    "rel": "icon",
                    "type": "image/png",
                    "href": "assets/robot-llm-head.png",
                },
            ),
            self.parser.elements,
        )
        self.assertIn(
            (
                "img",
                {
                    "src": "assets/robot-llm-head.png",
                    "alt": "",
                    "width": "512",
                    "height": "512",
                },
            ),
            self.parser.elements,
        )
        mascot = next(
            attributes
            for tag, attributes in self.parser.elements
            if tag == "img"
            and attributes.get("src") == "assets/robot-llm-mascot.png"
        )
        self.assertEqual(
            {
                name: mascot.get(name)
                for name in ("class", "src", "alt", "width", "height")
            },
            {
                "class": "welcome-mascot",
                "src": "assets/robot-llm-mascot.png",
                "alt": "Robot LLM Labs lätt griniga modulrobot vinkar",
                "width": "250",
                "height": "250",
            },
        )
        self.assertEqual(
            mascot.get("data-i18n-alt"),
            "workbench.welcome.mascot_alt",
        )
        for filename in (
            "robot-llm-mascot.png",
            "robot-llm-head.png",
        ):
            image = (WEB_ROOT / filename).read_bytes()
            self.assertTrue(image.startswith(b"\x89PNG\r\n\x1a\n"))
            self.assertEqual(image[25], 6)
            self.assertLess(len(image), 2 * 1024 * 1024)
        self.assertEqual(self.parser.inline_script_data, [])
        self.assertNotIn("<style", self.html.lower())

    def test_html_ids_core_surfaces_and_plain_language_welcome(self):
        self.assertEqual(
            len(self.parser.ids),
            len(set(self.parser.ids)),
        )
        required = {
            "view-workbench",
            "view-bodies",
            "view-map",
            "view-events",
            "view-experiments",
            "view-settings",
            "session-expired-notice",
            "message-feed",
            "composer-form",
            "registry-tree",
            "spatial-map-canvas",
            "map-path-layer",
            "map-empty-state",
            "map-empty-title",
            "map-empty-body",
            "map-metadata",
            "map-qualitative-list",
            "map-qualitative-count",
            "map-object-list",
            "map-mission-details",
            "map-mission-state",
            "map-mission-goal",
            "map-mission-action",
            "map-mission-plan-summary",
            "map-mission-plan",
            "map-mission-route-label",
            "map-mission-route-summary",
            "map-mission-route-heading",
            "map-mission-route",
            "map-mission-history-status",
            "map-mission-history-count",
            "map-mission-history-gap",
            "map-mission-timeline",
            "event-table-body",
            "settings-form",
            "status-motion",
            "status-ev3",
            "status-blast",
            "controller-details",
        }
        self.assertTrue(required <= set(self.parser.ids))
        self.assertIn('lang="sv"', self.html)

    def test_blast_runtime_is_rendered_as_read_only_controller_telemetry(self):
        self.assertIn('controller.controller_id === "blast-01.hub"', self.controller_panel)
        self.assertIn('"status-blast"', self.controller_panel)
        self.assertIn("runtimeObject.controllers", self.javascript)
        self.assertIn("controllerPanel.statusTone(ev3State)", self.javascript)
        self.assertIn('state === "connecting"', self.controller_panel)
        self.assertIn('state === "offline" || state === "stopped"', self.controller_panel)
        self.assertIn("observation.distance_mm", self.controller_panel)
        self.assertIn("observation.motor_angles_deg", self.controller_panel)
        self.assertIn(
            "/api/v1/controllers/${encodeURIComponent(controllerId)}/commands",
            self.controller_panel,
        )
        self.assertIn('"drive_forward"', self.controller_panel)
        self.assertIn('"claw_close"', self.controller_panel)
        self.assertIn('"body_right"', self.controller_panel)
        self.assertIn("pendingControllers", self.controller_panel)
        self.assertIn("pendingStops", self.controller_panel)
        self.assertIn("pendingConnections", self.controller_panel)
        self.assertIn(
            "/connection`",
            self.controller_panel,
        )
        self.assertIn('body: { action }', self.controller_panel)
        self.assertIn('"connect", "disconnect", "retry"', self.controller_panel)
        self.assertIn(
            'error.code === "controller_command_interrupted"',
            self.controller_panel,
        )
        self.assertIn(
            "Number.isFinite(runtime.last_observed_at_unix_ms)",
            self.controller_panel,
        )
        self.assertNotIn("set_speed", self.controller_panel)
        self.assertNotIn("speed_dps", self.controller_panel)
        self.assertIn(
            'node.status_reason_code !== "future_component"',
            self.controller_panel,
        )
        self.assertIn("controllerPanel.visibleRobots(", self.javascript)
        self.assertNotIn("blast.drive_pulse", self.controller_panel)
        self.assertIn("Hallå! Vad ska vi hitta på?", self.html)
        self.assertIn("Jag kör lokalt på din Mac.", self.html)
        self.assertNotIn("God kväll.", self.html)
        self.assertNotIn("godkända, skrivskyddade verktyg", self.html)
        self.assertNotIn("Ingen fysisk capability", self.html)
        self.assertNotIn("EV3RSTORM är offline just nu", self.html)
        self.assertNotIn("weather.current</span>", self.html)
        self.assertIn("Arbetsbänken är redo för samtal", self.i18n)
        self.assertNotIn(
            "Arbetsbänken är redo för samtal",
            self.javascript,
        )
        self.assertNotIn(
            "modellresultat saknar fysisk auktoritet",
            self.javascript_assets,
        )
        self.assertNotIn(
            "Read-only verktyg kan väljas semantiskt",
            self.javascript_assets,
        )
        for raw_value in (
            "Gemma",
            "EV3RSTORM",
            "EV3 Main",
            "LM Studio",
            "weather.current",
            "SSH",
            "TTS",
        ):
            with self.subTest(raw_value=raw_value):
                self.assertIn(">{}<".format(raw_value), self.html)

    def test_controller_connection_controls_render_and_submit_once(self):
        script = r"""
const fs = require("fs");
const vm = require("vm");
const source = fs.readFileSync(process.argv[1], "utf8");
const context = {};
vm.runInNewContext(source, context, { filename: process.argv[1] });

class Element {
  constructor(tag) {
    this.tag = tag;
    this.children = [];
    this.listeners = {};
    this.disabled = false;
    this.textContent = "";
  }
  appendChild(child) {
    this.children.push(child);
    return child;
  }
  replaceChildren(...children) {
    this.children = children;
  }
  addEventListener(name, listener) {
    this.listeners[name] = listener;
  }
  click() {
    if (!this.disabled && this.listeners.click) {
      return this.listeners.click();
    }
    return undefined;
  }
}

const document = { createElement: (tag) => new Element(tag) };
const requests = [];
let completeRequest;
const panel = context.RobotControllerPanel.create({
  document,
  translate: (key) => key,
  humanState: (state) => state,
  setStatus() {},
  formatDateTime: String,
  formatNumber: String,
  request: (url, options) => {
    requests.push({ url, options });
    return new Promise((resolve) => { completeRequest = resolve; });
  },
  showToast() {},
  onCommandComplete: async () => {},
});
const blastController = {
  node_kind: "controller",
  controller_id: "blast-01.hub",
  robot_id: "blast-01",
  display_name: "BLAST",
  lifecycle: "configured",
};
const container = new Element("main");

function buttonsFor(state) {
  panel.render(container, [blastController], [{
    controller_id: "blast-01.hub",
    state,
    observation: {},
  }]);
  const connectionSection = container.children[0].children[2];
  return connectionSection.children[1].children;
}

const states = {};
for (const state of ["configured", "connecting", "online", "offline", "stopped"]) {
  states[state] = buttonsFor(state).map((button) => button.disabled);
}
panel.render(container, [blastController], []);
const unconfiguredBlastDisabled = container.children[0].children[2]
  .children[1].children.map((button) => button.disabled);
const ev3Controller = {
  node_kind: "controller",
  controller_id: "ev3rstorm-01.ev3-main",
  robot_id: "ev3rstorm-01",
  display_name: "EV3RSTORM",
  lifecycle: "configured",
};

function ev3Button(status) {
  panel.render(container, [ev3Controller], [{
    controller_id: "ev3rstorm-01.ev3-main",
    state: "configured",
    reachability: { status },
    observation: {},
  }]);
  return container.children[0].children[2].children[1].children[0];
}

(async () => {
  const connect = buttonsFor("configured")[0];
  connect.click();
  connect.click();
  const pendingDisabled = container.children[0].children[2].children[1].children
    .map((button) => button.disabled);
  completeRequest({});
  await new Promise(setImmediate);
  const blastRequests = requests.splice(0);
  const retry = buttonsFor("connecting")[2];
  retry.click();
  retry.click();
  completeRequest({});
  await new Promise(setImmediate);
  const retryRequests = requests.splice(0);

  const ev3States = Object.fromEntries(
    ["not_checked", "checking", "passed", "failed"].map((status) => [
      status,
      panel.controllerDisplayState({
        controller_id: "ev3rstorm-01.ev3-main",
        state: "configured",
        reachability: { status },
      }),
    ]),
  );
  const checkingDisabled = ev3Button("checking").disabled;
  panel.render(container, [ev3Controller], []);
  const unavailableEV3Disabled = container.children[0].children[2]
    .children[1].children[0].disabled;
  const check = ev3Button("not_checked");
  check.click();
  check.click();
  const ev3PendingDisabled = container.children[0].children[2]
    .children[1].children[0].disabled;
  completeRequest({});
  await new Promise(setImmediate);
  process.stdout.write(JSON.stringify({
    states,
    unconfiguredBlastDisabled,
    pendingDisabled,
    blastRequests,
    retryRequests,
    ev3States,
    checkingDisabled,
    unavailableEV3Disabled,
    ev3PendingDisabled,
    ev3Requests: requests,
  }));
})().catch((error) => {
  process.stderr.write(String(error));
  process.exitCode = 1;
});
"""
        completed = subprocess.run(
            [
                "node",
                "--input-type=commonjs",
                "-e",
                script,
                str(WEB_ROOT / "controller_panel.js"),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
        )
        if completed.returncode != 0:
            self.fail(completed.stderr)
        result = json.loads(completed.stdout)
        self.assertEqual(result["states"]["configured"], [False, True, True])
        self.assertEqual(result["states"]["connecting"], [True, False, False])
        self.assertEqual(result["states"]["online"], [True, False, True])
        self.assertEqual(result["states"]["offline"], [True, False, False])
        self.assertEqual(result["states"]["stopped"], [False, True, True])
        self.assertEqual(result["unconfiguredBlastDisabled"], [True, True, True])
        self.assertEqual(result["pendingDisabled"], [True, True, True])
        self.assertEqual(len(result["blastRequests"]), 1)
        self.assertEqual(
            result["blastRequests"][0]["url"],
            "/api/v1/controllers/blast-01.hub/connection",
        )
        self.assertEqual(
            result["blastRequests"][0]["options"]["body"],
            {"action": "connect"},
        )
        self.assertEqual(len(result["retryRequests"]), 1)
        self.assertEqual(
            result["retryRequests"][0]["url"],
            "/api/v1/controllers/blast-01.hub/connection",
        )
        self.assertEqual(
            result["retryRequests"][0]["options"]["body"],
            {"action": "retry"},
        )
        self.assertEqual(result["ev3States"], {
            "not_checked": "configured",
            "checking": "connecting",
            "passed": "verified",
            "failed": "offline",
        })
        self.assertTrue(result["checkingDisabled"])
        self.assertTrue(result["unavailableEV3Disabled"])
        self.assertTrue(result["ev3PendingDisabled"])
        self.assertEqual(len(result["ev3Requests"]), 1)
        self.assertEqual(
            result["ev3Requests"][0]["url"],
            "/api/v1/controllers/ev3rstorm-01.ev3-main/reachability",
        )
        self.assertEqual(result["ev3Requests"][0]["options"]["body"], {})
        self.assertEqual(result["ev3Requests"][0]["options"]["timeout"], 70_000)

    def test_rejected_session_latches_once_and_stops_dashboard_pollers(self):
        notice = next(
            attributes
            for tag, attributes in self.parser.elements
            if tag == "div"
            and attributes.get("id") == "session-expired-notice"
        )
        self.assertEqual(notice.get("role"), "alert")
        self.assertEqual(notice.get("aria-live"), "assertive")
        self.assertIn("hidden", notice)
        self.assertIn(".session-expired-notice[hidden]", self.css)
        for key in (
            "session.expired.title",
            "session.expired.body",
        ):
            with self.subTest(key=key):
                self.assertIn('"{}"'.format(key), self.i18n)

        values = self.i18n_contract["catalogs"]
        self.assertEqual(
            values["sv"]["values"]["workbench.session.eyebrow"],
            "Aktuell konversation",
        )
        self.assertEqual(
            values["en"]["values"]["workbench.session.eyebrow"],
            "Current conversation",
        )
        self.assertEqual(
            values["sv"]["values"]["errors.session_token_rejected"],
            "Livekonsolens åtkomstnyckel avvisades.",
        )
        self.assertEqual(
            values["en"]["values"]["errors.session_token_rejected"],
            "The live-console access key was rejected.",
        )
        self.assertNotIn("Aktiv session", self.html)
        self.assertNotIn("Active session", self.i18n)

        self.assertIn("createDashboardRequest", self.dashboard_logic)
        self.assertIn("createSessionGuard", self.dashboard_logic)
        self.assertIn(
            'value.code === SESSION_REJECTED_CODE',
            self.dashboard_logic,
        )
        self.assertIn("value.status === 403", self.dashboard_logic)
        self.assertIn("sessionGuard.observe(requestError)", self.dashboard_logic)
        self.assertIn("sessionGuard.isExpired()", self.javascript)
        self.assertIn(
            'byId("session-expired-notice").hidden = false',
            self.javascript,
        )
        self.assertIn("sessionGuard.subscribe(stopPolling)", self.robot_control)
        self.assertIn("missionPanel.stopPolling()", self.robot_control)
        self.assertIn("function stopPolling()", self.robot_mission_panel)
        self.assertIn("if (!initialized || stopped)", self.robot_mission_panel)

    def test_spatial_map_surface_is_read_only_empty_and_provenance_aware(self):
        svg_elements = [
            attributes
            for tag, attributes in self.parser.elements
            if tag == "svg"
            and attributes.get("id") == "spatial-map-canvas"
        ]

        self.assertEqual(len(svg_elements), 1)
        self.assertEqual(svg_elements[0].get("role"), "img")
        local_odometry_layers = [
            attributes
            for tag, attributes in self.parser.elements
            if tag == "g"
            and attributes.get("id") == "map-local-odometry-layer"
        ]
        self.assertEqual(len(local_odometry_layers), 1)
        self.assertEqual(local_odometry_layers[0].get("role"), "group")
        self.assertEqual(
            local_odometry_layers[0].get("data-i18n-aria-label"),
            "map.local_odometry.aria_label",
        )
        mission_details = [
            attributes
            for tag, attributes in self.parser.elements
            if tag == "details"
            and attributes.get("id") == "map-mission-details"
        ]
        self.assertEqual(len(mission_details), 1)
        self.assertIn("map-mission-panel", self.html)
        self.assertIn("map-mission-timeline", self.html)
        self.assertIn("map-mission-route", self.html)
        self.assertIn("RobotMissionPanelUI.create", self.robot_control)
        self.assertIn("robot-spatial-map/v1", self.dashboard_logic)
        self.assertIn("normalizeSpatialMap", self.dashboard_logic)
        self.assertIn("selectSpatialMapEndpoint", self.javascript)
        self.assertIn('"/api/v1/map"', self.dashboard_logic)
        self.assertIn(".map-shared-robot-0", self.css)
        self.assertIn(".map-shared-robot-1", self.css)
        self.assertIn(
            "RobotSpatialMapPresenter.create",
            self.javascript,
        )
        self.assertIn(
            "map.sensorRays.forEach",
            self.spatial_map_presenter,
        )
        self.assertIn(
            "map.objectHypotheses.forEach",
            self.spatial_map_presenter,
        )
        self.assertIn(
            "map.robotPose.headingMdeg",
            self.spatial_map_presenter,
        )
        self.assertIn(
            'map.status === "degraded"',
            self.spatial_map_presenter,
        )
        self.assertIn(
            "map.qualitativeObservations",
            self.spatial_map_presenter,
        )
        self.assertIn(
            'map.status === "qualitative_only"',
            self.spatial_map_presenter,
        )
        self.assertIn(
            "renderLocalOdometryMap",
            self.spatial_map_presenter,
        )
        self.assertIn(
            '"data-geometry", "screen-space-nonmetric"',
            self.spatial_map_presenter,
        )
        self.assertIn(
            '"data-metric-distance": "none"',
            self.spatial_map_presenter,
        )
        self.assertIn(".map-local-ir-wedge", self.css)
        self.assertIn(".map-local-robot-heading", self.css)
        self.assertIn(".map-path", self.css)
        self.assertIn(".map-mission-panel", self.css)
        self.assertIn(".map-mission-timeline", self.css)
        self.assertIn(".map-mission-route-step.is-active", self.css)
        self.assertLess(len(self.javascript.splitlines()), 1900)
        self.assertNotIn("function mapProjection", self.javascript)
        self.assertIn("map.reason.observation_gap", self.i18n)
        self.assertIn("SIMULATION", self.i18n)
        self.assertIn("PROVISIONAL IR", self.i18n)
        self.assertIn("map.local_odometry.layer_label", self.i18n)
        self.assertIn("Ingen karta ännu", self.html)
        self.assertNotIn("map-drive", self.html)
        self.assertNotIn("map-waypoint", self.html)
        self.assertNotIn("map-stop", self.html)

    def test_enforced_blast_detour_is_visibly_distinct_from_reference(self):
        self.assertIn(
            '"data-navigation-enforced": String(goalEnforced)',
            self.spatial_map_presenter,
        )
        self.assertIn(
            '"map.navigation_trace.final_goal_enforced_label"',
            self.spatial_map_presenter,
        )
        self.assertIn(
            '"map.navigation_trace.local_detour_waypoint_label"',
            self.spatial_map_presenter,
        )
        self.assertIn(
            '.map-final-goal[data-navigation-enforced="true"] '
            ".map-final-goal-line",
            self.css,
        )
        self.assertIn(
            '.map-planned-leg[data-route-eligible="true"] '
            ".map-planned-leg-line",
            self.css,
        )
        self.assertIn("FINAL GOAL · ENFORCED", self.i18n)
        self.assertIn("FINAL-GOAL REFERENCE · NOT ENFORCED", self.i18n)
        self.assertIn("SLUTMÅL · STYRANDE", self.i18n)
        self.assertIn("SLUTMÅLSREFERENS · EJ STYRANDE", self.i18n)

    def test_shared_navigation_traces_keep_robot_color_and_caveats(self):
        self.assertIn(
            "appendNavigationTracePoints(\n          robot.navigationTrace",
            self.spatial_map_presenter,
        )
        self.assertIn(
            'class: `map-shared-navigation-trace ${robotClass}`',
            self.spatial_map_presenter,
        )
        self.assertIn(
            '"data-provisional": "true"',
            self.spatial_map_presenter,
        )
        self.assertIn(
            '"data-projection-frame": view.projectionFrame',
            self.spatial_map_presenter,
        )
        self.assertIn(
            '.map-shared-navigation-trace[data-provisional="true"]',
            self.css,
        )
        self.assertIn("stroke: currentColor", self.css)
        self.assertIn("fill: currentColor", self.css)
        self.assertIn(".map-shared-robot-7", self.css)
        self.assertLess(len(self.javascript.splitlines()), 1900)

    def test_speech_input_logic_is_bounded_and_emits_pcm16_wav(self):
        contract = self.speech_input_contract
        self.assertTrue(contract["frozen"])
        self.assertTrue(contract["defaultsFrozen"])
        self.assertEqual(
            set(contract["exports"]),
            {
                "DEFAULT_SETTINGS",
                "EARLIEST_SETTINGS_STORAGE_KEY",
                "LEGACY_SETTINGS_STORAGE_KEY",
                "SETTINGS_SCHEMA_VERSION",
                "SETTINGS_STORAGE_KEY",
                "TARGET_SAMPLE_RATE_HZ",
                "VAD_ACTION",
                "advanceVad",
                "concatenateSamples",
                "createVadState",
                "encodePCM16Wav",
                "levelDb",
                "meterPercent",
                "normalizeSettings",
                "resampleMono",
                "speechWindow",
                "thresholdDb",
            },
        )
        self.assertEqual(
            contract["defaults"],
            {
                "deviceId": "default",
                "language": "auto",
                "sensitivity": 65,
                "silenceMs": 1200,
                "maxUtteranceMs": 12000,
                "echoCancellation": True,
                "noiseSuppression": True,
                "autoGainControl": False,
                "keepReady": True,
                "autoSend": True,
            },
        )
        self.assertEqual(
            contract["bounded"],
            {
                "deviceId": "default",
                "language": "auto",
                "sensitivity": 100,
                "silenceMs": 400,
                "maxUtteranceMs": 30000,
                "echoCancellation": False,
                "noiseSuppression": True,
                "autoGainControl": False,
                "keepReady": False,
                "autoSend": False,
            },
        )
        self.assertEqual(contract["resampling"]["length"], 4)
        self.assertAlmostEqual(
            contract["resampling"]["samples"][0],
            0.25,
            places=5,
        )
        self.assertAlmostEqual(
            contract["resampling"]["samples"][-1],
            -0.5,
            places=5,
        )
        self.assertEqual(
            contract["speechWindow"],
            {
                "length": 16000,
                "first": 0,
                "paddedLength": 4000,
                "paddedFirst": 0.5,
                "paddedSecond": -0.5,
                "paddedLast": 0,
                "lateOnsetLength": 32000,
                "lateOnsetFirst": 1,
                "boundedPrerollLength": 40000,
                "boundedPrerollFirst": 8001,
            },
        )
        self.assertEqual(
            contract["wav"],
            {
                "length": 52,
                "riff": "RIFF",
                "wave": "WAVE",
                "format": 1,
                "channels": 1,
                "sampleRate": 16000,
                "bitsPerSample": 16,
                "dataBytes": 8,
            },
        )

    def test_speech_vad_requires_voice_then_stops_after_silence(self):
        contract = self.speech_input_contract
        self.assertEqual(
            contract["vadActions"],
            [
                "none",
                "none",
                "speech_started",
                "none",
                "stop_silence",
            ],
        )
        self.assertEqual(contract["vadPhase"], "speech")
        self.assertEqual(contract["quietAction"], "stop_no_speech")
        self.assertEqual(contract["quietPhase"], "waiting")
        self.assertEqual(
            contract["hysteresisActions"],
            [
                "none",
                "speech_started",
                "none",
                "none",
                "stop_silence",
            ],
        )

    def test_speech_window_retains_late_onset_with_bounded_preroll(self):
        window = self.speech_input_contract["speechWindow"]

        self.assertEqual(window["lateOnsetLength"], 32000)
        self.assertEqual(window["lateOnsetFirst"], 1)
        self.assertEqual(window["boundedPrerollLength"], 40000)
        self.assertEqual(window["boundedPrerollFirst"], 8001)

    def test_microphone_ui_is_accessible_explicit_and_browser_local(self):
        elements_by_id = {
            attrs["id"]: (tag, attrs)
            for tag, attrs in self.parser.elements
            if "id" in attrs
        }
        required = {
            "microphone-button",
            "microphone-status",
            "microphone-meter",
            "cancel-transcription-button",
            "microphone-settings-form",
            "microphone-device",
            "speech-input-language",
            "microphone-sensitivity",
            "microphone-silence-ms",
            "microphone-max-utterance-ms",
            "microphone-echo-cancellation",
            "microphone-noise-suppression",
            "microphone-auto-gain",
            "microphone-keep-ready",
            "microphone-auto-send",
            "microphone-settings-meter",
        }
        self.assertTrue(required <= set(elements_by_id))
        _, button = elements_by_id["microphone-button"]
        self.assertEqual(button.get("type"), "button")
        self.assertEqual(button.get("aria-pressed"), "false")
        self.assertEqual(
            button.get("aria-describedby"),
            "microphone-status",
        )
        _, status = elements_by_id["microphone-status"]
        self.assertEqual(status.get("role"), "status")
        self.assertEqual(status.get("aria-live"), "polite")
        for meter_id in ("microphone-meter", "microphone-settings-meter"):
            _, meter = elements_by_id[meter_id]
            self.assertEqual(meter.get("role"), "meter")
            self.assertEqual(
                (
                    meter.get("aria-valuemin"),
                    meter.get("aria-valuemax"),
                    meter.get("aria-valuenow"),
                ),
                ("0", "100", "0"),
            )
            self.assertIn("aria-valuetext", meter)
        _, auto_send = elements_by_id["microphone-auto-send"]
        self.assertEqual(auto_send.get("type"), "checkbox")
        self.assertIn("checked", auto_send)
        _, keep_ready = elements_by_id["microphone-keep-ready"]
        self.assertEqual(keep_ready.get("type"), "checkbox")
        self.assertIn("checked", keep_ready)
        self.assertIn(
            "robot-dashboard-microphone-v1",
            self.speech_input_logic,
        )
        self.assertIn(
            "robot-dashboard-microphone-v2",
            self.speech_input_logic,
        )
        self.assertIn(
            "robot-dashboard-microphone-v3",
            self.speech_input_logic,
        )
        self.assertNotIn(
            "microphone-device",
            self.javascript[
                self.javascript.index("  function settingsFromForm()"):
                self.javascript.index("\n  function settingsChanges()")
            ],
        )

    def test_conversation_target_is_clear_localized_and_shared_by_stt(self):
        elements = [
            (tag, attrs)
            for tag, attrs in self.parser.elements
            if attrs.get("id") == "composer-target"
        ]
        self.assertEqual(len(elements), 1)
        self.assertEqual(elements[0][0], "select")
        self.assertEqual(
            elements[0][1].get("aria-describedby"),
            "composer-target-help",
        )
        workbench_option = next(
            attrs
            for tag, attrs in self.parser.elements
            if tag == "option"
            and attrs.get("data-i18n") == "workbench.target.workbench"
        )
        robot_options = [
            attrs
            for tag, attrs in self.parser.elements
            if tag == "option"
            and attrs.get("value") in {"ev3rstorm-01", "blast-01"}
        ]
        self.assertEqual(
            [attrs.get("value") for attrs in robot_options],
            ["ev3rstorm-01", "blast-01"],
        )
        self.assertIn("selected", workbench_option)
        self.assertTrue(all("selected" not in attrs for attrs in robot_options))
        values = self.i18n_contract["catalogs"]
        self.assertEqual(
            values["sv"]["values"]["workbench.target.label"],
            "Vem pratar du med?",
        )
        self.assertEqual(
            values["en"]["values"]["workbench.target.label"],
            "Who are you talking to?",
        )
        for locale in ("sv", "en"):
            self.assertIn(
                "transkript"
                if locale == "sv"
                else "transcripts",
                values[locale]["values"]["workbench.target.help"],
            )
        self.assertIn(
            "await submitCurrentContent(selectedConversationTarget())",
            self.javascript,
        )
        transcript_handler = self.javascript[
            self.javascript.index("      onTranscript: (text, metadata) => {"):
            self.javascript.index(
                "\n      onError:",
                self.javascript.index(
                    "      onTranscript: (text, metadata) => {"
                ),
            )
        ]
        self.assertIn(
            "const target = microphoneTarget || selectedConversationTarget()",
            transcript_handler,
        )
        self.assertIn("void submitCurrentContent(target)", transcript_handler)
        self.assertIn(
            'byId("microphone-button").addEventListener("click"',
            self.javascript,
        )
        target_listener = self.javascript[
            self.javascript.index(
                'byId("composer-target").addEventListener("change"'
            ):
            self.javascript.index(
                '\n    byId("message-input").addEventListener("keydown"',
                self.javascript.index(
                    'byId("composer-target").addEventListener("change"'
                ),
            )
        ]
        self.assertIn("microphoneInput.cancel()", target_listener)

    def test_microphone_capture_is_generic_pcm_and_uses_the_stt_contract(self):
        combined = "\n".join(
            (
                self.speech_input_logic,
                self.microphone_input,
                self.pcm_capture_worklet,
                self.javascript,
            )
        )
        for forbidden in (
            "SpeechRecognition",
            "webkitSpeechRecognition",
            "MediaRecorder",
            "navigator.userAgent",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, combined)
        for required in (
            "getUserMedia",
            "enumerateDevices",
            "AudioWorkletNode",
            "audio/wav",
            "X-Robot-STT-Request-ID",
            "X-Robot-STT-Language",
            "/api/v1/stt/transcriptions",
            "/api/v1/stt/requests/",
            "pcm_capture_worklet.js",
            "requestSubmit()",
            'method: "DELETE"',
            '"stt_expired"',
            "valid_until_unix_ms",
            "speechWindow(",
        ):
            with self.subTest(required=required):
                self.assertIn(required, combined)
        self.assertIn(
            "safeObject(capabilities.speech_to_text)",
            self.javascript,
        )
        self.assertIn(
            "generation !== this.generation",
            self.microphone_input,
        )
        self.assertIn("requestController.abort()", self.microphone_input)
        self.assertIn("getTracks().forEach", self.microphone_input)
        self.assertIn("await value.context.close()", self.microphone_input)
        self.assertNotIn("innerHTML", self.microphone_input)
        self.assertIn(
            'meter.setAttribute(\n          "aria-valuetext"',
            self.microphone_input,
        )
        self.assertIn(
            '"microphone.meter.value"',
            self.microphone_input,
        )

    def _run_microphone_harness(self):
        harness = (
            Path(__file__).resolve().parent
            / "browser_microphone_cancel_harness.js"
        )
        completed = subprocess.run(
            [
                "node",
                str(harness),
                str(WEB_ROOT / "speech_input_logic.js"),
                str(WEB_ROOT / "microphone_input.js"),
                str(WEB_ROOT / "pcm_capture_worklet.js"),
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=10,
        )
        return json.loads(completed.stdout)

    def test_cancel_before_post_response_uses_request_id_and_drops_late_result(
        self,
    ):
        result = self._run_microphone_harness()

        self.assertEqual(
            result["beforeCancel"],
            [
                {
                    "method": "POST",
                    "path": "/api/v1/stt/transcriptions",
                }
            ],
        )
        self.assertEqual(
            result["callSequence"],
            [
                {
                    "method": "POST",
                    "path": "/api/v1/stt/transcriptions",
                },
                {
                    "method": "DELETE",
                    "path": (
                        "/api/v1/stt/requests/"
                        "stt-fixed-request"
                    ),
                },
            ],
        )
        self.assertTrue(result["cancelledSignal"])
        self.assertEqual(
            result["postRequestId"],
            "stt-fixed-request",
        )
        self.assertEqual(
            result["cancellationPath"],
            "/api/v1/stt/requests/stt-fixed-request",
        )
        self.assertEqual(result["transcriptCount"], 0)
        self.assertEqual(result["finalPhase"], "cancelled")

    def test_spoken_language_follows_ui_until_explicit_override(self):
        result = self._run_microphone_harness()

        self.assertEqual(result["swedishUiDefault"], "sv")
        self.assertEqual(result["englishUiDefault"], "en")
        self.assertEqual(result["explicitOverrideAfterUiChange"], "sv")
        self.assertEqual(result["explicitOverrideAfterReload"], "sv")
        self.assertTrue(result["persistedLanguageExplicit"])
        self.assertEqual(result["persistedSchemaVersion"], 3)

    def test_legacy_microphone_profile_migrates_without_losing_device(self):
        migrated = self._run_microphone_harness()["migratedSettings"]

        self.assertEqual(migrated["schemaVersion"], 3)
        self.assertEqual(migrated["deviceId"], "razer-device-id")
        self.assertEqual(migrated["language"], "en")
        self.assertFalse(migrated["languageExplicit"])
        self.assertEqual(migrated["silenceMs"], 1200)
        self.assertEqual(migrated["sensitivity"], 72)
        self.assertEqual(migrated["maxUtteranceMs"], 17000)
        self.assertFalse(migrated["echoCancellation"])
        self.assertFalse(migrated["noiseSuppression"])
        self.assertFalse(migrated["autoGainControl"])
        self.assertTrue(migrated["keepReady"])
        self.assertFalse(migrated["autoSend"])

    def test_microphone_reuses_warm_pipeline_and_vad_starts_after_setup(self):
        result = self._run_microphone_harness()
        expected_warm = {
            "audioContextCloses": 0,
            "audioContextCreations": 1,
            "getUserMediaCalls": 1,
            "trackStopCalls": 0,
            "workletModuleLoads": 1,
        }
        self.assertEqual(result["permissionPrimePhase"], "ready")
        self.assertEqual(
            result["permissionPrimeResources"],
            expected_warm,
        )
        self.assertEqual(
            result["warmResourcesAfterFirstTurn"],
            expected_warm,
        )
        self.assertEqual(
            result["warmResourcesDuringSecondTurn"],
            expected_warm,
        )
        self.assertEqual(result["secondTurnPhase"], "listening")
        self.assertTrue(result["portHandlerAfterFirstTurn"])
        self.assertEqual(
            result["resourcesAfterDestroy"],
            {
                **expected_warm,
                "audioContextCloses": 1,
                "trackStopCalls": 1,
            },
        )
        self.assertEqual(result["workletPortCloseCalls"], 1)

        start_source = self.microphone_input[
            self.microphone_input.index("    async start() {"):
            self.microphone_input.index(
                "\n    _receiveSamples(",
                self.microphone_input.index("    async start() {"),
            )
        ]
        self.assertLess(
            start_source.index("await this._ensureAudioReady()"),
            start_source.index(
                "this.vad = this.logic.createVadState(this._now())"
            ),
        )
        self.assertNotIn("this._openStream()", start_source)
        pipeline_source = self.microphone_input[
            self.microphone_input.index("    async _ensureAudioReady() {"):
            self.microphone_input.index(
                "\n    _effectiveSettings()",
                self.microphone_input.index(
                    "    async _ensureAudioReady() {"
                ),
            )
        ]
        self.assertLess(
            pipeline_source.index(
                "resources.stream = await this._openStream()"
            ),
            pipeline_source.index(
                "await resources.context.audioWorklet.addModule"
            ),
        )
        self.assertIn("void this.refreshDevices(false)", pipeline_source)

    def test_warm_microphone_never_buffers_audio_between_turns(self):
        result = self._run_microphone_harness()

        self.assertTrue(result["idlePortHandlerInstalled"])
        self.assertEqual(result["controlsBeforeFirstTalk"], [])
        self.assertEqual(result["idleChunkPhase"], "ready")
        self.assertEqual(result["staleChunkPhase"], "listening")
        self.assertEqual(
            result["captureControls"],
            [
                {
                    "type": "capture-control",
                    "action": "start",
                    "captureGeneration": 1,
                },
                {
                    "type": "capture-control",
                    "action": "stop",
                    "captureGeneration": 1,
                },
                {
                    "type": "capture-control",
                    "action": "start",
                    "captureGeneration": 3,
                },
                {
                    "type": "capture-control",
                    "action": "stop",
                    "captureGeneration": 3,
                },
            ],
        )
        protocol = result["workletProtocol"]
        self.assertEqual(protocol["idleOutputCount"], 0)
        self.assertEqual(protocol["idleOffset"], 0)
        self.assertEqual(protocol["partialOffset"], 512)
        self.assertEqual(protocol["stoppedOffset"], 0)
        self.assertEqual(
            protocol["outputsAfterStaleStop"],
            [
                {
                    "type": "samples",
                    "captureGeneration": 8,
                    "sampleBytes": 4096,
                },
                {
                    "type": "samples",
                    "captureGeneration": 8,
                    "sampleBytes": 4096,
                },
            ],
        )
        self.assertEqual(protocol["outputCountAfterFinalStop"], 2)
        self.assertIsNone(protocol["activeGenerationAfterStop"])

    def test_disabling_keep_ready_releases_pipeline_and_reports_idle(self):
        result = self._run_microphone_harness()

        self.assertEqual(result["phaseAfterKeepReadyDisabled"], "idle")
        self.assertEqual(
            result["resourcesAfterDestroy"],
            {
                "audioContextCloses": 1,
                "audioContextCreations": 1,
                "getUserMediaCalls": 1,
                "trackStopCalls": 1,
                "workletModuleLoads": 1,
            },
        )

    def test_microphone_number_fields_commit_without_fighting_keyboard_input(self):
        settings_source = self.microphone_input[
            self.microphone_input.index("    _settingsFromForm() {"):
            self.microphone_input.index(
                "\n    async initialize()",
                self.microphone_input.index("    _settingsFromForm() {"),
            )
        ]
        self.assertIn('silenceMs === ""', settings_source)
        self.assertIn('maxUtteranceMs === ""', settings_source)
        self.assertIn(
            "event.target === this.elements.sensitivity",
            settings_source,
        )
        input_listener = settings_source[
            settings_source.index(
                'this.elements.settingsForm.addEventListener(\n        "input"'
            ):
            settings_source.index(
                'this.elements.settingsForm.addEventListener(\n        "change"'
            )
        ]
        self.assertNotIn("this.elements.silenceMs", input_listener)
        self.assertNotIn("this.elements.maxUtteranceMs", input_listener)

    def test_active_agent_turn_cancels_and_disables_microphone(self):
        render_turn = self.javascript[
            self.javascript.index("  function renderTurn(turn) {"):
            self.javascript.index(
                "\n  function renderConversationSubtitle()",
                self.javascript.index("  function renderTurn(turn) {"),
            )
        ]
        submit_content = self.javascript[
            self.javascript.index("  async function submitCurrentContent(target) {"):
            self.javascript.index(
                "\n  async function submitTurn(event) {",
                self.javascript.index(
                    "  async function submitCurrentContent(target) {"
                ),
            )
        ]
        self.assertGreaterEqual(
            render_turn.count(
                "enforceCapabilities(safeObject(state.bootstrap))"
            ),
            2,
        )
        self.assertIn("microphoneInput.cancel()", submit_content)

    def test_device_refresh_cannot_replace_an_active_capture_phase(self):
        render_devices = self.microphone_input[
            self.microphone_input.index("    _renderDeviceOptions() {"):
            self.microphone_input.index(
                "\n    async refreshDevices(",
                self.microphone_input.index("    _renderDeviceOptions() {"),
            )
        ]
        self.assertIn(
            "if (!ACTIVE_PHASES.has(this.phase))",
            render_devices,
        )
        self.assertIn(
            'this._setPhase("device_fallback")',
            render_devices,
        )
        refresh_devices = self.microphone_input[
            self.microphone_input.index("    async refreshDevices("):
            self.microphone_input.index(
                "\n    _supportedAudioConstraints()",
                self.microphone_input.index("    async refreshDevices("),
            )
        ]
        self.assertIn(
            "if (ACTIVE_PHASES.has(this.phase))",
            refresh_devices,
        )
        self.assertIn(
            "this.onError(this.translate(STATUS_KEYS.device_missing))",
            refresh_devices,
        )

    def test_device_fallback_keeps_requesting_phase_until_stream_opens(self):
        open_stream = self.microphone_input[
            self.microphone_input.index("    async _openStream() {"):
            self.microphone_input.index(
                "\n    _effectiveSettings()",
                self.microphone_input.index("    async _openStream() {"),
            )
        ]
        self.assertIn(
            "return mediaDevices.getUserMedia(",
            open_stream,
        )
        self.assertNotIn(
            'this._setPhase("device_fallback")',
            open_stream,
        )

    def test_unsupported_browser_without_media_devices_initializes_safely(self):
        bind_source = self.microphone_input[
            self.microphone_input.index("    _bind() {"):
            self.microphone_input.index(
                "\n    async initialize()",
                self.microphone_input.index("    _bind() {"),
            )
        ]
        self.assertIn(
            "this.environment.navigator\n        "
            "&& this.environment.navigator.mediaDevices",
            bind_source,
        )
        self.assertIn(
            "mediaDevices\n        "
            '&& typeof mediaDevices.addEventListener === "function"',
            bind_source,
        )

    def test_i18n_catalogs_have_the_exact_same_nonempty_keyset(self):
        self.assertTrue(
            {
                "CATALOGS",
                "createDefaultI18n",
                "createI18n",
                "resolveLocale",
            }
            <= set(self.i18n_contract["exports"]),
        )
        catalogs = self.i18n_contract["catalogs"]
        self.assertEqual(set(catalogs), {"sv", "en"})
        self.assertGreater(len(catalogs["sv"]["keys"]), 0)
        self.assertEqual(catalogs["sv"]["keys"], catalogs["en"]["keys"])
        self.assertEqual(catalogs["sv"]["invalidValues"], [])
        self.assertEqual(catalogs["en"]["invalidValues"], [])
        self.assertEqual(catalogs["sv"]["formatLocale"], "sv-SE")
        self.assertEqual(catalogs["en"]["formatLocale"], "en-GB")
        self.assertEqual(catalogs["sv"]["direction"], "ltr")
        self.assertEqual(catalogs["en"]["direction"], "ltr")

    def test_locale_codes_match_html_javascript_and_python_contract(self):
        selector_values = [
            attributes["value"]
            for tag, attributes in self.parser.elements
            if tag == "option"
            and attributes.get("data-i18n")
            in {"locale.swedish", "locale.english"}
        ]

        self.assertEqual(selector_values, list(RESPONSE_LOCALES))
        self.assertEqual(
            list(self.i18n_contract["catalogs"]),
            list(RESPONSE_LOCALES),
        )
        self.assertEqual(
            self.i18n_contract["runtime"]["switching"][
                "supportedLocales"
            ],
            list(RESPONSE_LOCALES),
        )

    def test_curated_experiment_and_registry_keys_are_catalog_backed(self):
        catalogs = self.i18n_contract["catalogs"]
        expected_keys = (
            set(EXPERIMENT_TITLE_KEYS)
            | set(EXPERIMENT_SUMMARY_KEYS)
            | set(REGISTRY_DISPLAY_NAME_KEYS)
        )
        for locale in RESPONSE_LOCALES:
            with self.subTest(locale=locale):
                self.assertTrue(
                    expected_keys <= set(catalogs[locale]["keys"])
                )
                for key in expected_keys:
                    self.assertTrue(catalogs[locale]["values"][key])

        for key in EXPERIMENT_TITLE_KEYS + EXPERIMENT_SUMMARY_KEYS:
            with self.subTest(key=key):
                self.assertNotEqual(
                    catalogs["sv"]["values"][key],
                    catalogs["en"]["values"][key],
                )

        render_start = self.javascript.index(
            "  function renderExperiments(experiments) {"
        )
        render_end = self.javascript.index(
            "\n  function renderBootstrap(",
            render_start,
        )
        render_source = self.javascript[render_start:render_end]
        self.assertIn("localizedCatalogValue(", render_source)
        self.assertIn("experiment.title_key", render_source)
        self.assertIn("experiment.summary_key", render_source)
        self.assertIn(
            'safeText(experiment.title, t("experiments.untitled"))',
            render_source,
        )
        self.assertIn(
            'safeText(experiment.summary, t("experiments.no_summary"))',
            render_source,
        )
        self.assertIn("node.display_name_key", self.javascript)
        self.assertIn("robot.display_name_key", self.javascript)

    def test_swedish_dynamic_labels_share_canonical_static_copy(self):
        values = self.i18n_contract["catalogs"]["sv"]["values"]
        equivalent_pairs = (
            (
                "registry.field.controller_id",
                "bodies.controller.controller_id",
            ),
            (
                "registry.field.instance_id",
                "bodies.controller.instance_id",
            ),
            (
                "registry.field.physical_capabilities",
                "bodies.controller.physical_capabilities",
            ),
            (
                "chat.context.conversation_id",
                "inspector.context.conversation_id",
            ),
            (
                "chat.context.version",
                "inspector.context.context_version",
            ),
            (
                "chat.context.turn_count",
                "inspector.context.turn_count",
            ),
            ("events.field.time", "events.table.time"),
            ("events.field.level", "events.table.severity"),
        )
        for dynamic_key, static_key in equivalent_pairs:
            with self.subTest(
                dynamic_key=dynamic_key,
                static_key=static_key,
            ):
                self.assertEqual(
                    values[dynamic_key],
                    values[static_key],
                )

        self.assertEqual(
            {
                key: values[key]
                for key in (
                    "events.field.event_id",
                    "events.field.source_id",
                    "events.field.robot_id",
                    "events.field.node_id",
                    "events.field.turn_id",
                    "events.field.tool_call_id",
                    "events.field.request_id",
                )
            },
            {
                "events.field.event_id": "Händelse-ID",
                "events.field.source_id": "Käll-ID",
                "events.field.robot_id": "Robot-ID",
                "events.field.node_id": "Nod-ID",
                "events.field.turn_id": "Tur-ID",
                "events.field.tool_call_id": "Verktygsanrops-ID",
                "events.field.request_id": "Begärans-ID",
            },
        )

    def test_locale_resolver_uses_intl_locale_without_string_heuristics(self):
        self.assertIn("new Intl.Locale(", self.i18n)
        for forbidden in (
            "RegExp(",
            ".match(",
            ".test(",
            ".startsWith(",
            ".split(",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, self.i18n)

        self.assertEqual(
            self.i18n_contract["resolutions"],
            {
                "swedishRegion": {
                    "locale": "sv",
                    "formatLocale": "sv-SE",
                    "direction": "ltr",
                },
                "swedishExtension": {
                    "locale": "sv",
                    "formatLocale": "sv-FI",
                    "direction": "ltr",
                },
                "britishEnglish": {
                    "locale": "en",
                    "formatLocale": "en-GB",
                    "direction": "ltr",
                },
                "americanEnglish": {
                    "locale": "en",
                    "formatLocale": "en-US",
                    "direction": "ltr",
                },
                "invalid": {
                    "locale": "sv",
                    "formatLocale": "sv-SE",
                    "direction": "ltr",
                },
                "unsupported": {
                    "locale": "sv",
                    "formatLocale": "sv-SE",
                    "direction": "ltr",
                },
                "skipInvalidAndUnsupported": {
                    "locale": "en",
                    "formatLocale": "en-US",
                    "direction": "ltr",
                },
            },
        )

    def test_i18n_runtime_persists_switches_and_rebuilds_formatters(self):
        runtime = self.i18n_contract["runtime"]
        self.assertEqual(
            runtime["persisted"]["reads"],
            ["robot-dashboard-locale"],
        )
        self.assertEqual(runtime["persisted"]["locale"], "en")
        self.assertEqual(runtime["persisted"]["formatLocale"], "en-GB")
        self.assertNotEqual(
            runtime["persisted"]["translation"],
            "common.unknown",
        )

        self.assertEqual(
            runtime["blockedStorage"]["initial"],
            {
                "locale": "en",
                "formatLocale": "en-US",
            },
        )
        self.assertTrue(runtime["blockedStorage"]["setSurvived"])
        self.assertEqual(
            (
                runtime["blockedStorage"]["localeAfterSet"],
                runtime["blockedStorage"]["formatLocaleAfterSet"],
            ),
            ("sv", "sv-SE"),
        )

        switching = runtime["switching"]
        self.assertEqual(switching["supportedLocales"], ["sv", "en"])
        self.assertEqual(
            switching["switched"],
            {
                "locale": "en",
                "formatLocale": "en-US",
                "direction": "ltr",
            },
        )
        self.assertEqual(
            switching["notifications"],
            [{"locale": "en", "formatLocale": "en-US"}],
        )
        self.assertEqual(
            switching["storageWrites"],
            [["robot-dashboard-locale", "en"]],
        )
        self.assertEqual(
            (
                switching["finalLocale"],
                switching["finalFormatLocale"],
            ),
            ("sv", "sv-SE"),
        )
        self.assertTrue(switching["translationsChanged"])
        self.assertTrue(switching["numberChanged"])
        self.assertTrue(all(switching["formatterMatches"].values()))

    def test_language_switch_rerenders_state_without_network_or_reload(self):
        self.assertIn(
            "i18n.setLocale(event.currentTarget.value)",
            self.javascript,
        )
        self.assertIn(
            "i18n.subscribe(renderLocalizedState)",
            self.javascript,
        )
        render_start = self.javascript.index(
            "  function renderLocalizedState() {"
        )
        render_end = self.javascript.index(
            "\n  function bindInteractions()",
            render_start,
        )
        render_source = self.javascript[render_start:render_end]

        for expected in (
            "applyStaticTranslations()",
            "renderConversation()",
            "renderEvents()",
            "renderProbeResult()",
            "document.activeElement",
            "setSelectionRange(",
        ):
            with self.subTest(expected=expected):
                self.assertIn(expected, render_source)
        for forbidden in (
            "fetch(",
            "api(",
            "location.reload",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, render_source)
                self.assertNotIn(forbidden, self.i18n)

    def test_probe_result_is_stateful_and_relocalized_after_switch(self):
        probe_start = self.javascript.index(
            "  function renderProbeResult() {"
        )
        probe_end = self.javascript.index(
            "\n  async function refreshBootstrap(",
            probe_start,
        )
        probe_source = self.javascript[probe_start:probe_end]

        self.assertIn("state.lmProbe", self.javascript)
        for phase in ("idle", "checking", "completed", "failed"):
            with self.subTest(phase=phase):
                self.assertIn('"{}"'.format(phase), probe_source)
        self.assertGreaterEqual(
            probe_source.count("renderProbeResult()"),
            3,
        )
        self.assertIn(
            't("settings.runtime.probe_idle")',
            probe_source,
        )

    def test_evidence_copy_hides_internal_monotonic_deadline(self):
        evidence_start = self.javascript.index(
            "  function renderEvidence("
        )
        evidence_end = self.javascript.index(
            "\n  function renderTurnAnnouncement(",
            evidence_start,
        )
        evidence_source = self.javascript[evidence_start:evidence_end]
        values = self.i18n_contract["catalogs"]

        self.assertIn('t("chat.evidence.validity")', evidence_source)
        self.assertNotIn(
            "valid_until_monotonic_ms",
            evidence_source,
        )
        self.assertEqual(
            values["sv"]["values"]["chat.evidence.validity"],
            "Giltigheten verifieras av den lokala värden",
        )
        self.assertEqual(
            values["en"]["values"]["chat.evidence.validity"],
            "Freshness is verified by the local host",
        )
        self.assertIn(
            "leverantör, giltighet och hash",
            self.html,
        )

    def test_reviewed_english_copy_is_unambiguous(self):
        values = self.i18n_contract["catalogs"]["en"]["values"]
        self.assertEqual(
            values["bodies.safety.body"],
            (
                "Seeing a controller in the registry never automatically "
                "authorizes motion."
            ),
        )
        self.assertIn(
            "Robot episode settings live in the separate robot control.",
            values["settings.description"],
        )
        self.assertEqual(
            values["settings.safety.description"],
            "Status for the separate execution path.",
        )
        self.assertEqual(
            self.i18n_contract["copySamples"]["en"]["droppedEvents"],
            "Some log events are missing. Total events dropped: 7.",
        )
        self.assertEqual(
            self.i18n_contract["copySamples"]["sv"]["unknownKey"],
            "future.unknown.key",
        )
        self.assertEqual(
            self.i18n_contract["copySamples"]["en"]["unknownKey"],
            "future.unknown.key",
        )

    def test_every_declarative_translation_key_exists_in_both_catalogs(self):
        allowed_attributes = {
            "data-i18n",
            "data-i18n-alt",
            "data-i18n-aria-label",
            "data-i18n-placeholder",
            "data-i18n-prompt",
            "data-i18n-title",
        }
        target_attributes = {
            "data-i18n-alt": "alt",
            "data-i18n-aria-label": "aria-label",
            "data-i18n-placeholder": "placeholder",
            "data-i18n-prompt": "data-prompt",
            "data-i18n-title": "title",
        }
        catalog_keys = set(self.i18n_contract["catalogs"]["sv"]["keys"])
        localized = []
        invalid_attribute_names = set()
        missing_keys = set()

        for tag, attributes in self.parser.elements:
            for name, key in attributes.items():
                if not name.startswith("data-i18n"):
                    continue
                localized.append((tag, name, key))
                if name not in allowed_attributes:
                    invalid_attribute_names.add(name)
                self.assertIsInstance(key, str)
                self.assertTrue(key)
                if key not in catalog_keys:
                    missing_keys.add(key)
                if name in target_attributes:
                    self.assertIn(target_attributes[name], attributes)

        self.assertGreater(len(localized), 0)
        self.assertEqual(invalid_attribute_names, set())
        self.assertEqual(missing_keys, set())

        language_selector = next(
            attributes
            for tag, attributes in self.parser.elements
            if tag == "select" and attributes.get("id") == "ui-language"
        )
        self.assertEqual(
            language_selector.get("data-i18n-aria-label"),
            "locale.selector.aria_label",
        )
        self.assertEqual(
            {
                attributes.get("value")
                for tag, attributes in self.parser.elements
                if tag == "option"
                and attributes.get("data-i18n")
                in {"locale.swedish", "locale.english"}
            },
            {"sv", "en"},
        )

        static_calls = set(
            re.findall(
                r"""\b(?:i18n\.)?t\(\s*["']([a-z0-9_.-]+)["']""",
                self.javascript_assets,
            )
        )
        self.assertTrue(static_calls)
        self.assertEqual(static_calls - catalog_keys, set())

    def test_no_inline_handlers_external_urls_or_physical_controls(self):
        for tag, name, value in self.parser.attributes:
            with self.subTest(tag=tag, name=name):
                self.assertFalse(name.lower().startswith("on"))
                self.assertNotEqual(name.lower(), "style")
                if name.lower() in ("href", "src", "action"):
                    self.assertFalse(
                        (value or "").startswith(("http://", "https://", "//"))
                    )
        forbidden_control_ids = (
            "move-button",
            "drive-button",
            "motor-button",
            "ssh-button",
            "tts-button",
            "stop-button",
        )
        for control_id in forbidden_control_ids:
            self.assertNotIn('id="{}"'.format(control_id), self.html)

    def test_javascript_uses_dom_text_and_only_declared_api_shapes(self):
        forbidden = (
            "innerHTML",
            "outerHTML",
            "insertAdjacentHTML",
            "document.write",
            "eval(",
            "new Function",
        )
        for filename, source in (
            ("i18n.js", self.i18n),
            ("blast_map_semantics.js", self.blast_map_semantics),
            ("dashboard_logic.js", self.dashboard_logic),
            ("controller_panel.js", self.controller_panel),
            ("speech_input_logic.js", self.speech_input_logic),
            ("microphone_input.js", self.microphone_input),
            ("pcm_capture_worklet.js", self.pcm_capture_worklet),
            (
                "spatial_map_presenter.js",
                self.spatial_map_presenter,
            ),
            ("robot_mission_panel.js", self.robot_mission_panel),
            ("robot_control.js", self.robot_control),
            ("app.js", self.javascript),
        ):
            for token in forbidden:
                with self.subTest(filename=filename, token=token):
                    self.assertNotIn(token, source)
        self.assertIn("textContent", self.javascript_assets)
        self.assertIn("replaceChildren", self.javascript)
        self.assertIn("replaceChildren", self.spatial_map_presenter)
        self.assertNotIn("error.message", self.javascript)
        self.assertIn("ERROR_MESSAGE_KEYS", self.javascript)
        self.assertIn("localizedError(", self.javascript)
        self.assertIn(
            "/api/v1/events?after_sequence=",
            self.javascript,
        )
        self.assertIn(
            "/api/v1/runtime/lm-studio/probe",
            self.javascript,
        )
        self.assertIn("selectSpatialMapEndpoint", self.javascript)
        self.assertIn("/api/v1/map", self.dashboard_logic)
        for forbidden_route in (
            "/move",
            "/drive",
            "/motor",
            "/ssh",
            "/tts",
            "/stop",
        ):
            self.assertNotIn(forbidden_route, self.javascript)

    def test_ui_does_not_classify_natural_language_with_regex(self):
        for source in (self.javascript, self.robot_mission_panel):
            self.assertNotIn("RegExp(", source)
            self.assertNotIn(".match(", source)
            self.assertNotIn(".test(", source)
        self.assertIn(
            'mode: byId("turn-mode").value',
            self.javascript,
        )
        self.assertIn("response_locale: i18n.locale", self.javascript)
        self.assertNotIn('"sv-SE"', self.javascript)
        self.assertIn("i18n.time(", self.javascript)
        self.assertIn("i18n.dateTime(", self.javascript)

    def test_robot_control_module_has_typed_target_and_state_policy(self):
        script = r"""
const fs = require("fs");
const vm = require("vm");
const source = fs.readFileSync(process.argv[1], "utf8");
const window = {};
vm.runInNewContext(source, { window }, { filename: process.argv[1] });
const api = window.RobotControlUI;
const targetState = api.createConversationTargetState("workbench");
const targetTransitions = {
  workbenchDefault: targetState.selected(),
};
targetTransitions.workbenchOverride = targetState.override("blast-01");
targetTransitions.robotDefault = targetState.selectView("robot");
targetTransitions.robotOverride = targetState.override("workbench");
targetTransitions.workbenchRestored = targetState.selectView("workbench");
targetTransitions.workbenchCleared = targetState.clearOverride();
targetTransitions.robotRestored = targetState.selectView("robot");
targetTransitions.robotCleared = targetState.clearOverride();
let invalidTargetRejected = false;
try {
  targetState.override("anything-else");
} catch (error) {
  invalidTargetRejected = error && error.name === "TypeError";
}
const idle = api.normalizeControl({
  state: "IDLE",
  enabled: true,
  accepting: true,
  target: {
    robot_id: "blast-01",
    display_name: "BLAST",
  },
  settings: {
    revision: 4,
    model: "gemma-4bit",
    max_episode_ms: 120000,
    speech_enabled: true,
  },
  runtime: {
    plan: ["scan", "advance"],
    planner_context_bytes: 88000,
    prompt_tokens: 21000,
    completion_tokens: 120,
    total_tokens: 21120,
    speech_status: "idle",
  },
});
const disabled = api.normalizeControl({ state: "unexpected" });
const disabledWithTarget = api.normalizeControl({
  state: "DISABLED",
  enabled: false,
  accepting: true,
  target: {
    robot_id: "blast-01",
    display_name: "BLAST",
  },
});
const idleWithoutTarget = api.normalizeControl({
  state: "IDLE",
  enabled: true,
  accepting: true,
});
const faulted = api.normalizeControl({
  state: "FAULTED",
  enabled: true,
  accepting: true,
  target: {
    robot_id: "blast-01",
    display_name: "BLAST",
  },
});
const running = api.normalizeControl({
  state: "RUNNING",
  enabled: true,
  accepting: true,
  target: {
    robot_id: "blast-01",
    display_name: "BLAST",
  },
});
const speechFailed = api.normalizeControl({
  state: "RUNNING",
  enabled: true,
  accepting: true,
  target: {
    robot_id: "blast-01",
    display_name: "BLAST",
  },
  runtime: {
    speech_status: "failed",
    speech_error_code: "tts_audio_too_long",
  },
});
const stopping = api.normalizeControl({
  state: "STOPPING",
  enabled: true,
  accepting: true,
  target: {
    robot_id: "blast-01",
    display_name: "BLAST",
  },
});
process.stdout.write(JSON.stringify({
  exports: Object.keys(api).sort(),
  frozen: Object.isFrozen(api),
  states: Array.from(api.CONTROL_STATES),
  targets: Array.from(api.CONVERSATION_TARGETS),
  blastEndpoint: api.robotEndpoint("blast-01", "turns"),
  directory: api.normalizeRobotDirectory({ controls: [idle] }),
  idle,
  disabled,
  preferredLiveTarget: api.preferredInitialTarget(idle, false),
  preferredDisabledTarget: api.preferredInitialTarget(disabled, false),
  preferredDisabledNamedTarget: api.preferredInitialTarget(
    disabledWithTarget,
    false,
  ),
  preferredMissingTarget: api.preferredInitialTarget(idleWithoutTarget, false),
  preferredFaultedTarget: api.preferredInitialTarget(faulted, false),
  preferredAfterUserChoice: api.preferredInitialTarget(idle, true),
  robotPolicy: api.composerPolicy(idle, "blast-01", true, false),
  runningRobotPolicy: api.composerPolicy(running, "blast-01", true, false),
  speechFailed,
  stoppingRobotPolicy: api.composerPolicy(stopping, "blast-01", true, false),
  faultedRobotPolicy: api.composerPolicy(faulted, "blast-01", true, false),
  busyRobotPolicy: api.composerPolicy(running, "blast-01", true, true),
  physicalTurnControl: api.physicalTurnControl({
    intent: "PHYSICAL_TASK",
    episode: { control: { sequence: 9, state: "STARTING" } },
  }),
  nonPhysicalTurnControl: api.physicalTurnControl({
    intent: "READ_ONLY_TASK",
    episode: { control: { sequence: 99, state: "RUNNING" } },
  }),
  stopTurnControl: api.physicalTurnControl({
    intent: "STOP_TASK",
    control: { sequence: 10, state: "STOPPING" },
  }),
  missingEpisodeControl: api.physicalTurnControl({
    intent: "PHYSICAL_TASK",
    episode: null,
  }),
  workbenchPolicy: api.composerPolicy(idle, "workbench", true, false),
  workbenchWithoutRobot: api.composerPolicy(
    disabled,
    "workbench",
    true,
    false,
  ),
  unavailableRobotPolicy: api.composerPolicy(
    idleWithoutTarget,
    "blast-01",
    true,
    false,
  ),
  lowerSequenceAccepted: api.shouldApplySnapshot(
    { sequence: 8 },
    { sequence: 7 },
  ),
  equalSequenceAccepted: api.shouldApplySnapshot(
    { sequence: 8 },
    { sequence: 8 },
  ),
  higherSequenceAccepted: api.shouldApplySnapshot(
    { sequence: 8 },
    { sequence: 9 },
  ),
  invalidTargetRejected,
  targetTransitions,
}));
"""
        completed = subprocess.run(
            [
                "node",
                "--input-type=commonjs",
                "-e",
                script,
                str(WEB_ROOT / "robot_control.js"),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
        )
        if completed.returncode != 0:
            raise AssertionError(completed.stderr)
        contract = json.loads(completed.stdout)
        self.assertTrue(contract["frozen"])
        self.assertEqual(
            contract["exports"],
            [
                "ACTIVE_STATES",
                "CONTROL_STATES",
                "CONVERSATION_TARGETS",
                "CONVERSATION_VIEWS",
                "DEFAULT_ROBOT_TARGET_ID",
                "ROBOT_TARGETS",
                "composerPolicy",
                "create",
                "createConversationTargetState",
                "defaultConversationTarget",
                "isRobotTarget",
                "normalizeControl",
                "normalizeRobotDirectory",
                "physicalTurnControl",
                "preferredInitialTarget",
                "robotEndpoint",
                "shouldApplySnapshot",
            ],
        )
        self.assertEqual(
            contract["targetTransitions"],
            {
                "workbenchDefault": "workbench",
                "workbenchOverride": "blast-01",
                "robotDefault": "ev3rstorm-01",
                "robotOverride": "workbench",
                "workbenchRestored": "blast-01",
                "workbenchCleared": "workbench",
                "robotRestored": "workbench",
                "robotCleared": "ev3rstorm-01",
            },
        )
        self.assertTrue(contract["invalidTargetRejected"])
        self.assertEqual(contract["preferredLiveTarget"], "blast-01")
        self.assertEqual(contract["preferredDisabledTarget"], "workbench")
        self.assertEqual(
            contract["preferredDisabledNamedTarget"],
            "workbench",
        )
        self.assertEqual(contract["preferredMissingTarget"], "workbench")
        self.assertEqual(contract["preferredFaultedTarget"], "workbench")
        self.assertEqual(contract["preferredAfterUserChoice"], "workbench")
        self.assertEqual(
            contract["states"],
            [
                "DISABLED",
                "IDLE",
                "STARTING",
                "RUNNING",
                "STOPPING",
                "FAULTED",
            ],
        )
        self.assertEqual(
            contract["targets"],
            ["ev3rstorm-01", "blast-01", "workbench"],
        )
        self.assertEqual(
            contract["blastEndpoint"],
            "/api/v1/robots/blast-01/turns",
        )
        self.assertEqual(contract["directory"]["blast-01"]["state"], "IDLE")
        self.assertEqual(
            contract["directory"]["ev3rstorm-01"]["target"],
            {"robot_id": "ev3rstorm-01", "display_name": "EV3RSTORM"},
        )
        self.assertTrue(
            contract["robotPolicy"]["composerEnabled"]
        )
        self.assertTrue(contract["robotPolicy"]["robotInputEnabled"])
        self.assertTrue(contract["runningRobotPolicy"]["composerEnabled"])
        self.assertTrue(contract["stoppingRobotPolicy"]["composerEnabled"])
        self.assertTrue(contract["faultedRobotPolicy"]["composerEnabled"])
        self.assertFalse(contract["busyRobotPolicy"]["composerEnabled"])
        self.assertEqual(
            contract["physicalTurnControl"],
            {"sequence": 9, "state": "STARTING"},
        )
        self.assertIsNone(contract["nonPhysicalTurnControl"])
        self.assertEqual(
            contract["stopTurnControl"],
            {"sequence": 10, "state": "STOPPING"},
        )
        self.assertIsNone(contract["missingEpisodeControl"])
        self.assertFalse(
            contract["robotPolicy"]["turnModeEnabled"]
        )
        self.assertTrue(
            contract["workbenchPolicy"]["turnModeEnabled"]
        )
        self.assertTrue(
            contract["workbenchWithoutRobot"]["composerEnabled"]
        )
        self.assertFalse(
            contract["unavailableRobotPolicy"]["composerEnabled"]
        )
        self.assertEqual(
            contract["idle"]["target"],
            {"robot_id": "blast-01", "display_name": "BLAST"},
        )
        self.assertIsNone(contract["disabled"]["target"])
        self.assertFalse(contract["lowerSequenceAccepted"])
        self.assertTrue(contract["equalSequenceAccepted"])
        self.assertTrue(contract["higherSequenceAccepted"])
        self.assertEqual(
            contract["idle"]["runtime"]["planner_context_bytes"],
            88_000,
        )
        self.assertEqual(
            contract["idle"]["runtime"]["prompt_tokens"],
            21_000,
        )
        self.assertEqual(
            contract["idle"]["runtime"]["completion_tokens"],
            120,
        )
        self.assertEqual(
            contract["idle"]["runtime"]["total_tokens"],
            21_120,
        )
        self.assertEqual(
            contract["speechFailed"]["runtime"]["speech_error_code"],
            "tts_audio_too_long",
        )
        self.assertIn(
            "`${speechLabel} · ${runtime.speech_error_code}`",
            self.robot_control,
        )
        self.assertIn(
            "if (!shouldApplySnapshot(previous, next))",
            self.robot_control,
        )
        self.assertNotIn(
            'byId("status-ev3")',
            self.robot_control,
        )
        self.assertIn(
            'const motionStatus = byId("status-motion");',
            self.robot_control,
        )
        self.assertEqual(
            contract["disabled"]["state"],
            "DISABLED",
        )
        self.assertIn(
            'robotEndpoint(target, operation)',
            self.robot_control,
        )
        self.assertNotIn("/api/v1/robot/motor", self.robot_control)
        self.assertIn(
            'translate("workbench.target.named_robot"',
            self.robot_control,
        )
        self.assertIn("option.disabled = false", self.robot_control)
        for operation in (
            'robotEndpoint(target, "status")',
            'robotEndpoint(target, "turns")',
            'robotEndpoint(target, "settings")',
            'command(\n          "stop"',
            'command(\n          "emergency-stop"',
        ):
            self.assertIn(operation, self.robot_control)
        self.assertEqual(
            self.i18n_contract["copySamples"]["sv"]["namedRobot"],
            "BLAST · robot",
        )
        self.assertEqual(
            self.i18n_contract["copySamples"]["en"]["namedRobot"],
            "BLAST · robot",
        )

    def test_robot_composer_uses_composite_turns_and_keeps_dialogue_separate(self):
        submit_input = self.robot_control[
            self.robot_control.index("    async function submitInput("):
            self.robot_control.index(
                "\n    async function command(",
                self.robot_control.index("    async function submitInput("),
            )
        ]
        self.assertIn('request(robotEndpoint(target, "turns")', submit_input)
        self.assertIn("text: cleanText", submit_input)
        self.assertIn("locale,", submit_input)
        self.assertIn('client_request_id: randomId("robot-ui")', submit_input)
        self.assertIn(
            "expected_revision: targetControl.settings.revision",
            submit_input,
        )
        self.assertIn("timeout: 65000", submit_input)
        self.assertNotIn("goal:", submit_input)
        self.assertNotIn("/api/v1/robot/episodes", self.robot_control)
        self.assertIn("physicalTurnControl(turn)", submit_input)
        self.assertIn("setControlFor(target, nextControl)", submit_input)
        self.assertIn("onInputAccepted(cleanText, turn, target)", submit_input)

        self.assertIn("MAX_ROBOT_DIALOGUE_MESSAGES = 40", self.javascript)
        self.assertIn("robotDialogueByTarget: {}", self.javascript)
        self.assertIn("robotOptimisticByTarget: {}", self.javascript)
        self.assertIn("robotControl.submitInput(", self.javascript)
        self.assertIn('author_key: "robot"', self.javascript)
        self.assertIn(
            't(`chat.author.${safeText(message.author_key, role)}`)',
            self.javascript,
        )
        self.assertIn(
            "onInputAccepted: (originalText, turn, target) =>",
            self.javascript,
        )
        self.assertIn("onTargetChanged: (target) =>", self.javascript)
        self.assertIn(".slice(-MAX_ROBOT_DIALOGUE_MESSAGES)", self.javascript)
        self.assertIn(
            "const robotTarget = robotControl?.isRobotTarget(target);",
            self.javascript,
        )

        catalogs = self.i18n_contract["catalogs"]
        self.assertEqual(
            catalogs["sv"]["values"]["robot.actions.start"],
            "Skicka till roboten",
        )
        self.assertEqual(
            catalogs["en"]["values"]["robot.actions.start"],
            "Send to robot",
        )
        self.assertEqual(
            catalogs["sv"]["values"]["chat.author.robot"],
            "Roboten",
        )
        self.assertEqual(
            catalogs["en"]["values"]["chat.author.robot"],
            "Robot",
        )
        self.assertIn(
            "under körning",
            catalogs["sv"]["values"]["robot.composer.robot_note"],
        )
        self.assertIn(
            "while moving",
            catalogs["en"]["values"]["robot.composer.robot_note"],
        )

    def test_spatial_map_follows_selected_robot_without_cross_target_stale_data(self):
        refresh = self.javascript[
            self.javascript.index("  async function refreshSpatialMap("):
            self.javascript.index(
                "\n  function renderLocalizedState()",
                self.javascript.index("  async function refreshSpatialMap("),
            )
        ]
        self.assertIn(
            "`/api/v1/robots/${encodeURIComponent(target)}/spatial-map`",
            refresh,
        )
        self.assertIn("const generation = ++state.mapRequestGeneration", refresh)
        self.assertIn("target !== selectedConversationTarget()", refresh)
        self.assertIn(
            'renderSpatialMap(payload.map, "connected", target)',
            refresh,
        )
        self.assertIn(
            'renderSpatialMap(state.spatialMapByTarget[target], "offline", target)',
            refresh,
        )
        target_change = self.javascript[
            self.javascript.index("      onTargetChanged: (target) => {"):
            self.javascript.index(
                "\n      },",
                self.javascript.index("      onTargetChanged: (target) => {")
            )
        ]
        self.assertLess(
            target_change.index('byId("message-input").value = ""'),
            target_change.index("refreshSpatialMap(true, target)"),
        )
        self.assertIn(
            'renderSpatialMap(state.spatialMapByTarget[target], "waiting", target)',
            target_change,
        )

    def test_robot_selector_keeps_both_named_targets_visible_when_offline(self):
        script = r"""
const fs = require("fs");
const vm = require("vm");
const source = fs.readFileSync(process.argv[1], "utf8");

class Element {
  constructor(id) {
    this.id = id;
    this.value = "";
    this.textContent = "";
    this.disabled = false;
    this.hidden = false;
    this.checked = false;
    this.dataset = {};
    this.listeners = {};
    this.className = "";
    this.classList = { add() {} };
  }
  addEventListener(name, listener) {
    this.listeners[name] = listener;
  }
  querySelector() {
    return null;
  }
}

const ids = [
  "composer-target", "message-input", "send-button",
  "new-conversation-button", "turn-mode", "turn-mode-control",
  "mode-capability-note", "composer-status", "robot-control-state",
  "robot-control-summary", "robot-stop-button",
  "robot-emergency-stop-button", "robot-current-action",
  "robot-obstacle", "robot-plan", "robot-scan", "robot-model-latency",
  "robot-speech-status", "robot-setting-model",
  "robot-setting-max-episode-ms", "robot-setting-speech-enabled",
  "robot-save-settings-button", "robot-settings-status",
  "robot-settings-form", "status-motion",
];
const nodes = Object.fromEntries(ids.map((id) => [id, new Element(id)]));
const robotOptions = {
  "ev3rstorm-01": new Element("ev3-option"),
  "blast-01": new Element("blast-option"),
};
const sendLabel = new Element("send-label");
const motionValue = new Element("motion-value");
nodes["composer-target"].querySelector = (selector) => (
  robotOptions[selector.match(/option\[value="([^"]+)"\]/)?.[1]] || null
);
nodes["send-button"].querySelector = () => sendLabel;
nodes["status-motion"].querySelector = () => motionValue;

const named = {
  schema: "robot-control/v1",
  sequence: 2,
  state: "IDLE",
  enabled: true,
  accepting: true,
  target: { robot_id: "blast-01", display_name: "BLAST" },
  settings: {
    revision: 1,
    model: "gemma",
    max_episode_ms: 900000,
    speech_enabled: true,
  },
  runtime: { speech_status: "idle", plan: [] },
};
const unavailable = {
  ...named,
  sequence: 3,
  state: "DISABLED",
  enabled: false,
  target: { robot_id: "blast-01", display_name: "BLAST" },
};
const ev3Unavailable = {
  ...unavailable,
  sequence: 1,
  target: { robot_id: "ev3rstorm-01", display_name: "EV3RSTORM" },
};
const requests = [];
let locale = "sv";
const targetChanges = [];
const window = {
  RobotMissionPanelUI: {
    create: () => ({
      initialize: async () => {},
      renderLocale() {},
      setControl() {},
      setRobotId() {},
      stopPolling() {},
    }),
  },
  setTimeout: () => 1,
  clearTimeout() {},
};
vm.runInNewContext(source, { window }, { filename: process.argv[1] });
const ui = window.RobotControlUI.create({
  document: { getElementById: (id) => nodes[id] },
  request: async (path, options = {}) => {
    requests.push({ path, method: options.method || "GET" });
    if (path === "/api/v1/robots") {
      return { schema: "robot-control-directory/v1", controls: [
        ev3Unavailable, named,
      ] };
    }
    if (path === "/api/v1/robots/blast-01/status") {
      return { control: unavailable };
    }
    throw new Error("unexpected request");
  },
  translate: (key, args = {}) => {
    if (key === "workbench.target.named_robot") {
      return `${args.name} · robot-${locale}`;
    }
    return key;
  },
  randomId: () => "request-1",
  showToast() {},
  getLocale: () => locale,
  sessionGuard: {
    subscribe() {},
    isExpired: () => false,
  },
  onTargetChanged: (target) => targetChanges.push(target),
});

(async () => {
  await ui.initialize();
  const ready = {
    selected: ui.selectedTarget(),
    value: nodes["composer-target"].value,
    ev3Disabled: robotOptions["ev3rstorm-01"].disabled,
    blastDisabled: robotOptions["blast-01"].disabled,
    blastLabel: robotOptions["blast-01"].textContent,
  };
  locale = "en";
  ui.renderLocale();
  const localizedLabel = robotOptions["blast-01"].textContent;
  await ui.refresh(false);
  const unavailableView = {
    selected: ui.selectedTarget(),
    value: nodes["composer-target"].value,
    ev3Disabled: robotOptions["ev3rstorm-01"].disabled,
    blastDisabled: robotOptions["blast-01"].disabled,
    blastLabel: robotOptions["blast-01"].textContent,
  };
  nodes["message-input"].value = "Move";
  const accepted = await ui.submitInput("Move", "en");
  process.stdout.write(JSON.stringify({
    ready,
    localizedLabel,
    unavailableView,
    accepted,
    postRequests: requests.filter((request) => request.method === "POST"),
    targetChanges,
  }));
})();
"""
        completed = subprocess.run(
            [
                "node",
                "--input-type=commonjs",
                "-e",
                script,
                str(WEB_ROOT / "robot_control.js"),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
        )
        if completed.returncode != 0:
            raise AssertionError(completed.stderr)
        result = json.loads(completed.stdout)
        self.assertEqual(
            result["ready"],
            {
                "selected": "blast-01",
                "value": "blast-01",
                "ev3Disabled": False,
                "blastDisabled": False,
                "blastLabel": "BLAST · robot-sv",
            },
        )
        self.assertEqual(result["localizedLabel"], "BLAST · robot-en")
        self.assertEqual(
            result["unavailableView"],
            {
                "selected": "blast-01",
                "value": "blast-01",
                "ev3Disabled": False,
                "blastDisabled": False,
                "blastLabel": "BLAST · robot-en",
            },
        )
        self.assertFalse(result["accepted"])
        self.assertEqual(result["postRequests"], [])
        self.assertEqual(result["targetChanges"][-1], "blast-01")

    def test_robot_mission_history_is_cursor_safe_and_semantically_deduplicated(self):
        script = r"""
const fs = require("fs");
const vm = require("vm");
const source = fs.readFileSync(process.argv[1], "utf8");
const context = {};
vm.runInNewContext(source, context, { filename: process.argv[1] });
const api = context.RobotMissionPanelUI;

function snapshot(sequence, episodeId, updatedAt, runtime, state = "RUNNING") {
  return {
    schema: "robot-control/v1",
    sequence,
    state,
    enabled: true,
    accepting: true,
    episode: {
      episode_id: episodeId,
      goal: episodeId === "episode-1" ? "Explore the room" : "Other goal",
      started_at_unix_ms: 900,
      terminal_reason: null,
    },
    updated_at_unix_ms: updatedAt,
    last_error_code: null,
    runtime: {
      current_action: null,
      obstacle: null,
      plan: [],
      scan: null,
      model_latency_ms: null,
      speech_status: "idle",
      message: null,
      ...runtime,
    },
  };
}

function activeRoute(version, activeIndex) {
  return {
    route_id: "local-detour-1",
    version,
    status: "ACTIVE",
    active_index: activeIndex,
    detour_side: "LEFT_OF_GOAL",
    waypoints: [
      {
        ordinal: 0,
        kind: "LATERAL_CLEARANCE",
        x_mm: 150,
        y_mm: -220,
        heading_mdeg: -90000,
      },
      {
        ordinal: 1,
        kind: "REACQUIRE_GOAL_HEADING",
        x_mm: 300,
        y_mm: -220,
        heading_mdeg: 0,
      },
    ],
  };
}

const snapshots = [
  snapshot(1, "episode-1", 1000, {}, "STARTING"),
  snapshot(2, "episode-1", 1100, {
    current_action: "advance",
    obstacle: { relation: "CLEAR" },
    plan: ["advance", "scan"],
    active_route: activeRoute(1, 0),
    model_latency_ms: 42,
    speech_status: "generating",
    message: "Moving now",
  }),
  snapshot(3, "episode-1", 1200, {
    current_action: "advance",
    obstacle: { relation: "CLEAR" },
    plan: ["advance", "scan"],
    active_route: activeRoute(1, 0),
    model_latency_ms: 99,
    speech_status: "generating",
    message: "Moving now",
  }),
  snapshot(4, "episode-1", 1300, {
    current_action: "scan",
    obstacle: { relation: "CLEAR" },
    plan: ["scan", "turn"],
    active_route: activeRoute(2, 1),
    scan: { state: "pending" },
    model_latency_ms: 99,
    speech_status: "playing",
    message: "Moving now",
  }),
  snapshot(5, "episode-2", 1400, {
    current_action: "other",
    plan: ["other"],
  }),
];
const eventPage = {
  schema: "robot-control-event-page/v1",
  after_sequence: 0,
  oldest_sequence: 1,
  newest_sequence: 4,
  next_after_sequence: 4,
  gap: true,
  dropped_total: 1,
  events: [
    {
      sequence: 1,
      occurred_at_unix_ms: 1000,
      event_type: "robot.episode_starting",
      message: "starting",
      episode_id: "episode-1",
      state: "STARTING",
      level: "info",
      data: {},
    },
    {
      sequence: 2,
      occurred_at_unix_ms: 1100,
      event_type: "robot.runtime_update",
      message: "runtime",
      episode_id: "episode-1",
      state: "RUNNING",
      level: "debug",
      data: {},
    },
    {
      sequence: 3,
      occurred_at_unix_ms: 1100,
      event_type: "robot.episode_running",
      message: "running",
      episode_id: "episode-1",
      state: "RUNNING",
      level: "info",
      data: {},
    },
    {
      sequence: 4,
      occurred_at_unix_ms: 1400,
      event_type: "robot.episode_starting",
      message: "other",
      episode_id: "episode-2",
      state: "STARTING",
      level: "info",
      data: {},
    },
  ],
};
const snapshotPage = {
  schema: "robot-control-snapshot-page/v1",
  after_sequence: 0,
  oldest_sequence: 1,
  newest_sequence: 5,
  next_after_sequence: 5,
  gap: false,
  dropped_total: 0,
  snapshots,
};

const store = api.createHistoryStore(10);
store.ingestEvents(eventPage);
store.ingestSnapshots(snapshotPage);
store.ingestEvents(eventPage);
store.ingestSnapshots(snapshotPage);
const current = api.normalizeSnapshot(snapshots[3]);
const normalizedAgain = api.normalizeRoute(current.activeRoute);
const timeline = api.buildTimeline(store.values(), current);
const limited = api.buildTimeline(store.values(), current, 2);
const liveFirst = api.createHistoryStore(10);
liveFirst.addSnapshot(snapshots[3]);
const liveCursorBeforeHistory = liveFirst.cursors();
liveFirst.ingestSnapshots(snapshotPage);
const resetStore = api.createHistoryStore(10);
resetStore.ingestEvents(eventPage);
const resetAccepted = resetStore.ingestEvents({
  ...eventPage,
  newest_sequence: 1,
  next_after_sequence: 4,
  events: [],
});
const catchingUp = api.createHistoryStore(10);
catchingUp.ingestEvents({
  ...eventPage,
  newest_sequence: 1000,
  next_after_sequence: 500,
  events: [],
});
catchingUp.ingestSnapshots({
  ...snapshotPage,
  newest_sequence: 1000,
  next_after_sequence: 500,
  snapshots: [],
});
const catchUpBefore = {
  caughtUp: catchingUp.caughtUp(),
  progress: catchingUp.progress(),
};
catchingUp.ingestEvents({
  ...eventPage,
  after_sequence: 500,
  newest_sequence: 1000,
  next_after_sequence: 1000,
  events: [],
});
catchingUp.ingestSnapshots({
  ...snapshotPage,
  after_sequence: 500,
  newest_sequence: 1000,
  next_after_sequence: 1000,
  snapshots: [],
});

process.stdout.write(JSON.stringify({
  exports: Object.keys(api).sort(),
  frozen: Object.isFrozen(api),
  cursors: store.cursors(),
  gaps: store.gaps(),
  counts: {
    events: store.values().events.length,
    snapshots: store.values().snapshots.length,
  },
  route: {
    routeId: current.activeRoute.routeId,
    version: current.activeRoute.version,
    activeIndex: current.activeRoute.activeIndex,
    detourSide: current.activeRoute.detourSide,
    waypoint: current.activeRoute.waypoints[0],
    frozen: Object.isFrozen(current.activeRoute),
    waypointsFrozen: Object.isFrozen(current.activeRoute.waypoints),
    waypointFrozen: Object.isFrozen(current.activeRoute.waypoints[0]),
    normalizedAgain: {
      routeId: normalizedAgain.routeId,
      activeIndex: normalizedAgain.activeIndex,
      xMm: normalizedAgain.waypoints[0].xMm,
      yMm: normalizedAgain.waypoints[0].yMm,
      headingMdeg: normalizedAgain.waypoints[0].headingMdeg,
    },
  },
  timeline: {
    episodeId: timeline.episodeId,
    count: timeline.entries.length,
    eventTypes: timeline.entries
      .filter((entry) => entry.source === "event")
      .map((entry) => entry.eventType),
    snapshotSequences: timeline.entries
      .filter((entry) => entry.source === "snapshot")
      .map((entry) => entry.sequence),
    snapshotChanges: timeline.entries
      .filter((entry) => entry.source === "snapshot")
      .map((entry) => entry.changes.map((change) => change.kind)),
  },
  limitedCount: limited.entries.length,
  limitedTotal: limited.totalEntries,
  limitedTruncated: limited.truncated,
  liveCursorBeforeHistory,
  liveCountAfterHistory: liveFirst.values().snapshots.length,
  resetAccepted,
  resetCursor: resetStore.cursors().event,
  catchUpBefore,
  catchUpAfter: catchingUp.caughtUp(),
  catchUpPages: api.MAX_CATCH_UP_PAGES,
  catchUpPollMs: api.POLL_CATCH_UP_MS,
}));
"""
        completed = subprocess.run(
            [
                "node",
                "--input-type=commonjs",
                "-e",
                script,
                str(WEB_ROOT / "robot_mission_panel.js"),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
        )
        if completed.returncode != 0:
            raise AssertionError(completed.stderr)
        result = json.loads(completed.stdout)
        self.assertTrue(result["frozen"])
        self.assertEqual(result["cursors"], {"event": 4, "snapshot": 5})
        self.assertEqual(result["gaps"], {"event": True, "snapshot": False})
        self.assertEqual(result["counts"], {"events": 3, "snapshots": 5})
        self.assertEqual(
            result["route"],
            {
                "routeId": "local-detour-1",
                "version": 2,
                "activeIndex": 1,
                "detourSide": "LEFT_OF_GOAL",
                "waypoint": {
                    "ordinal": 0,
                    "kind": "LATERAL_CLEARANCE",
                    "xMm": 150,
                    "yMm": -220,
                    "headingMdeg": -90_000,
                },
                "frozen": True,
                "waypointsFrozen": True,
                "waypointFrozen": True,
                "normalizedAgain": {
                    "routeId": "local-detour-1",
                    "activeIndex": 1,
                    "xMm": 150,
                    "yMm": -220,
                    "headingMdeg": -90_000,
                },
            },
        )
        self.assertEqual(result["timeline"]["episodeId"], "episode-1")
        self.assertEqual(result["timeline"]["count"], 4)
        self.assertEqual(
            set(result["timeline"]["eventTypes"]),
            {"robot.episode_starting", "robot.episode_running"},
        )
        self.assertNotIn(
            "robot.runtime_update",
            result["timeline"]["eventTypes"],
        )
        self.assertEqual(
            result["timeline"]["snapshotSequences"],
            [4, 2],
        )
        self.assertEqual(
            result["timeline"]["snapshotChanges"],
            [
                ["action", "plan", "route", "scan", "speech"],
                [
                    "action",
                    "plan",
                    "route",
                    "obstacle",
                    "speech",
                    "message",
                ],
            ],
        )
        self.assertEqual(result["limitedCount"], 2)
        self.assertEqual(result["limitedTotal"], 4)
        self.assertTrue(result["limitedTruncated"])
        self.assertEqual(
            result["liveCursorBeforeHistory"],
            {"event": 0, "snapshot": 0},
        )
        self.assertEqual(result["liveCountAfterHistory"], 5)
        self.assertFalse(result["resetAccepted"])
        self.assertEqual(result["resetCursor"], 0)
        self.assertFalse(result["catchUpBefore"]["caughtUp"])
        self.assertEqual(
            result["catchUpBefore"]["progress"],
            {
                "event": {"cursor": 500, "newest": 1_000},
                "snapshot": {"cursor": 500, "newest": 1_000},
            },
        )
        self.assertTrue(result["catchUpAfter"])
        self.assertEqual(result["catchUpPages"], 4)
        self.assertEqual(result["catchUpPollMs"], 0)
        self.assertIn('connection = "catching_up";', self.robot_mission_panel)
        self.assertIn(
            "${endpoint}/events?after_sequence=",
            self.robot_mission_panel,
        )
        self.assertIn(
            "${endpoint}/snapshots?after_sequence=",
            self.robot_mission_panel,
        )
        self.assertNotIn("innerHTML", self.robot_mission_panel)

    def test_robot_mission_panel_renders_typed_active_route(self):
        script = r"""
const fs = require("fs");
const vm = require("vm");
const source = fs.readFileSync(process.argv[1], "utf8");
const context = {};
vm.runInNewContext(source, context, { filename: process.argv[1] });
const api = context.RobotMissionPanelUI;

class FakeNode {
  constructor(tagName = "div") {
    this.tagName = tagName;
    this.textContent = "";
    this.className = "";
    this.children = [];
    this.hidden = false;
    this.dateTime = "";
    this.attributes = {};
    this.classList = {
      add: (...names) => {
        const classes = new Set(this.className.split(/\s+/).filter(Boolean));
        names.forEach((name) => classes.add(name));
        this.className = Array.from(classes).join(" ");
      },
    };
  }

  appendChild(node) {
    this.children.push(node);
    return node;
  }

  replaceChildren(...nodes) {
    this.children = nodes;
  }

  setAttribute(name, value) {
    this.attributes[name] = String(value);
  }
}

const requiredIds = [
  "map-mission-state",
  "map-mission-mode",
  "map-mission-action-label",
  "map-mission-plan-label",
  "map-mission-plan-heading",
  "map-mission-route-label",
  "map-mission-route-heading",
  "map-mission-goal",
  "map-mission-action",
  "map-mission-message",
  "map-mission-speech",
  "map-mission-updated",
  "map-mission-live-announcement",
  "map-mission-plan",
  "map-mission-plan-summary",
  "map-mission-route",
  "map-mission-route-summary",
  "map-mission-timeline",
  "map-mission-history-count",
  "map-mission-history-gap",
  "map-mission-history-status",
];
const nodes = Object.fromEntries(
  requiredIds.map((id) => [id, new FakeNode()]),
);
const documentApi = {
  getElementById(id) {
    if (!Object.hasOwn(nodes, id)) {
      throw new Error(`Unexpected DOM id: ${id}`);
    }
    return nodes[id];
  },
  createElement(tagName) {
    return new FakeNode(tagName);
  },
};
const translations = {
  "common.missing": "missing",
  "common.none": "none",
  "mission.action.label": "Doing now",
  "mission.action.latest_label": "Latest action",
  "mission.goal.empty": "No goal",
  "mission.history.live": "live",
  "mission.history.waiting": "waiting",
  "mission.message.idle": "Mission stopped",
  "mission.message.waiting": "Waiting",
  "mission.mode.active": "Active mission",
  "mission.mode.latest": "Latest mission",
  "mission.mode.none": "No mission",
  "mission.plan.empty": "No plan",
  "mission.plan.label": "Current plan",
  "mission.plan.latest_label": "Latest plan",
  "mission.plan.latest_title": "Latest published plan",
  "mission.plan.title": "Plan right now",
  "mission.route.empty": "No route",
  "mission.route.label": "Active route",
  "mission.route.latest_label": "Latest route",
  "mission.route.latest_title": "Latest published detour route",
  "mission.route.side.LEFT_OF_GOAL": "left of the goal heading",
  "mission.route.status.ACTIVE": "active",
  "mission.route.title": "Active detour route",
  "mission.route.waypoint.LATERAL_CLEARANCE": "Create lateral clearance",
  "mission.route.waypoint.MERGE_GOAL_AXIS": "Merge onto the goal axis",
  "mission.route.waypoint.PASS_BEYOND_TARGET": "Pass beyond the obstacle",
  "mission.route.waypoint.REACQUIRE_GOAL_HEADING": "Reacquire the goal heading",
  "mission.route.waypoint.RESUME_GOAL_HEADING": "Resume the goal heading",
  "mission.timeline.empty": "No history",
  "mission.timeline.route": "Route updated",
  "robot.speech.idle": "idle",
  "robot.state.IDLE": "Idle",
  "robot.state.RUNNING": "Running",
};
function translate(key, args) {
  if (key === "mission.live_announcement") {
    return `${args.state}: ${args.action}. ${args.message}`;
  }
  return Object.hasOwn(translations, key) ? translations[key] : key;
}

function descendantTexts(node) {
  return [
    node.textContent,
    ...node.children.flatMap((child) => descendantTexts(child)),
  ];
}

const activeRoute = {
  route_id: "local-detour-7",
  version: 3,
  status: "ACTIVE",
  active_index: 1,
  detour_side: "LEFT_OF_GOAL",
  waypoints: [
    [0, "LATERAL_CLEARANCE", 0, -220, -90000],
    [1, "REACQUIRE_GOAL_HEADING", 180, -220, 0],
    [2, "PASS_BEYOND_TARGET", 520, -220, 0],
    [3, "MERGE_GOAL_AXIS", 520, 0, 90000],
    [4, "RESUME_GOAL_HEADING", 700, 0, 0],
  ].map(([ordinal, kind, xMm, yMm, headingMdeg]) => ({
    ordinal,
    kind,
    x_mm: xMm,
    y_mm: yMm,
    heading_mdeg: headingMdeg,
  })),
};
function snapshot(sequence, state, runtime) {
  return {
    schema: "robot-control/v1",
    sequence,
    state,
    enabled: true,
    accepting: state === "RUNNING",
    episode: {
      episode_id: "episode-route",
      goal: "Pass the obstacle",
      started_at_unix_ms: 1000,
      terminal_reason: null,
    },
    updated_at_unix_ms: 2000 + sequence,
    runtime,
  };
}

const panel = api.create({
  document: documentApi,
  request: async () => {
    throw new Error("Network access was not expected");
  },
  translate,
  getLocale: () => "en",
});
panel.setControl(snapshot(1, "RUNNING", {
  current_action: "ADVANCE",
  plan: ["ADVANCE", "OBSERVE"],
  active_route: activeRoute,
  speech_status: "idle",
  message: "Following detour",
}));
const activeRender = {
  label: nodes["map-mission-route-label"].textContent,
  heading: nodes["map-mission-route-heading"].textContent,
  summary: nodes["map-mission-route-summary"].textContent,
  count: nodes["map-mission-route"].children.length,
  firstClass: nodes["map-mission-route"].children[0].className,
  firstMarker: nodes["map-mission-route"].children[0].children[0].textContent,
  activeClass: nodes["map-mission-route"].children[1].className,
  activeText: nodes["map-mission-route"].children[1].children[1].textContent,
  timelineShowsRoute: descendantTexts(nodes["map-mission-timeline"])
    .includes("Route updated"),
  timelineShowsSummary: descendantTexts(nodes["map-mission-timeline"])
    .includes("active · left of the goal heading · 2/5"),
};
panel.setControl(snapshot(2, "IDLE", {
  speech_status: "idle",
  message: "Detour complete",
}));

process.stdout.write(JSON.stringify({
  activeRender,
  historicalRender: {
    label: nodes["map-mission-route-label"].textContent,
    heading: nodes["map-mission-route-heading"].textContent,
    summary: nodes["map-mission-route-summary"].textContent,
    count: nodes["map-mission-route"].children.length,
  },
}));
"""
        completed = subprocess.run(
            [
                "node",
                "--input-type=commonjs",
                "-e",
                script,
                str(WEB_ROOT / "robot_mission_panel.js"),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
        )
        if completed.returncode != 0:
            raise AssertionError(completed.stderr)
        result = json.loads(completed.stdout)
        self.assertEqual(
            result["activeRender"],
            {
                "label": "Active route",
                "heading": "Active detour route",
                "summary": "active · left of the goal heading · 2/5",
                "count": 5,
                "firstClass": "map-mission-route-step is-complete",
                "firstMarker": "✓",
                "activeClass": "map-mission-route-step is-active",
                "activeText": (
                    "Reacquire the goal heading · (180, -220) mm · 0°"
                ),
                "timelineShowsRoute": True,
                "timelineShowsSummary": True,
            },
        )
        self.assertEqual(
            result["historicalRender"],
            {
                "label": "Latest route",
                "heading": "Latest published detour route",
                "summary": "active · left of the goal heading · 2/5",
                "count": 5,
            },
        )

    def test_robot_mission_history_retention_is_generous_and_explicit(self):
        script = r"""
const fs = require("fs");
const vm = require("vm");
const source = fs.readFileSync(process.argv[1], "utf8");
const context = {};
vm.runInNewContext(source, context, { filename: process.argv[1] });
const api = context.RobotMissionPanelUI;
const count = api.HISTORY_CAPACITY + 2;
const snapshots = Array.from({ length: count }, (_value, index) => {
  const sequence = index + 1;
  return {
    sequence,
    state: "RUNNING",
    enabled: true,
    episode: {
      episode_id: "episode-retention",
      goal: "Retain a useful local mission history",
      started_at_unix_ms: 1,
      terminal_reason: null,
    },
    updated_at_unix_ms: sequence,
    runtime: {
      current_action: sequence % 2 ? "OBSERVE" : "ADVANCE",
      plan: [sequence % 2 ? "OBSERVE" : "ADVANCE"],
      speech_status: "idle",
      message: `step-${sequence}`,
    },
  };
});
const events = Array.from({ length: count }, (_value, index) => {
  const sequence = index + 1;
  return {
    sequence,
    occurred_at_unix_ms: sequence,
    event_type: "robot.test_progress",
    message: `event-${sequence}`,
    episode_id: "episode-retention",
    state: "RUNNING",
    level: "info",
    data: {},
  };
});
const store = api.createHistoryStore();
store.ingestEvents({
  schema: api.EVENT_PAGE_SCHEMA,
  after_sequence: 0,
  oldest_sequence: 1,
  newest_sequence: count,
  next_after_sequence: count,
  gap: false,
  dropped_total: 0,
  events,
});
store.ingestSnapshots({
  schema: api.SNAPSHOT_PAGE_SCHEMA,
  after_sequence: 0,
  oldest_sequence: 1,
  newest_sequence: count,
  next_after_sequence: count,
  gap: false,
  dropped_total: 0,
  snapshots,
});
const values = store.values();
const timeline = api.buildTimeline(
  values,
  api.normalizeSnapshot(snapshots[snapshots.length - 1]),
);
process.stdout.write(JSON.stringify({
  historyCapacity: api.HISTORY_CAPACITY,
  timelineCapacity: api.TIMELINE_CAPACITY,
  eventCount: values.events.length,
  eventFirst: values.events[0].sequence,
  snapshotCount: values.snapshots.length,
  snapshotFirst: values.snapshots[0].sequence,
  gaps: store.gaps(),
  timelineCount: timeline.entries.length,
  timelineTotal: timeline.totalEntries,
  timelineTruncated: timeline.truncated,
}));
"""
        completed = subprocess.run(
            [
                "node",
                "--input-type=commonjs",
                "-e",
                script,
                str(WEB_ROOT / "robot_mission_panel.js"),
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        result = json.loads(completed.stdout)
        self.assertEqual(result["historyCapacity"], 1_000)
        self.assertEqual(result["timelineCapacity"], 500)
        self.assertEqual(result["eventCount"], 1_000)
        self.assertEqual(result["snapshotCount"], 1_000)
        self.assertEqual(result["eventFirst"], 3)
        self.assertEqual(result["snapshotFirst"], 3)
        self.assertEqual(result["gaps"], {"event": True, "snapshot": True})
        self.assertEqual(result["timelineCount"], 500)
        self.assertGreater(result["timelineTotal"], 500)
        self.assertTrue(result["timelineTruncated"])
        self.assertIn(
            "${endpoint}/events?after_sequence=${cursors.event}&limit=500",
            self.robot_mission_panel,
        )
        self.assertIn(
            "${endpoint}/snapshots?after_sequence=${cursors.snapshot}&limit=500",
            self.robot_mission_panel,
        )
        self.assertIn("MAX_LOCAL_EVENTS = 2000", self.javascript)

    def test_workbench_safety_contract_is_scoped_from_robot_control(self):
        self.assertIn(
            "const workbench = safeObject(capabilities.workbench);",
            self.javascript,
        )
        self.assertIn(
            'workbench.tool_effects === "read_only"',
            self.javascript,
        )
        self.assertNotIn(
            "bootstrap.physical_control_enabled === false",
            self.javascript,
        )
        submit_content = self.javascript[
            self.javascript.index("  async function submitCurrentContent(target) {"):
            self.javascript.index(
                "\n  async function submitTurn(event) {",
                self.javascript.index(
                    "  async function submitCurrentContent(target) {"
                ),
            )
        ]
        self.assertLess(
            submit_content.index("robotControl.isRobotTarget(target)"),
            submit_content.index("!state.workbenchReadOnlyInvariant"),
        )

    def test_css_has_responsive_accessible_motion_aware_layout(self):
        self.assertIn("@media (max-width: 1220px)", self.css)
        self.assertIn("@media (max-width: 820px)", self.css)
        self.assertIn("@media (prefers-reduced-motion: reduce)", self.css)
        self.assertIn(":focus-visible", self.css)
        self.assertIn("-apple-system", self.css)
        self.assertIn("--canvas: #0b0e10", self.css.lower())
        self.assertIn("transform: scale(1.15)", self.css)
        self.assertNotIn("translate(-25px", self.css)
        self.assertNotIn("translate(-27px", self.css)
        self.assertNotIn("linear-gradient(", self.css)
        self.assertNotIn("@import", self.css)
        self.assertIn(".map-layout {", self.css)
        self.assertIn(".spatial-map-canvas {", self.css)
        self.assertIn(
            "grid-template-columns: repeat(6, minmax(0, 1fr))",
            self.css,
        )
        composer_meta = self.css[
            self.css.index(".composer-meta {"):
            self.css.index(".composer-meta label")
        ]
        capability_note = self.css[
            self.css.index(".capability-note {"):
            self.css.index(".composer textarea")
        ]
        self.assertIn("flex-wrap: wrap", composer_meta)
        self.assertIn("flex: 1 1 240px", capability_note)
        self.assertIn("white-space: normal", capability_note)

    def test_server_instance_change_forces_reload_before_reusing_state(self):
        comparison = "previousInstanceId !== nextInstanceId"
        invalidate_polling = "state.turnPollGeneration += 1"
        reload_page = "window.location.reload()"

        self.assertIn("server_instance_id", self.javascript)
        self.assertIn(comparison, self.javascript)
        self.assertIn(invalidate_polling, self.javascript)
        self.assertIn(reload_page, self.javascript)
        self.assertLess(
            self.javascript.index(comparison),
            self.javascript.index(reload_page),
        )

    def test_historical_citations_fetch_their_own_turn_evidence(self):
        self.assertIn("message.turn_id", self.javascript)
        self.assertIn(
            "/api/v1/turns/${encodeURIComponent(message.turn_id)}",
            self.javascript,
        )
        self.assertIn(
            "renderEvidence(safeObject(payload.turn), citationId)",
            self.javascript,
        )

    def test_settings_input_ranges_match_the_backend_contract(self):
        elements_by_id = {
            attrs["id"]: (tag, attrs)
            for tag, attrs in self.parser.elements
            if "id" in attrs
        }
        expected_ranges = {
            "setting-planner-latency-ms": ("1", "300000", "1"),
            "setting-max-planner-turns": ("1", "100", "1"),
            "setting-max-replans": ("0", "100", "1"),
            "setting-max-tool-calls": ("0", "100", "1"),
            "setting-max-elapsed-ms": ("1", "300000", "1"),
            "setting-tool-request-ttl-ms": ("1", "300000", "1"),
            "setting-evidence-ttl-ms": ("1", "86400000", "1"),
            "setting-weather-skew-ms": ("1", "86400000", "1"),
        }

        for element_id, expected in expected_ranges.items():
            with self.subTest(element_id=element_id):
                tag, attrs = elements_by_id[element_id]
                self.assertEqual(tag, "input")
                self.assertEqual(attrs.get("type"), "number")
                self.assertEqual(
                    (attrs.get("min"), attrs.get("max"), attrs.get("step")),
                    expected,
                )

    def test_runtime_requires_the_configured_model_to_be_loaded(self):
        self.assertIn("configured_model_loaded", self.javascript)
        self.assertIn("state.modelReady", self.javascript)
        self.assertIn("modelLoaded === true", self.javascript)
        self.assertIn("state.modelReady === true", self.javascript)
        self.assertIn("konfigurerad modell ej laddad", self.i18n)
        self.assertNotIn("konfigurerad modell ej laddad", self.javascript)

    def test_chat_history_uses_a_separate_live_announcer(self):
        elements_by_id = {
            attrs["id"]: attrs
            for _, attrs in self.parser.elements
            if "id" in attrs
        }
        message_feed = elements_by_id["message-feed"]
        announcer = elements_by_id["chat-announcer"]

        self.assertEqual(message_feed.get("role"), "region")
        self.assertNotIn("aria-live", message_feed)
        self.assertEqual(announcer.get("aria-live"), "polite")
        self.assertEqual(announcer.get("aria-atomic"), "true")
        self.assertIn("sr-only", (announcer.get("class") or "").split())
        self.assertIn('byId("chat-announcer").textContent', self.javascript)
        self.assertIn("visibleStateChanged", self.javascript)
        self.assertIn(
            "feed.scrollHeight - feed.scrollTop - feed.clientHeight",
            self.javascript,
        )


if __name__ == "__main__":
    unittest.main()
