((global) => {
  "use strict";

  const ACTIVE_STATES = new Set(["STARTING", "RUNNING", "STOPPING"]);
  const CONTROL_STATES = new Set([
    "DISABLED",
    "IDLE",
    "STARTING",
    "RUNNING",
    "STOPPING",
    "FAULTED",
  ]);
  const EVENT_PAGE_SCHEMA = "robot-control-event-page/v1";
  const SNAPSHOT_PAGE_SCHEMA = "robot-control-snapshot-page/v1";
  const HISTORY_CAPACITY = 1000;
  const TIMELINE_CAPACITY = 500;
  const MAX_CATCH_UP_PAGES = 4;
  const POLL_ACTIVE_MS = 1000;
  const POLL_IDLE_MS = 2500;
  const POLL_CATCH_UP_MS = 0;

  function record(value) {
    return value && typeof value === "object" && !Array.isArray(value)
      ? value
      : {};
  }

  function array(value) {
    return Array.isArray(value) ? value : [];
  }

  function text(value, fallback = "") {
    return typeof value === "string" && value.length > 0
      ? value
      : fallback;
  }

  function integer(value, fallback = null) {
    return Number.isSafeInteger(value) && value >= 0 ? value : fallback;
  }

  function normalizeRoute(value) {
    const route = record(value);
    const waypoints = array(route.waypoints).map((item) => {
      const waypoint = record(item);
      return Object.freeze({
        ordinal: integer(waypoint.ordinal, 0),
        kind: text(waypoint.kind),
        xMm: Number.isSafeInteger(waypoint.x_mm)
          ? waypoint.x_mm
          : Number.isSafeInteger(waypoint.xMm) ? waypoint.xMm : 0,
        yMm: Number.isSafeInteger(waypoint.y_mm)
          ? waypoint.y_mm
          : Number.isSafeInteger(waypoint.yMm) ? waypoint.yMm : 0,
        headingMdeg: Number.isSafeInteger(waypoint.heading_mdeg)
          ? waypoint.heading_mdeg
          : Number.isSafeInteger(waypoint.headingMdeg)
            ? waypoint.headingMdeg
            : 0,
      });
    }).filter((item) => item.kind);
    return Object.freeze({
      routeId: text(route.route_id, text(route.routeId)),
      status: text(route.status),
      version: integer(route.version, 0),
      activeIndex: integer(
        route.active_index,
        integer(route.activeIndex, 0),
      ),
      detourSide: text(route.detour_side, text(route.detourSide)),
      waypoints: Object.freeze(waypoints),
    });
  }

  function normalizeSnapshot(value) {
    const source = record(value);
    const episode = Object.keys(record(source.episode)).length > 0
      ? record(source.episode)
      : source;
    const runtime = Object.keys(record(source.runtime)).length > 0
      ? record(source.runtime)
      : source;
    const state = CONTROL_STATES.has(source.state)
      ? source.state
      : "DISABLED";
    return Object.freeze({
      sequence: integer(source.sequence, 0),
      updatedAtUnixMs: integer(
        source.updated_at_unix_ms,
        integer(source.updatedAtUnixMs),
      ),
      state,
      enabled: source.enabled === true,
      episodeId: text(episode.episode_id, text(episode.episodeId)),
      goal: text(episode.goal),
      startedAtUnixMs: integer(
        episode.started_at_unix_ms,
        integer(episode.startedAtUnixMs),
      ),
      terminalReason: text(
        episode.terminal_reason,
        text(episode.terminalReason),
      ),
      currentAction: text(
        runtime.current_action,
        text(runtime.currentAction),
      ),
      plan: Object.freeze(
        array(runtime.plan).filter((step) => (
          typeof step === "string" && step.length > 0
        )),
      ),
      activeRoute: normalizeRoute(
        runtime.active_route || runtime.activeRoute,
      ),
      obstacle: record(runtime.obstacle),
      scan: record(runtime.scan),
      modelLatencyMs: integer(
        runtime.model_latency_ms,
        integer(runtime.modelLatencyMs),
      ),
      plannerContextBytes: integer(
        runtime.planner_context_bytes,
        integer(runtime.plannerContextBytes),
      ),
      promptTokens: integer(
        runtime.prompt_tokens,
        integer(runtime.promptTokens),
      ),
      completionTokens: integer(
        runtime.completion_tokens,
        integer(runtime.completionTokens),
      ),
      totalTokens: integer(
        runtime.total_tokens,
        integer(runtime.totalTokens),
      ),
      speechStatus: text(
        runtime.speech_status,
        text(runtime.speechStatus, "idle"),
      ),
      message: text(runtime.message),
      lastErrorCode: text(
        source.last_error_code,
        text(source.lastErrorCode),
      ),
      primaryErrorCode: text(
        source.primary_error_code,
        text(source.primaryErrorCode),
      ),
      primaryErrorMessage: text(
        source.primary_error_message,
        text(source.primaryErrorMessage),
      ),
    });
  }

  function normalizeEvent(value) {
    const source = record(value);
    const sequence = integer(source.sequence, 0);
    const eventType = text(source.event_type, text(source.eventType));
    if (sequence < 1 || !eventType) {
      return null;
    }
    return Object.freeze({
      sequence,
      occurredAtUnixMs: integer(
        source.occurred_at_unix_ms,
        integer(source.occurredAtUnixMs),
      ),
      eventType,
      message: text(source.message),
      episodeId: text(source.episode_id, text(source.episodeId)),
      state: CONTROL_STATES.has(source.state) ? source.state : "DISABLED",
      level: text(source.level, "info"),
      data: record(source.data),
    });
  }

  function stableValue(value) {
    if (Array.isArray(value)) {
      return value.map(stableValue);
    }
    if (value && typeof value === "object") {
      return Object.fromEntries(
        Object.keys(value).sort().map((key) => [
          key,
          stableValue(value[key]),
        ]),
      );
    }
    return value;
  }

  function signature(value) {
    return JSON.stringify(stableValue(value));
  }

  function createHistoryStore(capacity = HISTORY_CAPACITY) {
    if (!Number.isSafeInteger(capacity) || capacity < 1 || capacity > 1000) {
      throw new TypeError("Mission history capacity is invalid.");
    }
    const events = new Map();
    const snapshots = new Map();
    let eventCursor = 0;
    let snapshotCursor = 0;
    let eventNewest = null;
    let snapshotNewest = null;
    let eventGap = false;
    let snapshotGap = false;

    function trim(items) {
      const sequences = Array.from(items.keys()).sort((left, right) => (
        left - right
      ));
      const evicted = sequences.slice(
        0,
        Math.max(0, sequences.length - capacity),
      );
      evicted.forEach(
        (sequence) => items.delete(sequence),
      );
      return evicted.length;
    }

    function pageSequence(page, name, fallback) {
      return integer(record(page)[name], fallback);
    }

    function ingestEvents(value) {
      const page = record(value);
      if (page.schema !== EVENT_PAGE_SCHEMA || !Array.isArray(page.events)) {
        throw new TypeError("Robot event history page is invalid.");
      }
      const newest = pageSequence(page, "newest_sequence", 0);
      if (newest < eventCursor) {
        events.clear();
        eventCursor = 0;
        eventGap = false;
        eventNewest = newest;
        return false;
      }
      eventNewest = newest;
      page.events.forEach((item) => {
        const event = normalizeEvent(item);
        if (event && event.eventType !== "robot.runtime_update") {
          events.set(event.sequence, event);
        }
      });
      eventCursor = Math.max(
        eventCursor,
        pageSequence(page, "next_after_sequence", eventCursor),
      );
      eventGap = eventGap || page.gap === true;
      if (trim(events) > 0) {
        eventGap = true;
      }
      return true;
    }

    function ingestSnapshots(value) {
      const page = record(value);
      if (
        page.schema !== SNAPSHOT_PAGE_SCHEMA
        || !Array.isArray(page.snapshots)
      ) {
        throw new TypeError("Robot snapshot history page is invalid.");
      }
      const newest = pageSequence(page, "newest_sequence", 0);
      if (newest < snapshotCursor) {
        snapshots.clear();
        snapshotCursor = 0;
        snapshotGap = false;
        snapshotNewest = newest;
        return false;
      }
      snapshotNewest = newest;
      page.snapshots.forEach((item) => {
        const snapshot = normalizeSnapshot(item);
        if (snapshot.sequence > 0) {
          snapshots.set(snapshot.sequence, snapshot);
        }
      });
      snapshotCursor = Math.max(
        snapshotCursor,
        pageSequence(page, "next_after_sequence", snapshotCursor),
      );
      snapshotGap = snapshotGap || page.gap === true;
      if (trim(snapshots) > 0) {
        snapshotGap = true;
      }
      return true;
    }

    function addSnapshot(value) {
      const snapshot = normalizeSnapshot(value);
      if (snapshot.sequence > 0) {
        snapshots.set(snapshot.sequence, snapshot);
        if (trim(snapshots) > 0) {
          snapshotGap = true;
        }
      }
      return snapshot;
    }

    return Object.freeze({
      addSnapshot,
      caughtUp: () => (
        eventNewest !== null
        && snapshotNewest !== null
        && eventCursor >= eventNewest
        && snapshotCursor >= snapshotNewest
      ),
      cursors: () => Object.freeze({
        event: eventCursor,
        snapshot: snapshotCursor,
      }),
      gaps: () => Object.freeze({
        event: eventGap,
        snapshot: snapshotGap,
      }),
      ingestEvents,
      ingestSnapshots,
      progress: () => Object.freeze({
        event: Object.freeze({
          cursor: eventCursor,
          newest: eventNewest,
        }),
        snapshot: Object.freeze({
          cursor: snapshotCursor,
          newest: snapshotNewest,
        }),
      }),
      values: () => Object.freeze({
        events: Object.freeze(
          Array.from(events.values()).sort((left, right) => (
            left.sequence - right.sequence
          )),
        ),
        snapshots: Object.freeze(
          Array.from(snapshots.values()).sort((left, right) => (
            left.sequence - right.sequence
          )),
        ),
      }),
    });
  }

  function latestEpisodeId(snapshots, control) {
    if (control && control.episodeId) {
      return control.episodeId;
    }
    for (let index = snapshots.length - 1; index >= 0; index -= 1) {
      if (snapshots[index].episodeId) {
        return snapshots[index].episodeId;
      }
    }
    return "";
  }

  function runtimeChanges(previous, current) {
    const changes = [];
    const errorValue = (snapshot) => {
      if (!snapshot) {
        return "";
      }
      return [
        snapshot.lastErrorCode,
        snapshot.primaryErrorCode,
        snapshot.primaryErrorMessage,
      ].filter(Boolean).join(" · ");
    };
    const plannerValue = (snapshot) => {
      if (!snapshot) {
        return {};
      }
      const value = {
        contextBytes: snapshot.plannerContextBytes,
        promptTokens: snapshot.promptTokens,
        completionTokens: snapshot.completionTokens,
        totalTokens: snapshot.totalTokens,
      };
      return Object.values(value).some(Number.isSafeInteger) ? value : {};
    };
    const comparable = [
      ["action", previous && previous.currentAction, current.currentAction],
      ["plan", previous && previous.plan, current.plan],
      ["route", previous && previous.activeRoute, current.activeRoute],
      ["obstacle", previous && previous.obstacle, current.obstacle],
      ["scan", previous && previous.scan, current.scan],
      [
        "planner",
        plannerValue(previous),
        plannerValue(current),
      ],
      ["speech", previous && previous.speechStatus, current.speechStatus],
      ["message", previous && previous.message, current.message],
      [
        "terminal",
        previous && previous.terminalReason,
        current.terminalReason,
      ],
      ["error", errorValue(previous), errorValue(current)],
    ];
    comparable.forEach(([kind, before, after]) => {
      const meaningful = (
        kind === "route" ? Boolean(record(after).routeId)
          : typeof after === "string" ? after.length > 0
          : Array.isArray(after) ? after.length > 0
            : Object.keys(record(after)).length > 0
      ) && !(
        kind === "speech"
        && (after === "idle" || after === "disabled")
      );
      if (
        signature(before) !== signature(after)
        && (previous !== null || meaningful)
      ) {
        changes.push(Object.freeze({ kind, value: after }));
      }
    });
    return Object.freeze(changes);
  }

  function buildTimeline(
    values,
    currentControl = null,
    limit = TIMELINE_CAPACITY,
  ) {
    if (!Number.isSafeInteger(limit) || limit < 1 || limit > 500) {
      throw new TypeError("Mission timeline limit is invalid.");
    }
    const source = record(values);
    const snapshots = array(source.snapshots).map(normalizeSnapshot);
    const events = array(source.events).map(normalizeEvent).filter(Boolean);
    const episodeId = latestEpisodeId(snapshots, currentControl);
    if (!episodeId) {
      return Object.freeze({
        episodeId: "",
        entries: Object.freeze([]),
        totalEntries: 0,
        truncated: false,
      });
    }
    const episodeSnapshots = snapshots.filter((item) => (
      episodeId ? item.episodeId === episodeId : !item.episodeId
    ));
    const episodeEvents = events.filter((item) => (
      item.eventType !== "robot.runtime_update"
      && (episodeId ? item.episodeId === episodeId : !item.episodeId)
    ));
    const entries = [];
    let previous = null;
    episodeSnapshots.forEach((snapshot) => {
      const changes = runtimeChanges(previous, snapshot);
      if (changes.length > 0) {
        entries.push(Object.freeze({
          source: "snapshot",
          sequence: snapshot.sequence,
          occurredAtUnixMs: snapshot.updatedAtUnixMs,
          level: snapshot.lastErrorCode ? "error" : "info",
          kind: changes[0].kind,
          changes,
          state: snapshot.state,
        }));
      }
      previous = snapshot;
    });
    episodeEvents.forEach((event) => {
      entries.push(Object.freeze({
        source: "event",
        sequence: event.sequence,
        occurredAtUnixMs: event.occurredAtUnixMs,
        level: event.level,
        kind: "event",
        eventType: event.eventType,
        message: event.message,
        data: event.data,
        state: event.state,
      }));
    });
    entries.sort((left, right) => {
      const byTime = (right.occurredAtUnixMs || 0)
        - (left.occurredAtUnixMs || 0);
      if (byTime !== 0) {
        return byTime;
      }
      const bySource = (right.source === "snapshot" ? 1 : 0)
        - (left.source === "snapshot" ? 1 : 0);
      return bySource || right.sequence - left.sequence;
    });
    const totalEntries = entries.length;
    return Object.freeze({
      episodeId,
      entries: Object.freeze(entries.slice(0, limit)),
      totalEntries,
      truncated: totalEntries > limit,
    });
  }

  function create(options = {}) {
    const documentApi = options.document;
    const request = options.request;
    const translate = options.translate;
    const getLocale = options.getLocale;
    if (
      !documentApi
      || typeof documentApi.getElementById !== "function"
      || typeof documentApi.createElement !== "function"
      || typeof request !== "function"
      || typeof translate !== "function"
      || typeof getLocale !== "function"
    ) {
      throw new TypeError("Robot mission panel dependencies are invalid.");
    }
    const byId = (id) => documentApi.getElementById(id);
    const history = createHistoryStore();
    let control = normalizeSnapshot({});
    let connection = "waiting";
    let busy = false;
    let initialized = false;
    let stopped = false;
    let hasControl = false;
    let pollTimer = null;
    let renderedTimelineSignature = "";
    let renderedAnnouncementSignature = "";

    function createElement(tag, className, value) {
      const node = documentApi.createElement(tag);
      if (className) {
        node.className = className;
      }
      if (value !== undefined && value !== null) {
        node.textContent = String(value);
      }
      return node;
    }

    function localize(key, fallback, args) {
      const value = translate(key, args);
      return value === key ? fallback : value;
    }

    function stateLabel(state) {
      return localize(`robot.state.${state}`, state);
    }

    function speechLabel(status) {
      return localize(`robot.speech.${status}`, status);
    }

    function formatTime(value) {
      if (!Number.isSafeInteger(value)) {
        return translate("common.missing");
      }
      try {
        return new Intl.DateTimeFormat(getLocale(), {
          hour: "2-digit",
          minute: "2-digit",
          second: "2-digit",
        }).format(new Date(value));
      } catch (_error) {
        return translate("common.missing");
      }
    }

    function isoTime(value) {
      if (!Number.isSafeInteger(value)) {
        return "";
      }
      const date = new Date(value);
      return Number.isFinite(date.getTime()) ? date.toISOString() : "";
    }

    function compactFact(value) {
      if (typeof value === "string" || Number.isFinite(value)) {
        return String(value);
      }
      const fact = record(value);
      const preferred = [
        "label",
        "relation",
        "distance_mm",
        "target_id",
        "state",
        "result",
        "direction",
        "reason",
      ];
      const keys = preferred.filter((key) => Object.hasOwn(fact, key));
      const selected = keys.length > 0 ? keys : Object.keys(fact).sort();
      const parts = selected.slice(0, 4).map((key) => String(fact[key]));
      return parts.length > 0 ? parts.join(" · ") : translate("common.none");
    }

    function latestPublishedRuntime() {
      const snapshots = history.values().snapshots;
      let action = "";
      let plan = Object.freeze([]);
      let route = normalizeRoute({});
      for (let index = snapshots.length - 1; index >= 0; index -= 1) {
        const snapshot = snapshots[index];
        if (snapshot.episodeId !== control.episodeId) {
          continue;
        }
        if (!action && snapshot.currentAction) {
          action = snapshot.currentAction;
        }
        if (plan.length === 0 && snapshot.plan.length > 0) {
          plan = snapshot.plan;
        }
        if (!route.routeId && snapshot.activeRoute.routeId) {
          route = snapshot.activeRoute;
        }
        if (action && plan.length > 0 && route.routeId) {
          break;
        }
      }
      return { action, plan, route };
    }

    function renderPlan(plan) {
      const list = byId("map-mission-plan");
      byId("map-mission-plan-summary").textContent = plan.length > 0
        ? plan.join(" → ")
        : translate("common.none");
      if (plan.length === 0) {
        list.replaceChildren(createElement(
          "li",
          "map-mission-plan-empty",
          translate("mission.plan.empty"),
        ));
        return;
      }
      list.replaceChildren(...plan.map((step, index) => {
        const item = createElement("li", "map-mission-plan-step");
        item.appendChild(createElement(
          "span",
          "map-mission-plan-index",
          index + 1,
        ));
        item.appendChild(createElement("span", "", step));
        return item;
      }));
    }

    function routeSummary(route) {
      if (!route || !route.routeId) {
        return translate("common.none");
      }
      const count = route.waypoints.length;
      const status = localize(
        `mission.route.status.${route.status}`,
        route.status,
      );
      const side = localize(
        `mission.route.side.${route.detourSide}`,
        route.detourSide,
      );
      const progress = count > 0
        ? `${Math.min(route.activeIndex + 1, count)}/${count}`
        : "0/0";
      return [status, side, progress].filter(Boolean).join(" · ");
    }

    function renderRoute(route) {
      const value = route && route.routeId ? route : normalizeRoute({});
      const list = byId("map-mission-route");
      const count = value.waypoints.length;
      byId("map-mission-route-summary").textContent = routeSummary(value);
      if (!value.routeId || count === 0) {
        list.replaceChildren(createElement(
          "li",
          "map-mission-route-empty",
          translate("mission.route.empty"),
        ));
        return;
      }
      list.replaceChildren(...value.waypoints.map((waypoint, index) => {
        const item = createElement(
          "li",
          `map-mission-route-step${index === value.activeIndex ? " is-active" : ""}${index < value.activeIndex ? " is-complete" : ""}`,
        );
        item.appendChild(createElement(
          "span",
          "map-mission-route-index",
          index < value.activeIndex ? "✓" : waypoint.ordinal + 1,
        ));
        const label = localize(
          `mission.route.waypoint.${waypoint.kind}`,
          waypoint.kind,
        );
        item.appendChild(createElement(
          "span",
          "",
          `${label} · (${waypoint.xMm}, ${waypoint.yMm}) mm · ${Math.round(waypoint.headingMdeg / 1000)}°`,
        ));
        return item;
      }));
    }

    function changeLabel(change) {
      return localize(
        `mission.timeline.${change.kind}`,
        change.kind,
      );
    }

    function changeValue(change) {
      if (change.kind === "plan") {
        return array(change.value).length > 0
          ? change.value.join(" → ")
          : translate("mission.timeline.cleared");
      }
      if (change.kind === "route") {
        const route = normalizeRoute(change.value);
        return route.routeId ? routeSummary(route) : translate(
          "mission.timeline.cleared",
        );
      }
      if (change.kind === "obstacle" || change.kind === "scan") {
        return compactFact(change.value);
      }
      if (change.kind === "speech") {
        return speechLabel(text(change.value, "idle"));
      }
      if (change.kind === "terminal") {
        const value = text(change.value, "");
        return value
          ? localize(`mission.terminal.${value}`, value)
          : translate("mission.timeline.cleared");
      }
      return text(change.value, translate("mission.timeline.cleared"));
    }

    function eventDiagnostics(entry) {
      const data = record(entry.data);
      return [
        "terminal_reason",
        "error_code",
        "error_message",
        "primary_error_code",
        "primary_error_message",
      ].filter((key) => (
        Object.hasOwn(data, key)
        && (typeof data[key] === "string" || Number.isFinite(data[key]))
        && String(data[key]).length > 0
      )).map((key) => ({
        key,
        value: key === "terminal_reason"
          ? localize(`mission.terminal.${data[key]}`, String(data[key]))
          : String(data[key]),
      }));
    }

    function eventLabel(entry) {
      const key = `mission.event.${entry.eventType}`;
      return localize(key, text(entry.message, entry.eventType));
    }

    function renderTimeline() {
      const values = history.values();
      const timeline = buildTimeline(values, control);
      const list = byId("map-mission-timeline");
      const gaps = history.gaps();
      const nextSignature = signature({
        entries: timeline.entries,
        gaps,
        locale: getLocale(),
        totalEntries: timeline.totalEntries,
        truncated: timeline.truncated,
      });
      if (nextSignature === renderedTimelineSignature) {
        return;
      }
      renderedTimelineSignature = nextSignature;
      byId("map-mission-history-count").textContent = timeline.truncated
        ? `${timeline.entries.length}+`
        : String(timeline.totalEntries);
      const gapNode = byId("map-mission-history-gap");
      const sourceGap = gaps.event || gaps.snapshot;
      gapNode.hidden = !(sourceGap || timeline.truncated);
      gapNode.textContent = sourceGap
        ? translate("mission.history.gap")
        : translate("mission.history.truncated", {
          count: timeline.entries.length,
          total: timeline.totalEntries,
        });
      if (timeline.entries.length === 0) {
        list.replaceChildren(createElement(
          "li",
          "map-mission-timeline-empty",
          translate("mission.timeline.empty"),
        ));
        return;
      }
      list.replaceChildren(...timeline.entries.map((entry) => {
        const item = createElement(
          "li",
          `map-mission-timeline-item is-${entry.level}`,
        );
        const marker = createElement("span", "map-mission-timeline-marker");
        marker.setAttribute("aria-hidden", "true");
        item.appendChild(marker);
        const content = createElement("div", "map-mission-timeline-content");
        const header = createElement("div", "map-mission-timeline-header");
        header.appendChild(createElement(
          "strong",
          "",
          entry.source === "event"
            ? eventLabel(entry)
            : changeLabel(entry.changes[0]),
        ));
        const timeNode = createElement(
          "time",
          "",
          formatTime(entry.occurredAtUnixMs),
        );
        const machineTime = isoTime(entry.occurredAtUnixMs);
        if (machineTime) {
          timeNode.dateTime = machineTime;
        }
        header.appendChild(timeNode);
        content.appendChild(header);
        if (entry.source === "snapshot") {
          const changes = createElement("dl", "map-mission-change-list");
          entry.changes.forEach((change) => {
            const row = createElement("div");
            row.appendChild(createElement("dt", "", changeLabel(change)));
            row.appendChild(createElement("dd", "", changeValue(change)));
            changes.appendChild(row);
          });
          content.appendChild(changes);
        } else if (entry.eventType) {
          content.appendChild(createElement(
            "small",
            "map-mission-event-type",
            entry.eventType,
          ));
          const diagnostics = eventDiagnostics(entry);
          if (diagnostics.length > 0) {
            const details = createElement(
              "dl",
              "map-mission-change-list",
            );
            diagnostics.forEach((diagnostic) => {
              const row = createElement("div");
              row.appendChild(createElement(
                "dt",
                "",
                localize(
                  `mission.event.data.${diagnostic.key}`,
                  diagnostic.key,
                ),
              ));
              row.appendChild(createElement("dd", "", diagnostic.value));
              details.appendChild(row);
            });
            content.appendChild(details);
          }
        }
        item.appendChild(content);
        return item;
      }));
    }

    function renderConnection() {
      const node = byId("map-mission-history-status");
      node.className = "state-chip";
      if (connection === "live") {
        node.classList.add("state-ready");
        node.textContent = translate("mission.history.live");
      } else if (connection === "catching_up") {
        node.classList.add("state-running");
        node.textContent = translate("mission.history.catching_up");
      } else if (connection === "partial") {
        node.classList.add("state-locked");
        node.textContent = translate("mission.history.partial");
      } else if (connection === "offline") {
        node.classList.add("state-fault");
        node.textContent = translate("mission.history.offline");
      } else {
        node.classList.add("state-idle");
        node.textContent = translate("mission.history.waiting");
      }
    }

    function render() {
      const stateNode = byId("map-mission-state");
      stateNode.className = "state-chip";
      stateNode.classList.add(
        control.state === "FAULTED"
          ? "state-fault"
          : ACTIVE_STATES.has(control.state)
            ? "state-running"
            : control.state === "IDLE"
              ? "state-ready"
              : "state-idle",
      );
      stateNode.textContent = stateLabel(control.state);
      const active = ACTIVE_STATES.has(control.state);
      const historical = !active && Boolean(control.episodeId);
      const presentedRuntime = historical
        ? latestPublishedRuntime()
        : {
          action: control.currentAction,
          plan: control.plan,
          route: control.activeRoute,
        };
      byId("map-mission-mode").textContent = active
        ? translate("mission.mode.active")
        : control.episodeId
          ? translate("mission.mode.latest")
          : translate("mission.mode.none");
      byId("map-mission-action-label").textContent = historical
        ? translate("mission.action.latest_label")
        : translate("mission.action.label");
      byId("map-mission-plan-label").textContent = historical
        ? translate("mission.plan.latest_label")
        : translate("mission.plan.label");
      byId("map-mission-plan-heading").textContent = historical
        ? translate("mission.plan.latest_title")
        : translate("mission.plan.title");
      byId("map-mission-route-label").textContent = historical
        ? translate("mission.route.latest_label")
        : translate("mission.route.label");
      byId("map-mission-route-heading").textContent = historical
        ? translate("mission.route.latest_title")
        : translate("mission.route.title");
      byId("map-mission-goal").textContent = text(
        control.goal,
        translate("mission.goal.empty"),
      );
      byId("map-mission-action").textContent = text(
        presentedRuntime.action,
        translate("common.none"),
      );
      const errorMessage = Array.from(new Set([
        control.primaryErrorMessage,
        control.primaryErrorCode,
        control.lastErrorCode,
      ].filter(Boolean))).join(" · ");
      const fallbackMessage = active
        ? translate("mission.message.waiting")
        : translate("mission.message.idle");
      const statusMessage = control.state === "FAULTED"
        ? text(errorMessage, text(control.message, fallbackMessage))
        : text(control.message, text(errorMessage, fallbackMessage));
      byId("map-mission-message").textContent = statusMessage;
      byId("map-mission-speech").textContent = speechLabel(
        control.speechStatus,
      );
      byId("map-mission-updated").textContent = formatTime(
        control.updatedAtUnixMs,
      );
      const announcement = {
        action: text(presentedRuntime.action, translate("common.none")),
        message: statusMessage,
        state: stateLabel(control.state),
      };
      const announcementSignature = signature(announcement);
      if (announcementSignature !== renderedAnnouncementSignature) {
        renderedAnnouncementSignature = announcementSignature;
        byId("map-mission-live-announcement").textContent = translate(
          "mission.live_announcement",
          announcement,
        );
      }
      renderPlan(presentedRuntime.plan);
      renderRoute(presentedRuntime.route);
      renderTimeline();
      renderConnection();
    }

    async function refreshHistory() {
      if (busy || stopped) {
        return false;
      }
      busy = true;
      try {
        for (let page = 0; page < MAX_CATCH_UP_PAGES; page += 1) {
          const cursors = history.cursors();
          const results = await Promise.allSettled([
            request(
              `/api/v1/robot/events?after_sequence=${cursors.event}&limit=500`,
              { timeout: 5000 },
            ),
            request(
              `/api/v1/robot/snapshots?after_sequence=${cursors.snapshot}&limit=500`,
              { timeout: 5000 },
            ),
          ]);
          if (stopped) {
            return false;
          }
          let accepted = 0;
          if (results[0].status === "fulfilled") {
            try {
              if (history.ingestEvents(results[0].value)) {
                accepted += 1;
              }
            } catch (_error) {
              // The independent snapshot stream may still be usable.
            }
          }
          if (results[1].status === "fulfilled") {
            try {
              if (history.ingestSnapshots(results[1].value)) {
                accepted += 1;
              }
            } catch (_error) {
              // The independent event stream may still be usable.
            }
          }
          if (accepted < 2) {
            connection = accepted === 1 ? "partial" : "offline";
            break;
          }
          if (history.caughtUp()) {
            connection = "live";
            break;
          }
          const advanced = history.cursors();
          if (
            advanced.event <= cursors.event
            && advanced.snapshot <= cursors.snapshot
          ) {
            connection = "partial";
            break;
          }
          connection = "catching_up";
        }
        render();
        return connection === "live";
      } catch (_error) {
        connection = "offline";
        renderConnection();
        return false;
      } finally {
        busy = false;
      }
    }

    function schedulePoll() {
      if (!initialized || stopped) {
        return;
      }
      if (pollTimer !== null) {
        global.clearTimeout(pollTimer);
      }
      const delay = connection === "catching_up"
        ? POLL_CATCH_UP_MS
        : ACTIVE_STATES.has(control.state)
          ? POLL_ACTIVE_MS
          : POLL_IDLE_MS;
      pollTimer = global.setTimeout(async () => {
        await refreshHistory();
        schedulePoll();
      }, delay);
    }

    function stopPolling() {
      if (stopped) {
        return;
      }
      stopped = true;
      initialized = false;
      connection = "offline";
      if (pollTimer !== null) {
        global.clearTimeout(pollTimer);
        pollTimer = null;
      }
      renderConnection();
    }

    function setControl(value) {
      const candidate = normalizeSnapshot(value);
      if (hasControl && candidate.sequence <= control.sequence) {
        return false;
      }
      const previousPollingMode = ACTIVE_STATES.has(control.state);
      control = history.addSnapshot(value);
      hasControl = true;
      render();
      if (
        initialized
        && previousPollingMode !== ACTIVE_STATES.has(control.state)
      ) {
        schedulePoll();
      }
      return true;
    }

    async function initialize() {
      if (stopped) {
        return;
      }
      initialized = true;
      render();
      await refreshHistory();
      schedulePoll();
    }

    return Object.freeze({
      initialize,
      refreshHistory,
      renderLocale: () => {
        renderedTimelineSignature = "";
        render();
      },
      setControl,
      stopPolling,
    });
  }

  global.RobotMissionPanelUI = Object.freeze({
    ACTIVE_STATES,
    EVENT_PAGE_SCHEMA,
    HISTORY_CAPACITY,
    MAX_CATCH_UP_PAGES,
    POLL_ACTIVE_MS,
    POLL_CATCH_UP_MS,
    POLL_IDLE_MS,
    SNAPSHOT_PAGE_SCHEMA,
    TIMELINE_CAPACITY,
    buildTimeline,
    create,
    createHistoryStore,
    normalizeEvent,
    normalizeRoute,
    normalizeSnapshot,
  });
})(typeof window === "undefined" ? globalThis : window);
