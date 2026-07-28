((global) => {
  "use strict";

  const TERMINAL_TURN_STATES = new Set([
    "answered",
    "clarification_required",
    "failed",
  ]);
  const TURN_POLL_POLICY = Object.freeze({
    unknownAfterFailures: 8,
    baseDelayMs: 800,
    maxDelayMs: 5000,
  });

  function replaceRenderedItems(container, items, renderItem) {
    if (!container || typeof container.replaceChildren !== "function") {
      throw new TypeError("A render container is required.");
    }
    if (typeof renderItem !== "function") {
      throw new TypeError("A render callback is required.");
    }
    const normalized = Array.isArray(items) ? items : [];
    container.replaceChildren(...normalized.map((item) => renderItem(item)));
    return normalized;
  }

  function transitionTurnPoll(current, event) {
    const previous = (
      current && typeof current === "object" && !Array.isArray(current)
        ? current
        : {}
    );
    const previousFailures = Number.isSafeInteger(previous.failures)
      && previous.failures >= 0
      ? previous.failures
      : 0;
    const previousConnection = (
      previous.connection === "retrying"
      || previous.connection === "unknown"
    )
      ? previous.connection
      : "connected";
    if (event && event.type === "success") {
      const turn = (
        event.turn && typeof event.turn === "object" && !Array.isArray(event.turn)
          ? event.turn
          : {}
      );
      return Object.freeze({
        failures: 0,
        connection: "connected",
        becameUnknown: false,
        recovered: previousConnection !== "connected",
        terminal: TERMINAL_TURN_STATES.has(turn.status),
        retry: false,
        retryDelayMs: null,
      });
    }
    if (!event || event.type !== "failure") {
      throw new TypeError("Turn polling requires a success or failure event.");
    }
    const failures = previousFailures + 1;
    const connection = failures >= TURN_POLL_POLICY.unknownAfterFailures
      ? "unknown"
      : "retrying";
    return Object.freeze({
      failures,
      connection,
      becameUnknown: (
        connection === "unknown"
        && previousConnection !== "unknown"
      ),
      recovered: false,
      terminal: false,
      retry: true,
      retryDelayMs: Math.min(
        TURN_POLL_POLICY.maxDelayMs,
        TURN_POLL_POLICY.baseDelayMs * failures,
      ),
    });
  }

  global.RobotDashboardLogic = Object.freeze({
    TURN_POLL_POLICY,
    replaceRenderedItems,
    transitionTurnPoll,
  });
})(typeof window === "undefined" ? globalThis : window);
